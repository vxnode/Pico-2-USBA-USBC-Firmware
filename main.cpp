// Pico 2 (RP2350) USB mouse passthrough + debug COM port
//
//   Mouse --> USB-A (PIO host on GPIO0/GPIO1) --> Pico 2 --> USB-C (native) --> Computer
//
// Core 1 runs the USB host (PIO-USB, timing sensitive), core 0 runs the USB device.
// The computer sees one composite device: a HID mouse (copy of the real mouse's report
// descriptor) plus a USB serial port that prints debug messages.
//
// The COM port is present from boot. When a mouse is plugged in/removed the Pico
// re-enumerates (so the HID descriptor matches the mouse); the COM port drops for about
// a second and comes back. Messages logged while no terminal is open are buffered.

#include <cctype>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/mutex.h"
#include "pico/util/queue.h"
#include "hardware/clocks.h"
#include "tusb.h"
#include "pio_usb.h"

// USB-A D+ on GPIO PIN_USB_HOST_DP, D- on the next GPIO.
// Waveshare RP2350-USB-A: D+ = GPIO12, D- = GPIO13.
// Bare Pico 2 with your own USB-A socket: use whatever pair you wired (e.g. 0).
#ifndef PIN_USB_HOST_DP
#define PIN_USB_HOST_DP 12
#endif

namespace {

constexpr size_t kMaxDescLen = 512;

struct Report {
    uint8_t id;        // 0 if the mouse doesn't use report IDs
    uint8_t len;
    uint8_t data[64];
};

struct LogMsg {
    char text[96];
};

queue_t g_report_q;
queue_t g_log_q;
mutex_t g_log_mutex;

// Descriptor staged by core 1 (host side)
uint8_t           g_stage[kMaxDescLen];
volatile uint16_t g_stage_len     = 0;
volatile bool     g_mouse_present = false;
volatile bool     g_has_ids       = false;
volatile bool     g_changed       = false;

// Descriptor actually presented to the computer (core 0 only, updated while disconnected)
uint8_t  g_desc[kMaxDescLen];
uint16_t g_desc_len = 0;

// Placeholder mouse shown to the computer while no real mouse is attached
const uint8_t kPlaceholderDesc[] = { TUD_HID_REPORT_DESC_MOUSE() };

// Report IDs allowed through (core 1 only)
uint8_t g_mouse_ids[32];

// Host interface being forwarded (core 1 only)
uint8_t g_dev_addr = 0;
uint8_t g_idx      = 0xFF;

// Counters for the status line
volatile uint32_t g_rx_reports  = 0;
volatile uint32_t g_fwd_reports = 0;
volatile uint32_t g_dropped     = 0;
volatile uint32_t g_filtered    = 0;   // reports blocked because their ID is not a mouse ID
volatile int      g_host_devs   = 0;

void logf(const char* fmt, ...) {
#if ENABLE_CDC
    LogMsg m;
    mutex_enter_blocking(&g_log_mutex);
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(m.text, sizeof(m.text) - 2, fmt, ap);
    va_end(ap);
    mutex_exit(&g_log_mutex);
    size_t n = strlen(m.text);
    m.text[n] = '\n';
    m.text[n + 1] = 0;
    queue_try_add(&g_log_q, &m);
#else
    (void)fmt;
#endif
}

// Tiny HID descriptor walker.
//  - is_mouse: first top-level collection is Generic Desktop / Mouse
//  - has_ids : descriptor uses Report IDs
//  - id_map  : bitmap (256 bits) of Report IDs that belong to a top-level Mouse collection.
// Only reports whose ID is in id_map are forwarded, so consumer-control / system-control
// (power, sleep, wireless radio...) reports on the same interface can never reach the computer.
bool parse_descriptor(const uint8_t* d, uint16_t len, bool& is_mouse, bool& has_ids, uint8_t* id_map,
                      uint8_t* out, uint16_t& out_len) {
    out_len = 0;
    uint16_t seg_start = 0;
    is_mouse = false;
    has_ids = false;
    memset(id_map, 0, 32);
    uint32_t usage_page = 0, usage = 0;
    int depth = 0;
    bool first_top = true, top_is_mouse = false;

    for (uint16_t i = 0; i < len;) {
        uint8_t prefix = d[i];
        if (prefix == 0xFE) {
            if (i + 1 >= len) return false;
            i += 3 + d[i + 1];
            continue;
        }
        uint8_t size = prefix & 0x03;
        if (size == 3) size = 4;
        uint8_t type = (prefix >> 2) & 0x03;  // 0 main, 1 global, 2 local
        uint8_t tag  = (prefix >> 4) & 0x0F;
        if (i + 1 + size > len) return false;

        uint32_t val = 0;
        for (uint8_t b = 0; b < size; b++) val |= (uint32_t)d[i + 1 + b] << (8 * b);

        if (type == 1 && tag == 0x0) usage_page = val;
        else if (type == 1 && tag == 0x8) {
            has_ids = true;
            if (depth > 0 && top_is_mouse && val < 256) id_map[val >> 3] |= (uint8_t)(1u << (val & 7));
        }
        else if (type == 2 && tag == 0x0) usage = val;
        else if (type == 0 && tag == 0xA) {            // Collection
            if (depth == 0) {
                top_is_mouse = (usage_page == 0x01 && usage == 0x02);
                if (first_top) { is_mouse = top_is_mouse; first_top = false; }
            }
            depth++;
        }
        else if (type == 0 && tag == 0xC) {            // End Collection
            if (depth > 0) depth--;
            if (depth == 0) {
                uint16_t end = i + 1 + size;
                // keep only top-level Mouse collections (with their leading Usage Page/Usage items)
                if (top_is_mouse && out_len + (end - seg_start) <= kMaxDescLen) {
                    memcpy(out + out_len, d + seg_start, end - seg_start);
                    out_len += end - seg_start;
                }
                seg_start = end;
            }
        }
        if (type == 0) usage = 0;
        i += 1 + size;
    }
    return true;
}

// ---- Input injection: commands typed/sent over the COM port become mouse reports ----
// The report is built to match whatever descriptor the computer currently sees, so this works with
// the placeholder mouse (no mouse plugged in) and with the real mouse's layout (16-bit XY etc.).
struct Field { bool has = false; uint16_t off = 0; uint8_t size = 0; int32_t min = 0, max = 0; };
struct Layout {
    bool valid = false;
    uint8_t id = 0;            // report ID (0 = none)
    uint8_t bytes = 0;         // payload bytes, excluding the ID byte
    uint16_t btn_off = 0;
    uint8_t btn_count = 0;
    Field x, y, wheel, pan;
};
Layout   g_layout;
uint16_t g_inj_buttons = 0;
bool     g_quiet = false;

constexpr int kRing = 512;
Report g_ring[kRing];
uint16_t g_ring_btn[kRing];          // injected button state at the time each report was queued
uint16_t g_real_buttons = 0;         // latest button state reported by the real mouse
int g_ring_head = 0, g_ring_tail = 0;
bool ring_push(const Report& r) {
    int n = (g_ring_head + 1) % kRing;
    if (n == g_ring_tail) return false;
    g_ring[g_ring_head] = r;
    g_ring_btn[g_ring_head] = g_inj_buttons;
    g_ring_head = n;
    return true;
}
bool ring_pop(Report& r, uint16_t* btn = nullptr) {
    if (g_ring_head == g_ring_tail) return false;
    r = g_ring[g_ring_tail];
    if (btn) *btn = g_ring_btn[g_ring_tail];
    g_ring_tail = (g_ring_tail + 1) % kRing;
    return true;
}
void ring_clear() { g_ring_head = g_ring_tail = 0; }

// Find the first report ID (or the only report) that has both X and Y, and where its fields sit.
bool parse_layout(const uint8_t* d, uint16_t len, Layout& out) {
    out = Layout{};
    Layout cur{};
    uint8_t cur_id = 0;
    uint32_t bitpos = 0;
    uint32_t usage_page = 0, rsize = 0, rcount = 0;
    int32_t lmin = 0, lmax = 0;
    uint32_t usages[16];
    uint8_t nusg = 0;
    uint32_t umin = 0, umax = 0;
    bool has_range = false;

    auto finish = [&]() -> bool {
        if (cur.x.has && cur.y.has && ((bitpos + 7) / 8) <= 64) {
            cur.valid = true;
            cur.id = cur_id;
            cur.bytes = (uint8_t)((bitpos + 7) / 8);
            out = cur;
            return true;
        }
        return false;
    };
    auto setf = [&](Field& f, uint32_t off) {
        if (rsize == 0 || rsize > 16 || off + rsize > 512) return;
        f.has = true; f.off = (uint16_t)off; f.size = (uint8_t)rsize; f.min = lmin; f.max = lmax;
    };

    for (uint16_t i = 0; i < len;) {
        uint8_t prefix = d[i];
        if (prefix == 0xFE) {
            if (i + 1 >= len) break;
            i += 3 + d[i + 1];
            continue;
        }
        uint8_t size = prefix & 0x03;
        if (size == 3) size = 4;
        uint8_t type = (prefix >> 2) & 0x03;
        uint8_t tag  = (prefix >> 4) & 0x0F;
        if (i + 1 + size > len) break;
        uint32_t val = 0;
        for (uint8_t b = 0; b < size; b++) val |= (uint32_t)d[i + 1 + b] << (8 * b);
        int32_t sval = size == 1 ? (int8_t)val : size == 2 ? (int16_t)val : (int32_t)val;

        if (type == 1) {                                   // global
            if (tag == 0x0) usage_page = val;
            else if (tag == 0x1) lmin = sval;
            else if (tag == 0x2) lmax = (lmin < 0) ? sval : (int32_t)val;
            else if (tag == 0x7) rsize = val;
            else if (tag == 0x9) rcount = val;
            else if (tag == 0x8) {                         // Report ID
                if (finish()) return true;
                cur = Layout{};
                cur_id = (uint8_t)val;
                bitpos = 0;
            }
        } else if (type == 2) {                            // local
            if (tag == 0x0 && nusg < 16) usages[nusg++] = val;
            else if (tag == 0x1) { umin = val; has_range = true; }
            else if (tag == 0x2) { umax = val; has_range = true; }
        } else if (type == 0) {                            // main
            if (tag == 0x8) {                              // Input
                if (!(val & 1) && (val & 2)) {             // data + variable
                    for (uint32_t k = 0; k < rcount && k < 64; k++) {
                        uint32_t page = usage_page, u = 0xFFFFFFFF;
                        if (has_range) { u = umin + k; if (u > umax) u = umax; }
                        else if (nusg) {
                            u = usages[k < nusg ? k : nusg - 1];
                            if (u >> 16) { page = u >> 16; u &= 0xFFFF; }
                        }
                        uint32_t off = bitpos + k * rsize;
                        if (page == 0x09) {
                            if (cur.btn_count == 0) cur.btn_off = (uint16_t)off;
                            if (cur.btn_count < 16) cur.btn_count++;
                        }
                        else if (page == 0x01 && u == 0x30) setf(cur.x, off);
                        else if (page == 0x01 && u == 0x31) setf(cur.y, off);
                        else if (page == 0x01 && u == 0x38) setf(cur.wheel, off);
                        else if (page == 0x0C && u == 0x238) setf(cur.pan, off);
                    }
                }
                bitpos += rsize * rcount;
            }
            nusg = 0; has_range = false;
        }
        i += 1 + size;
    }
    return finish();
}

void put_bits(uint8_t* buf, uint32_t off, uint8_t size, uint32_t v) {
    for (uint8_t b = 0; b < size; b++) {
        uint32_t pos = off + b;
        if ((v >> b) & 1) buf[pos >> 3] |= (uint8_t)(1u << (pos & 7));
        else              buf[pos >> 3] &= (uint8_t)~(1u << (pos & 7));
    }
}

void build_report(uint16_t btn, int dx, int dy, int wh, int pn, Report& r) {
    memset(&r, 0, sizeof(r));
    r.id = g_layout.id;
    r.len = g_layout.bytes;
    for (uint8_t b = 0; b < g_layout.btn_count; b++)
        if ((btn >> b) & 1) put_bits(r.data, g_layout.btn_off + b, 1, 1);
    auto put = [&](const Field& f, int v) {
        if (f.has) put_bits(r.data, f.off, f.size, (uint32_t)v & ((1u << f.size) - 1));
    };
    put(g_layout.x, dx); put(g_layout.y, dy); put(g_layout.wheel, wh); put(g_layout.pan, pn);
}

// Button bits of a report (payload without ID) in the layout's button field.
uint16_t get_buttons(const Report& r) {
    uint16_t m = 0;
    for (uint8_t b = 0; b < g_layout.btn_count; b++) {
        uint32_t pos = g_layout.btn_off + b;
        if ((pos >> 3) < r.len && ((r.data[pos >> 3] >> (pos & 7)) & 1)) m |= (uint16_t)(1u << b);
    }
    return m;
}
void set_buttons(Report& r, uint16_t m) {
    for (uint8_t b = 0; b < g_layout.btn_count; b++) {
        uint32_t pos = g_layout.btn_off + b;
        if ((pos >> 3) < r.len) put_bits(r.data, pos, 1, (m >> b) & 1);
    }
}

int take(int rem, const Field& f) {                        // largest chunk of rem that fits the field
    if (!f.has || rem == 0) return 0;
    if (rem > 0) return f.max <= 0 ? 0 : (rem < f.max ? rem : f.max);
    return f.min >= 0 ? 0 : (rem > f.min ? rem : f.min);
}

bool inject(int dx, int dy, int wh, int pn) {
    if (!g_layout.valid) return false;
    if (!g_layout.x.has) dx = 0;
    if (!g_layout.y.has) dy = 0;
    if (!g_layout.wheel.has) wh = 0;
    if (!g_layout.pan.has) pn = 0;
    int guard = 0;
    do {
        int cx = take(dx, g_layout.x), cy = take(dy, g_layout.y);
        int cw = take(wh, g_layout.wheel), cp = take(pn, g_layout.pan);
        Report r;
        build_report(g_inj_buttons, cx, cy, cw, cp, r);
        if (!ring_push(r)) return false;
        dx -= cx; dy -= cy; wh -= cw; pn -= cp;
        if (++guard > kRing) return false;
    } while (dx || dy || wh || pn);
    return true;
}

// Text commands, one per line (end with \n or \r). Replies start with "[cmd]".
//   m DX DY    move the cursor (also 50,20 / m50,20 / 50 20)       w N   scroll wheel      p N   horizontal scroll
//   d [B]      button down            u [B] button up         c [B] click (B: 1 left, 2 right, 3 middle)
//   reset      release all buttons    q 1/0 hide/show [status] lines     help
void handle_command(char* line) {
    while (*line == ' ' || *line == '\t') line++;
    if (!*line) return;
    for (char* p = line; *p; p++) *p = (char)tolower((unsigned char)*p);
    // Compact move format: "50 20", "50,20", "m50,20", "M 50, 20" (only m, digits, + - , and spaces)
    {
        bool compact = true, digit = false;
        for (const char* p = line; *p; p++) {
            if (isdigit((unsigned char)*p)) digit = true;
            else if (!strchr("m+-, \t", *p)) { compact = false; break; }
        }
        if (compact && digit) {
            char tmp[64];
            size_t j = 0;
            for (const char* p = line; *p && j < sizeof(tmp) - 1; p++)
                tmp[j++] = (*p == 'm' || *p == ',') ? ' ' : *p;
            tmp[j] = 0;
            int mx = 0, my = 0;
            if (sscanf(tmp, "%d %d", &mx, &my) == 2) {
                if (!g_layout.valid) { logf("[cmd] error: no usable mouse layout"); return; }
                logf(inject(mx, my, 0, 0) ? "[cmd] ok" : "[cmd] error: queue full, try again");
                return;
            }
        }
    }
    char cmd[16] = "";
    int a = 0, b = 0;
    int n = sscanf(line, "%15s %d %d", cmd, &a, &b);
    if (n < 1) return;

    auto is = [&](const char* x, const char* y = nullptr) { return !strcmp(cmd, x) || (y && !strcmp(cmd, y)); };
    bool ok = true;

    if (is("help", "?")) {
        logf("[cmd] m DX DY | w N | p N | d/u/c [BTN 1-3] | reset | q 1/0");
        return;
    }
    if (is("q")) { g_quiet = n >= 2 ? a != 0 : true; logf("[cmd] ok quiet=%d", g_quiet); return; }
    if (!g_layout.valid) { logf("[cmd] error: no usable mouse layout"); return; }

    if (is("m", "move")) {
        if (n < 3) { logf("[cmd] usage: m DX DY"); return; }
        ok = inject(a, b, 0, 0);
    } else if (is("w", "wheel")) {
        if (n < 2) { logf("[cmd] usage: w N"); return; }
        ok = inject(0, 0, a, 0);
    } else if (is("p", "pan")) {
        if (n < 2) { logf("[cmd] usage: p N"); return; }
        ok = inject(0, 0, 0, a);
    } else if (is("d", "down") || is("u", "up") || is("c", "click")) {
        int btn = n >= 2 ? a : 1;
        if (btn < 1 || btn > g_layout.btn_count) { logf("[cmd] error: button must be 1..%u", g_layout.btn_count); return; }
        uint16_t bit = (uint16_t)(1u << (btn - 1));
        if (is("d", "down")) { g_inj_buttons |= bit; ok = inject(0, 0, 0, 0); }
        else if (is("u", "up")) { g_inj_buttons &= (uint16_t)~bit; ok = inject(0, 0, 0, 0); }
        else {
            g_inj_buttons |= bit;  ok = inject(0, 0, 0, 0);
            g_inj_buttons &= (uint16_t)~bit; ok = ok && inject(0, 0, 0, 0);
        }
    } else if (is("reset")) {
        g_inj_buttons = 0; ok = inject(0, 0, 0, 0);
    } else {
        logf("[cmd] unknown command, type help");
        return;
    }
    logf(ok ? "[cmd] ok" : "[cmd] error: queue full, try again");
}

}  // namespace

// ---- Host side (core 1) ------------------------------------------------
extern "C" {

void tuh_mount_cb(uint8_t dev_addr) {
    uint16_t vid = 0, pid = 0;
    tuh_vid_pid_get(dev_addr, &vid, &pid);
    g_host_devs++;
    logf("[host] device %u mounted VID:PID=%04X:%04X", dev_addr, vid, pid);
}

void tuh_umount_cb(uint8_t dev_addr) {
    g_host_devs--;
    logf("[host] device %u unmounted", dev_addr);
}

void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t idx, uint8_t const* desc_report, uint16_t desc_len) {
    uint8_t proto = tuh_hid_interface_protocol(dev_addr, idx);
    bool is_mouse = false, has_ids = false;
    static uint8_t ids[32], filtered[kMaxDescLen];
    uint16_t filtered_len = 0;
    bool ok = desc_len > 0 && desc_len <= kMaxDescLen &&
              parse_descriptor(desc_report, desc_len, is_mouse, has_ids, ids, filtered, filtered_len);
    logf("[host] HID itf dev=%u idx=%u proto=%u desc_len=%u parse=%d mouse=%d ids=%d",
         dev_addr, idx, proto, desc_len, ok, is_mouse, has_ids);

    if (!ok) return;
    if (g_idx != 0xFF) { logf("[host] already forwarding another interface, ignoring"); return; }
    if (!is_mouse && proto != HID_ITF_PROTOCOL_MOUSE) { logf("[host] not a mouse interface, ignoring"); return; }

    if (filtered_len == 0) {               // no Mouse collection found (boot mouse): pass descriptor as is
        memcpy(filtered, desc_report, desc_len);
        filtered_len = desc_len;
    }
    memcpy(g_mouse_ids, ids, sizeof(g_mouse_ids));
    if (has_ids) {
        char txt[3 * 32 + 1] = "";
        size_t n = 0;
        for (int id = 0; id < 256 && n < 90; id++)
            if (g_mouse_ids[id >> 3] & (1u << (id & 7))) n += sprintf(txt + n, "%02X ", id);
        logf("[host] mouse report IDs: %s", n ? txt : "NONE (all reports will be blocked)");
    }
    logf("[host] descriptor filtered %u -> %u bytes (non-mouse collections removed)", desc_len, filtered_len);
    memcpy(g_stage, filtered, filtered_len);
    g_has_ids = has_ids;
    g_stage_len = filtered_len;
    g_dev_addr = dev_addr;
    g_idx = idx;
    g_mouse_present = true;
    __dmb();
    g_changed = true;

    if (!tuh_hid_receive_report(dev_addr, idx)) logf("[host] tuh_hid_receive_report FAILED");
    logf("[host] forwarding this mouse interface");
}

void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t idx) {
    if (dev_addr != g_dev_addr || idx != g_idx) return;
    logf("[host] mouse interface removed");
    g_idx = 0xFF;
    g_dev_addr = 0;
    g_mouse_present = false;
    __dmb();
    g_changed = true;
}

void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t idx, uint8_t const* report, uint16_t len) {
    if (dev_addr == g_dev_addr && idx == g_idx && len > 0) {
        g_rx_reports++;
        if (g_rx_reports <= 5) {
            char hex[3 * 16 + 1];
            uint16_t n = len > 16 ? 16 : len;
            for (uint16_t i = 0; i < n; i++) sprintf(hex + 3 * i, "%02X ", report[i]);
            logf("[host] report #%lu len=%u: %s", (unsigned long)g_rx_reports, len, hex);
        }
        Report r{};
        if (g_has_ids) {
            uint8_t id = report[0];
            if (!(g_mouse_ids[id >> 3] & (1u << (id & 7)))) {
                if (++g_filtered <= 5) logf("[host] blocked report with non-mouse ID %02X len=%u", id, len);
                tuh_hid_receive_report(dev_addr, idx);
                return;
            }
            r.id = id;
            len -= 1;
            report += 1;
        }
        r.len = len > sizeof(r.data) ? sizeof(r.data) : (uint8_t)len;
        memcpy(r.data, report, r.len);
        if (!queue_try_add(&g_report_q, &r)) g_dropped++;
    }
    tuh_hid_receive_report(dev_addr, idx);
}

}  // extern "C"

static void core1_main() {
    pio_usb_configuration_t cfg = PIO_USB_DEFAULT_CONFIG;
    cfg.pin_dp = PIN_USB_HOST_DP;
    tuh_configure(BOARD_TUH_RHPORT, TUH_CFGID_RPI_PIO_USB_CONFIGURATION, &cfg);

    // TinyUSB defaults boot-class HID interfaces to BOOT protocol (4-byte reports, no report ID),
    // but we forward the descriptor of the full REPORT protocol, so request REPORT protocol.
    tuh_hid_set_default_protocol(HID_PROTOCOL_REPORT);

    tusb_rhport_init_t host_init{};
    host_init.role  = TUSB_ROLE_HOST;
    host_init.speed = TUSB_SPEED_AUTO;
    tusb_init(BOARD_TUH_RHPORT, &host_init);
    logf("[host] PIO-USB host started, D+=GP%d D-=GP%d", PIN_USB_HOST_DP, PIN_USB_HOST_DP + 1);

    while (true) {
        tuh_task();
    }
}

// ---- Device side (core 0) ----------------------------------------------
extern "C" {

static const tusb_desc_device_t kDeviceDesc = {
    sizeof(tusb_desc_device_t), TUSB_DESC_DEVICE, 0x0200,
    TUSB_CLASS_MISC, MISC_SUBCLASS_COMMON, MISC_PROTOCOL_IAD, CFG_TUD_ENDPOINT0_SIZE,
    0xCafe, 0x4102, 0x0100,       // TinyUSB test VID; not the mouse's real IDs
    0x01, 0x02, 0x00, 0x01
};

uint8_t const* tud_descriptor_device_cb(void) {
    return (uint8_t const*)&kDeviceDesc;
}

uint8_t const* tud_hid_descriptor_report_cb(uint8_t) {
    return g_desc;
}

uint8_t const* tud_descriptor_configuration_cb(uint8_t) {
#if ENABLE_CDC
    constexpr uint16_t total = TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN + TUD_CDC_DESC_LEN;
    constexpr uint8_t itf_count = 3;
#else
    constexpr uint16_t total = TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN;
    constexpr uint8_t itf_count = 1;
#endif
    // HID wDescriptorLength must match the mouse's report descriptor, so build on demand
    static uint8_t cfg[total];
    const uint8_t tmp[] = {
        TUD_CONFIG_DESCRIPTOR(1, itf_count, 0, total, 0, 100),
        TUD_HID_DESCRIPTOR(0, 0, HID_ITF_PROTOCOL_NONE, g_desc_len, 0x81, CFG_TUD_HID_EP_BUFSIZE, 1),
#if ENABLE_CDC
        TUD_CDC_DESCRIPTOR(1, 3, 0x82, 8, 0x03, 0x83, 64),
#endif
    };
    memcpy(cfg, tmp, sizeof(tmp));
    return cfg;
}

static const char* const kStrings[] = {
    "",                          // 0: language
    "Pico",                      // 1: manufacturer
    "Pico 2 Mouse Passthrough",  // 2: product
    "Pico 2 Debug Serial",       // 3: CDC interface
};

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t) {
    static uint16_t buf[32];
    uint8_t n;
    if (index == 0) {
        buf[1] = 0x0409;
        n = 1;
    } else {
        if (index >= sizeof(kStrings) / sizeof(kStrings[0])) return nullptr;
        const char* s = kStrings[index];
        n = (uint8_t)strlen(s);
        if (n > 31) n = 31;
        for (uint8_t i = 0; i < n; i++) buf[1 + i] = s[i];
    }
    buf[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * n + 2));
    return buf;
}

uint16_t tud_hid_get_report_cb(uint8_t, uint8_t, hid_report_type_t, uint8_t*, uint16_t) { return 0; }
void tud_hid_set_report_cb(uint8_t, uint8_t, hid_report_type_t, uint8_t const*, uint16_t) {}

void tud_mount_cb(void)   { logf("[dev] enumerated by computer"); }
void tud_umount_cb(void)  { logf("[dev] unmounted by computer"); }
void tud_suspend_cb(bool) { logf("[dev] bus suspended"); }
void tud_resume_cb(void)  { logf("[dev] bus resumed"); }

#if ENABLE_CDC
void tud_cdc_line_state_cb(uint8_t, bool dtr, bool) {
    if (dtr) logf("[dev] terminal connected. Buffered log follows.");
}
#endif

uint32_t tusb_time_millis_api(void) {
    return to_ms_since_boot(get_absolute_time());
}

}  // extern "C"

static void flush_log() {
#if ENABLE_CDC
    if (!tud_cdc_connected()) return;
    LogMsg m;
    while (queue_try_peek(&g_log_q, &m)) {
        size_t n = strlen(m.text);
        if (tud_cdc_write_available() < n) break;
        queue_try_remove(&g_log_q, &m);
        tud_cdc_write(m.text, n);
    }
    tud_cdc_write_flush();
#endif
}

static void apply_descriptor_change() {
    tud_disconnect();
    sleep_ms(400);                           // let the computer notice the unplug
    if (g_mouse_present && g_stage_len > 0) {
        g_desc_len = g_stage_len;
        memcpy(g_desc, g_stage, g_desc_len);
    } else {
        g_desc_len = sizeof(kPlaceholderDesc);
        memcpy(g_desc, kPlaceholderDesc, g_desc_len);
    }
    Report junk;
    while (queue_try_remove(&g_report_q, &junk)) {}
    ring_clear();
    g_inj_buttons = 0;
    g_real_buttons = 0;
    parse_layout(g_desc, g_desc_len, g_layout);
    if (g_layout.valid)
        logf("[dev] inject layout: id=%u bytes=%u btns=%u X=%u bit Y=%u bit wheel=%d pan=%d",
             g_layout.id, g_layout.bytes, g_layout.btn_count, g_layout.x.size, g_layout.y.size,
             g_layout.wheel.has, g_layout.pan.has);
    else
        logf("[dev] inject layout: NOT FOUND (typed commands will not work)");
    logf("[dev] re-enumerating with %s descriptor (%u bytes)",
         g_mouse_present ? "mouse" : "placeholder", g_desc_len);
    tud_connect();
#ifdef PICO_DEFAULT_LED_PIN
    gpio_put(PICO_DEFAULT_LED_PIN, g_mouse_present);
#endif
}

int main() {
    set_sys_clock_khz(120000, true);         // PIO-USB needs a multiple of 12 MHz

    queue_init(&g_report_q, sizeof(Report), 64);
    queue_init(&g_log_q, sizeof(LogMsg), 128);
    mutex_init(&g_log_mutex);

#ifdef PICO_DEFAULT_LED_PIN
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
#endif

    g_desc_len = sizeof(kPlaceholderDesc);
    memcpy(g_desc, kPlaceholderDesc, g_desc_len);

    logf("[boot] Pico 2 mouse passthrough starting");
    logf("[boot] RP2350 chip_version=%u (2 = A2; E9 affects A2 only)", rp2350_chip_version());
    multicore_launch_core1(core1_main);

    tusb_rhport_init_t dev_init{};
    dev_init.role  = TUSB_ROLE_DEVICE;
    dev_init.speed = TUSB_SPEED_AUTO;
    tusb_init(BOARD_TUD_RHPORT, &dev_init);

    uint32_t last_status = 0;
    while (true) {
        tud_task();

        if (g_changed) {
            g_changed = false;
            __dmb();
            apply_descriptor_change();
        }

        if (tud_hid_ready()) {
            Report r;
            uint16_t ib = 0;
            if (queue_try_remove(&g_report_q, &r)) {
                // real mouse report: remember its buttons, and add any buttons held by commands
                if (g_layout.valid && r.id == g_layout.id) {
                    g_real_buttons = get_buttons(r);
                    if (g_inj_buttons) set_buttons(r, g_real_buttons | g_inj_buttons);
                }
                if (tud_hid_report(r.id, r.data, r.len)) g_fwd_reports++;
            } else if (ring_pop(r, &ib)) {
                // command report: keep the real mouse's held buttons so they don't flicker
                set_buttons(r, ib | g_real_buttons);
                tud_hid_report(r.id, r.data, r.len);
            }
        }

        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (!g_quiet && now - last_status >= 2000) {
            last_status = now;
            logf("[status] host_devs=%d mouse=%d rx=%lu fwd=%lu drop=%lu filt=%lu pc=%d pins D+=%d D-=%d",
                 g_host_devs, g_mouse_present, (unsigned long)g_rx_reports,
                 (unsigned long)g_fwd_reports, (unsigned long)g_dropped, (unsigned long)g_filtered, tud_mounted(),
                 gpio_get(PIN_USB_HOST_DP), gpio_get(PIN_USB_HOST_DP + 1));
        }

#if ENABLE_CDC
        static char line[64];
        static uint8_t line_n = 0;
        while (tud_cdc_available()) {
            uint8_t c;
            if (tud_cdc_read(&c, 1) != 1) break;
            if (c == '\n' || c == '\r') {
                line[line_n] = 0;
                if (line_n) handle_command(line);
                line_n = 0;
            } else if (line_n < sizeof(line) - 1) {
                line[line_n++] = (char)c;
            } else {
                line_n = 0;                  // line too long: discard
            }
        }
#endif

        flush_log();
    }
}