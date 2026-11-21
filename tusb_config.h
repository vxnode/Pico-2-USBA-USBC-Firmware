#pragma once

#ifndef CFG_TUSB_MCU
#error CFG_TUSB_MCU must be defined
#endif
#ifndef CFG_TUSB_OS
#define CFG_TUSB_OS            OPT_OS_NONE
#endif

// Device stack: native USB-C port (rhport 0)
#define CFG_TUD_ENABLED        1
#define BOARD_TUD_RHPORT       0
#define CFG_TUD_ENDPOINT0_SIZE 64

// Host stack: bit-banged USB-A port via PIO (rhport 1)
#define CFG_TUH_ENABLED        1
#define BOARD_TUH_RHPORT       1
#define CFG_TUH_RPI_PIO_USB    1

// ---- Device: one HID interface, descriptor copied from the mouse ----
#define CFG_TUD_HID            1
#define CFG_TUD_HID_EP_BUFSIZE 64
// Set to 0 for a HID-only build (no COM port / no debug log)
#define ENABLE_CDC             1
#define CFG_TUD_CDC            ENABLE_CDC
#define CFG_TUD_CDC_EP_BUFSIZE 64
#define CFG_TUD_CDC_RX_BUFSIZE 64
#define CFG_TUD_CDC_TX_BUFSIZE 512
#define CFG_TUD_MSC            0
#define CFG_TUD_MIDI           0
#define CFG_TUD_VENDOR         0

// ---- Host ----
#define CFG_TUH_HUB                 1
#define CFG_TUH_DEVICE_MAX          (CFG_TUH_HUB ? 4 : 1)
#define CFG_TUH_ENUMERATION_BUFSIZE 512   // big enough for gaming-mouse report descriptors
#define CFG_TUH_HID                 (4 * CFG_TUH_DEVICE_MAX)
#define CFG_TUH_HID_EPIN_BUFSIZE    64
#define CFG_TUH_HID_EPOUT_BUFSIZE   64
