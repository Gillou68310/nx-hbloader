#ifndef __USB_SERIAL_DATA_INTERFACE_H
#define __USB_SERIAL_DATA_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usb.h"

struct USBSerialDataInterface_t {
    Result (*initialize)(struct USBSerialDataInterface_t*, const char*);
    ssize_t (*read)(struct USBSerialDataInterface_t*, char *, size_t, u64);
    ssize_t (*write)(struct USBSerialDataInterface_t*, const char *, size_t, u64);
    int interface_index;
};

void USBSerialDataInterface(struct USBSerialDataInterface_t* _this, int index);

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* __USB_SERIAL_DATA_INTERFACE_H */
