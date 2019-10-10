#ifndef __USB_SERIAL_H
#define __USB_SERIAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "USBSerialComInterface.h"
#include "USBSerialDataInterface.h"

struct USBSerial_t {
    Result (*initialize)(struct USBSerial_t*, const char*);
    ssize_t (*read)(struct USBSerial_t*, char *, size_t);
    ssize_t (*write)(struct USBSerial_t*, const char *, size_t);
    ssize_t (*sendEvent)(struct USBSerial_t*, const char *, size_t);
    struct USBSerialComInterface_t com;
    struct USBSerialDataInterface_t data;
};

void USBSerial(struct USBSerial_t* _this);
extern void* USBSerial_interfaces[TOTAL_INTERFACES];
extern int USBSerial_interface_count;

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* __USB_SERIAL_H */
