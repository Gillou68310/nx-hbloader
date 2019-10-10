#ifndef __USB_SERIAL_COM_INTERFACE_H
#define __USB_SERIAL_COM_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usb.h"
#include "USBSerialDataInterface.h"

struct line_coding {
	uint32_t dwDTERate;
	uint8_t bCharFormat;
	uint8_t bParityType;
	uint8_t bDataBits;
} PACKED;

struct setup_thread_args {
    int* quit;
    void** interface;
    int* interface_count;
};

struct USBSerialComInterface_t {
    Result (*initialize)(struct USBSerialComInterface_t*, const char*, struct USBSerialDataInterface_t *);
    ssize_t (*sendEvent)(struct USBSerialComInterface_t*, const char *, size_t, u64);
    int interface_index;
    struct line_coding coding;
    bool DTE;
    bool Carrier;
};

void USBSerialComInterface(struct USBSerialComInterface_t* _this, int index);
void *handle_setup_packet(void* args);

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* __USB_SERIAL_COM_INTERFACE_H */
