#include "USBSerialDataInterface.h"

#define EP_IN 0
#define EP_OUT 1

extern void* USBSerial_interfaces[TOTAL_INTERFACES];

static Result _initialize(struct USBSerialDataInterface_t* _this, const char* str)
{
    (void)str;
    struct usb_interface_descriptor interface_descriptor = {
        .bLength = USB_DT_INTERFACE_SIZE,
        .bDescriptorType = USB_DT_INTERFACE,
        .bNumEndpoints = 2,
        .bInterfaceClass = 0x0A,
        .bInterfaceSubClass = 0x00,
        .bInterfaceProtocol = 0x00,
    };
    struct usb_endpoint_descriptor endpoint_descriptor_in = {
       .bLength = USB_DT_ENDPOINT_SIZE,
       .bDescriptorType = USB_DT_ENDPOINT,
       .bEndpointAddress = USB_ENDPOINT_IN,
       .bmAttributes = USB_TRANSFER_TYPE_BULK,
       .wMaxPacketSize = 0x200,
    };
    struct usb_endpoint_descriptor endpoint_descriptor_out = {
       .bLength = USB_DT_ENDPOINT_SIZE,
       .bDescriptorType = USB_DT_ENDPOINT,
       .bEndpointAddress = USB_ENDPOINT_OUT,
       .bmAttributes = USB_TRANSFER_TYPE_BULK,
       .wMaxPacketSize = 0x200,
    };

    Result rc = usbAddStringDescriptor(&interface_descriptor.iInterface, "CDC ACM Data");
    if (R_SUCCEEDED(rc)) rc = usbInterfaceInit(_this->interface_index, &interface_descriptor, NULL);
    if (R_SUCCEEDED(rc)) rc = usbAddEndpoint(_this->interface_index, EP_IN, &endpoint_descriptor_in);
    if (R_SUCCEEDED(rc)) rc = usbAddEndpoint(_this->interface_index, EP_OUT, &endpoint_descriptor_out);
    if (R_SUCCEEDED(rc)) rc = usbEnableInterface(_this->interface_index);
    return rc;
}

static ssize_t _read(struct USBSerialDataInterface_t* _this, char *ptr, size_t len, u64 timestamp)
{
    return usbTransfer(_this->interface_index, EP_OUT, UsbDirection_Read, (void*)ptr, len, timestamp);
}
static ssize_t _write(struct USBSerialDataInterface_t* _this, const char *ptr, size_t len, u64 timestamp)
{
    return usbTransfer(_this->interface_index, EP_IN, UsbDirection_Write, (void*)ptr, len, timestamp);
}

void USBSerialDataInterface(struct USBSerialDataInterface_t* _this, int index)
{
    _this->initialize = _initialize;
    _this->read = _read;
    _this->write = _write;
    _this->interface_index = index;

    USBSerial_interfaces[index] = (void*)_this;
}