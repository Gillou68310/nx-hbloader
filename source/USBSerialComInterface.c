#include "USBSerialComInterface.h"

extern void* USBSerial_interfaces[TOTAL_INTERFACES];
extern int USBSerial_interface_count;

struct usb_cdc_header {
    uint8_t  bFunctionLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubType;
    uint16_t bcdCDC;
} PACKED;

struct usb_cdc_call_mgmt {
    uint8_t  bFunctionLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubType;
    uint8_t  bmCapabilities;
    uint8_t  bDataInterface;
};

struct usb_cdc_acm {
    uint8_t  bFunctionLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubType;
    uint8_t  bmCapabilities;
};

struct usb_cdc_union {
    uint8_t  bFunctionLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubType;
    uint8_t  bControlInterface;
    uint8_t  bSubordinateInterface0;
};

 struct usb_ctrlrequest {
   uint8_t bRequestType;
   uint8_t bRequest;
   uint16_t wValue;
   uint16_t wIndex;
   uint16_t wLength;
 } PACKED;

#define EP0 0xFFFFFFFF
#define EP_INT 0

#define GET_LINE_CODING           0x21
#define SET_LINE_CODING           0x20
#define SET_CONTROL_LINE_STATE    0x22
#define SEND_BREAK                0x23

static Result _initialize(struct USBSerialComInterface_t* _this, const char* str, struct USBSerialDataInterface_t *data)
{
    struct usb_interface_association_descriptor interface_association_descriptor = {
        .bLength = 0x08,
        .bDescriptorType = 0x0B,
        .bFirstInterface = 0x00,
        .bInterfaceCount = 0x02,
        .bFunctionClass = 0x02,
        .bFunctionSubClass = 0x02,
        .bFunctionProtocol = 0x01,
        .iFunction = 0x08,
    };
    struct usb_interface_descriptor interface_descriptor = {
        .bLength = USB_DT_INTERFACE_SIZE,
        .bDescriptorType = USB_DT_INTERFACE,
        .bNumEndpoints = 0x01,
        .bInterfaceClass = 0x02,
        .bInterfaceSubClass = 0x02,
        .bInterfaceProtocol = 0x01,
    };
    struct usb_cdc_header cdc_com_header = {
        .bFunctionLength = 0x05,
        .bDescriptorType = 0x24,
        .bDescriptorSubType = 0x00,
        .bcdCDC = 0x110,
    };
    struct usb_cdc_call_mgmt cdc_com_call_mgmt = {
        .bFunctionLength = 0x05,
        .bDescriptorType = 0x24,
        .bDescriptorSubType = 0x01,
        .bmCapabilities = 0x03,
        .bDataInterface = 0x01,
    };
    struct usb_cdc_acm cdc_com_acm = {
        .bFunctionLength = 0x04,
        .bDescriptorType = 0x24,
        .bDescriptorSubType = 0x02,
        .bmCapabilities = 0x02,
    };
    struct usb_cdc_union cdc_com_union = {
        .bFunctionLength = 0x05,
        .bDescriptorType = 0x24,
        .bDescriptorSubType = 0x06,
        .bControlInterface = 0x00,
        .bSubordinateInterface0 = 0x01,
    };
    struct usb_endpoint_descriptor endpoint_descriptor_interrupt = {
        .bLength = USB_DT_ENDPOINT_SIZE,
        .bDescriptorType = USB_DT_ENDPOINT,
        .bEndpointAddress = USB_ENDPOINT_IN,
        .bmAttributes = USB_TRANSFER_TYPE_INTERRUPT,
        .wMaxPacketSize = 0xA,
        .bInterval = 9,
    };

    interface_association_descriptor.bFirstInterface = _this->interface_index;
    cdc_com_call_mgmt.bDataInterface = data->interface_index + 1;
    cdc_com_union.bControlInterface = _this->interface_index;
    cdc_com_union.bSubordinateInterface0 = data->interface_index + 1;

    Result rc = usbAddStringDescriptor(&interface_association_descriptor.iFunction, str);
    if (R_SUCCEEDED(rc)) rc = usbAddStringDescriptor(&interface_descriptor.iInterface, "CDC Abstract Control Model (ACM)");
    if (R_SUCCEEDED(rc)) rc = usbInterfaceInit(_this->interface_index, &interface_descriptor, &interface_association_descriptor);
    if (R_SUCCEEDED(rc)) rc = usbInterfaceAddData(_this->interface_index, (char*)&cdc_com_header);
    if (R_SUCCEEDED(rc)) rc = usbInterfaceAddData(_this->interface_index, (char*)&cdc_com_call_mgmt);
    if (R_SUCCEEDED(rc)) rc = usbInterfaceAddData(_this->interface_index, (char*)&cdc_com_acm);
    if (R_SUCCEEDED(rc)) rc = usbInterfaceAddData(_this->interface_index, (char*)&cdc_com_union);
    if (R_SUCCEEDED(rc)) rc = usbAddEndpoint(_this->interface_index, EP_INT, &endpoint_descriptor_interrupt);
    if (R_SUCCEEDED(rc)) rc = usbEnableInterface(_this->interface_index);
    return rc;
}

static ssize_t _sendEvent(struct USBSerialComInterface_t* _this, const char *ptr, size_t len, u64 timestamp)
{
    return usbTransfer(_this->interface_index, EP_INT, UsbDirection_Write, (void*)ptr, len, timestamp);
}

void USBSerialComInterface(struct USBSerialComInterface_t* _this, int index)
{
    _this->initialize = _initialize;
    _this->sendEvent = _sendEvent;

    _this->interface_index = index;
    _this->DTE = 0;
    _this->Carrier = 0;

    _this->coding.dwDTERate = 9600;
    _this->coding.bCharFormat = 0;
    _this->coding.bParityType = 0;
    _this->coding.bDataBits = 0x08;

    USBSerial_interfaces[index] = (void*)_this;
}

void *handle_setup_packet(void* args)
{
    int size;
    Result rc;
    struct usb_ctrlrequest ctrl;
    struct setup_thread_args* setup_args = (struct setup_thread_args*)args;
    
    while(!*setup_args->quit)
    {
        rc = usbGetSetupPacket(0, &ctrl, sizeof(struct usb_ctrlrequest), 1000000000LL);
        if (R_SUCCEEDED(rc)) {

            if(ctrl.wIndex >= *setup_args->interface_count)
                continue;
                
            struct USBSerialComInterface_t* com = (struct USBSerialComInterface_t*)setup_args->interface[ctrl.wIndex];

            if(com->initialize != _initialize) //Not a com interface
                continue;

            int interface = com->interface_index;
            struct line_coding *coding = &com->coding;

            switch (ctrl.bRequest) {
                case GET_LINE_CODING:
                    size = usbTransfer(interface, EP0, UsbDirection_Write, coding, sizeof(struct line_coding), UINT64_MAX);
                    if(size == sizeof(struct line_coding))
                        size = usbTransfer(interface, EP0, UsbDirection_Read, NULL, 0, UINT64_MAX);
                    break;
                case SET_LINE_CODING:
                    size = usbTransfer(interface, EP0, UsbDirection_Read, coding, sizeof(struct line_coding), UINT64_MAX);
                    if (size == sizeof(struct line_coding))
                        size = usbTransfer(interface, EP0, UsbDirection_Write, NULL, 0, UINT64_MAX);
                    break;
                case SET_CONTROL_LINE_STATE:
                    com->Carrier = (ctrl.wValue & 0x02) ? 1 : 0;
                    com->DTE = (ctrl.wValue & 0x01) ? 1 : 0;
                    size = usbTransfer(interface, EP0, UsbDirection_Write, NULL, 0, UINT64_MAX);
                    break;    
                default:
                    break;
            }
        }
    }
    return NULL;
}