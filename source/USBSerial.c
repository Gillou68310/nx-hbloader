#include "USBSerial.h"

void* USBSerial_interfaces[TOTAL_INTERFACES];
int USBSerial_interface_count;

static Result _initialize(struct USBSerial_t* _this, const char* str)
{
	Result rc = _this->com.initialize(&_this->com, str, &_this->data);
	if(R_SUCCEEDED(rc)) rc = _this->data.initialize(&_this->data, NULL);
	return rc;
}

static ssize_t _read(struct USBSerial_t* _this, char *ptr, size_t len)
{
	if(_this->com.DTE)
		return _this->data.read(&_this->data, ptr, len, 1000000000LL/*UINT64_MAX*/);
	else
		return -1;
}

static ssize_t _write(struct USBSerial_t* _this, const char *ptr, size_t len)
{
	if(_this->com.DTE)
		return _this->data.write(&_this->data, ptr, len, 1000000000LL/*UINT64_MAX*/);
	else
		return -1;
}

static ssize_t _sendEvent(struct USBSerial_t* _this, const char *ptr, size_t len)
{
	if(_this->com.DTE)
		return _this->com.sendEvent(&_this->com, ptr, len, 1000000000LL/*UINT64_MAX*/);
	else
		return -1;
}

void USBSerial(struct USBSerial_t* _this)
{
    _this->initialize = _initialize;
    _this->read = _read;
    _this->write = _write;
    _this->sendEvent = _sendEvent;

	USBSerialComInterface(&_this->com, USBSerial_interface_count);
	USBSerialDataInterface(&_this->data, USBSerial_interface_count + 1);
    USBSerial_interface_count += 2;
}