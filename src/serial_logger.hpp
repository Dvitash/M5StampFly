#ifndef SERIAL_LOGGER_HPP
#define SERIAL_LOGGER_HPP

#include <Arduino.h>
#include <stdint.h>

#define MAX_LOG_MESSAGES 200
#define MAX_MESSAGE_LENGTH 256

void serial_logger_init(void);
void serial_logger_set_target(void* serial);
void serial_logger_add(const char* msg);
void serial_logger_add_char(char c);
void serial_logger_flush(void);
void serial_logger_get_json(String& output, uint32_t& last_id);

void serial_logger_printf(const char* format, ...);
void serial_logger_println(const char* msg);
void serial_logger_print(const char* msg);

// wrapper functions that intercept USBSerial calls
// use these instead of direct USBSerial calls to enable logging
size_t print(const char* format, ...);
void serial_logger_usb_println(const char* msg);
void serial_logger_usb_print(const char* msg);

// overloads for different types
void serial_logger_usb_println(int val);
void serial_logger_usb_println(unsigned int val);
void serial_logger_usb_println(long val);
void serial_logger_usb_println(unsigned long val);
void serial_logger_usb_println(float val);
void serial_logger_usb_println(const String& val);
void serial_logger_usb_print(int val, int base = DEC);
void serial_logger_usb_print(unsigned int val, int base = DEC);
void serial_logger_usb_print(long val, int base = DEC);
void serial_logger_usb_print(unsigned long val, int base = DEC);
void serial_logger_usb_print(float val, int digits = 2);
void serial_logger_usb_print(const String& val);

// global hook instance - will be initialized to wrap USBSerial
extern Print* serial_logger_hook_target;

// function to intercept all write calls to USBSerial
// this should be called whenever data is written to serial
void serial_logger_intercept_write_byte(uint8_t c);
void serial_logger_intercept_write_buffer(const uint8_t* buffer, size_t size);

#endif

