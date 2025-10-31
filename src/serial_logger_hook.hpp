#ifndef SERIAL_LOGGER_HOOK_HPP
#define SERIAL_LOGGER_HOOK_HPP

#include "serial_logger.hpp"
#include <Arduino.h>
#include <cstdarg>

class SerialLoggerHook : public Print {
private:
    Print* target;
    
public:
    SerialLoggerHook(Print* t) : target(t) {}
    
    size_t write(uint8_t c) override {
        serial_logger_add_char((char)c);
        return target->write(c);
    }
    
    size_t write(const uint8_t* buffer, size_t size) override {
        for (size_t i = 0; i < size; i++) {
            serial_logger_add_char((char)buffer[i]);
        }
        return target->write(buffer, size);
    }
    
    void flush() { 
        serial_logger_flush();
        target->flush(); 
    }
    
    size_t printf(const char* format, ...) {
        va_list args;
        va_start(args, format);
        char temp[512];
        int len = vsnprintf(temp, sizeof(temp), format, args);
        va_end(args);
        
        if (len > 0 && len < (int)sizeof(temp)) {
            serial_logger_add(temp);
        }
        
        va_start(args, format);
        size_t result = target->printf(format, args);
        va_end(args);
        return result;
    }
    
    size_t println(const char* str) {
        if (str) {
            serial_logger_add(str);
        }
        return target->println(str);
    }
    
    size_t print(const char* str) {
        if (str) {
            for (const char* ptr = str; *ptr; ptr++) {
                serial_logger_add_char(*ptr);
            }
        }
        return target->print(str);
    }
};

#endif

