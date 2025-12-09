#include "serial_logger.hpp"
#include "serial_logger_hook.hpp"
#include <Arduino.h>
#include <cstring>
#include <cstdarg>
#include <cstdlib>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

struct LogEntry {
    char msg[MAX_MESSAGE_LENGTH];
    uint32_t id;
    uint32_t timestamp_ms;
};

static LogEntry log_buffer[MAX_LOG_MESSAGES];
static uint32_t log_write_index                      = 0;
static uint32_t log_id_counter                       = 0;
static uint32_t log_count                            = 0;
static String log_line_buffer                        = "";
static void* serial_target                           = nullptr;
Print* serial_logger_hook_target                     = nullptr;
static SerialLoggerHook* serial_logger_hook_instance = nullptr;
static SemaphoreHandle_t log_mutex                   = nullptr;

static inline void log_lock() {
    if (log_mutex) xSemaphoreTake(log_mutex, portMAX_DELAY);
}

static inline void log_unlock() {
    if (log_mutex) xSemaphoreGive(log_mutex);
}

static void log_store_unlocked(const char* msg, size_t len) {
    if (!msg || len == 0) return;
    if (len > MAX_MESSAGE_LENGTH - 1) len = MAX_MESSAGE_LENGTH - 1;

    uint32_t idx = log_write_index % MAX_LOG_MESSAGES;
    memcpy(log_buffer[idx].msg, msg, len);
    log_buffer[idx].msg[len]     = '\0';
    log_buffer[idx].id           = log_id_counter++;
    log_buffer[idx].timestamp_ms = millis();

    log_write_index++;
    if (log_count < MAX_LOG_MESSAGES) {
        log_count++;
    }
}

void serial_logger_init(void) {
    if (!log_mutex) {
        log_mutex = xSemaphoreCreateMutex();
    }
    log_lock();
    memset(log_buffer, 0, sizeof(log_buffer));
    log_write_index = 0;
    log_id_counter  = 0;
    log_count       = 0;
    log_line_buffer = "";
    log_unlock();
}

void serial_logger_set_target(void* serial) {
    serial_target             = serial;
    serial_logger_hook_target = (Print*)serial;
    if (serial_logger_hook_instance) {
        delete serial_logger_hook_instance;
    }
    serial_logger_hook_instance = new SerialLoggerHook((Print*)serial);
}

void serial_logger_intercept_write_byte(uint8_t c) {
    serial_logger_add_char((char)c);
}

void serial_logger_intercept_write_buffer(const uint8_t* buffer, size_t size) {
    for (size_t i = 0; i < size; i++) {
        serial_logger_add_char((char)buffer[i]);
    }
}

void serial_logger_printf(const char* format, ...) {
    if (!serial_target) return;

    char temp[512];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(temp, sizeof(temp), format, args);
    va_end(args);

    if (len > 0 && len < (int)sizeof(temp)) {
        serial_logger_add(temp);
    }

    Print* p = (Print*)serial_target;
    va_start(args, format);
    p->printf(format, args);
    va_end(args);
}

void serial_logger_println(const char* msg) {
    if (!serial_target) return;
    Print* p = (Print*)serial_target;
    if (msg) {
        serial_logger_add(msg);
    }
    p->println(msg);
}

void serial_logger_print(const char* msg) {
    if (!serial_target) return;
    Print* p = (Print*)serial_target;
    if (msg) {
        for (const char* ptr = msg; *ptr; ptr++) {
            serial_logger_add_char(*ptr);
        }
    }
    p->print(msg);
}

void serial_logger_add(const char* msg) {
    if (!msg) return;

    size_t len = strlen(msg);
    if (len == 0) return;

    log_lock();
    log_store_unlocked(msg, len);
    log_unlock();
}

size_t print(const char* format, ...) {
    char loc_buf[64];
    char* temp = loc_buf;
    va_list arg;
    va_list copy;
    va_start(arg, format);
    va_copy(copy, arg);
    int len = vsnprintf(temp, sizeof(loc_buf), format, copy);
    va_end(copy);

    if (len < 0) {
        va_end(arg);
        return 0;
    }

    if (len >= (int)sizeof(loc_buf)) {
        temp = (char*)malloc(len + 1);
        if (temp == NULL) {
            va_end(arg);
            return 0;
        }
        len = vsnprintf(temp, len + 1, format, arg);
    }

    if (len > 0 && len < MAX_MESSAGE_LENGTH) {
        serial_logger_add(temp);
    }

    va_end(arg);
    size_t written = USBSerial.write((uint8_t*)temp, len);
    if (temp != loc_buf) {
        free(temp);
    }
    return written;
}

void serial_logger_usb_println(const char* msg) {
    if (msg) {
        serial_logger_add(msg);
    }
    USBSerial.println(msg);
}

void serial_logger_usb_print(const char* msg) {
    if (msg) {
        for (const char* ptr = msg; *ptr; ptr++) {
            serial_logger_add_char(*ptr);
        }
    }
    USBSerial.print(msg);
}

void serial_logger_usb_println(int val) {
    String s = String(val);
    serial_logger_add(s.c_str());
    USBSerial.println(val);
}

void serial_logger_usb_println(unsigned int val) {
    String s = String(val);
    serial_logger_add(s.c_str());
    USBSerial.println(val);
}

void serial_logger_usb_println(long val) {
    String s = String(val);
    serial_logger_add(s.c_str());
    USBSerial.println(val);
}

void serial_logger_usb_println(unsigned long val) {
    String s = String(val);
    serial_logger_add(s.c_str());
    USBSerial.println(val);
}

void serial_logger_usb_println(float val) {
    String s = String(val);
    serial_logger_add(s.c_str());
    USBSerial.println(val);
}

void serial_logger_usb_println(const String& val) {
    serial_logger_add(val.c_str());
    USBSerial.println(val);
}

void serial_logger_usb_print(int val, int base) {
    String s = String(val, base);
    for (size_t i = 0; i < s.length(); i++) {
        serial_logger_add_char(s[i]);
    }
    USBSerial.print(val, base);
}

void serial_logger_usb_print(unsigned int val, int base) {
    String s = String(val, base);
    for (size_t i = 0; i < s.length(); i++) {
        serial_logger_add_char(s[i]);
    }
    USBSerial.print(val, base);
}

void serial_logger_usb_print(long val, int base) {
    String s = String(val, base);
    for (size_t i = 0; i < s.length(); i++) {
        serial_logger_add_char(s[i]);
    }
    USBSerial.print(val, base);
}

void serial_logger_usb_print(unsigned long val, int base) {
    String s = String(val, base);
    for (size_t i = 0; i < s.length(); i++) {
        serial_logger_add_char(s[i]);
    }
    USBSerial.print(val, base);
}

void serial_logger_usb_print(float val, int digits) {
    String s = String(val, digits);
    for (size_t i = 0; i < s.length(); i++) {
        serial_logger_add_char(s[i]);
    }
    USBSerial.print(val, digits);
}

void serial_logger_usb_print(const String& val) {
    for (size_t i = 0; i < val.length(); i++) {
        serial_logger_add_char(val[i]);
    }
    USBSerial.print(val);
}

void serial_logger_add_char(char c) {
    log_lock();
    if (c == '\n' || c == '\r') {
        if (log_line_buffer.length() > 0) {
            log_store_unlocked(log_line_buffer.c_str(), log_line_buffer.length());
            log_line_buffer = "";
        }
    } else {
        log_line_buffer += c;
        if (log_line_buffer.length() > MAX_MESSAGE_LENGTH - 1) {
            log_store_unlocked(log_line_buffer.c_str(), log_line_buffer.length());
            log_line_buffer = "";
        }
    }
    log_unlock();
}

void serial_logger_flush(void) {
    log_lock();
    if (log_line_buffer.length() > 0) {
        log_store_unlocked(log_line_buffer.c_str(), log_line_buffer.length());
        log_line_buffer = "";
    }
    log_unlock();
}

void serial_logger_get_json(String& output, uint32_t& last_id) {
    output.reserve(4096);
    output = "{\"logs\":[";

    log_lock();
    bool first         = true;
    uint32_t start_idx = 0;

    if (last_id > 0 && log_count > 0) {
        uint32_t oldest_id = (log_write_index >= log_count)
                                 ? log_buffer[(log_write_index - log_count) % MAX_LOG_MESSAGES].id
                                 : log_buffer[0].id;

        if (last_id >= oldest_id) {
            uint32_t offset = last_id - oldest_id + 1;
            start_idx       = (log_write_index - log_count + offset) % MAX_LOG_MESSAGES;
        }
    }

    uint32_t items_added = 0;
    uint32_t max_items   = (last_id > 0) ? MAX_LOG_MESSAGES : log_count;

    for (uint32_t i = 0; i < max_items && items_added < log_count; i++) {
        uint32_t idx = (start_idx + i) % MAX_LOG_MESSAGES;

        if (log_buffer[idx].id > last_id || last_id == 0) {
            if (!first) output += ",";
            first = false;

            output += "{\"id\":";
            output += String(log_buffer[idx].id);
            output += ",\"ts\":";
            output += String(log_buffer[idx].timestamp_ms);
            output += ",\"msg\":\"";

            String escaped = String(log_buffer[idx].msg);
            escaped.replace("\\", "\\\\");
            escaped.replace("\"", "\\\"");
            escaped.replace("\n", "\\n");
            escaped.replace("\r", "\\r");
            output += escaped;

            output += "\"}";

            last_id = log_buffer[idx].id;
            items_added++;
        }
    }
    log_unlock();
    output += "]}";
}
