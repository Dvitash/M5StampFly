#pragma once
#include <stdint.h>
#include "common.h"       // must declare pmw_read_reg, pmw_write_reg, pmw_read_burst
#include "esp_rom_sys.h"  // ets_delay_us

class PMW3901 {
   public:
    // csPin is ignored; CS is owned by spi_bus_add_device(pmw3901_cfg)
    bool begin() {
        // Datasheet suggests initial idle delays; be conservative.
        ets_delay_us(40000);

        // Verify IDs
        uint8_t chipId    = registerRead(0x00);
        uint8_t invChipId = registerRead(0x5F);
        
        // debug: print chip IDs if they don't match
        if (chipId != 0x49 || invChipId != 0xB6) {
            // Note: USBSerial might not be available here, but we can try
            // Will be handled by caller
            return false;
        }

        // Power-on reset
        registerWrite(0x3A, 0x5A);
        ets_delay_us(5000);

        // Dummy reads
        (void)registerRead(0x02);
        (void)registerRead(0x03);
        (void)registerRead(0x04);
        (void)registerRead(0x05);
        (void)registerRead(0x06);
        ets_delay_us(1000);

        // Load recommended register table
        initRegisters();

        return true;
    }

    void readMotion(int16_t &deltaX, int16_t &deltaY, bool &gotMotion) {
        pmw_burst_t burst;
        pmw_read_burst(&burst);

        deltaX = burst.dx;
        deltaY = burst.dy;

        // Datasheet: motion register == 0xB0 when new data valid
        gotMotion = (burst.motion == 0xB0);
    }

    void readMotion(int16_t &deltaX, int16_t &deltaY, bool &gotMotion, uint8_t &squal) {
        pmw_burst_t burst;
        pmw_read_burst(&burst);

        deltaX = burst.dx;
        deltaY = burst.dy;
        squal = burst.squal;

        // Datasheet: motion register == 0xB0 when new data valid
        gotMotion = (burst.motion == 0xB0);
    }

    void readMotion(int16_t &dx, int16_t &dy) {
        bool dummy;
        readMotion(dx, dy, dummy);
    }

   private:
    // Thin wrappers around common.c
    inline uint8_t registerRead(uint8_t reg) {
        uint8_t v = pmw_read_reg(reg);
        // Optional guard time NCS-high; usually not required with spi_master, but harmless:
        ets_delay_us(50);
        return v;
    }

    inline void registerWrite(uint8_t reg, uint8_t val) {
        pmw_write_reg(reg, val);
        // Optional guard time after write:
        ets_delay_us(200);
    }

    void initRegisters() {
        registerWrite(0x7F, 0x00);
        registerWrite(0x61, 0xAD);
        registerWrite(0x7F, 0x03);
        registerWrite(0x40, 0x00);
        registerWrite(0x7F, 0x05);
        registerWrite(0x41, 0xB3);
        registerWrite(0x43, 0xF1);
        registerWrite(0x45, 0x14);
        registerWrite(0x5B, 0x32);
        registerWrite(0x5F, 0x34);
        registerWrite(0x7B, 0x08);
        registerWrite(0x7F, 0x06);
        registerWrite(0x44, 0x1B);
        registerWrite(0x40, 0xBF);
        registerWrite(0x4E, 0x3F);
        registerWrite(0x7F, 0x08);
        registerWrite(0x65, 0x20);
        registerWrite(0x6A, 0x18);
        registerWrite(0x7F, 0x09);
        registerWrite(0x4F, 0xAF);
        registerWrite(0x5F, 0x40);
        registerWrite(0x48, 0x80);
        registerWrite(0x49, 0x80);
        registerWrite(0x57, 0x77);
        registerWrite(0x60, 0x78);
        registerWrite(0x61, 0x78);
        registerWrite(0x62, 0x08);
        registerWrite(0x63, 0x50);
        registerWrite(0x7F, 0x0A);
        registerWrite(0x45, 0x60);
        registerWrite(0x7F, 0x00);
        registerWrite(0x4D, 0x11);
        registerWrite(0x55, 0x80);
        registerWrite(0x74, 0x1F);
        registerWrite(0x75, 0x1F);
        registerWrite(0x4A, 0x78);
        registerWrite(0x4B, 0x78);
        registerWrite(0x44, 0x08);
        registerWrite(0x45, 0x50);
        registerWrite(0x64, 0xFF);
        registerWrite(0x65, 0x1F);
        registerWrite(0x7F, 0x14);
        registerWrite(0x65, 0x67);
        registerWrite(0x66, 0x08);
        registerWrite(0x63, 0x70);
        registerWrite(0x7F, 0x15);
        registerWrite(0x48, 0x48);
        registerWrite(0x7F, 0x07);
        registerWrite(0x41, 0x0D);
        registerWrite(0x43, 0x14);
        registerWrite(0x4B, 0x0E);
        registerWrite(0x45, 0x0F);
        registerWrite(0x44, 0x42);
        registerWrite(0x4C, 0x80);
        registerWrite(0x7F, 0x10);
        registerWrite(0x5B, 0x02);
        registerWrite(0x7F, 0x07);
        registerWrite(0x40, 0x41);
        registerWrite(0x70, 0x00);

        ets_delay_us(10000);

        registerWrite(0x32, 0x44);
        registerWrite(0x7F, 0x07);
        registerWrite(0x40, 0x40);
        registerWrite(0x7F, 0x06);
        registerWrite(0x62, 0xF0);
        registerWrite(0x63, 0x00);
        registerWrite(0x7F, 0x0D);
        registerWrite(0x48, 0xC0);
        registerWrite(0x6F, 0xD5);
        registerWrite(0x7F, 0x00);
        registerWrite(0x5B, 0xA0);
        registerWrite(0x4E, 0xA8);
        registerWrite(0x5A, 0x50);
        registerWrite(0x40, 0x80);

        registerWrite(0x7F, 0x00);
        registerWrite(0x5A, 0x10);
        registerWrite(0x54, 0x00);
    }
};
