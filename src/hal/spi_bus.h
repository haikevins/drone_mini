/*
 * spi_bus.h
 * SPI bus abstraction for device access.
 */

#ifndef SPI_BUS_H
#define SPI_BUS_H

#include <Arduino.h>
#include <SPI.h>

class SPIBus
{            
    public:
        SPIBus (SPIClass & spi_class = SPI);

        void begin (int sck_pin, int miso_pin, int mosi_pin, uint32_t frequency_hz = 8000000u, uint8_t data_mode = SPI_MODE3);

        void write_register (uint8_t chip_select_pin, uint8_t reg_addr, uint8_t reg_value);
        uint8_t read_register (uint8_t chip_select_pin, uint8_t reg_addr);
        void read_registers (uint8_t chip_select_pin, uint8_t start_reg_addr, uint8_t * p_buffer, size_t length);

    private:
        SPIClass * p_spi;
        SPISettings spi_settings;
};

#endif /* SPI_BUS_H */
