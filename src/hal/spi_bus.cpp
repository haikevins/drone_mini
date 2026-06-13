/*
 * spi_bus.cpp
 * SPI bus abstraction for device access.
 */

#include "hal/spi_bus.h"

SPIBus::SPIBus (SPIClass & spi_class) : p_spi(&spi_class)
{}

void SPIBus::begin (int sck_pin, int miso_pin, int mosi_pin, uint32_t frequency_hz, uint8_t data_mode)
{
    p_spi->begin(sck_pin, miso_pin, mosi_pin);

    spi_settings = SPISettings(frequency_hz, MSBFIRST, data_mode);
}

void SPIBus::write_register (uint8_t chip_select_pin, uint8_t reg_addr, uint8_t reg_value)
{
    p_spi->beginTransaction(spi_settings);

    digitalWrite(chip_select_pin, LOW);

    p_spi->transfer(reg_addr & 0x7Fu);
    p_spi->transfer(reg_value);

    digitalWrite(chip_select_pin, HIGH);

    p_spi->endTransaction();
}

uint8_t SPIBus::read_register (uint8_t chip_select_pin, uint8_t reg_addr)
{
    p_spi->beginTransaction(spi_settings);

    digitalWrite(chip_select_pin, LOW);

    p_spi->transfer(reg_addr | 0x80u);
    const uint8_t reg_value = p_spi->transfer(0x00u);

    digitalWrite(chip_select_pin, HIGH);

    p_spi->endTransaction();

    return reg_value;
}

void SPIBus::read_registers (uint8_t chip_select_pin, uint8_t start_reg_addr, uint8_t * p_buffer, size_t length)
{
    p_spi->beginTransaction(spi_settings);

    digitalWrite(chip_select_pin, LOW);

    p_spi->transfer(start_reg_addr | 0x80u);

    for (size_t byte_index = 0u; byte_index < length; ++byte_index)
    {
        p_buffer[byte_index] = p_spi->transfer(0x00u);
    }

    digitalWrite(chip_select_pin, HIGH);

    p_spi->endTransaction();
}
