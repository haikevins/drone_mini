#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <Arduino.h>

// Pin
static constexpr uint8_t g_pin_sck = 4u; // sck pin for both MPU6500 and BMP280
static constexpr uint8_t g_pin_miso = 5u; // miso pin for both MPU6500 and BMP280
static constexpr uint8_t g_pin_mosi = 6u; // mosi pin for both MPU6500 and BMP280
static constexpr uint8_t g_pin_ncs = 7u;  // chip select of MPU6500

// imu lpf filter
static constexpr float g_imu_gyro_lpf_alpha = 0.6f;
static constexpr float g_imu_accel_lpf_alpha = 0.8f;


#endif