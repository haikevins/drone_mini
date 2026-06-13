#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <Arduino.h>

static float ax = 0;
static float ay = 0;
static float az = 0;

static float gx = 0;
static float gy = 0;
static float gz = 0;

typedef struct
{
    float roll = 0;
    float pitch = 0;
    float yaw = 0;
}
attitude_data_t;

static constexpr float target_roll_angle = 0.0f;
static constexpr float target_pitch_angle = 0.0f;
static constexpr float target_yaw_angle = 0.0f;

// Pin
static constexpr uint8_t g_pin_sck = 4u; // sck pin for both MPU6500 and BMP280
static constexpr uint8_t g_pin_miso = 5u; // miso pin for both MPU6500 and BMP280
static constexpr uint8_t g_pin_mosi = 6u; // mosi pin for both MPU6500 and BMP280
static constexpr uint8_t g_pin_ncs = 7u;  // chip select of MPU6500

static constexpr float g_imu_read_default_hz = 1000.0f; // imu sample frequency default

// imu lpf filter
static constexpr float g_imu_gyro_lpf_alpha = 0.22f; // cutoff 40Hz
static constexpr float g_imu_accel_lpf_alpha = 0.10f; // cutoff 20Hz

static constexpr float g_serial_print_period_ms = 500.0f; // period to print debug info to serial
static uint32_t last_print_time = 0;

#endif