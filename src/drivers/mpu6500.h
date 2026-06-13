/*
 * mpu6500.h
 * MPU6500 IMU driver interface.
 */

#ifndef MPU6500_H
#define MPU6500_H

#include <Arduino.h>
#include "hal/spi_bus.h"

class MPU6500
{
    public:
        struct raw_data_t
        {
            int16_t ax = 0;
            int16_t ay = 0;
            int16_t az = 0;

            int16_t gx = 0;
            int16_t gy = 0;
            int16_t gz = 0;
        };

        struct scaled_data_t
        {
            float ax = 0.0f;
            float ay = 0.0f;
            float az = 0.0f;

            float gx = 0.0f;
            float gy = 0.0f;
            float gz = 0.0f;
        };

        struct timing_t
        {
            uint32_t now_us = 0u;
            uint32_t last_us = 0u;
            float dt = 0.0f;
            float imu_dt = 0.0f;
        };

        bool begin(SPIBus * p_spi_bus, uint8_t chip_select_pin, int sck_pin, int miso_pin, int mosi_pin);

        /*
         * Main IMU pipeline:
         * wait data-ready
         * -> read raw
         * -> scale
         * -> apply bias and accel scale
         * -> rotate IMU frame to body frame
         * -> low-pass filter
         */
        bool update();

        void calibrate_gyro(uint16_t samples = 2000u);
        void calibrate_accel(uint16_t samples = 2000u);

        /*
         * Set IMU mounting rotation relative to body frame.
         * Unit: radians.
         *
         * Example:
         * set_body_rotation(0.0f, 0.0f, 0.0f);        // sensor frame == body frame
         * set_body_rotation(0.0f, 0.0f, PI / 2.0f);   // like Flix default yaw +90 deg
         */
        void set_body_rotation(float roll_rad, float pitch_rad, float yaw_rad);

        void reset_filter();

        void set_gyro_lpf(float alpha);
        void set_accel_lpf(float alpha);

        const raw_data_t & get_raw() const;
        const scaled_data_t & get_scaled() const;
        const scaled_data_t & get_filtered() const;
        const timing_t & get_timing() const;

    private:
        SPIBus * p_spi_bus = nullptr;
        uint8_t chip_select_pin = 0u;

        raw_data_t raw;
        scaled_data_t scaled;
        scaled_data_t filtered;

        timing_t timing;

        float gyro_bias_x = 0.0f;
        float gyro_bias_y = 0.0f;
        float gyro_bias_z = 0.0f;

        float accel_bias_x = 0.0f;
        float accel_bias_y = 0.0f;
        float accel_bias_z = 0.0f;

        float accel_scale_x = 1.0f;
        float accel_scale_y = 1.0f;
        float accel_scale_z = 1.0f;

        float gyro_alpha = 0.8f;
        float accel_alpha = 0.8f;

        bool filter_initialized = false;

        float imu_rotation_roll = 0.0f;
        float imu_rotation_pitch = 0.0f;
        float imu_rotation_yaw = 0.0f;

        bool read_sensor();
        void scale_sensor();
        void apply_bias_and_scale();
        void rotate_to_body_frame();
        void rotate_vector_to_body(float & x, float & y, float & z) const;
        void low_pass_filter();

        bool validate_whoami();

        bool data_ready();
        bool wait_for_data(uint32_t timeout_us = 3000u);

        static float accel_raw_to_g(int16_t raw_value);
        static float gyro_raw_to_deg_s(int16_t raw_value);
        static float clamp_alpha(float alpha);
        static float lpf_step(float current, float target, float alpha);

        static constexpr float accel_divisor = 8192.0f;   // ±4g: 8192 LSB/g
        static constexpr float gyro_divisor = 16.4f;      // ±2000 dps: 16.4 LSB/(deg/s)
        static constexpr float one_g = 1.0f;

        static constexpr uint8_t who_am_i_reg = 0x75;
        static constexpr uint8_t pwr_mgmt_1_reg = 0x6B;
        static constexpr uint8_t config_reg = 0x1A;
        static constexpr uint8_t gyro_config_reg = 0x1B;
        static constexpr uint8_t accel_config_reg = 0x1C;
        static constexpr uint8_t accel_config_2_reg = 0x1D;
        static constexpr uint8_t smplrt_div_reg = 0x19;
        static constexpr uint8_t accel_xout_h_reg = 0x3B;

        static constexpr uint8_t int_enable_reg = 0x38;
        static constexpr uint8_t int_status_reg = 0x3A;
        static constexpr uint8_t data_ready_mask = 0x01;
};

#endif /* MPU6500_H */