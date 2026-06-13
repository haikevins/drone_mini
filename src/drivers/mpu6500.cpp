/*
 * mpu6500.cpp
 * MPU6500 IMU driver implementation.
 */

#include "drivers/mpu6500.h"
#include <math.h>

float MPU6500::accel_raw_to_g(int16_t raw_value)
{
    return static_cast<float>(raw_value) / accel_divisor;
}

float MPU6500::gyro_raw_to_deg_s(int16_t raw_value)
{
    return static_cast<float>(raw_value) / gyro_divisor;
}

float MPU6500::clamp_alpha(float alpha)
{
    if (alpha < 0.0f)
    {
        return 0.0f;
    }

    if (alpha > 1.0f)
    {
        return 1.0f;
    }

    return alpha;
}

float MPU6500::lpf_step(float current, float target, float alpha)
{
    return current + (alpha * (target - current));
}

bool MPU6500::begin(SPIBus * p_spi_bus_ref, uint8_t chip_select_pin_ref, int sck_pin, int miso_pin, int mosi_pin)
{
    if (p_spi_bus_ref == nullptr)
    {
        return false;
    }

    p_spi_bus = p_spi_bus_ref;
    chip_select_pin = chip_select_pin_ref;

    pinMode(chip_select_pin, OUTPUT);
    digitalWrite(chip_select_pin, HIGH);

    p_spi_bus->begin(sck_pin, miso_pin, mosi_pin);

    delay(100);

    if (validate_whoami() == false)
    {
        return false;
    }

    /*
     * Wake up MPU6500.
     */
    p_spi_bus->write_register(chip_select_pin, pwr_mgmt_1_reg, 0x00u);
    delay(100);

    /*
     * CONFIG      = 0x01: DLPF config
     * ACCEL_CFG2  = 0x01: accel DLPF config
     * SMPLRT_DIV  = 0x00: maximum sample rate from internal sampling
     * GYRO_CONFIG = 0x18: ±2000 dps
     * ACCEL_CONFIG= 0x08: ±4g
     */
    p_spi_bus->write_register(chip_select_pin, config_reg, 0x01u);
    p_spi_bus->write_register(chip_select_pin, accel_config_2_reg, 0x01u);
    p_spi_bus->write_register(chip_select_pin, smplrt_div_reg, 0x00u);

    p_spi_bus->write_register(chip_select_pin, gyro_config_reg, 0x18u);
    p_spi_bus->write_register(chip_select_pin, accel_config_reg, 0x08u);

    /*
     * Enable data-ready status bit.
     * You do NOT need to connect the physical INT pin for polling INT_STATUS.
     */
    p_spi_bus->write_register(chip_select_pin, int_enable_reg, 0x01u);

    /*
     * Clear any stale interrupt status.
     */
    (void)p_spi_bus->read_register(chip_select_pin, int_status_reg);

    timing.last_us = micros();
    timing.now_us = timing.last_us;
    timing.dt = 0.0f;
    timing.imu_dt = 0.0f;

    filter_initialized = false;

    return true;
}

bool MPU6500::validate_whoami()
{
    if (p_spi_bus == nullptr)
    {
        return false;
    }

    const uint8_t whoami = p_spi_bus->read_register(chip_select_pin, who_am_i_reg);

    return ((whoami == 0x70u) || (whoami == 0x71u));
}

bool MPU6500::data_ready()
{
    if (p_spi_bus == nullptr)
    {
        return false;
    }

    const uint8_t status = p_spi_bus->read_register(chip_select_pin, int_status_reg);
    return ((status & data_ready_mask) != 0u);
}

bool MPU6500::wait_for_data(uint32_t timeout_us)
{
    const uint32_t start_us = micros();

    while (data_ready() == false)
    {
        if ((micros() - start_us) >= timeout_us)
        {
            return false;
        }
    }

    return true;
}

bool MPU6500::update()
{
    /*
     * Similar to Flix's waitForData():
     * only read when a new IMU sample is available.
     */
    if (wait_for_data() == false)
    {
        return false;
    }

    timing.now_us = micros();
    timing.dt = static_cast<float>(timing.now_us - timing.last_us) * 1e-6f;
    timing.imu_dt = timing.dt;
    timing.last_us = timing.now_us;

    if (read_sensor() == false)
    {
        return false;
    }

    scale_sensor();
    apply_bias_and_scale();
    rotate_to_body_frame();

    /*
     * Avoid starting LPF from zero.
     * First filtered value should equal first corrected sample.
     */
    if (filter_initialized == false)
    {
        filtered = scaled;
        filter_initialized = true;
    }
    else
    {
        low_pass_filter();
    }

    return true;
}

bool MPU6500::read_sensor()
{
    if (p_spi_bus == nullptr)
    {
        return false;
    }

    uint8_t buffer[14];

    p_spi_bus->read_registers(chip_select_pin, accel_xout_h_reg, buffer, 14);

    raw.ax = static_cast<int16_t>((static_cast<uint16_t>(buffer[0]) << 8) | buffer[1]);
    raw.ay = static_cast<int16_t>((static_cast<uint16_t>(buffer[2]) << 8) | buffer[3]);
    raw.az = static_cast<int16_t>((static_cast<uint16_t>(buffer[4]) << 8) | buffer[5]);

    /*
     * buffer[6] and buffer[7] are temperature. Ignored here.
     */
    raw.gx = static_cast<int16_t>((static_cast<uint16_t>(buffer[8]) << 8) | buffer[9]);
    raw.gy = static_cast<int16_t>((static_cast<uint16_t>(buffer[10]) << 8) | buffer[11]);
    raw.gz = static_cast<int16_t>((static_cast<uint16_t>(buffer[12]) << 8) | buffer[13]);

    return true;
}

void MPU6500::scale_sensor()
{
    /*
     * Madgwick version you use expects:
     * gyro  = deg/s
     * accel = any consistent unit, here g
     */
    scaled.ax = accel_raw_to_g(raw.ax);
    scaled.ay = accel_raw_to_g(raw.ay);
    scaled.az = accel_raw_to_g(raw.az);

    scaled.gx = gyro_raw_to_deg_s(raw.gx);
    scaled.gy = gyro_raw_to_deg_s(raw.gy);
    scaled.gz = gyro_raw_to_deg_s(raw.gz);
}

void MPU6500::apply_bias_and_scale()
{
    /*
     * Gyro:
     * deg/s - deg/s bias
     */
    scaled.gx -= gyro_bias_x;
    scaled.gy -= gyro_bias_y;
    scaled.gz -= gyro_bias_z;

    /*
     * Accel:
     * Flix-style correction: (acc - bias) / scale.
     * For now scale defaults to 1.0 unless you add 6-side calibration later.
     */
    scaled.ax = (scaled.ax - accel_bias_x) / accel_scale_x;
    scaled.ay = (scaled.ay - accel_bias_y) / accel_scale_y;
    scaled.az = (scaled.az - accel_bias_z) / accel_scale_z;
}

void MPU6500::set_body_rotation(float roll_rad, float pitch_rad, float yaw_rad)
{
    imu_rotation_roll = roll_rad;
    imu_rotation_pitch = pitch_rad;
    imu_rotation_yaw = yaw_rad;

    filter_initialized = false;
}

void MPU6500::rotate_vector_to_body(float & x, float & y, float & z) const
{
    const float cr = cosf(imu_rotation_roll);
    const float sr = sinf(imu_rotation_roll);

    const float cp = cosf(imu_rotation_pitch);
    const float sp = sinf(imu_rotation_pitch);

    const float cy = cosf(imu_rotation_yaw);
    const float sy = sinf(imu_rotation_yaw);

    /*
     * R = Rz(yaw) * Ry(pitch) * Rx(roll)
     *
     * The stored rotation describes the IMU mounting rotation.
     * To transform sensor-frame vector into body-frame vector,
     * use inverse(R). Since R is orthonormal, inverse(R) = transpose(R).
     */
    const float r00 = cy * cp;
    const float r01 = (cy * sp * sr) - (sy * cr);
    const float r02 = (cy * sp * cr) + (sy * sr);

    const float r10 = sy * cp;
    const float r11 = (sy * sp * sr) + (cy * cr);
    const float r12 = (sy * sp * cr) - (cy * sr);

    const float r20 = -sp;
    const float r21 = cp * sr;
    const float r22 = cp * cr;

    const float old_x = x;
    const float old_y = y;
    const float old_z = z;

    /*
     * body = transpose(R) * sensor
     */
    x = (r00 * old_x) + (r10 * old_y) + (r20 * old_z);
    y = (r01 * old_x) + (r11 * old_y) + (r21 * old_z);
    z = (r02 * old_x) + (r12 * old_y) + (r22 * old_z);
}

void MPU6500::rotate_to_body_frame()
{
    rotate_vector_to_body(scaled.ax, scaled.ay, scaled.az);
    rotate_vector_to_body(scaled.gx, scaled.gy, scaled.gz);
}

void MPU6500::low_pass_filter()
{
    filtered.gx = lpf_step(filtered.gx, scaled.gx, gyro_alpha);
    filtered.gy = lpf_step(filtered.gy, scaled.gy, gyro_alpha);
    filtered.gz = lpf_step(filtered.gz, scaled.gz, gyro_alpha);

    filtered.ax = lpf_step(filtered.ax, scaled.ax, accel_alpha);
    filtered.ay = lpf_step(filtered.ay, scaled.ay, accel_alpha);
    filtered.az = lpf_step(filtered.az, scaled.az, accel_alpha);
}

void MPU6500::reset_filter()
{
    filtered = scaled;
    filter_initialized = true;
}

void MPU6500::calibrate_gyro(uint16_t samples)
{
    if (samples == 0u)
    {
        return;
    }

    int64_t sum_raw_x = 0;
    int64_t sum_raw_y = 0;
    int64_t sum_raw_z = 0;

    for (uint16_t sample_index = 0u; sample_index < samples; ++sample_index)
    {
        read_sensor();

        sum_raw_x += raw.gx;
        sum_raw_y += raw.gy;
        sum_raw_z += raw.gz;

        delay(2);
    }

    gyro_bias_x = gyro_raw_to_deg_s(static_cast<int16_t>(sum_raw_x / samples));
    gyro_bias_y = gyro_raw_to_deg_s(static_cast<int16_t>(sum_raw_y / samples));
    gyro_bias_z = gyro_raw_to_deg_s(static_cast<int16_t>(sum_raw_z / samples));

    filter_initialized = false;
}

void MPU6500::calibrate_accel(uint16_t samples)
{
    if (samples == 0u)
    {
        return;
    }

    int64_t sum_raw_x = 0;
    int64_t sum_raw_y = 0;
    int64_t sum_raw_z = 0;

    for (uint16_t sample_index = 0u; sample_index < samples; ++sample_index)
    {
        read_sensor();

        sum_raw_x += raw.ax;
        sum_raw_y += raw.ay;
        sum_raw_z += raw.az;

        delay(2);
    }

    const float avg_x = accel_raw_to_g(static_cast<int16_t>(sum_raw_x / samples));
    const float avg_y = accel_raw_to_g(static_cast<int16_t>(sum_raw_y / samples));
    const float avg_z = accel_raw_to_g(static_cast<int16_t>(sum_raw_z / samples));

    /*
     * One-position accel calibration.
     *
     * This assumes the board is level during calibration and sensor Z reads +1g.
     * If your sensor reads -1g when level, change:
     *
     * accel_bias_z = avg_z - one_g;
     *
     * to:
     *
     * accel_bias_z = avg_z + one_g;
     */
    accel_bias_x = avg_x;
    accel_bias_y = avg_y;
    accel_bias_z = avg_z - one_g;

    /*
     * Scale remains 1.0 until you implement 6-side accel calibration.
     */
    accel_scale_x = 1.0f;
    accel_scale_y = 1.0f;
    accel_scale_z = 1.0f;

    filter_initialized = false;
}

void MPU6500::set_gyro_lpf(float alpha)
{
    gyro_alpha = clamp_alpha(alpha);
}

void MPU6500::set_accel_lpf(float alpha)
{
    accel_alpha = clamp_alpha(alpha);
}

const MPU6500::raw_data_t & MPU6500::get_raw() const
{
    return raw;
}

const MPU6500::scaled_data_t & MPU6500::get_scaled() const
{
    return scaled;
}

const MPU6500::scaled_data_t & MPU6500::get_filtered() const
{
    return filtered;
}

const MPU6500::timing_t & MPU6500::get_timing() const
{
    return timing;
}