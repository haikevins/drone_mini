/*
 * main.cpp
 * MPU6500 + Madgwick attitude test.
 */

#include <Arduino.h>
#include <math.h>

#include "common/parameters.h"

#include "drivers/mpu6500.h"
#include "estimator/madgwick.h"

#include "control/pid_controller.h"

static SPIBus g_spi_bus;
static MPU6500 g_imu;
static Madgwick g_madgwick;

static PIDController g_pid_roll_angle(1.0f, 0.1f, 0.01f, 10.0f, 100.0f, 0.5f); // kp, ki, kd, integLimit, outputLimit, dFilterAlpha
static PIDController g_pid_pitch_angle(1.0f, 0.1f, 0.01f, 10.0f, 100.0f, 0.5f);
static PIDController g_pid_yaw_angle(1.0f, 0.1f, 0.01f, 10.0f, 100.0f, 0.5f);

static float throttle = 1400.0f;

attitude_data_t attitude_data;

void setup()
{
    Serial.begin(115200);
    delay(10000);

    const bool b_imu_ready = g_imu.begin(
        &g_spi_bus,
        g_pin_ncs,
        g_pin_sck,
        g_pin_miso,
        g_pin_mosi
    );

    if (b_imu_ready == false)
    {
        Serial.println("MPU6500 NOT DETECTED");
        for (;;) {}
    }
    else
    {
        Serial.println("MPU6500 DETECTED");
    }

    g_imu.set_body_rotation(0.0f, 0.0f, 0.0f);

    /*
     * g_imu.set_body_rotation(0.0f, 0.0f, PI / 2.0f);
     */

    g_imu.calibrate_gyro();
    g_imu.calibrate_accel_once();

    // Serial.println("\nStart accel 6-side calibration");

    // bool g_imu_calibrate_accel_ready = g_imu.calibrate_accel_6_side(1000, &Serial, true);

    // if (g_imu_calibrate_accel_ready == true)
    // {
    //     Serial.println("Accel calibration OK");
    //     g_imu.print_accel_calibration(Serial);
    // }
    // else
    // {
    //     Serial.println("Accel calibration FAILED");
    // }

    g_imu.set_gyro_lpf(g_imu_gyro_lpf_alpha);
    g_imu.set_accel_lpf(g_imu_accel_lpf_alpha);

    g_madgwick.begin(g_imu_read_default_hz);

    g_pid_roll_angle.reset();
    g_pid_pitch_angle.reset();
    g_pid_yaw_angle.reset();
}

void loop()
{
    if (g_imu.update() == false)
    {
        return;
    }

    const auto& imu_data = g_imu.get_filtered();
    const auto& imu_dt = g_imu.get_timing().dt;

    g_madgwick.updateIMUdt(
        imu_data.gx,
        imu_data.gy,
        imu_data.gz,
        imu_data.ax,
        imu_data.ay,
        imu_data.az,
        imu_dt
    );

    gx = imu_data.gx;
    gy = imu_data.gy;
    gz = imu_data.gz;

    attitude_data.roll = g_madgwick.getRoll();
    attitude_data.pitch = g_madgwick.getPitch();
    attitude_data.yaw = g_madgwick.getYaw();

    float roll_output = g_pid_roll_angle.update(target_roll_angle, attitude_data.roll, imu_dt, false, true);
    float pitch_output = g_pid_pitch_angle.update(target_pitch_angle, attitude_data.pitch, imu_dt, false, true);
    float yaw_output = g_pid_yaw_angle.update(target_yaw_angle, attitude_data.yaw, imu_dt, true, true);

    float motor1_speed = throttle + roll_output - pitch_output + yaw_output; // front left
    float motor2_speed = throttle - roll_output - pitch_output - yaw_output; // front
    float motor3_speed = throttle - roll_output + pitch_output + yaw_output; // rear
    float motor4_speed = throttle + roll_output + pitch_output - yaw_output; // rear

    if (millis() - last_print_time >= g_serial_print_period_ms)
    {
        last_print_time += g_serial_print_period_ms;

        Serial.print("motor1: ");
        Serial.print(motor1_speed);
        Serial.print(", motor2: ");
        Serial.print(motor2_speed);
        Serial.print(", motor3: ");
        Serial.print(motor3_speed);
        Serial.print(", motor4: ");
        Serial.println(motor4_speed);
    }
}