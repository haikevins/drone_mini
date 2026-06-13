/*
 * main.cpp
 * MPU6500 + Madgwick attitude test.
 */

#include <Arduino.h>
#include <math.h>

#include "common/parameters.h"

#include "drivers/mpu6500.h"
#include "estimator/madgwick.h"

static SPIBus g_spi_bus;
static MPU6500 g_imu;
static Madgwick g_madgwick;

float ax = 0.0f;
float ay = 0.0f;
float az = 0.0f;

float gx = 0.0f;
float gy = 0.0f;
float gz = 0.0f;

float roll = 0.0f;
float pitch = 0.0f;
float yaw = 0.0f;

uint16_t serial_period_us = 200u;
uint32_t last_serial_us = 0;

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
    g_imu.calibrate_accel();

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

    g_madgwick.begin(1000.0f); // sample period in Hz
}

void loop()
{
    if (g_imu.update() == true)
    {
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

        ax = imu_data.ax;
        ay = imu_data.ay;
        az = imu_data.az;

        gx = imu_data.gx;
        gy = imu_data.gy;
        gz = imu_data.gz;

        roll = g_madgwick.getRoll();
        pitch = g_madgwick.getPitch();
        yaw = g_madgwick.getYaw();
    }

    if (millis() - last_serial_us >= serial_period_us)
    {
        last_serial_us += serial_period_us;

        Serial.print("ax=");
        Serial.print(ax, 3);
        Serial.print(", ay=");
        Serial.print(ay, 3);
        Serial.print(", az=");
        Serial.print(az, 3);

        Serial.print(", gx=");
        Serial.print(gx, 3);
        Serial.print(", gy=");
        Serial.print(gy, 3);
        Serial.print(", gz=");
        Serial.print(gz, 3);

        Serial.print(", roll=");
        Serial.print(roll, 2);
        Serial.print(", pitch=");
        Serial.print(pitch, 2);
        Serial.print(", yaw=");
        Serial.println(yaw, 2);
    }
}