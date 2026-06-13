/*
 * main.cpp
 * MPU6500 + Madgwick attitude test.
 */

#include <Arduino.h>
#include <math.h>

#include "common/math_utils.h"
#include "common/parameters.h"

#include "drivers/mpu6500.h"
#include "estimator/madgwick_filter.h"

static SPIBus g_spi_bus;
static MPU6500 g_imu;
static Madgwick g_madgwick;

static uint32_t g_last_print_ms = 0u;
static uint32_t g_last_rate_ms = 0u;
static uint32_t g_update_count = 0u;
static uint32_t g_update_fail_count = 0u;

static float g_update_rate_hz = 0.0f;

static float wrap_180(float angle_deg)
{
    while (angle_deg >= 180.0f)
    {
        angle_deg -= 360.0f;
    }

    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }

    return angle_deg;
}

static void print_header()
{
    Serial.println();
    Serial.println(
        "dt,rate_hz,fail_count,"
        "ax,ay,az,gx,gy,gz,acc_norm,gyro_norm,"
        "roll_deg,pitch_deg,yaw_deg"
    );
}

static void print_attitude()
{
    const MPU6500::scaled_data_t & d = g_imu.get_filtered();
    const MPU6500::timing_t & timing = g_imu.get_timing();

    const float acc_norm = sqrtf(
        d.ax * d.ax +
        d.ay * d.ay +
        d.az * d.az
    );

    const float gyro_norm = sqrtf(
        d.gx * d.gx +
        d.gy * d.gy +
        d.gz * d.gz
    );

    /*
     * Madgwick getRoll/getPitch/getYaw return degree.
     * Note: your Madgwick getYaw() adds +180 internally.
     * wrap_180() makes yaw easier to read.
     */
    const float roll_deg = g_madgwick.getRoll();
    const float pitch_deg = g_madgwick.getPitch();
    const float yaw_deg = wrap_180(g_madgwick.getYaw());

    Serial.print(timing.dt, 6);
    Serial.print(",");
    Serial.print(g_update_rate_hz, 1);
    Serial.print(",");
    Serial.print(g_update_fail_count);
    Serial.print(",");

    Serial.print(d.ax, 4);
    Serial.print(",");
    Serial.print(d.ay, 4);
    Serial.print(",");
    Serial.print(d.az, 4);
    Serial.print(",");

    Serial.print(d.gx, 3);
    Serial.print(",");
    Serial.print(d.gy, 3);
    Serial.print(",");
    Serial.print(d.gz, 3);
    Serial.print(",");

    Serial.print(acc_norm, 4);
    Serial.print(",");
    Serial.print(gyro_norm, 4);
    Serial.print(",");

    Serial.print(roll_deg, 2);
    Serial.print(",");
    Serial.print(pitch_deg, 2);
    Serial.print(",");
    Serial.println(yaw_deg, 2);
}

void setup()
{
    Serial.begin(115200);
    delay(2000);

    Serial.println();
    Serial.println("======================================");
    Serial.println("MPU6500 + MADGWICK TEST");
    Serial.println("======================================");
    Serial.println("Keep the board still and level during calibration.");
    delay(1000);

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

    Serial.println("MPU6500 DETECTED");

    /*
     * IMPORTANT:
     * Use the same body rotation that passed your pre-Madgwick test.
     *
     * If your previous 14/14 test used 0,0,0, keep this:
     */
    g_imu.set_body_rotation(0.0f, 0.0f, 0.0f);

    /*
     * If your board is mounted like Flix default yaw +90 deg, use this instead:
     *
     * g_imu.set_body_rotation(0.0f, 0.0f, PI / 2.0f);
     */

    Serial.println("Calibrating gyro...");
    Serial.println("Do not move the board.");
    g_imu.calibrate_gyro();

    Serial.println("Calibrating accel...");
    Serial.println("Keep board level. This assumes sensor Z reads about +1g.");
    g_imu.calibrate_accel();

    g_imu.set_gyro_lpf(g_imu_gyro_lpf_alpha);
    g_imu.set_accel_lpf(g_imu_accel_lpf_alpha);

    /*
     * Your MPU data-ready test shows ~996Hz, so use 1000Hz.
     * Madgwick code uses invSampleFreq = 1.0f / sampleFrequency.
     */
    g_madgwick.begin(1000.0f);

    /*
     * Warm-up Madgwick while board is still.
     * This helps roll/pitch settle before printing.
     */
    Serial.println("Warming up Madgwick...");
    const uint32_t warmup_start_ms = millis();

    while ((millis() - warmup_start_ms) < 2000u)
    {
        if (g_imu.update())
        {
            const MPU6500::scaled_data_t & d = g_imu.get_filtered();

            g_madgwick.updateIMU(
                d.gx, d.gy, d.gz,   // deg/s
                d.ax, d.ay, d.az    // g
            );
        }
    }

    Serial.println();
    Serial.println("Ready.");
    Serial.println();
    Serial.println("Expected:");
    Serial.println("Level still       : roll ~= 0 deg, pitch ~= 0 deg");
    Serial.println("Roll right/left   : roll changes clearly");
    Serial.println("Pitch forward/back: pitch changes clearly");
    Serial.println("Yaw rotation      : yaw changes, but IMU-only yaw may drift over time");
    Serial.println();
    Serial.println("Important sign convention from your pre-test:");
    Serial.println("roll right  -> ay positive");
    Serial.println("roll left   -> ay negative");
    Serial.println("pitch down  -> ax negative");
    Serial.println("pitch up    -> ax positive");
    Serial.println();

    print_header();

    g_last_print_ms = millis();
    g_last_rate_ms = millis();
}

void loop()
{
    const bool ok = g_imu.update();

    if (ok == false)
    {
        g_update_fail_count++;

        static uint32_t last_fail_print_ms = 0u;
        const uint32_t now_ms = millis();

        if ((now_ms - last_fail_print_ms) >= 1000u)
        {
            last_fail_print_ms = now_ms;
            Serial.println("IMU update timeout/fail.");
        }

        return;
    }

    const MPU6500::scaled_data_t & d = g_imu.get_filtered();

    /*
     * Your Madgwick implementation expects:
     * gyro  = deg/s
     * accel = g
     */
    g_madgwick.updateIMU(
        d.gx, d.gy, d.gz,
        d.ax, d.ay, d.az
    );

    g_update_count++;

    const uint32_t now_ms = millis();

    if ((now_ms - g_last_rate_ms) >= 1000u)
    {
        g_update_rate_hz =
            static_cast<float>(g_update_count) * 1000.0f /
            static_cast<float>(now_ms - g_last_rate_ms);

        g_update_count = 0u;
        g_last_rate_ms = now_ms;
    }

    /*
     * Print attitude at 20Hz.
     */
    if ((now_ms - g_last_print_ms) >= 50u)
    {
        g_last_print_ms = now_ms;
        print_attitude();
    }
}