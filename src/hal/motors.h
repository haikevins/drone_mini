/*
 * motor.h
 * Motor PWM control interface.
 */

#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor
{
    public:
        void setup_motors ();
        void write_motors (float motor_1_us, float motor_2_us, float motor_3_us, float motor_4_us);

    private:
        static uint32_t throttle_to_pwm (float throttle);

        static constexpr uint8_t motor_1_pin = 1u;  // Front-left motor.
        static constexpr uint8_t motor_2_pin = 0u;  // Front-right motor.
        static constexpr uint8_t motor_3_pin = 10u; // Back-right motor.
        static constexpr uint8_t motor_4_pin = 3u;  // Back-left motor.

        static constexpr uint32_t motor_freq_hz = 78000u; // 78 kHz.
        static constexpr uint8_t motor_resolution_bits = 10u;

        static constexpr float motor_pwm_max = 1023.0f;

        static constexpr float motor_speed_min_us = 1000.0f;
        static constexpr float motor_speed_max_us = 2000.0f;

        static constexpr uint8_t motor_channel_1 = 0u;
        static constexpr uint8_t motor_channel_2 = 1u;
        static constexpr uint8_t motor_channel_3 = 2u;
        static constexpr uint8_t motor_channel_4 = 3u;
};

#endif /* MOTOR_H */
