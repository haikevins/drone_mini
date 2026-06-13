/*
 * motor.cpp
 * Motor PWM control implementation.
 */

#include "hal/motors.h"

uint32_t Motor::throttle_to_pwm (float throttle)
{
    const float clamped = constrain(throttle, motor_speed_min_us, motor_speed_max_us);
    const float pwm = (clamped - motor_speed_min_us) * motor_pwm_max / (motor_speed_max_us - motor_speed_min_us);

    return static_cast<uint32_t>(pwm);
}

void Motor::write_motors (float motor_1_throttle, float motor_2_throttle, float motor_3_throttle, float motor_4_throttle)
{
    ledcWrite(motor_channel_1, throttle_to_pwm(motor_1_throttle));
    ledcWrite(motor_channel_2, throttle_to_pwm(motor_2_throttle));
    ledcWrite(motor_channel_3, throttle_to_pwm(motor_3_throttle));
    ledcWrite(motor_channel_4, throttle_to_pwm(motor_4_throttle));
}

void Motor::setup_motors ()
{
    ledcSetup(motor_channel_1, motor_freq_hz, motor_resolution_bits);
    ledcSetup(motor_channel_2, motor_freq_hz, motor_resolution_bits);
    ledcSetup(motor_channel_3, motor_freq_hz, motor_resolution_bits);
    ledcSetup(motor_channel_4, motor_freq_hz, motor_resolution_bits);

    ledcAttachPin(motor_1_pin, motor_channel_1);
    ledcAttachPin(motor_2_pin, motor_channel_2);
    ledcAttachPin(motor_3_pin, motor_channel_3);
    ledcAttachPin(motor_4_pin, motor_channel_4);

    write_motors(motor_speed_min_us, motor_speed_min_us, motor_speed_min_us, motor_speed_min_us);
}
