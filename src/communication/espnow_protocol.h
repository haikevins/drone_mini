#ifndef ESPNOW_PROTOCOL_H
#define ESPNOW_PROTOCOL_H

#include <Arduino.h>

typedef struct
{
    float roll;
    float pitch;
    float yaw;
}
attitude_data_packet_t;

typedef struct
{
    bool arm;
    bool reset;

    bool throttle_up;
    bool throttle_down;
}
command_data_packet_t;

#endif /* ESPNOW_PROTOCOL_H */