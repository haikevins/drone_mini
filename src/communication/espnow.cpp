#include "communication/espnow.h"

ESPNow * ESPNow::active_instance = nullptr;
constexpr float ESPNow::rssi_filter_alpha;

ESPNow::ESPNow() :
    command_data{false, false},
    last_rssi_dbm(-127),
    filtered_rssi_dbm(-127.0f),
    last_receive_time_ms(0u),
    receive_count(0u),
    bad_packet_count(0u),
    last_send_success(false),
    rssi_filter_initialized(false)
{}

bool ESPNow::begin()
{
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK)
    {
        return false;
    }

    active_instance = this;
    register_send_callback(ESPNow::on_data_sent);
    register_recv_callback(ESPNow::on_data_recv);
    
    return true;
}

bool ESPNow::send(const uint8_t* data, size_t len)
{
    if (esp_now_send(controller_address, data, len) == ESP_OK)
    {
        return true;
    }

    return false;
}

bool ESPNow::send_attitude(const attitude_data_packet_t & attitude_data)
{
    return send(reinterpret_cast<const uint8_t*>(&attitude_data), sizeof(attitude_data));
}

bool ESPNow::is_armed() const
{
    return command_data.arm;
}

bool ESPNow :: is_reset() const
{
    return command_data.reset;
}

bool ESPNow :: is_throttle_up() const
{
    return command_data.throttle_up;
}

bool ESPNow :: is_throttle_down() const
{
    return command_data.throttle_down;
}

bool ESPNow :: is_direction_forward() const
{
    return command_data.direction_forward;
}

bool ESPNow :: is_direction_backward() const
{
    return command_data.direction_backward;
}

bool ESPNow :: is_direction_left() const
{
    return command_data.direction_left;
}

bool ESPNow :: is_direction_right() const
{
    return command_data.direction_right;
}

void ESPNow :: reset_command()
{
    command_data.arm = false;
    command_data.reset = false;

    command_data.throttle_up = false;
    command_data.throttle_down = false;

    command_data.direction_forward = false;
    command_data.direction_backward = false;
    command_data.direction_left = false;
    command_data.direction_right = false;
}

const command_data_packet_t & ESPNow::get_command_data() const
{
    return command_data;
}

bool ESPNow::has_received_packet() const
{
    return receive_count > 0u;
}

int ESPNow::get_last_rssi_dbm() const
{
    return last_rssi_dbm;
}

float ESPNow::get_filtered_rssi_dbm() const
{
    return filtered_rssi_dbm;
}

uint32_t ESPNow::get_last_receive_time_ms() const
{
    return last_receive_time_ms;
}

uint32_t ESPNow::get_last_packet_age_ms() const
{
    if (last_receive_time_ms == 0u)
    {
        return UINT32_MAX;
    }

    return millis() - last_receive_time_ms;
}

uint32_t ESPNow::get_receive_count() const
{
    return receive_count;
}

uint32_t ESPNow::get_bad_packet_count() const
{
    return bad_packet_count;
}

bool ESPNow::is_link_recent(uint32_t timeout_ms) const
{
    return get_last_packet_age_ms() <= timeout_ms;
}

bool ESPNow::was_last_send_success() const
{
    return last_send_success;
}

void ESPNow::register_recv_callback(esp_now_recv_cb_t callback)
{
    esp_now_register_recv_cb(callback);
}

void ESPNow::register_send_callback(esp_now_send_cb_t callback)
{
    esp_now_register_send_cb(callback);
}

void ESPNow::on_data_sent(const wifi_tx_info_t *info, esp_now_send_status_t status)
{
    (void)info;

    if (active_instance != nullptr)
    {
        active_instance->last_send_success = (status == ESP_NOW_SEND_SUCCESS);
    }
}

void ESPNow::on_data_recv(const esp_now_recv_info_t *info, const uint8_t *data, int data_len)
{
    if (active_instance == nullptr)
    {
        return;
    }

    ESPNow * self = active_instance;

    if ((info != nullptr) && (info->rx_ctrl != nullptr))
    {
        const int rssi = info->rx_ctrl->rssi;

        self->last_rssi_dbm = rssi;

        if (self->rssi_filter_initialized == false)
        {
            self->filtered_rssi_dbm = static_cast<float>(rssi);
            self->rssi_filter_initialized = true;
        }
        else
        {
            self->filtered_rssi_dbm += rssi_filter_alpha * (static_cast<float>(rssi) - self->filtered_rssi_dbm);
        }
    }

    self->last_receive_time_ms = millis();
    self->receive_count++;

    if ((data == nullptr) || (data_len < static_cast<int>(sizeof(self->command_data))))
    {
        self->bad_packet_count++;
        return;
    }

    memcpy(&self->command_data, data, sizeof(self->command_data));

}

bool ESPNow::register_peer()
{
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, controller_address, 6);

    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    const bool success = add_peer(&peerInfo);

    return success;
    
}

bool ESPNow::add_peer(const esp_now_peer_info_t* peerInfo)
{
    if (esp_now_add_peer(peerInfo) != ESP_OK)
    {
        return false;
    }

    return true;
}
