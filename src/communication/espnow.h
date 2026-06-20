#ifndef ESPNOW_H
#define ESPNOW_H

#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

#include "communication/espnow_protocol.h"

class ESPNow
{
    public:
        ESPNow();
        
        bool begin();
        bool send(const uint8_t* data, size_t len);
        bool send_attitude(const attitude_data_packet_t & attitude_data);

        bool is_armed() const;
        bool is_reset() const;
        bool is_throttle_up() const;
        bool is_throttle_down() const;
        bool is_direction_forward() const;
        bool is_direction_backward() const;
        bool is_direction_left() const;
        bool is_direction_right() const;
        void reset_command();
        const command_data_packet_t & get_command_data() const;

        // RSSI cua goi dieu khien controller -> drone, do tai drone.
        // Gia tri nay duoc cap nhat trong ESP-NOW receive callback.
        bool has_received_packet() const;
        int get_last_rssi_dbm() const;
        float get_filtered_rssi_dbm() const;
        uint32_t get_last_receive_time_ms() const;
        uint32_t get_last_packet_age_ms() const;
        uint32_t get_receive_count() const;
        uint32_t get_bad_packet_count() const;
        bool is_link_recent(uint32_t timeout_ms) const;
        bool was_last_send_success() const;        

        void register_recv_callback(esp_now_recv_cb_t callback);
        void register_send_callback(esp_now_send_cb_t callback);

        static void on_data_sent(const wifi_tx_info_t *info, esp_now_send_status_t status);
        static void on_data_recv(const esp_now_recv_info_t *info, const uint8_t *data, int data_len);

        bool register_peer();

    private:
        // replace with receiver's mac address
        uint8_t controller_address[6] = {0xAC, 0xA7, 0x04, 0xBB, 0x6D, 0x64};

        command_data_packet_t command_data;

        int last_rssi_dbm;
        float filtered_rssi_dbm;
        uint32_t last_receive_time_ms;
        uint32_t receive_count;
        uint32_t bad_packet_count;
        bool last_send_success;
        bool rssi_filter_initialized;

        static constexpr float rssi_filter_alpha = 0.20f;        

        static ESPNow * active_instance;

        bool add_peer(const esp_now_peer_info_t* peerInfo);
};

#endif /* ESPNOW_H */
