#pragma once
#include "esphome.h"
#include <ESP8266WiFi.h>

#define PIN_PANEL_IN 5   // D15
#define PIN_HOOD_OUT 4   // D14

#define BIT_0_LOW  250
#define BIT_0_HIGH 750
#define BIT_1_LOW  750
#define BIT_1_HIGH 250
#define MID_PAUSE  4850
#define PACKET_INTERVAL 36900
#define PACKET_BITS 58

class WeissgauffHoodController : public Component, public PollingComponent {
private:
    // Указатели на сущности ESPHome (устанавливаются из YAML)
    select::Select* speed_select_ = nullptr;
    switch_::Switch* light_switch_ = nullptr;
    text_sensor::TextSensor* status_sensor_ = nullptr;
    text_sensor::TextSensor* last_packet_sensor_ = nullptr;
    sensor::Sensor* packet_count_sensor_ = nullptr;
    sensor::Sensor* wifi_signal_sensor_ = nullptr;

    enum HoodState { STATE_OFF, STATE_SPEED1, STATE_SPEED2, STATE_SPEED3, STATE_LIGHT, STATE_LIGHT_SPD1, STATE_LIGHT_SPD2, STATE_LIGHT_SPD3 };
    HoodState current_state_ = STATE_OFF;
    HoodState target_state_ = STATE_OFF;
    
    uint32_t packet_count_ = 0;
    uint32_t last_receive_time_ = 0;
    String last_packet_raw_ = "";
    
    bool ha_command_pending_ = false;
    HoodState ha_command_state_ = STATE_OFF;

    const char* patterns_[8] = {
        "1111111111111111111111111111111001111111111111111010101010",
        "1011111011111111111111111011110101101111111111111011101010",
        "1011110111111111111111111011110001110111111111111010001010",
        "1011110011111111111111111011101101111011111111111010111010",
        "0111111111111111111111110111111001111110011111111010101100",
        "0011111011111111111111110011110101101110011111111011101100",
        "0011110111111111111111110011110001110110011111111010001100",
        "0011110011111111111111110011101101111010011111111010111100"
    };

    void IRAM_ATTR send_bit(bool bit) {
        if (bit) {
            digitalWrite(PIN_HOOD_OUT, LOW);
            delayMicroseconds(BIT_1_LOW);
            digitalWrite(PIN_HOOD_OUT, HIGH);
            delayMicroseconds(BIT_1_HIGH);
        } else {
            digitalWrite(PIN_HOOD_OUT, LOW);
            delayMicroseconds(BIT_0_LOW);
            digitalWrite(PIN_HOOD_OUT, HIGH);
            delayMicroseconds(BIT_0_HIGH);
        }
    }

    void send_packet(const char* pattern) {
        noInterrupts();
        digitalWrite(PIN_HOOD_OUT, HIGH);
        delayMicroseconds(50);

        for (int i = 0; i < PACKET_BITS; i++) {
            bool bit = (pattern[i] == '1');
            send_bit(bit);
            if (i == 32) {
                digitalWrite(PIN_HOOD_OUT, LOW);
                delayMicroseconds(MID_PAUSE);
                digitalWrite(PIN_HOOD_OUT, HIGH);
            }
        }
        interrupts();
        packet_count_++;
    }

    void update_ui_state(HoodState new_state) {
        current_state_ = new_state;
        
        // Обновляем Select (скорость)
        if (speed_select_ != nullptr) {
            int idx = 0;
            if (new_state == STATE_SPEED1 || new_state == STATE_LIGHT_SPD1) idx = 1;
            else if (new_state == STATE_SPEED2 || new_state == STATE_LIGHT_SPD2) idx = 2;
            else if (new_state == STATE_SPEED3 || new_state == STATE_LIGHT_SPD3) idx = 3;
            speed_select_->set_index(idx);
        }
        
        // Обновляем Switch (свет)
        if (light_switch_ != nullptr) {
            bool light_on = (new_state == STATE_LIGHT || new_state >= STATE_LIGHT_SPD1);
            light_switch_->publish_state(light_on);
        }
        
        // Обновляем статус
        if (status_sensor_ != nullptr) {
            const char* names[] = {"OFF", "SPEED1", "SPEED2", "SPEED3", "LIGHT", "LIGHT+SPD1", "LIGHT+SPD2", "LIGHT+SPD3"};
            status_sensor_->publish_state(names[new_state]);
        }
    }

public:
    WeissgauffHoodController() : PollingComponent(1000) {}

    // Метод для установки указателей из YAML
    void set_entities(
        select::Select* speed,
        switch_::Switch* light,
        text_sensor::TextSensor* status,
        text_sensor::TextSensor* last_pkt,
        sensor::Sensor* pkt_count,
        sensor::Sensor* wifi_sig
    ) {
        speed_select_ = speed;
        light_switch_ = light;
        status_sensor_ = status;
        last_packet_sensor_ = last_pkt;
        packet_count_sensor_ = pkt_count;
        wifi_signal_sensor_ = wifi_sig;
    }

    // Обработчик команды скорости из HA
    void on_speed_select_call(select::SelectCall call) {
        if (call.get_index().has_value()) {
            int idx = call.get_index().value();
            if (idx == 0) ha_command_state_ = STATE_OFF;
            else if (idx == 1) ha_command_state_ = (current_state_ >= STATE_LIGHT_SPD1) ? STATE_LIGHT_SPD1 : STATE_SPEED1;
            else if (idx == 2) ha_command_state_ = (current_state_ >= STATE_LIGHT_SPD2) ? STATE_LIGHT_SPD2 : STATE_SPEED2;
            else if (idx == 3) ha_command_state_ = (current_state_ >= STATE_LIGHT_SPD3) ? STATE_LIGHT_SPD3 : STATE_SPEED3;
            ha_command_pending_ = true;
        }
    }

    // Обработчик команды света из HA
    void on_light_switch_call(switch_::SwitchCall call) {
        if (call.get_state().has_value()) {
            bool state = call.get_state().value();
            if (state) {
                if (current_state_ == STATE_OFF) ha_command_state_ = STATE_LIGHT;
                else if (current_state_ == STATE_SPEED1) ha_command_state_ = STATE_LIGHT_SPD1;
                else if (current_state_ == STATE_SPEED2) ha_command_state_ = STATE_LIGHT_SPD2;
                else if (current_state_ == STATE_SPEED3) ha_command_state_ = STATE_LIGHT_SPD3;
            } else {
                if (current_state_ == STATE_LIGHT) ha_command_state_ = STATE_OFF;
                else if (current_state_ == STATE_LIGHT_SPD1) ha_command_state_ = STATE_SPEED1;
                else if (current_state_ == STATE_LIGHT_SPD2) ha_command_state_ = STATE_SPEED2;
                else if (current_state_ == STATE_LIGHT_SPD3) ha_command_state_ = STATE_SPEED3;
            }
            ha_command_pending_ = true;
        }
    }

    void setup() override {
        pinMode(PIN_HOOD_OUT, OUTPUT);
        pinMode(PIN_PANEL_IN, INPUT_PULLUP);
        digitalWrite(PIN_HOOD_OUT, HIGH);
        update_ui_state(STATE_OFF);
    }

    void loop() override {
        uint32_t now = micros();

        // 1. Обработка команды из HA
        if (ha_command_pending_) {
            send_packet(patterns_[ha_command_state_]);
            update_ui_state(ha_command_state_);
            target_state_ = ha_command_state_;
            ha_command_pending_ = false;
        }

        // 2. Непрерывная передача (сердцебиение)
        static uint32_t last_tx = 0;
        if (now - last_tx >= PACKET_INTERVAL) {
            if (!ha_command_pending_) {
                send_packet(patterns_[target_state_]);
            }
            last_tx = now;

            // Публикация сенсоров
            if (packet_count_sensor_) packet_count_sensor_->publish_state(packet_count_);
            if (wifi_signal_sensor_) wifi_signal_sensor_->publish_state(WiFi.RSSI());
        }

        // 3. Приём с панели (упрощённый пасстхру)
        // Здесь можно добавить декодер манчестера для полноценного перехвата
        // Пока просто ретранслируем, если нужно - расширьте логику
        
        yield();
    }

    void poll() override {
        // Обновление статуса при необходимости
    }
};
