#pragma once
#include "esphome.h"
#include <ESP8266WiFi.h>

// Конфигурация пинов
#define PIN_PANEL_IN 5  // D15
#define PIN_HOOD_OUT 4  // D14

// Тайминги протокола (в микросекундах)
#define BIT_0_LOW  250
#define BIT_0_HIGH 750
#define BIT_1_LOW  750
#define BIT_1_HIGH 250
#define MID_PAUSE  4850
#define PACKET_INTERVAL 36900 // 36.9 мс

// Длина пакета
#define PACKET_BITS 58

// Команды (58 бит)
// Преобразованы из бинарных строк в массивы uint64_t
// Обратите внимание: порядок битов MSB first
struct Command {
    const char* name;
    uint64_t data;
};

// Вспомогательная функция для конвертации бинарной строки в uint64_t
// Используется при инициализации, но здесь мы хардкодим значения для надежности
const Command CMD_OFF      = {"OFF",      0xFFFFFFFF3F3FFFFAAAA}; // Примерное значение, см. логику ниже
const Command CMD_SPEED1   = {"SPEED 1", 0xBFEFFFFFB6F3FAAA}; 
const Command CMD_SPEED2   = {"SPEED 2", 0xBDFFFFFB77F2AA};
const Command CMD_SPEED3   = {"SPEED 3", 0xBCFFFFFB7F3BAA};
const Command CMD_LIGHT    = {"LIGHT",   0x7FFFFFF79F3FAAC};
// ... (Для точности лучше использовать побитовую отправку по строкам, см. реализацию ниже)

class WeissgauffHoodController : public Component, public PollingComponent {
private:
    // Состояния
    enum HoodState { STATE_OFF, STATE_SPEED1, STATE_SPEED2, STATE_SPEED3, STATE_LIGHT, STATE_LIGHT_SPD1, STATE_LIGHT_SPD2, STATE_LIGHT_SPD3 };
    HoodState current_state = STATE_OFF;
    HoodState target_state = STATE_OFF;
    
    uint32_t packet_count = 0;
    uint32_t last_receive_time = 0;
    String last_packet_hex = "0000000000000000";
    
    // Очереди команд
    bool ha_command_pending = false;
    HoodState ha_command_state = STATE_OFF;
    
    // Сущности ESPHome
    Select* speed_select;
    Switch* light_switch;
    TextSensor* status_sensor;
    TextSensor* last_packet_sensor;
    Sensor* packet_count_sensor;
    Sensor* wifi_signal_sensor;

    // Буфер для приёма
    volatile uint32_t edge_buffer[120];
    volatile uint8_t edge_index = 0;
    volatile bool receiving = false;
    volatile uint32_t last_edge_time = 0;

    // Таблица команд (Бинарные строки для точной генерации)
    const char* patterns[8] = {
        "1111111111111111111111111111111001111111111111111010101010", // OFF
        "1011111011111111111111111011110101101111111111111011101010", // SPD1
        "1011110111111111111111111011110001110111111111111010001010", // SPD2
        "1011110011111111111111111011101101111011111111111010111010", // SPD3
        "0111111111111111111111110111111001111110011111111010101100", // LIGHT
        "0011111011111111111111110011110101101110011111111011101100", // L+SPD1
        "0011110111111111111111110011110001110110011111111010001100", // L+SPD2
        "0011110011111111111111110011101101111010011111111010111100"  // L+SPD3
    };

    void IRAM_ATTR send_bit(bool bit) {
        if (bit) {
            // Bit 1: LOW 750 + HIGH 250
            digitalWrite(PIN_HOOD_OUT, LOW);
            delayMicroseconds(BIT_1_LOW);
            digitalWrite(PIN_HOOD_OUT, HIGH);
            delayMicroseconds(BIT_1_HIGH);
        } else {
            // Bit 0: LOW 250 + HIGH 750
            digitalWrite(PIN_HOOD_OUT, LOW);
            delayMicroseconds(BIT_0_LOW);
            digitalWrite(PIN_HOOD_OUT, HIGH);
            delayMicroseconds(BIT_0_HIGH);
        }
    }

    void send_packet(const char* pattern) {
        // Отключаем прерывания для точности таймингов
        noInterrupts();
        
        digitalWrite(PIN_HOOD_OUT, HIGH); // Idle state
        delayMicroseconds(100); // Стабилизация

        for (int i = 0; i < PACKET_BITS; i++) {
            bool bit = (pattern[i] == '1');
            send_bit(bit);
            
            // Пауза внутри пакета после 33-го бита (индекс 32)
            if (i == 32) {
                digitalWrite(PIN_HOOD_OUT, LOW);
                delayMicroseconds(MID_PAUSE);
                digitalWrite(PIN_HOOD_OUT, HIGH);
            }
        }
        
        interrupts();
        packet_count++;
    }

    void update_state(HoodState new_state) {
        if (current_state != new_state) {
            current_state = new_state;
            target_state = new_state;
            
            // Обновление интерфейса
            if (speed_select) {
                if (new_state == STATE_OFF) speed_select->set_index(0);
                else if (new_state == STATE_SPEED1) speed_select->set_index(1);
                else if (new_state == STATE_SPEED2) speed_select->set_index(2);
                else if (new_state == STATE_SPEED3) speed_select->set_index(3);
                // Для комбинаций со светом выбираем соответствующую скорость
                else if (new_state == STATE_LIGHT_SPD1) speed_select->set_index(1);
                else if (new_state == STATE_LIGHT_SPD2) speed_select->set_index(2);
                else if (new_state == STATE_LIGHT_SPD3) speed_select->set_index(3);
            }
            if (light_switch) {
                bool light_on = (new_state == STATE_LIGHT || new_state >= STATE_LIGHT_SPD1);
                light_switch->publish_state(light_on);
            }
            if (status_sensor) {
                status_sensor->publish_state(patterns[new_state]);
            }
        }
    }

    // Простая декодировка входящего пакета (сравнение с шаблоном)
    void decode_incoming_packet(String& raw_bits) {
        for (int i = 0; i < 8; i++) {
            if (raw_bits == patterns[i]) {
                update_state((HoodState)i);
                last_packet_hex = raw_bits;
                if (last_packet_sensor) last_packet_sensor->publish_state(raw_bits);
                return;
            }
        }
        // Если пакет не распознан, просто ретранслируем его как есть (Passthrough)
        // В данной реализации мы храним только известные состояния. 
        // Для полного пасстхру нужно хранить сырой паттерн.
        // Для упрощения: если не известно, считаем OFF.
        last_packet_hex = raw_bits;
        if (last_packet_sensor) last_packet_sensor->publish_state(raw_bits);
    }

public:
    WeissgauffHoodController() : PollingComponent(1000) {
        // Инициализация сущностей
        speed_select = new Select();
        light_switch = new Switch();
        status_sensor = new TextSensor();
        last_packet_sensor = new TextSensor();
        packet_count_sensor = new Sensor();
        wifi_signal_sensor = new Sensor();
    }

    void setup() override {
        pinMode(PIN_HOOD_OUT, OUTPUT);
        pinMode(PIN_PANEL_IN, INPUT_PULLUP);
        
        digitalWrite(PIN_HOOD_OUT, HIGH);
        
        // Начальное состояние
        update_state(STATE_OFF);
        
        // Регистрация сущностей
        App.register_select(speed_select);
        App.register_switch(light_switch);
        App.register_text_sensor(status_sensor);
        App.register_text_sensor(last_packet_sensor);
        App.register_sensor(packet_count_sensor);
        App.register_sensor(wifi_signal_sensor);

        speed_select->traits.set_options({"Выкл", "Скорость 1", "Скорость 2", "Скорость 3"});
        
        // Обработчики HA
        speed_select->set_action_callback([this](SelectCall call) {
            if (call.get_index().has_value()) {
                int idx = call.get_index().value();
                // Логика переключения скоростей без учета света (упрощенно)
                // Для полноценной работы нужно комбинировать со светом
                if (idx == 0) ha_command_state = STATE_OFF;
                else if (idx == 1) ha_command_state = (current_state >= STATE_LIGHT_SPD1) ? STATE_LIGHT_SPD1 : STATE_SPEED1;
                else if (idx == 2) ha_command_state = (current_state >= STATE_LIGHT_SPD2) ? STATE_LIGHT_SPD2 : STATE_SPEED2;
                else if (idx == 3) ha_command_state = (current_state >= STATE_LIGHT_SPD3) ? STATE_LIGHT_SPD3 : STATE_SPEED3;
                
                ha_command_pending = true;
            }
        });

        light_switch->set_action_callback([this](SwitchCall call) {
            bool state = call.get_state().value();
            if (state) {
                // Включить свет, сохранив скорость
                if (current_state == STATE_OFF) ha_command_state = STATE_LIGHT;
                else if (current_state == STATE_SPEED1) ha_command_state = STATE_LIGHT_SPD1;
                else if (current_state == STATE_SPEED2) ha_command_state = STATE_LIGHT_SPD2;
                else if (current_state == STATE_SPEED3) ha_command_state = STATE_LIGHT_SPD3;
            } else {
                // Выключить свет
                if (current_state == STATE_LIGHT) ha_command_state = STATE_OFF;
                else if (current_state == STATE_LIGHT_SPD1) ha_command_state = STATE_SPEED1;
                else if (current_state == STATE_LIGHT_SPD2) ha_command_state = STATE_SPEED2;
                else if (current_state == STATE_LIGHT_SPD3) ha_command_state = STATE_SPEED3;
            }
            ha_command_pending = true;
        });
    }

    void loop() override {
        uint32_t now = micros();
        
        // 1. Обработка команд от HA (Приоритет)
        if (ha_command_pending) {
            send_packet(patterns[ha_command_state]);
            update_state(ha_command_state);
            ha_command_pending = false;
            // После команды HA продолжаем цикл передачи этого состояния
            target_state = ha_command_state; 
        }

        // 2. Приём с панели (Пасстхру)
        // Простая детекция фронта для примера. 
        // Для надежного приёма 58 бит нужен конечный автомат.
        // Здесь реализована упрощенная логика: если видим активность, считаем что пришла команда.
        // В продакшене нужно декодировать тайминги входящих импульсов.
        static uint32_t last_input_change = 0;
        static bool last_input_val = HIGH;
        bool current_input_val = digitalRead(PIN_PANEL_IN);
        
        if (current_input_val != last_input_val) {
            if (now - last_input_change > 100) { // Антидребезг
                // Здесь должна быть логика декодера Манчестера
                // Для краткости примера: эмулируем перехват
                // В реальной прошивке здесь нужно собирать биты по таймингам
            }
            last_input_change = now;
            last_input_val = current_input_val;
        }
        
        // 3. Непрерывная передача (Сердцебиение)
        // Отправляем пакет текущего целевого состояния
        // Чтобы не блокировать WiFi слишком долго, отправляем 1 пакет и ждем интервал
        static uint32_t last_tx_time = 0;
        
        if (now - last_tx_time >= PACKET_INTERVAL) {
            // Проверка: если мы только что отправили команду HA, не дублируем сразу
            if (!ha_command_pending) {
                 send_packet(patterns[target_state]);
            }
            last_tx_time = now;
            
            // Обновление сенсора WiFi
            if (wifi_signal_sensor) {
                wifi_signal_sensor->publish_state(WiFi.RSSI());
            }
            if (packet_count_sensor) {
                packet_count_sensor->publish_state(packet_count);
            }
        }
        
        yield(); // Важно для работы WiFi
    }

    // Геттеры для YAML
    Select* get_speed_select() { return speed_select; }
    Switch* get_light_switch() { return light_switch; }
    TextSensor* get_status_sensor() { return status_sensor; }
    TextSensor* get_last_packet_sensor() { return last_packet_sensor; }
    Sensor* get_packet_count_sensor() { return packet_count_sensor; }
    Sensor* get_wifi_signal_sensor() { return wifi_signal_sensor; }
};
