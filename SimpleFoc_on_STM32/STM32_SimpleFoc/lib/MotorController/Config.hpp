#pragma once
#include <Arduino.h>

// ==========================================================
// ===              WŁĄCZENIE DEBUGOWANIA                 ===
// ==========================================================

#define ENABLE_SERIAL_DEBUGGING

// ==========================================================
// ===              WYBÓR STEROWNIKA BLDC                 ===
// ==========================================================

#define ESC_TYPE_STM32_ESC1 extern
// #define ESC_ODRIVE extern

// ==========================================================
// ===              KONFIGURACJA CAN                      ===
// ==========================================================

struct CanNetworkConfig {
    const int MyNodeId;
    const int MasterNodeId;
};

extern const CanNetworkConfig AppCanConfig;

constexpr CanNetworkConfig MasterConfig_Node = { .MyNodeId = 0x05, .MasterNodeId = 0x05 };
constexpr CanNetworkConfig SlaveConfig_Node0 = { .MyNodeId = 0x09, .MasterNodeId = 0x05 };
constexpr CanNetworkConfig SlaveConfig_Node1 = { .MyNodeId = 0x0A, .MasterNodeId = 0x05 };

// Interwały wysyłania telemetrii w milisekundach
constexpr unsigned long ENCODER_ESTIMATES_INTERVAL_MS = 20;  // 50 Hz
constexpr unsigned long HEARTBEAT_INTERVAL_MS = 250;       // 4 Hz
constexpr unsigned long IQ_MEASUREMENT_INTERVAL_MS = 50;   // 20 Hz (opcjonalnie)

// ==========================================================
// ===              KONFIGURACJA SIMPLEFOC                ===
// ==========================================================

namespace AppConfig {

struct MotorConfig {
  // Piny PWM L i H
  uint8_t pwm_u_l, pwm_v_l, pwm_w_l;
  uint8_t pwm_u_h, pwm_v_h, pwm_w_h;

  // SPI3
  uint8_t spi3_mosi, spi3_miso, spi3_sck, spi3_cs;

  // Enkoder AS5048A
  uint8_t  bit_resolution;
  uint16_t angle_register;

  // ADC - Current sense (low-side)
  uint8_t curr_u, curr_v, curr_w;
  float   shunt_resistance;
  float   amp_gain;
  float voltage_sensor_align;

  // Parametry silnika
  int   pole_pairs;
  float kv_rating;
  float phase_resistance;

  // Driver/Voltage
  float    v_supply;
  float    v_limit;
  uint32_t pwm_frequency;

  // Sterowanie

    // Velocity
    float pid_v_p, pid_v_i, pid_v_d;
    float pid_v_output_ramp;
    float lpf_velocity_Tf;
    float current_limit;

    // Torque, Id, Iq
    // I_Q axis
    float pid_iq_p, pid_iq_i, pid_iq_d;
    float pid_iq_output_ramp;
    float lpf_iq_Tf;

    // I_D axis
    float pid_id_p, pid_id_i, pid_id_d;
    float pid_id_output_ramp;
    float lpf_id_Tf;

  // Startowy target
  float initial_target_velocity;
};

// Domyślna konfiguracja płytki/połączeń
extern const MotorConfig BoardConfig;

} // namespace AppConfig

#ifdef ENABLE_SERIAL_DEBUGGING
    #define LOG(format, ...) Serial.printf(format, ##__VA_ARGS__)
#else
    #define LOG(format, ...) do {} while (0)
#endif