#include "Config.hpp"

namespace AppConfig {

// Uzupełnij zgodnie z Twoim hardware
const MotorConfig ODrive330KVConfig = {
  
// PWM U_L, V_L, W_L
A_PHASE_UL, A_PHASE_VL, A_PHASE_WL,
// PWM U_H, V_H, W_H
A_PHASE_UH, A_PHASE_VH, A_PHASE_WH,

// SPI3: MOSI, MISO, SCK, CS
PB5, PB4, PB3, PA15,

// Enkoder AS5048A
14,         // bit_resolution
0x3FFF,     // angle_register

// Current sense pins + parametry
A_OP1_OUT, A_OP2_OUT, A_OP3_OUT,
0.003f,          // shunt_resistance
-64.0f/7.0f,     // amp_gain
1.0f,            // voltage_sensor_align

// Silnik
7,          // pole pairs
330.0f,     // KV [RPM/V]
0.037f,     // phase resistance [Ohm]
0.025f,     // Kt [Nm/A] - stała momentu obrotowego

// Driver/Voltage
18.0f,      // v_supply
17.0f,      // v_limit
20000,      // pwm_frequency [Hz]

// Sterowanie

// Velocity
0.05f, 0.05, 0.001f, // PID P,I,D
1000.0f,           // output ramp
0.08f,             // LPF Tf
5.0f,              // current_limit [A]

// 0.5f, 0.1f, 0.001f, // PID P,I,D
// 300.0f,           // output ramp
// 0.08f,             // LPF Tf
// 5.0f,              // current_limit [A]

// Torque, Id, Iq
// I_Q axis
0.5f, 2.0f, 0.01f, // PID P,I,D, I_d
1e2f,  // output ramp
0.05f, // LPF Tf
// 40.0f  // output limit

// I_D axis
0.7f, 10.0f, 0.0f, // PID P,I,D, I_d
1e3f,  // output ramp
0.08f, // LPF Tf

// Startowy target
0.5f
};

};



// // Torque, Id, Iq
// // I_Q axis
// 1.0f, 5.0f, 0.01f, // PID P,I,D, I_d
// 1e4f,  // output ramp
// 0.05f, // LPF Tf
// // 20.0f  // output limit

// // I_D axis
// 0.7f, 10.0f, 0.0f, // PID P,I,D, I_d
// 1e4f,  // output ramp
// 0.08f, // LPF Tf
