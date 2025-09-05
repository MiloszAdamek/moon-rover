#include "Config.hpp"

namespace AppConfig {

const MotorConfig GM2804MotorConfig = {
  
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
6.0f,            // voltage_sensor_align

// Silnik
7,          // pole pairs
168.0f,     // KV
9.0f,       // phase resistance
0.0306f,     // Kt [Nm/A] - stała momentu obrotowego

// Driver/Voltage
12.0f,      // v_supply
10.0f,      // v_limit
20000,      // pwm_frequency [Hz]

// Sterowanie

// Velocity
// 0.1f, 0.05f, 0.0f, // PID P,I,D
// 1000.0f,           // output ramp
// 0.01f,             // LPF Tf
// 1.0f,              // current_limit [A]

0.07f, 0.03f, 0.002f,
1000.0f,
0.2f,
1.0f,

// Torque, Id, Iq
// I_Q axis
5.0f, 1000.0f, 0.0f, // PID P,I,D, I_d
1e6f,  // output ramp
0.01f, // LPF Tf

// I_D axis
5.0f, 1000.0f, 0.0f, // PID P,I,D, I_d
1e6f,  // output ramp
0.01f, // LPF Tf

// Startowy target
0.1f              // [rad/s]
};

} // namespace AppConfig