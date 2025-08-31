#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "Config.hpp"

class MotorController {
public:
  explicit MotorController(const AppConfig::MotorConfig& cfg);

  void begin();        // wywołaj w setup()
  void update();       // wywołuj w loop() - FOC + sterowanie
  void runCommand();   // wywołuj w loop() - Commander

  void feedCommand(char* cmdString); // przekazanie komendy jako ciąg znaków

  // TODO: Settery, sprawdzić poprawność z SimpleFoc
  // void setTargetVelocity(float rad_s) {motor.target = rad_s;}
  // void setTargetPosition(float pos) { motor.target = pos; }
  // void setTargetTorque(float torque) { motor.target = torque; }
  void setTargetValue(float target) { motor.target = target; } // Depends on the motor controller mode, this could be position, velocity, or torque
  void setCurrentLimit(float current_limit);
  void setVelocityLimit(float velocity_limit) { motor.velocity_limit = velocity_limit; }

  void setControlMode(MotionControlType mode) { motor.controller = mode; }
  void setTorqueControlMode(TorqueControlType mode) { motor.torque_controller = mode; }

  float getTarget() const { return motor.target; }
  float getAngle() { return sensor.getAngle(); }
  MotionControlType getMotionControlType() const { return motor.controller; }

  float getVelocity() const { return motor.shaft_velocity; }
  float getCurrent_q() const { return motor.current.q; }
  float getCurrent_d() const { return motor.current.d; }

  void enable() { driver.enable(); }
  void disable() { driver.disable(); }
  
  void reboot() { NVIC_SystemReset(); } // Twardy reset
  // void clearErrors() { motor.clearError(); }
  void setPosPID(float p) { motor.P_angle.P = p;}
  void setVelPID(float p, float i, float d) { motor.PID_velocity.P = p; motor.PID_velocity.I = i; motor.PID_velocity.D = d; }

  // void getAngle(float& angle);

  // Commander
  Commander command;
  
private:
  const AppConfig::MotorConfig config;

  // Commander: statyczny wskaźnik do instancji + statyczny callback
  static MotorController* instance;
  static void onTargetCmd(char* cmd); // Zmiana zadanej prędkości obrotowej
  static void onModeCmd(char* cmd); // Zmiana trybu pracy kontrolera
  static void onCurrentCmd(char* cmd); // Zadanie prądu w trybie torque

  // Obiekty SimpleFOC
  BLDCMotor motor;
  // BLDCDriver3PWM driver;
  BLDCDriver6PWM driver;
  LowsideCurrentSense current_sense;
  MagneticSensorSPI sensor;

  // SPI3
  SPIClass spi3;


};