#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "Config.hpp"

class MotorController {
public:
  explicit MotorController(const AppConfig::MotorConfig& cfg, float torque_feed_forward = 0.0f);

  void begin();        // wywołaj w setup()
  void update();       // wywołuj w loop() - FOC + sterowanie
  void runCommand();   // wywołuj w loop() - Commander
  
  void feedCommand(char* cmdString); // przekazanie komendy jako ciąg znaków

  void setTarget(float target) { motor.target = target; } // Depends on the motor controller mode, this could be position, velocity, or torque
  void setCurrentLimit(float current_limit);
  void setVelocityLimit(float velocity_limit);
  void setTorqueFF(float torque_ff) { this->torque_feed_forward = torque_ff; }

  void changeControlMode(MotionControlType new_mode);
  void changeTorqueControlType(TorqueControlType new_mode);

  void updateLimits();
  float getTarget() const { return motor.target; }
 
  MotionControlType getMotionControlType() const { return motor.controller; }
  TorqueControlType getTorqueControlType() const { return motor.torque_controller; }

  float getI_q() const { return motor.current.q; }
  float getI_d() const { return motor.current.d; }

  float getVelocity() { return sensor.getVelocity(); }
  float getAngle() { return sensor.getAngle(); }
  
  float getShaftVelocity() const { return motor.shaft_velocity; }
  float getShaftAngle() const { return motor.shaft_angle; }

  float getBusVoltage() { return driver.voltage_power_supply; }
  float getBusCurrent() { return current_sense.getDCCurrent(motor.electrical_angle); }

  // uint32_t getErrorFlags() const { return motor.error_flags; } // Zmapowane błędy

  void enable() { driver.enable(); }
  void disable() { driver.disable(); }
  
  void reboot() { NVIC_SystemReset(); } // Twardy reset
  // void clearErrors() { motor.clearError(); }
  void setPosPID(float p) { motor.P_angle.P = p;}
  void setVelPID(float p, float i, float d) { motor.PID_velocity.P = p; motor.PID_velocity.I = i; motor.PID_velocity.D = d; }

  void printMonitoredValues();
  void printEncoder();
  void printCurrentSensor();
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

  float torque_feed_forward;
};