#include "MotorController.hpp"

MotorController* MotorController::instance = nullptr;

MotorController::MotorController(const AppConfig::MotorConfig& cfg)
  : config(cfg),
    motor(cfg.pole_pairs, cfg.phase_resistance, cfg.kv_rating),
    driver(cfg.pwm_u_h, cfg.pwm_u_l, cfg.pwm_v_h, cfg.pwm_v_l, cfg.pwm_w_h, cfg.pwm_w_l),
    current_sense(cfg.shunt_resistance, cfg.amp_gain, cfg.curr_u, cfg.curr_v, cfg.curr_w),
    sensor(cfg.spi3_cs, cfg.bit_resolution, cfg.angle_register),
    spi3(cfg.spi3_mosi, cfg.spi3_miso, cfg.spi3_sck),
    command(Serial)
{
  instance = this;
}

void MotorController::begin() {
  Serial.begin(115200);
  Serial.println("SimpleFOC G431RB: Starting configuration...");

  // ENCODER
  spi3.begin();
  sensor.init(&spi3);
  motor.linkSensor(&sensor);
  Serial.println("Sensor initialized.");

  // DRIVER
  driver.pwm_frequency = config.pwm_frequency;
  driver.voltage_power_supply = config.v_supply;
  driver.voltage_limit = config.v_limit;
  driver.init();
  driver.enable();
  motor.linkDriver(&driver);
  Serial.println("Driver initialized and enabled.");

  // CURRENT SENSE
  current_sense.linkDriver(&driver);
  current_sense.init();
  motor.linkCurrentSense(&current_sense);
  Serial.println("Current sense initialized.");

  // MOTOR
  motor.controller = MotionControlType::velocity;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.init();

  // PID / FILTRY / LIMITY

  // PID - velocity
  motor.PID_velocity.P = config.pid_v_p;
  motor.PID_velocity.I = config.pid_v_i;
  motor.PID_velocity.D = config.pid_v_d;
  motor.PID_velocity.output_ramp = config.pid_v_output_ramp;
  motor.LPF_velocity.Tf = config.lpf_velocity_Tf;
  motor.current_limit = config.current_limit;

  // PID - torque -> Id, Iq
 
  // Q axis
  motor.PID_current_q.P = config.pid_iq_p;                        
  motor.PID_current_q.I = config.pid_iq_i;                        
  motor.PID_current_q.D = config.pid_iq_d;
  motor.PID_current_q.limit = motor.voltage_limit; 
  motor.PID_current_q.output_ramp = config.pid_iq_output_ramp;    
  motor.LPF_current_q.Tf= config.lpf_iq_Tf;                      

  // D axis
  motor.PID_current_d.P = config.pid_id_p;                        
  motor.PID_current_d.I = config.pid_id_i;                        
  motor.PID_current_d.D = config.pid_id_d;
  motor.PID_current_d.limit = motor.voltage_limit; 
  motor.PID_current_d.output_ramp = config.pid_id_output_ramp;    
  motor.LPF_current_d.Tf= config.lpf_id_Tf;                       

  // FOC init
  motor.initFOC();
  Serial.println("Motor FOC initialized!");

  // Target startowy
  motor.target = config.initial_target_velocity;
  Serial.print("Initial motor target set to: ");
  Serial.print(motor.target);
  Serial.println(" rad/s");

  // Monitoring (opcjonalnie)
  motor.useMonitoring(Serial);
  motor.monitor_downsample = 100;
  motor.monitor_variables = _MON_VEL | _MON_ANGLE | _MON_TARGET | _MON_CURR_Q | _MON_CURR_D | _MON_VOLT_Q | _MON_VOLT_D;

  // Commander
  command.add('T', onTargetCmd, "target velocity [rad/s]");
  command.add('M', onModeCmd, "mode: 0-torque, 1-velocity, 2-angle"); 
  command.add('C', onCurrentCmd, "target current [A]"); // Zadanie prądu w trybie torque

  _delay(1000);
  Serial.println("SimpleFOC configuration complete. Entering loop...");
}

void MotorController::update() {
  motor.loopFOC();
  motor.move();
  // opcjonalnie: motor.monitor();
}

void MotorController::runCommand() {
  command.run();
}

void MotorController::onTargetCmd(char* cmd) { // Velocity target in velocity mode
  if (instance) {
    instance->command.scalar(&instance->motor.target, cmd);
  }
}

void MotorController::onCurrentCmd(char* cmd) { // Current target in torque mode
  if (instance) {
    instance->command.scalar(&instance->motor.target, cmd);
  }
}

void MotorController::onModeCmd(char* cmd) {
  if (instance) {
    int mode = atoi(cmd); // zamiana tekstu na liczbę
    switch (mode) {
      case 0:
        instance->motor.controller = MotionControlType::torque;
        Serial.println("Mode: torque");
        break;
      case 1:
        instance->motor.controller = MotionControlType::velocity;
        Serial.println("Mode: velocity");
        break;
      case 2:
        instance->motor.controller = MotionControlType::angle;
        Serial.println("Mode: angle");
        break;
      default:
        Serial.println("Unknown mode!");
        break;
    }
  }
}

void MotorController::feedCommand(char* cmdString) {
  command.run(cmdString);
}

void MotorController::changeControlMode(MotionControlType new_mode) {
    if (motor.controller == new_mode) return;
    motor.target = 0;
    motor.controller = new_mode;
    switch (new_mode) {
        case MotionControlType::angle: motor.P_angle.reset(); motor.PID_velocity.reset(); break;
        case MotionControlType::velocity: motor.PID_velocity.reset(); break;
        case MotionControlType::torque: motor.PID_current_q.reset(); motor.PID_current_d.reset(); break;
        default: break;
    }
    updateLimits();
}

void MotorController::changeTorqueControlType(TorqueControlType new_torque_mode) {
  if (motor.torque_controller == new_torque_mode) return;

  motor.torque_controller = new_torque_mode;

  // Po zmianie trybu TORQUE, również musimy zaktualizować limit pętli prędkości!
  updateLimits();

  // esetujemy pętle prądowe
  motor.PID_current_q.reset();
  motor.PID_current_d.reset();

  Serial.printf("Torque control type changed to: %d\n", (int)new_torque_mode);
}


void MotorController::setCurrentLimit(float limit) {
    motor.current_limit = limit;
    updateLimits();
}

void MotorController::setVelocityLimit(float limit) {
    motor.velocity_limit = limit;
    updateLimits();
}

void MotorController::updateLimits() {
    if (motor.torque_controller == TorqueControlType::voltage) {
        motor.PID_velocity.limit = motor.voltage_limit;
    } else {
        motor.PID_velocity.limit = motor.current_limit;
    }
    motor.P_angle.limit = motor.velocity_limit;
}

// void MotorController::setTorqueFF(float torque_ff) {
//     // Ustawiamy poprawną zmienną w obiekcie motor z biblioteki SimpleFOC
//     motor.torque_controller_target = torque_ff;
// }

