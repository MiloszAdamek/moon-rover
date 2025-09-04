#include <Arduino.h>
#include <SimpleFOC.h>
#include "CANProfile.h"

#define STM32_MASTER

// Konfiguracja adresacji magistrali CAN
const CanNetworkConfig AppCanConfig = MasterConfig_Node;
const CanNetworkConfig TargetConfig  = SlaveConfig_Node0;

// --- Obiekty globalne ---

MotorController motorController(AppConfig::BoardConfig); // Kontroler SimpleFoc
SimpleCan* canBusDriver = CreateCanLib(A_CAN_TX, A_CAN_RX); // Zwraca obiekt klasy SimpleCan_STM32G4xx
RxFromCAN canCommandHandler(&motorController); // Broker komend
CANMotorController canBus(canBusDriver, &canCommandHandler, AppCanConfig.MyNodeId, TargetConfig.MyNodeId); // Logika transmisji

// Komenda: T <torque> (np. T0.5)
void onCmdTorque(char* cmd) {
    char* token = strtok(cmd, " ");
    if (!token) return;
    float torque = atof(token);
    int can_id = PP_MAKE_CAN_ID(TargetConfig.MyNodeId, SET_INPUT_TORQUE);
    Serial.printf("TX -> SET_INPUT_TORQUE to Node %d: torque=%.2f A\n", TargetConfig.MyNodeId, torque);
    canBus.CANSendFloat(torque, can_id);
}

// Komenda: V <velocity> <torque_ff> (np. V10.0 0.1)
void onCmdVel(char* cmd) {
    char* token = strtok(cmd, " ");
    if (!token) return;
    float vel = atof(token);
    token = strtok(NULL, " ");
    float torque_ff = (token) ? atof(token) : 0.0f;
    int can_id = PP_MAKE_CAN_ID(TargetConfig.MyNodeId, SET_INPUT_VEL);
    Serial.printf("TX -> SET_INPUT_VEL to Node %d: vel=%.2f, torque_ff=%.2f\n", TargetConfig.MyNodeId, vel, torque_ff);
    canBus.CANSendFloat(vel, torque_ff, can_id);
}

// Komenda: P <position> (np. P3.14)
void onCmdPos(char* cmd) {
    char* token = strtok(cmd, " ");
    if (!token) return;
    float pos = atof(token);
    int16_t vel_ff = 0; // Na razie nie implementujemy
    int16_t torque_ff = 0;
    int can_id = PP_MAKE_CAN_ID(TargetConfig.MyNodeId, SET_INPUT_POS);
    Serial.printf("TX -> SET_INPUT_POS to Node %d: pos=%.2f\n", TargetConfig.MyNodeId, pos);
    
    // Budujemy ramkę ręcznie, bo CANSend nie ma wersji dla float, int16, int16
    uint8_t txData[8];
    memcpy(txData, &pos, 4);
    memcpy(txData + 4, &vel_ff, 2);
    memcpy(txData + 6, &torque_ff, 2);
    canBus.Can1->SendMessage(txData, 8, can_id);
}

// Komenda: M <mode> (np. M 1 2-> FOC Current, Torque Control Type Voltage)
void onCmdMode(char* cmd) {
    char* token = strtok(cmd, " ");
    if (!token) return;
    float motion_mode = atof(token);
    token = strtok(NULL, " ");
    float torque_mode = (token) ? atof(token) : 0.0f;
    int can_id = PP_MAKE_CAN_ID(TargetConfig.MyNodeId, SET_CONTROLLER_MODES);
    Serial.printf("TX -> SET_CONTROLLER_MODES to Node %d: motion_mode=%d, torque_mode=%d\n", TargetConfig.MyNodeId, motion_mode, torque_mode);
    canBus.CANSendInt((int32_t)motion_mode, (int32_t)torque_mode, can_id);
}

// Komenda: S <state> (np. S 8 -> Closed Loop)
void onCmdState(char* cmd) {
    char* token = strtok(cmd, " ");
    if (!token) return;
    int state = atoi(token);
    int can_id = PP_MAKE_CAN_ID(TargetConfig.MyNodeId, SET_AXIS_REQUESTED_STATE);
    Serial.printf("TX -> SET_AXIS_REQUESTED_STATE to Node %d: state=%d\n", TargetConfig.MyNodeId, state);
    canBus.CANSendInt((int32_t)state, can_id);
}

// Komenda: L <vel_lim> <curr_lim> (np. L 25.0 1.8)
void onCmdLimits(char* cmd) {
    char* token = strtok(cmd, " ");
    if (!token) return;
    float vel_limit = atof(token);
    token = strtok(NULL, " ");
    float curr_limit = (token) ? atof(token) : 0.0f;
    int can_id = PP_MAKE_CAN_ID(TargetConfig.MyNodeId, SET_LIMITS);
    Serial.printf("TX -> SET_LIMITS to Node %d: vel_limit=%.2f, curr_limit=%.2f\n", TargetConfig.MyNodeId, vel_limit, curr_limit);
    canBus.CANSendFloat(vel_limit, curr_limit, can_id);
}

// Komenda: R (bez argumentów)
void onCmdReboot(char* cmd) {
    (void)cmd;
    int can_id = PP_MAKE_CAN_ID(TargetConfig.MyNodeId, REBOOT);
    Serial.printf("TX -> REBOOT to Node %d\n", TargetConfig.MyNodeId);
    canBus.Can1->SendMessage(nullptr, 0, can_id); // Wysyłamy ramkę bez danych
}

void canTest(){

  // === TEST 1: Wysyłanie prostej ramki co 1 sekundę ===
  
  uint8_t frame1_data[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE};
  int frame1_id = AppCanConfig.MyNodeId; // Standardowy ID
  
  Serial.printf("Sending frame with ID 0x%X and 4 bytes of data...\n", frame1_id);
  
  // Używamy metody SendMessage() z biblioteki
  // Argumenty: (wskaźnik na dane, długość danych, ID ramki)
  canBus.Can1->SendMessage(frame1_data, 4, frame1_id);

  delay(1000); // Czekaj 1 sekundę
}

void setup() {
    Serial.begin(115200);
    while (!Serial);
    Serial.println("\n--- SimpleFOC CAN Commander (Master) ---");
    Serial.println("Target Node ID: " + String(TargetConfig.MyNodeId));
    Serial.println("Commands (separate with space, e.g., 'V 10.5 0.1'):");
    Serial.println(" T <torque>            (Amps)");
    Serial.println(" V <velocity> <torque_ff>");
    Serial.println(" P <position>");
    Serial.println(" M <motion_mode_id> <torque_mode_id");
    Serial.println(" S <state_id>");
    Serial.println(" L <vel_lim> <curr_lim>");
    Serial.println(" R (Reboot Slave)");

    // motorController.begin();
    canBus.canInit();
    
    motorController.command.add('T', onCmdTorque, "Set Torque");
    motorController.command.add('V', onCmdVel, "Set Velocity & TorqueFF");
    motorController.command.add('P', onCmdPos, "Set Position");
    motorController.command.add('M', onCmdMode, "Set Control Mode");
    motorController.command.add('S', onCmdState, "Set Axis State");
    motorController.command.add('L', onCmdLimits, "Set Velocity & Current Limits");
    motorController.command.add('R', onCmdReboot, "Reboot Slave");

    Serial.println("Ready.");
}

void loop() {

    motorController.runCommand();
    canBus.Can1->Loop();

    // canTest();
}

