#include <Arduino.h>
#include <SimpleFOC.h>
#include "CANProfile.h"

#define STM32_MASTER

// Konfiguracja adresacji magistrali CAN
const CanNetworkConfig AppCanConfig = MasterConfig_Node;
const CanNetworkConfig SlaveConfig = SlaveConfig_Node0; // Node ID 0x55

// --- Obiekty globalne ---

MotorController motorController(AppConfig::BoardConfig); // Kontroler SimpleFoc
SimpleCan* canBusDriver = CreateCanLib(A_CAN_TX, A_CAN_RX); // Zwraca obiekt klasy SimpleCan_STM32G4xx
RxFromCAN canCommandHandler(&motorController); // Broker komend
CANMotorController canBus(canBusDriver, &canCommandHandler, AppCanConfig.MyNodeId); // Logika transmisji

// Przykład komendy: "V 10.5 0.2"
void onCmdVel(char* cmd) {
    // strtok modyfikuje oryginalny string, więc pracujemy na nim
    char* token;
    
    // Pobierz pierwszy token (prędkość)
    token = strtok(cmd, " "); // Używamy spacji jako separatora
    if (!token) return; // Jeśli nie ma argumentów, wyjdź
    float vel = atof(token);

    // Pobierz drugi token (torque_ff), jeśli istnieje
    token = strtok(NULL, " ");
    float torque_ff = (token) ? atof(token) : 0.0f; // Jeśli nie ma drugiego argumentu, użyj 0.0

    int can_id = PP_MAKE_CAN_ID(SlaveConfig.MyNodeId, SET_INPUT_VALUE);
    Serial.printf("TX -> SET_INPUT_VEL to Node %d: vel=%.2f, torque_ff=%.2f\n", SlaveConfig.MyNodeId, vel, torque_ff);
    canBus.CANSendFloat(vel, torque_ff, can_id);
}

// Przykład komendy: "M 2"
void onCmdMode(char* cmd) {
    char* token = strtok(cmd, " ");
    if (!token) return;
    int mode = atoi(token); // atoi dla liczb całkowitych

    int can_id = PP_MAKE_CAN_ID(SlaveConfig.MyNodeId, SET_CONTROLLER_MODES);
    Serial.printf("TX -> SET_CONTROLLER_MODES to Node %d: mode=%d\n", SlaveConfig.MyNodeId, mode);
    canBus.CANSendInt((int32_t)mode, (int32_t)INPUT_MODE_PASSTHROUGH, can_id);
}

// Przykład komendy: "S 8"
void onCmdState(char* cmd) {
    char* token = strtok(cmd, " ");
    if (!token) return;
    int state = atoi(token);

    int can_id = PP_MAKE_CAN_ID(SlaveConfig.MyNodeId, SET_AXIS_REQUESTED_STATE);
    Serial.printf("TX -> SET_AXIS_REQUESTED_STATE to Node %d: state=%d\n", SlaveConfig.MyNodeId, state);
    canBus.CANSendInt((int32_t)state, can_id);
}

// Przykład komendy: "L 25.0 1.8"
void onCmdLimits(char* cmd) {
    char* token;
    
    token = strtok(cmd, " ");
    if (!token) return;
    float vel_limit = atof(token);

    token = strtok(NULL, " ");
    float curr_limit = (token) ? atof(token) : 0.0f;

    int can_id = PP_MAKE_CAN_ID(SlaveConfig.MyNodeId, SET_LIMITS);
    Serial.printf("TX -> SET_LIMITS to Node %d: vel_limit=%.2f, curr_limit=%.2f\n", SlaveConfig.MyNodeId, vel_limit, curr_limit);
    canBus.CANSendFloat(vel_limit, curr_limit, can_id);
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
    Serial.println("Commands:");
    Serial.println(" V <vel> <torque_ff>   (e.g., V 10.0 0.1)");
    Serial.println(" M <mode>              (e.g., M 2)");
    Serial.println(" S <state>             (e.g., S 8)");
    Serial.println(" L <vel_lim> <curr_lim>  (e.g., L 20.0 1.5)"); 

    // motorController.begin();
    canBus.canInit();
    
    motorController.command.add('V', onCmdVel, "Set Velocity & TorqueFF");
    motorController.command.add('M', onCmdMode, "Set Control Mode");
    motorController.command.add('S', onCmdState, "Set Axis State");
    motorController.command.add('L', onCmdLimits, "Set Velocity & Current Limits");

    Serial.println("Ready.");
}

void loop() {

    motorController.runCommand();
    canBus.Can1->Loop();

    // canTest();
}

