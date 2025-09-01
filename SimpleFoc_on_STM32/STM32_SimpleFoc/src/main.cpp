// main.cpp

#include "MotorController.hpp"
#include "Config.hpp"
#include "OdriveProtocol.h"
#include "CANProfile.h"

// Konfiguracja adresacji magistrali CAN
const CanNetworkConfig AppCanConfig = SlaveConfig_Node0;

MotorController motorController(AppConfig::BoardConfig); // Kontroler SimpleFoc
SimpleCan* canBusDriver = CreateCanLib(A_CAN_TX, A_CAN_RX); // Zwraca obiekt klasy SimpleCan_STM32G4xx
RxFromCAN canCommandHandler(&motorController); // Broker komend
CANMotorController canBus(canBusDriver, &canCommandHandler, AppCanConfig.MyNodeId); // Logika transmisji

// TODO: Wysyłanie telemtrii, w przerwaniu np.
void handleCAN() {
  static uint32_t last_can_telemetry_time = 0;
  // Wysyłaj telemetrię co 50ms (20 Hz)
  if (millis() - last_can_telemetry_time > 50) { 
    last_can_telemetry_time = millis();

    // Pobierz aktualne dane z kontrolera silnika
    float current_velocity = motorController.getVelocity();
    float current_angle = motorController.getAngle();
    
    // Zbuduj ID ramki - użyjemy protokołu ODrive
    // ID osi = MY_CAN_ID, ID komendy = ENCODER_ESTIMATES
    int can_id = (AppCanConfig.MyNodeId << 5) | ENCODER_ESTIMATES;
    
    // Użyj gotowej metody z CANMotorController (który dziedziczy z SimpleCANProfile)
    // do wysłania dwóch floatów. Ona sama zajmie się spakowaniem ich do 8 bajtów.
    // Argumenty: (wartość 1, wartość 2, ID ramki)
    canBus.CANSendFloat(current_angle, current_velocity, can_id);
  }

  // Ta funkcja jest kluczowa dla działania odbioru i kolejek!
  canBusDriver->Loop();
}

void setup() {

  Serial.begin(115200);
  while (!Serial);

  Serial.println("\n--- STM32G431 ESC1 with SimpleCAN ---");

  motorController.begin();
  canBus.canInit();

  Serial.println("--- Setup Complete. Entering main loop. ---");
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

void loop() {

  motorController.update();
  motorController.runCommand();
  canBusDriver->Loop();
    
  // handleCAN();
  // canTest();
}


