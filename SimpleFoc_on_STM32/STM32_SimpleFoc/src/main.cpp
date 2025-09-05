// main.cpp

#include "MotorController.hpp"
#include "Config.hpp"
#include "OdriveProtocol.h"
#include "CANProfile.h"

long timer_monitor = millis();

// Konfiguracja adresacji magistrali CAN
const CanNetworkConfig AppCanConfig = SlaveConfig_Node0;

// MotorController motorController(AppConfig::GM2804MotorConfig); // Kontroler SimpleFoc
MotorController motorController(AppConfig::ODrive330KVConfig);

SimpleCan* canBusDriver = CreateCanLib(A_CAN_TX, A_CAN_RX); // Zwraca obiekt klasy SimpleCan_STM32G4xx
RxFromCAN canCommandHandler(&motorController); // Broker komend
CANMotorController canBus(canBusDriver, &canCommandHandler, AppCanConfig.MyNodeId, AppCanConfig.MasterNodeId); // Logika transmisji

// TODO: Wysyłanie telemtrii, w przerwaniu np.
void handleCAN() {
  static uint32_t last_can_telemetry_time = 0;
  // Wysyłaj telemetrię co 10ms (100 Hz)
  if (millis() - last_can_telemetry_time > 10) { 
    last_can_telemetry_time = millis();

    canCommandHandler.sendIq();
    // canCommandHandler.SendEncoderEstimates();

  }
  // Ta funkcja jest kluczowa dla działania odbioru i kolejek!
  // canBusDriver->Loop();
}

void setup() {

  canCommandHandler.linkCanBus(&canBus);

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

  // motorController.motorMonitor();
    
  handleCAN();

  // canTest();

  // motorController.nonMotorTests();
  // motorController.printCurrentSensor();

  // if (millis() - timer_monitor > 1000) {
  //  motorController.printMonitoredValues();
  // //  motorController.printCurrentSensor();
  // //  motorController.printEncoder();
  //  timer_monitor = millis();
  // }
}

