// main.cpp

#include "MotorController.hpp"
#include "Config.hpp"
// #include "DriverSPI.hpp"
// #include "spi1_hal_init.hpp"
#include "PinmapCustom.h"
#include "Odrive.h"
#include "MC_CANProfile.h"

MotorController motorController(AppConfig::BoardConfig);

// 1. Stwórz obiekt SimpleCan
SimpleCan* canBusDriver = CreateCanLib(A_CAN_TX, A_CAN_RX); // Zwraca obiekt klasy SimpleCan_STM32G4xx

// 2. Stwórz obiekt, który będzie obsługiwał logikę komend
RxFromCAN canCommandHandler(&motorController);

// 3. Stwórz główny obiekt kontrolera CAN, wstrzykując mu zależności
CANMotorController canBus(canBusDriver, &canCommandHandler);

const int CAN_NODE_ID = 0;

void handleCAN() {
  static uint32_t last_can_telemetry_time = 0;
  // Wysyłaj telemetrię co 50ms (20 Hz)
  if (millis() - last_can_telemetry_time > 50) { 
    last_can_telemetry_time = millis();

    // Pobierz aktualne dane z kontrolera silnika
    float current_velocity = motorController.getVelocity();
    float current_angle = motorController.getAngle();
    
    // Zbuduj ID ramki - użyjemy protokołu ODrive
    // ID osi = CAN_NODE_ID, ID komendy = ENCODER_ESTIMATES
    int can_id = (CAN_NODE_ID << 5) | ENCODER_ESTIMATES;
    
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

  Serial.println("--- Setup Complete. Entering main loop. ---");
}

void canTest(){

  // === TEST 1: Wysyłanie prostej ramki co 1 sekundę ===
  
  uint8_t frame1_data[] = {0x11, 0x22, 0x33, 0x44};
  int frame1_id = 0x100; // Standardowy ID
  
  Serial.printf("Sending frame with ID 0x%X and 4 bytes of data...\n", frame1_id);
  
  // Używamy metody SendMessage() z biblioteki
  // Argumenty: (wskaźnik na dane, długość danych, ID ramki)
  canBus.Can1->SendMessage(frame1_data, 4, frame1_id);

  delay(1000); // Czekaj 1 sekundę


  // === TEST 2: Wysyłanie ramki z 8 bajtami danych ===

  uint8_t frame2_data[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE};
  int frame2_id = 0x25A; // Inny standardowy ID
  
  Serial.printf("Sending frame with ID 0x%X and 8 bytes of data...\n", frame2_id);
  canBus.Can1->SendMessage(frame2_data, 8, frame2_id);
  
  delay(1000); // Czekaj 1 sekundę


  // === TEST 3: Wysyłanie ramki z rozszerzonym ID (29-bitowym) ===
  
  uint8_t frame3_data[] = {0x01, 0x02};
  long frame3_id = 0x18FEF100; // Rozszerzony ID (często używany w J1939)
  
  Serial.printf("Sending frame with Extended ID 0x%lX and 2 bytes of data...\n", frame3_id);
  
  // Drugi opcjonalny argument `true` w SendMessage oznacza użycie rozszerzonego ID.
  canBus.Can1->SendMessage(frame3_data, 2, frame3_id, true);
  
  delay(1000); // Czekaj 1 sekundę
  
  
  // === TEST 4: Wysyłanie ramki bez danych (tylko "ping") ===
  
  int frame4_id = 0x7FF; // ID broadcast
  
  Serial.printf("Sending frame with ID 0x%X and 0 bytes of data...\n", frame4_id);
  canBus.Can1->SendMessage(nullptr, 0, frame4_id);

  delay(1000); // Czekaj 1 sekundę

}

void loop() {

    // motorController.update();
    // motorController.runCommand();
    // handleCAN();

    // canTest();

}


