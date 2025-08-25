#include "MotorController.hpp"
#include "Config.hpp"
// #include "DriverSPI.hpp"
// #include "spi1_hal_init.hpp"
#include "PinmapCustom.h"
#include "SimpleCAN.h"
#include "PingPongCANProfile.h"
#include "Odrive.h"
#include "fdcan_hal_init.h"

MotorController motorController(AppConfig::BoardConfig);

SimpleCan* canBus = CreateCanLib(PB9, PB8); 

const int CAN_NODE_ID = 0;

// === FUNKCJA CALLBACK DLA ODBIORU WIADOMOŚCI ===
// Ta funkcja będzie automatycznie wywoływana przez bibliotekę, gdy przyjdzie ramka.
void onCanMessageReceived(SimpleCanRxHeader header, const uint8_t* data, void* userData) {
  // Nazwy argumentów mogą być inne, ale typy i kolejność muszą się zgadzać.
  
  // userData nie jest używane w tym prostym przykładzie, więc możemy je zignorować.
  // Można dodać (void)userData; aby kompilator nie ostrzegał o nieużywanej zmiennej.
  (void)userData;

  int32_t cmd_id = header.Identifier & 0x01F;

  switch (cmd_id) {
    case SET_INPUT_VEL: {
      float target_velocity;
      // Używamy `data` (który jest teraz const uint8_t*) do odczytu
      memcpy(&target_velocity, data, sizeof(target_velocity));
      
      motorController.setTarget(target_velocity);
      break;
    }
    // ... obsługa innych komend
  }
}

void handleCAN() {
  static uint32_t last_can_telemetry_time = 0;
  if (millis() - last_can_telemetry_time > 100) {
    last_can_telemetry_time = millis();

    uint8_t txData[8];
    float current_position = motorController.getAngle();
    float current_velocity = motorController.getVelocity();

    memcpy(txData, &current_position, 4);
    memcpy(txData + 4, &current_velocity, 4);

    int can_id = (CAN_NODE_ID << 5) | ENCODER_ESTIMATES;
    
    canBus->SendMessage(txData, 8, can_id);
  }
  
  canBus->Loop(); 
}


void setup() {

  Serial.begin(115200);
  while (!Serial);
  Serial.println("\n--- STM32G431 ESC1 with SimpleCAN ---");

  motorController.begin();

  Serial.println("Initializing CAN bus...");

  if (canBus->Init(Mbit1) != CAN_OK) {
    Serial.println("CAN Init Failed!");
    while(1);
  }

  FilterDefinition filter;
  filter.FilterIndex = 0;
  filter.IdType = CAN_STDID;
  filter.FilterType = CAN_FILTER_MASK;
  filter.FilterID1 = 0;
  filter.FilterID2 = 0; // Maska 0 = akceptuj wszystko
  filter.FilterConfig = CAN_FILTER_TO_RXFIFO0;
  canBus->ConfigFilter(&filter);

  // Ustaw funkcję, która ma być wywoływana przy odbiorze ramki
  canBus->ActivateNotification(8, onCanMessageReceived, nullptr);

  // Włącz magistralę CAN
  if (canBus->Start() != CAN_OK) {
    Serial.println("CAN Start Failed!");
    while(1);
  }

  Serial.println("--- Setup Complete. Entering main loop. ---");
}

void loop() {

    motorController.update();
    motorController.runCommand();
    handleCAN();
}


