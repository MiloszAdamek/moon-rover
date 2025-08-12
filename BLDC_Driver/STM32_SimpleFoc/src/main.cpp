#include "MotorController.hpp"
#include "Config.hpp"
#include "DriverSPI.hpp"
#include "spi1_hal_init.hpp"

MotorController foc(AppConfig::BoardConfig);
float current_angle = 0.0f;

void setup() {

    foc.begin();
    Serial.println("MotorController initialized.");
}

void loop() {

    // foc.update();
    // foc.runCommand();
    // foc.getAngle(current_angle);
    // Serial.print("Current angle: "); 
    // Serial.println(foc.getTarget() * 180.0f / PI); // rad to degrees

}

