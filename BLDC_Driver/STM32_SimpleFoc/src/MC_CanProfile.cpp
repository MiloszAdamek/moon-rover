// MC_CANProfile.cpp

#include "MC_CANProfile.h"

void CANMotorController::canInit(){

    Serial.println("Initializing CAN bus...");

    if (this->Can1->Init(Mbit1) != CAN_OK) {
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
    this->Can1->ConfigFilter(&filter);

    // Włącz magistralę CAN
    if (this->Can1->Start() != CAN_OK) {
        Serial.println("CAN Start Failed!");
        while(1);
    }
}

void CANMotorController::HandleCanMessage(const SimpleCanRxHeader rxHeader, const uint8_t *rxData)
{
    int Device = PP_GET_DEVICE_ID(rxHeader.Identifier);
    int Command = PP_GET_MESSAGE_ID(rxHeader.IdType);

    // Sprawdzamy, czy mamy handler do obsłużenia komendy
    if (!pRxCommands) return;

    switch(Command)
    {
        case SET_INPUT_VEL: {
            // ODPakowujemy dwa floaty z 8 bajtów danych
            float velocity;
            // float torque_ff;
            memcpy(&velocity, rxData, sizeof(float));
            // memcpy(&torque_ff, rxData + 4, sizeof(float));
            // Przekazujemy zdekodowane wartości do handlera
            pRxCommands->Received_SetInputVel(Device, velocity);
            break;
        }

        case SET_CONTROLLER_MODES: {
            // ODPakowujemy dwa inty (które są enumami)
            Control_Mode_t control_mode;
            Input_Mode_t input_mode;
            memcpy(&control_mode, rxData, sizeof(int32_t));
            memcpy(&input_mode, rxData + 4, sizeof(int32_t));
            pRxCommands->Received_SetControllerModes(Device, control_mode, input_mode);
            break;
        }

        // TODO: Dodaj obsługę innych komend, np. SET_INPUT_POS
        // case SET_INPUT_POS: { ... break; }

        default:
            // Komenda nieobsługiwana przez nasz profil
            break;
    } 
}

void RxFromCAN::Received_SetInputVel(const int Device, float velocity) {
    if (pMotorController) {
        pMotorController->setTargetVelocity(velocity);

        Serial.printf("CAN CMD from Dev %d: Set Velocity -> %.2f rad/s", Device, velocity);
    }
}

void RxFromCAN::Received_SetInputPos(const int Device, float position) {
    if (pMotorController) {
        pMotorController->setTargetPosition(position);

        Serial.printf("CAN CMD from Dev %d: Set Position -> %.2f rad", Device, position);
    }
}

void RxFromCAN::Received_SetInputTorque(int Device, float torque) {
    if (pMotorController) {
        pMotorController->setTargetTorque(torque);

        Serial.printf("CAN CMD from Dev %d: Set Torque -> %.2f Nm", Device, torque);
    }
}

void RxFromCAN::Received_SetControllerModes(const int Device, Control_Mode_t control_mode, Input_Mode_t input_mode) {
    if (pMotorController) {
        // Ignorujemy input_mode na razie
        (void)input_mode;

        MotionControlType new_mode;
        TorqueControlType new_torque_mode;
        bool changed = true;
        bool new_torque_mode_set = false;
        switch(control_mode) {
            case CONTROL_MODE_TORQUE_VOLTAGE: new_mode = MotionControlType::torque; new_torque_mode = TorqueControlType::voltage; new_torque_mode_set = true; break;
            case CONTROL_MODE_TORQUE_DC_CURRENT: new_mode = MotionControlType::torque; new_torque_mode = TorqueControlType::dc_current; new_torque_mode_set = true; break;
            case CONTROL_MODE_TORQUE_FOC_CURRENT: new_mode = MotionControlType::torque; new_torque_mode = TorqueControlType::foc_current; new_torque_mode_set = true; break;
            case CONTROL_MODE_VELOCITY: new_mode = MotionControlType::velocity; break;
            case CONTROL_MODE_ANGLE: new_mode = MotionControlType::angle; break;
            case CONTROL_MODE_VELOCITY_OPENLOOP: new_mode = MotionControlType::velocity_openloop; break;
            case CONTROL_MODE_ANGLE_OPENLOOP: new_mode = MotionControlType::angle_openloop; break;
            default: changed = false; break;
        }
        
        if (changed) {
            pMotorController->setControlMode(new_mode);
            if (new_torque_mode_set) {
                pMotorController->setTorqueControlMode(new_torque_mode);
            }
            Serial.printf("CAN CMD from Dev %d: Set Control Mode -> %d\n", Device, control_mode);
        }
    }
}