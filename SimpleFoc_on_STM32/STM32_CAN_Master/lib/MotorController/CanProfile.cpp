// CANProfile.cpp

#include "CANProfile.h"
#include "Config.hpp"

void CANMotorController::canInit() {

    Serial.println("Initializing CAN bus...");

    SimpleCANProfile::Init(nullptr);

    FilterDefinition filter;
    filter.FilterIndex = 0;
    filter.IdType = CAN_STDID;
    filter.FilterType = CAN_FILTER_MASK;
    filter.FilterID1 = 0;
    filter.FilterID2 = 0; // Maska 0 = akceptuj wszystko
    filter.FilterConfig = CAN_FILTER_TO_RXFIFO0;
    this->Can1->ConfigFilter(&filter);

    Can1->EnableBlinkOnActivity(LED_BUILTIN);
}

void CANMotorController::HandleCanMessage(const SimpleCanRxHeader rxHeader, const uint8_t *rxData)
{
    int Device = PP_GET_DEVICE_ID(rxHeader.Identifier);
    if (Device != AppCanConfig.MyNodeId) return;

    int Command = PP_GET_MESSAGE_ID(rxHeader.Identifier);

    // Sprawdzamy, czy mamy handler do obsłużenia komendy
    if (!pRxCommands) return;

    switch(Command)
    {
        case SET_INPUT_POS: {
            float pos;
            int16_t vel_ff, torque_ff; // ODrive używa int16 dla FF w tej komendzie
            memcpy(&pos, rxData, sizeof(float));
            memcpy(&vel_ff, rxData + 4, sizeof(int16_t));
            memcpy(&torque_ff, rxData + 6, sizeof(int16_t));
            pRxCommands->Received_SetInputPos(Device, pos, vel_ff, torque_ff);
            break;
        }
        case SET_INPUT_VEL: {
            float vel, torque_ff;
            memcpy(&vel, rxData, sizeof(float));
            memcpy(&torque_ff, rxData + 4, sizeof(float));
            pRxCommands->Received_SetInputVel(Device, vel, torque_ff);
            break;
        }
        case SET_INPUT_TORQUE: {
            float torque;
            memcpy(&torque, rxData, sizeof(float));
            pRxCommands->Received_SetInputTorque(Device, torque);
            break;
        }

        case SET_CONTROLLER_MODES: {
            Control_Mode_t control_mode;
            Input_Mode_t input_mode;
            memcpy(&control_mode, rxData, sizeof(int32_t));
            memcpy(&input_mode, rxData + 4, sizeof(int32_t));
            pRxCommands->Received_SetControllerModes(Device, control_mode, input_mode);
            break;
        }

        case SET_LIMITS: {
            float velocity_limit, current_limit;
            memcpy(&velocity_limit, rxData, sizeof(float));
            memcpy(&current_limit, rxData + 4, sizeof(float));
            pRxCommands->Received_SetLimits(Device, current_limit, velocity_limit);
            break;
        }

        // TODO:
        case SET_POSITION_GAIN: {
            float pos_gain;
            memcpy(&pos_gain, rxData, sizeof(float));
            pRxCommands->Received_SetPosGain(Device, pos_gain);
            break;
        }
        case SET_VEL_GAINS: {
            float vel_gain, vel_integrator_gain;
            memcpy(&vel_gain, rxData, sizeof(float));
            memcpy(&vel_integrator_gain, rxData + 4, sizeof(float));
            pRxCommands->Received_SetVelGains(Device, vel_gain, vel_integrator_gain);
            break;
        }

        case REBOOT: {
            pRxCommands->Received_Reboot(Device);
            break;
        }

        // case CLEAR_ERRORS: {
        //     pRxCommands->Received_ClearErrors(Device);
        //     break;
        // }

        default:
            LOG("CAN CMD from Dev %d: Ignored (unknown command) %d\n", Device, Command);
            break;
    } 
}

void RxFromCAN::Received_SetInputPos(const int Dev, float pos, int16_t vff, int16_t tff) {
    if (pMotorController && pMotorController->getMotionControlType() == MotionControlType::angle) {
        pMotorController->setTarget(pos);
    }
}
void RxFromCAN::Received_SetInputVel(const int Dev, float vel, float tff) {
    if (pMotorController && pMotorController->getMotionControlType() == MotionControlType::velocity) {
        pMotorController->setTarget(vel);
        // pMotorController->setTorqueFF(tff);
    }
}
void RxFromCAN::Received_SetInputTorque(const int Dev, float torque) {
    if (pMotorController && pMotorController->getMotionControlType() == MotionControlType::torque) {
        pMotorController->setTarget(torque);
    }
}

void RxFromCAN::Received_SetControllerModes(const int Device, Control_Mode_t control_mode, Input_Mode_t input_mode) {
    if (pMotorController) {
        // Ignorujemy input_mode na razie
        (void)input_mode;

        MotionControlType new_motion_mode;
        TorqueControlType new_torque_mode;
        bool motion_mode_changed = true;
        bool torque_mode_to_set = false;

        switch(control_mode) {
            case CONTROL_MODE_TORQUE_VOLTAGE: 
                new_motion_mode = MotionControlType::torque; 
                new_torque_mode = TorqueControlType::voltage; 
                torque_mode_to_set = true; 
                break;
            case CONTROL_MODE_TORQUE_DC_CURRENT: 
                new_motion_mode = MotionControlType::torque; 
                new_torque_mode = TorqueControlType::dc_current; 
                torque_mode_to_set = true; 
                break;
            case CONTROL_MODE_TORQUE_FOC_CURRENT: 
                new_motion_mode = MotionControlType::torque; 
                new_torque_mode = TorqueControlType::foc_current; 
                torque_mode_to_set = true; 
                break;
            case CONTROL_MODE_VELOCITY: 
                new_motion_mode = MotionControlType::velocity; 
                break;
            case CONTROL_MODE_ANGLE: 
                new_motion_mode = MotionControlType::angle; 
                break;
            case CONTROL_MODE_VELOCITY_OPENLOOP: 
                new_motion_mode = MotionControlType::velocity_openloop; 
                break;
            case CONTROL_MODE_ANGLE_OPENLOOP: 
                new_motion_mode = MotionControlType::angle_openloop; 
                break;
            default: 
                motion_mode_changed = false; 
                break;
        }
        
        if (motion_mode_changed) {
            
            if (torque_mode_to_set) {
                // Wywołanie bezpiecznej metody, która resetuje regulatory prądu
                pMotorController->changeTorqueControlType(new_torque_mode);
            }

            pMotorController->changeControlMode(new_motion_mode);

            Serial.printf("CAN CMD from Dev %d: Set Control Mode -> %d\n", Device, control_mode);
        }
    }
}


void RxFromCAN::Received_SetLimits(const int Dev, float v_lim, float c_lim) {
    if (pMotorController) { pMotorController->setVelocityLimit(v_lim); pMotorController->setCurrentLimit(c_lim);
    LOG("CAN CMD from Dev %d: Set Limits -> Current: %.2f, Velocity: %.2f\n", Dev, c_lim, v_lim);}
}

void RxFromCAN::Received_SetPosGain(const int Device, float pos_p) {
    if (pMotorController) {
        pMotorController->setPosPID(pos_p);
        LOG("CAN CMD from Dev %d: Set Position PID Gains -> P: %.2f\n", Device, pos_p);
    }
}

void RxFromCAN::Received_SetVelGains(const int Device, float vel_p, float vel_i)  {
    if (pMotorController) {
        pMotorController->setVelPID(vel_p, vel_i, 0);
        LOG("CAN CMD from Dev %d: Set Velocity PID Gains -> P: %.2f, I: %.2f\n", Device, vel_p, vel_i);
    }
}

void RxFromCAN::Received_Reboot(const int Device) {
    if (pMotorController) {
        LOG("\nCAN CMD from Dev %d: Rebooting...\n", Device);
        delay(100);
        pMotorController->reboot();
    }
}

// void RxFromCAN::Received_SetAxisState(const int Device, Axis_State_t state) {
//     if (pMotorController) {
//         switch(state) {
//             case AXIS_STATE_IDLE:
//                 pMotorController->disable();
//                 break;
//             case AXIS_STATE_CLOSED_LOOP_CONTROL:
//                 pMotorController->enable();
//                 break;
//             case AXIS_STATE_FULL_CALIBRATION_SEQUENCE:
//                 pMotorController->runCalibration();
//                 break;
//             default:
//                 // Inne stany ignorujemy na razie
//                 LOG("CAN CMD from Dev %d: Ignored Set Axis State -> %d (unsupported state)\n", Device, state);
//                 return;
//         }

//     }
// }