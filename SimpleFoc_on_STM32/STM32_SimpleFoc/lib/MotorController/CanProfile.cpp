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

    // RTR frames
    if(rxHeader.RxFrameType == CAN_REMOTE_FRAME){
        switch (Command)
        {
            // To nie jest RTR
            // case ENCODER_ESTIMATES: {
            //     pRxCommands->Received_GetEncoderEstimates(Device);
            //     break;
            // }

            // case GET_ENCODER_COUNT: {
            //     pRxCommands->Received_GetEncoderCount(Device);
            //     break;
            // }

            case GET_BUS_VOLTAGE_CURRENT: {
                pRxCommands->Received_GetBusVoltageCurrent(Device);
                break;
            }

            case GET_IQ: {
                pRxCommands->Received_GetIQ(Device);
                break;
            }

            default:
                LOG("CAN RTR from Dev %d: Ignored (unknown command) %d\n", Device, Command);
                break;
        }
    }
    // DATA FRAMES
    else{
        switch(Command)
            {
                case ODRIVE_HEARTBEAT_MESSAGE: break;

                case SET_AXIS_NODE_ID: break;

                case SET_AXIS_REQUESTED_STATE: break;

                case ENCODER_ESTIMATES: break;

                case SET_CONTROLLER_MODES: {
                    Control_Mode_t motion_control_mode;
                    Control_Mode_t torque_control_mode;
                    memcpy(&motion_control_mode, rxData, sizeof(int32_t));
                    memcpy(&torque_control_mode, rxData + 4, sizeof(int32_t));
                    pRxCommands->Received_SetControllerModes(Device, motion_control_mode, torque_control_mode);
                    break;
                }

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

                case SET_LIMITS: {
                    float velocity_limit, current_limit;
                    memcpy(&velocity_limit, rxData, sizeof(float));
                    memcpy(&current_limit, rxData + 4, sizeof(float));
                    pRxCommands->Received_SetLimits(Device, velocity_limit, current_limit);
                    break;
                }

                case GET_IQ: break;

                case REBOOT: {
                    pRxCommands->Received_Reboot(Device);
                    break;
                }

                case GET_BUS_VOLTAGE_CURRENT: break;

                case CLEAR_ERRORS: break;

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

                default:
                    LOG("CAN CMD from Dev %d: Ignored (unknown command) %d\n", Device, Command);
                    break;
            } 
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
        pMotorController->setTorqueFF(tff);
    }
}
void RxFromCAN::Received_SetInputTorque(const int Dev, float torque) {
    if (pMotorController && pMotorController->getMotionControlType() == MotionControlType::torque) {
        pMotorController->setTarget(torque);
    }
}

// TODO: Nie działa zmiana trybu z Torque do Velocity, silnik wariuje. Trzeba rozkminić resetowanie PID.
void RxFromCAN::Received_SetControllerModes(const int Device, Control_Mode_t motion_control_mode, Control_Mode_t torque_control_mode) {
    if (pMotorController) {

        MotionControlType new_motion_mode;
        TorqueControlType new_torque_mode;
        bool motion_mode_changed = true;
        bool torque_mode_to_set = false;

        switch(motion_control_mode) {
            case CONTROL_MODE_TORQUE: 
                new_motion_mode = MotionControlType::torque; 
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

        switch(torque_control_mode) {
            case TORQUE_CONTROL_MODE_VOLTAGE: 
                new_torque_mode = TorqueControlType::voltage; 
                break;
            case TORQUE_CONTROL_MODE_DC_CURRENT: 
                new_torque_mode = TorqueControlType::dc_current; 
                break;
            case TORQUE_CONTROL_MODE_FOC_CURRENT: 
                new_torque_mode = TorqueControlType::foc_current; 
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
            Serial.printf("CAN CMD from Dev %d: Set Motion Control Mode -> %d, Torque Control Mode -> %d\n", Device, motion_control_mode, torque_control_mode);
        }
    }
}

void RxFromCAN::Received_SetLimits(const int Dev, float v_lim, float c_lim) {
    if (pMotorController) { pMotorController->setVelocityLimit(v_lim); pMotorController->setCurrentLimit(c_lim);
    LOG("CAN CMD from Dev %d: Set Limits -> Velocity: %.2f, Current: %.2f\n", Dev, v_lim, c_lim);}
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

void RxFromCAN::Received_GetBusVoltageCurrent(const int Device) {
    if (pMotorController && pCanBus) {
        float busVoltage = pMotorController->getBusVoltage();
        float busCurrent = pMotorController->getBusCurrent();

        int can_id = PP_MAKE_CAN_ID(Device, GET_BUS_VOLTAGE_CURRENT);
        pCanBus->CANSendFloat(busVoltage, busCurrent, can_id);
    }
}

void RxFromCAN::Received_GetIQ(const int Device) {
    if (pMotorController && pCanBus) {
        float iq = pMotorController->getI_q();

        int can_id = PP_MAKE_CAN_ID(Device, GET_IQ);
        pCanBus->CANSendFloat(iq, 0, can_id);
    }
}

void RxFromCAN::SendHeartbeat()
{

}

void RxFromCAN::SendEncoderEstimates() 
{
    float encPos = pMotorController->getShaftAngle();
    float encVel = pMotorController->getShaftVelocity();

    int can_id = PP_MAKE_CAN_ID(AppCanConfig.MyNodeId, ENCODER_ESTIMATES);
    pCanBus->CANSendFloat(encPos, encVel, can_id);
}


// void RxFromCAN::Received_GetEncoderEstimates(const int Device) {
//     if (pMotorController && pCanBus) {
//         float encPos = pMotorController->getShaftAngle();
//         float encVel = pMotorController->getShaftVelocity();
//         int response_can_id = PP_MAKE_CAN_ID(Device, GET_ENCODER_COUNT);
//         pCanBus->CANSendFloat(encPos, encVel, response_can_id);

//         LOG("CAN RTR Response: Sent Encoder Estimates for Dev %d\n", Device);
//     }
// }

// void RxFromCAN::Received_GetEncoderCount(const int Device) {
//     if (pMotorController && pCanBus) {
        
//         // --- WYSYŁANIE ODPOWIEDZI ---
//         // Zbuduj ID ramki odpowiedzi
//         int response_can_id = PP_MAKE_CAN_ID(pCanBus->getNodeId(), GET_ENCODER_COUNT);

//         // Wyślij dwie 32-bitowe liczby całkowite
//         pCanBus->CANSendInt(shadow_count, cpr_count, response_can_id);
        
//         // Logowanie do debugowania
//         LOG("CAN RTR Response for GET_ENCODER_COUNT: Shadow=%ld, CPR_Count=%ld\n", shadow_count, cpr_count);
//     }
// }

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

