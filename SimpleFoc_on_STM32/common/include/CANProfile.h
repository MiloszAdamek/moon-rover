// CANProfile.h

#pragma once
#include "SimpleCAN.h"
#include "MotorController.hpp"
#include "OdriveProtocol.h"

#define PP_MAKE_CAN_ID(Device, Message)     ((Device<<4) | Message) 
#define PP_GET_MESSAGE_ID(CanID)            (CanID & 0xf)
#define PP_GET_DEVICE_ID(CanID)             (CanID>>4)      // Max ID = 127!

class MotorControllerNotificationsFromCAN
{
    public:
        virtual ~MotorControllerNotificationsFromCAN() {}

        // Sterowanie ruchem
        virtual void Received_SetInputValue(const int Device, float target, float limit) = 0; // Uniwersalna metoda do ustawiania wartości wejściowej (prędkość, pozycja, moment)
        
        // // TODO:
        // virtual void Received_SetInputPos(const int Device, float pos, int16_t vel_ff, int16_t torque_ff) = 0;
        // virtual void Received_SetInputVel(const int Device, float vel, float torque_ff) = 0;
        // virtual void Received_SetInputTorque(const int Device, float torque) = 0;

        // Konfiguracja
        virtual void Received_SetControllerModes(const int Device, Control_Mode_t control_mode, Input_Mode_t input_mode) = 0;
        virtual void Received_SetLimits(const int Device, float current_limit, float velocity_limit) = 0;
        virtual void Received_SetPosGain(const int Device, float pos_p) = 0;
        virtual void Received_SetVelGains(const int Device, float vel_p, float vel_i) = 0;

        // Komendy systemowe

        // TODO:
        // virtual void Received_SetAxisState(const int Device, Axis_State_t state) = 0;
        // virtual void Received_ClearErrors(const int Device) = 0;
        virtual void Received_Reboot(const int Device) = 0;

    };

class CANMotorController : public SimpleCANProfile
{
    public:
        CANMotorController(SimpleCan* pCan, MotorControllerNotificationsFromCAN* _pRxCommands, int nodeId) 
            : SimpleCANProfile(pCan)
        {
            pRxCommands = _pRxCommands;
            this->nodeId = nodeId;
        }
        void canInit();
        void HandleCanMessage(const SimpleCanRxHeader rxHeader, const uint8_t *rxData) override;

        void setNodeId(int id) { this->nodeId = id; }
        int getNodeId() const { return this->nodeId; }

    private:
        MotorControllerNotificationsFromCAN* pRxCommands;
        int nodeId;
};

class RxFromCAN : public MotorControllerNotificationsFromCAN
{
public:
    RxFromCAN(MotorController* controller) : pMotorController(controller) {};
    
        // Sterowanie ruchem
        void Received_SetInputValue(const int Device, float target, float limit) override; // Uniwersalna metoda do ustawiania wartości wejściowej (prędkość, pozycja, moment)

        // // TODO:
        // virtual void Received_SetInputPos(const int Device, float pos, int16_t vel_ff, int16_t torque_ff) = 0;
        // virtual void Received_SetInputVel(const int Device, float vel, float torque_ff) = 0;
        // virtual void Received_SetInputTorque(const int Device, float torque) = 0;

        // Konfiguracja
       void Received_SetControllerModes(const int Device, Control_Mode_t control_mode, Input_Mode_t input_mode) override;
       void Received_SetLimits(const int Device, float current_limit, float velocity_limit) override;

        // TODO:
        void Received_SetPosGain(const int Device, float pos_p) override;
        void Received_SetVelGains(const int Device, float vel_p, float vel_i) override;

        // Komendy systemowe

        // TODO:
        // void Received_SetAxisState(const int Device, Axis_State_t state) override;
        // void Received_ClearErrors(const int Device) override;
        void Received_Reboot(const int Device) override;

private:
    MotorController* pMotorController;
};
