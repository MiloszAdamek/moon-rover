// MC_CANProfile.h

#pragma once
#include "SimpleCAN.h"
#include "MotorController.hpp"
#include "ODriverProtocol.h"

#define PP_MAKE_CAN_ID(Device, Message)     ((Device<<4) | Message) 
#define PP_GET_MESSAGE_ID(CanID)            (CanID & 0xf)
#define PP_GET_DEVICE_ID(CanID)             (CanID>>4)      // Max ID = 127!

class MotorControllerNotificationsFromCAN
{
    public:
        virtual ~MotorControllerNotificationsFromCAN() {}
        virtual void Received_SetInputVel(const int Device, float velocity) = 0;
        virtual void Received_SetInputPos(const int Device, float position) = 0;
        virtual void Received_SetInputTorque(const int Device, float torque) = 0;
        virtual void Received_SetControllerModes(const int Device, Control_Mode_t control_mode, Input_Mode_t input_mode) = 0;
        // TODO: Dodaj więcej metod w miarę potrzeb, np. dla SetInputPos
    };

class CANMotorController : public SimpleCANProfile
{
    public:
        CANMotorController(SimpleCan* pCan, MotorControllerNotificationsFromCAN* _pRxCommands) 
            : SimpleCANProfile(pCan)
        {
            pRxCommands = _pRxCommands;
        }
        void canInit();
        void HandleCanMessage(const SimpleCanRxHeader rxHeader, const uint8_t *rxData) override;

    private:
        MotorControllerNotificationsFromCAN* pRxCommands;
};

class RxFromCAN : public MotorControllerNotificationsFromCAN
{
public:
    RxFromCAN(MotorController* controller) : pMotorController(controller) {};
    
    void Received_SetInputVel(const int Device, float velocity) override;
    void Received_SetInputPos(const int Device, float position) override;
    void Received_SetInputTorque(const int Device, float torque) override;
    void Received_SetControllerModes(const int Device, Control_Mode_t control_mode, Input_Mode_t input_mode) override;

private:
    MotorController* pMotorController;
};
