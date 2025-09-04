// CANProfile.h

#pragma once
#include "SimpleCAN.h"
#include "MotorController.hpp"
#include "OdriveProtocol.h"
#include "Config.hpp"

#define PP_MAKE_CAN_ID(Device, Message)     ((Device << 5) | Message) 
#define PP_GET_MESSAGE_ID(CanID)            (CanID & 0x1F) // 0x1F to binarnie 11111 (5 bitów)
#define PP_GET_DEVICE_ID(CanID)             (CanID >> 5)   

class MotorControllerNotificationsFromCAN
{
    public:
        virtual ~MotorControllerNotificationsFromCAN() {}

        // Sterowanie ruchem
        virtual void Received_SetInputPos(const int Device, float pos, int16_t vel_ff, int16_t torque_ff) = 0;
        virtual void Received_SetInputVel(const int Device, float vel, float torque_ff) = 0;
        virtual void Received_SetInputTorque(const int Device, float torque) = 0;

        // Konfiguracja
        virtual void Received_SetControllerModes(const int Device, Control_Mode_t motion_control_mode, Control_Mode_t torque_control_mode) = 0;
        virtual void Received_SetLimits(const int Device, float velocity_limit, float current_limit) = 0;
        virtual void Received_SetPosGain(const int Device, float pos_p) = 0;
        virtual void Received_SetVelGains(const int Device, float vel_p, float vel_i) = 0;

        // Komendy systemowe
        // virtual void Received_SetAxisState(const int Device, Axis_State_t state) = 0;
        // virtual void Received_ClearErrors(const int Device) = 0;
        virtual void Received_Reboot(const int Device) = 0;

        // Przesyłanie danych
        // virtual void Received_GetEncoderCount(const int Device) = 0;
        virtual void Received_GetBusVoltageCurrent(const int Device) = 0;
        virtual void Received_GetIQ(const int Device) = 0;

};

class CANMotorController : public SimpleCANProfile
{
    public:
        CANMotorController(SimpleCan* pCan, MotorControllerNotificationsFromCAN* _pRxCommands, int myNodeId, int masterNodeId) 
            : SimpleCANProfile(pCan)
        {
            pRxCommands = _pRxCommands;
            this->myNodeId = myNodeId;
            this->masterNodeId = masterNodeId;
        }
        void canInit();
        void HandleCanMessage(const SimpleCanRxHeader rxHeader, const uint8_t *rxData) override;

        void setNodeId(int id) { this->myNodeId = id; }
        int getNodeId() const { return this->myNodeId; }

    private:
        MotorControllerNotificationsFromCAN* pRxCommands;
        int myNodeId;
        int masterNodeId;
};

class RxFromCAN : public MotorControllerNotificationsFromCAN
{
public:
    RxFromCAN(MotorController* controller) : pMotorController(controller) {};
    void linkCanBus(CANMotorController* canBus) { pCanBus = canBus; }

    // Sterowanie ruchem
    void Received_SetInputPos(const int Device, float pos, int16_t vel_ff, int16_t torque_ff) override;
    void Received_SetInputVel(const int Device, float vel, float torque_ff) override;
    void Received_SetInputTorque(const int Device, float torque) override;

    // Konfiguracja
    void Received_SetControllerModes(const int Device, Control_Mode_t motion_control_mode, Control_Mode_t torque_control_mode) override;
    void Received_SetLimits(const int Device, float velocity_limit, float current_limit) override;
    void Received_SetPosGain(const int Device, float pos_p) override;
    void Received_SetVelGains(const int Device, float vel_p, float vel_i) override;

    // Komendy systemowe
    // void Received_SetAxisState(const int Device, Axis_State_t state) override;
    // void Received_ClearErrors(const int Device) override;
    void Received_Reboot(const int Device) override;

    // Przesyłanie danych -> odpowiedź na RTR
    // void Received_GetEncoderCount(const int Device) override;
    void Received_GetBusVoltageCurrent(const int Device) override;
    void Received_GetIQ(const int Device) override;

    // Heartbeat functions
    // void SendHeartbeat();   
    void sendEncoderEstimates();
    void sendIq();

private:
    MotorController* pMotorController;
    CANMotorController* pCanBus = nullptr;
};
