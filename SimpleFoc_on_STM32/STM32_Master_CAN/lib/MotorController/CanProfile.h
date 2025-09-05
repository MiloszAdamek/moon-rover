// CanProfile.h

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
        virtual void Received_SetInputVel(const int Device, float vel) = 0;
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

        // Master-specific functions
        virtual void MasterReceived_I_Q_Currents(const int Device, float i_q_setpoint, float i_q_measured) {} // Optional for master
        virtual void MasterReceived_EncoderEstimates(const int Device, float pos, float vel) {} // Optional for master
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
        int getMasterNodeId() const { return this->masterNodeId; }

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
    void Received_SetInputVel(const int Device, float vel) override;
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

class MasterRxFromCAN : public MotorControllerNotificationsFromCAN
{
    public:
        // --- Implementacje dla komend (puste, ale wymagane przez interfejs) ---
        // Dodajemy puste ciała funkcji {}, aby spełnić "kontrakt" z klasą bazową.
        
        void Received_SetInputPos(const int Device, float pos, int16_t vel_ff, int16_t torque_ff) override {}
        void Received_SetInputVel(const int Device, float vel) override {}
        void Received_SetInputTorque(const int Device, float torque) override {}
        void Received_SetControllerModes(const int Device, Control_Mode_t m, Control_Mode_t t) override {}
        void Received_SetLimits(const int Device, float v_lim, float c_lim) override {}
        void Received_SetPosGain(const int Device, float pos_p) override {}
        void Received_SetVelGains(const int Device, float vel_p, float vel_i) override {}
        void Received_Reboot(const int Device) override {}
        void Received_GetBusVoltageCurrent(const int Device) override {}
        void Received_GetIQ(const int Device) override {}

        void MasterReceived_I_Q_Currents(const int Device, float i_q_setpoint, float i_q_measured) {
            LOG("CAN Master received Iq from Dev %d: Iq_set=%.2f, Iq_meas=%.2f\n", Device, i_q_setpoint, i_q_measured);
        }

        void MasterReceived_EncoderEstimates(const int Device, float pos, float vel) {
            LOG("CAN Master received Encoder Estimates from Dev %d: Pos=%.2f, Vel=%.2f\n", Device, pos, vel);
        }
};