// MC_CANProfile.h

#pragma once
#include "SimpleCAN.h"
#include "MotorController.hpp"

#define CANID_MD_GET         1       // Message is "Get telemetry values"
#define CANID_MD_SET         2       // Message is "Set telemetry values"
#define CANID_MD_MODE        3       // Message is "Mode"
#define CANID_MD_RTRINT      4       // Request confirmation

#define PP_MAKE_CAN_ID(Device, Message)     ((Device<<4) | Message) 
#define PP_GET_MESSAGE_ID(CanID)            (CanID & 0xf)
#define PP_GET_DEVICE_ID(CanID)             (CanID>>4)      // Max ID = 127!

class MotorControllerNotificationsFromCAN
{
    public:
        virtual void ReceivedSet(const int Device, const char* pText)=0;
        virtual void ReceivedGet(const int Device, const char* pText)=0;
        virtual void ReceivedMode(const int Device, const char* pText)=0;
        virtual void ReceivedRequestInt(const int Device)=0;
        virtual void ReceivedInt(const int Device, int Val)=0;
};

class CANMotorController : public SimpleCANProfile
{
    public:
        CANMotorController(SimpleCan* pCan, MotorControllerNotificationsFromCAN* _pRxCommands) 
            : SimpleCANProfile(pCan)
        {
            pRxCommands = _pRxCommands;
        }
        void CAN_Init();
        void CANRequestInt(int DeviceID);
        void HandleCanMessage(const SimpleCanRxHeader rxHeader, const uint8_t *rxData) override;

    private:
        MotorControllerNotificationsFromCAN* pRxCommands;
};

class RxFromCAN : public MotorControllerNotificationsFromCAN
{
    public:
		RxFromCAN::RxFromCAN(MotorController* controller) 
            : ReceivedID(-1), RTR(false), ReceivedFloatVal(1.0f), pMotorController(controller) {};
        void ReceivedSet(const int Device, const char* pText);
        void ReceivedGet(const int Device, const char* pText);
        void ReceivedMode(const int Device, const char* pText);
		void ReceivedRequestInt(const int Device);
		void ReceivedInt(const int Device, int Val);

    private:
        MotorController* pMotorController;
		int ReceivedID;		
		bool RTR = true;
		float ReceivedFloatVal;

};
