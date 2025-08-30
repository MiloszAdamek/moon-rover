// MC_CANProfile.cpp

#include "MC_CANProfile.h"

void CANMotorController::CAN_Init(){

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

void  CANMotorController::CANRequestInt(int DeviceID)
{
    Can1->RequestMessage(2, PP_MAKE_CAN_ID(DeviceID, CANID_MD_RTRINT));            
}

void  CANMotorController::HandleCanMessage(const SimpleCanRxHeader rxHeader, const uint8_t *rxData)
{   

    // Serial.println("@");

    #define MAX_STRLEN  16

    char Str[MAX_STRLEN];
    float Val=0;
    int Device = PP_GET_DEVICE_ID(rxHeader.Identifier);
    switch(PP_GET_MESSAGE_ID(rxHeader.Identifier))
    {
        case CANID_MD_GET:
            this->CANGetString(rxData, Str, min(MAX_STRLEN-1, (int)rxHeader.DataLength));
            this->pRxCommands->ReceivedGet(Device, Str);
            break;
        case CANID_MD_SET:
            this->CANGetString(rxData, Str, min(MAX_STRLEN, (int)rxHeader.DataLength));
            this->pRxCommands->ReceivedSet(Device, Str);
            break;
        case CANID_MD_MODE:
            this->CANGetString(rxData, Str, min(MAX_STRLEN, (int)rxHeader.DataLength));
            this->pRxCommands->ReceivedMode(Device, Str);
            break;
        case CANID_MD_RTRINT:
            if (rxHeader.RxFrameType==CAN_REMOTE_FRAME)
                this->pRxCommands->ReceivedRequestInt(Device);
            else
            {
                int ValI = this->CANGetInt(rxData);
                this->pRxCommands->ReceivedInt(Device, ValI);
            }
            break;
        default:
            Serial.printf("y:0x%x DLC=0x%x ", rxHeader.Identifier, rxHeader.DataLength);
    } 
}

void RxFromCAN::ReceivedSet(const int Device, const char* pText) {

    if (pText[0] == 'v' || pText[0] == 'V') {
        // atof (ascii to float) konwertuje string na liczbę
        float target_velocity = atof(pText + 1); // +1 aby pominąć literę 'v'
        
        if (pMotorController) {
            pMotorController->setTarget(target_velocity);
        }
        
        Serial.printf("CAN Command: Set velocity to %.2f rad/s\n", target_velocity);
    } 
    // Tutaj można dodać obsługę innych komend, np. "p 1.57" dla pozycji
}

void RxFromCAN::ReceivedGet(const int Device, const char* pText)
{
    Serial.printf("Received: %s from 0x%x\n", pText, Device);
    ReceivedID = CANID_MD_GET;
};

void RxFromCAN::ReceivedMode(const int Device, const char* pText) {
    // Komenda "Mode" może służyć do zmiany trybu pracy
    // np. "velocity", "torque", "angle"
    if (pMotorController) {
        if (strcmp(pText, "velocity") == 0) {
            pMotorController->setControlMode(MotionControlType::velocity);
            Serial.println("CAN Command: Switched to velocity mode");
        } else if (strcmp(pText, "torque") == 0) {
            pMotorController->setControlMode(MotionControlType::torque);
            Serial.println("CAN Command: Switched to torque mode");
        }
        // ... itd.
        // UWAGA: Potrzebujesz metody `setControlMode` w MotorController.hpp
    }
}

void RxFromCAN::ReceivedRequestInt(const int Device)
{
    Serial.printf("Received: RTR from 0x%x\n", Device);
    ReceivedID = CANID_MD_RTRINT;
    RTR = true;
};

void RxFromCAN::ReceivedInt(const int Device, int Val)
{
    Serial.printf("Rcvd int: %d from 0x%x\n", Val, Device);
    ReceivedID = CANID_MD_RTRINT;
};