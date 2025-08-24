#pragma once

#define CANID_MD_VELOCITY    1       // Message is "Velocity"
#define CANID_MD_ANGLE       2       // Message is "Angle"
#define CANID_MD_MODE        3       // Message is "Mode"
#define CANID_MD_CONFIRM     4       // Request confirmation

class MotorControllerCAN : public SimpleCANProfile
{
public:
    MotorControllerCAN(SimpleCAN* can) : SimpleCANProfile(can) {}

    void onMessageReceived(const CANMessage& msg) override {
        switch (msg.id) {
            case CANID_MD_VELOCITY:
                handleVelocityMessage(msg);
                break;
            case CANID_MD_ANGLE:
                handleAngleMessage(msg);
                break;
            case CANID_MD_MODE:
                handleModeMessage(msg);
                break;
            case CANID_MD_CONFIRM:
                handleConfirmMessage(msg);
                break;
            default:
                break;
        }
    }

private:
    void handleVelocityMessage(const CANMessage& msg) {
        // Handle velocity message
    }

    void handleAngleMessage(const CANMessage& msg) {
        // Handle angle message
    }

    void handleModeMessage(const CANMessage& msg) {
        // Handle mode message
    }

    void handleConfirmMessage(const CANMessage& msg) {
        // Handle confirmation message
    }
};