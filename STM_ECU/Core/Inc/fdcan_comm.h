#ifndef FDCAN_COMM_H
#define FDCAN_COMM_H

#include "stm32g4xx_hal.h"

// Typy wiadomości
typedef enum {
    MSG_ESC_CMD   = 0x200,   // przykładowy ID do wysyłania komend do ESC
    MSG_ESC_STAT  = 0x201,   // przykładowy ID z odpowiedzi statusu
    MSG_ESC_SPEED = 0x300 	 // zadawanie prędkości
} CAN_MessageID_t;

// Struktura dla ramki
typedef struct {
    uint16_t id;     // identyfikator CAN
    uint8_t  len;    // długość danych
    uint8_t  data[64]; // ramka CAN FD może mieć do 64 bajtów
} CAN_Message_t;

typedef enum {
	REULATOR_OPEN_LOOP = 0x01,
	REGULATOR_VELOCITY = 0x02,
	REGULATOR_TORQUE =   0x03
} RegulatorType_t;

void FDCAN_Comm_Init(FDCAN_HandleTypeDef *hfdcan);

HAL_StatusTypeDef FDCAN_Comm_Send(FDCAN_HandleTypeDef *hfdcan, CAN_Message_t *msg);

void FDCAN_Comm_RxCallback(FDCAN_HandleTypeDef *hfdcan, CAN_Message_t *rxMsg);

void BuildMessage_SetRegulator(CAN_Message_t *msg, RegulatorType_t regulator);

void BuildMessage_SetSpeed(CAN_Message_t *msg, uint16_t rpm);

#endif
