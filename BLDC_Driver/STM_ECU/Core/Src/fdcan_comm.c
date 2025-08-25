#include "fdcan_comm.h"

void FDCAN_Comm_Init(FDCAN_HandleTypeDef *hfdcan)
{
    HAL_FDCAN_Start(hfdcan);

    // Włącz przerwania dla nowej wiadomości w FIFO0
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    // Filtr – odbierz wszystko (na start)
    FDCAN_FilterTypeDef sFilterConfig;
    sFilterConfig.IdType       = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex  = 0;
    sFilterConfig.FilterType   = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1    = 0x000;   // akceptuj od 0x000
    sFilterConfig.FilterID2    = 0x000;   // maska = ignoruj wszystko
    HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig);
}

static uint32_t len_to_dlc(uint8_t len) {
    switch (len) {
        case 0:  return FDCAN_DLC_BYTES_0;
        case 1:  return FDCAN_DLC_BYTES_1;
        case 2:  return FDCAN_DLC_BYTES_2;
        case 3:  return FDCAN_DLC_BYTES_3;
        case 4:  return FDCAN_DLC_BYTES_4;
        case 5:  return FDCAN_DLC_BYTES_5;
        case 6:  return FDCAN_DLC_BYTES_6;
        case 7:  return FDCAN_DLC_BYTES_7;
        case 8:  return FDCAN_DLC_BYTES_8;
        case 12: return FDCAN_DLC_BYTES_12;
        case 16: return FDCAN_DLC_BYTES_16;
        case 20: return FDCAN_DLC_BYTES_20;
        case 24: return FDCAN_DLC_BYTES_24;
        case 32: return FDCAN_DLC_BYTES_32;
        case 48: return FDCAN_DLC_BYTES_48;
        case 64: return FDCAN_DLC_BYTES_64;
        default: return FDCAN_DLC_BYTES_8; // fallback
    }
}

HAL_StatusTypeDef FDCAN_Comm_Send(FDCAN_HandleTypeDef *hfdcan, CAN_Message_t *msg)
{
    FDCAN_TxHeaderTypeDef TxHeader;

    TxHeader.Identifier          = msg->id;
    TxHeader.IdType              = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType         = FDCAN_DATA_FRAME;
    TxHeader.DataLength          = len_to_dlc(msg->len); // albo FDCAN_DLC_BYTES_8 dla stałej
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch       = FDCAN_BRS_OFF;
    TxHeader.FDFormat            = FDCAN_CLASSIC_CAN;   // ustawione na klasyczny tryb
    TxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker       = 0;

    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, msg->data);
}

// Callback z HAL
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
    {
        FDCAN_RxHeaderTypeDef RxHeader;
        static uint8_t RxData[64];
        CAN_Message_t rxMsg;

        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData);

        rxMsg.id = RxHeader.Identifier;
        rxMsg.len = (RxHeader.DataLength >> 16);
        for (uint8_t i = 0; i < rxMsg.len; i++) {
            rxMsg.data[i] = RxData[i];
        }

        // Wywołaj callback użytkownika
        FDCAN_Comm_RxCallback(hfdcan, &rxMsg);
    }
}

// Obsługa callbacku odbioru ramki
__weak void FDCAN_Comm_RxCallback(FDCAN_HandleTypeDef *hfdcan, CAN_Message_t *rxMsg)
{
    // np. wypisz nr ID dla debug
    // printf("CAN RX ID:%03X len:%d\n", rxMsg->id, rxMsg->len);
}

void BuildMessage_SetRegulator(CAN_Message_t *msg, RegulatorType_t regulator){
	msg->id = MSG_ESC_CMD;
	msg->len = 1;
	msg->data[0] = (uint8_t)regulator;
}

void BuildMessage_SetSpeed(CAN_Message_t *msg, uint16_t rpm){
	msg->id = MSG_ESC_SPEED;
	msg->len = 2;
	msg->data[0] = (uint8_t)(rpm & 0xFF);
	msg->data[1] = (uint8_t)(rpm >> 8);
}


