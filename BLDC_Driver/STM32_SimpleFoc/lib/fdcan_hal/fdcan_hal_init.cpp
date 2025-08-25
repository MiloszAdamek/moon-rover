// #include "fdcan_hal_init.h"
// #include <Arduino.h> // Dołączamy dla funkcji Serial.println()

// // === PRYWATNE FUNKCJE MODUŁU (niewidoczne na zewnątrz) ===

// // Deklarujemy je jako 'static', aby były widoczne tylko w tym pliku.
// static void MX_FDCAN1_Init(void);
// static void Configure_CAN_Filter(void);

// // === DEFINICJA ZMIENNEJ GLOBALNEJ ===

// // To jest właściwa definicja uchwytu FDCAN. 
// // Istnieje tylko w tym jednym miejscu.
// FDCAN_HandleTypeDef hfdcan1;


// // === IMPLEMENTACJA FUNKCJI PUBLICZNYCH ===

// void FDCAN_Init(void) {
//   Serial.println("Initializing FDCAN peripheral...");
//   MX_FDCAN1_Init();

//   Serial.println("Configuring FDCAN filter...");
//   Configure_CAN_Filter();

//   Serial.println("Starting FDCAN...");
//   if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
//     Serial.println("ERROR: Failed to start FDCAN!");
//     Error_Handler();
//   }
  
//   // Aktywuj powiadomienia o nowych wiadomościach w kolejce RX FIFO 0
//   if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
//     Serial.println("ERROR: Failed to activate FDCAN notifications!");
//     Error_Handler();
//   }

//   Serial.println("FDCAN started successfully!");
// }

// void Error_Handler(void) {
//   Serial.println("!!! HAL ERROR !!! - System halted.");
//   __disable_irq();
//   while (1) {
//     // Zasygnalizuj błąd, np. migając diodą LED
//     digitalToggle(LED_BUILTIN); // Zakładając, że LED jest zdefiniowany
//     delay(100);
//   }
// }

// // === IMPLEMENTACJA FUNKCJI PRYWATNYCH I CALLBACKÓW HAL ===

// static void Configure_CAN_Filter(void) {
//   FDCAN_FilterTypeDef sFilterConfig;

//   sFilterConfig.IdType = FDCAN_STANDARD_ID;
//   sFilterConfig.FilterIndex = 0;
//   sFilterConfig.FilterType = FDCAN_FILTER_MASK;
//   sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RX_FIFO0;
//   sFilterConfig.FilterID1 = 0x000;
//   sFilterConfig.FilterID2 = 0x000; // Maska 0x000 oznacza "akceptuj wszystko"

//   if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
//     Serial.println("ERROR: Filter configuration failed!");
//     Error_Handler();
//   }
// }

// static void MX_FDCAN1_Init(void) {
//   hfdcan1.Instance = FDCAN1;
//   hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
//   hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
//   hfdcan1.Init.AutoRetransmission = DISABLE;
//   hfdcan1.Init.TransmitPause = DISABLE;
//   hfdcan1.Init.ProtocolException = DISABLE;
//   hfdcan1.Init.NominalPrescaler = 10;
//   hfdcan1.Init.NominalSyncJumpWidth = 1;
//   hfdcan1.Init.NominalTimeSeg1 = 12;
//   hfdcan1.Init.NominalTimeSeg2 = 4;
//   hfdcan1.Init.DataPrescaler = 1;
//   hfdcan1.Init.DataSyncJumpWidth = 1;
//   hfdcan1.Init.DataTimeSeg1 = 1;
//   hfdcan1.Init.DataTimeSeg2 = 1;
//   hfdcan1.Init.MessageRAMOffset = 0;
//   hfdcan1.Init.StdFiltersNbr = 1;
//   hfdcan1.Init.ExtFiltersNbr = 0;
//   hfdcan1.Init.RxFifo0ElmtsNbr = 8; // Zwiększmy trochę bufor
//   hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
//   hfdcan1.Init.RxFifo1ElmtsNbr = 0;
//   hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
//   hfdcan1.Init.RxBuffersNbr = 0;
//   hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
//   hfdcan1.Init.TxEventsNbr = 0;
//   hfdcan1.Init.TxBuffersNbr = 0;
//   hfdcan1.Init.TxFifoQueueElmtsNbr = 8; // Zwiększmy trochę bufor
//   hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
//   hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
//   if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
//   {
//     Error_Handler();
//   }
// }

// /*
//  * HAL wymaga zdefiniowania tej funkcji. Jest ona wywoływana z wnętrza HAL_FDCAN_Init.
//  * Musi być zadeklarowana jako extern "C", aby linker poprawnie ją połączył.
// */
// extern "C" void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* hfdcan) {
//   GPIO_InitTypeDef GPIO_InitStruct = {0};
//   RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
//   if(hfdcan->Instance==FDCAN1)
//   {
//     PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
//     PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
//     if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
//     {
//       Error_Handler();
//     }

//     __HAL_RCC_FDCAN_CLK_ENABLE();
//     __HAL_RCC_GPIOB_CLK_ENABLE();

//     GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
//     GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//     GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
//     HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//   }
// }