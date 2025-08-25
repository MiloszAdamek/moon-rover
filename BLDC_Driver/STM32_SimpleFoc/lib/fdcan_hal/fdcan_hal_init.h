// #ifndef FDCAN_HAL_INIT_H
// #define FDCAN_HAL_INIT_H

// // Dołączamy główny plik HAL, aby mieć dostęp do typów FDCAN_...
// #ifdef __cplusplus
// extern "C" {
// #endif
// #include "stm32g4xx_hal.h"
// #ifdef __cplusplus
// }
// #endif

// /* 
//  * Deklaracja 'extern' mówi kompilatorowi:
//  * "Ta zmienna istnieje, ale jest zdefiniowana w innym pliku.
//  * Linker ją znajdzie."
//  * Dzięki temu main.cpp może z niej korzystać.
// */
// extern FDCAN_HandleTypeDef hfdcan1;

// /**
//  * @brief  Główna funkcja inicjalizująca i uruchamiająca FDCAN.
//  *         Wywołuje wszystkie potrzebne funkcje HAL w odpowiedniej kolejności.
//  * @retval None
//  */
// void FDCAN_Init(void);


// /* 
//  * Funkcja Error_Handler również musi być widoczna globalnie,
//  * ponieważ jest wywoływana przez biblioteki HAL.
// */
// void Error_Handler(void);

// #endif // FDCAN_HAL_INIT_H