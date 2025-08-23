#pragma once

#include "Arduino.h"
#include "PeripheralPins.h"

//*** UART ***
#ifdef HAL_UART_MODULE_ENABLED
extern const PinMap PinMap_UART_TX[];
extern const PinMap PinMap_UART_RX[];
#endif

//*** SPI ***
#ifdef HAL_SPI_MODULE_ENABLED
extern const PinMap PinMap_SPI_MOSI[];
extern const PinMap PinMap_SPI_MISO[];
extern const PinMap PinMap_SPI_SCLK[];
extern const PinMap PinMap_SPI_SSEL[];
#endif