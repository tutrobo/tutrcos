#pragma once

#include "main.h"

#ifdef HAL_GPIO_MODULE_ENABLED
#include "peripheral/gpio.hpp"
#endif

#ifdef HAL_TIM_MODULE_ENABLED
#include "peripheral/tim.hpp"
#endif

#ifdef HAL_UART_MODULE_ENABLED
#include "peripheral/uart.hpp"
#endif

#ifdef HAL_CAN_MODULE_ENABLED
#include "peripheral/can.hpp"
#endif

#ifdef HAL_FDCAN_MODULE_ENABLED
#include "peripheral/fdcan.hpp"
#endif

#ifdef HAL_I2C_MODULE_ENABLED
#include "peripheral/i2c_master.hpp"
#endif

#ifdef HAL_SPI_MODULE_ENABLED
#include "peripheral/spi.hpp"
#endif