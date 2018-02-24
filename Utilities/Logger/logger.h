#pragma once

#include "stm32f7xx_hal.h"

void logger_init(void);
void logger_deinit(void);

void logger_send(uint8_t *data);
