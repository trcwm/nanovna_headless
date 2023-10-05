#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "ch.h"
#include "hal.h"

extern BaseSequentialStream *gs_usb_stream;
void submitCmdByte(uint8_t c);
void cmdBackgroundThread(void);
bool executeCmd(const uint8_t *data, uint8_t bytes);
