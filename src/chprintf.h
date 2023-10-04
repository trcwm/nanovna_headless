#pragma once
#include "ch.h"
#include "hal.h"

int chvprintf(BaseSequentialStream *chp, const char *fmt, va_list ap);
