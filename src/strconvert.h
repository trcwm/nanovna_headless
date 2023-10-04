#pragma once

#include <stdint.h>

// Rewrite universal standart str to value functions to more compact
//
// Convert string to int32
int32_t my_atoi(const char *p);

// Convert string to uint32
//  0x - for hex radix
//  0o - for oct radix
//  0b - for bin radix
//  default dec radix
uint32_t my_atoui(const char *p);

double my_atof(const char *p);
