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

// Function used for search substring v in list
// Example need search parameter "center" in "start|stop|center|span|cw" getStringIndex
// return 2 If not found return -1 Used for easy parse command arguments
int get_str_index(char *v, const char *list);