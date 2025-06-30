#pragma once
#include <stdint.h>

#define MAX_ARRAY_LENGHT 50

typedef struct
{
	uint8_t array[MAX_ARRAY_LENGHT];
	uint16_t length;
	uint16_t max_length;
}  array_container;