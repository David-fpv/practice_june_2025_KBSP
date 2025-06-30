#pragma once
#include <stdint.h>
#include "string.h"
#include "array_container.h"
#include "stm32f1xx_hal.h"


#define OK 0
#define ERROR 1


typedef struct
{
	uint8_t object_code;
	uint8_t operation_code;
	uint16_t data_length;
	uint16_t checksum;
	uint8_t data[40];
} data_package;


uint8_t remove_symbol(array_container* container, uint16_t number);

uint8_t add_symbol(array_container* container, uint16_t number, uint8_t symbol);

uint8_t remove_security_symbols(array_container* container);

uint8_t add_security_symbols(array_container* container);

uint8_t buffer_to_package(array_container* container, data_package* package);

uint16_t package_to_buffer(array_container* container, data_package* package);

uint8_t isNormalLenghtPackage(data_package* package, uint8_t min_normal_lenght);

