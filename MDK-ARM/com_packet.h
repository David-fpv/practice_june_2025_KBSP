#pragma once
#include <stdint.h>
#include "string.h"

#define OK 0
#define ERROR 1


struct data_package
{
	uint8_t object_code;
	uint8_t operation_code;
	uint16_t data_length;
	uint16_t checksum;
	uint8_t data[40];
};


uint8_t remove_symbol(uint8_t buffer[], uint16_t lenght_buffer, uint16_t number);
uint8_t add_symbol(uint8_t buffer[], uint16_t lenght_buffer, uint16_t max_length, uint16_t number, uint8_t symbol);
uint8_t remove_security_symbols(uint8_t buffer[], uint16_t lenght_buffer);
uint8_t add_security_symbols(uint8_t buffer[], uint16_t lenght_buffer, uint16_t max_buffer_lenght);
uint8_t buffer_to_package(uint8_t buffer[], uint16_t lenght_buffer, struct data_package* package);
uint16_t package_to_buffer(uint8_t buffer[], uint16_t max_buffer_lenght, struct data_package* package);
