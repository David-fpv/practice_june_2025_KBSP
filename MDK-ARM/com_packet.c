#include "com_packet.h"

/**
  * @brief Remove some symbol in array
  * @param buffer: array
	* @param lenght_buffer: count elements in buffer	
	* @param number: number of position
  * @retval uint8_t value: OK(0) or ERROR(1)
  */
uint8_t remove_symbol(uint8_t buffer[], uint16_t lenght_buffer, uint16_t number)
{
	if (number > lenght_buffer)
	{
		return ERROR;
	}
	
	for (uint16_t i = number; i < lenght_buffer-1; i++)
	{
		buffer[i] = buffer[i+1];
	}
	buffer[lenght_buffer-1] = 0;
	
	return OK;
}


/**
  * @brief Add some symbol in array
  * @param buffer: array
	* @param lenght_buffer: count elements in buffer
	* @param max_buffer_length: max count elements in buffer	
	* @param number: number of position
  * @retval uint8_t value: OK(0) or ERROR(1)
  */
uint8_t add_symbol(uint8_t buffer[], uint16_t lenght_buffer, uint16_t max_buffer_length, uint16_t number, uint8_t symbol)
{
	if (number > lenght_buffer && lenght_buffer >= max_buffer_length)
	{
		return ERROR;
	}
	
	for (uint16_t i = lenght_buffer; i > number; i--)
	{
		buffer[i] = buffer[i-1];
	}
	buffer[number] = symbol;
	
	return OK;
}


/**
  * @brief Delete 0xA6 after 0xAE in array
  * @param buffer: array
	* @param lenght_buffer: count elements in buffer
	* @retval uint16_t: length array after function or 0 as error
  */
uint8_t remove_security_symbols(uint8_t buffer[], uint16_t lenght_buffer)
{
	for (uint16_t i = 0; i < lenght_buffer-1; i++)
	{
		if (buffer[i] == 0xAE && buffer[i+1] == 0xA6)
		{
			remove_symbol(buffer, lenght_buffer, i+1);
			lenght_buffer--;
		}
	}
	
	return lenght_buffer;
}


/**
  * @brief Add 0xA6 after 0xAE in array, except the begining and ending package
  * @param buffer: array
	* @param lenght_buffer: count elements in buffer
	* @param max_buffer_length: max count elements in buffer
	* @retval uint16_t: length array after function or 0 as error
  */
uint8_t add_security_symbols(uint8_t buffer[], uint16_t lenght_buffer, uint16_t max_buffer_lenght)
{
	for (uint16_t i = 0; i < lenght_buffer-1; i++)
	{
		if (buffer[i] == 0xAE && (buffer[i+1] != 0x55 && buffer[i+1] != 0x56))
		{
			if (add_symbol(buffer, lenght_buffer, max_buffer_lenght, i+1, 0xA6) == OK)
			{
				lenght_buffer++;
			} else
			{
				return 0;
			}
		}
	}
	
	return lenght_buffer;
}


/**
  * @brief Fills in the packet structure from the buffer
  * @param buffer: array
	* @param lenght_buffer: count elements in buffer
	* @param struct data_package* package: a pointer to the structure of the package object
	* @retval uint8_t value: OK(0) or ERROR(1)
  */
uint8_t buffer_to_package(uint8_t buffer[], uint16_t lenght_buffer, struct data_package* package)
{
	if(lenght_buffer < 10)
	{
		return ERROR;
	}
	if(buffer[0] != 0xAE || buffer[1] != 0x55)
	{
		return ERROR;
	}
	if(buffer[lenght_buffer-2] != 0xAE || buffer[lenght_buffer-1] != 0x56)
	{
		return ERROR;
	}
	
	package->object_code = buffer[2];
	package->operation_code = buffer[3];
	package->data_length = buffer[4] << 8 | buffer[5];
	package->checksum = buffer[6] << 8 | buffer[7];
	
	for (uint16_t i = 0; i < package->data_length; i++)
	{
		package->data[i] = buffer[i+8];
	}
	
	return OK;
}


/**
  * @brief Fills the buffer with a packet from the structure
  * @param buffer: array
	* @param max_buffer_lenght: max count elements in buffer
	* @param struct data_package* package: a pointer to the structure of the package object
	* @retval uint16_t: length array after function or 0 as error
  */
uint16_t package_to_buffer(uint8_t buffer[], uint16_t max_buffer_lenght, struct data_package* package)
{
	if (package->data_length + 10 > max_buffer_lenght)
	{
		return 0;
	}
	
	memset(buffer, 0, sizeof(buffer[0])*max_buffer_lenght);
	
	// package start marker
	buffer[0] = 0xAE;
	buffer[1] = 0x55;
	
	buffer[2] = package->object_code;
	buffer[3] = package->operation_code;
	
	buffer[4] = package->data_length >> 8;
	buffer[5] = package->data_length & 255;
	
	buffer[6] = package->checksum >> 8;
	buffer[7] = package->checksum & 255;
	
	for(uint16_t i = 0; i < package->data_length; i++)
	{
			buffer[i+8] = package->data[i];		
	}
	
	// package end marker
	buffer[package->data_length-1 + 9] = 0xAE;
	buffer[package->data_length-1 + 10] = 0x56;
	
	return package->data_length + 10;
}