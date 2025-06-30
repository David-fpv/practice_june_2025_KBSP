#include "com_packet.h"


/**
  * @brief Remove some symbol in array
  * @param array_container* container: container that stores an array and its properties
	* @param number: number of position
  * @retval uint8_t value: OK(0) or ERROR(1)
  */
uint8_t remove_symbol(array_container* container, uint16_t number)
{
	if (number > container->length)
	{
		return ERROR;
	}
	
	for (uint16_t i = number; i < container->length-1; i++)
	{
		container->array[i] = container->array[i+1];
	}
	container->array[container->length-1] = 0;
	
	return OK;
}


/**
  * @brief Add some symbol in array
  * @param array_container* container: container that stores an array and its properties
	* @param number: number of position
  * @param uint8_t symbol: the symbol that will be added
  * @retval uint8_t value: OK(0) or ERROR(1)
  */
uint8_t add_symbol(array_container* container, uint16_t number, uint8_t symbol)
{
	if (number > container->length && container->length >= container->max_length)
	{
		return ERROR;
	}
	
	for (uint16_t i = container->length; i > number; i--)
	{
		container->array[i] = container->array[i-1];
	}
	container->array[number] = symbol;
	
	return OK;
}


/**
  * @brief Delete 0xA6 after 0xAE in array
  * @param array_container* container: container that stores an array and its properties
	* @retval uint16_t: length array after function or 0 as error
  */
uint8_t remove_security_symbols(array_container* container)
{
	for (uint16_t i = 0; i < container->length-1; i++)
	{
		if (container->array[i] == 0xAE && container->array[i+1] == 0xA6)
		{
			remove_symbol(container, i+1);
			container->length--;
		}
	}
	
	return container->length;
}


/**
  * @brief Add 0xA6 after 0xAE in array, except the begining and ending package
  * @param array_container* container: container that stores an array and its properties
	* @retval uint16_t: length array after function or 0 as error
  */
uint8_t add_security_symbols(array_container* container)
{
	for (uint16_t i = 0; i < container->length-1; i++)
	{
		if (container->array[i] == 0xAE && (container->array[i+1] != 0x55 && container->array[i+1] != 0x56))
		{
			if (add_symbol(container, i+1, 0xA6) == OK)
			{
				container->length++;
			} else
			{
				return 0;
			}
		}
	}
	
	return container->length;
}


/**
  * @brief Fills in the packet structure from the buffer
  * @param array_container* container: container that stores an array and its properties
	* @param struct data_package* package: a pointer to the structure of the package object
	* @retval uint8_t value: OK(0) or ERROR(1)
  */
uint8_t buffer_to_package(array_container* container, data_package* package)
{
	if(container->length < 10)
	{
		return ERROR;
	}
	if(container->array[0] != 0xAE || container->array[1] != 0x55)
	{
		return ERROR;
	}
	if(container->array[container->length-2] != 0xAE || container->array[container->length-1] != 0x56)
	{
		return ERROR;
	}
	
	package->object_code = container->array[2];
	package->operation_code = container->array[3];
	package->data_length = container->array[4] << 8 | container->array[5];
	package->checksum = container->array[6] << 8 | container->array[7];
	
	for (uint16_t i = 0; i < package->data_length; i++)
	{
		package->data[i] = container->array[i+8];
	}
	
	return OK;
}


/**
  * @brief Fills the buffer with a packet from the structure
  * @param array_container* container: container that stores an array and its properties
	* @param max_buffer_lenght: max count elements in buffer
	* @param data_package* package: a pointer to the structure of the package object
	* @retval uint16_t: length array after function or 0 as error
  */
uint16_t package_to_buffer(array_container* container, data_package* package)
{
	if (package->data_length + 10 > container->max_length)
	{
		return 0;
	}
	
	memset(container->array, 0, sizeof(container->array[0])*container->max_length);
	
	// package start marker
	container->array[0] = 0xAE;
	container->array[1] = 0x55;
	
	container->array[2] = package->object_code;
	container->array[3] = package->operation_code;
	
	container->array[4] = package->data_length >> 8;
	container->array[5] = package->data_length & 255;
	
	container->array[6] = package->checksum >> 8;
	container->array[7] = package->checksum & 255;
	
	for(uint16_t i = 0; i < package->data_length; i++)
	{
			container->array[i+8] = package->data[i];		
	}
	
	// package end marker
	container->array[package->data_length-1 + 9] = 0xAE;
	container->array[package->data_length-1 + 10] = 0x56;
	
	return package->data_length + 10;
}


/**
  * @brief Fills the buffer with a packet from the structure
	* @param struct data_package* package: a pointer to the structure of the package object
	* @param min_normal_length: minimum number of items to be checked
	* @retval uint8_t value: OK(0) or ERROR(1)
  */
uint8_t isNormalLenghtPackage(data_package* package, uint8_t min_normal_lenght)
{
	if (package->data_length < min_normal_lenght)
	{
		return ERROR;
	}
	return OK;
}

