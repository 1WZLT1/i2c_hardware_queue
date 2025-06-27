#ifndef __i2c_e_h
#define __i2c_e_h

#include <i2c.h>
#include <stdint.h>
#include <string.h>

#include "rm3100.h"


typedef struct
{
	uint16_t 			dev_addr;		//高位地址
	uint16_t			mem_addr_size;	//内部地址所占字节数可选取 I2C_MEMADD_SIZE_8BIT 或者 I2C_MEMADD_SIZE_16BIT
}Sensor_I2C_Type;

enum
{
	SensorFunction_Idle,
	SensorFunction_Ready,
	SensorFunction_Sending,
	SensorFunction_Suspended,
};

enum
{
	SensorFunction_InterfaceUndefined,
	SensorFunction_InterfaceSPI,
	SensorFunction_InterfaceI2C,
};

enum
{
	Sensor_Mode_Read,
	Sensor_Mode_Write,
	Sensor_Mode_Cmd,
};

void sensorInit(void);

#endif

