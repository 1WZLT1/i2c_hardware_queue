#ifndef __rm3100_h
#define __rm3100_h

#include <stdint.h>

struct SensorFunction;
typedef void (*SensorResultCallBack)(struct SensorFunction *function);

typedef struct Sensor_Hardware_List
{
	int64_t until;
	struct SensorFunction* sending; 
	struct SensorFunction* head;
	struct SensorFunction* tail;
	void* interface;
	uint32_t delay;
	void (*stop)(struct Sensor_Hardware_List*);
	int error_count;
	uint8_t* buffer;
}Sensor_Hardware_List_t;

typedef struct SensorFunction
{
	uint32_t length;
	volatile uint32_t status;
	uint32_t type;//spi i2c
	uint32_t mode;//读/写/命令
	uint32_t delay;
	int32_t id;
	const void *interface; //硬件接口
	Sensor_Hardware_List_t* hardware;
	void *parameter;	   //传感器结构体
	struct SensorFunction *next;
	struct SensorFunction *prev;
	SensorResultCallBack result;   //传感器解析
	int64_t time;
	uint32_t error_count;
	uint8_t buffer[300];
} SensorFunction_t;

typedef struct
{
    SensorFunction_t    Function;
    float               Mags[3];
    uint8_t             MagsValid[3];
    // struct rt_semaphore	Semaphore;
    volatile uint64_t   lastUpdate;
    volatile int        Initialized;
} RM3100_Status_Type;


extern RM3100_Status_Type MS5525;


#endif

