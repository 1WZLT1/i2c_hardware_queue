#include "i2c_e.h"
#define HAS_MS5525

typedef struct SensorCMD
{
	uint8_t  len;
	uint8_t  mode;
	uint16_t delay;
	int32_t  id;
	
	union {
					const uint8_t *buf;
					uint32_t 			 addr;
				};
} SensorCMD_t __attribute__((aligned));//默认4字节对齐


#define DefineSensorCMD_DRead(name, _addr, _len)          \
	__attribute__((unused)) SensorCMD_t name = {            \
		.len = _len,                                          \
		.mode = Sensor_Mode_Read,                             \
		.delay = 0,                                           \
		.id = -1,                                             \
		.addr = _addr};
	
/*定长输入*/		
#define DefineSensorCMD_DWriteFL(name,_addr,_len)   \
	__attribute__((unused)) SensorCMD_t name = {            \
		.len = _len,                                          \
		.mode = Sensor_Mode_Write,                             \
		.delay = 0,                                           \
		.id = -1,                                             \
		.addr = _addr};

		
#define LinkedListTakeFirst(head, tail, p)  \
do{                                         \
    if(head == 0)                           \
    {                                       \
        p = 0;                              \
    }                                       \
    else                                    \
    {                                       \
        p = head;                           \
        head = head->next;                  \
        p->prev = p->next = 0;              \
        if(head == 0)                       \
        {                                   \
            tail = 0;                       \
        }                                   \
    }                                       \
}while(0)

static void Sensor_ErrorHandler_I2C(Sensor_Hardware_List_t* hardware);

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)    __attribute__((alias ("I2C_SendOver")));
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)    __attribute__((alias ("I2C_SendOver")));
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) __attribute__((alias ("I2C_SendOver")));


void Sensor_Send_I2C(SensorFunction_t * function)
{
	Sensor_I2C_Type* i2c_t = (Sensor_I2C_Type*) (function->interface);
	Sensor_Hardware_List_t* hardware = function->hardware;
	
	I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef*)hardware->interface;
	hardware->sending = function;
	function->next = 0;//这一步可能多余
	function->status = SensorFunction_Sending;
	uint32_t addr = *((uint32_t*) function->buffer);
	
	switch (function->mode)
	{
		case Sensor_Mode_Read:
		{
				if(hi2c->hdmarx)
				{
					 HAL_I2C_Mem_Read_DMA(hi2c, i2c_t->dev_addr, addr, i2c_t->mem_addr_size, hardware->buffer, function->length - 4);
				}
				else 
				{
					if(hi2c->Instance->CR1 &= (0x01 << 1))
					{
						HAL_I2C_Mem_Read_IT(hi2c, i2c_t->dev_addr, addr, i2c_t->mem_addr_size, hardware->buffer, function->length - 4);
					}
					else
					{
						HAL_I2C_Mem_Read(hi2c,i2c_t->dev_addr,addr,i2c_t->mem_addr_size, hardware->buffer, function->length - 4 , 0xffff);
					}
				}
		}break;
		
		case Sensor_Mode_Write:
		{
				if(hi2c->hdmatx)
				{
					if(hi2c->hdmatx->State == HAL_DMA_STATE_BUSY)HAL_DMA_Abort(hi2c->hdmatx);
					
					memcpy(hardware->buffer, &function->buffer[4], function->length - 4);
					HAL_I2C_Mem_Write_DMA(hi2c, i2c_t->dev_addr, addr, i2c_t->mem_addr_size, hardware->buffer, function->length - 4);
				}
				else 
				{
					if(hi2c->Instance->CR1 &= (0x01 << 1))
					{
						memcpy(hardware->buffer, &function->buffer[4],function->length - 4);
						HAL_I2C_Mem_Write_IT(hi2c, i2c_t->dev_addr, addr, i2c_t->mem_addr_size, hardware->buffer, function->length - 4);
					}
					else
					{
						memcpy(hardware->buffer, &function->buffer[4],function->length - 4);
						uint8_t addr_copy = 0x1e;
						HAL_I2C_Master_Transmit(&hi2c2, 0xEc ,&addr_copy , 1, 0xFFFF);
					}
				}
		}break;
		
		case Sensor_Mode_Cmd:
		{
			break;
		}	
	}
}

void Sensor_Handler(Sensor_Hardware_List_t* Hardware)
{
	SensorFunction_t* function;
	LinkedListTakeFirst(Hardware->head,Hardware->tail,function);
	
	if(function == 0)
	{
		//空队列,不做任何事
		//rt_interrupt_leave();
		return;
	}
	
	switch (function->type)
	{
		case SensorFunction_InterfaceI2C:
		{
			Sensor_Send_I2C(function);
			break;
		}
	}
}

		
/*
 *@brief Sensor_ErrorHandler_I2C 发送失败跳转函数
 */
static void Sensor_ErrorHandler_I2C(Sensor_Hardware_List_t* hardware)
{

}

/*
  *@brief fdi_calloc      分配内存
  *@param size            每个元素的大小
  *@param fdi_memory_used 当前已分配的(字)个数
  *@return                返回分配的内存地址
*/
__attribute__((used)) uint32_t fdi_memory_pool[0x5000];
int fdi_memory_used = 0;
void* fdi_calloc(int count, int size)
{
  int words = (count * size + 3) >> 2;
	if ((fdi_memory_used + words) >= 0x5000)
	{
		return 0;
	}
	void* data = &fdi_memory_pool[fdi_memory_used];
	fdi_memory_used += words;
	return data;
}


/*
  *@brief Sensor_Init_I2C I2C传感器初始化
  *@param function        传感器函数指针
  *@param hi2c            I2C句柄指针
  *@param dev_addr        设备地址
*/
#define DMA_SECTION __attribute__((aligned (8),section(".dma")))

DMA_SECTION static Sensor_Hardware_List_t I2C_Hardware;
static uint8_t I2C_HardwareBufferList[64] __attribute__((zero_init,aligned(8),section(".dma")));

int Sensor_Init_I2C(SensorFunction_t * function, I2C_HandleTypeDef * hi2c, uint32_t dev_addr, uint8_t mem_size)
{
  uint16_t MemAddSize;

  switch(mem_size)//目标地址位长
	{
		case 1:MemAddSize = I2C_MEMADD_SIZE_8BIT;break;
		case 2:MemAddSize = I2C_MEMADD_SIZE_16BIT;break;
	}
	
	Sensor_I2C_Type * interface = (Sensor_I2C_Type*)fdi_calloc(1, sizeof(Sensor_I2C_Type));
	
	interface->dev_addr = dev_addr;       //目标外设地址
	interface->mem_addr_size = MemAddSize;//外设地址长度
	
	function->length    = 0;
	function->status    = SensorFunction_Idle;
	function->type      = SensorFunction_InterfaceI2C;
	function->id        = 0;
	function->interface = interface;//目标外设地址 与 size
	function->parameter = 0;
	function->next      = 0;
	function->result    = 0;
	function->mode      = Sensor_Mode_Cmd;
	
	Sensor_Hardware_List_t* hardware = &I2C_Hardware;
	function->hardware = hardware;
	
	if(hardware->until == 0)
	{
		hardware->until     = 1;
		hardware->interface = hi2c;
		hardware->sending   = 0;
		hardware->head      = 0;
		hardware->tail      = 0;
		hardware->stop      = Sensor_ErrorHandler_I2C;
		int index           = 0;//源代码确定是确定i2c_x
		hardware->buffer    = &I2C_HardwareBufferList[index];//指向对应地址 感觉是和function->hardware指向的内存是一个位置的
	}
	
	while (HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY);
	
	if(HAL_I2C_IsDeviceReady(hi2c,dev_addr,300,300) == HAL_TIMEOUT)return 0;
	else                  																				 return -1;    
}

static void I2C_SendOver(I2C_HandleTypeDef *hi2c)
{
	Sensor_Hardware_List_t* hardware = &I2C_Hardware;
	if(hardware->sending)
	{
		SensorFunction_t* function = hardware->sending;//器件本身 这次赋值在最后发送部分
		hardware->sending->status = SensorFunction_Idle;
		hardware->sending = 0;
		function->error_count = 0;
		hardware->error_count = 0;
		
		if (function->mode == Sensor_Mode_Read)
			memcpy(&(function->buffer[4]),hardware->buffer,function->length);
		if (function->result)
			function->result(function);
	}
	Sensor_Handler(hardware);//再次发送
}


#define LinkedListAddLast(head, tail, p)    \
do{                                         \
    p->next = 0;                            \
    if(head == 0)                           \
    {                                       \
        head = tail = p;                    \
        p->prev = 0;                        \
    }                                       \
    else                                    \
    {                                       \
        p->prev = tail;/*P的上一个结点指向tail*/ \
        tail->next = p;/*tail的下一个结点指向p*/ \
        tail = p;/*重新设置尾部位置*/            \
    }                                       \
}while(0) 

/*
 * @brief Sensor_NoDelaySend 无延时发送函数
 * @param function 传感器函数指针
 */
void Sensor_NoDelaySend(SensorFunction_t* function)
{
	function->status = SensorFunction_Ready;
	LinkedListAddLast((function->hardware->head),(function->hardware->tail), function);/*尾插*/
	
	CONTROL_Type control = (CONTROL_Type)__get_CONTROL();
	
	if(!control.b.SPSEL)//无写系统所以其肯定是msp栈区
	{
		//使用MSP（还未进入系统 使用主进程栈）
		Sensor_Handler(function->hardware);
	}
}

/*
 * @brief SensorCMD_Send 发送命令函数
 */
#define SensorFunctionADDR(function) (*((uint32_t *)(function->buffer)))
int SensorCMD_Send(SensorFunction_t* function, const SensorCMD_t* cmd)
{
		if(function->status != SensorFunction_Idle)
	{
		return -1;
	}
	function->delay = cmd->delay;//0
	function->mode  = cmd->mode;//Sensor_Mode_Read
	function->id    = cmd->id;//-1
	switch(cmd->mode)
	{
		case Sensor_Mode_Read:
		{
			function->length = cmd->len + 4;
			SensorFunctionADDR(function) = cmd->addr;//addr 值赋值到 (*((uint32_t *)(function->buffer)))
			memset(function->buffer + 4, 0xff, cmd->len);//
			break;
		}
		case Sensor_Mode_Write:
		{
			function->length = cmd->len + 4;
			memcpy(function->buffer, cmd->buf, function->length);
			break;
		}
		case Sensor_Mode_Cmd:
		{
			function->length = cmd->len;
			memcpy(function->buffer, cmd->buf, cmd->len);
			break;
		}
		default:
		{
			return -1;
		}
	}
	if (function->delay)
	{
		//return Sensor_DelaySend(function);
	}
	else
	{
		Sensor_NoDelaySend(function);
	}
	return 1;
}

/*
 *@brief MS5525_ReadReg MS5525 读取从机数据
 */
static uint8_t MS5525_ReadReg(SensorFunction_t* function,uint8_t addr,uint8_t len)
{
	DefineSensorCMD_DRead(buffer,addr,len);
	SensorCMD_Send(function,&buffer);
	return 1;
}

/*
 *@brief MS5525_ReadReg MS5525 写入从机数据
 */
static int MS5525_WriteReg(SensorFunction_t* function, uint8_t addr,uint8_t len)
{
		DefineSensorCMD_DWriteFL(buffer,addr,len);
		SensorCMD_Send(function, &buffer);
		return 1;
}


/*
  *@brief MS5525_StartUp MS5525 初始化发送函数
*/
void MS5525_StartUp(RM3100_Status_Type* MS5525)
{
	SensorFunction_t * function = &MS5525->Function;
	MS5525->Initialized = 0;
	MS5525_WriteReg(function,0x00,1);
}

static void MS5525_ReceiveValue(SensorFunction_t* function)
{

}

/*
  *@brief MS5525_Init MS5525 模块模式基本配置
  *@param MS5525              MS5525传感器
*/
int MS5525_Init(RM3100_Status_Type* MS5525)
{
		SensorFunction_t * function = &MS5525->Function;
		MS5525->Initialized         = 0;
		function->parameter         = MS5525;
		function->result            = MS5525_ReceiveValue;//暂时result不给值因为ms5525没有对应dma不会从sendover中进行接收数据 
		MS5525_StartUp(MS5525);
    return 0;
}

/**
 * @brief sensorInit 器件初始化函数
 */
void sensorInit() 
{
#ifdef HAS_MS5525
	Sensor_Init_I2C(&MS5525.Function, &hi2c2, 0xEC, 1);
	MS5525_Init(&MS5525);
#endif 
}
