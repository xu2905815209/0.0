#include "i2c.h"
#include "mpu.h"
#define MPU6050_ADDRESS  0xD0  //从机地址 + w

/*
//等待事件，超时退出(硬件I2C)
void MPU6050_WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT)
{
	uint32_t Timeout;
	Timeout = 10000;
	while (I2C_CheckEvent(I2Cx, I2C_EVENT) != SUCCESS)
	{
		Timeout --;
		if (Timeout == 0)
		{
			break;
		}
	}
}
*/
//指定地址写
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
	/******************************************************************************************************/
	/***********************************软件I2C************************************************************/
	MyI2C_Start();//起始条件
	MyI2C_SendByte(MPU6050_ADDRESS);//指定从机设备
	MyI2C_ReceiveAck();
	MyI2C_SendByte(RegAddress);//指定写入的寄存器
	MyI2C_ReceiveAck();//接收应答
	MyI2C_SendByte(Data);//写入数据（一个字节）
	MyI2C_ReceiveAck();//接收应答
	MyI2C_Stop();//终止条件
	/******************************************************************************************************/
	/***********************************硬件I2C实现********************************************************/
	/*
	I2C_GenerateSTART(I2C2, ENABLE);//生成起始条件
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT); 
	
	I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Transmitter);//发送从机地址
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
	
	I2C_SendData(I2C2, RegAddress);//像DR寄存器写入寄存器地址
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTING);
	
	I2C_SendData(I2C2, Data);//像DR寄存器写入数据
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
	
	I2C_GenerateSTOP(I2C2, ENABLE);//生成退出条件
	*/
	/*****************************************************************************************************/
}

//指定地址读
uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
	/******************************************************************************************************/
	/***********************************软件I2C************************************************************/
	/*******为了指定读取的地址********/
	uint8_t Data;
	
	MyI2C_Start();//起始条件
	MyI2C_SendByte(MPU6050_ADDRESS);//指定从机设备
	MyI2C_ReceiveAck();
	MyI2C_SendByte(RegAddress);//指定读取的寄存器
	MyI2C_ReceiveAck();//接收应答
	
	MyI2C_Start();
	MyI2C_SendByte(MPU6050_ADDRESS | 0x01);
	MyI2C_ReceiveAck();
	Data = MyI2C_ReceiveByte();//读取一个字节
	MyI2C_SendAck(1);
	MyI2C_Stop();
	
	return Data;
	/******************************************************************************************************/
	/***********************************硬件I2C实现********************************************************/
//	uint8_t Data;
//	/***********指定读取的地址***********************/
//	I2C_GenerateSTART(I2C2, ENABLE);
//	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);
//	
//	I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Transmitter);
//	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
//	
//	I2C_SendData(I2C2, RegAddress);
//	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
//	/***********************************************/
//	I2C_GenerateSTART(I2C2, ENABLE);//生成重复起始条件
//	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);
//	
//	I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Receiver);//指定从机地址
//	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);
//	
//	I2C_AcknowledgeConfig(I2C2, DISABLE);//在接收数据之前给应答，设置STOP停止条件（为了响应快速的时序）
//	I2C_GenerateSTOP(I2C2, ENABLE);
//	
//	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);
//	Data = I2C_ReceiveData(I2C2);//接收一个字节
//	
//	I2C_AcknowledgeConfig(I2C2, ENABLE);
//	
//	return Data;
}


//设置MPU6050的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
void MPU6050_Set_LPF(uint16_t lpf)
{
	uint8_t data = 0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6;
	MPU6050_WriteReg(MPU6050_CONFIG,data);
}

//设置MPU6050的采样率(假定Fs=1KHz)
void MPU6050_Set_Rate(uint16_t rate)
{
	uint8_t data;
	if(rate > 1000) rate = 1000;
	if(rate < 4) rate = 4;
	data = 1000 / rate -1;
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV,data);
	MPU6050_Set_LPF(rate / 2);
}


//获取设备的ID
uint8_t MPU6050_GetID(void)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

void MPU6050_Init(void)
{
	MX_I2C1_Init();
	
	/***************************************硬件I2C初始化********************************************************************/
	/*
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,ENABLE);//开启I2C2的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//开启引脚的时钟
	//初始化引脚为复用开漏模式 
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//初始化I2C外设
	I2C_InitTypeDef I2C_InitStructure;
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_ClockSpeed = 50000;//通信频率，最高400KHz
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;//占空比参数
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;//地址响应位数
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;//stm32作为从机的地址
	I2C_Init(I2C2, &I2C_InitStructure);
	
	I2C_Cmd(I2C2, ENABLE);//使能I2C2
	*/
	/***************************************************************************************/
	uint8_t res;
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x80);//复位MPU6050
	HAL_Delay(100);//等待MPU6050复位完成
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x00);//唤醒MPU6050
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);//配置陀螺仪寄存器
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);//配置加速度寄存器
	MPU6050_Set_Rate(200);
	MPU6050_WriteReg(MPU6050_INT_EN_REG,0x10);//开启中断,中断源：FIFO
	MPU6050_WriteReg(MPU6050_FIFO_EN_REG,0x40);//开启FIFO中断
	MPU6050_WriteReg(MPU6050_INTBP_CFG_REG,0x80);//INT引脚高电平有效
	res = MPU6050_GetID();//获取设备ID
	if(res == 0x68)
	{
		MPU6050_WriteReg(MPU6050_PWR_MGMT_1,0x01);//设置CLKSEL,PLL X轴为参考
		MPU6050_WriteReg(MPU6050_PWR_MGMT_2,0x00);//加速度与陀螺仪都工作
		MPU6050_Set_Rate(200);//使INT引脚输出5ms的中断信号
	}
}


//获取MPU6050的6轴传感数据
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
	uint16_t DataH, DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
	*AccX = (DataH << 8) | DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
	*AccY = (DataH << 8) | DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
	*AccZ = (DataH << 8) | DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
	*GyroX = (DataH << 8) | DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
	*GyroY = (DataH << 8) | DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
	*GyroZ = (DataH << 8) | DataL;
}


