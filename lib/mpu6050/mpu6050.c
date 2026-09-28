#include "mpu6050.h"
#define i2c hi2c1
static int16_t Mpu6050Addr = 0x68;
MPU6050DATATYPE Mpu6050_Data;
// 用于互补滤波的全局变量（静态，只在mpu6050.c内使用）
static float pitch_angle = 0.0f;   // 俯仰角（度）
static float roll_angle = 0.0f;    // 横滚角（度）

int8_t Sensor_I2C2_Read(uint16_t DevAddr, uint16_t MemAddr, uint8_t *oData, uint8_t DataLen)
{
	return HAL_I2C_Mem_Read(&i2c,DevAddr,MemAddr,1,oData,DataLen,1000);
}
int8_t Sensor_I2C2_Write(uint16_t DevAddr, uint16_t MemAddr, uint8_t *iData, uint8_t DataLen)
{
	return HAL_I2C_Mem_Write(&i2c,DevAddr,MemAddr,1,iData,DataLen,1000);
}
int16_t Sensor_I2C2_Serch(void)
{
	for(uint8_t i = 1; i < 255; i++)
	{
		if(HAL_I2C_IsDeviceReady(&i2c, i, 1, 1000) == HAL_OK)
		{
			Mpu6050Addr = i;
			return i;
		}
	}
	return 0xD1;
}
int8_t MPU6050_Init(int16_t Addr)
{
	uint8_t check;
	HAL_I2C_Mem_Read(&i2c,Addr,WHO_AM_I,1,&check,1,1000);
	if(check == 0x68) // 确认设备用 地址寄存器
	{	
		check = 0x00;
		Sensor_I2C2_Write(Addr,PWR_MGMT_1,&check, 1); 	    // 唤醒
		check = 0x07;	
		Sensor_I2C2_Write(Addr,SMPLRT_DIV,&check, 1);	    // 1Khz的速率
		check = 0x00;
		Sensor_I2C2_Write(Addr,ACCEL_CONFIG,&check, 1);	 	// 加速度配置
		check = 0x00;
		Sensor_I2C2_Write(Addr,GYRO_CONFIG,&check, 1);		// 陀螺配置
		return 0;
	}
	return -1;
}
void MPU6050_Read_Accel(void)
{
	uint8_t Read_Buf[6];
	
	// 寄存器依次是加速度X高 - 加速度X低 - 加速度Y高位 - 加速度Y低位 - 加速度Z高位 - 加速度度Z低位
	Sensor_I2C2_Read(Mpu6050Addr, ACCEL_XOUT_H, Read_Buf, 6); 
	
	Mpu6050_Data.Accel_X = (int16_t)(Read_Buf[0] << 8 | Read_Buf[1]);
	Mpu6050_Data.Accel_Y = (int16_t)(Read_Buf[2] << 8 | Read_Buf[3]);
	Mpu6050_Data.Accel_Z = (int16_t)(Read_Buf[4] << 8 | Read_Buf[5]);
	
	Mpu6050_Data.Accel_X = Mpu6050_Data.Accel_X / 16384.0f;
	Mpu6050_Data.Accel_Y = Mpu6050_Data.Accel_Y / 16384.0f;
	Mpu6050_Data.Accel_Z = Mpu6050_Data.Accel_Z / 16384.0f;
	
}
void MPU6050_Read_Gyro(void)
{
	uint8_t Read_Buf[6];
	
	// 寄存器依次是角度X高 - 角度X低 - 角度Y高位 - 角度Y低位 - 角度Z高位 - 角度Z低位
	Sensor_I2C2_Read(Mpu6050Addr, GYRO_XOUT_H, Read_Buf, 6); 
	
	Mpu6050_Data.Gyro_X = (int16_t)(Read_Buf[0] << 8 | Read_Buf[1]);
	Mpu6050_Data.Gyro_Y = (int16_t)(Read_Buf[2] << 8 | Read_Buf[3]);
	Mpu6050_Data.Gyro_Z = (int16_t)(Read_Buf[4] << 8 | Read_Buf[5]);
	
	Mpu6050_Data.Gyro_X = Mpu6050_Data.Gyro_X / 131.0f;
	Mpu6050_Data.Gyro_Y = Mpu6050_Data.Gyro_Y / 131.0f;
	Mpu6050_Data.Gyro_Z = Mpu6050_Data.Gyro_Z / 131.0f;
	
}
void MPU6050_Read_Temp(void)
{
    uint8_t Read_Buf[2];
	
	Sensor_I2C2_Read(Mpu6050Addr, TEMP_OUT_H, Read_Buf, 2); 
	
	Mpu6050_Data.Temp = (int16_t)(Read_Buf[0] << 8 | Read_Buf[1]);
	
	Mpu6050_Data.Temp = 36.53f + (Mpu6050_Data.Temp / 340.0f);
}

/**
 * @brief MPU6050互补滤波姿态解算
 * @param pitch  输出：俯仰角（度）
 * @param roll   输出：横滚角（度）
 * @param dt     采样时间间隔（秒）
 */
void MPU6050_Calculate_Angles(float *pitch, float *roll, float dt)
{
    float accel_pitch, accel_roll;
    float gyro_pitch_rate, gyro_roll_rate;
    float alpha = 0.98f;  // 互补滤波系数，陀螺仪权重
    // 系数alpha = DT / (DT + tau)，通常tau取0.1s，当DT=0.01s时alpha≈0.98 [[3]]
    
    // 1. 用加速度计计算静态角度（只依赖重力，不会漂移）
    // pitch = atan2(-ax, sqrt(ay*ay + az*az)) * 180/PI [[4]]
    // roll  = atan2(ay, az) * 180/PI [[5]][[6]]
    // 注意：ax, ay, az 是MPU6050_Read_Accel()已除过16384.0f的值，单位为g
    accel_pitch = atan2f(-Mpu6050_Data.Accel_X,
                         sqrtf(Mpu6050_Data.Accel_Y * Mpu6050_Data.Accel_Y +
                               Mpu6050_Data.Accel_Z * Mpu6050_Data.Accel_Z))
                  * (180.0f / PI);
    
    accel_roll = atan2f(Mpu6050_Data.Accel_Y, Mpu6050_Data.Accel_Z)
                 * (180.0f / PI);
    
    // 2. 陀螺仪角速度（°/s，已经除以131.0f）
    gyro_pitch_rate = Mpu6050_Data.Gyro_X;
    gyro_roll_rate  = Mpu6050_Data.Gyro_Y;
    
    // 3. 互补滤波融合
    // 公式：融合角度 = α × (前一角度 + 陀螺仪积分) + (1-α) × 加速度计角度 [[7]][[8]]
    pitch_angle = alpha * (pitch_angle + gyro_pitch_rate * dt)
                  + (1.0f - alpha) * accel_pitch;
    roll_angle  = alpha * (roll_angle  + gyro_roll_rate  * dt)
                  + (1.0f - alpha) * accel_roll;
    
    // 输出结果
    *pitch = pitch_angle;
    *roll  = roll_angle;
}
