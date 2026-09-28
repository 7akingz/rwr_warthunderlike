#include "AS5600.h"
#include "math.h"
float angleprev = 0.0f; //上次角度
int32_t full_rotations = 0; //转过的圈数
unsigned char write_reg(unsigned char reg,unsigned char* value,unsigned short len)
{
    return HAL_I2C_Mem_Write(&hi2c1, Slave_Address, reg, I2C_MEMADD_SIZE_8BIT, value, len, 100);
}

unsigned char read_register(unsigned char reg, unsigned char* data, unsigned short len)
{
    return HAL_I2C_Mem_Read(&hi2c1, Slave_Address, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

float i2c_AS5600_read_rawangle(void)
{
    //float angle_d;
    int16_t in_angle;
    uint8_t temp_hi = 0;
    uint8_t temp_lo = 0;
    read_register(Angle_High_Register, &temp_hi, 1);
    read_register(Angle_Low_Register, &temp_lo, 1);
    in_angle = (temp_hi << 8) | temp_lo;
    return in_angle;
}

float getAngle_Without_track(void)
{
    return i2c_AS5600_read_rawangle() *0.08789*PI/180.0f; // 将原始角度转换为弧度

}

float getAngle(void)
{
    float val = getAngle_Without_track();
    float d_angle = val - angleprev;
    if (fabs(d_angle)>(0.8f*2*6.28318530718f)) // 发生了跳变，说明转过了零点
    {
        full_rotations += (d_angle > 0) ? -1 : 1; // 根据跳变方向更新转过的圈数
    }
    angleprev = val;
    return  (float)(full_rotations * 6.28318530718f + angleprev);// 返回总角度，单位为弧度

}






