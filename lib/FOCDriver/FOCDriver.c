
#include "FOCDriver.h"
#include "main.h"
#include "math.h"
#include "../../hardware/AS5600lib/AS5600.h"

//记得定义PWM引脚


uint32_t micros(void) {
    uint32_t m0 = HAL_GetTick(); // 获取毫秒级时间
    __IO uint32_t u0 = SysTick->VAL; // 获取当前计数值
    uint32_t m1 = HAL_GetTick(); // 再次获取毫秒级时间，以检测是否发生了毫秒进位
    __IO uint32_t u1 = SysTick->VAL; // 再次获取计数值

    // 如果在获取 m0 与 u0 之间恰好发生了毫秒进位，需要修正计数值
    if (m0 != m1) {
        u0 = u1;
        m0 = m1;
    }

    // SysTick->LOAD + 1 通常等于 1000（因为 SysTick 通常配置为 1ms 中断），
    // 通过 (tms - u0) 计算当前毫秒内已经过去的微秒数
    const uint32_t tms = SysTick->LOAD + 1;
    return (m0 * 1000) + ((tms - u0) / (tms / 1000));
}


//电角度求解
int direction,pole_pairs; //旋转方向，1或-1
float Electrical_Angle(int direction, int pole_pairs)
{
    return Angle_Normalize((float)(direction * pole_pairs) * getAngle_Without_track()-Zero_Electric_Angle);
    //return Angle_Normalize((float)(direction * pole_pairs) * getAngle_Without_track()-Zero_Electric_Angle);
} 
//角度归一化
float Angle_Normalize (float angle)
{
    float a = fmod(angle, 2*PI);
    return (a >= 0) ? a : (a + 2*PI);
}

//写入占空比
void SetPWM(float Ua, float Ub, float Uc)
{
    //限制电压幅值
    Ua = _constrain(Ua, 0.0f, Voltage_Power_Supply);
    Ub = _constrain(Ub, 0.0f, Voltage_Power_Supply);
    Uc = _constrain(Uc, 0.0f, Voltage_Power_Supply);


    // 限制占空比在0到1之间
    dc_a = _constrain(Ua / Voltage_Power_Supply, 0.0f, 1.0f);
    dc_b = _constrain(Ub / Voltage_Power_Supply, 0.0f, 1.0f);
    dc_c = _constrain(Uc / Voltage_Power_Supply, 0.0f, 1.0f);

    //写入PWM占空比
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, (uint32_t)(dc_a * __HAL_TIM_GET_AUTORELOAD(&htim1)));
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, (uint32_t)(dc_b * __HAL_TIM_GET_AUTORELOAD(&htim1)));
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, (uint32_t)(dc_c * __HAL_TIM_GET_AUTORELOAD(&htim1)));

}

void SetphaseVoltage(float Uq, float Angle_el)
{

    Angle_el = Angle_Normalize(Angle_el);
    //clark变换
    Ualpha = -Uq * sinf(Angle_el);
    Ubeta = Uq * cosf(Angle_el);

    //park变换
    // 克拉克逆变换
    Ua = Ualpha + Voltage_Power_Supply/2;
    Ub = (sqrt(3)*Ubeta-Ualpha)/2 + Voltage_Power_Supply/2;
    Uc = (-Ualpha-sqrt(3)*Ubeta)/2 + Voltage_Power_Supply/2;

    SetPWM(Ua, Ub, Uc);
    
}

/*float velocityOpenloop(float target_velocity){
  unsigned long now_us = micros();  //获取从开启芯片以来的微秒数，它的精度是 4 微秒。 micros() 返回的是一个无符号长整型（unsigned long）的值
  
  //计算当前每个Loop的运行时间间隔
  float Ts = (now_us - open_loop_timestamp) * 1e-6f;

  //由于 micros() 函数返回的时间戳会在大约 70 分钟之后重新开始计数，在由70分钟跳变到0时，TS会出现异常，因此需要进行修正。如果时间间隔小于等于零或大于 0.5 秒，则将其设置为一个较小的默认值，即 1e-3f
  if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;
  

  // 通过乘以时间间隔和目标速度来计算需要转动的机械角度，存储在 shaft_angle 变量中。在此之前，还需要对轴角度进行归一化，以确保其值在 0 到 2π 之间。
  shaft_angle = Angle_Normalize(shaft_angle + target_velocity*Ts);
  //以目标速度为 10 rad/s 为例，如果时间间隔是 1 秒，则在每个循环中需要增加 10 * 1 = 10 弧度的角度变化量，才能使电机转动到目标速度。
  //如果时间间隔是 0.1 秒，那么在每个循环中需要增加的角度变化量就是 10 * 0.1 = 1 弧度，才能实现相同的目标速度。因此，电机轴的转动角度取决于目标速度和时间间隔的乘积。

  // 使用早前设置的voltage_power_supply的1/3作为Uq值，这个值会直接影响输出力矩
  // 最大只能设置为Uq = voltage_power_supply/2，否则ua,ub,uc会超出供电电压限幅
  float Uq = Voltage_Power_Supply/3;
  
  SetphaseVoltage(Uq, Electrical_Angle(shaft_angle, pole_pairs));
  
  open_loop_timestamp = now_us;  //用于计算下一个时间间隔

  return Uq;
}
  */

  