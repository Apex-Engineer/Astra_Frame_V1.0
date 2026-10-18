#ifndef PID_H
#define PID_H

#include "main.h"
#include "stdint.h"
#include "usart.h"
#include "stm32f4xx_it.h"
#include "Fuzzy.h"

/**
  * @brief  PID结构体
  * @param  Kp  比例系数
  * @param  Ki  积分系数
  * @param  Kd  微分系数
  * @param  pout    比例计算结果
  * @param  iout    积分计算结果
  * @param  dout    微分计算结果
  * @param  error   差值
  * @param  last_error  上一帧差值
  * @param  set 预期值
  * @param  fdb 实际值
  * @param  out_max 最大输出值
  * @param  out 输出值
  */
typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float pout;
    float iout;
    float dout;

    float error;
    float last_error;
    float last_last_error;

    float set;
    float fdb;
    float out_max;
    float i_max;
    float out;

} pid_type_def;

void PID_init(pid_type_def *pid, float Kp, float Ki, float Kd, float i_max, float max_out);
float PID_Calc(pid_type_def *pid, float ref, float set, int max);
float PID_Calc_Inc(pid_type_def *pid, float ref, float set, int max);

/**
  * @brief  模糊PID结构体
  * @param  pid         基础PID控制器
  * @param  fuzzy_kp    在线调节Kp的模糊控制器
  * @param  fuzzy_ki    在线调节Ki的模糊控制器
  * @param  fuzzy_kd    在线调节Kd的模糊控制器
  * @param  Kp0,Ki0,Kd0 PID初始参数 (基准值)
  * @param  Kp_max      Kp最大修正量 (模糊输出 ±6 映射到 ±Kp_max)
  * @param  Ki_max      Ki最大修正量
  * @param  Kd_max      Kd最大修正量
  * @param  delta_kp    当前Kp修正量
  * @param  delta_ki    当前Ki修正量
  * @param  delta_kd    当前Kd修正量
  */
typedef struct
{
    pid_type_def pid;

    fuzzy_type_def fuzzy_kp;
    fuzzy_type_def fuzzy_ki;
    fuzzy_type_def fuzzy_kd;

    float Kp0;
    float Ki0;
    float Kd0;

    float Kp_max;
    float Ki_max;
    float Kd_max;

    float delta_kp;
    float delta_ki;
    float delta_kd;

} fuzzy_pid_type_def;

void Fuzzy_PID_init(fuzzy_pid_type_def *fpid, float Kp0, float Ki0, float Kd0, float i_max, float max_out, float Kp_max,
                    float Ki_max, float Kd_max, float Ke, float Kec);
float Fuzzy_PID_Calc(fuzzy_pid_type_def *fpid, float ref, float set, int max);

#endif