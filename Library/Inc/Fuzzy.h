#ifndef __FUZZY_H__
#define __FUZZY_H__

#include "main.h"
#include "stdint.h"
#include "usart.h"
#include "stm32f4xx_it.h"

/* ============================================================
 *  模糊控制模块 — 完整教学实现
 *
 *  双输入单输出模糊控制器
 *  输入: 误差 E, 误差变化率 EC
 *  输出: 控制量 U
 *
 *  7 个模糊子集: NB(负大) NM(负中) NS(负小) ZO(零) PS(正小) PM(正中) PB(正大)
 *  隶属度函数: 三角形
 *  推理方法: Mamdani (min)
 *  去模糊化: 重心法
 * ============================================================ */

#define FUZZY_SETS 7 // 7个模糊子集
#define FUZZY_MIN -6 // 模糊论域最小值
#define FUZZY_MAX 6  // 模糊论域最大值

/* 模糊控制结构体: 一个控制器的完整状态 */
typedef struct
{
    /* --- 配置参数 --- */
    float Ke;  // 误差的量化因子 (物理值 → 模糊论域)
    float Kec; // 误差变化率的量化因子
    float Ku;  // 输出的比例因子 (模糊论域 → 物理值)

    float out_max; // 输出上限 (绝对值)

    const int (*rule_table)[FUZZY_SETS]; // 指向 7x7 规则表的指针, 支持自定义规则

    /* --- 运行时状态 --- */
    float error;        // 当前误差
    float last_error;   // 上次误差
    float error_change; // 误差变化率
    float out;          // 最终输出值

} fuzzy_type_def;

/* 初始化模糊控制器 (使用默认规则表) */
void Fuzzy_init(fuzzy_type_def *fuzzy, float Ke, float Kec, float Ku, float out_max);

/* 初始化模糊控制器 (使用自定义规则表) */
void Fuzzy_init_ex(fuzzy_type_def *fuzzy, float Ke, float Kec, float Ku, float out_max,
                   const int rule_table[FUZZY_SETS][FUZZY_SETS]);

/* 执行一次模糊控制计算, 返回精确控制量 */
float Fuzzy_Calc(fuzzy_type_def *fuzzy, float ref, float set);

/* 直接输入 e, ec 进行模糊推理, 不更新控制器内部误差状态 (供模糊 PID 复用) */
float Fuzzy_Calc_Raw(fuzzy_type_def *fuzzy, float e, float ec);

#endif /* __FUZZY_H__ */
