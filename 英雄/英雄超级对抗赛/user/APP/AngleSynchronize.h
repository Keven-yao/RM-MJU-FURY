#ifndef ANGLESYNCHRONIZE_H
#define ANGLESYNCHRONIZE_H

#include "stm32f4xx.h"
#define OVERFLOW_JUDGMENT_SCALE 0.2f //上溢/下溢检测区间长度占总区间的长度

typedef struct
{
	int16_t lastAngle; //上一次反馈值的值
	int16_t *angle[2]; //要同步的两个角度变量指针  [0]: 反馈值  [1]: 设定值
	int8_t turn; //反馈值的圈数
	int16_t upperLimit, lowerLimit; //角度上限和下限
	int16_t overflowJudgmentRange; //当角度在(upperLimit - overflowJudgmentRange ~ upperLimit)和
	                               //(lowerLimit ~ lowerLimit + overflowJudgmentRange)之间跳跃时视作发生上溢/下溢
} AngleSynchronize_t;

//初始化数据结构体
//data: 存储数据的全局变量的指针  feedback: 电机反馈值的指针  setting: 设置值的指针  upperLimit: 反馈角度值的最大值  lowerLimit: 反馈角度值的最小值
void InitSynchronizeStruct(AngleSynchronize_t *data, int16_t *feedback, int16_t *setting, int16_t lowerLimit, int16_t upperLimit);

//在获取电机反馈角度后使用，将角度进行圈数换算
//data: 存储数据的全局变量的指针
void AngleLimit(AngleSynchronize_t *data);

//在更改完设定值后使用，将设定角度与反馈角度进行同步
//data: 存储数据的全局变量的指针
void AngleSynchronize(AngleSynchronize_t *data);
#endif
