#include "anglesynchronize.h"

//初始化数据结构体
//data: 存储数据的全局变量的指针  feebback: 电机反馈值的指针  setting: 设置值的指针  upperLimit: 反馈角度值的最大值  lowerLimit: 反馈角度值的最小值
void InitSynchronizeStruct(AngleSynchronize_t *data, int16_t *feedback, int16_t *setting, int16_t lowerLimit, int16_t upperLimit)
{
	data->lastAngle = (upperLimit - lowerLimit) / 2;
	data->angle[0] = feedback;
	data->angle[1] = setting;
	data->turn = 0;
	data->upperLimit = upperLimit;
	data->lowerLimit = lowerLimit;
	data->overflowJudgmentRange = (int)((upperLimit - lowerLimit) * OVERFLOW_JUDGMENT_SCALE);
}

//在获取电机反馈角度后使用，将角度进行圈数换算
//data: 存储数据的全局变量的指针
void AngleLimit(AngleSynchronize_t *data)
{
	if (*data->angle[0] >= data->upperLimit - data->overflowJudgmentRange && data->lastAngle < data->lowerLimit + data->overflowJudgmentRange)
	{
		data->turn--;
	}
	else if (*data->angle[0] < data->lowerLimit + data->overflowJudgmentRange && data->lastAngle >= data->upperLimit - data->overflowJudgmentRange)
	{
		data->turn++;
	}
	data->lastAngle = *data->angle[0];
	*data->angle[0] += (data->upperLimit - data->lowerLimit) * data->turn;
}

//在更改完设定值后使用，将设定角度与反馈角度进行同步
//data: 存储数据的全局变量的指针
void AngleSynchronize(AngleSynchronize_t *data)
{
	if (*data->angle[0] >= data->upperLimit && *data->angle[1] >= data->upperLimit)
	{
		*data->angle[0] -= data->upperLimit - data->lowerLimit;
		*data->angle[1] -= data->upperLimit - data->lowerLimit;
		data->turn--;
	}
	else if (*data->angle[0] < data->lowerLimit && *data->angle[1] < data->lowerLimit)
	{
		*data->angle[0] += data->upperLimit - data->lowerLimit;
		*data->angle[1] += data->upperLimit - data->lowerLimit;
		data->turn++;
	}
}
