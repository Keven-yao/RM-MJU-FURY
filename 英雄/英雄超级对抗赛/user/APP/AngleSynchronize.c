#include "anglesynchronize.h"

//��ʼ�����ݽṹ��
//data: �洢���ݵ�ȫ�ֱ�����ָ��  feebback: �������ֵ��ָ��  setting: ����ֵ��ָ��  upperLimit: �����Ƕ�ֵ�����ֵ  lowerLimit: �����Ƕ�ֵ����Сֵ
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

//�ڻ�ȡ��������ǶȺ�ʹ�ã����ǶȽ���Ȧ������
//data: �洢���ݵ�ȫ�ֱ�����ָ��
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

//�ڸ������趨ֵ��ʹ�ã����趨�Ƕ��뷴���ǶȽ���ͬ��
//data: �洢���ݵ�ȫ�ֱ�����ָ��
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
