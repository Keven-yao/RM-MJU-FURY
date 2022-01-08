#ifndef ANGLESYNCHRONIZE_H
#define ANGLESYNCHRONIZE_H

#include "stm32f4xx.h"
#define OVERFLOW_JUDGMENT_SCALE 0.2f //����/���������䳤��ռ������ĳ���

typedef struct
{
	int16_t lastAngle; //��һ�η���ֵ��ֵ
	int16_t *angle[2]; //Ҫͬ���������Ƕȱ���ָ��  [0]: ����ֵ  [1]: �趨ֵ
	int8_t turn; //����ֵ��Ȧ��
	int16_t upperLimit, lowerLimit; //�Ƕ����޺�����
	int16_t overflowJudgmentRange; //���Ƕ���(upperLimit - overflowJudgmentRange ~ upperLimit)��
	                               //(lowerLimit ~ lowerLimit + overflowJudgmentRange)֮����Ծʱ������������/����
} AngleSynchronize_t;

//��ʼ�����ݽṹ��
//data: �洢���ݵ�ȫ�ֱ�����ָ��  feedback: �������ֵ��ָ��  setting: ����ֵ��ָ��  upperLimit: �����Ƕ�ֵ�����ֵ  lowerLimit: �����Ƕ�ֵ����Сֵ
void InitSynchronizeStruct(AngleSynchronize_t *data, int16_t *feedback, int16_t *setting, int16_t lowerLimit, int16_t upperLimit);

//�ڻ�ȡ��������ǶȺ�ʹ�ã����ǶȽ���Ȧ������
//data: �洢���ݵ�ȫ�ֱ�����ָ��
void AngleLimit(AngleSynchronize_t *data);

//�ڸ������趨ֵ��ʹ�ã����趨�Ƕ��뷴���ǶȽ���ͬ��
//data: �洢���ݵ�ȫ�ֱ�����ָ��
void AngleSynchronize(AngleSynchronize_t *data);
#endif
