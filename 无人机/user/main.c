#include "stm32f4xx.h"
#include "sys.h"
//#include "usart2.h"
#include "init.h"
//#include "motor.h"
//#include "can.h"
//#include "gimbal_move.h"
//#include "feedback.h"
//#include "PWM.h"


int main(void)
{		
	
		
	Init();
	while (1)
	{
		;
	}
/*			
	u16 led0pwmval=0;
	u8 dir=1;	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 	 //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ� 	
 	PWM_Init();
   	while(1)
	{
 		delay_ms(10);	 
		if(dir)led0pwmval++;
		else led0pwmval--;
 
 		if(led0pwmval>300)dir=0;
		if(led0pwmval==0)dir=1;										 
		TIM_SetCompare2(TIM3,led0pwmval);		   
	}
*/
}
