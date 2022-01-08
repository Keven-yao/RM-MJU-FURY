/****************************************
name:Feedback.c
brief:receive motor feedback by can
function:接收和处理can口接收到的数据，分析后供各个函数调用
author:CadenLei
date:2019.4.21  V1.0

****************************************/


#include "Feedback.h"
#include "can.h"
#include "usart.h"
#include "stdio.h"

feedback_data M3508_1,M3508_2,M3508_3,M3508_4,M3508_5,M3508_6,GM6020_Yaw,GM6020_Pitch,M2006_1,M2006_2;  //定义4个底盘电机和2个云台一共拨弹电机 
static u16 t_fb=0;


unsigned char CAN1_RX0_IRQHandler(void)
{
//    static CanRxMsg rx1_message;

//    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
//    {
//        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
//        CAN_Receive(CAN1, CAN_FIFO0, &rx1_message);
//    }
////unsigned char CAN1_receive_data()
//	{
//    unsigned char i;
  	CanRxMsg RxMessage;
		if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;
    CAN_Receive(CAN1, 0, &RxMessage);

    switch(RxMessage.StdId)
    {
        case 0x201:
							{
								M3508_1.Mechanical_angle = RxMessage.Data[0]<<8|RxMessage.Data[1];
								M3508_1.Rotor_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
								M3508_1.Torque_current = RxMessage.Data[4]<<8|RxMessage.Data[5];
								M3508_1.Temperature = RxMessage.Data[6];
							}
            break;
        case 0x202:
							{
								M3508_2.Mechanical_angle = RxMessage.Data[0]<<8|RxMessage.Data[1];
								M3508_2.Rotor_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
								M3508_2.Torque_current = RxMessage.Data[4]<<8|RxMessage.Data[5];
								M3508_2.Temperature = RxMessage.Data[6];
							}
            break;
				case 0x203:
							{
								M3508_3.Mechanical_angle = RxMessage.Data[0]<<8|RxMessage.Data[1];
								M3508_3.Rotor_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
								M3508_3.Torque_current = RxMessage.Data[4]<<8|RxMessage.Data[5];
								M3508_3.Temperature = RxMessage.Data[6];
							}
            break;
				case 0x204:
            
							{
								M3508_4.Mechanical_angle = RxMessage.Data[0]<<8|RxMessage.Data[1];
								M3508_4.Rotor_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
								M3508_4.Torque_current = RxMessage.Data[4]<<8|RxMessage.Data[5];
								M3508_4.Temperature = RxMessage.Data[6];
							}
            break;
			/*	case 0x205:
            
							{
								M3508_5.Mechanical_angle = RxMessage.Data[0]<<8|RxMessage.Data[1];
								M3508_5.Rotor_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
								M3508_5.Torque_current = RxMessage.Data[4]<<8|RxMessage.Data[5];
								M3508_5.Temperature = RxMessage.Data[6];
							}
            break;
				case 0x206:
            
							{
								M3508_6.Mechanical_angle = RxMessage.Data[0]<<8|RxMessage.Data[1];
								M3508_6.Rotor_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
								M3508_6.Torque_current = RxMessage.Data[4]<<8|RxMessage.Data[5];
								M3508_6.Temperature = RxMessage.Data[6];
							}
            break;							*/
				case 0x209:
							{
								GM6020_Yaw.Mechanical_angle = RxMessage.Data[0]<<8|RxMessage.Data[1];
								GM6020_Yaw.Rotor_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
								GM6020_Yaw.Torque_current = RxMessage.Data[4]<<8|RxMessage.Data[5];
								GM6020_Yaw.Temperature = RxMessage.Data[6];
							}
					  break;
				case 0x20a:
       
							{
								GM6020_Pitch.Mechanical_angle = RxMessage.Data[0]<<8|RxMessage.Data[1];
								GM6020_Pitch.Rotor_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
								GM6020_Pitch.Torque_current = RxMessage.Data[4]<<8|RxMessage.Data[5];
								GM6020_Pitch.Temperature = RxMessage.Data[6];
							}
            break;
				case 0x205:
            
							{
								M2006_1.Mechanical_angle = RxMessage.Data[0]<<8|RxMessage.Data[1];
								M2006_1.Rotor_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
								M2006_1.Torque_current = RxMessage.Data[4]<<8|RxMessage.Data[5];
								M2006_1.Temperature = RxMessage.Data[6];
							}
//				case 0x206:
//            
//							{
//								M2006_2.Mechanical_angle = RxMessage.Data[0]<<8|RxMessage.Data[1];
//								M2006_2.Rotor_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
//								M2006_2.Torque_current = RxMessage.Data[4]<<8|RxMessage.Data[5];
//								M2006_2.Temperature = RxMessage.Data[6];
//							}
//            break;

				
        default:
            break;
    }
//		if (t_fb>=1000)
//		{
//			printf("%d;%d\r\n",GM6020_Yaw.Mechanical_angle,GM6020_Pitch.Mechanical_angle);
//			t_fb=0;
//		}
//		else 
//		{
//			t_fb++;
//		}
	//	printf("the Mechanical angle is : %d \r\n",GM6020_Yaw.Mechanical_angle);
    return RxMessage.DLC;

}



/*
unsigned char CAN2_RX0_IRQHandler(void)
{

  	CanRxMsg RxMessage;
		if( CAN_MessagePending(CAN2,CAN_FIFO1)==0)return 0;
    CAN_Receive(CAN2, 0, &RxMessage);

     switch(RxMessage.StdId)
    {
//        case 0x201:
//							{
//								M3508_1.Mechanical_angle = RxMessage.Data[0]<<8|RxMessage.Data[1];
//								M3508_1.Rotor_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
//								M3508_1.Torque_current = RxMessage.Data[4]<<8|RxMessage.Data[5];
//								M3508_1.Temperature = RxMessage.Data[6];
//							}
//            break;
//        case 0x202:
//							{
//								M3508_2.Mechanical_angle = RxMessage.Data[0]<<8|RxMessage.Data[1];
//								M3508_2.Rotor_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
//								M3508_2.Torque_current = RxMessage.Data[4]<<8|RxMessage.Data[5];
//								M3508_2.Temperature = RxMessage.Data[6];
//							}
//            break;
//				case 0x203:
//							{
//								M3508_3.Mechanical_angle = RxMessage.Data[0]<<8|RxMessage.Data[1];
//								M3508_3.Rotor_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
//								M3508_3.Torque_current = RxMessage.Data[4]<<8|RxMessage.Data[5];
//								M3508_3.Temperature = RxMessage.Data[6];
//							}
//            break;
//				case 0x204:
//            
//							{
//								M3508_4.Mechanical_angle = RxMessage.Data[0]<<8|RxMessage.Data[1];
//								M3508_4.Rotor_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
//								M3508_4.Torque_current = RxMessage.Data[4]<<8|RxMessage.Data[5];
//								M3508_4.Temperature = RxMessage.Data[6];
//							}
//            break;
				case 0x205:
            
							{
								M3508_5.Mechanical_angle = RxMessage.Data[0]<<8|RxMessage.Data[1];
								M3508_5.Rotor_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
								M3508_5.Torque_current = RxMessage.Data[4]<<8|RxMessage.Data[5];
								M3508_5.Temperature = RxMessage.Data[6];
							}
            break;
				case 0x206:
            
							{
								M3508_6.Mechanical_angle = RxMessage.Data[0]<<8|RxMessage.Data[1];
								M3508_6.Rotor_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
								M3508_6.Torque_current = RxMessage.Data[4]<<8|RxMessage.Data[5];
								M3508_6.Temperature = RxMessage.Data[6];
							}
            break;							
				case 0x209:
							{
								GM6020_Yaw.Mechanical_angle = RxMessage.Data[0]<<8|RxMessage.Data[1];
								GM6020_Yaw.Rotor_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
								GM6020_Yaw.Torque_current = RxMessage.Data[4]<<8|RxMessage.Data[5];
								GM6020_Yaw.Temperature = RxMessage.Data[6];
							}
					  break;
				case 0x20a:
       
							{
								GM6020_Pitch.Mechanical_angle = RxMessage.Data[0]<<8|RxMessage.Data[1];
								GM6020_Pitch.Rotor_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
								GM6020_Pitch.Torque_current = RxMessage.Data[4]<<8|RxMessage.Data[5];
								GM6020_Pitch.Temperature = RxMessage.Data[6];
							}
            break;
//				case 0x207:
//            
//							{
//								M2006_1.Mechanical_angle = RxMessage.Data[0]<<8|RxMessage.Data[1];
//								M2006_1.Rotor_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
//								M2006_1.Torque_current = RxMessage.Data[4]<<8|RxMessage.Data[5];
//								M2006_1.Temperature = RxMessage.Data[6];
//							}
//				case 0x208:
//            
//							{
//								M2006_2.Mechanical_angle = RxMessage.Data[0]<<8|RxMessage.Data[1];
//								M2006_2.Rotor_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
//								M2006_2.Torque_current = RxMessage.Data[4]<<8|RxMessage.Data[5];
//								M2006_2.Temperature = RxMessage.Data[6];
//							}
//            break;

				
        default:
            break;
    }
		if (t_fb>=1000)
		{
			printf("feedback-->%d\r\n",M2006_1.Rotor_speed);
			
			t_fb=0;
		}
		else 
		{
			t_fb++;
		}
	//	printf("the Mechanical angle is : %d \r\n",GM6020_Yaw.Mechanical_angle);
    return RxMessage.DLC;

}*/









