/****************************************
name:Feedback.c
brief:receive motor feedback by can
function:���պʹ���can�ڽ��յ������ݣ������󹩸�����������
author:CadenLei
date:2019.4.21  V1.0

****************************************/


#include "main.h"

feedback_data M3508_1,M3508_2,M3508_3,M3508_4,M3508_5,M3508_6,GM6020_Yaw,GM6020_Pitch,GM6020_Roll,M2006_1,M2006_2,GM6020_give,GM6020_Pitch2;;  //����4�����̵����2����̨һ��������� 


//static u16 t_fb=0;
//static int16_t last_angle = 9000;
//static int16_t count=0;
int32_t num=0;

  

unsigned char CAN1_RX0_IRQHandler(void)    //�����ж�
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
		if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;   //û�������ݷ���0
    CAN_Receive(CAN1, 0, &RxMessage);                     //�����ݽ���RxMessage��

    switch(RxMessage.StdId)//��ʶ����0x200+���ID ��ID Ϊ1���ñ�ʶ��Ϊ0x201  ��׼֡ DATA  8�ֽ�
    {
        case 0x201:  //M3508 ID:1      //���̵��        ����:0x200[0,1]
							{
								M3508_1.Mechanical_angle = RxMessage.Data[0]<<8|RxMessage.Data[1];
								M3508_1.Rotor_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
								M3508_1.Torque_current = RxMessage.Data[4]<<8|RxMessage.Data[5];
								M3508_1.Temperature = RxMessage.Data[6];
							}
            break;
        case 0x202:  //M3508 ID:2       //���̵��        ����:0x200[2,3]
							{
								M3508_2.Mechanical_angle = RxMessage.Data[0]<<8|RxMessage.Data[1];
								M3508_2.Rotor_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
								M3508_2.Torque_current = RxMessage.Data[4]<<8|RxMessage.Data[5];
								M3508_2.Temperature = RxMessage.Data[6];
							}
            break;
				case 0x203:   //M3508 ID:3        //���̵��        ����:0x200[4,5]
							{
								M3508_3.Mechanical_angle = RxMessage.Data[0]<<8|RxMessage.Data[1];
								M3508_3.Rotor_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
								M3508_3.Torque_current = RxMessage.Data[4]<<8|RxMessage.Data[5];
								M3508_3.Temperature = RxMessage.Data[6];
							}
            break;
				case 0x204:   //M3508 ID:4        //���̵��        ����:0x200[6,7]
            
							{
								M3508_4.Mechanical_angle = RxMessage.Data[0]<<8|RxMessage.Data[1];
								M3508_4.Rotor_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
								M3508_4.Torque_current = RxMessage.Data[4]<<8|RxMessage.Data[5];
								M3508_4.Temperature = RxMessage.Data[6];
							}
            break;

//				case 0x205:    //M3508 ID:5    //Ħ���ֵ��        ����:0x1ff[0,1]           
//							{
//								M3508_5.Mechanical_angle = RxMessage.Data[0]<<8|RxMessage.Data[1];
//								M3508_5.Rotor_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
//								M3508_5.Torque_current = RxMessage.Data[4]<<8|RxMessage.Data[5];
//								M3508_5.Temperature = RxMessage.Data[6];
//							}
//            break;						
//				case 0x206:    //M3508 ID:6     //Ħ���ֵ��        ����:0x1ff[2,3]            
//							{
//								M3508_6.Mechanical_angle = RxMessage.Data[0]<<8|RxMessage.Data[1];
//								M3508_6.Rotor_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
//								M3508_6.Torque_current = RxMessage.Data[4]<<8|RxMessage.Data[5];
//								M3508_6.Temperature = RxMessage.Data[6];
//							}
//            break;
							
//				case 0x208:     //M2006   ID:8   ����               ����:0x1ff[4,5]
//            
//							{
//								M2006_1.Mechanical_angle = RxMessage.Data[0]<<8|RxMessage.Data[1];
//								M2006_1.Rotor_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
//								M2006_1.Torque_current = RxMessage.Data[4]<<8|RxMessage.Data[5];
//								M2006_1.Temperature = RxMessage.Data[6];
//							}
//				
//				case 0x207:    //GM6020 ID:3    ����               ����:0x1ff[6,7]
//       
//							{
//								GM6020_give.Mechanical_angle = RxMessage.Data[0]<<8|RxMessage.Data[1];
//								GM6020_give.Rotor_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
//								GM6020_give.Torque_current = RxMessage.Data[4]<<8|RxMessage.Data[5];
//								GM6020_give.Temperature = RxMessage.Data[6];
//							}
//            break;
//				
				case 0x209:    //GM6020 ID:5   Yaw    ��̨ˮƽ               ����:0x2ff[0,1]
							{
								GM6020_Yaw.Mechanical_angle = RxMessage.Data[0]<<8|RxMessage.Data[1];
								GM6020_Yaw.Rotor_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
								GM6020_Yaw.Torque_current = RxMessage.Data[4]<<8|RxMessage.Data[5];
								GM6020_Yaw.Temperature = RxMessage.Data[6];
							}
					  break;
				case 0x20a:    //GM6020 ID:6   pitch1   ��̨����             ����:0x2ff[2,3]
       
							{
								GM6020_Pitch.Mechanical_angle = RxMessage.Data[0]<<8|RxMessage.Data[1];
								GM6020_Pitch.Rotor_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
								GM6020_Pitch.Torque_current = RxMessage.Data[4]<<8|RxMessage.Data[5];
								GM6020_Pitch.Temperature = RxMessage.Data[6];
							}
            break;
//				case 0x20b:    //GM6020 ID:7    roll                     ����:0x2ff[4,5]
//       
//							{
//								GM6020_Roll.Mechanical_angle = RxMessage.Data[0]<<8|RxMessage.Data[1];
//								GM6020_Roll.Rotor_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
//								GM6020_Roll.Torque_current = RxMessage.Data[4]<<8|RxMessage.Data[5];
//								GM6020_Roll.Temperature = RxMessage.Data[6];
//							}
//            break;
//				

//				
//        default:
//            break;
    }
		
		
//		if(last_angle==9000)  {
//		last_angle = M2006_1.Mechanical_angle;
//	  }
//		if (M2006_1.Mechanical_angle - last_angle > 4500) count--;
//		else if(M2006_1.Mechanical_angle - last_angle < -4500) count++;
//		
//		num=8191*count+M2006_1.Mechanical_angle;
//		last_angle = M2006_1.Mechanical_angle;
//	
		
		
//		
//		if (t_fb>=1000)
//		{
//			printf("feedback-->%d\r\n",GM6020_give.Rotor_speed);
//		
//			t_fb=0;
//		}
//		else t_fb++;

//	//	printf("the Mechanical angle is : %d \r\n",GM6020_Yaw.);
    return RxMessage.DLC;            //�������ݳ���
}




     //��can2������̨�ϵĵ��
unsigned char CAN2_RX0_IRQHandler(void)     //�����ж�
{

  	CanRxMsg RxMessage;
		if( CAN_MessagePending(CAN2,CAN_FIFO0)==0)return 0;
    CAN_Receive(CAN2, 0, &RxMessage);

     switch(RxMessage.StdId)
    {
//        case 0x201:  //���̵������
//							{
//								M3508_1.Mechanical_angle = RxMessage.Data[0]<<8|RxMessage.Data[1];
//								M3508_1.Rotor_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
//								M3508_1.Torque_current = RxMessage.Data[4]<<8|RxMessage.Data[5];
//								M3508_1.Temperature = RxMessage.Data[6];
//							}
//            break;
//        case 0x202:  //���̵������
//							{
//								M3508_2.Mechanical_angle = RxMessage.Data[0]<<8|RxMessage.Data[1];
//								M3508_2.Rotor_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
//								M3508_2.Torque_current = RxMessage.Data[4]<<8|RxMessage.Data[5];
//								M3508_2.Temperature = RxMessage.Data[6];
//							}
//            break;
//				case 0x203: //���̵������
//							{
//								M3508_3.Mechanical_angle = RxMessage.Data[0]<<8|RxMessage.Data[1];
//								M3508_3.Rotor_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
//								M3508_3.Torque_current = RxMessage.Data[4]<<8|RxMessage.Data[5];
//								M3508_3.Temperature = RxMessage.Data[6];
//							}
//            break;
//				case 0x204: //���̵������
//            
//							{
//								M3508_4.Mechanical_angle = RxMessage.Data[0]<<8|RxMessage.Data[1];
//								M3508_4.Rotor_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
//								M3508_4.Torque_current = RxMessage.Data[4]<<8|RxMessage.Data[5];
//								M3508_4.Temperature = RxMessage.Data[6];
//							}
//            break;
//						case 0x205:    //GM6020 ID:1    ����               ����:0x1ff[6,7]
//     
//							{
//								GM6020_give.Mechanical_angle = RxMessage.Data[0]<<8|RxMessage.Data[1];
//								GM6020_give.Rotor_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
//								GM6020_give.Torque_current = RxMessage.Data[4]<<8|RxMessage.Data[5];
//								GM6020_give.Temperature = RxMessage.Data[6];
//							}
//            break;//GM6020_give����
//							case 0x206:     //M2006   ID:6   ����               ����:0x1ff[4,5]
//            
//							{
//								M2006_1.Mechanical_angle = RxMessage.Data[0]<<8|RxMessage.Data[1];
//								M2006_1.Rotor_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
//								M2006_1.Torque_current = RxMessage.Data[4]<<8|RxMessage.Data[5];
//								M2006_1.Temperature = RxMessage.Data[6];
//							}
//				case 0x205:
//            
//							{
//								M3508_5.Mechanical_angle = RxMessage.Data[0]<<8|RxMessage.Data[1];
//								M3508_5.Rotor_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
//								M3508_5.Torque_current = RxMessage.Data[4]<<8|RxMessage.Data[5];
//								M3508_5.Temperature = RxMessage.Data[6];
//							}
//            break;
//				case 0x206:
//            
//							{
//								M3508_6.Mechanical_angle = RxMessage.Data[0]<<8|RxMessage.Data[1];
//								M3508_6.Rotor_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
//								M3508_6.Torque_current = RxMessage.Data[4]<<8|RxMessage.Data[5];
//								M3508_6.Temperature = RxMessage.Data[6];
//							}
//            break;							
//				case 0x209:     //ID 5  M6020 yaw
//							{
//								GM6020_Yaw.Mechanical_angle = RxMessage.Data[0]<<8|RxMessage.Data[1];
//								GM6020_Yaw.Rotor_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
//								GM6020_Yaw.Torque_current = RxMessage.Data[4]<<8|RxMessage.Data[5];
//								GM6020_Yaw.Temperature = RxMessage.Data[6];
//							}
//					  break;
//											case 0x20a:    //GM6020 ID:6   pitch1   ��̨����             ����:0x2ff[2,3]
//       
//							{
//								GM6020_Pitch.Mechanical_angle = RxMessage.Data[0]<<8|RxMessage.Data[1];
//								GM6020_Pitch.Rotor_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
//								GM6020_Pitch.Torque_current = RxMessage.Data[4]<<8|RxMessage.Data[5];
//								GM6020_Pitch.Temperature = RxMessage.Data[6];
//							}
//            break;
//				case 0x20b:    //GM6020 ID:7    roll                     ����:0x2ff[4,5]
//       
//							{
//								GM6020_Roll.Mechanical_angle = RxMessage.Data[0]<<8|RxMessage.Data[1];
//								GM6020_Roll.Rotor_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
//								GM6020_Roll.Torque_current = RxMessage.Data[4]<<8|RxMessage.Data[5];
//								GM6020_Roll.Temperature = RxMessage.Data[6];
//							}
//            break;
//				case 0x20a:
//       
//							{
//								GM6020_Pitch.Mechanical_angle = RxMessage.Data[0]<<8|RxMessage.Data[1];
//								GM6020_Pitch.Rotor_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
//								GM6020_Pitch.Torque_current = RxMessage.Data[4]<<8|RxMessage.Data[5];
//								GM6020_Pitch.Temperature = RxMessage.Data[6];
//							}
//            break;
				case 0x201:
            
							{
								M2006_1.Mechanical_angle = RxMessage.Data[0]<<8|RxMessage.Data[1];
								M2006_1.Rotor_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
								M2006_1.Torque_current = RxMessage.Data[4]<<8|RxMessage.Data[5];
								M2006_1.Temperature = RxMessage.Data[6];
							}
							break;
//				case 0x208:
//            
//							{
//								M2006_2.Mechanical_angle = RxMessage.Data[0]<<8|RxMessage.Data[1];
//								M2006_2.Rotor_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
//								M2006_2.Torque_current = RxMessage.Data[4]<<8|RxMessage.Data[5];
//								M2006_2.Temperature = RxMessage.Data[6];
//							}
//            break;

				
//        default:
//            break;
//    }
//		if (t_fb>=1000)
//		{
//			printf("feedback-->%d\r\n",M2006_1.Rotor_speed);
//			
//			t_fb=0;
//		}
//		else 
//		{
//			t_fb++;
		}
	//	printf("the Mechanical angle is : %d \r\n",GM6020_Yaw.Mechanical_angle);
    return RxMessage.DLC;             //�������ݳ���

}








