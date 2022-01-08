#include "judge.h"
#include "string.h"

#include "crc.h"
#include "main.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "init.h"
#include "time.h"
#include "main.h"

/***********************************

��һ�β���ϵͳ���ݴ���Ͷ�ȡ�����ǲο����ѧԺ��Դ������иĶ�������ϵͳ�����Զ�ȡ�������ݣ���Ҫ���µ��˶���������

***********************************/

/*****************ϵͳ���ݶ���**********************/
ext_game_state_t       				GameState;					//0x0001
ext_game_result_t            		GameResult;					//0x0002
ext_game_robot_survivors_t          GameRobotSurvivors;			//0x0003
ext_event_data_t        			EventData;					//0x0101
ext_supply_projectile_action_t		SupplyProjectileAction;		//0x0102
ext_supply_projectile_booking_t		SupplyProjectileBooking;	//0x0103
ext_game_robot_state_t			  	GameRobotStat;				//0x0201
ext_power_heat_data_t		  		PowerHeatData;				//0x0202
ext_game_robot_pos_t				GameRobotPos;				//0x0203
ext_buff_musk_t						BuffMusk;					//0x0204
aerial_robot_energy_t				AerialRobotEnergy;			//0x0205
ext_robot_hurt_t					RobotHurt;					//0x0206
ext_shoot_data_t					ShootData;					//0x0207

xFrameHeader              FrameHeader;		//����֡ͷ��Ϣ
ext_SendClientData_t      ShowData;			//�ͻ�����Ϣ
ext_CommunatianData_t     CommuData;		//����ͨ����Ϣ
/****************************************************/

bool Judge_Data_TF = false;//���������Ƿ����,������������
uint8_t Judge_Self_ID;//��ǰ�����˵�ID
uint16_t Judge_SelfClient_ID;//�����߻����˶�Ӧ�Ŀͻ���ID


/**************����ϵͳ���ݸ���****************/
uint16_t ShootNum;//ͳ�Ʒ�����,0x0003����һ������Ϊ������һ��
bool Hurt_Data_Update = false;//װ�װ��˺������Ƿ����,ÿ��һ���˺���true,Ȼ��������false,������������
#define BLUE  0
#define RED   1

/**
  * @brief  ��ȡ��������,�ж��ж�ȡ��֤�ٶ�
  * @param  ��������
  * @retval �Ƿ�������ж�������
  * @attention  �ڴ��ж�֡ͷ��CRCУ��,������д�����ݣ����ظ��ж�֡ͷ
  */
bool Judge_Read_Data(uint8_t *ReadFromUsart)
{
	bool retval_tf = false;//������ȷ����־,ÿ�ε��ö�ȡ����ϵͳ���ݺ�������Ĭ��Ϊ����
	
	uint16_t judge_length;//ͳ��һ֡���ݳ��� 
	
	int CmdID = 0;//�������������
	
	/***------------------*****/
	//�����ݰ��������κδ���
	if (ReadFromUsart == NULL)
	{
		return -1;
	}

			
	
	//д��֡ͷ����,�����ж��Ƿ�ʼ�洢��������
	memcpy(&FrameHeader, ReadFromUsart, LEN_HEADER);
	
	//�ж�֡ͷ�����Ƿ�Ϊ0xA5
	if(ReadFromUsart[ SOF ] == JUDGE_FRAME_HEADER)
	{
		
		//֡ͷCRC8У��
		if (Verify_CRC8_Check_Sum( ReadFromUsart, LEN_HEADER ) == true)
		{
			//ͳ��һ֡���ݳ���,����CR16У��
			judge_length = ReadFromUsart[ DATA_LENGTH ] + LEN_HEADER + LEN_CMDID + LEN_TAIL;;

			//֡βCRC16У��
			if(Verify_CRC16_Check_Sum(ReadFromUsart,judge_length) == true)
			{
				
				retval_tf = true;//��У�������˵�����ݿ���
				
				CmdID = (ReadFromUsart[6] << 8 | ReadFromUsart[5]);
				//��������������,�����ݿ�������Ӧ�ṹ����(ע�⿽�����ݵĳ���)
				switch(CmdID)
				{
					case ID_game_state:        			//0x0001
						memcpy(&GameState, (ReadFromUsart + DATA), LEN_game_state);
					break;
					
					case ID_game_result:          		//0x0002
						memcpy(&GameResult, (ReadFromUsart + DATA), LEN_game_result);
					break;
					
					case ID_game_robot_survivors:       //0x0003
						memcpy(&GameRobotSurvivors, (ReadFromUsart + DATA), LEN_game_robot_survivors);
					break;
					
					case ID_event_data:    				//0x0101
						memcpy(&EventData, (ReadFromUsart + DATA), LEN_event_data);
					break;
					
					case ID_supply_projectile_action:   //0x0102
						memcpy(&SupplyProjectileAction, (ReadFromUsart + DATA), LEN_supply_projectile_action);
					break;
					
					case ID_supply_projectile_booking:  //0x0103
						memcpy(&SupplyProjectileBooking, (ReadFromUsart + DATA), LEN_supply_projectile_booking);
					break;
					
					case ID_game_robot_state:      		//0x0201
						memcpy(&GameRobotStat, (ReadFromUsart + DATA), LEN_game_robot_state);
					break;
					
					case ID_power_heat_data:      		//0x0202
						memcpy(&PowerHeatData, (ReadFromUsart + DATA), LEN_power_heat_data);
					break;
					
					case ID_game_robot_pos:      		//0x0203
						memcpy(&GameRobotPos, (ReadFromUsart + DATA), LEN_game_robot_pos);
					break;
					
					case ID_buff_musk:      			//0x0204
						memcpy(&BuffMusk, (ReadFromUsart + DATA), LEN_buff_musk);
					break;
					
					case ID_aerial_robot_energy:      	//0x0205
						memcpy(&AerialRobotEnergy, (ReadFromUsart + DATA), LEN_aerial_robot_energy);
					break;
					
					case ID_robot_hurt:      			//0x0206
						memcpy(&RobotHurt, (ReadFromUsart + DATA), LEN_robot_hurt);
						if(RobotHurt.hurt_type == 0)//��װ�װ���������˺�
						{	Hurt_Data_Update = true;	}//װ������ÿ����һ�����ж�Ϊ�ܵ�һ���˺�
					break;
					
					case ID_shoot_data:      			//0x0207
						memcpy(&ShootData, (ReadFromUsart + DATA), LEN_shoot_data);
						JUDGE_ShootNumCount();//������ͳ
					break;
				}
				//�׵�ַ��֡����,ָ��CRC16��һ�ֽ�,�����ж��Ƿ�Ϊ0xA5,�����ж�һ�����ݰ��Ƿ��ж�֡����
				if(*(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL) == 0xA5)
				{
					//���һ�����ݰ������˶�֡����,���ٴζ�ȡ
					Judge_Read_Data(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL);
				}
			}
		}
		//�׵�ַ��֡����,ָ��CRC16��һ�ֽ�,�����ж��Ƿ�Ϊ0xA5,�����ж�һ�����ݰ��Ƿ��ж�֡����
		if(*(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL) == 0xA5)
		{
			//���һ�����ݰ������˶�֡����,���ٴζ�ȡ
			Judge_Read_Data(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL);
		}
	}
	
	if (retval_tf == true)
	{
		Judge_Data_TF = true;//����������
	}
	else		//ֻҪCRC16У�鲻ͨ����Ϊfalse
	{
		Judge_Data_TF = false;//����������
	}
	
	return retval_tf;//����������������
}

/**
  * @brief  �ϴ��Զ�������
  * @param  void
  * @retval void
  * @attention  ���ݴ��,�����ɺ�ͨ�����ڷ��͵�����ϵͳ
  */
#define send_max_len     200
unsigned char CliendTxBuffer[send_max_len];
void JUDGE_Show_Data(void)
{
//	static u8 datalength,i;
//	uint8_t judge_led = 0xff;//��ʼ��ledΪȫ��
//	static uint8_t auto_led_time = 0;
//	static uint8_t buff_led_time = 0;
//	
//	determine_ID();//�жϷ�����ID�����Ӧ�Ŀͻ���ID
//	
//	ShowData.txFrameHeader.SOF = 0xA5;
//	ShowData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(client_custom_data_t);
//	ShowData.txFrameHeader.Seq = 0;
//	memcpy(CliendTxBuffer, &ShowData.txFrameHeader, sizeof(xFrameHeader));//д��֡ͷ����
//	Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(xFrameHeader));//д��֡ͷCRC8У����
//	
//	ShowData.CmdID = 0x0301;
//	
//	ShowData.dataFrameHeader.data_cmd_id = 0xD180;//�����ͻ��˵�cmd,�ٷ��̶�
//	//ID�Ѿ����Զ���ȡ����
//	ShowData.dataFrameHeader.send_ID 	 = Judge_Self_ID;//�����ߵ�ID
//	ShowData.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
	
	/*- �Զ������� -*/
//	ShowData.clientData.data1 = (float)Capvoltage_Percent();//����ʣ�����
//	ShowData.clientData.data2 = (float)Base_Angle_Measure();//����ǶȲ�
//	ShowData.clientData.data3 = GIMBAL_PITCH_Judge_Angle();//��̨̧ͷ�Ƕ�
//	
//	/***************Ť��ָʾ****************/
//	if(Chassis_IfCORGI() == true)//Ťƨ��ģʽ,��һ�ŵ�����
//	{
//		judge_led &= 0xfe;//��1λ��0,���
//	}
//	else
//	{
//		judge_led |= 0x01;//��1λ��1
//	}
//	
//	/****************45��ģʽ****************/
//	if(Chassis_IfPISA() == true)//45��ģʽ���ڶ��ŵ�����
//	{
//		judge_led &= 0xfd;//��2λ��0,���
//	}
//	else
//	{
//		judge_led |= 0x02;//��2λ��1
//	}
//	
//	/***************����ָʾ****************/
//	if(VISION_isColor() == ATTACK_RED)//�����
//	{
//		judge_led &= 0xfb;//��3λ��0,���
//	}
//	else if(VISION_isColor() == ATTACK_BLUE)//������ɫ
//	{
//		judge_led |= 0x04;//��3λ��1
//	}
//	else//������
//	{
//		auto_led_time++;
//		if(auto_led_time > 3)
//		{
//			(judge_led)^=(1<<2);//��3λȡ��,��˸
//			auto_led_time = 0;
//		}
//	}
//	
//	/****************���ָʾ***************/
//	if(VISION_BuffType() == VISION_RBUFF_CLOCKWISE	//˳ʱ��
//			|| VISION_BuffType() == VISION_BBUFF_CLOCKWISE)
//	{
//		judge_led &= 0xf7;//��4λ��0,���
//	}
//	else if(VISION_BuffType() == VISION_RBUFF_ANTI	//��ʱ��
//			|| VISION_BuffType() == VISION_BBUFF_ANTI)
//	{
//		judge_led |= 0x08;//��4λ��1
//	}
//	else//�����
//	{
//		buff_led_time++;
//		if(buff_led_time > 3)
//		{
//			(judge_led)^=(1<<3);//��4λȡ��,��˸
//			buff_led_time = 0;
//		}
//	}
//	/*--------------*/
//	ShowData.clientData.masks = judge_led;//0~5λ0���,1�̵�
//	
//	//���д�����ݶ�
//	memcpy(	
//			CliendTxBuffer + 5, 
//			(uint8_t*)&ShowData.CmdID, 
//			(sizeof(ShowData.CmdID)+ sizeof(ShowData.dataFrameHeader)+ sizeof(ShowData.clientData))
//		  );			
//			
//	Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(ShowData));//д�����ݶ�CRC16У����	

//	datalength = sizeof(ShowData); 
//	for(i = 0;i < datalength;i++)
//	{
//		USART_SendData(UART5,(uint16_t)CliendTxBuffer[i]);
//		while(USART_GetFlagStatus(UART5,USART_FLAG_TC)==RESET);
//	}	 
}

/**
  * @brief  �������ݸ�����
  * @param  void
  * @retval void
  * @attention  
  */
#define Teammate_max_len     200
unsigned char TeammateTxBuffer[Teammate_max_len];
//bool Send_Color = 0;
//bool First_Time_Send_Commu = false;
uint16_t send_time = 0;
void Send_to_Teammate(void)
{
//	static u8 datalength,i;
//	
//	Send_Color = is_red_or_blue();//�жϷ��͸��ڱ�����ɫ,17�ڱ�(��),7�ڱ�(��)��
//	
//	memset(TeammateTxBuffer,0,200);
//	
//	CommuData.txFrameHeader.SOF = 0xA5;
//	CommuData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(robot_interactive_data_t);
//	CommuData.txFrameHeader.Seq = 0;
//	memcpy(TeammateTxBuffer, &CommuData.txFrameHeader, sizeof(xFrameHeader));
//	Append_CRC8_Check_Sum(TeammateTxBuffer, sizeof(xFrameHeader));	
//	
//	CommuData.CmdID = 0x0301;
//	
//	   
//	CommuData.dataFrameHeader.send_ID = Judge_Self_ID;//�����ߵ�ID
//	
//	
//	if( Senty_Run() == true)
//	{
//		Senty_Run_Clean_Flag();
//		First_Time_Send_Commu = true;
//	}
//	
//	if( First_Time_Send_Commu == true )
//	{
//		CommuData.dataFrameHeader.data_cmd_id = 0x0292;//��0x0200-0x02ff֮��ѡ��
//		send_time++;
//		if(send_time >= 20)
//		{
//			First_Time_Send_Commu = false;
//		}
//		if(Send_Color == BLUE)//�Լ��������������ڱ�
//		{
//			CommuData.dataFrameHeader.receiver_ID = 17;//������ID
//		}
//		else if(Send_Color == RED)//�Լ��Ǻ죬�������ڱ�
//		{
//			CommuData.dataFrameHeader.receiver_ID = 7;//������ID
//		}
//	}
//	else
//	{
//		CommuData.dataFrameHeader.data_cmd_id = 0x0255;
//		send_time = 0;
//		CommuData.dataFrameHeader.receiver_ID = 88;//������ID��������
//	}
//	
//	CommuData.interactData.data[0] = 0;//���͵����� //��С��Ҫ���������ı�������   
//	
//	memcpy(TeammateTxBuffer+5,(uint8_t *)&CommuData.CmdID,(sizeof(CommuData.CmdID)+sizeof(CommuData.dataFrameHeader)+sizeof(CommuData.interactData)));		
//	Append_CRC16_Check_Sum(TeammateTxBuffer,sizeof(CommuData));
//	
//	datalength = sizeof(CommuData); 
//	if( First_Time_Send_Commu == true )
//	{
//		for(i = 0;i < datalength;i++)
//		{
//			USART_SendData(UART5,(uint16_t)TeammateTxBuffer[i]);
//			while(USART_GetFlagStatus(UART5,USART_FLAG_TC)==RESET);
//		}	 
//	}
}

/**
  * @brief  �ж��Լ�������
  * @param  void
  * @retval RED   BLUE
  * @attention  ���ݴ��,�����ɺ�ͨ�����ڷ��͵�����ϵͳ
  */
bool Color;
bool is_red_or_blue(void)
{
	Judge_Self_ID = GameRobotStat.robot_id;//��ȡ��ǰ������ID
	
	if(GameRobotStat.robot_id > 10)
	{
		return BLUE;
	}
	else 
	{
		return RED;
	}
}

/**
  * @brief  �ж�����ID��ѡ��ͻ���ID
  * @param  void
  * @retval RED   BLUE
  * @attention  ���ݴ��,�����ɺ�ͨ�����ڷ��͵�����ϵͳ
  */
void determine_ID(void)
{
	Color = is_red_or_blue();
	if(Color == BLUE)
	{
		Judge_SelfClient_ID = 0x0110 + (Judge_Self_ID-10);//����ͻ���ID
	}
	else if(Color == RED)
	{
		Judge_SelfClient_ID = 0x0100 + Judge_Self_ID;//����ͻ���ID
	}
}

/********************�������ݸ����жϺ���***************************/

/**
  * @brief  �����Ƿ����
  * @param  void
  * @retval  true����   false������
  * @attention  �ڲ��ж�ȡ������ʵʱ�ı䷵��ֵ
  */
bool JUDGE_sGetDataState(void)
{
	return Judge_Data_TF;
} 

/**
  * @brief  ��ȡ˲ʱ����
  * @param  void
  * @retval ʵʱ����ֵ
  * @attention  
  */
float JUDGE_fGetChassisPower(void)
{
	return (PowerHeatData.chassis_power);
}

/**
  * @brief  ��ȡʣ�ཹ������
  * @param  void
  * @retval ʣ�໺�役������(���60)
  * @attention  
  */
uint16_t JUDGE_fGetRemainEnergy(void)
{
	return (PowerHeatData.chassis_power_buffer);
}

/**
  * @brief  ��ȡ��ǰ�ȼ�
  * @param  void
  * @retval ��ǰ�ȼ�
  * @attention  
  */
uint8_t JUDGE_ucGetRobotLevel(void)
{
    return	GameRobotStat.robot_level;
}

/**
  * @brief  ��ȡǹ������
  * @param  void
  * @retval 17mm
  * @attention  ʵʱ����
  */
uint16_t JUDGE_usGetRemoteHeat17(void)
{
	return PowerHeatData.shooter_heat0;
}

/**
  * @brief  ��ȡ����
  * @param  void
  * @retval 17mm
  * @attention  ʵʱ����
  */
float JUDGE_usGetSpeedHeat17(void)
{
	return ShootData.bullet_speed;
}

/**
  * @brief  ͳ�Ʒ�����
  * @param  void
  * @retval void
  * @attention  
  */
portTickType shoot_time;//������ʱ����
portTickType shoot_ping;//����������շ����ӳ�
float Shoot_Speed_Now = 0;
float Shoot_Speed_Last = 0;
void JUDGE_ShootNumCount(void)
{
	Shoot_Speed_Now = ShootData.bullet_speed;
	if(Shoot_Speed_Last != Shoot_Speed_Now)//��Ϊ��float�ͣ�������������ȫ���,�����ٶȲ���ʱ˵��������һ�ŵ�
	{
		ShootNum++;
		Shoot_Speed_Last = Shoot_Speed_Now;
	}
	shoot_time = xTaskGetTickCount();//��ȡ���跢��ʱ��ϵͳʱ��
//	shoot_ping = shoot_time - REVOL_uiGetRevolTime();//�����ӳ�
}

/**
  * @brief  ��ȡ������
  * @param  void
  * @retval ������
  * @attention ��������˫ǹ��
  */
uint16_t JUDGE_usGetShootNum(void)
{
	return ShootNum;
}

/**
  * @brief  ����������
  * @param  void
  * @retval void
  * @attention 
  */
void JUDGE_ShootNum_Clear(void)
{
	ShootNum = 0;
}

/**
  * @brief  ��ȡǹ������
  * @param  void
  * @retval ��ǰ�ȼ�17mm��������
  * @attention  
  */
uint16_t JUDGE_usGetHeatLimit(void)
{
	return GameRobotStat.shooter_heat0_cooling_limit;
}

/**
  * @brief  ��ǰ�ȼ���Ӧ��ǹ��ÿ����ȴֵ
  * @param  void
  * @retval ��ǰ�ȼ�17mm��ȴ�ٶ�
  * @attention  
  */
uint16_t JUDGE_usGetShootCold(void)
{
	return GameRobotStat.shooter_heat0_cooling_rate;
}

/****************�����Զ������ж���*******************/
/**
  * @brief  װ�װ��˺������Ƿ����
  * @param  void
  * @retval true�Ѹ���   falseû����
  * @attention  
  */
bool JUDGE_IfArmorHurt(void)
{
	static portTickType ulCurrent = 0;
	static uint32_t ulDelay = 0;
	static bool IfHurt = false;//Ĭ��װ�װ崦������״̬

	
	ulCurrent = xTaskGetTickCount();

	if (Hurt_Data_Update == true)//װ�װ����ݸ���
	{
		Hurt_Data_Update = false;//��֤���жϵ��´�װ�װ��˺�����
		ulDelay = ulCurrent + TIME_STAMP_200MS;//
		IfHurt = true;
	}
	else if (ulCurrent > ulDelay)//
	{
		IfHurt = false;
	}
	
	return IfHurt;
}

bool Judge_If_Death(void)
{
	if(GameRobotStat.remain_HP == 0 && JUDGE_sGetDataState() == true)
	{
		return true;
	}
	else
	{
		return false;
	}
}



