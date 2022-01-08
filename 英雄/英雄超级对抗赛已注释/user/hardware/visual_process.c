#include "main.h"
#include "gimbal.h"


//�Ӿ����ݽ���  ��һ���ֶ������㷨����ͨ��Э�飬Ȼ��������ݽ����������̨�������
extern RecData  rec_data;

void vision_process(uint8_t *ReadFromUsart)
{
	


    int FirstIndex = -1;
    int LastIndex = -1;
    for(int i=0;i<15;i++){
        	
        if(ReadFromUsart[i] == 0xaa&&FirstIndex == -1){
            FirstIndex = i;
						
					
        }else if(ReadFromUsart[i] == 0xbb&&FirstIndex != -1&&i - FirstIndex == 14){
            LastIndex = i;
				
            break;
        }
    }
		//���մ������ݺ���
		if(FirstIndex != -1&&LastIndex != -1)
		{ 
			
		  
			rec_data.Rec_pitch_data.c[0]=ReadFromUsart[FirstIndex+1];
			rec_data.Rec_pitch_data.c[1]=ReadFromUsart[FirstIndex+2];
			rec_data.Rec_pitch_data.c[2]=ReadFromUsart[FirstIndex+3];
			rec_data.Rec_pitch_data.c[3]=ReadFromUsart[FirstIndex+4];
			
			rec_data.Rec_yaw_data.c[0]=ReadFromUsart[FirstIndex+6];
			rec_data.Rec_yaw_data.c[1]=ReadFromUsart[FirstIndex+7];
			rec_data.Rec_yaw_data.c[2]=ReadFromUsart[FirstIndex+8];
			rec_data.Rec_yaw_data.c[3]=ReadFromUsart[FirstIndex+9];
			
			//������ת����float����
			if(ReadFromUsart[FirstIndex+5] == 0x00){
            rec_data.Rec_yaw_data.f = -fabs(rec_data.Rec_yaw_data.f);
        }else{
            rec_data.Rec_yaw_data.f = fabs(rec_data.Rec_yaw_data.f);
        }
        if(ReadFromUsart[FirstIndex+10] == 0x00){
            rec_data.Rec_pitch_data.f = -fabs(rec_data.Rec_pitch_data.f);
        }else{
            rec_data.Rec_pitch_data.f = fabs(rec_data.Rec_pitch_data.f);
        }
         // sum =  rec_data.Rec_pitch_data.f;
				
				//�Ƿ�ʶ������
        if(ReadFromUsart[FirstIndex+11] == 0x00){
            rec_data.Is_Identify = false;
        }else{
           rec_data.Is_Identify = true;
        }
				
			rec_data.distance = ReadFromUsart[FirstIndex+12]/10*16+ReadFromUsart[FirstIndex+12]%10;

		 if(ReadFromUsart[FirstIndex+13] == 0x00){
            rec_data.Is_shoot = false;
        }else{
           rec_data.Is_shoot = true;
        }
				 
		}

}

