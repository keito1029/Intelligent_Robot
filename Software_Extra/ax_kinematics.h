/**			                                                    
		   ____                    _____ _____  _____        XTARK@���˴���
		  / __ \                  / ____|  __ \|  __ \  
		 | |  | |_ __   ___ _ __ | |    | |__) | |__) |
		 | |  | | '_ \ / _ \ '_ \| |    |  _  /|  ___/ 
		 | |__| | |_) |  __/ | | | |____| | \ \| |     
		  \____/| .__/ \___|_| |_|\_____|_|  \_\_|     
		    		| |                                    
		    		|_|  OpenCRP ��ݮ�� ר��ROS�����˿�����                                   
									 
  ****************************************************************************** 
  *           
  * ��Ȩ���У� XTARK@���˴���  ��Ȩ���У�����ؾ�
  * ������վ�� www.xtark.cn
  * �Ա����̣� https://shop246676508.taobao.com  
  * ����ý�壺 www.cnblogs.com/xtark�����ͣ�
	* ����΢�ţ� ΢�Ź��ںţ����˴��£���ȡ������Ѷ��
  *      
  ******************************************************************************
  * @��  ��  Musk Han@XTARK
  * @��  ��  V1.0
  * @��  ��  2019-7-26
  * @��  ��  �������˶�ѧ����
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_KINEMATICS_H
#define __AX_KINEMATICS_H


/* Includes ------------------------------------------------------------------*/	

#include "stm32f10x.h"

#define ENCODER_MAX 32767        
#define ENCODER_MIN -32768 
#define WHEEL_TRACK             0.2194	 //�־�
#define WHEEL_DIAMETER          0.1292   //�־�
#define ENCODER_RESOLUTION      1440.0     //�������ֱ���
#define PID_RATE                50      //PIDƵ��
#define ROBOT_LINEAR_SPEED_LIMIT 5000   //���������ٶ���ֵ m/s*1000
#define ROBOT_ANGULAR_SPEED_LIMIT 5000  //�����˽��ٶ���ֵ rad/s*1000

#define ENCODER_LOW_WRAP  ((ENCODER_MAX - ENCODER_MIN)*0.3+ENCODER_MIN)
#define ENCODER_HIGH_WRAP ((ENCODER_MAX - ENCODER_MIN)*0.7+ENCODER_MIN)
#define PI 3.1415926

#define ANGLE_CAL(X) (PI-((X)-1500)/2000*PI)

//ROBOT���ܺ���
void AX_Kinematics_Init(int16_t* robot_params);  //������ʼ��
void AX_Kinematics_Forward(int16_t* input, int16_t* output); //����(ForwardKinematics)�����ӱ���ֵ->����������̼�����
void AX_Kinematics_Inverse(int16_t* input, int16_t* output); //���(InverseKinematics)�����������ٶ�->�����ٶ�

#endif
