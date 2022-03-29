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
  * @��  ��  ���PID����
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_PID_H
#define __AX_PID_H

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f10x.h"

//OpenCRP���ܺ���
int16_t AX_PID_MotorVelocityCtlA(int16_t spd_target, int16_t spd_current);   //PID���ƺ��������A
int16_t AX_PID_MotorVelocityCtlB(int16_t spd_target, int16_t spd_current);   //PID���ƺ��������B
int16_t AX_PID_MotorVelocityCtlC(int16_t spd_target, int16_t spd_current);   //PID���ƺ��������C
int16_t AX_PID_MotorVelocityCtlD(int16_t spd_target, int16_t spd_current);   //PID���ƺ��������D

#endif


