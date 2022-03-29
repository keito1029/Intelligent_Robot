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
  * @��  ��  2019-8-8
  * @��  ��  �������˶�ѧ����
  *
  ******************************************************************************
  * @˵  ��
  *  
  * 
  ******************************************************************************
  */


#include "ax_kinematics.h"
#include <stdio.h>
#include "ax_delay.h"
#include <math.h>


//��������
int32_t  current_count[4] = {0};
double    ticks_per_meter = 0;
double   linear_correction_factor = 1.0;
double   angular_correction_factor = 1.0;
int32_t  wheel_mult[4] = {0};
float  wheel_track_cali = 0.3;

extern int16_t robot_odom[6];
extern int16_t robot_target_speed[3];

/**
  * @��  ��  �������˶���������
  * @��  ��  ��
  * @����ֵ  ��
  */
void AX_Kinematics_Init(int16_t* robot_params)
{

	linear_correction_factor    = (float)robot_params[0]/1000;
  angular_correction_factor   = (float)robot_params[1]/1000;
	wheel_track_cali = WHEEL_TRACK/angular_correction_factor;


	robot_odom[0]  = 0;
	robot_odom[1]  = 0;
	robot_odom[2]  = 0;

	ticks_per_meter    = (float)ENCODER_RESOLUTION/(WHEEL_DIAMETER*3.1415926*linear_correction_factor);		
}

/**
  * @��  ��  �����˶�ѧ���������������ٶ�->�����ٶ�
  * @��  ��  input:  robot_target_speed[]  �����������ٶ� m/s*1000
  *          output��ax_encoder_delta_target[] ��������ٶ� count
  * @����ֵ  ��
  */
void AX_Kinematics_Inverse(int16_t* input, int16_t* output)
{
	float x_speed   = ((float)input[0])/1000;
	float y_speed   = ((float)input[1])/1000;
	float yaw_speed = ((float)input[2])/1000;
	static float wheel_velocity[4] = {0};
	
	wheel_velocity[0] = -y_speed + x_speed - (wheel_track_cali)*yaw_speed;
	wheel_velocity[1] = y_speed + x_speed + (wheel_track_cali)*yaw_speed;
	wheel_velocity[2] = y_speed + x_speed - (wheel_track_cali)*yaw_speed;
	wheel_velocity[3] = -y_speed + x_speed + (wheel_track_cali)*yaw_speed;

	output[0] = (int16_t)(wheel_velocity[0] * ticks_per_meter/PID_RATE);
	output[1] = (int16_t)(wheel_velocity[1] * ticks_per_meter/PID_RATE);
	output[2] = (int16_t)(wheel_velocity[2] * ticks_per_meter/PID_RATE);
	output[3] = (int16_t)(wheel_velocity[3] * ticks_per_meter/PID_RATE);
}

/**
  * @��  ��  �����˶�ѧ���������ӱ���ֵ->����������̼�����
  * @��  ��  input: ax_encoder[]  �������ۼ�ֵ
  *          output: robot_odom[] ������̼� x y yaw
  * @����ֵ  ��
  */
void AX_Kinematics_Forward(int16_t* input, int16_t* output)
{
	static double delta_count[4];  
  static double delta_v_ave[3];
	static double delta_v_integral[2];
	static int16_t recv_count[4];
	
	recv_count[0] = -input[0];
	recv_count[1] = input[1];
	recv_count[2] = -input[2];
	recv_count[3] = input[3];
	
		//�����������������
	for(int i=0;i<4;i++)
	{
			if(recv_count[i] < ENCODER_LOW_WRAP && current_count[i] > ENCODER_HIGH_WRAP)
				wheel_mult[i]++;
			else if(recv_count[i] > ENCODER_HIGH_WRAP && current_count[i] < ENCODER_LOW_WRAP)
				wheel_mult[i]--;
			else
				wheel_mult[i]=0;
	}
	//	printf("%d %d %d %d\r\n",wheel_mult[0],wheel_mult[1],wheel_mult[2],wheel_mult[3]);
	//����������ֵת��Ϊǰ���ľ��룬��λm
	for(int i=0;i<4;i++)
	{	
			delta_count[i] = 1.0*(recv_count[i] + wheel_mult[i]*(ENCODER_MAX-ENCODER_MIN)-current_count[i])/ticks_per_meter;
			current_count[i] = recv_count[i];
	}
		//�������x��仯����m��Yaw�ᳯ��仯rad
	delta_v_ave[0] = (delta_count[3] - delta_count[2])/2.0;
	delta_v_ave[1] = (delta_count[2] - delta_count[0])/2.0;
	delta_v_ave[2] = (delta_count[2]+delta_count[1])/(2*wheel_track_cali);
		//�����������ϵ�µ�x����Yaw����ٶ�
	delta_v_integral[0] = cos(delta_v_ave[2])*delta_v_ave[0] - sin(delta_v_ave[2])*delta_v_ave[1];
	delta_v_integral[1] = -sin(delta_v_ave[2])*delta_v_ave[0] - cos(delta_v_ave[2])*delta_v_ave[1];


	
		//���ּ�����̼�����ϵ(odom_frame)�µĻ�����X,Y,Yaw������
	output[0] += (int16_t)((cos((double)output[2]/1000)*delta_v_integral[0] - sin((double)output[2]/1000)*delta_v_integral[1])*1000);
	output[1] += (int16_t)((sin((double)output[2]/1000)*delta_v_integral[0] + cos((double)output[2]/1000)*delta_v_integral[1])*1000);
	output[2] += (int16_t)(delta_v_ave[2]*1000);
		
    //Yaw������仯��Χ����-2�� -> 2��
		if(output[2] > PI*1000)
			output[2] -= 2*PI*1000;
		else if(output[2] < -PI*1000)
			output[2] += 2*PI*1000;
		
		//���ͻ�����X��Yaw���ٶȷ���
	output[3] = (int16_t)(delta_v_ave[0]*1000);
	output[4] = (int16_t)(-delta_v_ave[1]*1000);
	output[5] = (int16_t)(delta_v_ave[2]*1000);
}


/******************* (C) ��Ȩ 2019 XTARK **************************************/
