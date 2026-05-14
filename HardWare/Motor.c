#include "Motor.h"
#include "stdio.h"
#include "adc.h"
#include "usart.h"
#include "rs485.h"
//motor1&2：MCP内收外展+伸直；motor3：PIP伸直；motor4：总弯曲*	
Motor_Struct motor[Motor_Num];
PID_Increment_Struct PID_Speed[Motor_Num];//电机出轴转速PID参数
PID_Increment_Struct PID_Position[Motor_Num];//电机出轴转速PID参数
PID_Increment_Struct PID_Angle[Motor_Num];//电机出轴转过的角度PID参数

//电机：实测给到目标值359.2°，实际转过约为1圈，所以认为一圈是359.2°

//标定数据集，共4行（4个关节角度/4路AD）-CalibrationLEN列（列数越多越精细）
//左手-大拇指标定结果（前三行数据有效）：
static float JointAngle[4][CalibrationLEN]={{0,90},
											{0,90},
											{0,90},
											{0,90}};//x均升序，查找y，对应标志位11
static float ADValue[4][CalibrationLEN]={{2420,1310},//关节1
										 {1510,2555},//关节2
										 {2390,1310},//关节3
										 {1610,2675}};//关节4	

////右手-大拇指标定结果（前三行数据有效）：
//static float JointAngle[4][CalibrationLEN]={{0,75},
//											{0,95},
//											{0,100},
//											{0,0}};//x均升序，查找y，对应标志位11
//static float ADValue[4][CalibrationLEN]={{1400,550},//关节1
//										 {2500,1460},//关节2
//										 {1670,2690},//关节3
//										 {0,0}};//关节4	
										 
static int InterpolationFlag[4]={11,11,11,11};//（10：已知x=data且x降序；11：已知x=data且x升序——查找y）（20：已知y=data且y降序；21：已知y=data且y升序——查找x）

//*HALL_AD值的全局变量
uint16_t ADC_HallValue[Hall_Num] = {0};//经平均滤波后的4个通道各自的测量值
//利用手指关节位置传感器做PID，对应的PID参数
float Kp_Position[Hall_Num]={5, 5, 5, 5};
float Ki_Position[Hall_Num]={0, 0, 0, 0};
float Kd_Position[Hall_Num]={0, 0, 0, 0};//关节位置PID

//13mm大朗络电机3500RPM——PID参数
//float Kp_Angle[Motor_Num]={0.5, 0.5, 0.5, 0.5};
//float Ki_Angle[Motor_Num]={0, 0, 0, 0};
//float Kd_Angle[Motor_Num]={0, 0, 0, 0};//电机出轴角度PID
//float Kp_Speed[Motor_Num]={0.4, 0.4, 0.4, 0.4};
//float Ki_Speed[Motor_Num]={0.1, 0.1, 0.1, 0.1};
//float Kd_Speed[Motor_Num]={0, 0, 0, 0};//电机速度PID

//10mm小朗络电机5000RPM——PID参数
float Kp_Angle[Motor_Num]={0.5, 0.5, 0.5, 0.5};
float Ki_Angle[Motor_Num]={0, 0, 0, 0};
float Kd_Angle[Motor_Num]={0, 0, 0, 0};//电机出轴角度PID
float Kp_Speed[Motor_Num]={0.1, 0.1, 0.1, 0.1};
float Ki_Speed[Motor_Num]={0.1, 0.1, 0.1, 0.1};
float Kd_Speed[Motor_Num]={0, 0, 0, 0};//电机速度PID

//*打开各个定时器，并且开启各项功能和参数初始化*
void Motor_Init(void)
{
	//*开启PWM定时器 + Encoder定时器 + 10ms中断定时器*
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);//开启TIM1的PWM输出
	
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);//开启TIM5的PWM输出	
	
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);//开启TIM2的Encoder
    HAL_TIM_Base_Start_IT(&htim2);//开启Encoder更新中断,防溢出处理	
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);//开启TIM8的Encoder
    HAL_TIM_Base_Start_IT(&htim3);//开启Encoder更新中断,防溢出处理	
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);//开启TIM4的Encoder
    HAL_TIM_Base_Start_IT(&htim4);//开启Encoder更新中断,防溢出处理	
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);//开启TIM8的Encoder
    HAL_TIM_Base_Start_IT(&htim8);//开启Encoder更新中断,防溢出处理	
	
	HAL_TIM_Base_Start_IT(&htim6);//开启TIM6的10ms中断
	
	//*赋初值*
	//*根据目标关节AD与当前关节AD，求差值可得电机目标出轴角度*
//	motor[0].TargetAngle = 268 * Serial_RxPacket[0];
//	motor[1].TargetAngle = 268 * Serial_RxPacket[1];	
//	motor[2].TargetAngle = 268 * Serial_RxPacket[2];
//	motor[3].TargetAngle = 268 * Serial_RxPacket[3];	
	
//	motor[0].TargetAngle = Cal_Angle_1(TargetPosition[0],TargetPosition[1]) - Cal_Angle_1(ADC_HallValue[0],ADC_HallValue[1]);
//	motor[1].TargetAngle = Cal_Angle_2(TargetPosition[0],TargetPosition[1]) - Cal_Angle_2(ADC_HallValue[0],ADC_HallValue[1]);	
//	motor[2].TargetAngle = Cal_Angle_3(TargetPosition[2]) - Cal_Angle_3(ADC_HallValue[2]);
	//*循环设置位置-角度-速度PID值，并清零电机当前出轴角度&速度*
	uint8_t i=0;
	for(i=0;i<Motor_Num;i++)
	{
		//*清零电机当前角度&速度*
		motor[i].CurrentAngle = 0;
		motor[i].CurrentSpeed = 0;
		//*设置电机角度式PID值*
		PID_Angle[i].Kp = Kp_Angle[i];
		PID_Angle[i].Ki = Ki_Angle[i];
		PID_Angle[i].Kd = Kd_Angle[i];
		PID_Angle[i].Error_Last1 = 0;
		PID_Angle[i].Error_Last2 = 0;
		PID_Angle[i].Out_Last = 0;
		//*设置电机速度式PID值*	
		PID_Speed[i].Kp = Kp_Speed[i];
		PID_Speed[i].Ki = Ki_Speed[i];
		PID_Speed[i].Kd = Kd_Speed[i];
		PID_Speed[i].Error_Last1 = 0;
		PID_Speed[i].Error_Last2 = 0;
		PID_Speed[i].Out_Last = 0;	
	}
	for(i=0;i<Hall_Num;i++)
	{
		//*设置手指关节位置PID值*
		PID_Position[i].Kp = Kp_Position[i];
		PID_Position[i].Ki = Ki_Position[i];
		PID_Position[i].Kd = Kd_Position[i];
		PID_Position[i].Error_Last1 = 0;
		PID_Position[i].Error_Last2 = 0;
		PID_Position[i].Out_Last = 0;
	}
}
//*1ms的485接收+10ms的PID计算——定时中断处理函数*
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	int16_t set_speed[Motor_Num] = {0};
	
	if(htim->Instance == htim7.Instance){//【1】若进入1ms的485接收中断
		if(modbus.timrun != 0){//运行时间！=0表明已经收到了第二个字节的数据
			modbus.timout++;
			if(modbus.timout >=8){//设定为8ms后，接收完毕
				modbus.timrun = 0;//运行时间清零
				modbus.reflag = 1;//接收数据完毕，标志位modbus.reflag置1
			}
		}
 		modbus.Host_Sendtime++;//发送完上一帧后的时间计数
 		if(modbus.Host_Sendtime>1000){//距离发送上一帧数据过去1s
			//1s时间到
			modbus.Host_time_flag=1;//发送数据标志位置1	
 		}
	}	

	if(htim->Instance == htim6.Instance)//【2】若进入10ms的PID计算中断
	{
	    //*对Motor.Num个电机做PID计算*	 		
		uint8_t i;
			static int delay = 0;
		for(i=0;i<4;i++){
			motor[i].IOFlag  = Reg[4]%256;//*启停标志位——Reg[4]低八位*
			motor[i].CurrentAngle += Get_Angle(i);//*采集电机1234当前出轴角度*
			motor[i].CurrentSpeed = Get_Speed(i);//*采集电机1234当前出轴转速*
			motor [i].CurrentPosition = ADC_HallValue[i];//*采集关节1234当前霍尔值*
			
			if((Reg[4]/256)==1){//*【2-1】运行模式位——Reg[4]高八位，为1时，顺时针正转的出轴角度PID*
				motor[i].TargetAngle = Real_OneTurn * (Reg[i]/256);//*电机i目标出轴角度——Reg[i]高八位*
				if(motor[i].TargetAngle >= Real_OneTurn * 50)motor[i].TargetAngle = Real_OneTurn * 50;//最多转+50圈【拇指三关节电机】
				else if(motor[i].TargetAngle <= -Real_OneTurn * 50)motor[i].TargetAngle = -Real_OneTurn * 50;//最多转-50圈【拇指三关节电机】
			    motor[i].TargetSpeed = PID_Increment(&PID_Angle[i], motor[i].CurrentAngle, motor[i].TargetAngle);//角度式PID
				if(motor[i].TargetSpeed > Real_MaxSpeed * (Reg[i]%256))motor[i].TargetSpeed = Real_MaxSpeed * (Reg[i]%256);//*电机i目标速度——Reg[i]低八位*
				else if(motor[i].TargetSpeed < -Real_MaxSpeed * (Reg[i]%256))motor[i].TargetSpeed = -Real_MaxSpeed * (Reg[i]%256);//设定速度阈值，低8位，范围0~100
//			    if(PID_Angle[i].Error_Last1 > 3*Real_OneTurn)motor[i].TargetSpeed = Real_MaxSpeed * (Reg[i]%256);
//			    else if(PID_Angle[i].Error_Last1 < -3*Real_OneTurn)motor[i].TargetSpeed = -36 * (Reg[i]%256);//大于3圈满速运行
			}
//			else if((Reg[4]/256)==2){//*【2-2】运行模式位——Reg[4]高八位，为2时，关节位置PID*
//				motor[i].TargetPosition = Reg[i];//*i关节：接收且合并目标关节霍尔值的高8位+低8位，电机1~4对应Reg[0]~Reg[3]*
//				motor[i].TargetSpeed = PID_Increment(&PID_Position[i], motor[i].CurrentPosition, motor[i].TargetPosition);//位置式PID
//				if(PID_Position[i].Error_Last1 >= 30)motor[i].TargetSpeed = 80 * Real_MaxSpeed;
//				else if(PID_Position[i].Error_Last1 <= -30)motor[i].TargetSpeed = -80 * Real_MaxSpeed;//目标与实际关节霍尔值，相差超过25 固定以速度MAX2300运行
//				if(PID_Position[i].Error_Last1 <= 5 && PID_Position[i].Error_Last1 >= -5)motor[i].TargetSpeed = 0;
//				if(motor[i].CurrentAngle <-10800 || motor[i].CurrentAngle >10800)motor[i].TargetSpeed = 0;//电机转到+-30圈之外了，紧急急停，防止拉坏结构
////				if(i == 1)motor[i].TargetSpeed = -motor[i].TargetSpeed;//*******如果角度传感器数值与实际关节角度负相关，则需要加这一句*********
//			}
			else if((Reg[4]/256)==3){//*【2-3】运行模式位——Reg[4]高八位，为3时，反转的出轴角度PID*
				motor[i].TargetAngle = -Real_OneTurn * (Reg[i]/256);//*电机i目标出轴角度——Reg[i]高八位*
				if(motor[i].TargetAngle >= Real_OneTurn * 50)motor[i].TargetAngle = Real_OneTurn * 50;//最多转+50圈【拇指三关节电机】
				else if(motor[i].TargetAngle <= -Real_OneTurn * 50)motor[i].TargetAngle = -Real_OneTurn * 50;//最多转-50圈【拇指三关节电机】
			    motor[i].TargetSpeed = PID_Increment(&PID_Angle[i], motor[i].CurrentAngle, motor[i].TargetAngle);//角度式PID
				if(motor[i].TargetSpeed > Real_MaxSpeed * (Reg[i]%256))motor[i].TargetSpeed = Real_MaxSpeed * (Reg[i]%256);
				else if(motor[i].TargetSpeed < -Real_MaxSpeed * (Reg[i]%256))motor[i].TargetSpeed = -Real_MaxSpeed * (Reg[i]%256);//设定速度阈值，低8位，范围0~100
//			    if(PID_Angle[i].Error_Last1 > 3 * Real_OneTurn)motor[i].TargetSpeed = Real_MaxSpeed * (Reg[i]%256);//*电机i目标速度——Reg[i]低八位*
//			    else if(PID_Angle[i].Error_Last1 < -3 * Real_OneTurn)motor[i].TargetSpeed = -Real_MaxSpeed * (Reg[i]%256);//大于3圈满速运行
			}
			else if((Reg[4]/256)==4){//*【2-4】运行模式位——Reg[4]高八位，为4时,编码器圈数清零*
			    motor[i].CurrentAngle = 0;motor[i].TargetAngle = 0;
				motor[i].CurrentSpeed = 0;motor[i].TargetSpeed = 0;//全部清零，并且急停下来
			}
			else if((Reg[4]/256)==5){//*【2-5】运行模式位——Reg[4]高八位，为5时，关节位置PID，带AD-JointAngle插值，接收为目标关节角度*
				if(Reg[i] <= 5){Reg[i] = 5;}
				else if(Reg[i] >= 100){Reg[i] = 100;}
				motor[i].TargetPosition = lineInterp(JointAngle[i], ADValue[i], CalibrationLEN, Reg[i], InterpolationFlag[i]);//线性插值
				motor[i].TargetSpeed = -PID_Increment(&PID_Position[i], motor[i].CurrentPosition, motor[i].TargetPosition);//位置式PID
				//离目标值太远则定速运行【容易发生抖震，不使用】
//				if(PID_Position[i].Error_Last1 >= 30)motor[i].TargetSpeed = -2500;
//				else if(PID_Position[i].Error_Last1 <= -30)motor[i].TargetSpeed = 2500;//目标与实际关节霍尔值，相差超过50 固定以速度MAX3600运行				
				if(PID_Position[i].Error_Last1 <= 5&&PID_Position[i].Error_Last1 >= -5)motor[i].TargetSpeed = 0;//目标与实际关节AD码值，其相差在5以内停止运行
				if(motor[i].CurrentAngle < -50*Real_OneTurn || motor[i].CurrentAngle > 50*Real_OneTurn)motor[i].TargetSpeed = 0;//电机转到+-50圈之外了，紧急急停，防止拉坏结构
//				motor[i].TargetSpeed = -motor[i].TargetSpeed;//若电机顺时针转则手指弯曲，则需要加这一句
			}			
			//*分割：上面是角度/位置环PID，下面是速度环PID
			set_speed[i] = PID_Increment(&PID_Speed[i], motor[i].CurrentSpeed, motor[i].TargetSpeed);//【3】速度式PID
			if(set_speed[i] >= 1000)set_speed[i] = 1000;
			else if(set_speed[i] <= -1000)set_speed[i] = -1000;//满占空比对应的CCR值为1000，起到限幅作用
//			if(set_speed[i] >= -50 && set_speed[i] <= 50)set_speed[i] = 0;//占空比在5%以内认为已到位置
			Set_Motor(i, set_speed[i], motor[i].IOFlag);//*电机1234做出响应*	0
			
						if(delay <30)
						{
							delay ++;
						}
						
						else
						{
							delay = 0;
							printf("P = %.2f , TP = %.2f , V = %.2f , TV = %.2f",motor[2].CurrentAngle,motor[2].TargetAngle,motor[2].CurrentSpeed,motor[2].TargetSpeed);
						}

			}
		}		
}		
//	else if(htim->Instance == htim2.Instance){}//【2】TIM2 编码器更新中断,即10ms内,脉冲数超过了计数范围
//	else if(htim->Instance == htim8.Instance){}//【2】TIM3 编码器更新中断,即10ms内,脉冲数超过了计数范围
//	else if(htim->Instance == htim4.Instance){}//【2】TIM4 编码器更新中断,即10ms内,脉冲数超过了计数范围
//	else if(htim->Instance == htim8.Instance){}//【2】TIM8 编码器更新中断,即10ms内,脉冲数超过了计数范围


////*电机做出响应*
//void Set_Motor(uint8_t i, int16_t set_speed, uint8_t flag)
//{
//	if(flag == 0)set_speed = 0;
//	switch(i)
//	{
//		case 0:__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 500 + set_speed/2);__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 500 - set_speed/2);break;
//		case 1:__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 500 + set_speed/2);__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 500 - set_speed/2);break;
//		case 2:__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 500 + set_speed/2);__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 500 - set_speed/2);break;
//		case 3:__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 500 + set_speed/2);__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, 500 - set_speed/2);break;
//		default:break;
//	}
//}
//*电机做出响应*：hand4版本的PCB设计(金涛改版)改动
void Set_Motor(uint8_t i, int16_t set_speed, uint8_t flag)
{
	if(flag == 0)set_speed = 0;
	switch(i)
	{
		case 3:__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 500 + set_speed/2);__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 500 - set_speed/2);break;
		case 2:__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 500 + set_speed/2);__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 500 - set_speed/2);break;
		case 1:__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 500 + set_speed/2);__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, 500 - set_speed/2);break;
		case 0:__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 500 + set_speed/2);__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 500 - set_speed/2);break;
		default:break;
	}
}

//***************************************************13mm，3000RPM大朗络电机********************************************************
////*分别获取4输出轴转速：电机1 & 2 & 3 & 4*
//float Get_Speed(uint8_t i)
//{
//    int16_t zj = 0 ;
//	float CurrentSpeed = 0;
//	switch(i)
//	{
//		//为计算转速做转换，单位是RPM（采样周期10ms、电机1/2/3/4减速比3.75:1(15:4)，编码器线数1024、设定是一个周期测4次）
//		case 0:zj = __HAL_TIM_GetCounter(&htim2);__HAL_TIM_SetCounter(&htim2, 0); CurrentSpeed = (zj * 100 * 60 ) / (3.75 * 1024 * 4) ;break;
//		case 1:zj = __HAL_TIM_GetCounter(&htim3);__HAL_TIM_SetCounter(&htim3, 0); CurrentSpeed = (zj * 100 * 60 ) / (3.75 * 1024 * 4) ;break;
//		case 2:zj = __HAL_TIM_GetCounter(&htim4);__HAL_TIM_SetCounter(&htim4, 0); CurrentSpeed = (zj * 100 * 60 ) / (3.75 * 1024 * 4) ;break;
//		case 3:zj = __HAL_TIM_GetCounter(&htim8);__HAL_TIM_SetCounter(&htim8, 0); CurrentSpeed = (zj * 100 * 60 ) / (3.75 * 1024 * 4) ;break;
//		default:break;
//	}	
//    return CurrentSpeed;	
//}

////*分别获取4输出轴转过的角度：电机1 & 2 & 3 & 4*
//float Get_Angle(uint8_t i)
//{
//	int16_t zj = 0 ;
//	float CurrentAngle = 0;
//	switch(i)
//	{
//		//为计算角度做转换，单位是°（一圈360°、电机1/2/3/4减速比3.75:1(15:4)，编码器线数1024、设定是一个周期测4次）
//		case 0:zj = __HAL_TIM_GetCounter(&htim2);CurrentAngle = (zj * 360 ) / (3.75 * 1024 * 4) ;break;
//		case 1:zj = __HAL_TIM_GetCounter(&htim3);CurrentAngle = (zj * 360 ) / (3.75 * 1024 * 4) ;break;
//		case 2:zj = __HAL_TIM_GetCounter(&htim4);CurrentAngle = (zj * 360 ) / (3.75 * 1024 * 4) ;break;
//		case 3:zj = __HAL_TIM_GetCounter(&htim8);CurrentAngle = (zj * 360 ) / (3.75 * 1024 * 4) ;break;
//		default:break;
//	}
//	
//    return CurrentAngle;
//}

//**********************************************10mm，5000RPM小朗络电机******************************************************
//*分别获取4输出轴转速：电机1 & 2 & 3 & 4*
float Get_Speed(uint8_t i)
{
    int16_t zj = 0 ;
	float CurrentSpeed = 0;
	switch(i)
	{
		//为计算转速做转换，单位是RPM（采样周期10ms、电机1/2/3/4减速比4:1，编码器线数1024、设定是一个周期测4次）【360°实测读数18，所以乘以20】
		case 0:zj = __HAL_TIM_GetCounter(&htim2);__HAL_TIM_SetCounter(&htim2, 0); CurrentSpeed = (float)(zj * 100 * 60 *1.15) / (4 * 1024 * 4) ;break;
		case 1:zj = __HAL_TIM_GetCounter(&htim3);__HAL_TIM_SetCounter(&htim3, 0); CurrentSpeed = (float)(zj * 100 * 60 *1.15 ) / (4 * 1024 * 4) ;break;
		case 2:zj = __HAL_TIM_GetCounter(&htim4);__HAL_TIM_SetCounter(&htim4, 0); CurrentSpeed = (float)(zj * 100 * 60 *1.15 ) / (4 * 1024 * 4) ;break;
		case 3:zj = __HAL_TIM_GetCounter(&htim8);__HAL_TIM_SetCounter(&htim8, 0); CurrentSpeed = (float)(zj * 100 * 60 *1.15 ) / (4 * 1024 * 4) ;break;
		default:break;
	}	
    return CurrentSpeed;	
}

//*分别获取4输出轴转过的角度：电机1 & 2 & 3 & 4*
float Get_Angle(uint8_t i)
{
	int16_t zj = 0 ;
	float CurrentAngle = 0;
	switch(i)
	{
		//为计算角度做转换，单位是°（一圈360°、电机1/2/3/4减速比4:1，编码器线数1024、设定是一个周期测4次）【360°实测读数18，所以乘以20】
		case 0:zj = __HAL_TIM_GetCounter(&htim2);CurrentAngle = (float)(zj * 360 *1.15) / (4 * 1024 * 4) ;break;
		case 1:zj = __HAL_TIM_GetCounter(&htim3);CurrentAngle = (float)(zj * 360 *1.15 ) / (4 * 1024 * 4) ;break;
		case 2:zj = __HAL_TIM_GetCounter(&htim4);CurrentAngle = (float)(zj * 360 *1.15 ) / (4 * 1024 * 4) ;break;
		case 3:zj = __HAL_TIM_GetCounter(&htim8);CurrentAngle = (float)(zj * 360 *1.15) / (4 * 1024 * 4) ;break;
		default:break;                                                           
	}
	
    return CurrentAngle;
}

// 分段插值函数（没有考虑超出两端的情况），二分法查找
float lineInterp(float xa[], float ya[], int length, float data, int flag)
	//data为已知的数据；flag为查找类型标志（10为x.data且x降序；11为x.data且x升序）（20为y.data且y降序；21为y.data且y升序）
{
	int low	= 0;
	int high = length - 1;
	int mid = 0;
    switch(flag){
		case 10:{// flag == 10时，data为x且x降序，二分查找区间标号，线性插值得到y	
			if(data >= xa[low])return ya[low];
			else if(data <= xa[high])return ya[high];//超出范围则取靠近的端点值
			else{
				while(high - low != 1){
					mid = (high + low)/2;
					if(xa[mid] < data)high = mid;//降序数组x：目标值在 左半部分				
					else if(xa[mid] > data)low = mid;//降序数组x：目标值在 右半部分				
					else return ya[mid];}
				return ya[low]*(data - xa[high])/(xa[low] - xa[high]) + ya[high]*(data - xa[low])/(xa[high] - xa[low]);
			}		
		}
		case 11:{// flag == 11时，data为x且x升序，二分查找区间标号，线性插值得到y	
			if(data >= xa[high])return ya[high];
			else if(data <= xa[low])return ya[low];//超出范围则取靠近的端点值
			else{
				while(high - low != 1){
					mid = (high + low)/2;
					if(xa[mid] < data)low = mid;//升序数组x：目标值在 右半部分				
					else if(xa[mid] > data)high = mid;//降序数组x：目标值在 左半部分				
					else return ya[mid];}
				return ya[low]*(data - xa[high])/(xa[low] - xa[high]) + ya[high]*(data - xa[low])/(xa[high] - xa[low]);
			}
		}
//		case 20:{// flag == 20时，data为y且y降序，二分查找区间标号，线性插值得到x
//			if(data >= ya[low])return xa[low];
//			else if(data <= ya[high])return xa[high];//超出范围则取靠近的端点值
//			else{
//				while(high - low != 1){
//					mid = (high + low)/2;
//					if(ya[mid] < data)high = mid;//降序数组y：目标值在 左半部分			
//					else if(ya[mid] > data)low = mid;//降序数组y：目标值在 右半部分				
//					else return xa[mid];}
//				return xa[low]*(data - ya[high])/(ya[low] - ya[high]) + xa[high]*(data - ya[low])/(ya[high] - ya[low]);
//			}		
//		}
//		case 21:{// flag == 21时，data为y且y升序，二分查找区间标号，线性插值得到x
//			if(data >= ya[high])return xa[high];
//			else if(data <= ya[low])return xa[low];//超出范围则取靠近的端点值
//			else{
//				while(high - low != 1){
//					mid = (high + low)/2;
//					if(ya[mid] < data)low = mid;//升序数组y：目标值在 右半部分			
//					else if(ya[mid] > data)high = mid;//升序数组y：目标值在 半半部分				
//					else return xa[mid];}
//				return xa[low]*(data - ya[high])/(ya[low] - ya[high]) + xa[high]*(data - ya[low])/(ya[high] - ya[low]);
//			}		
//		}
		default: return 0;//flag值不符合规范，输出0
	}
}

//*增量式PID计算*
float PID_Increment(PID_Increment_Struct *PID, float Current, float Target)
{
    float err,//误差
          out,//输出
          proportion,//比例
          differential;//微分
    err = (float)Target - (float)Current;//计算误差
    proportion = (float)err - (float)PID->Error_Last1;//计算比例项
    differential = (float)err - 2 * (float)PID->Error_Last1 + (float)PID->Error_Last2;//计算微分项
    out = (float)PID->Out_Last + (float)PID->Kp * proportion + (float)PID->Ki * err + (float)PID->Kd * differential;//计算PID本次输出
    
	PID->Error_Last2 = PID->Error_Last1;//更新上上次误差
    PID->Error_Last1 = err;//更新上次误差
    PID->Out_Last = out;//更新上次输出
    return out;
}

