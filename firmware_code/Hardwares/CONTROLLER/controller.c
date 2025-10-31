#include "controller.h"
#include "imu.h"
#include "pid.h"
#include "encoder.h"
#include "mpu6050.h"
#include "imath.h"
#include "tim.h"
#include "bluetooth.h"
_OUT_Motor Motor1 = {0};
_OUT_Motor Motor2 = {0};


//static uint16_t pwmspeed;
static float Mid = 1.7;                                  //机械中值   占空比中值42.5-42.7
float Vertical_Kp=860;   //1000   2050                              //直立环kp（正）
float Vertical_Kd=3;			//30															//直立环kd（正）
int pwm_out1;																							//直立环输出 负反馈
float Velocity_Kp=-0.07;	//-0.375																		//速度环kp（负）
float Velocity_Ki=-0.0;																			//速度环ki（负）  ki值大约为kp值的200分之一
int pwm_out2;																							//速度环输出 正反馈



/**
  * @brief   外环速度控制器
  */
static void vel_controller(void)                                       
{                            
  all.vel_encoder.expect = 0.0f;                              //外环角度期望角度给0
  all.vel_encoder.feedback = encoderINFO.mainNumberValue;    //编码器速度值作为反馈量   此处根据实际情况进行正负选择 +
	
  pid_controller(&all.vel_encoder);                           //外环pid控制器  
}
/**
  * @brief   内环角度控制器
  */
static void angle_controller(void)                                     
{
  all.rol_angle.expect = all.vel_encoder.out;                 //外环输出作为内环期望值
	//all.rol_angle.expect = -2.5f;                 //外环输出作为内环期望值
  all.rol_angle.feedback = (-att.rol)+(0);                    //姿态解算的rol角作为反馈量
  pid_controller(&all.rol_angle);                             //内环pid控制器      
}
/**
  * @brief   内环角速度控制器
  */
static void gyro_controller(void)                                      
{                            
  all.rol_gyro.expect = all.rol_angle.out;                    //外环输出作为内环期望值
	//all.rol_gyro.expect = -0.0f;                    //外环输出作为内环期望值
  all.rol_gyro.feedback = -(Mpu.deg_s.y);                       //角速度值作为反馈量
  pid_controller(&all.rol_gyro);                              //内环pid控制器  
}
/**
  * @brief   三环串级PID控制器运行
  */
void _controller_perform(void)                                  
{   
  vel_controller();                                           //速度控制器
  angle_controller();                                         //角度控制器
  gyro_controller();                                          //角速度控制器    
}

/**
  * @brief    直立环PD
使用车身倾斜角度和机械角度零点以及倾斜速度这三个数据，
车身倾斜角度与机械角度零点之差作为比例项，车身倾斜角度与机械角度零点之差的累加作为积分项，车身倾斜速度作为微分项。
  */

int Vertical(float Mid, float angle, float gyro)
{
  int pwmout1;
	pwmout1 = Vertical_Kp*(angle-Mid)+Vertical_Kd*gyro;
  return pwmout1; 
}

/**
  * @brief   速度环PI

  */

int Velocity(int Encoder_motor)
{
  static int Encoder_s,Encoder_Err_Lowout_Last,pwmout2,Encoder_Err,Encoder_Err_Lowout;
	float a=0.7;
	//计算速度偏差
	Encoder_Err = Encoder_motor;
	//对速度偏差进行低通滤波
	Encoder_Err_Lowout=(1-a)*Encoder_Err+a*Encoder_Err_Lowout_Last;	
	Encoder_Err_Lowout_Last=Encoder_Err_Lowout;
	//对速度偏差积分，积分出位移
	Encoder_s+=Encoder_Err_Lowout;
	//积分限幅
	Encoder_s=Encoder_s>10000?10000:(Encoder_s<(-10000)?(-10000):Encoder_s);
	//速度环控制输出计算
	pwmout2=Velocity_Kp*Encoder_Err_Lowout+Velocity_Ki*Encoder_s;
	return pwmout2;
	
}


/**
  * @brief   检测小车是否倒下
  * @param   inAngleData 输入的参考角度值
  * @retval  detectionMark 返回是否倒下标志 1：倒下 0：正常
  */
static uint8_t detectionFallDown(float inAngleData)
{
  uint8_t detectionMark = 0;

  if( f_abs(inAngleData) > 20.0f )                                             
  {
    detectionMark = 1;                                              //关闭输出标志置位                 
  }                                                            
  return detectionMark;     
}
/**
  * @brief   PWM输出
  * @param   pwm1 输入第一路pwm
  * @param   pwm2 输入第二路pwm     20v->1650
  * @retval  x
  */
uint16_t pwm3=1144;
static void pwmMotorOut(uint16_t pwm1 , uint16_t pwm2 )
{
  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, u16RangeLimit(pwm1 , 0 , 2000));	          //控制占空比
  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, u16RangeLimit(pwm2 , 0 , 2000));
	//__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, u16RangeLimit(pwm3 , 0 , 2000));					//50hz 18000 启动17225 
																																												//500hz 1800 启动1355 占空比24
																																												//100hz 9000 启动6797 占空比24
}

/**
  * @brief   控制器输出
  */
 uint16_t motor_speed; 

void _controller_output(void)                                    
{
  static uint8_t FalldownAndRestart = 0;
	
  
  /* 检测是否倒下 */
  if( detectionFallDown(att.rol)==1 )                 
  {
    motor_speed = 2000;                                 //后轮电机速度设为最小
    FalldownAndRestart = 1;                                 //倒下之后需复位重启将其清除才能正常进入到平衡状态
    DisableAuxMotor();                                      //失能关闭前后动力电机
    Motor1.out = 0;                                         //清除电机PWM输出值           
    Motor2.out = 0;                                         //清除电机PWM输出值    
    pwm_out1 = 0;
		pwm_out2 = 0;
    clear_integral(&all.vel_encoder);                       //清除积分        
    clear_integral(&all.rol_angle);                         //清除积分                
    clear_integral(&all.rol_gyro);                          //清除积分
		
  }                                                                   
  else if( (acc_raw.z >= 2500 && acc_raw.z <= 5000) && f_abs(att.rol) <= 20.0f && FalldownAndRestart == 0)                                                               
  {
    //N20_motor_speed = 1500;
		
    EnableAuxMotor();                                       //使能前后动力电机
    
		Motor1.out =  all.rol_gyro.out;                         //正常输出    
		
  }
  /* 根据输出正负来判断方向 */
//  if(Motor1.out>0)  dirAnticlockwise();   
//  else  dirClockwise();
	//pwm_out1=Vertical(Mid,att.rol,Mpu.deg_s.y+1.1);
	//pwm_out2=Velocity(encoderINFO.mainNumberValue);
	
//	if(pwm_out1+pwm_out2>0)  dirAnticlockwise();              //预留方向控制输出
//  else  dirClockwise();
// 
  TurnLeftOrRight(BluetoothParseMsg.Xrocker);               //左右转向控制
  goForwardOrBackward(BluetoothParseMsg.Yrocker);           //前后方向控制

	

  //pwmMotorOut( pwm_out1+pwm_out2+1150 , motor_speed);                 //串级pid电机输出
	//pwmMotorOut( pwm_out1+1150 , motor_speed);                          //并级pid电机输出
	
	
	pwmMotorOut( Motor1.out+1150 , motor_speed);                 //电机输出
	//pwmMotorOut( int_abs(Motor1.out) , motor_speed);                 //电机输出

	
} 

/**
  * @brief   舵机转向控制 舵机的控制一般需要一个20ms左右的时基脉冲，该脉冲的高电平部分一般为0.5ms-2.5ms范围内的角度控制脉冲部分，总间隔为2ms。
             以180度角度伺服为例，那么对应的控制关系是这样的：
    0.5ms------------0度； 
    1.0ms------------45度；
　　1.5ms------------90度；
　　2.0ms-----------135度；
　　2.5ms-----------180度；
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 5);  //0.5ms
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 10); //1.0ms 右转45度
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 15); //1.5ms 前方向
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 20); //2.0ms 左转45度
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 25); //2.5ms
  * @param   接收到的蓝牙遥控数据（左右摇杆数据）
  * @retval  x
  */
void TurnLeftOrRight(float inXrocker)
{
  static float LastXrocker = 0;
  if(inXrocker != LastXrocker)//判断X摇杆值
  {
    if( inXrocker == 0  )
    {
      __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, (dirBASE + DIR_SIZE ) + dirADJUST );// 左转一定角度    
    }
    else if( inXrocker == 5 )
    {
      __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, (dirBASE - DIR_SIZE ) + dirADJUST );// 右转一定角度    
    }              
    else if( inXrocker == 2 )
    {
      __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, dirBASE + dirADJUST );//1.5ms 前方向
    }    
    LastXrocker = inXrocker;//X摇杆值回0
  }
}
/**
  * @brief   进行前后控制
  * @param   接收到的蓝牙遥控数据（前后摇杆数据）
  * @retval  x
  */
void goForwardOrBackward(float inYrcoker)
{
	static uint16_t speed1=1400,speed2=1000;
	
//  if( inYrcoker >= 0 && inYrcoker <= 1 )                    
//  {
//    EnableAuxMotor();                                       //使能前后动力电机
//    CarBackward();                                          //后退   
//    dirClockwise();
//  }
//	
//	
//  else if( inYrcoker >= 4 && inYrcoker <= 5 )
//  {
//    EnableAuxMotor();                                       //使能前后动力电机
//    CargoForward();                                         //前进
//		dirAnticlockwise(); 
//  }
	  if( inYrcoker == 0  )                    
  {
    EnableAuxMotor();                                       //使能前后动力电机
    CarBackward();                                           
    dirClockwise();                                         //前进2挡
		motor_speed = speed2;
  }
	 else if( inYrcoker == 1  )                    
  {
    EnableAuxMotor();                                       //使能前后动力电机
    CarBackward();                                             
    dirClockwise();                                         //前进1挡
		motor_speed = speed1;
  }
	
  else if( inYrcoker == 4  )
  {
    EnableAuxMotor();                                       //使能前后动力电机
    CargoForward();                                         
		dirAnticlockwise();                                     //后退1挡
		motor_speed = speed1;
  }
	else if( inYrcoker == 5  )
  {
    EnableAuxMotor();                                       //使能前后动力电机
    CargoForward();                                         
		dirAnticlockwise();                                     //后退2挡
		motor_speed = speed2;
  }
  else 
  {
    motor_speed = 2000;                                 //电机速度设置为最低
    DisableAuxMotor();                                      //失能关闭前后动力电机
  }
}

/**
 * @brief  小车前进
 */
void CargoForward(void)
{ 
  HAL_GPIO_WritePin(AUX_AIN1_GPIO_Port, AUX_AIN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(AUX_AIN2_GPIO_Port, AUX_AIN2_Pin, GPIO_PIN_SET);
}
/**
 * @brief  小车后退
 */
void CarBackward(void)
{
  HAL_GPIO_WritePin(AUX_AIN1_GPIO_Port, AUX_AIN1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(AUX_AIN2_GPIO_Port, AUX_AIN2_Pin, GPIO_PIN_RESET);
}
/**
 * @brief  使能辅助电机
 */
void EnableAuxMotor(void)
{
  HAL_GPIO_WritePin(AUX_STBY_GPIO_Port, AUX_STBY_Pin, GPIO_PIN_SET);
}
/**
 * @brief  失能辅助电机
 */
void DisableAuxMotor(void)
{
  HAL_GPIO_WritePin(AUX_STBY_GPIO_Port, AUX_STBY_Pin, GPIO_PIN_RESET);
}


