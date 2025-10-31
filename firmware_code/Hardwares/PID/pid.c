#include "pid.h"



_ALL_PID all;

/**
  * @brief   存储pid控制器参数 三行五列，每行代表一个pid控制器参数，每列对应参数 0.kp 1.ki 2.kd 3.积分限幅  4.pid输出限幅值
  * @param   x
  * @retval  x
  */
/*//机械中值-5.8
角速度环

角度环
在调试过程中，角度环主要是用来补偿速度环带来的溢出或者延迟，因此角度环我只采用了简单的位置式P进行处理，并且调试过程中是粗调的。

速度环
速度环虽然作为最外环，我却认为是很重要的一环，因为他决定了你单车受到干扰后保持稳定的能力，采用的是位置式PID进行精调。
很简单的例子，假设单车往左倒需要动轮逆时针转动（车屁股视角），通过编码器的极性你就可以判断出此时车身的姿态到底是往哪边倒的，
此时为了让车往右的方向归，便需要不断加速来平衡，这中间就涉及到了溢出的问题，
也就是车子会向右倒，这时候就需要粗调角度环来抑制这个影响。（当车身右倒，角度环大就抵消了）

调试方法
先精调角速度环，到往一个方向倒开始觉得困难了就可以调速度环，加上了速度环以后，车子能立2s左右倒下后开始调速度环，
一开始建议大家三个环都只进行普通的P控制，实测效果也非常好，能立且抗干扰能力比Lqr好太多。
调参过程中尽可能大的增加速度环，外界干扰姿态，观察归正情况，若有过冲则增大角度环，而角速度环更像是一个比例系数（如果是纯P），
所有的修正均可通过修正速度环处理，调好以后加上D值精调。*/

const float  controller_parameter[3][5] =
{
  /* 0.kp 1.ki 2.kd 3.积分限幅  4.pid输出限幅值 */
//    {7.3 , 0.0,  0,  550 , 2000 },                           //rol_angle     内环角度环    小车面包板   
//    {0.070 , 0.00005,  0.055,  500 , 2000 },                 //vel_encoder   外环速度环       
//    {26.5 ,  0.0,  0,  500 , 2000 },                         //gyro          内环角速度环 
		
//    {7.5 , 0.0,  0,  550 , 2000 },                           //rol_angle     内环角度环       小车PCB
//    {0.068 , 0.00005,  0.055,  500 , 2000 },                 //vel_encoder   外环速度环       
//    {26.5 ,  0.0,  0,  500 , 2000 },                         //gyro          内环角速度环 
		
		{4.72 , 0.0,  0,  550 , 2000 },                            //rol_angle     内环角度环       大车  （1）4.56 0.0075 11.65 （2）0.00735
    {0.007 , 0.0,  0,  550 , 2000 },                 						//vel_encoder   外环速度环     加后轮驱动 （1）4.72 0.007 12               
    {11.9 , 0.0,  0,  550 , 2000 },                            //gyro          内环角速度环 
		
};

/**
  * @brief   PID参数初始化配置
  * @param   *controller PID控制器指针，指向不同的控制器
  * @param   label PID参数标号，选择对应控制器的参数数组标号
  * @retval  x
  */
void pid_init(_PID *controller,uint8_t label)
{
    controller->kp              = controller_parameter[label][0];         
    controller->ki              = controller_parameter[label][1];         
    controller->kd              = controller_parameter[label][2];         
    controller->integral_max    = controller_parameter[label][3];         
    controller->out_max         = controller_parameter[label][4];               
}

//PID参数初始化
void all_pid_init(void)
{
    pid_init(&all.rol_angle,0);
    pid_init(&all.vel_encoder,1);
    pid_init(&all.rol_gyro,2);
} 

/**
  * @brief   PID控制器
  * @param   *controller PID控制器指针，指向不同的控制器
  * @retval  controller->out 经过控制后的输出值
  */
float pid_controller(_PID *controller)
{
    controller->err_last = controller->err;                                                  //保存上次偏差
    controller->err = controller->expect - controller->feedback;                            //偏差计算
    controller->integral += controller->ki * controller->err;                               //积分  
    //积分限幅
    if(controller->integral >  controller->integral_max)     controller->integral =  controller->integral_max;
    if(controller->integral < -controller->integral_max)     controller->integral = -controller->integral_max;
    //pid运算
    controller->out =  controller->kp*controller->err + controller->integral + controller->kd*(controller->err-controller->err_last);
   
    //输出限幅
    if(controller->out >  controller->out_max)   controller->out =  controller->out_max;
    if(controller->out < -controller->out_max)   controller->out = -controller->out_max;
    return controller->out;
}
/**
  * @brief   PID控制器积分项清除
  * @param   *controller PID控制器指针，指向不同的控制器
  * @retval  x
  */
void clear_integral(_PID *controller)
{
    controller->integral = 0.0f;
}











