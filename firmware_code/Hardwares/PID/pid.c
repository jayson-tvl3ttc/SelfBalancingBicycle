#include "pid.h"



_ALL_PID all;

/**
  * @brief   �洢pid���������� �������У�ÿ�д���һ��pid������������ÿ�ж�Ӧ���� 0.kp 1.ki 2.kd 3.�����޷�  4.pid����޷�ֵ
  * @param   x
  * @retval  x
  */
/*//��е��ֵ-5.8
���ٶȻ�

�ǶȻ�
�ڵ��Թ����У��ǶȻ���Ҫ�����������ٶȻ���������������ӳ٣���˽ǶȻ���ֻ�����˼򵥵�λ��ʽP���д������ҵ��Թ������Ǵֵ��ġ�

�ٶȻ�
�ٶȻ���Ȼ��Ϊ���⻷����ȴ��Ϊ�Ǻ���Ҫ��һ������Ϊ���������㵥���ܵ����ź󱣳��ȶ������������õ���λ��ʽPID���о�����
�ܼ򵥵����ӣ����赥��������Ҫ������ʱ��ת������ƨ���ӽǣ���ͨ���������ļ�����Ϳ����жϳ���ʱ�������̬���������ıߵ��ģ�
��ʱΪ���ó����ҵķ���飬����Ҫ���ϼ�����ƽ�⣬���м���漰������������⣬
Ҳ���ǳ��ӻ����ҵ�����ʱ�����Ҫ�ֵ��ǶȻ����������Ӱ�졣���������ҵ����ǶȻ���͵����ˣ�

���Է���
�Ⱦ������ٶȻ�������һ�����򵹿�ʼ���������˾Ϳ��Ե��ٶȻ����������ٶȻ��Ժ󣬳�������2s���ҵ��º�ʼ���ٶȻ���
һ��ʼ��������������ֻ������ͨ��P���ƣ�ʵ��Ч��Ҳ�ǳ��ã������ҿ�����������Lqr��̫�ࡣ
���ι����о����ܴ�������ٶȻ�����������̬���۲������������й���������ǶȻ��������ٶȻ�������һ������ϵ��������Ǵ�P����
���е���������ͨ�������ٶȻ����������Ժ����Dֵ������*/

const float  controller_parameter[3][5] =
{
  /* 0.kp 1.ki 2.kd 3.�����޷�  4.pid����޷�ֵ */
//    {7.3 , 0.0,  0,  550 , 2000 },                           //rol_angle     �ڻ��ǶȻ�    С�������   
//    {0.070 , 0.00005,  0.055,  500 , 2000 },                 //vel_encoder   �⻷�ٶȻ�       
//    {26.5 ,  0.0,  0,  500 , 2000 },                         //gyro          �ڻ����ٶȻ� 
		
//    {7.5 , 0.0,  0,  550 , 2000 },                           //rol_angle     �ڻ��ǶȻ�       С��PCB
//    {0.068 , 0.00005,  0.055,  500 , 2000 },                 //vel_encoder   �⻷�ٶȻ�       
//    {26.5 ,  0.0,  0,  500 , 2000 },                         //gyro          �ڻ����ٶȻ� 
		
		{4.72 , 0.0,  0,  550 , 2000 },                            //rol_angle     �ڻ��ǶȻ�       ��  ��1��4.56 0.0075 11.65 ��2��0.00735
    {0.007 , 0.0,  0,  550 , 2000 },                 						//vel_encoder   �⻷�ٶȻ�     �Ӻ������� ��1��4.72 0.007 12               
    {11.9 , 0.0,  0,  550 , 2000 },                            //gyro          �ڻ����ٶȻ� 
		
};

/**
  * @brief   PID������ʼ������
  * @param   *controller PID������ָ�룬ָ��ͬ�Ŀ�����
  * @param   label PID������ţ�ѡ���Ӧ�������Ĳ���������
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

//PID������ʼ��
void all_pid_init(void)
{
    pid_init(&all.rol_angle,0);
    pid_init(&all.vel_encoder,1);
    pid_init(&all.rol_gyro,2);
} 

/**
  * @brief   PID������
  * @param   *controller PID������ָ�룬ָ��ͬ�Ŀ�����
  * @retval  controller->out �������ƺ�����ֵ
  */
float pid_controller(_PID *controller)
{
    controller->err_last = controller->err;                                                  //�����ϴ�ƫ��
    controller->err = controller->expect - controller->feedback;                            //ƫ�����
    controller->integral += controller->ki * controller->err;                               //����  
    //�����޷�
    if(controller->integral >  controller->integral_max)     controller->integral =  controller->integral_max;
    if(controller->integral < -controller->integral_max)     controller->integral = -controller->integral_max;
    //pid����
    controller->out =  controller->kp*controller->err + controller->integral + controller->kd*(controller->err-controller->err_last);
   
    //����޷�
    if(controller->out >  controller->out_max)   controller->out =  controller->out_max;
    if(controller->out < -controller->out_max)   controller->out = -controller->out_max;
    return controller->out;
}
/**
  * @brief   PID���������������
  * @param   *controller PID������ָ�룬ָ��ͬ�Ŀ�����
  * @retval  x
  */
void clear_integral(_PID *controller)
{
    controller->integral = 0.0f;
}











