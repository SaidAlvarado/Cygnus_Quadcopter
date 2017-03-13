/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

ros::NodeHandle nh;

float motor_1, motor_2, motor_3, motor_4, Roll_SetPoint, Pitch_SetPoint, Yaw_SetPoint, Pitch_Kp, Pitch_Kd, Pitch_Ki, Roll_Kp, Roll_Kd, Roll_Ki, Yaw_Kp, Yaw_Kd, Yaw_Ki;
uint8_t Motor_Master_Switch, Controller_ON_OFF;

void messageCb( const std_msgs::Float32MultiArray& dyn_reconf_msg){
  Motor_Master_Switch = (uint8_t)dyn_reconf_msg.data[0];
  motor_1 = dyn_reconf_msg.data[1];
  motor_2 = dyn_reconf_msg.data[2];
  motor_3 = dyn_reconf_msg.data[3];
  motor_4 = dyn_reconf_msg.data[4];
  Controller_ON_OFF = (uint8_t)dyn_reconf_msg.data[5];
  Roll_SetPoint = dyn_reconf_msg.data[6];
  Pitch_SetPoint = dyn_reconf_msg.data[7];
  Yaw_SetPoint = dyn_reconf_msg.data[8];
  Pitch_Kp = dyn_reconf_msg.data[9];
  Pitch_Kd = dyn_reconf_msg.data[10];
  Pitch_Ki = dyn_reconf_msg.data[11];
  Roll_Kp = dyn_reconf_msg.data[12];
  Roll_Kd = dyn_reconf_msg.data[13];
  Roll_Ki = dyn_reconf_msg.data[14];
  Yaw_Kp = dyn_reconf_msg.data[15];
  Yaw_Kd = dyn_reconf_msg.data[16];
  Yaw_Ki = dyn_reconf_msg.data[17];
}

ros::Subscriber<std_msgs::Float32MultiArray> sub("teensy_dynamic_reconfigure", &messageCb );

void setup()
{
  nh.initNode();
  nh.subscribe(sub);
  motor_setup();
}

void loop()
{
  nh.spinOnce();
  delay(1);

  if (Motor_Master_Switch > 0){
      motor_pwm_write(1,motor_1);
      motor_pwm_write(2,motor_2);
      motor_pwm_write(3,motor_3);
      motor_pwm_write(4,motor_4);
  }
  else{
      motor_pwm_write(1,0);
      motor_pwm_write(2,0);
      motor_pwm_write(3,0);
      motor_pwm_write(4,0);
  }
}
