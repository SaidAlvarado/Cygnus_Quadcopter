
#include <geometry_msgs/Vector3.h>


//ROs message
geometry_msgs::Vector3 PID_moments_msg;
ros::Publisher PID_moments_pub("PID_moments", &PID_moments_msg);

// Import global variables. SETPOINTS and GAINS
extern float dynrec_motor_1, dynrec_motor_2, dynrec_motor_3, dynrec_motor_4, dynrec_Roll_SetPoint, dynrec_Pitch_SetPoint;
extern float dynrec_Yaw_SetPoint, dynrec_Pitch_Kp, dynrec_Pitch_Kd, dynrec_Pitch_Ki, dynrec_Roll_Kp, dynrec_Roll_Kd, dynrec_Roll_Ki;
extern float dynrec_Yaw_Kp, dynrec_Yaw_Kd, dynrec_Yaw_Ki, dynrec_General_Thrust;
// Import data for error calculation
extern float motor_1_percent, motor_2_percent, motor_3_percent, motor_4_percent;
// Thrust vector variable to export for later plotting
float pitch_moment, roll_moment, yaw_moment, general_thrust;

// errors
float pitch_error_old;
float roll_error_old;
float yaw_error_old;
// error integral
float pitch_error_accumulated;
float roll_error_accumulated;
float yaw_error_accumulated;

// delta of time
elapsedMicros PID_dt;


// Initialize the error variables
void PID_setup(){

    // errors
    pitch_error_old = 0;
    roll_error_old = 0;
    yaw_error_old = 0;
    // error integral
    pitch_error_accumulated = 0;
    roll_error_accumulated = 0;
    yaw_error_accumulated = 0;

    // start timer
    PID_dt = 0;
}


void PID_step(float pitch, float roll, float yaw, float pitch_sp, float roll_sp, float yaw_sp, float* motor_output){

// Error Variables
float pitch_error, roll_error, yaw_error;
float pitch_deriv_error, roll_deriv_error, yaw_deriv_error;

// Update the error calculation
pitch_error = pitch_sp - pitch;
roll_error = roll_sp - roll;
yaw_error = yaw_sp - yaw;
// Derivative error calculation
pitch_deriv_error = (pitch_error - pitch_error_old) / PID_dt / 1000000.0;
roll_deriv_error = (roll_error - roll_error_old) / PID_dt / 1000000.0;
yaw_deriv_error = (yaw_error - yaw_error_old) / PID_dt / 1000000.0;
//Integral error calculation
if (pitch_moment < 100) pitch_error_accumulated += pitch_error*PID_dt / 1000000.0;
if (roll_moment < 100) roll_error_accumulated += roll_error*PID_dt / 1000000.0;
if (yaw_moment < 100) yaw_error_accumulated += yaw_error*PID_dt / 1000000.0;

//Constraint the integral portion
if (pitch_error_accumulated > 100) pitch_error_accumulated = 100;
if (roll_error_accumulated > 100) roll_error_accumulated = 100;
if (yaw_error_accumulated > 100) yaw_error_accumulated = 100;

// Calculate the moments
pitch_moment = dynrec_Pitch_Kp * pitch_error + dynrec_Pitch_Kd * pitch_deriv_error +  dynrec_Pitch_Ki * pitch_error_accumulated;
roll_moment = dynrec_Roll_Kp * roll_error + dynrec_Roll_Kd * roll_deriv_error +  dynrec_Roll_Ki * roll_error_accumulated;
yaw_moment = dynrec_Yaw_Kp * yaw_error + dynrec_Yaw_Kd * yaw_deriv_error +  dynrec_Yaw_Ki * yaw_error_accumulated;

// Motor mixing!
motor_output[0]  = dynrec_General_Thrust + pitch_moment - roll_moment + yaw_moment;
motor_output[1]  = dynrec_General_Thrust - pitch_moment - roll_moment - yaw_moment;
motor_output[2]  = dynrec_General_Thrust - pitch_moment + roll_moment + yaw_moment;
motor_output[3]  = dynrec_General_Thrust + pitch_moment + roll_moment - yaw_moment;

// Update message
PID_moments_msg.x = pitch_moment;
PID_moments_msg.y = roll_moment;
PID_moments_msg.z = yaw_moment;

}



// Publishes the message
void PID_publish(){
    //Publish message
    PID_moments_pub.publish( &PID_moments_msg );
}
