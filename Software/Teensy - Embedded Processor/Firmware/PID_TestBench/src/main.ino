/*
* rosserial Publisher Example
* Prints "hello world!"
*/
#include <ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32MultiArray.h>

//Timer variables for the example
elapsedMillis lipo_serial_timer;
elapsedMillis state_vector_serial_timer;
elapsedMillis ultrasonic_range_serial_timer;
elapsedMillis PID_moments_serial_timer;

// Initialize the ROS Handle
ros::NodeHandle nh;
// Global variables
extern ros::Publisher ultrasonic_range_pub;
extern ros::Publisher lipo_state_pub;
// extern ros::Publisher quadcopter_pose_pub;
extern ros::Publisher quadcopter_twist_pub;
extern ros::Publisher quadcopter_magn_pub;
extern ros::Publisher quadcopter_imu_pub;
extern ros::Publisher PID_moments_pub;
// Bring the State vector messages to usethem if neccesary
extern geometry_msgs::Pose  quadcopter_pose_msg;
extern geometry_msgs::Twist quadcopter_twist_msg;



/*
############################### DYNAMIC RECONFIGURE PROXY CODE #####################################
*/
float dynrec_motor_1, dynrec_motor_2, dynrec_motor_3, dynrec_motor_4, dynrec_Roll_SetPoint, dynrec_Pitch_SetPoint;
float dynrec_Yaw_SetPoint, dynrec_Pitch_Kp, dynrec_Pitch_Kd, dynrec_Pitch_Ki, dynrec_Roll_Kp, dynrec_Roll_Kd, dynrec_Roll_Ki;
float dynrec_Yaw_Kp, dynrec_Yaw_Kd, dynrec_Yaw_Ki, dynrec_General_Thrust;
uint8_t dynrec_Motor_Master_Switch, dynrec_Controller_ON_OFF;

void messageCb( const std_msgs::Float32MultiArray& dyn_reconf_msg){
  dynrec_Motor_Master_Switch = (uint8_t)dyn_reconf_msg.data[0];
  dynrec_motor_1 = dyn_reconf_msg.data[1];
  dynrec_motor_2 = dyn_reconf_msg.data[2];
  dynrec_motor_3 = dyn_reconf_msg.data[3];
  dynrec_motor_4 = dyn_reconf_msg.data[4];
  dynrec_Controller_ON_OFF = (uint8_t)dyn_reconf_msg.data[5];
  dynrec_Roll_SetPoint = dyn_reconf_msg.data[6];
  dynrec_Pitch_SetPoint = dyn_reconf_msg.data[7];
  dynrec_Yaw_SetPoint = dyn_reconf_msg.data[8];
  dynrec_Pitch_Kp = dyn_reconf_msg.data[9];
  dynrec_Pitch_Kd = dyn_reconf_msg.data[10];
  dynrec_Pitch_Ki = dyn_reconf_msg.data[11];
  dynrec_Roll_Kp = dyn_reconf_msg.data[12];
  dynrec_Roll_Kd = dyn_reconf_msg.data[13];
  dynrec_Roll_Ki = dyn_reconf_msg.data[14];
  dynrec_Yaw_Kp = dyn_reconf_msg.data[15];
  dynrec_Yaw_Kd = dyn_reconf_msg.data[16];
  dynrec_Yaw_Ki = dyn_reconf_msg.data[17];
  dynrec_General_Thrust = dyn_reconf_msg.data[18];
}

ros::Subscriber<std_msgs::Float32MultiArray> dyn_rec_sub("teensy_dynamic_reconfigure", &messageCb );

/*
#########################################################################################################
*/



/*
############################### External Madgwick Filter #####################################
*/

float current_pitch, current_roll, current_yaw;

void messageCb( const geometry_msgs::Vector3& rpi_euler_msg){
  current_pitch = rpi_euler_msg.x;
  current_roll = rpi_euler_msg.y;
  current_yaw = rpi_euler_msg.z;
}

ros::Subscriber<geometry_msgs::Vector3> rpi_euler_sub("rpi_euler", &messageCb );
/*
#########################################################################################################
*/

// Controller and motor variables
//Variables that actually gets actualized
float motor_1_percent, motor_2_percent, motor_3_percent, motor_4_percent;
// Controller output
float controller_motor_ouput[4];



void setup()
{
    pinMode(LED_BUILTIN,OUTPUT);
    delay(500);
    // Initialize ROS node, and advertise all the topics
    nh.getHardware()->setBaud(1000000); //Run serial port at 1M baud
    nh.initNode();
    // Publish all the topics
    nh.advertise(ultrasonic_range_pub);
    nh.advertise(lipo_state_pub);
    // nh.advertise(quadcopter_pose_pub);
    nh.advertise(quadcopter_twist_pub);
    nh.advertise(quadcopter_magn_pub);
    nh.advertise(quadcopter_imu_pub);
    nh.advertise(PID_moments_pub);
    // Subscribe to all the topics
    nh.subscribe(dyn_rec_sub);
    nh.subscribe(rpi_euler_sub);

    // Setup the peripherals.
    ultrasonic_range_setup();
    lipo_state_setup();
    motor_setup();
    state_vector_setup();
    PID_setup();
    digitalWriteFast(LED_BUILTIN, HIGH);


}

void loop()
{
    // Run the update functions

    // Publish the state vector 100 times per second
    if (state_vector_serial_timer >= 20) {

        state_vector_serial_timer = 0;
        state_vector_update();
        state_vector_publish();
    }


    // Publish Battery state every 5 seconds
    if (lipo_serial_timer >= 5000) {
        lipo_serial_timer = 0;

        lipo_state_update();
        lipo_state_publish();
    }

    // // Publish Range finder state 20 timer per second
    // if (ultrasonic_range_serial_timer >= 500) {
    //     ultrasonic_range_serial_timer = 0;
    //
    //     ultrasonic_range_publish();
    //
    //     digitalWriteFast(LED_BUILTIN, HIGH);
    //     delay(10);
    //     digitalWriteFast(LED_BUILTIN, LOW);
    // }

    // Publish PID moments
    if (PID_moments_serial_timer >= 100) {
        PID_moments_serial_timer = 0;

        PID_publish();

        digitalWriteFast(LED_BUILTIN, HIGH);
        delay(10);
        digitalWriteFast(LED_BUILTIN, LOW);
    }


    /* Implement controller */

    // Check the Master flag to shutdown the motors
    if (dynrec_Motor_Master_Switch == 0) {
        // Shutdown all motor
        motor_1_percent = 0;
        motor_2_percent = 0;
        motor_3_percent = 0;
        motor_4_percent = 0;
    }
    else{
        // Motors are ON, check between manual control or controller
        if (dynrec_Controller_ON_OFF == 0){
            // Manual control
            motor_1_percent = dynrec_motor_1;
            motor_2_percent = dynrec_motor_2;
            motor_3_percent = dynrec_motor_3;
            motor_4_percent = dynrec_motor_4;
        }
        else{
            //controller output
            PID_step(current_pitch, current_roll, current_yaw, dynrec_Pitch_SetPoint, dynrec_Roll_SetPoint, dynrec_Yaw_SetPoint, controller_motor_ouput);

            // motor_1_percent = 0;
            // motor_2_percent = 0;
            // motor_3_percent = 0;
            // motor_4_percent = 0;
            motor_1_percent = controller_motor_ouput[0];
            motor_2_percent = controller_motor_ouput[1];
            motor_3_percent = controller_motor_ouput[2];
            motor_4_percent = controller_motor_ouput[3];
        }

    }

    // Actualize the motor speed
    motor_pwm_write(1, motor_1_percent);
    motor_pwm_write(2, motor_2_percent);
    motor_pwm_write(3, motor_3_percent);
    motor_pwm_write(4, motor_4_percent);

    nh.spinOnce();
}
