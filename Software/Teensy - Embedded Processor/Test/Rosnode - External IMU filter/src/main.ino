/*
* rosserial Publisher Example
* Prints "hello world!"
*/
#include <ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>

//Timer variables for the example
elapsedMillis lipo_serial_timer;
elapsedMillis state_vector_serial_timer;
elapsedMillis ultrasonic_range_serial_timer;

// Initialize the ROS Handle
ros::NodeHandle nh;
// Global variables
extern ros::Publisher ultrasonic_range_pub;
extern ros::Publisher lipo_state_pub;
extern ros::Publisher quadcopter_pose_pub;
extern ros::Publisher quadcopter_twist_pub;
extern ros::Publisher quadcopter_magn_pub;
extern ros::Publisher quadcopter_imu_pub;
// Bring the State vector messages to usethem if neccesary
extern geometry_msgs::Pose  quadcopter_pose_msg;
extern geometry_msgs::Twist quadcopter_twist_msg;



/*
############################### DYNAMIC RECONFIGURE PROXY CODE #####################################
*/
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

ros::Subscriber<std_msgs::Float32MultiArray> dyn_rec_sub("teensy_dynamic_reconfigure", &messageCb );

/*
#########################################################################################################
*/

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
    nh.advertise(quadcopter_pose_pub);
    nh.advertise(quadcopter_twist_pub);
    nh.advertise(quadcopter_magn_pub);
    nh.advertise(quadcopter_imu_pub);
    // Subscribe to all the topics
    nh.subscribe(dyn_rec_sub);

    // Setup the peripherals.
    ultrasonic_range_setup();
    lipo_state_setup();
    motor_setup();
    state_vector_setup();
    digitalWriteFast(LED_BUILTIN, HIGH);


}

void loop()
{
    // Run the update functions
    state_vector_update();

    // Publish the state vector 100 times per second
    if (state_vector_serial_timer >= 50) {

        state_vector_serial_timer = 0;
        state_vector_publish();
    }


    // Publish Battery state every 5 seconds
    if (lipo_serial_timer >= 5000) {
        lipo_serial_timer = 0;

        lipo_state_update();
        lipo_state_publish();
    }

    // Publish Range finder state 20 timer per second
    if (ultrasonic_range_serial_timer >= 500) {
        ultrasonic_range_serial_timer = 0;

        ultrasonic_range_publish();

        digitalWriteFast(LED_BUILTIN, HIGH);
        delay(10);
        digitalWriteFast(LED_BUILTIN, LOW);
    }


    /* Implement controller */





    nh.spinOnce();
}
