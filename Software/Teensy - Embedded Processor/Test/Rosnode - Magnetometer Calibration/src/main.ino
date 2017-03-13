/*
* rosserial Publisher Example
* Prints "hello world!"
*/
#include <ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

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
// Bring the State vector messages to usethem if neccesary
extern geometry_msgs::Pose  quadcopter_pose_msg;
extern geometry_msgs::Twist quadcopter_twist_msg;



void setup()
{
    pinMode(LED_BUILTIN,OUTPUT);
    // Initialize ROS node, and advertise all the topics
    nh.getHardware()->setBaud(1000000); //Run serial port at 1M baud
    nh.initNode();
    nh.advertise(ultrasonic_range_pub);
    nh.advertise(lipo_state_pub);
    nh.advertise(quadcopter_pose_pub);
    nh.advertise(quadcopter_twist_pub);
    nh.advertise(quadcopter_magn_pub);

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


    nh.spinOnce();
}
