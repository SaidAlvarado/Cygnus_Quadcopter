/*
* rosserial Publisher Example
* Prints "hello world!"
*/

#include <ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Range.h>
#include "AHRS_L3GD20H_LSM303.h"

// Initialize ROS Publiser and message
geometry_msgs::Pose  quadcopter_pose_msg;
geometry_msgs::Twist quadcopter_twist_msg;
sensor_msgs::MagneticField quadcopter_magn_msg;
sensor_msgs::Imu quadcopter_imu_msg;
ros::Publisher quadcopter_pose_pub("quadcopter_pose", &quadcopter_pose_msg);
ros::Publisher quadcopter_twist_pub("quadcopter_twist", &quadcopter_twist_msg);
ros::Publisher quadcopter_magn_pub("imu/mag", &quadcopter_magn_msg);
ros::Publisher quadcopter_imu_pub("imu/data_raw", &quadcopter_imu_msg);

// Bring in the RangeFinder altitude message
extern sensor_msgs::Range ultrasonic_range_msg;


// AHRS variables
MadgwicksFilter ahrs(2,0.1);
float euler[3];
float quaternion_vector[4];
float ang_vel_vector[3];
int16_t magn_vector[3];
int16_t acc_vector[3];
int16_t gyro_vector[3];

elapsedMicros z_speed_timer;


// Fills in basic information about the Range msg, and initializes the interrupts
void state_vector_setup(){
    // Initialize Pose message
    // Position component
    quadcopter_pose_msg.position.x = 0;
    quadcopter_pose_msg.position.y = 0;
    quadcopter_pose_msg.position.z = 0;
    // Orientation component
    quadcopter_pose_msg.orientation.x = 0;
    quadcopter_pose_msg.orientation.y = 0;
    quadcopter_pose_msg.orientation.z = 0;
    quadcopter_pose_msg.orientation.w = 0;
    // Initialize Twist message
    quadcopter_twist_msg.linear.x = 0;
    quadcopter_twist_msg.linear.y = 0;
    quadcopter_twist_msg.linear.z = 0;
    quadcopter_twist_msg.angular.x = 0;
    quadcopter_twist_msg.angular.y = 0;
    quadcopter_twist_msg.angular.z = 0;

    //start magnetometer
    // quadcopter_magn_msg.vector.x = 0;
    // quadcopter_magn_msg.vector.y = 0;
    // quadcopter_magn_msg.vector.z = 0;

    // Initialize AHRS library
    ahrs.begin();

    // Clear the timer used to differentiate z.
    z_speed_timer = 0;
}

// Functions that actualize the messages with the state vector
void state_vector_update(){

    // Run filter
    ahrs.step();
    // Get orientation and angular velocity once per iteration
    if (ahrs.getStatus() == 0) {
        ahrs.getAttitude(quaternion_vector);
        ahrs.getAngularVelocity(ang_vel_vector);

        // Fill in the message
        // Orientation component
        // quadcopter_pose_msg.orientation.x = quaternion_vector[0];
        // quadcopter_pose_msg.orientation.y = quaternion_vector[1];
        // quadcopter_pose_msg.orientation.z = quaternion_vector[2];
        // quadcopter_pose_msg.orientation.w = quaternion_vector[3];
        // Angular velocity
        // quadcopter_twist_msg.angular.x = ang_vel_vector[0];
        // quadcopter_twist_msg.angular.y = ang_vel_vector[1];
        // quadcopter_twist_msg.angular.z = ang_vel_vector[2];

        // Calculate the differential of the z speed.
        quadcopter_twist_msg.linear.z = (ultrasonic_range_msg.range - quadcopter_pose_msg.position.z) / (z_speed_timer/1000000);
        z_speed_timer = 0;

        // Altitude according to the Ultrasonic range sensor
        quadcopter_pose_msg.position.z = ultrasonic_range_msg.range;


        // Add the magnetometer data.
        ahrs.getMagnData(magn_vector);
        quadcopter_magn_msg.magnetic_field.x = (float) magn_vector[0];
        quadcopter_magn_msg.magnetic_field.y = (float) magn_vector[1];
        quadcopter_magn_msg.magnetic_field.z = (float) magn_vector[2];

        // Add the IMU raw data
        ahrs.getAccelData(acc_vector);
        ahrs.getGyroData(gyro_vector);
        quadcopter_imu_msg.angular_velocity.x = gyro_vector[0] * 0.0012217304764;
        quadcopter_imu_msg.angular_velocity.y = gyro_vector[1] * 0.0012217304764;
        quadcopter_imu_msg.angular_velocity.z = gyro_vector[2] * 0.0012217304764;
        quadcopter_imu_msg.linear_acceleration.x = acc_vector[0];
        quadcopter_imu_msg.linear_acceleration.y = acc_vector[1];
        quadcopter_imu_msg.linear_acceleration.z = acc_vector[2];
    }
}


// Publishes the message
void state_vector_publish(){
    //Publish message
    // quadcopter_pose_pub.publish( &quadcopter_pose_msg );
    quadcopter_twist_pub.publish( &quadcopter_twist_msg );
    quadcopter_magn_pub.publish( &quadcopter_magn_msg );
    quadcopter_imu_pub.publish( &quadcopter_imu_msg );
}
