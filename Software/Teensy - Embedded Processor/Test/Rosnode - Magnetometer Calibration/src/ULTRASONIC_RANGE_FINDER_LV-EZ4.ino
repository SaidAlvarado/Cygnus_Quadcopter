/*
* rosserial Publisher Example
* Prints "hello world!"
*/

#include <ros.h>
#include <sensor_msgs/Range.h>

// Initialize ROS Publiser and message
sensor_msgs::Range ultrasonic_range_msg;
ros::Publisher ultrasonic_range_pub("ultrasonic_range", &ultrasonic_range_msg);

// Interrupt Measurement object and variables.
#define ULTRASONIC_RANGE_PIN    15
#define MICROSEC_PER_METER_SCALE_FACTOR 5787.4
elapsedMicros ultrasonic_range_timer;


// Fills in basic information about the Range msg, and initializes the interrupts
void ultrasonic_range_setup(){
    // Type of snesor, ultrasonic vs. IR
    ultrasonic_range_msg.radiation_type = ultrasonic_range_msg.INFRARED;
    // angle of measure aperture (RAD)
    ultrasonic_range_msg.field_of_view = 0;   //Nan = Not Measured
    // Minimum range (m)
    ultrasonic_range_msg.min_range = 0.15;    //Nan = Not Measured
    // maximum range (m)
    ultrasonic_range_msg.max_range = 6.45;
    // Full Charge Nominal Capacity (Ah)
    ultrasonic_range_msg.range = 0;


    // Initialize Interrupt
    // Set pin directions
    pinMode(ULTRASONIC_RANGE_PIN, INPUT);

    //Configure Interrupt
    attachInterrupt(ULTRASONIC_RANGE_PIN, ultrasonic_range_isr, CHANGE);
}

// Interrupt service routine for the Ultrasonic range finder to measure the pulse width of the train pulse
void ultrasonic_range_isr(){
    //check if the pulse is falling of rising
    if (digitalReadFast(ULTRASONIC_RANGE_PIN)){
        // RISING PULSE - reset timer, to start counting
        ultrasonic_range_timer = 0;
    }
    else{
        // FALLING (ENDING) PULSE - Convert time to meters
        ultrasonic_range_msg.range = ultrasonic_range_timer / MICROSEC_PER_METER_SCALE_FACTOR;
    }
}


// Publishes the message
void ultrasonic_range_publish(){
    //Publish message
    ultrasonic_range_pub.publish( &ultrasonic_range_msg );
}
