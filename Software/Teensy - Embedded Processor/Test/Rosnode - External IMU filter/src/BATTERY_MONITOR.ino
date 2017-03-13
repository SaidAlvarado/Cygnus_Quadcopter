/*
* rosserial Publisher Example
* Prints "hello world!"
*/

#include <ros.h>
#include <sensor_msgs/BatteryState.h>
#include <ADC.h>

// Initialize ROS Publiser and message
sensor_msgs::BatteryState lipo_state_msg;
ros::Publisher lipo_state_pub("lipo_battery_state", &lipo_state_msg);

// ADC object and variables.
#define CELL_PIN_1    A6
#define CELL_PIN_12   A7
#define CELL_PIN_123  A8
#define VCELL_THD    3.5
// ADC object
ADC *adc = new ADC(); // adc object;
// FLoating array to pass to the pointer in lipo_state_msg.cell_voltage
float cell_voltage_array[3];

// Fills in basic information about the battery to the message
void lipo_state_setup(){
    // Nominal Voltage (Volts)
    lipo_state_msg.voltage = 11.1;
    // Current entering (+) or leaving (-) the battery (Amp)
    lipo_state_msg.current = 0.0/0.0;   //Nan = Not Measured
    // Current Capacity (Ah)
    lipo_state_msg.charge = 0.0/0.0;    //Nan = Not Measured
    // Full Charge Capacity (Ah)
    lipo_state_msg.capacity = 4;
    // Full Charge Nominal Capacity (Ah)
    lipo_state_msg.design_capacity = 4;
    // Current Percentage of the Battery (0..1)
    lipo_state_msg.percentage = 0.0/0.0;    //Nan = Not Measured
    // Status of the battery
    lipo_state_msg.power_supply_status = lipo_state_msg.POWER_SUPPLY_STATUS_DISCHARGING;
    lipo_state_msg.power_supply_health = lipo_state_msg.POWER_SUPPLY_HEALTH_GOOD;
    lipo_state_msg.power_supply_technology = lipo_state_msg.POWER_SUPPLY_TECHNOLOGY_LIPO;
    // Is the battery coonnected?
    lipo_state_msg.present = true;


    //Initialize cell voltage
    cell_voltage_array[0] = 0.0;
    cell_voltage_array[1] = 0.0;
    cell_voltage_array[2] = 0.0;

    lipo_state_msg.cell_voltage_length = 3;
    lipo_state_msg.cell_voltage = cell_voltage_array;


    // Initialize ADC
    // Set pin directions
    pinMode(CELL_PIN_1, INPUT);
    pinMode(CELL_PIN_12, INPUT);
    pinMode(CELL_PIN_123, INPUT);

    //Configure the ADC
    adc->setAveraging(8); // set number of averages
    adc->setResolution(12); // set bits of resolution
    adc->setConversionSpeed(ADC_HIGH_SPEED); // change the conversion speed
    adc->setSamplingSpeed(ADC_HIGH_SPEED); // change the sampling speed
}

// Measures the battery and publishes the message
void lipo_state_update(){

    // Variables to store adc readings
    int raw_v_cell_1, raw_v_cell_12, raw_v_cell_123;
    //variables to store voltage
    float v_cell1, v_cell2, v_cell3;

    // Read ADC Values
    raw_v_cell_1   = adc->analogRead(CELL_PIN_1,ADC_0);
    raw_v_cell_12  = adc->analogRead(CELL_PIN_12,ADC_0);
    raw_v_cell_123 = adc->analogRead(CELL_PIN_123,ADC_0);

    // Transform to voltages
    v_cell1 = 1.463*raw_v_cell_1*3.3/adc->getMaxValue(ADC_0);
    v_cell2 = 3.07*(raw_v_cell_12)*3.3/adc->getMaxValue(ADC_0);
    v_cell3 = 5.908*(raw_v_cell_123)*3.3/adc->getMaxValue(ADC_0);

    //Separate cells
    v_cell3 -= v_cell2;
    v_cell2 -= v_cell1;

    //Update ROS Battery State Message and send.
    cell_voltage_array[0] = v_cell1;
    cell_voltage_array[1] = v_cell2;
    cell_voltage_array[2] = v_cell3;
    lipo_state_msg.cell_voltage = cell_voltage_array;
}

// Publish the message
void lipo_state_publish(){
    //Publish message
    lipo_state_pub.publish( &lipo_state_msg );
}
