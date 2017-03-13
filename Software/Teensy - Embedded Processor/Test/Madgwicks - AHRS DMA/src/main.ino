#include "AHRS_L3GD20H_LSM303.h"

// Timer to Non-blockingly send the data through the serial port every <insert time here> miliseconds
elapsedMillis serial_timer;
// Time variables for storing how much time is spent inside the filter function
elapsedMicros bench_mark_timer;
float bench_mark;
// Counter to check how many times per second the step function is called.
uint32_t iteration_counter;
MadgwicksFilter ahrs(5,0.2);
float euler[3];
int16_t acc[3];


void setup(){
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWriteFast(LED_BUILTIN, HIGH);
    delay(300);

    Serial.begin(115200);
    // digitalWriteFast(LED_BUILTIN, LOW);
    ahrs.begin();
}


void loop(){

    bench_mark_timer = 0;
    ahrs.step();
    bench_mark = bench_mark_timer;
    iteration_counter ++;
    ahrs.getAttitudeEuler(euler);




    if (serial_timer >= 200) {
        serial_timer = 0;
        Serial.print("Pitch: ");Serial.print(euler[0]);Serial.print(", ");
        Serial.print("Roll: "); Serial.print(euler[1]);Serial.print(", ");
        Serial.print("Yaw: ");  Serial.print(euler[2]);Serial.print(", ");
        Serial.print("STATUS: ");  Serial.print(ahrs.getStatus());Serial.print(", ");
        Serial.print("AvDone: ");  Serial.print(Wire.available());Serial.print(Wire.done());Serial.print(", ");
        Serial.print("Counter: ");  Serial.print(iteration_counter*5);Serial.print(", ");
        Serial.print("timer: ");  Serial.print(bench_mark);Serial.println("");
        digitalWriteFast(LED_BUILTIN, HIGH);
        delay(10);
        digitalWriteFast(LED_BUILTIN, LOW);
        iteration_counter = 0;
    }
}

/*  I2C bus speed vs blocking step speed, benchmark.
400Khz  --> 890uS
800khz  --> 570uS
1.2Mhz  --> 470uS
2Mhz    --> 385uS

getAttitudeEuler takes 145uS.

*/
