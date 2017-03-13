#include "AHRS_L3GD20H_LSM303.h"


elapsedMillis serial_timer;
elapsedMicros bench_mark_timer;
float pito;
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
    pito = bench_mark_timer;
    ahrs.getAttitudeEuler(euler);
    ahrs.getAccelData(acc);



    if (serial_timer >= 200) {
        serial_timer = 0;
        Serial.print("Pitch: ");Serial.print(euler[0]);Serial.print(", ");
        Serial.print("Roll: "); Serial.print(euler[1]);Serial.print(", ");
        Serial.print("Yaw: ");  Serial.print(euler[2]);Serial.print(", ");
        Serial.print("timer: ");  Serial.print(pito);Serial.println("");
        digitalWriteFast(LED_BUILTIN, HIGH);
        delay(10);
        digitalWriteFast(LED_BUILTIN, LOW);
    }
}
