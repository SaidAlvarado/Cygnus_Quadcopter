
#define DELAY_TIMES  1000

void setup(){
    pinMode(LED_BUILTIN, OUTPUT);
}


void loop(){

        digitalWriteFast(LED_BUILTIN, HIGH);
        delay(DELAY_TIMES);
        digitalWriteFast(LED_BUILTIN, LOW);
        delay(DELAY_TIMES);
}
