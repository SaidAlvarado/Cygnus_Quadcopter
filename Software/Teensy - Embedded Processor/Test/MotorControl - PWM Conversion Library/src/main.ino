/*
* rosserial Publisher Example
* Prints "hello world!"
*/


// Interrupt Measurement object and variables.
#define MOTOR1_PIN     3        //1 -- motor 2
#define MOTOR2_PIN     4        //2 -- motor 1
#define MOTOR3_PIN     5        //3
#define MOTOR4_PIN     6
#define PWM_FRECUENCY  500
#define PWM_RESOLUTION 16



// Initialize the PWM peripheral
void motor_setup(){

    // Setup pin directions
    pinMode(MOTOR1_PIN, OUTPUT);
    pinMode(MOTOR2_PIN, OUTPUT);
    pinMode(MOTOR3_PIN, OUTPUT);
    pinMode(MOTOR4_PIN, OUTPUT);

    // Configure PWM frecuency and resolution
    analogWriteFrequency(MOTOR1_PIN, PWM_FRECUENCY);
    analogWriteFrequency(MOTOR2_PIN, PWM_FRECUENCY);
    analogWriteFrequency(MOTOR3_PIN, PWM_FRECUENCY);
    analogWriteFrequency(MOTOR4_PIN, PWM_FRECUENCY);
    analogWriteResolution(PWM_RESOLUTION);

    // Arm the motors and put the pwm in the stopped position
    motor_pwm_write(1, 0);
    motor_pwm_write(2, 0);
    motor_pwm_write(3, 0);
    motor_pwm_write(4, 0);
}



// Function to changes the pwm to the motors
//
// motor_number     =  1, 2, 3, 4.
// motor_percentage =  [0.0, 100.0]    (0 is completely stopped)
void motor_pwm_write(uint8_t motor_number, float motor_percentage){

    // Max  => 60948,  52.85% @ 500Hz
    // Min  => 34636,  92.8%  @ 500Hz
    // Span => 26312

    // Only allow writing to the appropiate motor numbers
    if ((motor_number < 1)  || (motor_number > 4)) return;
    // Constraint percentage values.
    if (motor_percentage < 0) motor_percentage = 0;
    if (motor_percentage > 100) motor_percentage = 100;
    // If zero is received, completely stop the motors
    if (motor_percentage == 0.0) analogWrite(motor_number + 2, 25000);
    // Calculate percentage to bits
    else analogWrite(motor_number + 2,(uint16_t) (motor_percentage * 26312 / 100  + 34636));
}



void setup()
{
    motor_setup();
}

void loop()
{
    // turn on motor.
    motor_pwm_write(1, 0);
    motor_pwm_write(2, 0);
    motor_pwm_write(3, 0);
    motor_pwm_write(4, 0);
    // Wait 3 seconds.
    delay(3000);
    // Turn off motor.
    motor_pwm_write(1,0);
    motor_pwm_write(2,0);
    motor_pwm_write(3,0);
    motor_pwm_write(4,0);
    // Wait 3 seconds
    delay(3000);
}
