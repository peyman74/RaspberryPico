#include "motor.h"
#include <Arduino.h>


#define MAX_SPEED 255    // Maximum PWM speed
#define PWM_MIN_SPEED 50  // Dead band of the motor compensation. Min value for PWM in order to make motors move!
#define ZERO_INTERPOLATION_FEED 50
#define PID_SAMPLE_TIME 100 // ms  // T = Sample period
#define BOUNCING 2 //based on sample time (this number * sample time), filter for arriving setpoint
#define POT_BOUNC_FILTER_NO   5 // POT_BOUNC_FILTER_NO * PID_SAMPLE_TIME msec. eg: 10*100ms=1sec
#define POT_MAX_DEVIATION  10 // Default ADC resolution with 12 bits/: 3.3 volt / 2^12= 0.8 mv. POT_MAX_DEVIATION*0.8. with 10 it is 8mv - Filter on POT

// Definition of static member
motor* motor::motorInstances[2] = { nullptr, nullptr };

// Definition of motors and numMotors
int numMotors = 2;
motor** motors = new motor*[numMotors];

motor::motor(int id, int pwm_pin, int forward_pin, int backward_pin, int encoderA_pin, int encoderB_pin, int potentiometer_jogging_pin, int speed_pwmValue, double Kp, double Ki, double Kd)
    : mode(operationMode::SEQUENTIAL_MOVEMENT), Kp(Kp), Ki(Ki), Kd(Kd) {
    this->id = id;
    this->pwm_pin = pwm_pin;
    this->forward_pin = forward_pin;
    this->backward_pin = backward_pin;
    this->encoderA_pin = encoderA_pin;
    this->encoderB_pin = encoderB_pin;
    this->potentiometer_jogging_pin = potentiometer_jogging_pin;
    this->speed_pwmValue = speed_pwmValue;
    this->setStatus(MotorStatus::STOP_REST);
    //this-> setMode(operationMode::MANUAL);
    pinMode(pwm_pin, OUTPUT);
    pinMode(forward_pin, OUTPUT);
    pinMode(backward_pin, OUTPUT);
    pinMode(encoderA_pin, INPUT_PULLUP);
    pinMode(encoderB_pin, INPUT_PULLUP);
    pinMode(potentiometer_jogging_pin, INPUT);
    analogWrite(pwm_pin, speed_pwmValue);

    controlMode = set_value = current_position = EncoderCounter = zeroInterpolationDone = 0;
    interpolationSuccessfullyDone = true;

    if (id < 2) motorInstances[id] = this;  // this method is recommended to access multiple interrupts services in Raspberrypi pico
}

motor::~motor() {
    if(motors != nullptr) {
        for (int i = 0; i < numMotors; i++) {
            delete motors[i];
        }
        delete[] motors;
    }
    Serial.print("destructive executed");
}

void motor::begin() {
    if (motorInstances[0] == this) {
        attachInterrupt(digitalPinToInterrupt(encoderA_pin), handleInterrupt0, CHANGE);
        attachInterrupt(digitalPinToInterrupt(encoderB_pin), handleInterrupt0, CHANGE);
    }
    else if (motorInstances[1] == this) {
        attachInterrupt(digitalPinToInterrupt(encoderA_pin), handleInterrupt1, CHANGE);
        attachInterrupt(digitalPinToInterrupt(encoderB_pin), handleInterrupt1, CHANGE);
    }
}

void motor::handleInterrupt0() {
    if (motorInstances[0]) motorInstances[0]->updateEncoder();
}

void motor::handleInterrupt1() {
    if (motorInstances[1]) motorInstances[1]->updateEncoder();
}

bool motor::PID_Function() {

    unsigned long now = millis();
    if ((now - last_time >= PID_SAMPLE_TIME) && getMode() == operationMode::MANUAL) {  // checking whether POT value changed by putting filter on it
        potValue = analogRead(potentiometer_jogging_pin);
        // this part is ignoring little mvolt changes on POT value reading, otherwise motor oscilate in place
        if (abs(potValue - previous_potValue) < POT_MAX_DEVIATION) { //So if the Read Pot Val was less than POT_BOUNC_FILTER_NO
            potFilterCounter = 0;  //reset counter in order to make consecutive reading
        }
        else
            potFilterCounter++;
        if (potFilterCounter == POT_BOUNC_FILTER_NO) {   //POT_BOUNC_FILTER_NO*PID_SAMPLE_TIME msec. here 5*100=500 msec - if consecutive reading POT for this time stay out of POT_MAX_DEVIATION FiLter value then POT value has been changed in real
            potFilterCounter = 0;
            interpolationSuccessfullyDone = false;          // trigering to start interpolation
            set_value += map(potValue - previous_potValue, 0, 4095, 0, 360);
            previous_potValue = potValue;
        }

    }
    if (now - last_time >= PID_SAMPLE_TIME && !interpolationSuccessfullyDone) {  //needs interpolation
        last_time = now;

        // Update feedback and calculate error
       // current_position = (EncoderCounter * 360.0) / EncoderCountsPerRevolution;
        //feedback = current_position;
        error = set_value - getPosition();

        // PID calculations diffrent method

        //output +=  this-> Kp * ((1 + Ki + Kd) * error - (1 + 2 * Kd) * error_1 + Kd * error_2 );  // PID formula right point integration
        //output +=  this->Kp * ( (1 +  Kd) * error + (Ki - 1 - 2*Kd) * error_1 + Kd * error_2 );  // PID formula left point integration
        output += this->Kp * ((1 + Kd + 0.5 * Ki) * error + (0.5 * Ki - 1 - 2 * Kd) * error_1 + Kd * error_2);  // PID formula Mid point integration


        error_2 = error_1;
        error_1 = error;

        //if an error was less and limited (for a periode) then consider the interpolation has bin accomplished - movingAvrageFilter
        if (error > -0.5 && error < 0.5)   filterCount++;  // Act same as POT filter
        else filterCount = 0;

        if (filterCount == BOUNCING) {
            interpolationSuccessfullyDone = true;
            output = 0;
            filterCount = 0;
        }
        setSpeed(MAX_SPEED);
        setMotor();  // Update motor control
        printDebug(); // Debugging commands
    }
    return interpolationSuccessfullyDone;
}

void motor::updateEncoder() {

    int MSB = digitalRead(encoderA_pin); // Most Significant Bit
    int LSB = digitalRead(encoderB_pin); // Least Significant Bit
    currState = (MSB << 1) | LSB; // Convert the two pin states to a single number.
    int sum = (prevState << 2) | currState;

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) EncoderCounter--;
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) EncoderCounter++;

    prevState = currState; // Store current state
}

void motor::setMotor() {
    if (output > 0) {
        analogWrite(pwm_pin, constrain(output, PWM_MIN_SPEED, speed_pwmValue));
        digitalWrite(forward_pin, HIGH);
        digitalWrite(backward_pin, LOW);
    }
    else if (output < 0) {
        analogWrite(pwm_pin, constrain(-output, PWM_MIN_SPEED, speed_pwmValue));
        digitalWrite(forward_pin, LOW);
        digitalWrite(backward_pin, HIGH);
    }
    else {
        analogWrite(pwm_pin, 0);
        digitalWrite(forward_pin, LOW);
        digitalWrite(backward_pin, LOW);
    }
}

void motor::printDebug() {
    Serial.println();
    Serial.print("MotorId:");
    Serial.print(getId());
    Serial.print(", Set point: ");
    Serial.print(getSetPoint());
    Serial.print(", Current position: ");
    Serial.print(getPosition());
    Serial.print(", Error: ");
    Serial.print(getError());
    Serial.print(", Encoder Count: ");
    Serial.print(getEncoderCount());
    Serial.print(", Output: ");
    Serial.print(getOutput());
    Serial.print(", filterCount: ");
    Serial.print(getFilterCount());
}

void motor::printMode() const {
    switch (mode) {
    case operationMode::MANUAL:
        Serial.print("Mode: Manual");
        break;
    case operationMode::PARALLEL_MOVEMENT:
        Serial.print("Mode: Parallel Movement");
        break;
    case operationMode::SEQUENTIAL_MOVEMENT:
        Serial.print("Mode: Sequential Movement");
        break;
    }
}
