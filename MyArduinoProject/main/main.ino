#include <Arduino.h>
#include "motor.h" // Include the motor class definition

#define MAX_SPEED 255    // Maximum PWM speed
#define PWM_MIN_SPEED 50  // Dead band of the motor compensation. Min value for PWM in order to make motors move!
#define ZERO_INTERPOLATION_FEED 50
#define PID_SAMPLE_TIME 100 // ms  // T = Sample period
#define BOUNCING 2 //based on sample time (this number * sample time), filter for arriving setpoint
#define POT_BOUNC_FILTER_NO   5 // POT_BOUNC_FILTER_NO * PID_SAMPLE_TIME msec. eg: 10*100ms=1sec
#define POT_MAX_DEVIATION  10 // Default ADC resolution with 12 bits/: 3.3 volt / 2^12= 0.8 mv. POT_MAX_DEVIATION*0.8. with 10 it is 8mv - Filter on POT
#define LED_PIN LED_BUILTIN

const long interval = 4000;
int ledState = LOW;  // ledState used to set the LED...

//extern int numMotors;
//extern motor** motors;
double temp;

void setup() {
    Serial.begin(115200);

    motors[0] = new motor(0, 10, 14, 15, 18, 19, 26, PWM_MIN_SPEED, 5.0, 0.01, 0);
    motors[1] = new motor(1, 11, 16, 17, 20, 21, 26, PWM_MIN_SPEED, 5.0, 0.01, 0);

    for (int i = 0; i < numMotors; i++) {
        // Adjust pin numbers as needed to make programming easier
        //  motors[i] = new motor(i, 10 + i, 14 + i, 15 + i, 18 + i, 19 + i, 26 + i, PWM_MIN_SPEED);
        motors[i]->begin();
    }
}

void loop() {
    static bool previousAllMotorInterspolationDone = false;
    static unsigned long currentMillis = 0;
    static bool ledState = LOW;
    // Assume all motors are interpolated until proven otherwise
    bool allMotorInterspolationDone = true;

    if (Serial.available()) {
        Serial.print("m or M = Manual, e.g: P 30 60, paralell movement, eg: S 30 60, Sequencial movement");
        String input = Serial.readStringUntil('\n');

        String setpoints[numMotors + 1];
        int setpointCount = 0;

        // Split the input string into individual mode + setpoints
        int startIndex = 0;
        for (int i = 0; i < input.length(); i++) {
            if (input.charAt(i) == ' ' || i == input.length() - 1) {
                setpoints[setpointCount] = input.substring(startIndex, i == input.length() - 1 ? i + 1 : i);
                startIndex = i + 1;
                setpointCount++;
                if (setpointCount == numMotors + 1) break;
            }
        }
        if (setpoints[0] == "M" || setpoints[0] == "m") {
            Serial.println("Manual mode active for all motors");
            for (int i = 0; i < numMotors; i++) {
                motors[i]->initializeMotorInManual();
                motors[i]->setMode(operationMode::MANUAL);
            }
        }
        else if (setpointCount == numMotors + 1) {
            if (setpoints[0] == "P" || setpoints[0] == "p") {
                for (int i = 0; i < numMotors; i++) {
                    motors[i]->initializeMotorInAuto();
                    motors[i]->setMode(operationMode::PARALLEL_MOVEMENT);
                }
            }
            else if (setpoints[0] == "S" || setpoints[0] == "s") {
                for (int i = 0; i < numMotors; i++) {
                    motors[i]->initializeMotorInAuto();
                    motors[i]->setMode(operationMode::SEQUENTIAL_MOVEMENT);
                }
            }
            for (int i = 0; i < numMotors; i++) {
                motors[i]->setSetPoint(constrain(setpoints[i + 1].toDouble(), 0, 360));
                motors[i]->setStatus(MotorStatus::RUN);
                Serial.print("Motor ");
                Serial.print(i + 1);
                Serial.print(" setpoint: ");
                Serial.println(motors[i]->getSetPoint());
                motors[i]->setPrevSetPoint(motors[i]->getPosition());
            }
        }
        else { Serial.println("Invalid input. Please enter setpoints for all motors."); }
    }

    switch (motors[0]->getMode()) {

    case operationMode::MANUAL:
        //      for (int i = 0; i < numMotors; i++)
        //        motors[i]->PID_Function();
        //      break;
    case operationMode::PARALLEL_MOVEMENT:
        for (int i = 0; i < numMotors; i++)
            motors[i]->PID_Function();
        for (int i = 0; i < numMotors; i++)
            if (!motors[i]->getIntorpolationStatus())
                allMotorInterspolationDone = false;  //even one motor is interpollating, then go false for entire proccess

        break;

    case operationMode::SEQUENTIAL_MOVEMENT:
        for (int i = 0; i < numMotors;) {
            if (!(motors[i]->getIntorpolationStatus())) {
                motors[i]->PID_Function();
                motors[i]->setStatus(MotorStatus::NORM_INTERPOL);
                allMotorInterspolationDone = false;
            }
            else {
                motors[i]->setStatus(MotorStatus::STOP_REST);
                i++;
                //          if(i == numMotors) allMotorInterspolationDone = true;
            }
        }
        break;
    }

    if (motors[0]->getMode() != operationMode::MANUAL)
    {
        // Detect rising edge of allMotorInterspolationDone
        if (allMotorInterspolationDone && !previousAllMotorInterspolationDone) {
            currentMillis = millis();  // This will only execute once when the rising edge is detected
            ledState = !ledState;
            for (int i = 0; i < numMotors; i++) {   // swap prev and current set points
                temp = motors[i]->getSetPoint();
                motors[i]->setSetPoint(motors[i]->getPrevSetPoint());
                motors[i]->setPrevSetPoint(temp);
            }
            digitalWrite(LED_PIN, ledState);
        }

        if (allMotorInterspolationDone && (millis() - currentMillis >= interval)) {
            for (int i = 0; i < numMotors; i++)
                motors[i]->initializeMotorInAuto();
        }

        previousAllMotorInterspolationDone = allMotorInterspolationDone;  // Update the previous state
    }
}
