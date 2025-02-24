#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <pico/multicore.h>
const int EncoderCountsPerRevolution = 2450;


// Forward declaration of class motor
class motor;

// Enums must be declared before the class that uses them
enum class MotorStatus {
    STOP_REST,
    HOLD,
    RUN,
    ZERO_INTERPOL,
    NORM_INTERPOL,
    TIME_OUT,
    ACCELERATION,
    DECELERATION
};

enum class operationMode {
    MANUAL,
    PARALLEL_MOVEMENT,
    SEQUENTIAL_MOVEMENT
};

// Declaration of motors and numMotors
extern int numMotors;
extern motor** motors;

class motor {
private:
    int id;
    int pwm_pin;
    int forward_pin;
    int backward_pin;
    int encoderA_pin;
    int encoderB_pin;
    int potentiometer_jogging_pin;

    MotorStatus motorStatus;
    operationMode mode;

    bool controlMode;
    bool interpolationSuccessfullyDone;
    bool zeroInterpolationDone;
    int speed_pwmValue;

    double Kp, Ki, Kd;
    double set_value;
    double prev_set_value;
    double potValue, previous_potValue;
    unsigned long last_time;
    int filterCount, numberOfCoefficient;
    int potFilterCounter;
    double error_1, error_2, error, output;
    volatile uint8_t prevState;
    volatile uint8_t currState;
    volatile int EncoderCounter;
    volatile double current_position;
    static motor* motorInstances[2];

public:
    // Constructor
    motor(int id, int pwm_pin, int forward_Pin, int backward_pin, int encoderA_pin, int encoderB_pin, int potentiometer_jogging_pin, int speed_pwmValue, double Kp, double Ki, double Kd);
    ~motor();

    void setSpeed(int feed) { speed_pwmValue = feed; };
    void begin();
    void setMode(operationMode newMode) { mode = newMode; };
    void initializeMotorInAuto() { interpolationSuccessfullyDone = false; filterCount = 0; potFilterCounter = 0; };
    void initializeMotorInManual() { previous_potValue = analogRead(potentiometer_jogging_pin); interpolationSuccessfullyDone = false; filterCount = 0; potFilterCounter = 0; };
    int getId() const { return id; };
    operationMode getMode() const { return mode; };
    void setSetPoint(double inputValue) { set_value = inputValue; };
    void setPrevSetPoint(double inputValue) { prev_set_value = inputValue; };
    void setStatus(MotorStatus statusValue) { motorStatus = statusValue; };
    double getSetPoint() const { return set_value; };
    double getPrevSetPoint() const { return prev_set_value; };
    bool getControlMode() const { return  controlMode; };
    MotorStatus getStatus() const { return motorStatus; };
    double getError() const { return error; };
    volatile int getEncoderCount() const { return EncoderCounter; };
    double getOutput() const { return output; };
    bool getIntorpolationStatus() const { return interpolationSuccessfullyDone; };
    bool getZeroInterpolation() const { return zeroInterpolationDone; };
    int getFilterCount() const { return filterCount; };
    int getPwm_pin() const { return pwm_pin; };
    int getForward_pin() const { return forward_pin; };
    int getBackward_pin() const { return backward_pin; };
    double getPosition() { current_position = (EncoderCounter * 360.0) / EncoderCountsPerRevolution; return current_position; };
    bool PID_Function();
    void updateEncoder();
    void setMotor();
    void printDebug();
    void printMode() const;
    static void handleInterrupt0();
    static void handleInterrupt1();
};

#endif
