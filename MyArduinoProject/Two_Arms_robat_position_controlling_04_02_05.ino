// Ver 4.02.05 revision for presentation in order to move between two points on and off
//Peiman Edalatjoo 94112 - HKA - February 2025

#include <Arduino.h>
#include <pico/multicore.h>

#define MAX_SPEED 255    // Maximum PWM speed  
#define PWM_MIN_SPEED 50  // Dead band of the motor compensation. Min value for PWM in order to make motors move! 
#define ZERO_INTERPOLATION_FEED 50
#define PID_SAMPLE_TIME 100 // ms  // T = Sample period
#define BOUNCING 2 //based on sample time (this number * sample time), filter for arriving setpoint
#define POT_BOUNC_FILTER_NO   5 // POT_BOUNC_FILTER_NO * PID_SAMPLE_TIME msec. eg: 10*100ms=1sec
#define POT_MAX_DEVIATION  10 // Default ADC resolution with 12 bits/: 3.3 volt / 2^12= 0.8 mv. POT_MAX_DEVIATION*0.8. with 10 it is 8mv - Filter on POT
#define LED_PIN LED_BUILTIN

const double tempSetPoint1 [2]= {0,0};
const double tempSetPoint2 [2]= {90,90};

const int EncoderCountsPerRevolution = 2000;  //// Adjust based on the encoder experiment
const int ZeroInterpolationFeed = 50;
const long interval = 2000;  
int ledState = LOW;  // ledState used to set the LED

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

class motor {
  int id;
  //    hardware configuration
  int pwm_pin;
  int forward_pin;
  int backward_pin;
  int encoderA_pin;
  int encoderB_pin;
  int potentiometer_jogging_pin;
  
  MotorStatus motorStatus;  // Use the new strongly typed enum
  operationMode mode;  // Variable to store the current mode of operation

  bool controlMode;   // 1- Manual mode or jogging mode through potetiometer or 0 - Auto mode by entering target (raduios) as a set point
  bool interpolationSuccessfullyDone;
  bool zeroInterpolationDone;
  int speed_pwmValue;  // 255 is the max speed if resolution would be 1 byte - later change it to maximum by intializing pico pwm registers

  double Kp, Ki, Kd; // Motor-specific PID gains Ki= T/Ti, Kd= Td/T , T = PID sample periode. imprtant: Ki= T/Ti , T sample  and Kd= Td/T. if in PID control loop you are not using either of them, put their value equal zero. in order to avoid from devision by zero I used Ki instead of Ti!
  double set_value;
  double prev_set_value=0;
  double potValue=0, previous_potValue=0;
  unsigned long last_time = 0;  // Last time the PID was computed
  int filterCount=0, numberOfCoefficient=0 ;
  int potFilterCounter=0;
  double error_1 = 0, error_2=0, error=0, output=0 ;  // For PID calculations
  volatile uint8_t prevState = 0;  // Stores the previous state of A and B
  volatile uint8_t currState = 0;  // Stores the current state of A and B
  volatile  int EncoderCounter;             //based on the number of encoder count
  volatile  double current_position;          // Based on the degree
  static motor* motorInstances[2];  // Static array to hold pointers to motor instances

public:
  //custructor
  motor(int id, int pwm_pin, int forward_Pin, int backward_pin, int encoderA_pin, int encoderB_pin, int potentiometer_jogging_pin, int speed_pwmValue, double Kp, double Ki, double Kd);
  ~motor();
  //Methodes
  void setSpeed(int feed) {speed_pwmValue= feed;};
  // zeroInterpolation();
  void begin(); 
  void setMode(operationMode newMode) { mode = newMode;};
  void initializeMotorInAuto() { interpolationSuccessfullyDone = false; filterCount=0; potFilterCounter=0;};
  void initializeMotorInManual() { previous_potValue = analogRead(potentiometer_jogging_pin); interpolationSuccessfullyDone = true; filterCount=0; potFilterCounter=0;};
  int getId(){return id;};
  operationMode getMode() const { return mode;};
  void setSetPoint(double inputValue) {set_value = inputValue;};
  void setPrevSetPoint(double inputValue) {prev_set_value = inputValue;};
  void setStatus(MotorStatus statusValue) { motorStatus = statusValue; };
  double getSetPoint() {return set_value;};
  double getPrevSetPoint() { return prev_set_value; };
  bool getControlMode() {return  controlMode;};
  MotorStatus getStatus() { return motorStatus; };
  double getError() {return error;};
  volatile int getEncoderCount() {return EncoderCounter;};  
  double getOutput() {return output;};
  bool getIntorpolationStatus() { return interpolationSuccessfullyDone; };
  bool getZeroInterpolation(){return zeroInterpolationDone; };
  int getFilterCount() {return filterCount;};
  int getPwm_pin() {return pwm_pin;};
  int getForward_pin(){return forward_pin;};
  int getBackward_pin(){return backward_pin;};
  double getPosition() { current_position = (EncoderCounter * 360.0) / EncoderCountsPerRevolution; return current_position; };
  bool PID_Function();
  void updateEncoder();
  void setMotor();
  void printDebug();
  void printMode() const;
  static void handleInterrupt0();
  static void handleInterrupt1();
};

motor* motor::motorInstances[2] = { nullptr, nullptr };  //for two motors
int numMotors=2;
motor** motors = new motor*[numMotors];  // Correctly allocate memory for motor pointers

motor::motor(int id, int pwm_pin, int forward_pin, int backward_pin, int encoderA_pin, int encoderB_pin, int potentiometer_jogging_pin, int speed_pwmValue, double Kp, double Ki, double Kd)
 : mode(operationMode::PARALLEL_MOVEMENT), Kp(Kp), Ki(Ki), Kd(Kd) {
  this->id = id;
  this->pwm_pin = pwm_pin;
  this->forward_pin = forward_pin;
  this->backward_pin = backward_pin;
  this->encoderA_pin = encoderA_pin;
  this->encoderB_pin = encoderB_pin;
  this->potentiometer_jogging_pin = potentiometer_jogging_pin;
  this->speed_pwmValue = speed_pwmValue;  
  this-> setStatus(MotorStatus::STOP_REST); 
  //this-> setMode(operationMode::MANUAL);
  pinMode(pwm_pin, OUTPUT);
  pinMode(forward_pin, OUTPUT);
  pinMode(backward_pin, OUTPUT);
  pinMode(encoderA_pin, INPUT_PULLUP);
  pinMode(encoderB_pin, INPUT_PULLUP);
  pinMode(potentiometer_jogging_pin, INPUT);
  analogWrite(pwm_pin, speed_pwmValue);

  controlMode = set_value = current_position = EncoderCounter  = zeroInterpolationDone= 0;
  interpolationSuccessfullyDone=true;
 
  if (id < 2) motorInstances[id] = this;  // this method is recommended to access multiple interrupts services in Raspberrypi pico
}  

motor::~motor(){
  for (int i = 0; i < numMotors; i++) {
    delete motors[i];
  }
  delete[] motors;
  Serial.print("destructive executed");

} 

void motor::begin() {
    if (motorInstances[0] == this) {
        attachInterrupt(digitalPinToInterrupt(encoderA_pin), handleInterrupt0, CHANGE);
        attachInterrupt(digitalPinToInterrupt(encoderB_pin), handleInterrupt0, CHANGE);
    } else if (motorInstances[1] == this) {
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

bool motor::PID_Function(){

  unsigned long now = millis();  
  if ((now - last_time >= PID_SAMPLE_TIME) &&   getMode() == operationMode::MANUAL) {  // checking whether POT value changed by putting filter on it
    potValue = analogRead(potentiometer_jogging_pin); 
    // this part is ignoring little mvolt changes on POT value reading, otherwise motor oscilate in place
    if (abs(potValue - previous_potValue) < POT_MAX_DEVIATION ){ //So if the Read Pot Val was less than POT_BOUNC_FILTER_NO 
      potFilterCounter=0;  //reset counter in order to make consecutive reading 
    }
    else
      potFilterCounter++;
    if (potFilterCounter == POT_BOUNC_FILTER_NO){   //POT_BOUNC_FILTER_NO*PID_SAMPLE_TIME msec. here 5*100=500 msec - if consecutive reading POT for this time stay out of POT_MAX_DEVIATION FiLter value then POT value has been changed in real
      potFilterCounter=0;
      interpolationSuccessfullyDone=false;          // trigering to start interpolation
      set_value += map(potValue - previous_potValue, 0, 4095, 0, 360); 
      previous_potValue=potValue;
    }

  } 
  if ( now - last_time >= PID_SAMPLE_TIME &&  !interpolationSuccessfullyDone ) {  //needs interpolation 
    last_time = now;

    // Update feedback and calculate error
   // current_position = (EncoderCounter * 360.0) / EncoderCountsPerRevolution;
    //feedback = current_position;
    error = set_value - getPosition();

    // PID calculations diffrent method
   
    //output +=  this-> Kp * ((1 + Ki + Kd) * error - (1 + 2 * Kd) * error_1 + Kd * error_2 );  // PID formula right point integration
    //output +=  this->Kp * ( (1 +  Kd) * error + (Ki - 1 - 2*Kd) * error_1 + Kd * error_2 );  // PID formula left point integration
    output +=  this-> Kp * ( (1 +  Kd + 0.5*Ki) * error + (0.5*Ki - 1 - 2* Kd) * error_1 + Kd * error_2 );  // PID formula Mid point integration
    
  
    error_2 = error_1;
    error_1 = error;
    
    //if an error was less and limited (for a periode) then consider the interpolation has bin accomplished - movingAvrageFilter
    if (error > -0.5 && error < 0.5)   filterCount++;  // Act same as POT filter
      else filterCount=0;
       
    if (filterCount == BOUNCING) {
       interpolationSuccessfullyDone = true;
       output =0;
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
  } else {
    analogWrite(pwm_pin, 0);
    digitalWrite(forward_pin, LOW);
    digitalWrite(backward_pin, LOW);
  }    

}


void setup() {
  Serial.begin(115200);
   

  motors[0] = new motor(0, 10, 14, 15, 18, 19, 26, PWM_MIN_SPEED, 3.0, 0.0, 0);
  motors[1] = new motor(1, 11, 16, 17, 20, 21, 26, PWM_MIN_SPEED, 3.0, 0.0, 0);
  
  for (int i = 0; i < numMotors; i++) {
    // Adjust pin numbers as needed to make programming easier
  //  motors[i] = new motor(i, 10 + i, 14 + i, 15 + i, 18 + i, 19 + i, 26 + i, PWM_MIN_SPEED);
    motors[i]->begin();
    motors[i]->setSetPoint(tempSetPoint2[i]);
    motors[i]->initializeMotorInAuto();

  }
}  

void loop() {
    static bool previousAllMotorInterpolationDone = false;
    static unsigned long currentMillis = 0;
    static bool ledState = LOW;  
    // Assume all motors are interpolated until proven otherwise
    bool allMotorInterpolationDone = true;

    for (int i = 0; i < numMotors; i++)
        motors[i]->PID_Function();
    for (int i = 0; i < numMotors; i++) 
        if (!motors[i]->getIntorpolationStatus()) 
            allMotorInterpolationDone = false;  //even one motor is interpolating, then go false for entire process 

    // Detect rising edge of allMotorInterpolationDone
    if (allMotorInterpolationDone && !previousAllMotorInterpolationDone) {
        currentMillis = millis();  // This will only execute once when the rising edge is detected
        ledState = !ledState;
        if (ledState == HIGH)
            for (int i = 0; i < numMotors; i++) 
                motors[i]->setSetPoint(tempSetPoint1[i]);
        else
            for (int i = 0; i < numMotors; i++) 
                motors[i]->setSetPoint(tempSetPoint2[i]);    
        digitalWrite(LED_PIN, ledState); 
    }

    if (allMotorInterpolationDone && (millis() - currentMillis >= interval)) {
        for (int i = 0; i < numMotors; i++) 
            motors[i]->initializeMotorInAuto();
    }     

    previousAllMotorInterpolationDone = allMotorInterpolationDone;  // Update the previous state
}


void motor::printDebug(){
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
    Serial.print(", InterpolStatus: ");
//    Serial.print(getIntorpolationStatus()); 
//    Serial.print(", Status: ");
    Serial.print(static_cast<int>(getStatus())); 
    printMode();  
}
void motor::printMode()  const{
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



