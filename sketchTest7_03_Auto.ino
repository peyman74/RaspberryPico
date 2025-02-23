
// Ver V7.03   put set point from keybord and positioning with  various PID  

#define MAX_SPEED 255    // Maximum PWM speed 
#define PWM_MIN_SPEED 50
#define PID_SAMPLE_TIME 100 // ms
#define BOUNCING 3 //based on sample time 

const int EncoderCountsPerRevolution = 2450;  // Adjust based on your motor's encoder
const int pwm_pin = 10; // PWM pin to control motor speed
const int forward_pin = 14;  // Pin for forward motion
const int backward_pin = 15; // Pin for backward motion
const int encoderA_pin = 18;  // Encoder A pin
const int encoderB_pin = 19;  // Encoder B pin

volatile int EncoderCounter = 0;  // Track encoder position (counts)
volatile double current_position = 0;  // Motor position in degrees (calculated from encoder)

// PID variables
double Kp = 3 , Ki = 0 , Kd = 0.0 , set_point = 0, feedback = 0, output = 0;
double integral = 0, derivative = 0, error_1 = 0, error_2=0, error=0, sampleTime ;  // For PID calculations
unsigned long last_time = 0;  // Last time the PID was computed
int i=0;
bool interpolationSuccessfullyDone = true;

void interruptHandler();
void setMotor(double out);

void setup() {
  pinMode(pwm_pin, OUTPUT);
  pinMode(forward_pin, OUTPUT);
  pinMode(backward_pin, OUTPUT);
  pinMode(encoderA_pin, INPUT_PULLUP);
  pinMode(encoderB_pin, INPUT_PULLUP);

  Serial.begin(115200);  // Start serial communication
 // Serial.println("Manual PID Motor Control Initialized.");
  attachInterrupt(digitalPinToInterrupt(encoderA_pin), interruptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB_pin), interruptHandler, CHANGE);
  analogWrite(pwm_pin, PWM_MIN_SPEED);  // Set initial PWM value

  sampleTime = PID_SAMPLE_TIME / 1000.0;  //convert T to Sec to used in PID formula
  Ki = Ki * sampleTime;  // corrected Ki, Kd based on T in PID formula
  Kd = Kd / sampleTime;
}

void loop() {
  unsigned long now = millis();
  if ((now - last_time >= PID_SAMPLE_TIME) &&  !interpolationSuccessfullyDone) {
    last_time = now;

    // Update feedback and calculate error
    current_position = (EncoderCounter * 360.0) / EncoderCountsPerRevolution;
    //feedback = current_position;
    error = set_point - current_position;

    // PID calculations diffrent method
   
    //output +=  Kp * ((1 + Ki + Kd) * error - (1 + 2 * Kd) * error_1 + Kd * error_2 );  // PID formula right point integration
    //output +=  Kp * ( (1 +  Kd) * error + (Ki - 1 - 2*Kd) * error_1 + Kd * error_2 );  // PID formula left point integration
    output +=  Kp * ( (1 +  Kd + 0.5*Ki) * error + (0.5*Ki - 1 - 2* Kd) * error_1 + Kd * error_2 );  // PID formula Mid point integration
    
    //integral += Ki * error;  // Accumulate integral 
    //derivative = Kd * (error - error_1);  // Calculate derivative
    //output = Kp * error + integral + derivative;  // //traditinal PID
    
    error_2 = error_1;
    error_1 = error;
    
    //if an error was less and limited (for a periode) then consider the interpolation has bin accomplished
    if (error > -0.5 && error < 0.5)   i++; 
      else i=0;
       
    if (i == BOUNCING) {
       interpolationSuccessfullyDone = true;
       output =0;
       i = 0;
    }

    // Update motor control  
    setMotor(output);

    // Debugging
    Serial.println();
    Serial.print(" Set point: ");
    Serial.print(set_point);
    Serial.print(", Current position (degrees): ");
    Serial.print(current_position);
    Serial.print(", Error: ");
    Serial.print(error);
    Serial.print(", Encoder Count: ");
    Serial.print( EncoderCounter);
    Serial.print(", Output: ");
    Serial.print(output);
    Serial.print(", i: ");
    Serial.print(i);
  }

  // Read set point from serial input
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    set_point = input.toDouble();  // Convert input to double
    interpolationSuccessfullyDone = false;
    i=0;
  }
}

//another type of A and B tracking. Gray code which is filtering and more reliable accorging google search
volatile uint8_t prevState = 0;  // Stores the previous state of A and B
volatile uint8_t currState = 0;  // Stores the current state of A and B
void interruptHandler() {

    int MSB = digitalRead(encoderA_pin); // Most Significant Bit
    int LSB = digitalRead(encoderB_pin); // Least Significant Bit
    currState = (MSB << 1) | LSB; // Convert the two pin states to a single number.
    int sum = (prevState << 2) | currState;

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) EncoderCounter--;
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) EncoderCounter++;

    prevState = currState; // Store current state
    //panPosCount = panEncoderCount;
}


void setMotor(double out) {
  
  if (out > 0) {
    analogWrite(pwm_pin, constrain(out, PWM_MIN_SPEED, MAX_SPEED));
    digitalWrite(forward_pin, HIGH);
    digitalWrite(backward_pin, LOW);
  } else if (out < 0) {
    analogWrite(pwm_pin, constrain(-out, PWM_MIN_SPEED, MAX_SPEED));
    digitalWrite(forward_pin, LOW);
    digitalWrite(backward_pin, HIGH);
  } else {
    analogWrite(pwm_pin, 0);
    digitalWrite(forward_pin, LOW);
    digitalWrite(backward_pin, LOW);
  }
}
