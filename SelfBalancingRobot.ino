#include "Arduino_BMI270_BMM150.h"

// Define the control inputs for motors
#define LM_PIN1 10
#define LM_PIN2 9
#define RM_PIN1 3
#define RM_PIN2 2

const int MIN_PWM_THRESHOLD = 0;

//Raw Acceleration and Gyro values
float xAcc, yAcc, zAcc;
float xGyro, yGyro, zGyro;

//Angle based on gyroscope
double gyroAngleX = 0;
double gyroAngleY = 0;

//Angle based on accelerometer
double accelAngleX;

//time stamp for calculating time elapsed between last measurement
unsigned long previous_time = 0;

//PID parameter values
double Kp = 6.7; //4.5
double Ki = 21; //14
double Kd = 0.175; //0

//PID Terms
double pAngle, iAngle, dAngle;

//PID
double setpoint = -2.5; //Angle setpoint
//double error;
double previous_error = 0;

double integral = 0;
double derivative = 0;
double output = 0;
int constrainedoutput = 0;
int pwm = 0;

void setup()
{
  // Initialize the serial UART at 9600 baud
  Serial.begin(9600);

  if (!IMU.begin()) {
    //Serial.println("Failed to initialize IMU!");
    while (1);
  }
  // Set all the motor control inputs to OUTPUT
  pinMode(LM_PIN1, OUTPUT);
  pinMode(LM_PIN2, OUTPUT);
  pinMode(RM_PIN1, OUTPUT);
  pinMode(RM_PIN2, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(LM_PIN1, LOW);
  digitalWrite(LM_PIN2, LOW);
  digitalWrite(RM_PIN1, LOW);
  digitalWrite(RM_PIN2, LOW);
}

void loop(void) {

  unsigned long current_time = millis();
  unsigned long elapsed_time = current_time - previous_time;

  double error = setpoint - gyroAngleX; //Error calculation 
  //Serial.println(gyroAngleX);
  // Proportional term
  pAngle = Kp * error;

  // Integral term
  integral += error * (elapsed_time / 1000.0);
  iAngle = Ki * integral;
  
  // Derivative term
  derivative = (error - previous_error) / (elapsed_time / 1000.0);
  dAngle = Kd * derivative;

  //Motor output control
  output = constrain(int(pAngle + iAngle + dAngle), -255, 255);

  if (output >= 0){
    analogWrite(LM_PIN1, 255);
    analogWrite(RM_PIN1, 255);  
    analogWrite(LM_PIN2, (238-output));
    analogWrite(RM_PIN2, (238-output));
  }
  else if ((output < 0)){
    analogWrite(LM_PIN2, 255);
    analogWrite(RM_PIN2, 255);
    analogWrite(LM_PIN1, (238-(-output)));
    analogWrite(RM_PIN1, (238-(-output)));
  }

  //Serial.println(output);

  IMU.readGyroscope(xGyro, yGyro, zGyro);
  gyroAngleX += xGyro * (elapsed_time / 1000.0); //Calculate angle according to gyroscope
  IMU.readAcceleration(xAcc, yAcc, zAcc);
  accelAngleX = atan2(yAcc, -zAcc) * RAD_TO_DEG; //Calculate angle according to accelerometer
  gyroAngleX = 0.997 * (gyroAngleX + xGyro * (elapsed_time / 1000.0)) + 0.003 * accelAngleX;  //Combine results using complimentary filter
  Serial.println(gyroAngleX);
  
  // Save current error for next iteration
  previous_error = error;
  previous_time = current_time;
}
/*
void move(int pwm_L, int pwm_R)
{
  set_motor_currents(pwm_L, pwm_R);
}

void set_motor_currents(int pwm_L, int pwm_R)
{
  set_motor_pwm(pwm_L, LM_PIN1, LM_PIN2);
  set_motor_pwm(pwm_R, RM_PIN1, RM_PIN2);
}

void set_motor_pwm(int pwm, int IN1_PIN, int IN2_PIN)
{
  if (pwm < 0) {  // reverse speeds 
    digitalWrite(IN2_PIN, LOW);
    analogWrite(IN1_PIN, -pwm);

  } else { // stop or forward
    digitalWrite(IN1_PIN, LOW);
    analogWrite(IN2_PIN, pwm);
  }
}*/