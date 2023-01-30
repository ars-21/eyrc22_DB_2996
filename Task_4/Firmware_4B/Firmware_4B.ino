/* ----------------------------------------------------------------
 *  
 *  eYRC 22 - e-Yantra 2022-23
 *  
 *  Theme - Delivery Bike
 *  Stage - 2
 *  
 *  Task - 4A
 *  
 *  Team ID : 2996
 *  
 *  Team Leader Name: Janhavi Deshpande
 *  Team Member Name: Aryan S. Shah, Vedant R. Nimje, Vaidic Gupta 
 *  
 *  
 *  ----------------------------------------------------------------
*/


#include "Wire.h"
#include <MPU6050_light.h>
#include <math.h>


MPU6050 mpu(Wire);


// x: Angular deviation (roll : along X-Axis) obtained from MPU6050, in degrees
float x=0.0; 

// x1: Angular deviation in radians, which is required in Calculations
// Angular deviation in degrees received from MPU6050 is converted to radians 
float x1 = 0.0;

// prev_x: Angular deviation of DB in previous timestep
float prev_x = 0.0;

// x_dot: Angular velocity of DB
float x_dot = 0.0;

//Variables for determining time step
// curr_time: Time in milliseconds from when the board started
// prev_time: 'curr_time' in previous time step
// elapsed_time: Time step
float prev_time = 0, curr_time = 0, elapsed_time = 0;

//pwm: PWM frequency for NIDEC motor
float pwm = 0;

//prev_pwm: PWM frequency of previous time step
float prev_pwm = 0;

//sendpwm: PWM frequency to be sent to NIDEC motor, rounded from 'pwm'
int sendpwm = 0;

//k[4]: K matrix, obtained from Octave modelling 
float k[4] = {-104.4799, -9.7686, -0.7071, -0.8402};


//Pin Definition Declaration for NIDEC Motor on eYFi-Mega
#define brake         8  //brake=0, go=1                    Green Wire(NIDEC Motor)
#define cw            4  //cw=1, ccw=0                      Orange Wire(NIDEC Motor)
#define rpm           9  //PWM=255=stop, PWM=0=max_speed    Yellow Wire(NIDEC Motor)


/////////////NIDEC Motor//////////////

// Function name: nidec_motor_init()
// Logic: To initlialise the NIDEC motor
// Eg call: nidec_motor_init();
void nidec_motor_init()
{ 
  pinMode(brake, OUTPUT);
  pinMode(cw, OUTPUT);
  pinMode(rpm, OUTPUT);
  
  digitalWrite(brake, HIGH);  //go=1
  digitalWrite(cw, LOW);      //direction ccw
  analogWrite(rpm, 255);
}

// Function name: nidec_motor_control(int pwm)
// Input: pwm - PWM frequency to be sent to the NIDEC motor
// Logic: Sends the required PWM to the NIDEC motor
// Eg call: nidec_motor_control(100);
void nidec_motor_control(int pwm) 
{ 
  if (pwm < 0) 
  { digitalWrite(cw, HIGH);
    pwm = -pwm;} 
  else { digitalWrite(cw, LOW); }
  analogWrite(rpm, 255 - pwm);
}

//Function name: nidec_motor_brake()
//Logic: Stalls the NIDEC motor
//Eg call: nidec_motor_brake();
void nidec_motor_brake() 
{ 
  digitalWrite(brake, LOW);  //go=1
  analogWrite(rpm, 255);
}
//////////////////////////////////////


/////////////Timer1 ISR for IMU///////
void timer1_init()
{
    cli(); //Clears the global interrupts
    TIMSK1 = 0x01; //timer5 overflow interrupt enable
    TCCR1B = 0x00; //stop
    TCNT1H = 0xA2; //Counter higher 8 bit value
    TCNT1L = 0x3F; //Counter lower 8 bit value
    TCCR1A = 0x00;
    TCCR1C = 0x00;
    TCCR1B = 0x02; //start Timer, prescaler 8
    sei();   //Enables the global interrupts
}

// Sends timely interrupts to the MPU6050, for sending the angular deviation
ISR (TIMER1_OVF_vect)
{
    sei();  
    TCNT1H = 0xA2; //Counter higher 8 bit value
    TCNT1L = 0x3F; //Counter lower 8 bit value
    mpu.update();
    cli();
    x=mpu.getAngleX();
}
//////////////////////////////////////

/////////////DC Motor//////////////

//Function name: dc_motor_init()
//Description: Used for initlialising the DC motor
// Eg call: dc_motor_init();
void dc_motor_init()
{
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  
  //Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}

//Function name: dc_motor_forward(int enablePWM)
//Input: enablePWM - Represents the PWM to be given to the DC motor
//Description: Used for giving PWM frequency to the DC motor, and moves it in forward direction
// Eg call: dc_motor_forward(100);
void dc_motor_forward(int enablePWM)
{
  Serial.println(run_rear_motor);
  analogWrite(enA, enablePWM);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}

//Function name: dc_motor_backward(int enablePWM)
//Input: enablePWM - Represents the PWM to be given to the DC motor
//Description: Used for giving PWM frequency to the DC motor, and moves it in backward direction
// Eg call: dc_motor_backward(100);
void dc_motor_backward(int enablePWM)
{
  analogWrite(enA, enablePWM);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}
//////////////////////////////////////


//Function name: setup()
//Logic: Initialises the MPU6050 and the NIDEC motor

void setup() 
{
  Serial.begin(9600);
  Wire.begin();
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  Serial.println("MPU begin done!\n");

  Serial.println("Begin Device initialization:\n");
  nidec_motor_init();
  Serial.println("NIDEC initialized\n");
  timer1_init(); 
  Serial.println("Timer initialized\n");
}


// Main loop function
void loop() 
{
  curr_time = millis();
  elapsed_time = ((curr_time - prev_time)*0.1);


  x1 = ((x*3.14159)/180); //Converts x (in degrees) to radians
  x_dot = (x1 - prev_x)/elapsed_time; // Calculation of x_dot (angular deviation rate)
  float u = (k[0]*(x1) + k[1]*x_dot); // u = -K*x, from LQR controller
  
  prev_pwm = pwm;
  pwm = 1.305*u*elapsed_time; // Multiplied by 'elapsed_time' to convert required torque to PWM frequency 
  sendpwm = round(pwm); //Round PWM frequency since we can only give 'int' PWM frequency

  //If-else ladder for limiting sent PWM value
  if (sendpwm>255){
    sendpwm = 255;
  }
  else if (sendpwm < -255){
    sendpwm = -255;
  }

  //Debugging - to print on Serial Monitor
  Serial.print(x1);
  Serial.print("|");
  Serial.print(x_dot);
  Serial.print("|");
  Serial.print(pwm);
  Serial.print("|");

  //Thresholding PWM values
  if (sendpwm < 10 and sendpwm > 0)
  {
    sendpwm = 10;
  }
  else if (sendpwm > -10 and sendpwm < 0)
  {
    sendpwm = -10;
  }
  
  nidec_motor_control(sendpwm);

  //Giving DC motor required PWM
  dc_motor_forward(50);

  //Debugging - to print on Serial Monitor
  Serial.print(sendpwm);
  Serial.println("|");

  //Assigning values from previous time step
  prev_time = curr_time;
  prev_x = x1;
  
}
