/* ----------------------------------------------------------------
 *  
 *  eYRC 22 - e-Yantra 2022-23
 *  
 *  Theme - Delivery Bike
 *  Stage - 2
 *  
 *  Task - 6
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
#include <Servo.h> 
#include <math.h>

MPU6050 mpu(Wire);

// x: Angular deviation (roll) obtained from MPU6050, in degrees
float x; 

// x1: Angular deviation in radians
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

// For Rear DC Motor
int rear_pwm = 45;
float l = 45.0;

//k[4]: K matrix, obtained from Octave modelling 
float k[4] = {  -222.5687  , -15.3226  ,  1.0104  ,  1.0145};

// count: number of times the void loop() has run upto value 500
// init_sum: offset of MPU6050 at the start
int count = 0;
float init_sum = 0;

volatile int encoderPosAL = 0;                      // left count 
float alpha = 0, prev_alpha = 0, alpha_dot = 0;     // count in prev. time step

// For Line Following
int lsa[5] = {0};
float pos=0, last_proportional=0, proportional=0, derivative=0, integral=0, angle=0;
float Kp = 4.80, Ki = 2.15, Kd = 0.00;

//Declare NIDEC Motor pins
#define brake         8            //brake=0, go=1
#define cw            4            //cw=1, ccw=0
#define rpm           9            //PWM=255=stop, PWM=0=max_speed 
#define encodPinAL      2          // encoder A pin (INT4)
#define encodPinBL      3          // encoder B pin (INT5)

// Pin Definition Declarations for DC Motor on eYFi-Mega
#define enA           7
#define in1           22
#define in2           23

// Create a servo object
Servo Servo1;
int pos1=0;
#define servoPin      6   // Declare the Servo pin

// Create a servo object 2
Servo Servo2;
int pos2=0;
#define servoPin2      8   // Declare the Servo pin "Pin 4C"

void rencoderL()  
{                                  
    if(digitalRead(encodPinBL)==HIGH)
    {
         encoderPosAL++;
    }
    else
    {
         encoderPosAL--;
    }
    if (encoderPosAL > 360 || encoderPosAL < -360){
      encoderPosAL = 0; 
    }
    //Serial.println(encoderPosAL);
}

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
// 
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

/////////// Servo 1 ///////////////////
void servo_init()
{
  Servo1.attach(servoPin);
  pos1=93;
  Servo1.write(pos1);
}
void servo_move(int nextpos)
{ 
  if(pos1<nextpos)
  { for(int i=pos1;i<=nextpos;i+=1)
    { Servo1.write(i);
      delay(10);}
  }
  else if(pos1>nextpos)
  { for(int i=pos1;i>=nextpos;i-=1)
    { Servo1.write(i);
      delay(10); }
  }
  else  { pos1=nextpos; }
  pos1=nextpos;
}
//////////////////////////////////////

/////////// Servo 2 ///////////////////
void servo_init2()
{
  Servo2.attach(servoPin2);
  pos2=180;
  Servo2.write(pos2);
}
void servo_move2(int nextpos2)
{ 
  if(pos2 != nextpos2)
  {
    Servo2.write(nextpos2);
  }
  else  { pos2=nextpos2; }
  pos2=nextpos2;
}
//////////////////////////////////////

int sign(int a){
  return a/abs(a);
}

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
  pinMode(encodPinAL, INPUT);
  pinMode(encodPinBL, INPUT);
  digitalWrite(encodPinAL, HIGH);               // turn on pullup resistor
  digitalWrite(encodPinBL, HIGH);               // turn on pullup resistor
  attachInterrupt(digitalPinToInterrupt(2), rencoderL, RISING);        //
  nidec_motor_init();
  Serial.println("NIDEC initialized\n");
  timer1_init(); 
  Serial.println("Timer initialized\n");
  servo_init();
  Serial.println("Servo initialized\n");
  servo_init2();
  Serial.println("Servo2 initialized\n");
  rear_pwm = 45;
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
}


// Main loop function
void loop() 
{

  if(count < 201)
  {
    if (count < 200 and count > 50){
      init_sum += ((x*3.1415)/180);
    }
    count++;
  }
  
  curr_time = millis();
  elapsed_time = ((curr_time - prev_time)*0.1);

  float offset = (init_sum/150);
  x1 = (x*3.14159)/180; //Converts x (in degrees) to radians
  x_dot = (x1 - prev_x)/elapsed_time; // Calculation of x_dot (angular deviation rate)
  alpha = (encoderPosAL*3.14159)/180;
  alpha_dot = (alpha - prev_alpha)/elapsed_time;
  float u = (k[0]*(x1 - offset) + k[1]*x_dot + k[2]*alpha + k[3]*alpha_dot); // u = -K*x, from LQR controller
  
  pwm = (1.305*u*elapsed_time); // Multiplied by 'elapsed_time' to convert required torque to PWM frequency 

  if (x > 3 and x < -3){
    pwm = pwm;
  }
  else if (x > 3){
    pwm = -255;
  }
  else if (x < -3){
    pwm = 255;
  }

  sendpwm = round(pwm); //Round PWM frequency since we can only give 'int' PWM frequency

  //If-else ladder for limiting sent PWM value
  if (sendpwm>255){
    sendpwm = 255;
  }
  else if (sendpwm < -255){
    sendpwm = -255;
  }

  //Debugging
  Serial.print(offset);
  Serial.print("|");
  Serial.print(x1);
  Serial.print("|");
  Serial.print(x_dot);
  Serial.print("|");
  Serial.print(alpha);
  Serial.print("|");
  Serial.print(alpha_dot);

  if ((pwm > 0 && prev_pwm < 0)||(pwm < 0 && prev_pwm > 0)){
    nidec_motor_brake();
  }
  nidec_motor_control(sendpwm);
  Serial.print("|");
  Serial.println(sendpwm);

  if (rear_pwm >= 45 and rear_pwm < 54){
    rear_pwm=round(l);
  }
  else{
    l=55;
    rear_pwm=round(l);
  }

  if (x >= 3){
    rear_pwm = 45;
    l=45;
  }
  else if (x <= -3){
    rear_pwm = 45;
    l=45;
  }

  lsa[0] = (analogRead(A0));
  lsa[1] = (analogRead(A1));
  lsa[2] = (analogRead(A2));
  lsa[3] = (analogRead(A3));
  lsa[4] = (analogRead(A4));
  pos = (0 * lsa[0] + 1000 * lsa[1] + 2000 * lsa[2] + 3000 * lsa[3] + 4000 * lsa[4]) / (lsa[0] + lsa[1] + lsa[2] + lsa[3] + lsa[4]);
  last_proportional = proportional;
  proportional = pos - 2000;
  derivative = proportional - last_proportional;
  integral += proportional;
  angle = proportional * Kp + integral * Ki + derivative * Kd;
  servo_move(round(angle));

  dc_motor_forward(rear_pwm);

  // Delivery Mechanism Code
  if (curr_time > 120000)
  {
    servo_move2(20);
  }
  if (curr_time > 100000)
  {
    servo_move2(50);
  }
  else if (curr_time > 65000)
  {
    servo_move2(77);
  }
  else if (curr_time > 40500)
  {
    servo_move2(118);
  }
  else if(curr_time > 40000)
  {
    //dc_motor_forward(70);
    servo_move2(150);
  }
  else
  {
    servo_move2(180);
  }

  //Assigning values from previous time step
  prev_time = curr_time;
  prev_x = x1;
  prev_pwm = pwm;
  prev_alpha = alpha;
}
