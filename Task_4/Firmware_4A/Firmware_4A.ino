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
int rear_send = 0;

//k[4]: K matrix, obtained from Octave modelling 
//float k[4] = {  -233.0391  , -16.0481  ,  0.9252  ,  1.0528};
float k[4] = {    -212.2702  , -15.2923  ,  0.9057  ,  1.0366};
//float k[4] = {-218.7902 ,  -15.0624  ,  -0.9932   , -0.9972};
//   -233.0391   -16.0481    0.9252    1.0528

int count = 0;
float init_sum = 0;

//Declare NIDEC Motor pins
#define brake         8  //brake=0, go=1
#define cw            4  //cw=1, ccw=0
#define rpm           9  //PWM=255=stop, PWM=0=max_speed 
#define encodPinAL      2                      // encoder A pin (INT4)
#define encodPinBL      3                      // encoder B pin (INT5)

volatile int encoderPosAL = 0;                  // left count 
float alpha = 0, prev_alpha = 0, alpha_dot = 0;             // count in prev. time step

// Create a servo object
Servo Servo1;
int pos=0;
#define servoPin      6   // Declare the Servo pin

// Pin Definition Declarations for DC Motor on eYFi-Mega
#define enA           7
#define in1           22
#define in2           23

//Pin Defs for Buzzer
#define A12           

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

void servo_init()
{
  Servo1.attach(servoPin);
  pos=90;
  Servo1.write(pos);
}
void servo_move(int nextpos)
{ 
  if(pos<nextpos)
  { for(int i=pos;i<=nextpos;i+=1)
    { Servo1.write(i);
      delay(10);}
  }
  else if(pos>nextpos)
  { for(int i=pos;i>=nextpos;i-=1)
    { Servo1.write(i);
      delay(10); }
  }
  else  { pos=nextpos; }
  pos=nextpos;
}

int sign(int a){
  return a/abs(a);
}

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
  pinMode(29, OUTPUT);
  digitalWrite(29, LOW);
  delay(1000);
  digitalWrite(29, HIGH);
  servo_init();
  Serial.println("Servo initialized\n");
}


// Main loop function
void loop() 
{
    count++;
    x1 = (x*3.14159)/180; //Converts x (in degrees) to radians

    if(count < 201){
       if (count < 200 and count > 100){
           init_sum = init_sum + x1;
           //return;
       }
    }
    else {
    
      curr_time = millis();
      elapsed_time = ((curr_time - prev_time)*0.1);
    
      
      x_dot = (x1 - prev_x)/elapsed_time; // Calculation of x_dot (angular deviation rate)
      alpha = (encoderPosAL*3.14159)/180;
      alpha_dot = (alpha - prev_alpha)/elapsed_time;
      float u = (k[0]*(x1 + 0.06) + k[1]*x_dot + k[2]*alpha + k[3]*alpha_dot); // u = -K*x, from LQR controller
      
      pwm = (1.305*u*elapsed_time); // Multiplied by 'elapsed_time' to convert required torque to PWM frequency 
    
      sendpwm = round(pwm); //Round PWM frequency since we can only give 'int' PWM frequency
    
      //If-else ladder for limiting sent PWM value
      if (sendpwm>255){
        sendpwm = 255;
      }
      else if (sendpwm < -255){
        sendpwm = -255;
      }
    
      //Debugging
      Serial.print(init_sum/100);
      Serial.print("|");
      Serial.print(x1);
      Serial.print("|");
      Serial.print(x_dot);
      Serial.print("|");
      Serial.print(alpha);
      Serial.print("|");
      Serial.print(alpha_dot);
      Serial.println("|");
    
      if ((pwm > 0 && prev_pwm < 0)||(pwm < 0 && prev_pwm > 0)){
        nidec_motor_brake();
      }
  
      rear_send = 150;
  
      if (abs(x1*180/3.14159) > 30){
        sendpwm = 0;
        rear_send = 0;
      }
  //    
      nidec_motor_control(sendpwm);
      Serial.print("|");
      Serial.println(sendpwm);
      servo_move(86);
  
  //    if (count++ > 500){
  //      dc_motor_forward(rear_send);
  //    }
    
      //Assigning values from previous time step
      prev_time = curr_time;
      prev_x = x1;
      prev_pwm = pwm;
      prev_alpha = alpha;
     }
}
