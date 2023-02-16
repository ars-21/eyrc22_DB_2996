#include "Wire.h"
#include <MPU6050_light.h>
#include <Servo.h> 
#include <math.h>


MPU6050 mpu(Wire);
float x=0.0;
float prev_x = 0.0;
float x_dot = 0.0;
float prev_time = 0, curr_time = 0, elapsed_time = 0;
float pwm = 0;
float prev_pwm = 0;
int sendpwm = 0;
int rear_pwm = 45;
float l = 45.0;
float k[4] = {-143.8081, -14.7661, -1.0000, -1.2089};
int lsa[5] = {0};
float pos=0, last_proportional=0, proportional=0, derivative=0, integral=0, angle=0;
float Kp = 0, Ki = 0, Kd = 0;

//Declare NIDEC Motor pins
#define brake         8  //brake=0, go=1
#define cw            4  //cw=1, ccw=0
#define rpm           9  //PWM=255=stop, PWM=0=max_speed  

// Declare the DC Motor Pins
#define enA           7
#define in1           22
#define in2           23

// Create a servo object
Servo Servo1;

#define servoPin      6   // Declare the Servo pin


/////////////NIDEC Motor//////////////
void nidec_motor_init()
{ 
  pinMode(brake, OUTPUT);
  pinMode(cw, OUTPUT);
  pinMode(rpm, OUTPUT);
  
  digitalWrite(brake, HIGH);  //go=1
  digitalWrite(cw, LOW);      //direction ccw
  analogWrite(rpm, 255);
}

void nidec_motor_control(int pwm) 
{ 
  if (pwm < 0) 
  { digitalWrite(cw, HIGH);
    pwm = -pwm;} 
  else { digitalWrite(cw, LOW); }
  analogWrite(rpm, 255 - pwm);
}

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

ISR (TIMER1_OVF_vect)
{
    sei();  
    TCNT1H = 0xA2; //Counter higher 8 bit value
    TCNT1L = 0x3F; //Counter lower 8 bit value
    mpu.update();
    cli();
    x=mpu.getAngleX();
    l+= 0.01;
}

/////////////DC Motor//////////////
void dc_motor_init()
{
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  
  //Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}
void dc_motor_forward(int enablePWM)
{
  analogWrite(enA, enablePWM);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}
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
  //servo_init();
  //Serial.println("Servo initialized\n");
  dc_motor_init();
  Serial.println("DC Motor initialized\n");
  servo_init();
  Serial.println("Servo initialized\n");
  rear_pwm = 45;
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
}

void loop() 
{
  curr_time = millis();
  elapsed_time = ((curr_time - prev_time)*0.1);

  float x1 = ((x*3.14159)/180);
  x_dot = (x1 - prev_x)/elapsed_time;
  float u = (k[0]*(x1) + k[1]*x_dot);
  prev_pwm = pwm;
  pwm = 1.305*u*elapsed_time;

  if (x > 3 and x < -3){
    pwm = pwm;
  }
  else if (x > 3){
    pwm = -255;
  }
  else if (x < -3){
    pwm = 255;
  }

  
  sendpwm = round(pwm);
  
  if (sendpwm > 255){
    sendpwm = 255;
  }
  else if (sendpwm < -255){
    sendpwm = -255;
  }
  
  if (sendpwm < 10 and sendpwm > 0)
  {
    sendpwm = 10;
  }
  else if (sendpwm > -10 and sendpwm < 0)
  {
    sendpwm = -10;
  }
  nidec_motor_control(sendpwm);
  
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
  prev_time = curr_time;
  prev_x = x1;
  
}
