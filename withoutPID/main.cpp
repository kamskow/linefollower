#include <Arduino.h>  
#include <Servo.h>

enum Direction 
{
  Both,
  Left,
  Right  
};

enum Angle 
{
  AngInit   = 90,
  AngMax    = 140,
  AngMin    = 45
};

void init_servo();
void init_motors();
void init_distance_sensor();
void init_reflective_sensors();
void set_speed(short speed, Direction dir);

void dist();

// --- servo
Servo servo;

// --- motors
short inital_speed;
short motors_speedL;
short motors_speedR;
short min_speed;
short max_speed;
int ENA;
int ENB;
int IN1;
int IN2;
int IN3; 
int IN4;

const byte barrier_sens = A0;

// --- reflective_sensors
//refl_reads[0], refl_reads[1] - digital pins
//refl_reads[...] - analog pins
int refl_reads[7];
const byte refl_dig1 = 13;
const byte refl_dig2 = 12;
const byte refl_a1 = A1;
const byte refl_a2 = A2;
const byte refl_a3 = A3;
const byte refl_a4 = A4;
const byte refl_a5 = A5;

void setup() 
{
  init_servo();
  init_motors();
  init_reflective_sensors();

  pinMode(barrier_sens, INPUT);

  Serial.begin(9600); 
}

void init_reflective_sensors()
{
  // --- reflective sensors
  //from left side 13, A1, ... , A5, 12
  pinMode(refl_dig1, INPUT); 
  
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  
  pinMode(refl_dig2, INPUT);
}

void init_motors()
{
  ENA = 5 ;
  ENB = 6 ;

  IN1 = 7 ;
  IN2 = 8 ;
  IN3 = 9 ; 
  IN4 = 10 ;

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  min_speed=120;
  max_speed=255;

  inital_speed = 185;

  motors_speedL=inital_speed;
  motors_speedR=inital_speed;
}

void init_servo()
{
  //  1-2ms Duty Cycle 
  //   __
  //  |  |___________
  //     
  //20ms (PWM Period)
  //
  //
  //Position "0" (1.5 ms pulse) is middle, 
  //"90" (~2ms pulse) is middle, is all the way to the right, 
  //"-90" (~1ms pulse) is all the way to the left.  
  servo.attach(11);  // attaches the servo on pin 11 to the servo object
  servo.write(AngInit);
}

/*
0 - LOW
1 - HIGH

ENX IN1 IN2
0   -    -    wylaczone
1   0   1     do przodu
1   1   0     do tylu
1   1   1     stop
1   0   0     stop
*/

void stop_moving()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void turn_right()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW); 
  digitalWrite(IN3, HIGH); 
  digitalWrite(IN4, LOW);
}

void turn_left()
{
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH); 
}

void move()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void move_back()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void move_forward()
{
  set_speed(inital_speed, Both);
  move();
}

void move_right()
{
  set_speed(motors_speedL, Right);
  turn_left();
}

void move_left()
{
  set_speed(motors_speedR, Left);
  turn_right();
}

void move_backward()
{
  set_speed(inital_speed, Both);
  move_back();
}

void set_speed(short speed, Direction dir)
{
  if (speed<min_speed) 
  {
    speed = min_speed;
  } else if (speed>max_speed) 
  {
    speed = max_speed;
  }

  switch (dir)
  {
    default:
    case Both:
        motors_speedL = speed;
        motors_speedR = speed;
        analogWrite(ENA, speed) ; //set speed range 0 to 255
        analogWrite(ENB, speed);  //set speed range 0 to 255
    break;  

    case Left:
        motors_speedL = speed;
        analogWrite(ENA, speed) ; //set speed range 0 to 255
    break;

    case Right:
        motors_speedR = speed;
        analogWrite(ENB, speed);  //set speed range 0 to 255
    break;
  }
}

void read_sensors()
{
  //digital 0 - white
  //digital 1 - black

  //analogs:
  //high reads (500+) - black
  //low reads -- white

  refl_reads[0] = digitalRead(refl_dig1);
  delay(1);
  refl_reads[2] = analogRead(refl_a1);
  delay(1);
  refl_reads[3] = analogRead(refl_a2);
  delay(1);
  refl_reads[4] = analogRead(refl_a3);
  delay(1);
  refl_reads[5] = analogRead(refl_a4);
  delay(1);
  refl_reads[6] = analogRead(refl_a5);
  delay(1);
  refl_reads[1] = digitalRead(refl_dig2);
}

int get_current_distance()
{
   static float voltage;
   static int dist;

   dist = analogRead(barrier_sens);
   voltage = dist * (5.0 / 1024.0);
  
  switch ((int)voltage*100)
  {
    case 30 ... 40:
      return 80;      

    case 41 ... 50:
      return 65;

    case 51 ... 75:
      return 40;

    case 76 ... 90:
      return 30;

    case 91 ... 110:
      return 25;

    case 111 ... 130:
      return 20;

    case 131 ... 150:
      return 17;

    case 151 ... 350:
      return 1;

    default:
      return 255;
  }
}

unsigned long last_barier_check=0;
bool isP = true;
int angle = AngInit; 
int step = 2;
bool check_if_barrier()
{    
  static int detection_distance=30;
  
  if (get_current_distance() <= detection_distance ) 
  {      
    return true;
  }

  if(isP)
  {
    if(angle + step <= AngMax)
    {
      angle += step;
    }
    else
    {
      isP = false;
    }
  }
  else
  {
    if(angle - step >= AngMin)
    {
      angle -= step;
    }
    else
    {
      isP = true;
    }
  }

  servo.write(angle); // tell servo to go to position in variable 'pos'
  delay(2);

  return false;
}

bool stopProgram()
{
  return false; 
}


void loop() 
{    
  if (check_if_barrier())
  {
      stop_moving();            
      return;
  }
  //digital 0 - white
  //digital 1 - black

  //analogs:
  //high reads (500+) - black
  //low reads -- white

  //float sensors_avg=0;
  //int sensors=0;

  read_sensors();
  if ((refl_reads[0]==1) && (refl_reads[1]==1)) //if digits sensors see black line -> loading space
  {    
    move_forward();
    delay(100);
    stop_moving();
    delay(3000); //loading time - 3 sec
    move_forward();    
  } 

  for (int i=2; i<7; i++)
  {
    if (refl_reads[i]>290)
      refl_reads[i] = 1;
    else
      refl_reads[i] =0;
  }

  int position = -4*refl_reads[2] + -2* refl_reads[3] + refl_reads[4] + 2* refl_reads[5] + 4* refl_reads[6];
    
  switch (position)
  {
    case 0:            
    case 1: 
        move_forward(); 
    break;

    case 2 ... 100:
        move_left();
    break;

    case -100 ... -2: 
        move_right();
    break;
  }
}