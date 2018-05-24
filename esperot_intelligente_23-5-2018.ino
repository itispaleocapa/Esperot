//QUESTA VERSIONE FUNZIONA COL SERIALE

#define MOTORLATCH 12
#define MOTORCLK 4
#define MOTORENABLE 7
#define MOTORDATA 8

#define MOTOR1_A 2
#define MOTOR1_B 3
#define MOTOR2_A 1
#define MOTOR2_B 4
#define MOTOR3_A 5
#define MOTOR3_B 7
#define MOTOR4_A 0
#define MOTOR4_B 6

#define MOTOR1_PWM 11
#define MOTOR2_PWM 3
#define MOTOR3_PWM 6
#define MOTOR4_PWM 5

#define FORWARD 1//valori funzione di base
#define BACKWARD 2
#define BRAKE 3

#define RIGHT 4//valori per switch
#define LEFT 5

#include <SoftwareSerial.h>//bluetooth
#define BT_TX_PIN A3//questi sono per l'arduino, (rx arduino <>tx modulo bt)
#define BT_RX_PIN A4
SoftwareSerial bt = SoftwareSerial(BT_RX_PIN, BT_TX_PIN);

#define TRIG A0
#define ECHO A1

#define SPEED_DX 253
#define SPEED_SX 255

short dir1;//direzione
short dir2;
double b;

void setup(){
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  
  pinMode(BT_RX_PIN, INPUT);
  pinMode(BT_TX_PIN, OUTPUT);
 
  Serial.begin(9600);
  bt.begin(9600);
  
  Serial.println("Set up completed");
  
  dir1=BRAKE;//direzione
  dir2=BRAKE;
}


void loop() { 
 while(bt.available()>0){
  unsigned char c=bt.read();
  //Serial.println(c);
   switch (c){
    case 'w'://avanti
      dir1=FORWARD;
    break;
    case 'd'://destra
      dir1=RIGHT;
    break;
    case 's'://indietro
      dir1=BACKWARD;
    break;
    case 'a'://sinistra
      dir1=LEFT;
    break;
    case 'I':
      conn();
      dir1=BRAKE;
      dir2=BRAKE;
    break;
    case 'U':
      disconn();
      dir1=BRAKE;
      dir2=BRAKE;
    break;
    case 'M':
      moonWalk();
      dir1=BRAKE;
      dir2=BRAKE;
    break;
    case 'N':
      spinner();
      dir1=BRAKE;
      dir2=BRAKE;
    break;
    case 'B':
      cuboid();
      dir1=BRAKE;
      dir2=BRAKE;
    break;
    default:
      dir1=BRAKE;
    break;  
  }
  
  while(bt.available()<1){
    switch (dir1){
      case FORWARD://avanti
      
        if(dir1!=dir2/* && dist()>26*/){
          motor(1, BRAKE, 0);
          motor(4, BRAKE, 0);
          delay(100);
          motor(1, FORWARD, SPEED_SX);
          motor(4, FORWARD, SPEED_DX);
          dir2=FORWARD;
        }
        b=dist();
        Serial.println(b);
        if(b<15 && b>0){
          halt();
          dir1=BRAKE;
          dir2=BRAKE;
        }
      break;
      case RIGHT://destra
        if(dir1!=dir2){
          motor(1, BRAKE, 0);
          motor(4, BRAKE, 0);
          delay(100);
          motor(1, FORWARD, 230);
          motor(4, BACKWARD, 230);
          dir2=RIGHT;
        }
      break;
      case BACKWARD://indietro
        if(dir1!=dir2){
          motor(1, BRAKE, 0);
          motor(4, BRAKE, 0);
          delay(100);
          motor(1, BACKWARD, SPEED_SX);
          motor(4, BACKWARD, SPEED_DX);
          dir2=BACKWARD;
        }
      break;
      case LEFT://sinistra
        if(dir1!=dir2){
          motor(1, BRAKE, 0);
          motor(4, BRAKE, 0);
          delay(100);
          motor(1, BACKWARD, 230);
          motor(4, FORWARD, 230);
          dir2=LEFT;
        }
      break;
      case BRAKE://sinistra
        motor(1, BRAKE, 0);
        motor(4, BRAKE, 0);
        dir2=BRAKE;
      break;
    }
   }
 }
}

void conn(){
  motor(1, BRAKE, 0);       motor(4, BRAKE, 0);
  delay(200);
  motor(1, BACKWARD, SPEED_SX);  motor(4, FORWARD, SPEED_DX);
  delay(200);
  motor(1, BRAKE, 0);       motor(4, BRAKE, 0);
  delay(200);
  motor(1, FORWARD, SPEED_SX);   motor(4, BACKWARD, SPEED_DX);
  delay(200);
  motor(1, BRAKE, 0);       motor(4, BRAKE, 0);
}  
void disconn(){
  motor(1, BRAKE, 0);       motor(4, BRAKE, 0);
  delay(200);
  motor(1, FORWARD, SPEED_SX);   motor(4, BACKWARD, SPEED_DX);
  delay(200);
  motor(1, BRAKE, 0);       motor(4, BRAKE, 0);
  delay(200);
  motor(1, BACKWARD, SPEED_SX);  motor(4, FORWARD, SPEED_DX);
  delay(200);
  motor(1, BRAKE, 0);       motor(4, BRAKE, 0);
}  
void halt(){
  motor(1, BRAKE, 0);         motor(4, BRAKE, 0);
  delay(100);
  motor(1, BACKWARD, SPEED_SX);    motor(4, BACKWARD, SPEED_DX);
  delay(200);
  motor(1, BRAKE, 0);         motor(4, BRAKE, 0);
}
void moonWalk(){
  motor(1, BRAKE, 0);         motor(4, BRAKE, 0);
  delay(200);
  motor(1, FORWARD, SPEED_SX);     motor(4, BACKWARD, SPEED_DX);
  delay(900);
  motor(1, BACKWARD, SPEED_SX);    motor(4, BACKWARD, SPEED_DX);
  delay(2000);
  motor(1, BRAKE, 0);         motor(4, BRAKE, 0);
  delay(200);
  motor(1, BACKWARD, SPEED_SX);     motor(4, FORWARD, SPEED_DX);
  delay(900);
  motor(1, BRAKE, 0);         motor(4, BRAKE, 0);
  delay(200);
  motor(1, BACKWARD, SPEED_SX);    motor(4, BACKWARD, SPEED_DX);
  delay(2000);
  motor(1, BRAKE, 0);         motor(4, BRAKE, 0);
}
void spinner(){
  motor(1, BRAKE, 0);         motor(4, BRAKE, 0);
  delay(200);
  motor(1, FORWARD, SPEED_SX);     motor(4, BACKWARD, SPEED_DX);
  delay(5000);
  motor(1, BRAKE, 0);         motor(4, BRAKE, 0);
}
void cuboid(){
  motor(1, BRAKE, 0);         motor(4, BRAKE, 0);
  delay(200);
  motor(1, FORWARD, SPEED_SX);     motor(4, FORWARD, SPEED_DX);
  delay(1000);
  motor(1, BRAKE, 0);         motor(4, BRAKE, 0);
  delay(200);
  motor(1, FORWARD, SPEED_SX);     motor(4, BACKWARD, SPEED_DX);
  delay(500);
  motor(1, BRAKE, 0);         motor(4, BRAKE, 0);
  delay(200);
  motor(1, FORWARD, SPEED_SX);     motor(4, FORWARD, SPEED_DX);
  delay(1000);
  motor(1, BRAKE, 0);         motor(4, BRAKE, 0);
  delay(200);
  for(short i=0;i<3;i++){
    motor(1, FORWARD, SPEED_SX);     motor(4, BACKWARD, SPEED_DX);
    delay(500);
    motor(1, BRAKE, 0);         motor(4, BRAKE, 0);
    delay(200);
    motor(1, FORWARD, SPEED_SX);     motor(4, FORWARD, SPEED_DX);
    delay(2000);
    motor(1, BRAKE, 0);         motor(4, BRAKE, 0);
    delay(200);
  }
  motor(1, FORWARD, SPEED_SX);     motor(4, BACKWARD, SPEED_DX);
  delay(500);
  motor(1, BRAKE, 0);         motor(4, BRAKE, 0);
  delay(200);
  motor(1, FORWARD, SPEED_SX);     motor(4, FORWARD, SPEED_DX);
  delay(1000);
  motor(1, BRAKE, 0);         motor(4, BRAKE, 0);
  delay(200);
  motor(1, BACKWARD, SPEED_SX);     motor(4, FORWARD, SPEED_DX);
  delay(500);
  motor(1, BACKWARD, SPEED_SX);     motor(4, BACKWARD, SPEED_DX);
  delay(1000);
  motor(1, BRAKE, 0);         motor(4, BRAKE, 0);
}

double dist(){
  double a[3],temp;
  for(short i=0;i<3;i++){
    digitalWrite(TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);
    a[i]=pulseIn(ECHO, HIGH)/29.1/2;
  }
  if(a[0]<a[1]){temp=a[0]; a[0]=a[1]; a[1]=temp;}
  if(a[1]<a[2]){temp=a[1]; a[1]=a[2]; a[2]=temp;}
  if(a[0]<a[1]){temp=a[0]; a[0]=a[1]; a[1]=temp;}
  return a[1];
}
  


// Initializing
// ------------
// There is no initialization function.
//
// The shiftWrite() has an automatic initializing.
// The PWM outputs are floating during startup, 
// that's okay for the Motor Shield, it stays off.
// Using analogWrite() without pinMode() is valid.
//
// ---------------------------------
// motor
//
// Select the motor (1-4), the command, 
// and the speed (0-255).
// The commands are: FORWARD, BACKWARD, BRAKE.
//
void motor(int nMotor, int command, int speed)
{
  int motorA, motorB;

  if (nMotor >= 1 && nMotor <= 4)
  {  
    switch (nMotor)
    {
    case 1:
      motorA   = MOTOR1_A;
      motorB   = MOTOR1_B;
      break;
    case 2:
      motorA   = MOTOR2_A;
      motorB   = MOTOR2_B;
      break;
    case 3:
      motorA   = MOTOR3_A;
      motorB   = MOTOR3_B;
      break;
    case 4:
      motorA   = MOTOR4_A;
      motorB   = MOTOR4_B;
      break;
    default:
      break;
    }

    switch (command)
    {
    case FORWARD:
      motor_output (motorA, HIGH, speed);
      motor_output (motorB, LOW, -1);     // -1: no PWM set
      break;
    case BACKWARD:
      motor_output (motorA, LOW, speed);
      motor_output (motorB, HIGH, -1);    // -1: no PWM set
      break;
    case BRAKE:
      motor_output (motorA, LOW, 0);  // 0: output floating.
      motor_output (motorB, LOW, -1); // -1: no PWM set
      break;
    default:
      break;
    }
  }
}


// ---------------------------------
// motor_output
//
// The function motor_ouput uses the motor driver to
// drive normal outputs like lights, relays, solenoids, 
// DC motors (but not in reverse).
//
// It is also used as an internal helper function 
// for the motor() function.
//
// The high_low variable should be set 'HIGH' 
// to drive lights, etc.
// It can be set 'LOW', to switch it off, 
// but also a 'speed' of 0 will switch it off.
//
// The 'speed' sets the PWM for 0...255, and is for 
// both pins of the motor output.
//   For example, if motor 3 side 'A' is used to for a
//   dimmed light at 50% (speed is 128), also the 
//   motor 3 side 'B' output will be dimmed for 50%.
// Set to 0 for completelty off (high impedance).
// Set to 255 for fully on.
// Special settings for the PWM speed:
//    Set to -1 for not setting the PWM at all.
//
void motor_output (int output, int high_low, int speed)
{
  int motorPWM;

  switch (output)
  {
  case MOTOR1_A:
  case MOTOR1_B:
    motorPWM = MOTOR1_PWM;
    break;
  case MOTOR2_A:
  case MOTOR2_B:
    motorPWM = MOTOR2_PWM;
    break;
  case MOTOR3_A:
  case MOTOR3_B:
    motorPWM = MOTOR3_PWM;
    break;
  case MOTOR4_A:
  case MOTOR4_B:
    motorPWM = MOTOR4_PWM;
    break;
  default:
    // Use speed as error flag, -3333 = invalid output.
    speed = -3333;
    break;
  }

  if (speed != -3333)
  {
    // Set the direction with the shift register 
    // on the MotorShield, even if the speed = -1.
    // In that case the direction will be set, but
    // not the PWM.
    shiftWrite(output, high_low);

    // set PWM only if it is valid
    if (speed >= 0 && speed <= 255)    
    {
      analogWrite(motorPWM, speed);
    }
  }
}


// ---------------------------------
// shiftWrite
//
// The parameters are just like digitalWrite().
//
// The output is the pin 0...7 (the pin behind 
// the shift register).
// The second parameter is HIGH or LOW.
//
// There is no initialization function.
// Initialization is automatically done at the first
// time it is used.
//
void shiftWrite(int output, int high_low)
{
  static int latch_copy;
  static int shift_register_initialized = false;

  // Do the initialization on the fly, 
  // at the first time it is used.
  if (!shift_register_initialized)
  {
    // Set pins for shift register to output
    pinMode(MOTORLATCH, OUTPUT);
    pinMode(MOTORENABLE, OUTPUT);
    pinMode(MOTORDATA, OUTPUT);
    pinMode(MOTORCLK, OUTPUT);

    // Set pins for shift register to default value (low);
    digitalWrite(MOTORDATA, LOW);
    digitalWrite(MOTORLATCH, LOW);
    digitalWrite(MOTORCLK, LOW);
    // Enable the shift register, set Enable pin Low.
    digitalWrite(MOTORENABLE, LOW);

    // start with all outputs (of the shift register) low
    latch_copy = 0;

    shift_register_initialized = true;
  }

  // The defines HIGH and LOW are 1 and 0.
  // So this is valid.
  bitWrite(latch_copy, output, high_low);

  // Use the default Arduino 'shiftOut()' function to
  // shift the bits with the MOTORCLK as clock pulse.
  // The 74HC595 shiftregister wants the MSB first.
  // After that, generate a latch pulse with MOTORLATCH.
  shiftOut(MOTORDATA, MOTORCLK, MSBFIRST, latch_copy);
  delayMicroseconds(5);    // For safety, not really needed.
  digitalWrite(MOTORLATCH, HIGH);
  delayMicroseconds(5);    // For safety, not really needed.
  digitalWrite(MOTORLATCH, LOW);
}

