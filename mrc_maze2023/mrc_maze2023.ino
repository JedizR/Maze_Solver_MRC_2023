#include <POP32.h>
#include <gp2d120.h>

#define DIR1 5
#define PWM1 6
#define DIR2 4
#define PWM2 7
#define DIR3 16
#define PWM3 19
#define DIR4 17
#define PWM4 20

/*const int MOTOR1_PIN = 5;
const int MOTOR2_PIN = 6;
const int PWM_VALUE = 255;

// Define the IR Sharp sensor pins
const int SHARP_FRONT_PIN = A2;
const int SHARP_LEFT_FRONT_PIN = A1;
const int SHARP_LEFT_BACK_PIN = A0;
const int SHARP_RIGHT_FRONT_PIN = A3;
const int SHARP_RIGHT_BACK_PIN = A4;

// Define the target angle and PID parameters
const int TARGET_ANGLE = 0;
const double KP = 0.5;
const double KI = 0.1;
const double KD = 0.1;
double integral = 0;
double derivative = 0;
double last_error = 0;*/

int Count;
const int base_speed = 90; //40 60 70 80 90

void setup() {
  // Initialize the serial communication and the motor pins
  Serial.begin(9600);
  oled.clear();
  oled.textSize(1);
  initial();
  waitSW_OK();
  //  pinMode(MOTOR1_PIN, OUTPUT);
  //  pinMode(MOTOR2_PIN, OUTPUT);

  // Initialize the IR Sharp sensor pins as inputs
  //  pinMode(SHARP_FRONT_PIN, INPUT);
  //  pinMode(SHARP_LEFT_FRONT_PIN, INPUT);
  //  pinMode(SHARP_LEFT_BACK_PIN, INPUT);
  //  pinMode(SHARP_RIGHT_FRONT_PIN, INPUT);
  //  pinMode(SHARP_RIGHT_BACK_PIN, INPUT);
}

static float previous_error = 0;
static float Kp = 0.05, Ki = 0.0001, Kd = 0.035;
//////main left dis2450 base90 0.048 0.003 0.035
/////right not tune yet 
//0.035 0.003 0.03 40 left
//0.035 0.003 0.028 60 left
//Kp = 0.046, Ki = 0.003, Kd = 0.033 90
//0.039 0.003 0.03  80 //best fast left
static float error, P, I = 0,  D;      // error variables
float total;

void loop() {
    int x = knob(1);
    if(x==0){
      oled.clear();
      oled.text(0,0,"%d left",x);
    }
    else{
      oled.clear();
      oled.text(0,0,"%d right",x);
    }
    oled.show();
    //attachInterrupt(0, CountEn, RISING);
    //Serial.println(Count);
    //ao();
    int left90_sensor = analogRead(0);            //map(analogRead(0) , 0 , 3000 , 0 , 1023);
    int left45_sensor = analogRead(1);            //map(analogRead(1) , 0 , 3000 , 0 , 1023);
    int front = analogRead(2);                    //map(analogRead(2) , 0 , 3000 , 0 , 1023);
    int right45_sensor = analogRead(3);           //map(analogRead(3) , 0 , 3000 , 0 , 1023);
    int right90_sensor = analogRead(4);           //map(analogRead(4) , 0 , 3000 , 0 , 1023);
    if(x==0){
      error = (left90_sensor+left45_sensor)/2-2450; //2150 //2400 //2200
    }
    else{
      error = (right90_sensor+right45_sensor)/2-2470; //2150
    }

    P = error * Kp;

    I = (I + error)*Ki;

    D = (error - previous_error) * Kd;
    previous_error = error;

    total = P+I+D;
    
    Serial.print("total = ");
    Serial.print(total);
    Serial.print(" error = ");
    Serial.print(error);

    Serial.print(" , left90 = ");
    Serial.print(left90_sensor);
    Serial.print(" , left45 = ");
    Serial.print(left45_sensor);
    Serial.print(" , front = ");
    Serial.print(front);
    Serial.print(" , right45 = ");
    Serial.print(right45_sensor);
    Serial.print(" , right90 = ");
    Serial.print(right90_sensor);
    Serial.print("\n");

    //int d_speed=front/3500*base_speed;
    //delay(350);
    
    if(x==0){
      if(front<2800||left90_sensor>1700&&front<2800) motorcontrol(base_speed+total,base_speed-total);
      else if(front>2800){
        if(left90_sensor>1650||right90_sensor<1700){
          motorcontrol(90,-90);
          delay(120);
          motorcontrol(0,0);
          delay(5);
        }
        else{
          motorcontrol(-90,90);
          delay(120);
          motorcontrol(0,0);
          delay(5);
        }
      }
      else {
        motorcontrol(70,70);
      }
    }
    else{
      if(front<2800||right90_sensor>1700&&front<2800) motorcontrol(base_speed-total,base_speed+total);
      else if(front>2800){
        if(right90_sensor>1700||left90_sensor<1700){
          motorcontrol(-90,90);
          delay(120);
          motorcontrol(0,0);
          delay(5);
        }
        else{
          motorcontrol(90,-90);
          delay(120);
          motorcontrol(0,0);
          delay(5);
        }
      }
      else {
        motorcontrol(70,70);
      }
    }


    
    
}
/*void drive_straight(int left45_sensor, int left90_sensor, int right45_sensor, int right90_sensor) // 2 or 4 sensors?
  {
                                           // analog read values of 2 or 4 sensors passed to function
  static int previous_error = 0;
  static int Kp = 16, Ki = 1, Kd = 4;      // constants for scaling P I D effects (will need adjusting)
  static int error, P, I = 0,  D;      // error variables
  int total;

  error = (right45_sensor + right90_sensor)/2 - (left45_sensor + left90_sensor)/2;

  P = error * Kp;

  I = (I + error)*Ki;

  D = (error - previous_error) * Kd;       // may take out
  previous_error = error;

  total = (P+I+D);

  {
    L_enable_val -= (total);
      L_enable_val = constrain(L_enable_val, 30000, 65535);      // may need to adjust

    R_enable_val += (total);
      R_enable_val = constrain(R_enable_val, 30000, 65535);

    pwmWrite(left_enable, L_enable_val);            // enable pins and values
                                                     // must be global
    pwmWrite(right_enable, R_enable_val);          // arduino uses analogWrite
  }
  }*/




void initial() {
  for (int pins = 4; pins <= 20; pins++) {
    pinMode(pins, OUTPUT);
    digitalWrite(pins, LOW);
  }
}
void motor1(int speed1) {
  bool dir = ( speed1 < 0 ? 1 : 0);
  digitalWrite(DIR1, dir);
  analogWrite(PWM1, abs(speed1));
}
void motor2(int speed1) {
  bool dir = ( speed1 < 0 ? 1 : 0);
  digitalWrite(DIR2 , dir);
  analogWrite(PWM2, abs(speed1));
}
void motor3(int speed2) {
  bool dir = ( speed2 < 0 ? 1 : 0);
  digitalWrite(DIR3 , dir);
  analogWrite(PWM3, abs(speed2));
}
void motor4(int speed2) {
  bool dir = ( speed2 < 0 ? 1 : 0);
  digitalWrite(DIR4 , dir);
  analogWrite(PWM4, abs(speed2));
}
void MOTORCONTROL(int speed1, int speed2) { //broken
  if(speed1>100) speed1=100;
  if(speed1<-100) speed1=-100;
  if(speed2>100) speed2=100;
  if(speed2<-100) speed2=-100;
  motor1(speed1);
  motor2(speed1);
  motor3(speed2);
  motor4(speed2);
}

void motorcontrol(int speed1, int speed2) {
  if(speed1>100) speed1=100;
  if(speed1<-100) speed1=-100;
  if(speed2>100) speed2=100;
  if(speed2<-100) speed2=-100;
  motor(1,speed1);
  motor(2,speed1);
  motor(3,speed2);
  motor(4,speed2);
}

void STOP() {
  motorControl( 0, 0);
}
