#include <Servo.h>
#include <math.h>

#define Echo 7 //"Echo" labeled pin of HC-SR04 ultrasonic module attach to pin 7 on Arduino UNO board
#define Trigger 8 //"Trig" labeled pin of HC-SR04 ultrasonic module attach to pin 8 on Arduino UNO board

#define S0 A1 //"S0" labeled pin of TCS230 color sensing module attach to pin A1 on Arduino UNO board
#define S1 A2 //"S1" labeled pin of TCS230 color sensing module attach to pin A2 on Arduino UNO board
#define S2 A3 //"S2" labeled pin of TCS230 color sensing module attach to pin A3 on Arduino UNO board
#define S3 A4 //"S3" labeled pin of TCS230 color sensing module attach to pin A4 on Arduino UNO board
#define sensorOut A5 //"OUT" labeled pin of TCS230 color sensing module attach to pin A5 of Arduino UNO board

long Time;//the time variable for ultrasnonic senor
float Distance;//the distance variable for ultrasonic sensor

int mapped_red_freq;
int mapped_green_freq;
int mapped_blue_freq;
int Red;
int Green;
int Blue;

Servo servo1; //servo of the 1st joint
Servo servo2; //servo of the 2nd joint
Servo servo3; //servo of the 3rd joint
Servo servo4; //servo of the 4th joint
Servo gripper; //servo to drive the gripping mechanism of gripper module

float min_PWM; //default arduino 544
float max_PWM; //default arduino 2400

float Pi = 3.141592653589793;
float Yoffset;
float D;
float d;
float R;
float L1 = 64.50; //height of the first link from surface to 2nd joint position
float L2 = 105.00; //length of the second link from 2nd joint to 3rd joint
float L3 = 98.50; //length of the 3rd joint to 4th joint
float L4 = 115.00; //length of the 4th joint to the tip of the arm (gripper)

float X_End_Effector; //x axis coordinate of the gripper
float Y_End_Effector; //y axis coordinate of the gripper
float Z_End_Effector; //z axis coordinate of the gripper

int pin_servo1 = 9; //joint 1 servo (yellow cable) attach to pin 9 on the arduino board
int pin_servo2 = 5; //joint 2 servo (yellow cable) attach to pin 5 on the arduino board
int pin_servo3 = 10; //joint 3 servo (yellow cable) attach to pin 10 on the arduino board
int pin_servo4 = 6; //joint 4 servo (yellow cable) attach to pin 6 on the arduino board
int pin_gripper = 3; //gripper servo (yellow cable) attach to pin 3 on the arduino board

float alpha1;
float alpha2;
float Theta_2;  
float Theta_4;  
float Theta_1;  
float Theta_3;  

void Rumus_IK()
{
  if (X_End_Effector > 0 && Y_End_Effector >= L1)
  {
    D = sqrt(pow(X_End_Effector,2) + pow(Z_End_Effector,2));
    Theta_1 = (atan(Z_End_Effector/X_End_Effector))*(180.00/Pi); //theta 1
    d = D - L4;
    Yoffset = Y_End_Effector - L1;
    R = sqrt(pow(d,2) + pow(Yoffset,2));
    alpha1 = (acos(d/R))*(180.00/Pi);
    alpha2 = (acos((pow(L2,2) + pow(R,2) - pow(L3,2))/(2*L2*R)))*(180.00/Pi);
    Theta_2 = (alpha1 + alpha2); //theta 2
    Theta_3 = ((acos((pow(L2,2) + pow(L3,2) - pow(R,2))/(2*L2*L3)))*(180.00/Pi)); //theta 3
    Theta_4 = 180.00 - ((180.00 - (alpha2 + Theta_3)) - alpha1); //theta 4
  }
  else if (X_End_Effector > 0 && Y_End_Effector <= L1)
  {
    D = sqrt(pow(X_End_Effector,2) + pow(Z_End_Effector,2));
    Theta_1 = (atan(Z_End_Effector/X_End_Effector))*(180.00/Pi); //theta 1
    d = D - L4;
    Yoffset = Y_End_Effector - L1;
    R = sqrt(pow(d,2) + pow(Yoffset,2));
    alpha1 = (acos(d/R))*(180.00/Pi);
    alpha2 = (acos((pow(L2,2) + pow(R,2) - pow(L3,2))/(2*L2*R)))*(180.00/Pi);
    Theta_2 = (alpha2 - alpha1); //theta 2
    Theta_3 = ((acos((pow(L2,2) + pow(L3,2) - pow(R,2))/(2*L2*L3)))*(180.00/Pi)); //theta 3
    Theta_4 = 180.00 - ((180.00 - (alpha2 + Theta_3)) + alpha1); //theta 4
  }
  else if (X_End_Effector == 0 && Y_End_Effector >= L1)
  {
    D = sqrt(pow(X_End_Effector,2) + pow(Z_End_Effector,2));
    Theta_1 = 90.00; //theta 1
    d = D - L4;
    Yoffset = Y_End_Effector - L1;
    R = sqrt(pow(d,2) + pow(Yoffset,2));
    alpha1 = (acos(d/R))*(180.00/Pi);
    alpha2 = (acos((pow(L2,2) + pow(R,2) - pow(L3,2))/(2*L2*R)))*(180.00/Pi);
    Theta_2 = (alpha1 + alpha2); //theta 2
    Theta_3 = ((acos((pow(L2,2) + pow(L3,2) - pow(R,2))/(2*L2*L3)))*(180.00/Pi)); //theta 3
    Theta_4 = 180.00 - ((180.00 - (alpha2 + Theta_3)) - alpha1); //theta 4
  }
  else if (X_End_Effector == 0 && Y_End_Effector <= L1)
  {
    D = sqrt(pow(X_End_Effector,2) + pow(Z_End_Effector,2));
    Theta_1 = 90.00; //theta 1
    d = D - L4;
    Yoffset = Y_End_Effector - L1;
    R = sqrt(pow(d,2) + pow(Yoffset,2));
    alpha1 = (acos(d/R))*(180.00/Pi);
    alpha2 = (acos((pow(L2,2) + pow(R,2) - pow(L3,2))/(2*L2*R)))*(180.00/Pi);
    Theta_2 = (alpha2 - alpha1); //theta 2
    Theta_3 = ((acos((pow(L2,2) + pow(L3,2) - pow(R,2))/(2*L2*L3)))*(180.00/Pi)); //theta 3
    Theta_4 = 180.00 - ((180.00 - (alpha2 + Theta_3)) + alpha1); //theta 4
  }
  else if (X_End_Effector < 0 && Y_End_Effector >= L1)
  {
    D = sqrt(pow(X_End_Effector,2) + pow(Z_End_Effector,2));
    Theta_1 = 90.00 + (90.00 - abs((atan(Z_End_Effector/X_End_Effector))*(180.00/Pi))); //theta 1
    d = D - L4;
    Yoffset = Y_End_Effector - L1;
    R = sqrt(pow(d,2) + pow(Yoffset,2));
    alpha1 = (acos(d/R))*(180.00/Pi);
    alpha2 = (acos((pow(L2,2) + pow(R,2) - pow(L3,2))/(2*L2*R)))*(180.00/Pi);
    Theta_2 = (alpha1 + alpha2); //theta 2
    Theta_3 = ((acos((pow(L2,2) + pow(L3,2) - pow(R,2))/(2*L2*L3)))*(180.00/Pi)); //theta 3
    Theta_4 = 180.00 - ((180.00 - (alpha2 + Theta_3)) - alpha1); //theta 4
  }
  else if (X_End_Effector < 0 && Y_End_Effector <= L1)
  {
    D = sqrt(pow(X_End_Effector,2) + pow(Z_End_Effector,2));
    Theta_1 = 90.00 + (90.00 - abs((atan(Z_End_Effector/X_End_Effector))*(180.00/Pi))); //theta 1
    d = D - L4;
    Yoffset = Y_End_Effector - L1;
    R = sqrt(pow(d,2) + pow(Yoffset,2));
    alpha1 = (acos(d/R))*(180.00/Pi);
    alpha2 = (acos((pow(L2,2) + pow(R,2) - pow(L3,2))/(2*L2*R)))*(180.00/Pi);
    Theta_2 = (alpha2 - alpha1); //theta 2
    Theta_3 = ((acos((pow(L2,2) + pow(L3,2) - pow(R,2))/(2*L2*L3)))*(180.00/Pi)); //theta 3
    Theta_4 = 180.00 - ((180.00 - (alpha2 + Theta_3)) + alpha1); //theta 4
  }
}

void setup() 
{
  Serial.begin(9600);
  pinMode(Trigger, OUTPUT);
  pinMode(Echo, INPUT);
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  servo1.attach(pin_servo1, min_PWM = 550.0, max_PWM = 550.00+(180.00/(209.00/(2250.00-550.00))));
  servo2.attach(pin_servo2, min_PWM = 500.0, max_PWM = 2500.00);
  servo3.attach(pin_servo3, min_PWM = 500.0, max_PWM = 2500.00);
  servo4.attach(pin_servo4, min_PWM = 550.0, max_PWM = 550.00+(180.00/(164.00/(2000.00-550.00))));
  gripper.attach(pin_gripper); //gripper
}

void loop()
{
  digitalWrite(Trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(Trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trigger, LOW);
  Time = pulseIn(Echo, HIGH);
  Distance = Time*0.034/2;
  //Serial.print("Distance = ");
  //Serial.println(Distance);
  
  //filter Red
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  mapped_red_freq = pulseIn(sensorOut,LOW);
  Red = map(mapped_red_freq,144,340,0,512 );
  Serial.print("R= ");
  Serial.print(mapped_red_freq);
  Serial.print("  ");
  delay(10);
  
  //filter Green
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  mapped_green_freq = pulseIn(sensorOut,LOW);
  Green = map(mapped_green_freq,200,350,0,512);
  Serial.print("G= ");
  Serial.print(mapped_green_freq);
  Serial.print("  ");
  delay(10);
  
  //filter Blue
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  mapped_blue_freq = pulseIn(sensorOut,LOW);
  Blue = map(mapped_blue_freq,175,320,0,512); 
  Serial.print("B= ");
  Serial.print(mapped_blue_freq);
  Serial.println("  ");
  delay(10);
  
  if(Distance > 10.00)
  {
    gripper.write(150);
    delay(200);
    X_End_Effector = 180.00;
    Z_End_Effector = 10.00;
    Y_End_Effector = 150.00;
    Rumus_IK();
    servo1.write(Theta_1);
    servo2.write(Theta_2);
    servo3.write(Theta_3);
    servo4.write(Theta_4 - 45.00);
    delay(100);
  }
  else if(Distance <= 10.00 && mapped_blue_freq < 150 && mapped_red_freq > 150 && mapped_green_freq > 150)//blue object detected
  {
    gripper.write(90);
    for(Y_End_Effector = 150.00; Y_End_Effector >= 40.00; Y_End_Effector -= 0.10)
    {
      X_End_Effector = 180.00;
      Z_End_Effector = 10.00;
      Rumus_IK();
      servo1.write(Theta_1);
      servo2.write(Theta_2);
      servo3.write(Theta_3);
      servo4.write(Theta_4 - 45.00);
      delayMicroseconds(100);
    }
    delay(300);
    gripper.write(120);
    delay(300);
    for(Y_End_Effector = 40.00; Y_End_Effector <= 150.00; Y_End_Effector += 0.10)
    {
      X_End_Effector = 180.00;
      Z_End_Effector = 10.00;
      Rumus_IK();
      servo1.write(Theta_1);
      servo2.write(Theta_2);
      servo3.write(Theta_3);
      servo4.write(Theta_4 - 45.00);
      delayMicroseconds(100);
    }
    for(Z_End_Effector = 10.00; Z_End_Effector <= 100.00; Z_End_Effector += 0.10)
    {
      Y_End_Effector = 150.00;
      X_End_Effector = 180.00;
      Rumus_IK();
      servo1.write(Theta_1);
      servo2.write(Theta_2);
      servo3.write(Theta_3);
      servo4.write(Theta_4 - 45.00);
      delayMicroseconds(100);
    }
    for(X_End_Effector = 180.00; X_End_Effector >= -50.00; X_End_Effector -= 0.10)
    {
      Y_End_Effector = 150.00;
      Z_End_Effector = 100.00;
      Rumus_IK();
      servo1.write(Theta_1);
      servo2.write(Theta_2);
      servo3.write(Theta_3);
      servo4.write(Theta_4 - 45.00);
      delayMicroseconds(100);
    }
    for(Z_End_Effector = 100.00; Z_End_Effector <= 250.00; Z_End_Effector += 0.10)
    {
      Y_End_Effector = 150.00;
      X_End_Effector = -50.00;
      Rumus_IK();
      servo1.write(Theta_1);
      servo2.write(Theta_2);
      servo3.write(Theta_3);
      servo4.write(Theta_4 - 45.00);
      delayMicroseconds(100);
    }
    for(Y_End_Effector = 150.00; Y_End_Effector >= 50.00; Y_End_Effector -= 0.10)
    {
      X_End_Effector = -50.00;
      Z_End_Effector = 250.00;
      Rumus_IK();
      servo1.write(Theta_1);
      servo2.write(Theta_2);
      servo3.write(Theta_3);
      servo4.write(Theta_4 - 45.00);
      delayMicroseconds(100);
    }
    delay(100);
    gripper.write(90);
    delay(100);
    for(Y_End_Effector = 50.00; Y_End_Effector <= 150.00; Y_End_Effector += 0.10)
    {
      X_End_Effector = -50.00;
      Z_End_Effector = 250.00;
      Rumus_IK();
      servo1.write(Theta_1);
      servo2.write(Theta_2);
      servo3.write(Theta_3);
      servo4.write(Theta_4 - 45.00);
      delayMicroseconds(100);
    }
    delay(100);
  }
  else if (Distance <= 10.00 && mapped_blue_freq < 180 && mapped_red_freq < 200 && mapped_green_freq < 150)//Green object detected
  {
    gripper.write(90);
    for(Y_End_Effector = 150.00; Y_End_Effector >= 40.00; Y_End_Effector -= 0.10)
    {
      X_End_Effector = 180.00;
      Z_End_Effector = 10.00;
      Rumus_IK();
      servo1.write(Theta_1);
      servo2.write(Theta_2);
      servo3.write(Theta_3);
      servo4.write(Theta_4 - 45.00);
      delayMicroseconds(100);
    }
    delay(300);
    gripper.write(120);
    delay(300);
    for(Y_End_Effector = 40.00; Y_End_Effector <= 150.00; Y_End_Effector += 0.10)
    {
      X_End_Effector = 180.00;
      Z_End_Effector = 10.00;
      Rumus_IK();
      servo1.write(Theta_1);
      servo2.write(Theta_2);
      servo3.write(Theta_3);
      servo4.write(Theta_4 - 45.00);
      delayMicroseconds(100);
    }
    for(Z_End_Effector = 10.00; Z_End_Effector <= 100.00; Z_End_Effector += 0.10)
    {
      Y_End_Effector = 150.00;
      X_End_Effector = 180.00;
      Rumus_IK();
      servo1.write(Theta_1);
      servo2.write(Theta_2);
      servo3.write(Theta_3);
      servo4.write(Theta_4 - 45.00);
      delayMicroseconds(100);
    }
    for(X_End_Effector = 180.00; X_End_Effector >= -150.00; X_End_Effector -= 0.10)
    {
      Y_End_Effector = 150.00;
      Z_End_Effector = 100.00;
      Rumus_IK();
      servo1.write(Theta_1);
      servo2.write(Theta_2);
      servo3.write(Theta_3);
      servo4.write(Theta_4 - 45.00);
      delayMicroseconds(100);
    }
    for(Z_End_Effector = 100.00; Z_End_Effector <= 250.00; Z_End_Effector += 0.10)
    {
      Y_End_Effector = 150.00;
      X_End_Effector = -150.00;
      Rumus_IK();
      servo1.write(Theta_1);
      servo2.write(Theta_2);
      servo3.write(Theta_3);
      servo4.write(Theta_4 - 45.00);
      delayMicroseconds(100);
    }
    for(Y_End_Effector = 150.00; Y_End_Effector >= 60.00; Y_End_Effector -= 0.10)
    {
      X_End_Effector = -150.00;
      Z_End_Effector = 250.00;
      Rumus_IK();
      servo1.write(Theta_1);
      servo2.write(Theta_2);
      servo3.write(Theta_3);
      servo4.write(Theta_4 - 45.00);
      delayMicroseconds(100);
    }
    delay(100);
    gripper.write(90);
    delay(100);
    for(Y_End_Effector = 60.00; Y_End_Effector <= 150.00; Y_End_Effector += 0.10)
    {
      X_End_Effector = -150.00;
      Z_End_Effector = 250.00;
      Rumus_IK();
      servo1.write(Theta_1);
      servo2.write(Theta_2);
      servo3.write(Theta_3);
      servo4.write(Theta_4 - 45.00);
      delayMicroseconds(100);
    }
    delay(100);
  }
  else if (Distance <= 10.00 && mapped_blue_freq < 180 && mapped_red_freq < 155 && mapped_green_freq < 210)//Red object detected
  {
    gripper.write(90);
    for(Y_End_Effector = 150.00; Y_End_Effector >= 40.00; Y_End_Effector -= 0.10)
    {
      X_End_Effector = 180.00;
      Z_End_Effector = 10.00;
      Rumus_IK();
      servo1.write(Theta_1);
      servo2.write(Theta_2);
      servo3.write(Theta_3);
      servo4.write(Theta_4 - 45.00);
      delayMicroseconds(100);
    }
    delay(300);
    gripper.write(120);
    delay(300);
    for(Y_End_Effector = 40.00; Y_End_Effector <= 150.00; Y_End_Effector += 0.10)
    {
      X_End_Effector = 180.00;
      Z_End_Effector = 10.00;
      Rumus_IK();
      servo1.write(Theta_1);
      servo2.write(Theta_2);
      servo3.write(Theta_3);
      servo4.write(Theta_4 - 45.00);
      delayMicroseconds(100);
    }
    for(Z_End_Effector = 10.00; Z_End_Effector <= 100.00; Z_End_Effector += 0.10)
    {
      Y_End_Effector = 150.00;
      X_End_Effector = 180.00;
      Rumus_IK();
      servo1.write(Theta_1);
      servo2.write(Theta_2);
      servo3.write(Theta_3);
      servo4.write(Theta_4 - 45.00);
      delayMicroseconds(100);
    }
    for(X_End_Effector = 180.00; X_End_Effector >= -150.00; X_End_Effector -= 0.10)
    {
      Y_End_Effector = 150.00;
      Z_End_Effector = 100.00;
      Rumus_IK();
      servo1.write(Theta_1);
      servo2.write(Theta_2);
      servo3.write(Theta_3);
      servo4.write(Theta_4 - 45.00);
      delayMicroseconds(100);
    }
    for(Z_End_Effector = 100.00; Z_End_Effector <= 150.00; Z_End_Effector += 0.10)
    {
      Y_End_Effector = 150.00;
      X_End_Effector = -150.00;
      Rumus_IK();
      servo1.write(Theta_1);
      servo2.write(Theta_2);
      servo3.write(Theta_3);
      servo4.write(Theta_4 - 45.00);
      delayMicroseconds(100);
    }
    for(Y_End_Effector = 150.00; Y_End_Effector >= 50.00; Y_End_Effector -= 0.10)
    {
      X_End_Effector = -150.00;
      Z_End_Effector = 150.00;
      Rumus_IK();
      servo1.write(Theta_1);
      servo2.write(Theta_2);
      servo3.write(Theta_3);
      servo4.write(Theta_4 - 45.00);
      delayMicroseconds(100);
    }
    delay(100);
    gripper.write(90);
    delay(100);
    for(Y_End_Effector = 50.00; Y_End_Effector <= 150.00; Y_End_Effector += 0.10)
    {
      X_End_Effector = -150.00;
      Z_End_Effector = 150.00;
      Rumus_IK();
      servo1.write(Theta_1);
      servo2.write(Theta_2);
      servo3.write(Theta_3);
      servo4.write(Theta_4 - 45.00);
      delayMicroseconds(100);
    }
    delay(100);
  }
}
//object mass. Red = 31g, Blue = 32g, Green = 29g
//Blue object dimension = 3,1cm x 3,4cm x 5,1cm
//Green object dimension = 3,1cm x 3,4cm x 4,6cm
//red object dimension = 3,1cm x 3,4cm x 4,9cm
