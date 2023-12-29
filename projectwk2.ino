#include <LiquidCrystal.h>  

int IN1 = A1;
int IN2 = 2;
int IN3 = 13;
int IN4 = 12;
int enA = 3;
int enB = 11;
int En = A4;
int RIn= A2;
int LIn = A3;

//initialize the pins

volatile int Pulses = 0;
int time = millis()/1000;
const float circumference = 11.59;
volatile boolean lastA;

// initialize the library with the pins on the Arduino board  
LiquidCrystal lcd(8, 9, 4, 5, 6, 7); 
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(En, INPUT);
  pinMode(RIn, INPUT);
  pinMode(LIn, INPUT);
  pinMode(enA, OUTPUT);
	pinMode(enB, OUTPUT);
  pinMode(IN1, OUTPUT);
	pinMode(IN2, OUTPUT);
	pinMode(IN3, OUTPUT);
	pinMode(IN4, OUTPUT);

  

  // put your setup code here, to run once:
  //Here, 16 and 2 are the columns and rows of the LCD  
  lcd.begin(16, 2);  
  // It prints the message on the LCD.   
  lcd.print("Distance travel: ");
  // We can modify the message as per our choice.  
}

void loop() {
  // put your main code here, to run repeatedly:
  UpdateEncoder();
  distanceMeasure();
  if((digitalRead(RIn) == 0)&&(digitalRead(LIn) == 0)){forward();}   //if Right Sensor and Left Sensor are at White color then it will call forword function
  if((digitalRead(RIn) == 1)&&(digitalRead(LIn) == 0)){turnRight();} //if Right Sensor is Black and Left Sensor is White then it will call turn Right function  
  if((digitalRead(RIn) == 0)&&(digitalRead(LIn) == 1)){turnLeft();} //if Right Sensor is White and Left Sensor is Black then it will call turn Left function
  if((digitalRead(RIn) == 1)&&(digitalRead(LIn) == 1)){stop();}
}
void forward(){
  analogWrite(enA, 80);
  analogWrite(enB,80);
  digitalWrite(IN1, HIGH);
	digitalWrite(IN2, LOW);
	digitalWrite(IN3, HIGH);
	digitalWrite(IN4, LOW);
}

void turnLeft(){
  analogWrite(enA, 140);
  analogWrite(enB,140);
  digitalWrite(IN1, HIGH);
	digitalWrite(IN2, LOW);
	digitalWrite(IN3, LOW);
	digitalWrite(IN4, HIGH);
  delay(140);
}

void turnRight(){
  analogWrite(enA, 140);
  analogWrite(enB,140);
  digitalWrite(IN1, LOW);
	digitalWrite(IN2, HIGH);
	digitalWrite(IN3, HIGH);
	digitalWrite(IN4, LOW);
  delay(140);
}

void stop(){
  digitalWrite(enA, LOW);
  digitalWrite(enB, LOW);
  digitalWrite(IN1, LOW);
	digitalWrite(IN2, LOW);
	digitalWrite(IN3, LOW);
	digitalWrite(IN4, LOW);
}

void UpdateEncoder(){
  boolean currentA = digitalRead(En);
  if (lastA != currentA){
    Pulses++;
    lastA = currentA;
  }
}

void distanceMeasure(){
  float distance = (circumference*( (float)Pulses/20.0));
  lcd.setCursor(7, 1);
  lcd.print(distance);
  lcd.print("cm");
}

