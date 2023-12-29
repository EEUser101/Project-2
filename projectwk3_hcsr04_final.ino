#include "NewPing.h" //this library is very easily used to measure distance using ultrasonic sensor
#include <LiquidCrystal.h>

const int IN2 = A4;
const int IN1 = 11;
const int IN3 = A5;
const int IN4 = 12;
const int enABle = 3;
const int TRIG = 13;
const int ECHO = 2;

//the maximum obstacle distance in centimeters 
const int MAX_DISTANCE = 400;

//initialise the lcd pins
LiquidCrystal lcd(8, 9, 4, 5, 6, 7); 

// NewPing setup of pins and maximum distance. 
NewPing sonar(TRIG, ECHO, MAX_DISTANCE);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //pinMode(En, INPUT);
  //pinMode(RIn, INPUT);
  //pinMode(LIn, INPUT);
  pinMode(enABle, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  lcd.begin(16,2);
  lcd.print("Presence of obstacle:");
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println(sonar.ping_cm());
  //delay(1000);

  if (sonar.ping_cm() >= 10){
    analogWrite(enABle, 100);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    lcd.setCursor(7,1);
    lcd.print("No.");
  } 
  if (sonar.ping_cm() < 10){
    analogWrite(enABle, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    lcd.setCursor(7,1);
    lcd.print("Yes");
  }
}
