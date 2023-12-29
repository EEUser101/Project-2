  #include <LiquidCrystal.h>  
  #include <Adafruit_MPU6050.h>
  #include <Adafruit_Sensor.h>
  #include <Wire.h>

  Adafruit_MPU6050 mpu;

  int IN1 = 2;
  int IN2 = 10;
  int IN3 = 11;
  int IN4 = 12;
  int enABle = 3;
  int En = A3;
  int RIn= A2;
  int LIn = A1;
  bool beforeRamp = true;
  bool onRamp = false;

  //initialize the pins

  volatile int Pulses = 0;
  int time = millis()/1000;
  int Startclk = millis();
  const float circumference = 11.59;
  volatile boolean lastA;
  int Time = millis()/1000;  // this variable added is to calculate time after the car starts
  int distanceFlag = 1;

  // initialize the library with the pins on the Arduino board  
  LiquidCrystal lcd(8, 9, 4, 5, 6, 7); 
  void setup() {
    // put your setup code here, to run once:
    Wire.begin();
    Serial.begin(9600);
    pinMode(En, INPUT);
    pinMode(RIn, INPUT);
    pinMode(LIn, INPUT);

    pinMode(enABle, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
  
  lcd.begin(16, 2);
  lcd.print("Ramp angle: ");
  lcd.setCursor(0,1);
  lcd.print("Distance:");
  if(!mpu.begin()){
    while(1){
      delay(10);
    }
  }

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);


  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  void loop() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    int LIR = digitalRead(LIn);
    int RIR = digitalRead(RIn);

    float accZ = a.acceleration.z;
    float accY = a.acceleration.y;
    float accX = a.acceleration.x;
   
    float rampAngle = atan2(accY, sqrt(pow(accX,2)+pow(accZ,2)))*180.0/3.142;
    //float rampAngle = acos(accZ/9.2)*180.0/3.142;
    lcd.setCursor(7, 1);
    float distance = (circumference*( (float)Pulses/20.0));
    lcd.print(distance);

    if (beforeRamp){
      if (!onRamp){
        if (accY < 1.2 && accY > -3.0){
          lineFollow(LIR, RIR);
        } else if (accY >= 3.0){
          analogWrite(enABle, 220);
          delay(1300);
          onRamp = true;
        } else if (accY <= -3.0){
          forward();
          beforeRamp = false;
        }
      } else {
        stop(); 
        delay(4000);
        motorTurn();
        stop();
        delay(2000);
        onRamp = false;
      }
    } else {
      attachInterrupt(digitalPinToInterrupt(En), updateEncoder, CHANGE);

      if (distanceFlag ==1){
        if (distance < 90){
          lineFollow(LIR, RIR);
        } else{
          stop();
          delay(2000);

          distanceFlag = 0;
        }

      } else {

        lineFollow(LIR, RIR);
      }
    }



    //UpdateEncoder();
    //distanceMeasure();
}

void lineFollow(int LIR, int RIR){//forward function 
  updateEncoder();
  distanceMeasure();
  if((RIR == 0)&&(LIR == 0)){forward();}   
  if((RIR == 1)&&(LIR == 0)){turnRight();} //if Right Sensor is Black and Left Sensor is White then it will call turn Right function  
  if((RIR == 0)&&(LIR == 1)){turnLeft();} //if Right Sensor is White and Left Sensor is Black then it will call turn Left function
  if((RIR == 1)&&(LIR == 1)){stop();} 
}
  void forward(){
    analogWrite(enABle, 80);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }

  void turnRight(){
    analogWrite(enABle, 140);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    delay(140);
  }

  void turnLeft(){
    analogWrite(enABle, 140);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH); 
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    delay(140);
  }

  void stop(){
    digitalWrite(enABle, LOW);-
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  void updateEncoder(){
    boolean currentA = digitalRead(En);
    if (lastA != currentA){
      Pulses++;
      lastA = currentA;
    }
  }

  void distanceMeasure(){
    lcd.setCursor(0,0);
    lcd.print("Distance travel:");
    lcd.setCursor(7, 1);
    float distance = (circumference*( (float)Pulses/20.0));
    lcd.print(distance);
    lcd.print("cm");
  }

  void motorTurn(){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    delay(1500);
    
  }
