#include <Servo.h>

//color sensors
#define S0 29
#define S1 30
#define S2 31
#define S3 32
#define sensorOut 3

int frequency = 0;

bool bucketHasRaisedForRamp = false;

// Create Servo objects
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;

const int rightSensorPin = A0;   // Left distance sensor connected to analog pin A0
const int leftSensorPin = A1;  // Right distance sensor connected to analog pin A1
const int frontSensorPin = A2;  // Front distance sensor connected to analog pin A2
const int rearSensorPin = A3;

const float thresholdDistance = 21.5; // Threshold distance in cm
const float backUpThresholdDistance = 22.0;

// Define motor control pins
int RPWM_Output1 = 11; 
int LPWM_Output1 = 12;
int RPWM_Output2 = 13; // Arduino PWM output pin 11; connect to IBT-2 pin 1 (RPWM)
int LPWM_Output2 = 46;

void setup() {

  Serial.begin(9600);

  // Initialize motor control pins as outputs
  pinMode(RPWM_Output1, OUTPUT);
  pinMode(LPWM_Output1, OUTPUT);
  pinMode(RPWM_Output2, OUTPUT);
  pinMode(LPWM_Output2, OUTPUT);

  // color sensors
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);

  //color sensor frequency scaling to 20%
  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);

  // Attach servos to the corresponding pins
  servo1.attach(5);  // Connect the first servo to digital pin 2
  servo2.attach(6);  // Connect the second servo to digital pin 3
  servo3.attach(7);  // ... and so on
  servo4.attach(8);
  servo5.attach(9);
  servo6.attach(10);
}

void loop() {

  servo1.write(119);
  servo2.write(6);
  servo3.write(60);
  servo4.write(150);
  servo5.write(20);
  servo6.write(96);
  //detectGreenLed();
delay(1000);
  testServo();
  delay(1000);
  servo4.write(150);

  while(true) {
    float leftDistance = readDistance(leftSensorPin);
    float rightDistance = readDistance(rightSensorPin);
    float frontDistance = readDistance(frontSensorPin);
    float rearDistance = readDistance(rearSensorPin);
    

    // if the rear distance is = 24 then we are raising the bucket slightly for the ramp
    // we only want to do this once so we will set bucketraise to true after it happens
    // if (rearDistance > 20 && bucketHasRaisedForRamp == false) {
    //   stopMotors();
    //   delay(1000);
    //   servo4.write(125);
    //   delay(2000);
    //   for (int i = 130; i <= 150; i++)
    //   {
    //     servo4.write(i);
    //     delay(40);
    //     moveForward();
    //   }
    //   bucketHasRaisedForRamp = true;
    // }

    if (frontDistance < thresholdDistance && leftDistance > 25) {
    // Stop
      stopMotors();
      delay (1000);

      while (frontDistance < 28){ // back up until sensor reads less than 28 cm 
        backup();
        frontDistance = readDistance(frontSensorPin);
      }
    stopMotors();
    delay(400);
    turnLeft();
    delay(1000);
    while (frontDistance < backUpThresholdDistance){
      moveForward();
      frontDistance = readDistance(frontSensorPin);
    }
    stopMotors();
  } 
  else if (leftDistance < thresholdDistance) {
    // Veer right
    veerRight();
  } 
  else if (rightDistance < thresholdDistance) {
    // Veer left
    veerLeft();
  } 
  else {
    // Move forward
    moveForward();
  }
  delay(100); // Adjust delay as needed
}

  }




void testServo() {
  servo1.write(119);
  servo2.write(6);
  servo3.write(60);
  servo4.write(150);
  servo6.write(91);
  delay(50);

  servo3.write(100);
  delay(500);
  for (int i = 10; i <= 153; i++) {
    servo2.write(i);
    delay(20);
  }
  servo4.write(135);

}

void MoveToFront() {
  servo1.write(119);
  servo3.write(100);
  delay(1000);
  servo2.write(150);
  servo4.write(140);
  servo6.write(0);
}



float readDistance(int pin) {
  int sensorValue = analogRead(pin);
  float voltage = sensorValue * (5.0 / 1023.0);
  float distance = 27.86 * pow(voltage, -1.15); // Convert voltage to distance
  return distance;
}

void backup() {
  analogWrite(RPWM_Output1, 150); // Full speed forward
  analogWrite(LPWM_Output1, 0);
  analogWrite(RPWM_Output2, 0); // Full speed forward
  analogWrite(LPWM_Output2, 140);
}
void moveForward() {
  analogWrite(RPWM_Output1, 0); // Full speed forward
  analogWrite(LPWM_Output1, 155);
  analogWrite(RPWM_Output2, 170); // Full speed forward
  analogWrite(LPWM_Output2, 0);
}

void stopMotors() {
  analogWrite(RPWM_Output1, 0); // Stop
  analogWrite(LPWM_Output1, 0);
  analogWrite(RPWM_Output2, 0); // Stop
  analogWrite(LPWM_Output2, 0);
}

void veerRight() {
  analogWrite(RPWM_Output1, 0); // Right motor forward
  analogWrite(LPWM_Output1, 150);
  analogWrite(RPWM_Output2, 120); // Left motor stop or reverse if needed
  analogWrite(LPWM_Output2, 0);
}

void veerLeft() {
  analogWrite(RPWM_Output1, 0); // Right motor stop or reverse if needed
  analogWrite(LPWM_Output1, 125);
  analogWrite(RPWM_Output2, 150); // Left motor forward
  analogWrite(LPWM_Output2, 0);
}

void turnLeft() {
  analogWrite(RPWM_Output1, 140); // Right motor stop or reverse if needed
  analogWrite(LPWM_Output1, 0);
  analogWrite(RPWM_Output2, 150); // Left motor forward
  analogWrite(LPWM_Output2, 0);
}
void detectGreenLed() {
  bool canStart = false;
  while(!canStart){
    // set scaling to 20% in startup (may have to tweak this code at comp)
    // Setting Green filtered photodiodes to be read
    digitalWrite(S2,HIGH);
    digitalWrite(S3,HIGH);
    // Reading the output frequency
    frequency = pulseIn(sensorOut, LOW);
      Serial.print(frequency);
      Serial.println("");
    delay(100);
    if (frequency < 100) {
      canStart = true;
      
    }
  }

}