#include <SoftwareSerial.h>
#include <ArduinoJson.h>

SoftwareSerial nodemcu(10, 11);//10rx 11tx

const int trigger = 6;
const int echo = 7;
const int motorPin = 8;
const int proximityPin = 2;
const float annoFactor = 2.5;
const float distancePerRevolution = 0.07225663103;
const int pulsesPerRevolution = 3;
unsigned long startTime;
unsigned long endTime;
volatile unsigned int pulseCount = 0;
float speedOfObject;
float distance_mm;
int motor_flag = 0;
float speedSum = 0;
int speedCount = 0;
bool motorActivated = false;

void setup() {
  Serial.begin(9600);
  nodemcu.begin(9600);  // Initiate the communication with NodeMCU
  pinMode(trigger, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(motorPin, OUTPUT);
  pinMode(proximityPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(proximityPin), countPulse, RISING);
}

void loop() {
  StaticJsonDocument<200> doc;

  float total_distance = 0;
  int num_readings = 50;

  startTime = millis();
  delay(1000);

  if (!motorActivated) {
    detachInterrupt(digitalPinToInterrupt(proximityPin));
  }

  endTime = millis();

  for (int i = 0; i < num_readings; i++) {
    digitalWrite(trigger, LOW);
    delayMicroseconds(5);
    digitalWrite(trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger, LOW);
    total_distance += pulseIn(echo, HIGH) * 0.1657;
    delay(100);
  }

  distance_mm = total_distance / num_readings;

  unsigned long elapsedTime = endTime - startTime;
  float distance = (distancePerRevolution * pulseCount) / pulsesPerRevolution;
  speedOfObject = distance / (elapsedTime / 1000.0);

  if (!isnan(speedOfObject)) {
    speedSum += speedOfObject;
    speedCount++;

    if (speedCount >= 1) {
      float averageSpeed = annoFactor * speedSum / speedCount;

      if (distance_mm <= 25) {
        motor_flag = 1;
      } else if (distance_mm > 50) {
        motor_flag = 2;
      } else {
        motor_flag = 0;
      }

      if (motor_flag == 1) {
        digitalWrite(motorPin, HIGH);
        motorActivated = true;
      } else if (motor_flag == 2) {
        digitalWrite(motorPin, LOW);
        motorActivated = false;
      }
      Serial.print("average_speed: ");
      Serial.println(averageSpeed);
      Serial.print("distance_mm: ");
      Serial.println(distance_mm);


      doc["average_speed"] = averageSpeed;  // Add average speed to JSON object
      doc["average_distance"] = distance_mm;  // Add average distance to JSON object

      // Send data to NodeMCU
      serializeJson(doc, nodemcu);
      delay(500);


      speedSum = 0;
      speedCount = 0;
    }
  }

  pulseCount = 0;

  if (!motorActivated) {
    attachInterrupt(digitalPinToInterrupt(proximityPin), countPulse, RISING);
  }
}

void countPulse() {
  pulseCount++;
}
