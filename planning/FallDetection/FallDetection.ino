#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085.h>
#include "Wire.h"
#include "I2Cdev.h"
#include <Adafruit_ADXL345_U.h>
#include <Servo.h>

Servo myservo;
int pos = 0;

//settings//
int n = 8; // number of samples to average
float seaLevelPressure = 100600; // pressure at sea level in Pa
float threshold = 0.4; // threshold for altitude change in meters
float threshold_is_moving = 0.3; // threshold for acceleration change in m/s^2
int num_readings = 10; // number of readings to average for isSensorMoving
//-----//
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified();
float altitude1 = 0;
float altitude2 = 0;
Adafruit_BMP085 bmp;
float altitude;
int32_t lastMicros;
#define LED_PIN 13 
bool blinkState = false;
volatile bool threshold_exceeded = false;

void setup() {
  myservo.attach(3);
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
    attachInterrupt(digitalPinToInterrupt(2), isr_deploy_parachute, RISING);
    Wire.begin();
    Serial.begin(115200);
    Serial.println("Initializing I2C devices...");
    Serial.println("Testing device connections...");
    Serial.println("Initialize ADXL345");

    if(!accel.begin())
   {
      Serial.println("No valid sensor found");
      while(1);
   }
    if(!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  Serial.println("Initialization done");
  delay(1000);
}


void isr_deploy_parachute() {
  if (threshold_exceeded) {
    for (pos = 180; pos >= 0; pos -= 20) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(14);    
                      // waits 15 ms for the servo to reach the position
  }
    Serial.println("Parachute deployed");
    delay(10);
    threshold_exceeded = false;
    exit(0); 
  }
}


bool isSensorMoving(Adafruit_ADXL345_Unified& accel, int num_readings, float threshold) {
  float readings[num_readings][3];
  float sum_diff = 0;
  float last_reading[3];

  // Read and store initial readings
  for (int i = 0; i < num_readings; i++) {
    sensors_event_t event;
    accel.getEvent(&event);
    readings[i][0] = event.acceleration.x;
    readings[i][1] = event.acceleration.y;
    readings[i][2] = event.acceleration.z;
    delay(10);
  }

  // Compute the average of the initial readings
  float avg[3] = {0, 0, 0};
  for (int i = 0; i < num_readings; i++) {
    avg[0] += readings[i][0];
    avg[1] += readings[i][1];
    avg[2] += readings[i][2];
  }
  avg[0] /= num_readings;
  avg[1] /= num_readings;
  avg[2] /= num_readings;

  // Check if the sensor is moving by comparing the current reading to the average of the initial readings
  for (int i = 0; i < num_readings; i++) {
    sensors_event_t event;
    accel.getEvent(&event);
    float diff[3] = {event.acceleration.x - avg[0], event.acceleration.y - avg[1], event.acceleration.z - avg[2]};
    float diff_norm = sqrt(pow(diff[0], 2) + pow(diff[1], 2) + pow(diff[2], 2));

    // Compare the difference to the threshold
    if (diff_norm > threshold) {
      return true;  // Sensor is moving
    }

    // Compute the sum of the differences
    if (i > 0) {
      sum_diff += sqrt(pow(last_reading[0] - event.acceleration.x, 2) + pow(last_reading[1] - event.acceleration.y, 2) + pow(last_reading[2] - event.acceleration.z, 2));
    }
    last_reading[0] = event.acceleration.x;
    last_reading[1] = event.acceleration.y;
    last_reading[2] = event.acceleration.z;
    delay(10);
  }

  // Check if the sensor is still based on the sum of the differences
  float avg_diff = sum_diff / (num_readings - 1);
  if (avg_diff < threshold) {
    return false;  // Sensor is still
  } else {
    return true;  // Sensor is moving
  }
}


float get_altitude() {
  float altitude = 0;
float measurements[100];

for (int i = 0; i < n; i++) {
  measurements[i] = bmp.readAltitude(seaLevelPressure);
  delay(13);
}

float sum_altitude = 0;
for (int i = 0; i < n; i++) {
  sum_altitude += measurements[i];
}
float avg_altitude = sum_altitude / n;

for (int i = 0; i < n - 1; i++) {
  for (int j = i + 1; j < n; j++) {
    if (abs(measurements[i] - avg_altitude) < abs(measurements[j] - avg_altitude)) {
      float temp = measurements[i];
      measurements[i] = measurements[j];
      measurements[j] = temp;
    }
  }
}

int num_measurements = n - (n / 3);
float remaining_altitude = 0;
for (int i = 0; i < num_measurements; i++) {
  remaining_altitude += measurements[i];
}
remaining_altitude /= num_measurements;

return remaining_altitude;

}

void loop() {
  
    bool moving = isSensorMoving(accel, num_readings, threshold_is_moving);
    if (moving) {
      altitude1 = get_altitude();
      delay(2);
      altitude2 = get_altitude();
      if (altitude1 - altitude2 > threshold || altitude1 - altitude2 < -threshold) {
        if (altitude1 - altitude2 > threshold){
          Serial.println("Altitude is going down");
          threshold_exceeded = true;
          isr_deploy_parachute();
          }
        else {
          Serial.println("Altitude is going up");
          }
      }
    else {
      Serial.println("Altitude is not changing");
    }
}
  else {
    Serial.println("Not moving");
  }
    
}