#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <math.h>

#define EEPROM_SIZE 512 // Adjust this based on your EEPROM size

Adafruit_BMP280 bme;

const int buzzer = 3;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  pinMode(buzzer, OUTPUT);
  
  if (bme.begin(0x76)) {
    Serial.println("BME280 sensor found.");
  } else {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  float pressure = bme.readPressure() / 100.0F; // Convert pressure to hPa
    float altitude = 44330 * (1.0 - pow(pressure / 1013.25, 0.1903)); // Calculate altitude in meters
    int altitudeInt = static_cast<int>(altitude);
    Serial.print("Initial Altitude: ");
    Serial.print(altitudeInt);
    Serial.println(" meters");

  Serial.println("Enter 'w' to log altitude data or 'r' to read altitude data:");
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    
    if (command == 'w') {
      Serial.println("2 minutes to countdown")
      for (int i = 0; i <= 120; i++) {
        tone(buzzer, 300);
        delay(500);
        noTone(buzzer);
        delay(500); 
  }
      
      Serial.println("Countdown started");
      for (int i = 0; i <= 10; i++) {
        tone(buzzer, 1000);
        delay(1000);
        noTone(buzzer);
        delay(500); 
  }

        tone(buzzer, 2000);
        delay(500);
        tone(buzzer, 1000);

      
      writeAltitudeToEEPROM();
    } else if (command == 'r') {
      readAltitudeFromEEPROM();
    } else {
      Serial.println("Invalid command. Enter 'w' or 'r'.");
    }
  }
}

void writeAltitudeToEEPROM() {
  for (int i = 0; i < 60; i++) {
    float pressure = bme.readPressure() / 100.0F; // Convert pressure to hPa
    float altitude = 44330 * (1.0 - pow(pressure / 1013.25, 0.1903)); // Calculate altitude in meters

    // Store the data in EEPROM
    int address = i * sizeof(int);
    int altitudeInt = static_cast<int>(altitude);
    EEPROM.put(address, altitudeInt);
    delay(250);

    // Log the altitude data to serial monitor
    Serial.print("Address: ");
    Serial.print(address);
    Serial.print(", Altitude: ");
    Serial.print(altitudeInt);
    Serial.println(" meters");
    
  }
  Serial.println("Altitude data logged to EEPROM.");
}

void readAltitudeFromEEPROM() {
  for (int i = 0; i < 60; i++) {
    int address = i * sizeof(int);
    int altitudeInt;
    
    EEPROM.get(address, altitudeInt);
    
    Serial.print("Address: ");
    Serial.print(address);
    Serial.print(", Altitude: ");
    Serial.print(altitudeInt);
    Serial.println(" meters");
  }
}
