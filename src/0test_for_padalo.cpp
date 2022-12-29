#include "Wire.h"
#include "Adafruit_BMP085.h"

int altitude;
int initial_altitude;

int a=0;
int b=0;

Adafruit_BMP085 mySensor;
  
void setup(){
Serial.begin(9600); //turn on serial monitor
mySensor.begin();   //initialize altitude sensor mySensor
initial_altitude=mySensor.readAltitude();
Serial.print("Initial Altitude: ");
Serial.print(initial_altitude);

}

void test() {

    for (int i=0; i<5; i++) {
        altitude=mySensor.readAltitude();
        a=a+altitude;
        delay(10);
    }
    a=a/5;
    delay(800);
    for (int i=0; i<5; i++) {
        altitude=mySensor.readAltitude();
        b=b+altitude;
        delay(10);
    }

    b=b/5;


    for (int i = b-1; i < b+1; i++) {

    if (i > a+1 or i==a or i==a+1) {
        Serial.println("Going UP");
        delay(10);
    }

    else if (i < a-1) {
        Serial.println("Going DOWN");
        delay(10);
    }
    delay(50);

}




void loop() {

    //check if altitude is changing (going up or down)
    test();

    
    //check if altitude is lower than initial altitude + x meters
    if (altitude < initial_altitude+2) {
    Serial.println("Sprožam padalo");
    delay(10);
    }


}


