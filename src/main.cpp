#include<SPI.h> 
#include<SD.h>
#include "I2Cdev.h"
#include <Adafruit_I2CDevice.h>
#include <Adafruit_BMP085.h>
#include <Arduino.h>
#include "Wire.h" 
#include "Adafruit_Sensor.h" 

Adafruit_BMP085 bmp;

#define DOLZINA_TESTA_DELA 3
char* test_list_working[DOLZINA_TESTA_DELA] = {};

#define DOLZINA_TESTA_NE_DELA 3
char* test_list_notworking[DOLZINA_TESTA_NE_DELA] = {};


#define DOLZINA_UKAZA 30
#define cam 8
char poziv[20] = "ARDUINO> ";
char poziv2[20] = "akjsdk";
char ukaz[DOLZINA_UKAZA] = "";
#define VELIKOST_THP 100
char tmp[VELIKOST_THP];
float pressure;
int chipSelect = 4; //chipSelect pin for the SD card Reader
File mySensorData; //Data object you will write your sesnor data to
char input;
int rel_visina;
int abs_visina;
void dump(unsigned, unsigned);

#define STEVILO_UKAZOV 7
const char* ukazi[STEVILO_UKAZOV] = {"test", "CAM", "AT", "dump", "print", "clear", "help"};

const char* pomoc[STEVILO_UKAZOV]={
  "Pretestira vse komponente in vrne njihovo stanje",
  "začne snemanje kamere",
  "Odgovori z OK", 
  "Izpiše 1024 bajtov pomnilnika od 0x100", 
  "Izpiše argument na zaslon", 
  "Pobriše zaslon",
  "Izpiše pomoc (poskusite help)"
};

//create a function that tracks output od bmp085 and prints it to the serial monitor
void printValues(){
  //make var with alltitude data from bmp085
  float alt = bmp.readAltitude();
  //make while loop that runs until values of alt are getting bigger
  int main() {

  while (alt <= 0) { // while alt is not getting bigger...
    std::cout << "Altitude: " << alt << std::endl;
    alt++; // increase alt by 1
  }

  std::cout << "Altitude is now bigger than 0." << std::endl;

  return 0;
}

}


void test(){

  if(Serial.available()){
  //remove all elements from test_list_working 
  for(int i = 0; i < DOLZINA_TESTA_DELA; i++){
    test_list_working[i] = "";
  }
  //remove all elements from test_list_notworking
  for(int i = 0; i < DOLZINA_TESTA_NE_DELA; i++){
    test_list_notworking[i] = "";
  }
  }

  //testira Serial
  if(Serial){
  test_list_working[0] = "Serial";
    }
  else{ 
  test_list_notworking[0] = "Serial";
  }
    if (!SD.begin(4)) {
      test_list_notworking[1] = "SD card";
      }
    else{
    test_list_working[1] = "SD card";

      }
  if (!bmp.begin()) {
  test_list_notworking[2] = "BMP085";
  }
  else{
    test_list_working[2] = "BMP085";
    }

  //delet blank elemnets from test_list_working
  for(int i = 0; i < DOLZINA_TESTA_DELA; i++){
    if(test_list_working[i] == ""){
      for(int j = i; j < DOLZINA_TESTA_DELA - 1; j++){
        test_list_working[j] = test_list_working[j+1];
      }
      test_list_working[DOLZINA_TESTA_DELA - 1] = "";
    }
  }
  //delet blank elemnets from test_list_notworking
  for(int i = 0; i < DOLZINA_TESTA_NE_DELA; i++){
    if(test_list_notworking[i] == ""){
      for(int j = i; j < DOLZINA_TESTA_NE_DELA - 1; j++){
        test_list_notworking[j] = test_list_notworking[j+1];
      }
      test_list_notworking[DOLZINA_TESTA_NE_DELA - 1] = "";
    }
  }
  //pokaže katere stvari delajo in katere ne
  
  Serial.println("Testi delovanja:");
  Serial.println("Delujejo:");
  for(int i = 0; i < DOLZINA_TESTA_DELA; i++){
  Serial.println(test_list_working[i]);
  }
  
  Serial.println("Ne delujejo:");
  for(int i = 0; i < DOLZINA_TESTA_NE_DELA; i++){
  Serial.println(test_list_notworking[i]);
  }


}

void dump(unsigned start = 0, unsigned block = 512){
  // Naslovi zgoraj
  for(int i = 0; i < 7; i++) Serial.print(' ');

  for(unsigned i = 0; i < 16; i++){ // naslovi (00-0F)
    snprintf(tmp, 30, "%02X ", i);
    Serial.print(tmp);

    if(i == 7){
      Serial.print(" ");
    }
  }
  Serial.print(" ASCII\n");

  // Okvirček zgoraj
  for(int i = 0; i < 5; i++) Serial.print(' ');
  Serial.print("┌");
  for (int i = 0; i < 67; i++) Serial.print("─");
  Serial.print("\n");

  for(unsigned i = start; i < start + block; i+=16){
    snprintf(tmp, 30, "%04X │ ", i); // naslov pristrani
    Serial.print(tmp);
    for(unsigned j = i; j < i+16; j++){
      byte* a = (byte*)(j);
      snprintf(tmp, 30, "%02X ", *a);
      Serial.print(tmp);

      if(j%8 == 7){ // Dvojni presledek na sredini
        Serial.print(" ");
      }
    }
    for(unsigned j = i; j < i+16; j++){
      char* b = (char*)(j);
      if(*b < 32){
        Serial.print('.');
      }
      else {
        Serial.print(*b);
      }
    }
    Serial.println();
  }
}

void beriUkaz() {
  char znak;
  unsigned pozicija = 0;

  while (1) {
    while (Serial.available() == 0); // čaka, ko podatkov ni

    znak = Serial.read();

    if (znak == 0x0D) { // CR
      continue;
    } 
    else if (znak == 0x0A) { // LF
      ukaz[pozicija] = '\0'; // 0, 0x00
      break;
    } 
    else if (znak == 0x08) {       // BS // \b
      if (pozicija > 0) {        // če ima niz kater znak
        Serial.print("\b \b");
        pozicija--;
      }
    }
    else {
      if(pozicija < DOLZINA_UKAZA - 1){
        Serial.write(znak);
        ukaz[pozicija] = znak;
        pozicija++;
      }
    }
  }
  Serial.println();
}

char* lociUkaz(char* niz){
  unsigned int i;
  for(i = 0; i < strlen(niz); i++){
    if(niz[i] == ' '){
      niz[i] = 0x00;
      return &niz[i+1];
    }
  }
  return &niz[i];
}

int pretvoriVStevilo(char* niz){
  int a = 0;
  int i = 0;
  while(*niz != 0x00){
    a *= 10;
    a += *niz - '0';
    if(a < i){
      Serial.println("Pri pretvorbi je prišlo do napake");
      return i;
    }
    i = a;
    niz++;
  }
  return a;
}

void setup() {

  bmp.begin();
  pinMode(10, OUTPUT);
  pinMode(8, OUTPUT);
  Serial.begin(115200);
  
}

void loop() {
  printValues();
  delay(50);

  mySensorData = SD.open("index.txt", FILE_WRITE);
         if (mySensorData) {
          delay(20); //Pause between readings.         
          mySensorData.println(pressure);                        //write pressure and end the line (println)
          mySensorData.close();                                  //close the file
       }

  Serial.print(poziv);
  beriUkaz();
  //dump(0x100);
  char* argument = lociUkaz(ukaz);
  //dump(0x100);

  //Serial.println((int)ukazi);
  int i = 0;
  while(i < STEVILO_UKAZOV){
    if(strcmp(ukaz, ukazi[i]) == 0){
      break;
    }
    i++;
  }

  // v i-ju je zaporedna številka ukaza
  //Serial.println(i);

  switch(i){
  case 0: // prizgi
    test();
    break;
  case 1: //vklopi kamero
    digitalWrite(cam, HIGH);
    delay(1500); 
    digitalWrite(cam, LOW);  
    delay(10000); //snema 10s
    digitalWrite(cam, HIGH);
    delay(1500);
    break;
  case 2: // AT
    Serial.println("OK");
    break;
  case 3:
    dump(0x100, 0x400);
    break;
  case 4:
    break;
  case 5:
    break;
  case 6:
    for(i=0; i<STEVILO_UKAZOV; i++){
      snprintf(tmp, VELIKOST_THP, "%d %6s - %s\n", i, ukazi[i], pomoc[i]);
      Serial.print(tmp);
    }
    break;
  default:
    Serial.println("Neznan ukaz :");
    break;
  }
}