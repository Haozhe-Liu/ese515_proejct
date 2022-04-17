/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp8266-nodemcu-arduino-ide/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <ESP8266WiFi.h>
#include <espnow.h>
#include "config.h"

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
    //char a[32];
    int b;
    //float c;
    //String d;
    //bool e;
} struct_message;

// Create a struct_message called myData
struct_message myData;
unsigned long lastTime = 0;  
unsigned long timer_Delay = 5000;  // send readings timer
String string_test = "Motion mode:";
String standby = "STANDBY";
String forw = "Forward";
String back = "Backward";
String un = "Unknown";

// Adafruit Feeds
// Change "soil_moisture" to fit your application.
AdafruitIO_Feed *motion_mode = io.feed("motion_mode");

// Callback function that will be executed when data is received
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  //Serial.print("Char: ");
  //Serial.println(myData.a);
  Serial.print("Int: ");
  Serial.println(myData.b);
  //Serial.print("Float: ");
  //Serial.println(myData.c);
  //Serial.print("String: ");
  //Serial.println(myData.d);
  //Serial.print("Bool: ");
  //Serial.println(myData.e);
  Serial.println();
}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
int data = myData.b;

io.run();

if ((millis() - lastTime) > timer_Delay) {

  if(data == 3){
    digitalWrite(4, HIGH);
    digitalWrite(5, HIGH);
    motion_mode->save(string_test+standby);
    }
    
  else if(data == 2){
    digitalWrite(4, LOW);
    digitalWrite(5, HIGH);
    motion_mode->save(string_test+back);
    }
    
  else if(data == 1){
    digitalWrite(4, HIGH);
    digitalWrite(5, LOW);
    motion_mode->save(string_test+forw);
    }
    
  else if(data == 0){
    digitalWrite(4, LOW);
    digitalWrite(5, LOW);
    motion_mode->save(string_test+un);
    }
    

  lastTime = millis();
  delay(200);
}
  
}
 
