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


// REPLACE WITH RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xBC, 0xFF, 0x4D, 0x2B, 0x1E, 0x14};



// Variables
int x = 0;


// Adafruit Feeds
// Change "soil_moisture" to fit your application.
AdafruitIO_Feed *emg_data_reading = io.feed("emg_data_reading");


void sendSensor()
{

  x = analogRead(A0); //input from photoresistor
  emg_data_reading->save(x);
  delay(200);
}


// Structure example to send data
// Must match the receiver structure
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
unsigned long timerDelay = 5000;  // send readings timer

// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0){
    Serial.println("Delivery success");
  }
  else{
    Serial.println("Delivery fail");
  }
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // connect to io.adafruit.com
  io.connect();

  // wait for a connection
  while(io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  // we are connected
  Serial.println();
  Serial.println(io.statusText());

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
}
 
void loop() {


  
int pin_D1 = digitalRead(5); //ESP8266 D1
int pin_D2 = digitalRead(4); //ESP8266 D2
int data;
Serial.print(pin_D1);
Serial.print(pin_D2);

if(pin_D1 == 1 && pin_D2 == 1) data = 3;
else if(pin_D1 == 1 && pin_D2 == 0) data = 2;
else if(pin_D1 == 0 && pin_D2 == 1) data = 1;
else data = 0;
  
  if ((millis() - lastTime) > timerDelay) {

    io.run();
    sendSensor();
    
    // Set values to send
//    strcpy(myData.a, "THIS IS A CHAR");
    myData.b = data;
//    myData.c = 1.2;
//    myData.d = "Hello";
//    myData.e = false;

    // Send message via ESP-NOW
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

    lastTime = millis();
  }

  
}
