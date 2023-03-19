/*
 * Project Midterm2
 * Description:Midterm2
 * Author:Vernon Cox
 * Date:17Mar23
 */


#include "Adafruit_MQTT_SPARK.h"
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include "credentials.h"



 int soilentGreen=A5; //moistSensor readings
 int pumpState;
 int pumpread;
 const int pumpPIN=D11;
 
 


/*
Copy the Adafruit.io Setup line and the next four lines to a credentials.h file
//************************* Adafruit.io Setup *****************************************
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883        // use 1883 for SSL
#define AIO_USERNAME    "username"	// replace with your Adafruit.io username
#define AIO_KEY         "key"		    // replace with your Adafruit.io key
*/

/************ Global State (you don't need to change this!) ***   ***************/ 
TCPClient TheClient; 

 // Setup the MQTT client class by passing in the WiFi client and MQTT server and login details. 
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 

/****************************** Feeds ***************************************/ 
// Setup Feeds to publish or subscribe 
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname> 
Adafruit_MQTT_Subscribe subFeed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/suvabuttononoff"); 
Adafruit_MQTT_Publish pubFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/soilentGreen");

/************Declare Variables*************/
unsigned int last, lastTime;
float subValue,pubValue;

/************Declare Functions*************/
void MQTT_connect();
bool MQTT_ping();


SYSTEM_MODE(SEMI_AUTOMATIC);



 void setup() {

    Serial.begin(9600);
    waitFor(Serial.isConnected,10000);
    delay(1000);
    WiFi.clearCredentials();
    Serial.printf("Has Credentials = %i\n\n",WiFi.hasCredentials());
    delay(1000);
    WiFi.setCredentials("DDCIOT","ddcIOT2020");
    WiFi.setCredentials("ParentHouse","Suva9fiji");

  // Connect to Internet but not Particle Cloud
    WiFi.on();
    WiFi.connect();
    while(WiFi.connecting()) {
      Serial.printf(".");
      delay(100);
  }
  Serial.printf("\n\n");
  delay(3000);
  pinMode(soilentGreen, INPUT);
  pinMode(pumpPIN, OUTPUT);
  digitalWrite(pumpPIN,HIGH);
  delay(6000);
  digitalWrite(pumpPIN,LOW);
  delay(6000);
  digitalWrite(pumpPIN,HIGH);
  delay(6000);
  digitalWrite(pumpPIN,LOW);

  // Setup MQTT subscription
  mqtt.subscribe(&subFeed);//must tell Argon to subscribe

 }
 void loop () {
  soilentGreen=analogRead(soilentGreen);
  MQTT_connect();
  MQTT_ping();

//lines below for publishing
  if((millis()-lastTime > 6000)) {
    if(mqtt.Update()) {
      pubFeed.publish(soilentGreen);
    
      } 
      Serial.printf("Moisture reading is %i \n",soilentGreen); 
    lastTime = millis();
  }
 }
// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
  void MQTT_connect() {
    int8_t ret;
    // Return if already connected.
    if (mqtt.connected()) {
     return;
    }
  
  Serial.print("Connecting to MQTT... ");
 
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.printf("Error Code %s\n",mqtt.connectErrorString(ret));
       Serial.printf("Retrying MQTT connection in 5 seconds...\n");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds and try again
  }
  Serial.printf("MQTT Connected!\n");

  }
bool MQTT_ping(){
  static unsigned int last;
  bool pingStatus;

  //keep the connection alive

  if ((millis()-last)>120000) {
      Serial.printf("Pinging MQTT \n");
      pingStatus = mqtt.ping();
      if(!pingStatus) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
      last = millis();
  

  return pingStatus;

}
}