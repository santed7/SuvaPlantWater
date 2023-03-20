/*
 * ProjectRapidIoTandPrototypingMidterm#2
 * Description: Midterm#2v7
 * Author: Vernon Cox
 * Date: 20-MAR-2023
 */


#include <math.h>

#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"

#include "credentials.h"

#include "Adafruit_SSD1306.h"
#include "Adafruit_BME280.h"
#include "Grove_Air_Quality_Sensor.h"

int soilentGreenpin=A5; //moistSensor pin
int soilentReadgreen;//moistSensor readings


int pumpState;//is pump onOff
int pumpread;
const int pumpPIN=D11;//pump pin

int airSensorpin=A3; //airQualUnit pin
int airQuality;//air Quality reading


const int dustReadpin = 8;
int dustReadg;

unsigned long duration;
unsigned long starttime;
unsigned long sampletime_ms = 30000;//sampe 30s ;
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
float concentration = 0;

float tempC;
float tempF = 0;
float pressPA;
float humidRH;


Adafruit_BME280 bme;
bool status;
AirQualitySensor airsensor(A0);


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
//Adafruit_MQTT_Subscribe subFeed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/suvabuttononoff");
Adafruit_MQTT_Subscribe waterPump = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/pumpPIN");

//Adafruit_MQTT_Publish pubFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/suvaRand");
//Adafruit_MQTT_Publish pubFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/suvaButtonOnOff");

Adafruit_MQTT_Publish soilMoisture = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/soilentReadgreen");
Adafruit_MQTT_Publish dust = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/concentration");
Adafruit_MQTT_Publish air = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/airQuality");
/************Declare Variables*************/
unsigned int last, lastTime;
float subValue,pubValue;

/************Declare Functions*************/
void MQTT_connect();
bool MQTT_ping();

SYSTEM_MODE(SEMI_AUTOMATIC);

void setup() {
  pinMode(soilentGreenpin, INPUT);
  pinMode(pumpPIN, OUTPUT);
  pinMode(dustReadpin,INPUT);
  starttime = millis();//get the current time;
  pinMode(D7,OUTPUT);
  Serial.begin(9600);
  waitFor(Serial.isConnected,10000);

  // Connect to Internet but not Particle Cloud
  WiFi.on();
  WiFi.connect();
  while(WiFi.connecting()) {
    Serial.printf(".");
    delay(100);
  }
  Serial.printf("\n\n");

  // Setup MQTT subscription
  //mqtt.subscribe(&subFeed);//must tell Argon to subscribe
}

void loop() {
  MQTT_connect();
  MQTT_ping();
  soilentReadgreen=analogRead(soilentGreenpin);
  dustReadg=analogRead(dustReadpin);
  airQuality=analogRead(airSensorpin);
  
  
  

  // this is our 'wait for incoming subscription packets' busy subloop 
  Adafruit_MQTT_Subscribe *subscription;
  // while ((subscription = mqtt.readSubscription(100))) {
  //  if (subscription == &subFeed) {
  //     suvaButtonOnOff = atoi((char *)subFeed.lastread);//ASCII to float
  //     Serial.printf("Button is Pushed %i \n",suvaButtonOnOff); 
  //     if(suvaButtonOnOff){
  //      digitalWrite(D7,HIGH);//turns on the (small) D7 light on the Argon when Adafruit button is pushed
  //    }
  //      else{
  //        digitalWrite(D7,LOW);//turns off the (small) D7 light on the Argon when Adafruit button is pushed
  //      }
  //  }
  
    
  // }
 //lines below for publishing
  if((millis()-lastTime > 9000)) {
    if(soilentReadgreen>=200){
    if(mqtt.Update()) {
      soilMoisture.publish(soilentReadgreen);
      air.publish(airQuality);
      dust.publish(concentration);
      //pumpread.publish(pumpPIN);
      Serial.printf("Moisture reading is %i \n",soilentReadgreen);
        if(soilentReadgreen>2000) {
        Serial.printf("Plantsoil is too dry at %i \n",soilentReadgreen);
         digitalWrite(pumpPIN,HIGH);
         Serial.printf("Plant is getting H20 at %i \n",soilentReadgreen);
         delay(350);
         digitalWrite(pumpPIN,LOW);
       }
          if(soilentReadgreen<=1500) {
          Serial.printf("Plantsoil is too wet at %i \n",soilentReadgreen);
          }
        Serial.printf("Air Quality is %i \n",airQuality);
    lastTime = millis(); //gets the current time
    }
      else{

      }
  }
  }

    duration = pulseIn(dustReadpin, LOW);
    lowpulseoccupancy = lowpulseoccupancy+duration;

    if ((millis()-starttime) > sampletime_ms)//if the sampel time == 30s
    {
        ratio = lowpulseoccupancy/(sampletime_ms*10.0);  // Integer percentage 0=>100
        concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; // using spec sheet curve
       if(lowpulseoccupancy>0){
        Serial.printf("LowPO is %i ,",lowpulseoccupancy);
        Serial.printf("Ratio is %f ,",ratio);
        Serial.printf("Concentration is %f \n",concentration);
      }

        lowpulseoccupancy = 0;
        starttime = millis();
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

bool MQTT_ping() {
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
  }
  return pingStatus;
}
