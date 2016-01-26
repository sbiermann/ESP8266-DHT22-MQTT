/*************************************************/
/* Includes                                      */
/*************************************************/
#include <stdio.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include "Client.h"

#include <DHT.h>
#include <MQTT.h>
#include <PubSubClient.h>

#include <Wire.h>

// Include API-Headers
extern "C" {
#include "ets_sys.h"
#include "os_type.h"
#include "osapi.h"
#include "mem.h"
#include "user_interface.h"
#include "cont.h"
}

/*************************************************/
/* Debugging                                     */
/*************************************************/
const bool debugOutput = true;  // set to true for serial OUTPUT

/*************************************************/
/* Definitions                                   */
/*************************************************/
// in seconds, time between measurements, sketch goes into deep sleep between readings
// should be at least 30secs, to thingspeak only allows update every 20s and script excekution is about 20s
#define REPEAT_INTERVAL 360

#define SERIAL_SPEED 115200

#define DHT22_PIN 12       // Pin 12 as 1-wire bus for DHT22

/*************************************************/
/* Settings for WLAN                             */
/*************************************************/
const char* ssid = "WLANFR";
const char* password = "supergeheimespasswort";

/*************************************************/
/* Static IP                                     */
/*************************************************/
IPAddress ip(192,168,1,210);
IPAddress gateway(192,168,1,254);
IPAddress subnet(255,255,255,0);

/*************************************************/
/* Settings for MQTT                             */
/*************************************************/
char mqttBroker[] = "192.168.1.98";
int  mqttPort = 1883;

#define MQTTDeviceID "01"
#define MQTTClientID "ESP8266." MQTTDeviceID
#define MQTT_USER ""
#define MQTT_PASSWORD ""

#define TOPIC_ClientID "ESP8266/" MQTTDeviceID
#define TOPIC_LastWill  TOPIC_ClientID "/connected"
#define TOPIC_HumidityState  TOPIC_ClientID "/humidity/value"
#define TOPIC_TemperatureState  TOPIC_ClientID "/temperature/value"

// Create an WiFiClient object, here called "ethClient":
WiFiClient ethClient;

//Create MQTT client object
PubSubClient client(ethClient, mqttBroker, mqttPort);

// Create an DHT object, here called "dht":
DHT dht;
float temperature;
float humidity;
char charBuffer[32];

int wifiConnectCounter = 0;

// Pin14: VCC for Sensors
const int GPIO_Power_Sensors = 14;


// ------------------------
void setup() {
   Serial.begin(115200);   

   if (debugOutput) Serial.println("ESP8266 starts...");

   // power on sensors
   pinMode(GPIO_Power_Sensors, OUTPUT);
   digitalWrite(GPIO_Power_Sensors, HIGH);
   //disable internal LED
   pinMode(2, OUTPUT); 
   digitalWrite(2, HIGH );
   long start = millis();
    
   if (debugOutput) Serial.println("Wifi wait for connection");

   system_deep_sleep_set_option(1); // alt. 2 to not recalibrate WiFi after wake up, 1 is more stable needs more power
   wifi_set_sleep_type (LIGHT_SLEEP_T);

   if (debugOutput) Serial.setDebugOutput(true); // enable WIFI Debug output
   if (debugOutput) Serial.println();
   if (debugOutput) Serial.println();
   if (debugOutput) Serial.print("Connecting to ");
   if (debugOutput) Serial.println(ssid);

   // Connect to WiFi network
   WiFi.mode(WIFI_STA);
   yield();

   WiFi.disconnect();
   yield();

   WiFi.begin(ssid, password);
   WiFi.config(ip, gateway, subnet);

   // Wait for connection
   while (WiFi.status() != WL_CONNECTED) {
     delay(500);
     wifiConnectCounter++;
     if (debugOutput)  Serial.print(".");
     if (wifiConnectCounter > 100) {
       if (debugOutput)  Serial.println("No WiFi connection, try again in " + String(REPEAT_INTERVAL) + " seconds");
       system_deep_sleep(REPEAT_INTERVAL * 1000 * 1000); // after deep sleep automtic reset
       delay(200); // give some time to activate sleep state
     }
   }
   if (debugOutput) Serial.println("");
   if (debugOutput) Serial.println("WiFi connected");
   if (debugOutput) Serial.println("IP address: ");
   if (debugOutput) Serial.println(WiFi.localIP());
   wifi_set_sleep_type (LIGHT_SLEEP_T);

   if (debugOutput) Serial.println("Wifi Started");
   client.disconnect();
   //wait at least 3 seconds...
   long wait = millis()-start;
   if (debugOutput) Serial.println("WiFi connection time "+String(wait)+" ms");
   if( wait < 3000)
    delay(3000-wait);
   //starting the sensor 
   startSensors();
   // connect to MQTT Broker
   while (!client.connected()) {
   if (client.connect((char *)TOPIC_ClientID,(char *)TOPIC_LastWill,1,1,(char *)"0"))  
    {
      if (debugOutput) Serial.println("MQTT is connected");
      if(processDHT()) //read DHT22
      {
        yield();
        processMqttItems();//send temp and hum to MQTT Broker
        delay(500);
      }
    }
    else
    {
      if (debugOutput) Serial.println("MQTT is not connected... retrying");
      delay(200);
    }
  }
    // disconnect from MQTT
    client.disconnect();
    if (debugOutput) Serial.println("MQTT is disconnected");
    yield();

    // disconnect from WiFi
    WiFi.disconnect();
    if (debugOutput) Serial.println("WiFi is disconnected");
    yield();


  // all done enable deep_sleep
   long sleep_dur = REPEAT_INTERVAL * 1000 * 1000 - millis() * 1000;
   if (debugOutput) Serial.println("sleeping for " + String(REPEAT_INTERVAL) + " seconds");
   system_deep_sleep(sleep_dur);
   if (debugOutput) Serial.println("ESP8266 run complete in "+String(millis())+" ms");
   delay(1000);//waiting a little bit, deep sleep needs some time to start
}

void startSensors(void) {
   // Setup DHT22
   dht.setup(DHT22_PIN);
}

// ------------------------
// publish mit retain
boolean myPublish(char* topic, char* payload)
{
  client.publish(topic,(uint8_t*)payload,strlen(payload),true);
}

// ------------------------
// process DHT22
boolean processDHT()
{
  if (debugOutput) Serial.println("DHT22: +++DHT start");
  
  dht.readSensor();
  if (dht.getStatus() != dht.ERROR_NONE)
  {
    if (debugOutput) Serial.print("DHT22 Error: ");
    if (debugOutput) Serial.println(dht.getStatusString());
    return false;
  }

  temperature = dht.getTemperature();
  if (isnan(temperature)) 
  {
    if (debugOutput) Serial.println("DHT22: Failed reading temperature from DHT");
  } 
  if (debugOutput) Serial.print("DHT22 Deg: "); Serial.println(temperature);
  
  humidity = dht.getHumidity();
  if (isnan(humidity)) {
    if (debugOutput) Serial.println("DHT22: Failed reading humidity from DHT");
  } 
  if (debugOutput) Serial.print("DHT22 Hum: "); Serial.println(humidity);
  if (debugOutput) Serial.println("DHT22: ---DHT done");
  return true;
}

// ---------------------------
// processes the items to be published
void processMqttItems()
{
  String strBuffer;
  strBuffer =  String(humidity);
  strBuffer.toCharArray(charBuffer,10);
  myPublish((char *)TOPIC_HumidityState,charBuffer);  
  if (debugOutput) Serial.println("Action: update hum");  
  yield();
 
  strBuffer =  String(temperature);
  strBuffer.toCharArray(charBuffer,10);
  myPublish((char *)TOPIC_TemperatureState,charBuffer);  
  if (debugOutput) Serial.println("Action: update temp");  

}
// -------------------------------

void loop() {
  //not needed
}

