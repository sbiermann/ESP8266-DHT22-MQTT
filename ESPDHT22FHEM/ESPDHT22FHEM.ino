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
uint16 readvdd33(void);
}

/*************************************************/
/* Deep Sleep                                    */
/*************************************************/
// Please see also: http://forum.fhem.de/index.php/topic,35572.msg279312.html#msg279312
// 
const bool DeepSleep = true;  // set to true to activate deep sleep

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

#define DHT22_PIN 2       // Pin 2 as 1-wire bus for DHT22

/*************************************************/
/* Settings for WLAN                             */
/*************************************************/
const char* ssid = "WLANFR";
const char* password = "supergeheimespasswort";

/*************************************************/
/* Static IP                                     */
/*************************************************/
const bool dhcp = false;         // set to true for DHCP
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
#define TOPIC_CounterState    TOPIC_ClientID "/counter/value"
#define TOPIC_HumidityState  TOPIC_ClientID "/humidity/value"
#define TOPIC_TemperatureState  TOPIC_ClientID "/temperature/value"


// stati der mqtt statemachine
#define ST_INIT 0
#define ST_CONNECT 2
#define ST_CONNECTED 3
#define ST_DISTRIBUTED 4

bool firstInit = true;

// Create an WiFiClient object, here called "ethClient":
WiFiClient ethClient;

//mqtt client verbindung definieren, samt callback
PubSubClient client(ethClient, mqttBroker, mqttPort); // instanziierung ueber DNS Namen

// Create an DHT object, here called "dht":
DHT dht;
unsigned long dhtWaiter;
bool tempReq=false;
bool humReq=false;
float lastTemp;
float lastHum;

int MqState = 0;
unsigned long MqWaiter; 
boolean callbackEnable=false;

char charBuffer[32];

byte counter = 1;
unsigned long counterWaiter;
bool counterReq = false;

// Variables
long wiFiConnTime = 0; // measure time till wifi connect success
int wifiConnectCounter, loopCounter, httpConnectCounter = 0;

// Pin14: VCC for Sensors
const int GPIO_Power_Sensors = 14;


int min(int x, int y) {if (x<y) return x; else return y;}
int max(int x, int y) {if (x>y) return x; else return y;}
// ------------------------
void setup() {
  
   Serial.begin(115200);   

   Serial.println("ESPtoFHEM starts...");

   // power on sensors
   pinMode(GPIO_Power_Sensors, OUTPUT);
   digitalWrite(GPIO_Power_Sensors, HIGH);
   
   // wait for sensors to power on
   delay(2000);
    
   Serial.println("Wifi wait for connection");

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
   if (!dhcp) WiFi.config(ip, gateway, subnet);

   // Wait for connection
   while (WiFi.status() != WL_CONNECTED) {
     delay(500);
     wifiConnectCounter++;
     if (debugOutput)  Serial.print(".");
     if (wifiConnectCounter > 100) {
       if (debugOutput)  Serial.println("No WiFi connection, try again in " + String(REPEAT_INTERVAL) + " seconds");
       system_deep_sleep(REPEAT_INTERVAL * 1000 * 1000); // after deep sleep automtic reset
       delay(200); // give some time to activate sleep state
       abort(); // just for convience, when disabling deep sleep
     }
     Serial.print(".");
   }
   wifiConnectCounter = 0;
   if (debugOutput) Serial.println("");
   if (debugOutput) Serial.println("WiFi connected");
   if (debugOutput) Serial.println("IP address: ");
   if (debugOutput) Serial.println(WiFi.localIP());
   wiFiConnTime = millis(); // establish time for connection in milliseconds
   wifi_set_sleep_type (LIGHT_SLEEP_T);

   Serial.println("Wifi Started");

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
void processDHT()
{
  // process after wait time or counter turn around 
    
  if (millis() < dhtWaiter && !DeepSleep)
    return;
        
  dhtWaiter = millis() + max((REPEAT_INTERVAL * 1000),dht.getMinimumSamplingPeriod());

  // process after wait time or counter turn around 
  // if (millis() < dhtWaiter && abs(dhtWaiter-millis()) < 10000 )
  //     return;
        
  // dhtWaiter = millis() + max(5000,dht.getMinimumSamplingPeriod()); 

  if (debugOutput) Serial.println("DHT22: +++DHT start");
  
  dht.readSensor();
  if (dht.getStatus() != dht.ERROR_NONE)
  {
    if (debugOutput) Serial.print("DHT22 Error: ");
    if (debugOutput) Serial.println(dht.getStatusString());
    return;
  }

  float temperature = dht.getTemperature();
  
  if (isnan(temperature)) 
  {
    if (debugOutput) Serial.println("DHT22: Failed reading temperature from DHT");
  } 
  else if (temperature != lastTemp || firstInit) {
    lastTemp = temperature;
    tempReq = true;
    if (debugOutput) Serial.print("DHT22 Deg: "); Serial.println(temperature);
  }

  float humidity = dht.getHumidity();

  if (isnan(humidity)) {
    if (debugOutput) Serial.println("DHT22: Failed reading humidity from DHT");
  } 
  else if (humidity != lastHum || firstInit) 
  {
    lastHum = humidity;
    humReq = true;

    if (debugOutput) Serial.print("DHT22 Hum: "); Serial.println(humidity);
  }

  if (debugOutput) Serial.println("DHT22: ---DHT done");

}

// ---------------------------
// processes the items to be published
void processMqttItems()
{
  char buffer[30];
  String strBuffer;
  
  if (counterReq)
  {
      counterReq=false;
      strBuffer =  String(counter);
      strBuffer.toCharArray(charBuffer,10);
      myPublish((char *)TOPIC_CounterState,charBuffer);  
      if (debugOutput) Serial.println("Action: update counter");  
  }
  yield();

  if (humReq)
  {
     humReq=false;
     strBuffer =  String(lastHum);
     strBuffer.toCharArray(charBuffer,10);
     myPublish((char *)TOPIC_HumidityState,charBuffer);  
     if (debugOutput) Serial.println("Action: update hum");  
  }
  yield();
  
  if (tempReq)
  {
     tempReq=false;
     strBuffer =  String(lastTemp);
     strBuffer.toCharArray(charBuffer,10);
     myPublish((char *)TOPIC_TemperatureState,charBuffer);  
     if (debugOutput) Serial.println("Action: update temp");  
  }
}
// -------------------------------

void processMQTT()
{
 
  int nextState=MqState;

  if (millis() < MqWaiter)
    return;

  if (MqState == ST_INIT)
  {
    if (debugOutput) Serial.println("MQTT: Init");
    client.disconnect();
    MqWaiter = millis() + 2000;
    nextState = ST_CONNECT;

  } else if (MqState == ST_CONNECT) {
    if (debugOutput) Serial.println("MQTT: +Try to connect");
    if (debugOutput) Serial.print("MQTT ClientID:");Serial.println(TOPIC_ClientID);
    
    callbackEnable=false;
    
    // clientID, user,password,Will-Topic, Qos, Retain, will msg
    //if (client.connect((char *)TOPIC_ClientID))  
     if (client.connect((char *)TOPIC_ClientID,(char *)TOPIC_LastWill,1,1,(char *)"0"))  
    {
      if (debugOutput) Serial.println("MQTT: connected");

      nextState = ST_CONNECTED;
    } else {
      if (debugOutput) Serial.println("MQTT: not connected");
      nextState = ST_INIT;
    }
  } else if (MqState >= ST_CONNECTED) {
    // mqtt loop
    if ( ! client.loop())
    {
      if (debugOutput) Serial.println("MQTT: disconnected");
      nextState = ST_INIT;
    }
    else {
      processMqttItems();
      // if (debugOutput) Serial.println("MQTT: distributed");
      nextState = ST_DISTRIBUTED;
    }
  }

  if (nextState != MqState)
  {
    MqState = nextState;
    if (debugOutput) Serial.print("MQTT new state:");
    if (debugOutput) Serial.println(nextState);
  }

}  

// -------------------------------
// process the counter

void processCounter()
{
   if (millis() < counterWaiter)
    return;
  
   counterWaiter=millis()+5000;
   counter++;
   counterReq=true;
  
}

void loop() {
  loopCounter++; // count loops for quality

  if (firstInit) {
    startSensors();
  }
  if (MqState >= ST_CONNECTED) {
    processDHT();
    yield();
  }
  processMQTT();
  yield();
  processCounter();
  
  firstInit = false;
  
  // all done enable deep_sleep

  if (MqState == ST_DISTRIBUTED  && DeepSleep) {
    firstInit = true;

    // disconnect from MQTT
    client.disconnect();
    yield();

    // disconnect from WiFi
    WiFi.disconnect();
    yield();

    // power off sensors
    pinMode(GPIO_Power_Sensors, INPUT);

    long sleep_dur = REPEAT_INTERVAL * 1000 * 1000 - millis() * 1000;
    Serial.println("sleeping for " + String(REPEAT_INTERVAL) + " seconds");
    if (sleep_dur < 0) sleep_dur = REPEAT_INTERVAL;
    system_deep_sleep(sleep_dur);
    delay(1000);
    abort();
  }

}

// Helpers for math function
double pow(double x,double y) {
  double z , p=1;
  //y<0 ? z=-y : z=y ;
  if(y<0)
    z = fabs(y);
  else
    z = y;
 
  for(int i=0; i<z ; ++i)
  {
    p *= x;
  }

  if(y<0)
    return 1/p;
  else
    return p;
}

double fabs(double x) {
  return( x<0 ?  -x :  x );
}

