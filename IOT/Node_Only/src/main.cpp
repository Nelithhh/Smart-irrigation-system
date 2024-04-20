#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>
#include "ThingSpeak.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <RTClib.h>

#define RELAYPIN D3                     // For Solenoid
#define SENSOR_PIN D6                   // For Waterflow Sensor
#define SOILSENSOR A0
#define RAINSENSOR D7

#define SEALEVELPRESSURE_HPA (1013.25)

#define VALVE_OPEN HIGH
#define VALVE_CLOSED LOW

Adafruit_BME280 bme;                  
RTC_DS3231 rtc;
ESP8266WebServer server(80);
WiFiClient client;

StaticJsonDocument<512> jsonDoc;        // Json Document

const char* ssid = "Nelith";            // Network Credentials
const char* password = "nelith123";

unsigned long myChannelNumber = 2453905; // ThingSpeak Channel Details
const char * myWriteAPIKey = "H4VBD7OCL23WPQQY";


unsigned long oldTime = 0;              // ThingSpeak Timer
unsigned long timerDelay = 30000;

char t[32];
float FlowRate;
volatile long pulseCount;
unsigned long lastTime;

String RainStatus;
int RainSensor, SoilMoistureSensor;
float Temperature, Pressure, ApproxAltitude, Humidity;
volatile float dispensedWater = 0;

int predef01 = 0001;
int predef02 = 0002;
int predef03 = 0003;
int predef04 = 0100;
int oncode   = 1111;
int offcode  = 0000;

void openValve() {
  digitalWrite(RELAYPIN, VALVE_OPEN); 
}

void closeValve() {
  digitalWrite(RELAYPIN, VALVE_CLOSED);
}

void dispenseWater(float targetVolume) {
  unsigned long startTime = millis(); 
  dispensedWater = 0; 
  openValve();
  while (dispensedWater < targetVolume) {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - startTime;
    dispensedWater = (float)pulseCount / 1000 * elapsedTime / 1000;
    if (dispensedWater >= targetVolume) {
      break; 
    }
    delay(100);
  }
  closeValve();
}

void conditionCheck(){
  if (((Temperature > 4.5) && (Temperature < 35)) && (Humidity > 70) && (Pressure > 1000)){
    closeValve();
  }
  else if (SoilMoistureSensor < 204){
    openValve();
  }
}

void openValveAtSpecificTime() {
  DateTime now = rtc.now();
  if (now.hour() == 10 && now.minute() == 0 && now.second() == 0) {
    if (RainSensor == 1){
    conditionCheck();
    } 
  }
}

void publishToTS(){                                                 // --> sending data to thingspeak
  if ((millis() - oldTime) > timerDelay) {
    if (WiFi.status() == WL_CONNECTED) {
        
        ThingSpeak.setField(1, SoilMoistureSensor);
        ThingSpeak.setField(2, Pressure);
        ThingSpeak.setField(3, Humidity);
        ThingSpeak.setField(4, Temperature);
        ThingSpeak.setField(5, FlowRate);
        int resultCode = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
        if (resultCode == 200) {
            Serial.println("Channel update successful.");
        } else {
            Serial.println("Problem updating channel. HTTP error code " + String(resultCode));
        }
        oldTime = millis();
    }
  }
}

void hostJSON(){                                            // --> showing data to the json ip address/data
  jsonDoc["FlowRate"] = FlowRate;
  jsonDoc["Temperature"] = Temperature;
  jsonDoc["Pressure"] = Pressure;
  jsonDoc["ApproxAltitude"] = ApproxAltitude;
  jsonDoc["Humidity"] = Humidity;
  jsonDoc["RainSensor"] = RainStatus;
  jsonDoc["SoilMoistureSensor"] = SoilMoistureSensor; 
}
void handleData() {                                            // --> get data from client side
  if (server.hasArg("dataField")) {
    String data = server.arg("dataField");
    Serial.println("Received data: " + data);
    server.send(200, "text/plain", "Data received: " + data);

    int num_data = data.toInt();
    if (num_data == predef01){
      dispenseWater(1);
    }
    else if (num_data == predef02){
      dispenseWater(2);;
    }
    else if (num_data == predef03){
      dispenseWater(3);
    }
    else if (num_data == predef04){
      dispenseWater(0.1);
    }
    else if (num_data == oncode){
      openValve();
    }
    else if (num_data == offcode){
      closeValve();
    }
  } else {
    server.send(400, "text/plain", "400: Invalid Request - Data not found");
  }
}

void getI2CData() {                    // --> get Data From BME280 Sensor And RTC
  DateTime now = rtc.now();
  sprintf(t, "%02d:%02d:%02d %02d/%02d/%02d", now.hour(), now.minute(), now.second(), now.day(), now.month(), now.year());

  Temperature = bme.readTemperature();
  Pressure = bme.readPressure() / 100.0F;
  ApproxAltitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  Humidity = bme.readHumidity();
}

void jsonRain(){            // --> setting rain sensor data to rainning or not
  if (RainSensor == 1){
    RainStatus = "Not Raining";
  }
  else if (RainSensor == 0){
    RainStatus = "Raining";
  }
}

void getADCData() {                     //  --> get Data Soil Rain And Soil Sensor
  SoilMoistureSensor = analogRead(SOILSENSOR);
  RainSensor = digitalRead(RAINSENSOR);
}

void printParameters(){                                     // --> prints all data on serial monitor
  Serial.println("==============================");
  Serial.print("DateTime:");
  Serial.println(t);
  Serial.print("Temperature:");
  Serial.println(Temperature);
  Serial.print("Pressure:");
  Serial.println(Pressure);
  Serial.print("ApproxAltitude:");
  Serial.println(ApproxAltitude);
  Serial.print("Humidity:");
  Serial.println(Humidity);
  Serial.print("Flow Rate:");
  Serial.print(FlowRate);          
  Serial.println(" mL/m");
  Serial.print("RainSensor:");
  Serial.println(RainSensor);
  Serial.print("SoilMoistureSensor:");
  Serial.println(SoilMoistureSensor);

}

ICACHE_RAM_ATTR void pulseCounter() {        // --> countinf the speed of waterflow sensor
  pulseCount++;
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);             //  --> Initiate WIFI Connnections
  Serial.println("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
  Serial.println("IP address: " + WiFi.localIP().toString());

  ThingSpeak.begin(client);              // --> Start ThingSpeak Client

    server.on("/data", HTTP_GET, [](){   // --> Server Endpoint Definition --> http://192.168.8.188/data
      String jsonData;
      serializeJson(jsonDoc, jsonData);
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", jsonData);
  });

  server.on("/trigger", HTTP_POST, handleData);

  server.begin();                                   // --> finding whether sensor is connected or not
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  Wire.begin();
  rtc.begin();
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  pinMode(RELAYPIN, OUTPUT);
  digitalWrite(RELAYPIN, LOW);          //  --> Relay Controlling Valve

  pinMode(SENSOR_PIN, INPUT);
  digitalWrite(SENSOR_PIN, HIGH);

  pinMode(RELAYPIN, OUTPUT); 
  digitalWrite(RELAYPIN, LOW);

  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), pulseCounter, FALLING);
}

void loop() {
  server.handleClient();

  FlowRate = 2.663 * pulseCount;
  if (millis() - lastTime > 2000) {
    pulseCount = 0;
    lastTime = millis();
  }
  
  getI2CData();
  getADCData();
  jsonRain();
  hostJSON();
  publishToTS();
  openValveAtSpecificTime();

  // printParameters();
}