#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <WebSocketsClient.h> //  https://github.com/kakopappa/sinric/wiki/How-to-add-dependency-libraries
#include <ArduinoJson.h> // https://github.com/kakopappa/sinric/wiki/How-to-add-dependency-libraries
#include <StreamString.h>
#include <Servo.h>

ESP8266WiFiMulti WiFiMulti;
WebSocketsClient webSocket;
WiFiClient client;
Servo servo;  // create servo object to control a servo
int pos = 0;    // variable to store the servo position

#define MyApiKey "Api key here" // TODO: Change to your sinric API Key. Your API Key is displayed on sinric.com dashboard
#define MySSID "SSID here" // TODO: Change to your Wifi network SSID
#define MyWifiPassword "Password here" // TODO: Change to your Wifi network password

#define TEMPRATURE_INTERVAL 300000 // 3 Minutes 
#define WIFI_INTERVAL 300000 // 3 Minutes 
#define HEARTBEAT_INTERVAL 300000 // 5 Minutes 

const int servoPin = D4;

int currenttemp;
float roomtemp;
float humid;
#define SERVER_URL "iot.sinric.com" //"iot.sinric.com"
#define SERVER_PORT 80 // 80
void setSetTemperatureSettingOnServer(String deviceId, float setPoint, String scale, float ambientTemperature, float ambientHumidity);
uint64_t WifiStatusTimeStamp = 0;
uint64_t heartbeatTimestamp = 0;
uint64_t tempratureUpdateTimestamp = 0;
bool isConnected = false;


#include "DHT.h"
#define DHTPIN D3     // what digital pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
DHT dht(DHTPIN, DHTTYPE);

#define settemperature 69

void turnOn(String deviceId) {
  if (deviceId == "Insert Device id here") // Device ID of first device
  {
    Serial.print("Turn on device id: ");
    Serial.println(deviceId);

    servo.attach(servoPin);
    servo.write(90);

    for (pos = 90; pos <= 135; pos += 1) { // goes from 0 degrees to 180 degrees
      servo.write(pos);              // tell servo to go to position in variable 'pos'
      Serial.println(pos);
      delay(15);                       // waits 15ms for the servo to reach the position
    }

    servo.detach();
  }
  else {
    Serial.print("Turn on for unknown device id: ");
    Serial.println(deviceId);
  }
}

void turnOff(String deviceId) {
  if (deviceId == "Insert Decice id here") // Device ID of first device
  {
    Serial.print("Turn off Device ID: ");
    Serial.println(deviceId);
    servo.attach(servoPin);

    for (pos = 90; pos >= 45; pos += 1) { // goes from 180 degrees to 0 degrees
      servo.write(pos);              // tell servo to go to position in variable 'pos'
      Serial.println(pos);
      delay(15);                       // waits 15ms for the servo to reach the position
    }

    servo.detach();
  }
  else {
    Serial.print("Turn off for unknown device id: ");
    Serial.println(deviceId);
  }
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      isConnected = false;
      Serial.printf("[WSc] Webservice disconnected from sinric.com!\n");
      break;
    case WStype_CONNECTED: {
        isConnected = true;
        Serial.printf("[WSc] Service connected to sinric.com at url: %s\n", payload);
        Serial.printf("Waiting for commands from sinric.com ...\n");
      }
      break;
    case WStype_TEXT: {
        Serial.printf("[WSc] get text: %s\n", payload);
        // Example payloads

        // For Switch or Light device types
        // {"deviceId": xxxx, "action": "setPowerState", value: "ON"} // https://developer.amazon.com/docs/device-apis/alexa-powercontroller.html

        // For Light device type
        // Look at the light example in github

        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject((char*)payload);
        String deviceId = json ["deviceId"];
        String action = json ["action"];

        if (action == "setPowerState") { // Switch or Light
          String value = json ["value"];
          if (value == "ON") {
            turnOn(deviceId);
          } else {
            turnOff(deviceId);
          }
        }
        else if (action == "SetTargetTemperature") {
          String deviceId = json ["deviceId"];
          String action = json ["action"];
          String value = json ["value"];
        }
        else if (action == "test") {
          Serial.println("[WSc] received test command from sinric.com");
        }
      }
      break;
    case WStype_BIN:
      Serial.printf("[WSc] get binary length: %u\n", length);
      break;
  }
}

void setup() {
  Serial.begin(115200);

  WiFiMulti.addAP(MySSID, MyWifiPassword);
  Serial.println();
  Serial.print("Connecting to Wifi: ");
  Serial.println(MySSID);

  // Waiting for Wifi connect
  while (WiFiMulti.run() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  if (WiFiMulti.run() == WL_CONNECTED) {
    Serial.println("");
    Serial.print("WiFi connected. ");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }

  // server address, port and URL
  webSocket.begin("iot.sinric.com", 80, "/");

  // event handler
  webSocket.onEvent(webSocketEvent);
  webSocket.setAuthorization("apikey", MyApiKey);

  // try again every 5000ms if connection has failed
  webSocket.setReconnectInterval(5000);   // If you see 'class WebSocketsClient' has no member named 'setReconnectInterval' error update arduinoWebSockets
  {
    Serial.begin(9600);
    Serial.println("DHT Eric Robb test!");

    dht.begin();
  }
}

void loop() {
  heart();
  temp();
}


void heart() {
  webSocket.loop();

  if (isConnected) {
    uint64_t now = millis();

    // Send heartbeat in order to avoid disconnections during ISP resetting IPs over night. Thanks @MacSass
    if ((now - heartbeatTimestamp) > HEARTBEAT_INTERVAL) {
      heartbeatTimestamp = now;
      webSocket.sendTXT("H");
    }
  }
}

void temp() {

  static unsigned long lastTime = 0;
  const long interval = 4000;
  static bool state = 0;

  Serial.print(getTemp("f"));
  Serial.println (" *F");
  Serial.println("-----------------");
  if (getTemp("f") <= settemperature +- 1)
    servo.attach(servoPin);
  servo.write(100);
  for (pos = 90; pos <= 135; pos += 1) { // goes from 0 degrees to 180 degrees
    //servo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }

  servo.detach();
  if (getTemp("f") >= settemperature +- 1)
    servo.attach(servoPin);
  servo.write(6);
  for (pos = 0; pos <= 135; pos += 1) { // goes from 0 degrees to 180 degrees
    //servo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }

  servo.detach();

}

float getTemp(String req)
{

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println("Failed to read from DHT sensor!");
    {
      return 0;
    }
  }
  // Compute heat index in Kelvin
  float k = t + 273.15;
  if (req == "c") {
    return t;//return Cilsus
  } else if (req == "f") {
    return f;// return Fahrenheit
  } else if (req == "h") {
    return h;// return humidity
  } else if (req == "hif") {
    return hif;// return heat index in Fahrenheit
  } else if (req == "hic") {
    return hic;// return heat index in Cilsus
  } else if (req == "k") {
    return k;// return temprature in Kelvin
  } else {
    return 0.000;// if no reqest found, retun 0.000
  }
}
