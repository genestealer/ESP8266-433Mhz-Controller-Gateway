/***************************************************
  ESP8266 433Mhz Controller Gateway (Formally: Lighting Controller)
  Richard Huish 2016-2017
  ESP8266 based with local home-assistant.io GUI,
    433Mhz transmitter for remote lighting control
    and DHT22 temperature-humidity sensor.
    Temperature and humidity sent as JSON via MQTT
  ----------
  Github: https://github.com/Genestealer/ESP8266-433Mhz-Controller-Gateway
  ----------
  Key Libraries:
  ESP8266WiFi.h     https://github.com/esp8266/Arduino
  RCSwitch.h        https://github.com/sui77/rc-switch
  DHT.h             https://github.com/adafruit/DHT-sensor-library
  Adafruit_Sensor.h https://github.com/adafruit/Adafruit_Sensor required for DHT.h
  ESP8266mDNS.h     https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266mDNS
  WiFiUdp.h         https://github.com/esp8266/Arduino
  ArduinoOTA.h      https://github.com/esp8266/Arduino
  ArduinoJson.h     https://bblanchon.github.io/ArduinoJson/
  ----------
  GUI: Locally hosted home assistant
  MQTT: Locally hosted broker https://mosquitto.org/
  OTA updates
  ----------
  The circuit:
    NodeMCU Amica (ESP8266)
  Inputs:
    DHT22 temperature-humidity sensor - GPIO pin 5 (NodeMCU Pin D1)
  Outputs:
    433Mhz Transmitter - GPIO pin 2 (NODEMCU Pin D4)
    LED_NODEMCU        - GPIO pin 16 (NodeMCU pin D0)
    LED_ESP            - GPIO pin 2  (NodeMCU pin D4) (Shared with 433Mhz TX)
    ----------
  Notes:
    NodeMCU lED lights to show MQTT conenction.
    ESP lED lights to show WIFI conenction.
  ----------
  Edits made to the PlatformIO Project Configuration File:
    platform = espressif8266_stage = https://github.com/esp8266/Arduino/issues/2833 as the standard has an outdated Arduino Core for the ESP8266, ref http://docs.platformio.org/en/latest/platforms/espressif8266.html#over-the-air-ota-update
    build_flags = -DMQTT_MAX_PACKET_SIZE=512 = Overide max JSON size, until libary is updated to inclde this option https://github.com/knolleary/pubsubclient/issues/110#issuecomment-174953049
  ----------
  Sources:
  https://github.com/mertenats/open-home-automation/tree/master/ha_mqtt_sensor_dht22
  Create a JSON object
    Example https://github.com/mertenats/Open-Home-Automation/blob/master/ha_mqtt_sensor_dht22/ha_mqtt_sensor_dht22.ino
    Doc : https://github.com/bblanchon/ArduinoJson/wiki/API%20Reference
****************************************************/

// Note: Libaries are inluced in "Project Dependencies" file platformio.ini
#include <private.h>         // Passwords etc not for github
#include <ESP8266WiFi.h>     // ESP8266 core for Arduino https://github.com/esp8266/Arduino
#include <PubSubClient.h>    // Arduino Client for MQTT https://github.com/knolleary/pubsubclient
#include <ESP8266mDNS.h>     // Needed for Over-the-Air ESP8266 programming https://github.com/esp8266/Arduino
#include <WiFiUdp.h>         // Needed for Over-the-Air ESP8266 programming https://github.com/esp8266/Arduino
#include <ArduinoOTA.h>      // Needed for Over-the-Air ESP8266 programming https://github.com/esp8266/Arduino
#include <ArduinoJson.h>     // For sending MQTT JSON messages https://bblanchon.github.io/ArduinoJson/
#include <RCSwitch.h>        // RF control lib, https://github.com/sui77/rc-switch
#include <DHT.h>             // DHT Sensor lib, https://github.com/adafruit/DHT-sensor-library
#include <Adafruit_Sensor.h> // Have to add for the DHT to work https://github.com/adafruit/Adafruit_Sensor


// #include <IRremoteESP8266.h> //https://github.com/markszabo/IRremoteESP8266

// WiFi parameters
const char* wifi_ssid = secret_wifi_ssid; // Wifi access point SSID
const char* wifi_password = secret_wifi_password; // Wifi access point password
int noWifiConnectionCount = 0;
const int noWifiConnectionCountLimit = 5;
int noWifiConnectionCountRebootCount = 0;
const int noWifiConnectionCountRebootLimit = 5;

// MQTT Settings
WiFiClient espClient;
PubSubClient mqttClient(espClient);
char message_buff[100];
long lastReconnectAttempt = 0; // Reconnecting MQTT - non-blocking https://github.com/knolleary/pubsubclient/blob/master/examples/mqtt_reconnect_nonblocking/mqtt_reconnect_nonblocking.ino
const char* mqtt_server = secret_mqtt_server; // E.G. 192.168.1.xx
const char* clientName = secret_clientName; // Client to report to MQTT
const char* mqtt_username = secret_mqtt_username; // MQTT Username
const char* mqtt_password = secret_mqtt_password; // MQTT Password
bool willRetain = true; // MQTT Last Will and Testament
const char* willMessage = "offline"; // MQTT Last Will and Testament Message
const int json_buffer_size = 256;
int noMqttConnectionCount = 0;
const int noMqttConnectionCountLimit = 5;
// MQTT Subscribe
const char* subscribeLightingGatewayTopic = secret_subscribeLightingGatewayTopic; // E.G. Home/LightingGateway/transmit
// MQTT Publish
const char* publishLastWillTopic = secret_publishLastWillTopic;              // MQTT last will
const char* publishNodeStatusJsonTopic = secret_publishNodeStatusJsonTopic;  // State of the node
const char* publishNodeHealthJsonTopic = secret_publishNodeHealthJsonTopic;  // Health of the node
// MQTT publish frequency
unsigned long previousMillis = 0;
const long publishInterval = 60000; // Publish frequency in milliseconds 60000 = 1 min

// LED output parameters
const int DIGITAL_PIN_LED_ESP = 2; // Define LED on ESP8266 sub-modual
const int DIGITAL_PIN_LED_NODEMCU = 16; // Define LED on NodeMCU board - Lights on pin LOW


// DHT sensor parameters
#define DHTPIN 5 // GPIO pin 5 (NodeMCU Pin D1)
#define DHTTYPE DHT22
// DHT sensor instance
DHT dht(DHTPIN, DHTTYPE, 15);

// 433MHZ TX instance
RCSwitch mySwitch = RCSwitch();
// 433Mhz transmitter parameters
const int tx433Mhz_pin = 2; // GPIO pin 2 (NODEMCU Pin D4)
const int setPulseLength = 305;

// Setup the connection to WIFI. Normally called only once from setup
void setup_wifi() {
  /* Explicitly set the ESP8266 to be a WiFi-client, otherwise, it by default,
     would try to act as both a client and an access-point and could cause
     network-issues with your other WiFi-devices on your WiFi-network. */
  WiFi.mode(WIFI_STA);
  // Connect to the WiFi network
  Serial.println("Connecting to " + String(wifi_ssid) + "...");
  WiFi.begin(wifi_ssid, wifi_password);
  while ((WiFi.status() != WL_CONNECTED) && (noWifiConnectionCount < noWifiConnectionCountLimit)) {
    delay(500);
    noWifiConnectionCount = ++noWifiConnectionCount; //Increment the counter
    Serial.print("Wifi connection attempt number: ");
    Serial.println(noWifiConnectionCount);
  }
  if (WiFi.status() != WL_CONNECTED) {
    // If no wifi, fall-back to local mode with pre-set values.
    Serial.println("WiFi not connected. Running in Local Mode!");
  } else {
    Serial.println("WiFi connected");

    Serial.printf("RSSI: %d dBm\n", WiFi.RSSI());
    Serial.println("IP address: " + String(WiFi.localIP()));
    Serial.printf("Hostname: %s\n", WiFi.hostname().c_str());

    digitalWrite(DIGITAL_PIN_LED_NODEMCU, LOW); // Lights on LOW. Light the NodeMCU LED to show wifi connection.
  }
}

// If disconnected from Wifi, and attempt reconnection.
// Code source: https://github.com/alukach/Amazon-Alexa-RF-Outlets-Integration/blob/master/wemos.ino
bool reconnectWifi() {
  Serial.println("Disconnected; Attempting reconnect to " + String(wifi_ssid) + "...");
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_ssid, wifi_password);
  // Output reconnection status info every second over the next 10 sec
  for ( int i = 0; i < 10 ; i++ )  {
    delay(500);
    Serial.print("WiFi status = ");
    if ( WiFi.status() == WL_CONNECTED ) {
      Serial.println("Connected");
      return true;
    } else {
      Serial.println("Disconnected");
    }
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Failure to establish connection after 10 sec. Returning to Local Mode");
    return false;
  }
}

// Setup Over-the-Air programming, called from the setup.
// https://www.penninkhof.com/2015/12/1610-over-the-air-esp8266-programming-using-platformio/
void setup_OTA() {
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);
  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");
  // No authentication by default
  // ArduinoOTA.setPassword("admin");
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";
    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
}



// 433Mhz Gatway
void transmit433Msg(int msgToSend) {
  // Acknowledgements
  // thisoldgeek/ESP8266-RCSwitch
  // https://github.com/thisoldgeek/ESP8266-RCSwitch
  // Find the codes for your RC Switch using https://github.com/ninjablocks/433Utils (RF_Sniffer.ino)
  // sui77/rc-switch
  // https://github.com/sui77/rc-switch/tree/c5645170be8cb3044f4a8ca8565bfd2d221ba182
  mySwitch.send(msgToSend, 24);
  Serial.println(F("433Mhz TX command sent!"));
}

// Publish this nodes state via MQTT
void publishNodeHealth() {
  // Update status to online, retained = true - last will Message will drop in if we go offline
  mqttClient.publish(publishLastWillTopic, "online", true);
  // Gather data
  char bufIP[16]; // Wifi IP address
  sprintf(bufIP, "%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] );
  char bufMAC[6]; // Wifi MAC address
  sprintf(bufMAC, "%02x:%02x:%02x:%02x:%02x:%02x", WiFi.macAddress()[0], WiFi.macAddress()[1], WiFi.macAddress()[2], WiFi.macAddress()[3], WiFi.macAddress()[4], WiFi.macAddress()[5] );
  // Create and publish the JSON object.
  StaticJsonBuffer<json_buffer_size> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  // INFO: the data must be converted into a string; a problem occurs when using floats...
  root["ClientName"] = String(clientName);
  root["IP"] = String(bufIP);
  root["MAC"] = String(bufMAC);
  root["RSSI"] = String(WiFi.RSSI());
  root["HostName"] = String(WiFi.hostname());
  root["ConnectedSSID"] = String(WiFi.SSID());
  root.prettyPrintTo(Serial);
  Serial.println(""); // Add new line as prettyPrintTo leaves the line open.
  char data[json_buffer_size];
  root.printTo(data, root.measureLength() + 1);
  if (!mqttClient.publish(publishNodeHealthJsonTopic, data, true)) // retained = true
    Serial.println("Failed to publish JSON Node Health to [" + String(publishNodeHealthJsonTopic) + "]");
  else
    Serial.println("JSON Node Health Published [" + String(publishNodeHealthJsonTopic) + "]");
}

// Subscribe to MQTT topics
void mqttSubscribe() {
  mqttClient.subscribe(subscribeLightingGatewayTopic);
}

/*
  Non-Blocking mqtt reconnect.
  Called from checkMqttConnection.
  Based on example from 5ace47b Sep 7, 2015 https://github.com/knolleary/pubsubclient/blob/master/examples/mqtt_reconnect_nonblocking/mqtt_reconnect_nonblocking.ino
*/
boolean mqttReconnect() {
  Serial.println("mqttReconnect");
  // Call on the background functions to allow them to do their thing
  yield();
  // Attempt to connect
  if (mqttClient.connect(clientName, mqtt_username, mqtt_password, publishLastWillTopic, 0, willRetain, willMessage)) {
    Serial.print("Attempting MQTT connection...");
    // Publish node state data
    publishNodeHealth();
    // Resubscribe to feeds
    mqttSubscribe();
    Serial.println("Connected to MQTT server");
  } else {
    Serial.println("Failed MQTT connection, rc=" + String(publishNodeStatusJsonTopic) + "] ");
  }
  return mqttClient.connected(); // Return connection state
}

/*
  Checks if connection to the MQTT server is ok. Client connected
  using a non-blocking reconnect function. If the client loses
  its connection, it attempts to reconnect every 5 seconds
  without blocking the main loop.
  Called from main loop.
  If MQTT connection fails after x attempts it tries to reconnect wifi
  If wifi connections fails after x attempts it reboots the esp
*/
void checkMqttConnection() {
  if (!mqttClient.connected()) {
    // We are not connected. Turn off the wifi LED
    digitalWrite(DIGITAL_PIN_LED_ESP, HIGH); // Lights on LOW
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      // Attempt to reconnect
      if (mqttReconnect()) {
        // We are connected.
        lastReconnectAttempt = 0;
        noMqttConnectionCount = 0;
        noWifiConnectionCountRebootCount = 0;
        digitalWrite(DIGITAL_PIN_LED_ESP, LOW); // Lights on LOW
      } else  {
        // Connection to MQTT failed.
        // If no connection after x attempts, then reconnect wifi, if no connection after x attempts reboot.
        noMqttConnectionCount = ++noMqttConnectionCount; //Increment the counter
        Serial.println("MQTT connection attempt number: " + String(noMqttConnectionCount));
        if (noMqttConnectionCount > noMqttConnectionCountLimit) {
          // Max MQTT connection attempts reached, reconnect wifi.
          noMqttConnectionCount = 0; // Reset MQTT connection attempt counter.
          Serial.println("MQTT connection count limit reached, reconnecting wifi");
          // Try to reconnect wifi, if this fails after x attemps then reboot.
          if (!reconnectWifi()) {
            noWifiConnectionCountRebootCount = ++noWifiConnectionCountRebootCount;
            Serial.println("Wifi connection attempt number: " + String(noWifiConnectionCountRebootCount));
            if (noWifiConnectionCountRebootCount > noWifiConnectionCountRebootLimit) {
              Serial.println("Wifi re-connection count limit reached, reboot esp");
              // Reboot ESP
              ESP.restart();
            }
          }
        }
      }


    }
  } else {
    //Call on the background functions to allow them to do their thing.
    yield();
    // Client connected: MQTT client loop processing
    mqttClient.loop();
  }
}

// MQTT Publish
void mqttPublishStatusData() {
  // Only run when publishInterval in milliseonds exspires
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= publishInterval) {
    previousMillis = currentMillis; // Save the last time this ran
    // Check connection to MQTT server
    if (mqttClient.connected()) {
      // Publish node state data
      publishNodeHealth();

      // JSON data method
      StaticJsonBuffer<json_buffer_size> jsonBuffer;
      JsonObject& root = jsonBuffer.createObject();
      // INFO: the data must be converted into a string; a problem occurs when using floats...
      root["Temperature"] = String(dht.readTemperature());
      root["Humidity"] = String(dht.readHumidity());
      root.prettyPrintTo(Serial);
      Serial.println(""); // Add new line as prettyPrintTo leaves the line open.
      char data[json_buffer_size];
      root.printTo(data, root.measureLength() + 1);
      if (!mqttClient.publish(publishNodeStatusJsonTopic, data, true))  //Retain data.
        Serial.println("Failed to publish JSON sensor data to [" + String(publishNodeStatusJsonTopic) + "]");
      else
        Serial.println("JSON Sensor data published to [" + String(publishNodeStatusJsonTopic) + "] ");
    }
  }
}


// MQTT payload
// to transmit via out gateway
void mqttcallback(char* topic, byte* payload, unsigned int length) {
  //If you want to publish a message from within the message callback function, it is necessary to make a copy of the topic and payload values as the client uses the same internal buffer for inbound and outbound messages:
  //http://www.hivemq.com/blog/mqtt-client-library-encyclopedia-arduino-pubsubclient/
  Serial.println("Message arrived [" + String(topic) + "] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  // Create character buffer with ending null terminator (string)
  int i = 0;
  for (i = 0; i < length; i++) {
    message_buff[i] = payload[i];
  }
  message_buff[i] = '\0';
  // Check the value of the message
  String msgString = String(message_buff);
  Serial.println(msgString);
  // Check the message topic
  String srtTopic = topic;

  if (srtTopic.equals(subscribeLightingGatewayTopic)) {

    int msgSend = msgString.toInt();
    //do the transmittt
    transmit433Msg(msgSend);
    delay(500);

  }
}

// Custom setup for this program.
void customSetup() {
  // Initialize pin start values
  digitalWrite(DIGITAL_PIN_LED_NODEMCU, LOW); // Lights on HIGH
  digitalWrite(DIGITAL_PIN_LED_ESP, HIGH); // Lights on LOW

  // Setup 433Mhz Transmitter
  pinMode(tx433Mhz_pin, OUTPUT);
  // Set 433Mhz pin to output
  mySwitch.enableTransmit(tx433Mhz_pin);
  // Set pulse length.
  mySwitch.setPulseLength(setPulseLength);
  // Optional set protocol (default is 1, will work for most outlets)
  mySwitch.setProtocol(1);

  // Init temperature and humidity sensor
  dht.begin();

}
void setup() {
  // Set serial speed
  Serial.begin(115200);
  Serial.println("Setup Starting");
  // Initialize pins
  pinMode(DIGITAL_PIN_LED_NODEMCU, OUTPUT);
  pinMode(DIGITAL_PIN_LED_ESP, OUTPUT);
  // Initialize pin start values
  digitalWrite(DIGITAL_PIN_LED_NODEMCU, LOW); // Lights on HIGH
  digitalWrite(DIGITAL_PIN_LED_ESP, HIGH); // Lights on LOW

  // Call on the background functions to allow them to do their thing
  yield();
  // Setup wifi
  setup_wifi();
  // Call on the background functions to allow them to do their thing
  yield();
  // Setup OTA updates.
  setup_OTA();
  // Call on the background functions to allow them to do their thing
  yield();
  // Set MQTT settings
  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setCallback(mqttcallback);
  // Call on the background functions to allow them to do their thing
  yield();
  // Setup for this project.
  customSetup();
  Serial.println("Setup Complete");
}

// Main working loop
void loop() {
  // Call on the background functions to allow them to do their thing.
  yield();
  // Check for Over The Air updates
  ArduinoOTA.handle();
  // Call on the background functions to allow them to do their thing.
  yield();
  // Check if we are connected to the MQTT broker
  checkMqttConnection();
  // Publish MQTT
  mqttPublishStatusData();
  // Call on the background functions to allow them to do their thing.
  yield();
  // Deal with millis rollover, hack by resetting the esp every 48 days
  if (millis() > 4147200000)
    ESP.restart();
}
