// #include <ESP8266WiFi.h>     // ESP8266 core for Arduino https://github.com/esp8266/Arduino
// #include <PubSubClient.h>    // Arduino Client for MQTT https://github.com/knolleary/pubsubclient
// #include <ArduinoJson.h>     // For sending MQTT JSON messages https://bblanchon.github.io/ArduinoJson/

// #include <LC_NodeMcuCode.cpp>
//
//
//   // PubSubClient & mqttClient(WiFiClient & espClient),
//   // extern char & publishLastWillTopic,
//   // char & publishStatusJsonTopic,
//   // char & clientName,
//   // int & json_buffer_size
// void publishNodeState() {
//   // Update status to online, retained = true - last will Message will drop in if we go offline
//   mqttClient.publish(publishLastWillTopic, "online", true);
//   // Gather data
//   char bufIP[16]; // Wifi IP address
//   sprintf(bufIP, "%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] );
//   char bufMAC[6]; // Wifi MAC address
//   sprintf(bufMAC, "%02x:%02x:%02x:%02x:%02x:%02x", WiFi.macAddress()[0], WiFi.macAddress()[1], WiFi.macAddress()[2], WiFi.macAddress()[3], WiFi.macAddress()[4], WiFi.macAddress()[5] );
//   // Create and publish the JSON object.
//   StaticJsonBuffer<json_buffer_size> jsonBuffer;
//   JsonObject& root = jsonBuffer.createObject();
//   // INFO: the data must be converted into a string; a problem occurs when using floats...
//   root["ClientName"] = String(clientName);
//   root["IP"] = String(bufIP);
//   root["MAC"] = String(bufMAC);
//   root["RSSI"] = String(WiFi.RSSI());
//   root["HostName"] = String(WiFi.hostname());
//   root["ConnectedSSID"] = String(WiFi.SSID());
//   root.prettyPrintTo(Serial);
//   Serial.println(""); // Add new line as prettyPrintTo leaves the line open.
//   char data[json_buffer_size];
//   root.printTo(data, root.measureLength() + 1);
//   if (!mqttClient.publish(publishStatusJsonTopic, data, true)) // retained = true
//     Serial.print(F("Failed to publish JSON Status to [")), Serial.print(publishStatusJsonTopic), Serial.print("] ");
//   else
//     Serial.print(F("JSON Status Published [")), Serial.print(publishStatusJsonTopic), Serial.println("] ");
// }
