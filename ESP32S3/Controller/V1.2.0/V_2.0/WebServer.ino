#include <Wire.h>
#include <WiFi.h>
#include <EEPROM.h>
#include <WebServer.h>
#include <Arduino.h>

#define EEPROM_SIZE 512

#include <DallasTemperature.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>

#define DEVICE_ADDRESS_1 0x6D
#define DEVICE_ADDRESS_2 0x6D
#define PRESSURE_RANGE_1 2000.0
#define PRESSURE_RANGE_2 100.0

TwoWire Sensor1Wire = TwoWire(0);
TwoWire Sensor2Wire = TwoWire(1);

#define LED_PIN_R 37
#define LED_PIN_G 36
#define LED_PIN_B 35

#define BUTTON_PIN 0 // GPIO0 für manuellen AP-Modus-Schalter

WebServer server(80);

struct Config {
  char wifiSSID[32];
  char wifiPassword[32];
  char thingsboardServer[64];
  char token[32];
  uint16_t thingsboardPort;
  bool configValid;
};

Config config;

void setup() {
  Serial.begin(9600);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN_R, OUTPUT);
  pinMode(LED_PIN_G, OUTPUT);
  pinMode(LED_PIN_B, OUTPUT);
  digitalWrite(LED_PIN_R, HIGH);
  digitalWrite(LED_PIN_G, HIGH);
  digitalWrite(LED_PIN_B, HIGH);

  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(0, config);

  if (!config.configValid || digitalRead(BUTTON_PIN) == LOW) {
    startAPMode();
  } else {
    initWiFi();
  }

  initWebServer();
}

void loop() {
  server.handleClient();
}

void startAPMode() {
  WiFi.softAP("ESP32_Config", "admin123");
  IPAddress myIP = WiFi.softAPIP();
  server.begin();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
}

void initWebServer() {
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", getHTML());
  });

  server.on("/save", HTTP_POST, []() {
    String ssid = server.arg("ssid");
    String password = server.arg("password");
    String serverAddr = server.arg("server");
    String token = server.arg("token");
    uint16_t port = server.arg("port").toInt();

    if (ssid.length() < sizeof(config.wifiSSID) && password.length() < sizeof(config.wifiPassword)) {
      strncpy(config.wifiSSID, ssid.c_str(), sizeof(config.wifiSSID));
      strncpy(config.wifiPassword, password.c_str(), sizeof(config.wifiPassword));
      strncpy(config.thingsboardServer, serverAddr.c_str(), sizeof(config.thingsboardServer));
      strncpy(config.token, token.c_str(), sizeof(config.token));
      config.thingsboardPort = port;
      config.configValid = true;

      saveConfig();

      server.sendHeader("Connection", "close");
      server.send(200, "text/html", "<p>Configuration saved. Rebooting...</p>");
      delay(1000);
      ESP.restart();
    } else {
      server.sendHeader("Connection", "close");
      server.send(400, "text/html", "<p>Error: Input too long.</p>");
    }
  });
}

void initWiFi() {
  WiFi.begin(config.wifiSSID, config.wifiPassword);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected.");
}

String getHTML() {
  String html = "<!DOCTYPE html><html lang='en'><head><meta charset='UTF-8'><title>Device Configuration</title>"
                "<style>body{font-family: Arial, sans-serif; margin: 20px; background-color: #f4f4f4; color: #333;}"
                "h1{color: #0056b3;} .container{background-color: #fff; padding: 20px; border-radius: 8px; box-shadow: 0 0 10px 0 rgba(0,0,0,0.1);}"
                "label{margin-top: 10px;} input, button{margin-top: 5px;}</style></head><body>"
                "<div class='container'><h1>OWIPEX ESP32S3 Config Tool</h1>"
                "<form action='/save' method='post'><label>SSID:</label><input type='text' name='ssid' value='" + String(config.wifiSSID) + "' required>"
                "<label>Password:</label><input type='password' name='password' value='" + String(config.wifiPassword) + "' required>"
                "<label>Server:</label><input type='text' name='server' value='" + String(config.thingsboardServer) + "' required>"
                "<label>Token:</label><input type='text' name='token' value='" + String(config.token) + "' required>"
                "<label>Port:</label><input type='number' name='port' value='" + String(config.thingsboardPort) + "' required>"
                "<button type='submit'>Save</button></form>"
                "<p style='font-size: 0.8em; margin-top: 20px;'>Version V0.1<br>Powered by KARIM Technologies</p></div></body></html>";
  return html;
}

void saveConfig() {
  EEPROM.put(0, config);
  EEPROM.commit();
}