#include <Wire.h>
#include <WiFi.h>
#include <DallasTemperature.h>
//#define THINGSBOARD_ENABLE_PROGMEM 1
//#define THINGSBOARD_ENABLE_DYNAMIC 0
//#define THINGSBOARD_ENABLE_STREAM_UTILS 1
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include "I2CPressSens.h"


constexpr char WIFI_SSID[] = "FamMorbius";
constexpr char WIFI_PASSWORD[] = "45927194145938492747";
//constexpr char WIFI_SSID[] = "WiFi-Repeater2";
//constexpr char WIFI_PASSWORD[] = "111906111906";
constexpr char THINGSBOARD_SERVER[] = "46.101.195.97";
constexpr char TOKEN[] = "xN6NIu7y5ayOXn9Uff1f";
constexpr uint16_t THINGSBOARD_PORT = 1883U;
constexpr uint32_t MAX_MESSAGE_SIZE = 1024U;

// Globale Variablen
float temperature1 = 0.0; // in °C
float temperature2 = 0.0; // in °C
float pressureBar1 = 0.0; // Umrechnung in Bar
float pressureBar2 = 0.0; // Umrechnung in Bar
float mass = 0.0;

I2CPressSens pressSensor;
WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient); // Verwenden Sie Arduino_MQTT_Client mit WiFiClient
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE); // Passen Sie ThingsBoard an, um mqttClient zu verwenden


//Shared Attributes Declaration
constexpr char CO2_RELAY_ATTR[] = "co2Relay";
constexpr char HEATING_RELAY_ATTR[] = "heatingRelay";
constexpr char PUMP_RELAY_ATTR[] = "pumpRelais";



constexpr int CO2_RELAY_PIN = 1;
constexpr int HEATING_RELAY_PIN = 3;
constexpr int PUMP_RELAY_PIN = 1;

constexpr std::array<const char *, 3U> SHARED_ATTRIBUTES_LIST = {
  CO2_RELAY_ATTR,
  HEATING_RELAY_ATTR,
  PUMP_RELAY_ATTR
};

constexpr int16_t telemetrySendInterval = 3000U;
uint32_t previousDataSend;

bool subscribed = false;
bool heatingRelayControlledByTB = false; // Globale Variable

void processSharedAttributes(const Shared_Attribute_Data &data) {
  for (auto it = data.begin(); it != data.end(); ++it) {
    if (strcmp(it->key().c_str(), CO2_RELAY_ATTR) == 0) {
      digitalWrite(CO2_RELAY_PIN, it->value().as<bool>() ? HIGH : LOW);
    } 
    else if (strcmp(it->key().c_str(), HEATING_RELAY_ATTR) == 0) {
      bool heatingRelayState = it->value().as<bool>();
      digitalWrite(HEATING_RELAY_PIN, heatingRelayState ? HIGH : LOW);
      heatingRelayControlledByTB = heatingRelayState;
    }
    else if(strcmp(it->key().c_str(), PUMP_RELAY_ATTR) == 0) {
      digitalWrite(PUMP_RELAY_PIN, it->value().as<bool>() ? HIGH : LOW);
    }
  }
}

const Shared_Attribute_Callback attributes_callback(&processSharedAttributes, SHARED_ATTRIBUTES_LIST.cbegin(), SHARED_ATTRIBUTES_LIST.cend());

void InitWiFi() {
    Serial.println("Connecting to AP ...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("Connected to AP");
}

void reconnect() {
  // Versucht, die Verbindung wiederherzustellen, wenn sie verloren geht
  if (WiFi.status() != WL_CONNECTED) {
    InitWiFi();
  }
  if (!tb.connected()) {
    if (tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
      subscribed = false;
    }
  }
  if (!subscribed && tb.connected()) {
    if (tb.Shared_Attributes_Subscribe(attributes_callback)) {
      subscribed = true;
    }
  }
}



void setup() {
  Serial.begin(115200);
  pressSensor.begin(); // Initialisiert die I2C-Busse
  pinMode(CO2_RELAY_PIN, OUTPUT);
  pinMode(HEATING_RELAY_PIN, OUTPUT);
  digitalWrite(CO2_RELAY_PIN, false);
  digitalWrite(HEATING_RELAY_PIN, false);
  pinMode(PUMP_RELAY_PIN, OUTPUT);
  
  digitalWrite(PUMP_RELAY_PIN, false);
  

  delay(1000);
  InitWiFi();
}

void readAndProcessSensors() {
    // Lesen der Sensorwerte
    float pressureHigh = pressSensor.readPressureHigh() / 100.0; // Druck aus Sensor in Pascal umgerechnet auf Bar
    float pressureLow = pressSensor.readPressureLow() / 100.0;   // Druck aus Sensor in Pascal umgerechnet auf Bar
    float temperatureCelsius = 25.0; // Externe Temperatur in °C vom PH Sensor
    float volumeLiters = 10.0; // Volumen in Litern
    float mass = pressSensor.calculateMass(temperatureCelsius, volumeLiters);
    float temperatureHigh = pressSensor.readTemperatureHigh();
    float temperatureLow = pressSensor.readTemperatureLow();

    temperature1 = temperatureHigh; // in °C
    temperature2 = temperatureLow; // in °C
    pressureBar1 = pressureHigh; // Umrechnung in Bar
    pressureBar2 = pressureLow; // Umrechnung in Bar

}

void loop() {

    reconnect(); // Überprüft und stellt die Verbindung bei Bedarf wieder her

    // Lesen der Sensorwerte
    readAndProcessSensors();


    if (temperature2 < 35.0 && !heatingRelayControlledByTB) {
        digitalWrite(HEATING_RELAY_PIN, HIGH); // Heizer einschalten
    } else if (temperature2 >= 35.0 && !heatingRelayControlledByTB) {
        digitalWrite(HEATING_RELAY_PIN, LOW); // Heizer ausschalten
    }

    // Senden der Telemetriedaten
    if (millis() - previousDataSend > telemetrySendInterval) {
        previousDataSend = millis();
        // Senden der Druck- und Temperaturdaten
        tb.sendTelemetryData("pressureBar1", pressureBar1);
        tb.sendTelemetryData("temperature1", temperature1);
        tb.sendTelemetryData("pressureBar2", pressureBar2);
        tb.sendTelemetryData("temperature2", temperature2);
        tb.sendTelemetryData("CO2_MasseKG", mass);
        // Senden der WLAN-Signalstärke
        long rssi = WiFi.RSSI();
        tb.sendTelemetryData("wifiSignalStrength", rssi);

        //DEBUG
        Serial.print("Temperatur 1: ");
        Serial.println(temperature1);
        Serial.print("Temperatur 2: ");
        Serial.println(temperature2);
        Serial.print("Druck 1 (Bar): ");
        Serial.println(pressureBar1);
        Serial.print("Druck 2 (Bar): ");
        Serial.println(pressureBar2);
    }

  tb.loop();
}
