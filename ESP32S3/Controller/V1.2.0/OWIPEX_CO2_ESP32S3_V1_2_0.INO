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
float           temperature1    = 0.0; // in °C
float           temperature2    = 0.0; // in °C
float           pressureBar1    = 0.0; // Umrechnung in Bar
float           pressureBar2    = 0.0; // Umrechnung in Bar
float           mass            = 0.0; // berechnete Masse
float           masterPressIs   = 99.9; //Druck am Master Module / sensed nur wenn MASTER True
bool            isMaster        = true; //Ist ein Master Module (true = Ja / false ist ein Slave)
float           swPressMaster   = 0.0; //Value for lowest pressure of master before Switching.

volatile float  phSensTemp      = 0.0;
volatile int    gasBottleSize = 10;

bool  serialDebug     = false;

I2CPressSens pressSensor;
WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient); // Verwenden Sie Arduino_MQTT_Client mit WiFiClient
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE); // Passen Sie ThingsBoard an, um mqttClient zu verwenden


//Shared Attributes Declaration
constexpr char CO2_RELAY_ATTR[] = "co2Relay";
constexpr char HEATING_RELAY_ATTR[] = "heatingRelay";
constexpr char PUMP_RELAY_ATTR[] = "pumpRelais";
constexpr char IS_MASTER[] = "isMaster"; //Module is a MASTER (yes/no)
constexpr char MASTER_PRESS[] = "masterPressIs"; //Pressure of Master
constexpr char SWITCH_PRESS_MASTER[] = "swPressMaster"; //Value for lowest pressure of master before Switching.
constexpr char PH_S_TEMP_ATTR[] = "phSensTemp"; //Value from PH Sensor Temp Sensor
constexpr char GAS_BOTTEL_ATTR[] = "gasBottleSize"; // Gas Bottle Size




constexpr int CO2_RELAY_PIN = 12;
constexpr int HEATING_RELAY_PIN = 13;
constexpr int PUMP_RELAY_PIN = 14;

constexpr std::array<const char *, 8U> SHARED_ATTRIBUTES_LIST = {
    CO2_RELAY_ATTR,
    HEATING_RELAY_ATTR,
    PUMP_RELAY_ATTR,
    IS_MASTER,
    MASTER_PRESS,
    SWITCH_PRESS_MASTER,
    PH_S_TEMP_ATTR,
    GAS_BOTTEL_ATTR

};



constexpr int16_t telemetrySendInterval_fast = 15000U;
constexpr int16_t telemetrySendInterval_slow = 30000U;
uint32_t previousDataSend_fast;
uint32_t previousDataSend_slow;


bool subscribed = false;
bool heatingRelayControlledByTB = false; // Globale Variable

void processSharedAttributes(const Shared_Attribute_Data &data) {
    for (auto it = data.begin(); it != data.end(); ++it) {
        // Aktualisieren Sie isMaster basierend auf dem Shared Attribute
        if (strcmp(it->key().c_str(), "isMaster") == 0) {
            isMaster = it->value().as<bool>();
        }
        // Aktualisieren Sie masterPressIs basierend auf dem Shared Attribute
        else if (strcmp(it->key().c_str(), "masterPressIs") == 0) {
            masterPressIs = it->value().as<float>();
        }
        // Aktualisieren Sie swPressMaster basierend auf dem Shared Attribute
        else if (strcmp(it->key().c_str(), "swPressMaster") == 0) {
            swPressMaster = it->value().as<float>();
        }
        // Aktualisieren Sie phSensTemp basierend auf dem Shared Attribute
        else if (strcmp(it->key().c_str(), "phSensTemp") == 0) {
            phSensTemp = it->value().as<float>();
        }
        // Aktualisieren Sie phSensTemp basierend auf dem Shared Attribute
        else if (strcmp(it->key().c_str(), "gasBottleSize") == 0) {
            gasBottleSize = it->value().as<int>();
        }
        // Verarbeitung für CO2_RELAY_ATTR
        else if (strcmp(it->key().c_str(), CO2_RELAY_ATTR) == 0) { 
          if (isMaster) {
            if (strcmp(it->key().c_str(), CO2_RELAY_ATTR) == 0) {
            digitalWrite(CO2_RELAY_PIN, it->value().as<bool>() ? HIGH : LOW);
            }
          }
        }
        // Verarbeitung für andere Shared Attributes
        else if (strcmp(it->key().c_str(), HEATING_RELAY_ATTR) == 0) {
            bool heatingRelayState = it->value().as<bool>();
            digitalWrite(HEATING_RELAY_PIN, heatingRelayState ? HIGH : LOW);
        }
        else if(strcmp(it->key().c_str(), PUMP_RELAY_ATTR) == 0) {
            digitalWrite(PUMP_RELAY_PIN, it->value().as<bool>() ? HIGH : LOW);
        }
    }
}

const Shared_Attribute_Callback attributes_callback(&processSharedAttributes, SHARED_ATTRIBUTES_LIST.cbegin(), SHARED_ATTRIBUTES_LIST.cend());
const Attribute_Request_Callback attribute_shared_request_callback(&processSharedAttributes, SHARED_ATTRIBUTES_LIST.cbegin(), SHARED_ATTRIBUTES_LIST.cend());

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
    float temperatureCelsius = phSensTemp; // Externe Temperatur in °C vom PH Sensor
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

    // Senden der Telemetriedaten FAST
    if (millis() - previousDataSend_fast > telemetrySendInterval_fast) {
        previousDataSend_fast = millis();
        // Senden der Druck- und Temperaturdaten
        tb.sendTelemetryData("H_press", pressureBar1);
        tb.sendTelemetryData("H_press_temp", temperature1);
        tb.sendTelemetryData("L_press", pressureBar2);
        tb.sendTelemetryData("L_press_temp", temperature2);
        tb.sendTelemetryData("CO2_MasseKG", mass);
        //Master Slave Vars
        tb.sendTelemetryData("is_Master", isMaster);
        if (isMaster){
          tb.sendTelemetryData("press_Master", pressureBar2);
          // Senden der WLAN-Signalstärke
        }
    }

    // Senden der Telemetriedaten SLOW
    if (millis() - previousDataSend_slow > telemetrySendInterval_slow) {
        previousDataSend_slow = millis();
        tb.sendTelemetryData("temperature", random(10, 20));
        tb.sendAttributeData("rssi", WiFi.RSSI());
        tb.sendAttributeData("channel", WiFi.channel());
        tb.sendAttributeData("bssid", WiFi.BSSIDstr().c_str());
        tb.sendAttributeData("localIp", WiFi.localIP().toString().c_str());
        tb.sendAttributeData("ssid", WiFi.SSID().c_str());

        //DEBUG
        if (serialDebug == true){
          Serial.print("Temperatur 1: ");
          Serial.println(temperature1);
          Serial.print("Temperatur 2: ");
          Serial.println(temperature2);
          Serial.print("Druck 1 (Bar): ");
          Serial.println(pressureBar1);
          Serial.print("Druck 2 (Bar): ");
          Serial.println(pressureBar2);
          Serial.print("is_Master): ");
          Serial.println(isMaster);
          Serial.print("Druck am Master): ");
          Serial.println(masterPressIs);
          Serial.print("Midestdruck Master bevore Slave active): ");
          Serial.println(swPressMaster);
        }

    }

  tb.loop();
}
