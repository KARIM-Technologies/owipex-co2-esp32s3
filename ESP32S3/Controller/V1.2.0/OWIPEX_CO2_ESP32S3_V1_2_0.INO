#include <Wire.h>
#include <WiFi.h>
#include <DallasTemperature.h>
//#define THINGSBOARD_ENABLE_PROGMEM 1
//#define THINGSBOARD_ENABLE_DYNAMIC 0
//#define THINGSBOARD_ENABLE_STREAM_UTILS 1
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include "I2CPressSens.h"
#include <Arduino.h>

/* Altes Board
#define LED_PIN_R 37
#define LED_PIN_G 36
#define LED_PIN_B 35
*/


// Neues Board
#define LED_PIN_R 37
#define LED_PIN_G 36
#define LED_PIN_B 35
//


//constexpr char WIFI_SSID[] = "FamMorbius";
//constexpr char WIFI_PASSWORD[] = "45927194145938492747";
constexpr char WIFI_SSID[] = "WB-NET_924_0007";
constexpr char WIFI_PASSWORD[] = "OWiP3X12";
constexpr char THINGSBOARD_SERVER[] = "192.168.100.100";
constexpr char TOKEN[] = "0kMbTKCfiFHZo05hsM5b";
constexpr uint16_t THINGSBOARD_PORT = 1883U;                                                                                                              
constexpr uint32_t MAX_MESSAGE_SIZE = 1024U;

// Setzen Sie Ihre statischen IP-Adresse Informationen hier
/* Wenn Master Gerät
// Setzen Sie Ihre statischen IP-Adresse Informationen hier
IPAddress local_IP(192, 168, 100, 99); // ESP32 statische IP
IPAddress gateway(192, 168, 100, 1); // Gateway-Adresse (normalerweise der Router)
IPAddress subnet(255, 255, 255, 0); // Subnetzmaske
*/
//Wenn Slave
IPAddress local_IP(192, 168, 100, 99); // ESP32 statische IP
IPAddress gateway(192, 168, 100, 1); // Gateway-Adresse (normalerweise der Router)
IPAddress subnet(255, 255, 255, 0); // Subnetzmaske
//


// Globale Variablen
float           temperature1        = 0.0; // in °C
float           temperature2        = 0.0; // in °C
float           pressureBar1        = 0.0; // Umrechnung in Bar
float           pressureBar2        = 0.0; // Umrechnung in Bar
float           mass                = 0.0; // berechnete Masse
bool            isBackupController  = false; //Ist ein Backup Modul
float           swPressMaster   = 0.0; //Value for lowest pressure of master before Switching.
bool            gasEmptySig     = false;
bool            gasValve        = false;
bool            co2SlaveRelaisSig  =false;
bool            co2SlaveHeaterSig  =false;
 

volatile float  phSensTemp      = 0.0;
volatile int    gasBottleSize = 10;
volatile bool   isEmptyMessage = false;





bool  serialDebug     = false;

I2CPressSens pressSensor;
WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient); // Verwenden Sie Arduino_MQTT_Client mit WiFiClient
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE); // Passen Sie ThingsBoard an, um mqttClient zu verwenden


//Shared Attributes Declaration
constexpr char CO2_RELAY_ATTR[] = "co2Relay";
constexpr char HEATING_RELAY_ATTR[] = "heatingRelay";
constexpr char PUMP_RELAY_ATTR[] = "pumpRelais";
constexpr char IS_BACKUP_CONTROLLER[] = "isBackupController"; //Module is a MASTER (yes/no)
constexpr char SWITCH_PRESS_MASTER[] = "swPressMaster"; //Value for lowest pressure of master before Switching.
constexpr char PH_S_TEMP_ATTR[] = "phSensTemp"; //Value from PH Sensor Temp Sensor
constexpr char GAS_BOTTEL_ATTR[] = "gasBottleSize"; // Gas Bottle Size






/*alte Platine
constexpr int CO2_RELAY_PIN = 12;
constexpr int HEATING_RELAY_PIN = 13;
constexpr int PUMP_RELAY_PIN = 14;
*/

//neue Platine
constexpr int CO2_RELAY_PIN = 10;
constexpr int HEATING_RELAY_PIN = 11;
constexpr int PUMP_RELAY_PIN = 12;
//
constexpr std::array<const char *, 7U> SHARED_ATTRIBUTES_LIST = {
    CO2_RELAY_ATTR,
    HEATING_RELAY_ATTR,
    PUMP_RELAY_ATTR,
    IS_BACKUP_CONTROLLER,
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

void setup() {
  Serial.begin(115200);
  pressSensor.begin(); // Initialisiert die I2C-Busse
  pinMode(CO2_RELAY_PIN, OUTPUT);
  pinMode(HEATING_RELAY_PIN, OUTPUT);
  digitalWrite(CO2_RELAY_PIN, false);
  digitalWrite(HEATING_RELAY_PIN, false);
  pinMode(PUMP_RELAY_PIN, OUTPUT);

  digitalWrite(PUMP_RELAY_PIN, false);
  pinMode(LED_PIN_R, OUTPUT);
  pinMode(LED_PIN_G, OUTPUT);
  pinMode(LED_PIN_B, OUTPUT);
  digitalWrite(LED_PIN_R, true);
  digitalWrite(LED_PIN_G, true);
  digitalWrite(LED_PIN_B, true);
  

  delay(1000);
  InitWiFi();
}

void processSharedAttributes(const Shared_Attribute_Data &data) {
    for (auto it = data.begin(); it != data.end(); ++it) {
        // Aktualisieren Sie isBackupController basierend auf dem Shared Attribute
        if (strcmp(it->key().c_str(), "isBackupController") == 0) {
            isBackupController = it->value().as<bool>();
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
        else if (strcmp(it->key().c_str(), CO2_RELAY_ATTR) == 0) {
            digitalWrite(CO2_RELAY_PIN, it->value().as<bool>() ? HIGH : LOW);
            //gasValve =  it->value().as<bool>();
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
    
    // Konfigurieren der festen IP-Adresse
    //IP Festschreuibe
    if (!WiFi.config(local_IP, gateway, subnet)) {
      Serial.println("STA Failed to configure");
    }
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        digitalWrite(LED_PIN_B, true);
        digitalWrite(LED_PIN_G, true);
        digitalWrite(LED_PIN_R, false);
        
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

  if (pressureBar1 <= swPressMaster){
    gasEmptySig     = true;
    co2SlaveRelaisSig = true;
    co2SlaveHeaterSig = true; 
    }
  else {
    gasEmptySig     = false;
    co2SlaveRelaisSig = false;
    co2SlaveHeaterSig = false; 
  }

  if (!gasEmptySig){
      digitalWrite(CO2_RELAY_PIN, gasValve);
    }
  else {
      digitalWrite(CO2_RELAY_PIN, false);
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
  if (WiFi.status() == !WL_CONNECTED) {
    InitWiFi();
    }
      
  if (WiFi.status() == WL_CONNECTED && !tb.connected()) {
      digitalWrite(LED_PIN_B, true);
      digitalWrite(LED_PIN_G, false);
      digitalWrite(LED_PIN_R, false);
    }

  if (WiFi.status() == WL_CONNECTED && tb.connected()) {
      digitalWrite(LED_PIN_B, false);
      digitalWrite(LED_PIN_G, true);
      digitalWrite(LED_PIN_R, true);
    
  }
  if (WiFi.status() == WL_CONNECTED && tb.connected() && subscribed) {
      digitalWrite(LED_PIN_B, true);
      digitalWrite(LED_PIN_G, false);
      digitalWrite(LED_PIN_R, true);
    }
    
    //reconnect(); // Überprüft und stellt die Verbindung bei Bedarf wieder her

    // Lesen der Sensorwerte
    readAndProcessSensors();

    if (temperature2 < 25.0 && !heatingRelayControlledByTB) {
        digitalWrite(HEATING_RELAY_PIN, HIGH); // Heizer einschalten
    } else if (temperature2 >= 25.0 && !heatingRelayControlledByTB) {
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
        tb.sendTelemetryData("gasEmptySig", gasEmptySig);
        tb.sendTelemetryData("co2SlaveRelaisSig", co2SlaveRelaisSig);
        tb.sendTelemetryData("co2SlaveHeaterSig", co2SlaveHeaterSig);
    }

    // Senden der Telemetriedaten SLOW
    if (millis() - previousDataSend_slow > telemetrySendInterval_slow) {
        previousDataSend_slow = millis();
        tb.sendAttributeData("rssi", WiFi.RSSI());
        tb.sendAttributeData("channel", WiFi.channel());
        tb.sendAttributeData("bssid", WiFi.BSSIDstr().c_str());
        tb.sendAttributeData("localIp", WiFi.localIP().toString().c_str());
        tb.sendAttributeData("ssid", WiFi.SSID().c_str());

        //DEBUG
        if (serialDebug == false){
          Serial.print("Temperatur 1: ");
          Serial.println(temperature1);
          Serial.print("Temperatur 2: ");
          Serial.println(temperature2);
          Serial.print("Druck 1 (Bar): ");
          Serial.println(pressureBar1);
          Serial.print("Druck 2 (Bar): ");
          Serial.println(pressureBar2);
          Serial.print("IS_BACKUP_CONTROLLER): ");
          Serial.println(isBackupController);
          Serial.print("Druck am Master): ");
          Serial.print("Midestdruck Master bevore Slave active): ");
          Serial.println(swPressMaster);
        }

    }

  tb.loop();
}
