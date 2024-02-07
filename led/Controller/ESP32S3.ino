#include <Wire.h>
#include <WiFi.h>
#include <DallasTemperature.h>
//#define THINGSBOARD_ENABLE_PROGMEM 1
//#define THINGSBOARD_ENABLE_DYNAMIC 0
//#define THINGSBOARD_ENABLE_STREAM_UTILS 1
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>

constexpr char WIFI_SSID[] = "FamMorbius";
constexpr char WIFI_PASSWORD[] = "45927194145938492747";
//constexpr char WIFI_SSID[] = "WiFi-Repeater2";
//constexpr char WIFI_PASSWORD[] = "111906111906";
constexpr char THINGSBOARD_SERVER[] = "46.101.195.97";
constexpr char TOKEN[] = "xN6NIu7y5ayOXn9Uff1f";


constexpr uint16_t THINGSBOARD_PORT = 1883U;
constexpr uint32_t MAX_MESSAGE_SIZE = 1024U;

constexpr char CO2_RELAY_ATTR[] = "co2Relay";
constexpr char HEATING_RELAY_ATTR[] = "heatingRelay";
constexpr char PUMP_RELAY_ATTR[] = "pumpRelais";


constexpr int CO2_RELAY_PIN = 1;
constexpr int HEATING_RELAY_PIN = 3;
constexpr int PUMP_RELAY_PIN = 1;
constexpr int SEC_RELAY_PIN = 2;


constexpr std::array<const char *, 3U> SHARED_ATTRIBUTES_LIST = {
  CO2_RELAY_ATTR,
  HEATING_RELAY_ATTR,
  PUMP_RELAY_ATTR
};

WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient); // Verwenden Sie Arduino_MQTT_Client mit WiFiClient
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE); // Passen Sie ThingsBoard an, um mqttClient zu verwenden


constexpr int16_t telemetrySendInterval = 3000U;
uint32_t previousDataSend;

bool subscribed = false;
bool heatingRelayControlledByTB = false; // Globale Variable

TwoWire Sensor1Wire = TwoWire(0);
TwoWire Sensor2Wire = TwoWire(1);

#define DEVICE_ADDRESS 0x6D
#define PRESSURE_RANGE 500.0 // kPa
#define VOLUME 10.0 // Liter
#define R 8.314 // J/(mol·K)
#define M 0.04401 // kg/mol

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
  Sensor1Wire.begin(5, 4);
  Sensor2Wire.begin(7, 6);
  Serial.begin(115200);
  pinMode(CO2_RELAY_PIN, OUTPUT);
  pinMode(HEATING_RELAY_PIN, OUTPUT);
  digitalWrite(CO2_RELAY_PIN, false);
  digitalWrite(HEATING_RELAY_PIN, false);
  pinMode(PUMP_RELAY_PIN, OUTPUT);
  pinMode(SEC_RELAY_PIN, OUTPUT);
  digitalWrite(PUMP_RELAY_PIN, false);
  digitalWrite(SEC_RELAY_PIN, false);

  delay(1000);
  InitWiFi();
}

void loop() {
  reconnect(); // Überprüft und stellt die Verbindung bei Bedarf wieder her

  // Lesen der Sensorwerte
  float pressure1 = readPressure(Sensor1Wire); // in kPa
  float temperature1 = readTemperature(Sensor1Wire); // in °C
  float pressure2 = readPressure(Sensor2Wire); // in kPa
  float temperature2 = readTemperature(Sensor2Wire); // in °C

  float pressureBar1 = pressure1 / 100.0; // Umrechnung in Bar
  float mass1 = calculateMass(pressure1, temperature1, VOLUME); // Berechnung der Masse in kg
  float pressureBar2 = pressure2 / 100.0; // Umrechnung in Bar
  
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
    tb.sendTelemetryData("CO2_MasseKG", mass1);

    // Senden der WLAN-Signalstärke
    long rssi = WiFi.RSSI();
    tb.sendTelemetryData("wifiSignalStrength", rssi);
  }

  tb.loop();
}

float readPressure(TwoWire &wire) {
  wire.beginTransmission(DEVICE_ADDRESS);
  wire.write(0x06);
  wire.endTransmission(false);

  wire.requestFrom(DEVICE_ADDRESS, 3);

  if (wire.available() == 3) {
    long dat = wire.read() << 16 | wire.read() << 8 | wire.read();
    if (dat & 0x800000) {
      dat = dat - 16777216;
    }
    float fadc = dat;
    float adc = 3.3 * fadc / 8388608.0;
    float pressure = PRESSURE_RANGE * (adc - 0.5) / 2.0;
    return pressure;
  }
  return NAN;
}

float readTemperature(TwoWire &wire) {
  wire.beginTransmission(DEVICE_ADDRESS);
  wire.write(0x09);
  wire.endTransmission(false);

  wire.requestFrom(DEVICE_ADDRESS, 3);

  if (wire.available() == 3) {
    long dat = wire.read() << 16 | wire.read() << 8 | wire.read();
    if (dat & 0x800000) {
      dat = dat - 16777216;
    }
    float fadc = dat;
    float temperature = 25.0 + fadc / 65536.0;
    return temperature;
  }
  return NAN;
}

float calculateMass(float pressure, float temperature, float volume) {
  float pressurePa = pressure * 1000.0; // Umrechnung in Pa
  float temperatureK = temperature + 273.15; // Umrechnung in Kelvin
  float volumeM3 = volume / 1000.0; // Umrechnung von Liter in m³
  
  float mass = (pressurePa * volumeM3 * M) / (R * temperatureK);
  return mass;
}
