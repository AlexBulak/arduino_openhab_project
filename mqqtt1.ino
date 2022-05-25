/*OpenHAB MQTT device*/

//connection libraries
#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>
//device libraries
#include <Adafruit_BMP280.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <MQ135.h>
#include <DHT.h>

//WIFI Credentials
#define WIFI_SSID "TP-LINK_FD22"
#define WIFI_PASSWORD "10791325"

// Mosquitto MQTT Broker connection
#define MQTT_HOST IPAddress(192, 168, 0, 110)
// For a cloud MQTT broker
//#define MQTT_HOST "example.com"
#define MQTT_PORT 1883

//device's pins
#define BMP280_I2C_ADDRESS  0x76
#define ONE_WIRE_BUS D3
#define MQ135_WIRE_BUS A0
#define DHT_PIN D4
#define DHT_TYPE DHT11

// MQTT Topics
#define MQTT_PUB_TEMP "esp/bmp/temperature"
#define MQTT_PUB_PRESS "esp/bmp/pressure"
#define MQTT_PUB_DALLASTEMP "esp/DS18B20/tempereture"
#define MQTT_PUB_MQ135 "esp/mq135/CO2ppm"
#define MQTT_PUB_DHT_TEMP "esp/DHT11/temperature"
#define MQTT_PUB_DHT_HUM "esp/DHT11/humidity"

// device initialize

Adafruit_BMP280  bmp280;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
MQ135 gasSensor(MQ135_WIRE_BUS);
DHT dht(DHT_PIN, DHT_TYPE);

// Variables to hold sensor readings
DeviceAddress Thermometer;

float temp;
float pressure;
float dallasTemp;
float co2ppm;
float dhtTemp;
float dhtHum;

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

unsigned long previousMillis = 0;   // Stores last time message was published
const long interval = 10000;        // Interval at which to publish sensor readings

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  Serial.println("Connected to Wi-Fi.");
  connectToMqtt();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  Serial.println("Disconnected from Wi-Fi.");
  mqttReconnectTimer.detach(); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
  wifiReconnectTimer.once(2, connectToWifi);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
}

/*void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
  }

  void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  }*/

void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setup() {

  Serial.begin(115200);
  Serial.println();

  bmp280.begin(BMP280_I2C_ADDRESS);
  sensors.begin();
  dht.begin();

  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  //mqttClient.onSubscribe(onMqttSubscribe);
  //mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  // If your broker requires authentication (username and password), set them below
  //mqttClient.setCredentials("mqttuser", "1qaz!QAZ");

  connectToWifi();
}

void loop() {

  unsigned long currentMillis = millis();
  // Every X number of seconds (interval = 10 seconds)
  // it publishes a new MQTT message
  if (currentMillis - previousMillis >= interval) {
    // Save the last time a new reading was published
    previousMillis = currentMillis;

    // reading from sensors
    temp     = bmp280.readTemperature();   // get temperature
    pressure = bmp280.readPressure() / 100.0F;      // get pressure
    sensors.requestTemperatures();
    dallasTemp = sensors.getTempCByIndex(0);
    co2ppm = gasSensor.getPPM();
    dhtTemp = dht.readTemperature();
    dhtHum = dht.readHumidity();

    // Publish an MQTT message on topic
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_TEMP, 1, true, String(temp).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i ", MQTT_PUB_TEMP, packetIdPub1);
    Serial.printf("Message: %.2f \n", temp);

    uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_PRESS, 1, true, String(pressure).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_PRESS, packetIdPub2);
    Serial.printf("Message: %.2f \n", pressure);

    uint16_t packetIdPub3 = mqttClient.publish(MQTT_PUB_DALLASTEMP, 1, true, String(dallasTemp).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i ", MQTT_PUB_DALLASTEMP, packetIdPub3);
    Serial.printf("Message: %.2f \n", dallasTemp);

    uint16_t packetIdPub4 = mqttClient.publish(MQTT_PUB_MQ135, 1, true, String(co2ppm).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i ", MQTT_PUB_MQ135, packetIdPub4);
    Serial.printf("Message: %.2f \n", co2ppm);

    uint16_t packetIdPub5 = mqttClient.publish(MQTT_PUB_DHT_TEMP, 1, true, String(dhtTemp).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i ", MQTT_PUB_DHT_TEMP, packetIdPub5);
    Serial.printf("Message: %.2f \n", dhtTemp);

    uint16_t packetIdPub6 = mqttClient.publish(MQTT_PUB_DHT_HUM, 1, true, String(dhtHum).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i ", MQTT_PUB_DHT_HUM, packetIdPub6);
    Serial.printf("Message: %.2f \n", dhtHum);
  }
}
