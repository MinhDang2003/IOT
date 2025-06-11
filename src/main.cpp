#include <WiFi.h>
#include <Arduino_MQTT_Client.h>

#include <ThingsBoard.h>
#include "DHT20.h"
#include "Wire.h"
#include <Adafruit_NeoPixel.h>

#define LED_PIN 48
#define FAN_LED_PIN 47
#define LIGHT_SENSOR_PIN 34

#define SDA_PIN GPIO_NUM_11
#define SCL_PIN GPIO_NUM_12

constexpr char WIFI_SSID[] = "Minh_S24";
constexpr char WIFI_PASSWORD[] = "123456789";


constexpr char TOKEN[] = "a83a4bd0-3d31-11f0-aae0-0f85903b3644";
// constexpr char PASSWORD[] = "z3mldzslardtk4udglcq";
// constexpr char CLIENT_ID[] = "uj83jghmzc11xx5etlce";

constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;

constexpr uint32_t MAX_MESSAGE_SIZE = 1024U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;

constexpr char LED_STATE_ATTR[] = "ledState";
constexpr char FAN_SPEED_ATTR[] = "fanSpeed";

volatile bool attributesChanged = false;
volatile bool ledState = false;
volatile int fanSpeedPercent = 0;  // 0 - 100

constexpr uint32_t telemetrySendInterval = 10000U;
uint32_t previousDataSend = 0;

WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);

DHT20 dht20;
Adafruit_NeoPixel rgb(4, 3, NEO_GRB + NEO_KHZ800);

// Fan PWM config
#define FAN_PWM_CHANNEL 0
#define FAN_FREQUENCY 5000
#define FAN_RESOLUTION 8

// ---------- RPC HANDLERS ----------

RPC_Response toggleLed(const RPC_Data &data) {
  ledState = !ledState;
  digitalWrite(LED_PIN, ledState);
  attributesChanged = true;
  return RPC_Response("toggleLed", ledState);
}

RPC_Response setFanSpeed(const RPC_Data &data) {
  if (!data.isNull()) return RPC_Response("setFanSpeed", "Invalid type");

  fanSpeedPercent = constrain((int)data, 0, 100);
  int pwmValue = map(fanSpeedPercent, 0, 100, 0, 255);
  ledcWrite(FAN_PWM_CHANNEL, pwmValue);

  Serial.printf("Set fan to %d%% (PWM = %d)\n", fanSpeedPercent, pwmValue);
  attributesChanged = true;
  return RPC_Response("setFanSpeed", fanSpeedPercent);
}

const std::array<RPC_Callback, 2U> callbacks = {
  RPC_Callback{"toggleLed", toggleLed},
  RPC_Callback{"setFanSpeed", setFanSpeed}
};

// ---------- ATTRIBUTE HANDLER ----------

void processSharedAttributes(const Shared_Attribute_Data &data) {
  for (auto it = data.begin(); it != data.end(); ++it) {
    if (strcmp(it->key().c_str(), LED_STATE_ATTR) == 0) {
      ledState = it->value().as<bool>();
      digitalWrite(LED_PIN, ledState);
      Serial.printf("LED state updated to %d\n", ledState);
    } else if (strcmp(it->key().c_str(), FAN_SPEED_ATTR) == 0) {
      fanSpeedPercent = constrain(it->value().as<int>(), 0, 100);
      int pwmValue = map(fanSpeedPercent, 0, 100, 0, 255);
      ledcWrite(FAN_PWM_CHANNEL, pwmValue);
      Serial.printf("Fan speed updated to %d%% (PWM = %d)\n", fanSpeedPercent, pwmValue);
    }
  }
  attributesChanged = true;
}

constexpr std::array<const char *, 2U> SHARED_ATTRIBUTES_LIST = {
  LED_STATE_ATTR,
  FAN_SPEED_ATTR
};

const Shared_Attribute_Callback attributes_callback(
  &processSharedAttributes,
  SHARED_ATTRIBUTES_LIST.cbegin(),
  SHARED_ATTRIBUTES_LIST.cend()
);

const Attribute_Request_Callback attribute_shared_request_callback(
  &processSharedAttributes,
  SHARED_ATTRIBUTES_LIST.cbegin(),
  SHARED_ATTRIBUTES_LIST.cend()
);

// ---------- WIFI ----------

void InitWiFi() {
  Serial.println("Connecting to AP ...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
}

bool reconnect() {
  if (WiFi.status() == WL_CONNECTED) return true;
  InitWiFi();
  return true;
}

// ---------- SETUP ----------

void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD);
  rgb.begin();

  pinMode(LED_PIN, OUTPUT);
  pinMode(LIGHT_SENSOR_PIN, INPUT);
  InitWiFi();

  Wire.begin(SDA_PIN, SCL_PIN);
  dht20.begin();

  // Setup fan PWM
  ledcSetup(FAN_PWM_CHANNEL, FAN_FREQUENCY, FAN_RESOLUTION);
  ledcAttachPin(FAN_LED_PIN, FAN_PWM_CHANNEL);
  ledcWrite(FAN_PWM_CHANNEL, map(fanSpeedPercent, 0, 100, 0, 255));
}

// ---------- LOOP ----------

void loop() {
  delay(10);

  if (!reconnect()) return;

  if (!tb.connected()) {
    Serial.print("Connecting to: ");
    Serial.println(THINGSBOARD_SERVER);
    if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
      Serial.println("Failed to connect");
      return;
    }

    tb.sendAttributeData("macAddress", WiFi.macAddress().c_str());

    if (!tb.RPC_Subscribe(callbacks.cbegin(), callbacks.cend())) {
      Serial.println("RPC subscription failed");
      return;
    }

    if (!tb.Shared_Attributes_Subscribe(attributes_callback)) {
      Serial.println("Attribute subscription failed");
      return;
    }

    if (!tb.Shared_Attributes_Request(attribute_shared_request_callback)) {
      Serial.println("Attribute request failed");
      return;
    }

    Serial.println("Connected and subscribed to ThingsBoard.");
  }

  if (attributesChanged) {
    attributesChanged = false;
    tb.sendAttributeData(LED_STATE_ATTR, digitalRead(LED_PIN));
    tb.sendAttributeData(FAN_SPEED_ATTR, fanSpeedPercent);
  }

  if (millis() - previousDataSend > telemetrySendInterval) {
    previousDataSend = millis();

    dht20.read();
    float temperature = dht20.getTemperature();
    float humidity = dht20.getHumidity();
    int light = analogRead(LIGHT_SENSOR_PIN);

    if (!isnan(temperature) && !isnan(humidity)) {
      tb.sendTelemetryData("temperature", temperature);
      tb.sendTelemetryData("humidity", humidity);
    } else {
      Serial.println("Failed to read DHT20");
    }

    tb.sendTelemetryData("light", light);
    tb.sendAttributeData("rssi", WiFi.RSSI());
    tb.sendAttributeData("localIp", WiFi.localIP().toString().c_str());
    tb.sendAttributeData("ssid", WiFi.SSID().c_str());
  }

  tb.loop();
}
