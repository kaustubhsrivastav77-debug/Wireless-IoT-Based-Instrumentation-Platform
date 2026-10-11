// ================================================================
//            MAKERNOVA IoT - FINAL RECEIVER
// ================================================================
//
// FLOW:
//
// LoRa SX1278
//      |
//      v
// ESP32 Receiver
//    |        |
//    v        v
//   TFT      WiFi
//             |
//             v
//        Mosquitto MQTT
//             |
//             v
//        Web Dashboard
//
// ================================================================

#include <SPI.h>
#include <LoRa.h>

#include <WiFi.h>
#include <PubSubClient.h>

#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

#include <math.h>

// ================================================================
// WIFI
// ================================================================

const char* WIFI_SSID =
  "YOUR_WIFI_NAME";

const char* WIFI_PASSWORD =
  "YOUR_WIFI_PASSWORD";

// ================================================================
// MOSQUITTO
// ================================================================

const char* MQTT_SERVER =
  "YOUR_PC_IP";

const uint16_t MQTT_PORT =
  1883;

const char* MQTT_TOPIC =
  "makernova/sensor/data";

// ================================================================
// MQTT OBJECTS
// ================================================================

WiFiClient wifiClient;

PubSubClient mqttClient(
  wifiClient
);

// ================================================================
// SPI
// ================================================================

#define SPI_SCK  18
#define SPI_MISO 19
#define SPI_MOSI 23

// ================================================================
// LoRa SX1278
// ================================================================

#define LORA_CS    13
#define LORA_RST  -1
#define LORA_DIO0 -1

#define LORA_FREQ      433E6
#define LORA_SYNC_WORD 0x12

#define LORA_SF 7
#define LORA_BW 125E3
#define LORA_CR 5

// ================================================================
// TFT ILI9341
// ================================================================
//
// IMPORTANT:
//
// LoRa CS = GPIO13
// TFT CS  = GPIO27
//
// ================================================================

#define TFT_CS   27
#define TFT_DC   2
#define TFT_RST -1

Adafruit_ILI9341 tft(
  TFT_CS,
  TFT_DC,
  TFT_RST
);

// ================================================================
// SENSOR PACKET
// ================================================================
//
// MUST MATCH TRANSMITTER EXACTLY.
//
// ================================================================

struct __attribute__((packed)) SensorData
{
  uint16_t magic;

  uint8_t version;

  uint32_t packetNumber;

  float distance;

  float temperature;
  float humidity;
  float pressure;
  float altitude;

  float yaw;
  float pitch;
  float roll;

  uint16_t red;
  uint16_t green;
  uint16_t blue;
  uint16_t clear;

  uint16_t aqiRaw;

  float CO2;
  float NH3;
  float NOx;
  float alcohol;
  float benzene;
  float smoke;
};

SensorData data;

// ================================================================
// PACKET CONSTANTS
// ================================================================

const uint16_t PACKET_MAGIC =
  0xBEEF;

const uint8_t PACKET_VERSION =
  1;

// ================================================================
// STATE
// ================================================================

uint32_t receivedPackets = 0;

int lastRSSI = 0;

unsigned long lastWiFiAttempt = 0;

unsigned long lastMQTTAttempt = 0;

// ================================================================
// VALID FLOAT
// ================================================================

bool validFloat(
  float value
)
{
  return
    !isnan(value) &&
    !isinf(value);
}

// ================================================================
// START WIFI
// ================================================================

void startWiFi()
{
  Serial.println();

  Serial.print(
    "Connecting WiFi: "
  );

  Serial.println(
    WIFI_SSID
  );


  WiFi.mode(
    WIFI_STA
  );


  WiFi.begin(
    WIFI_SSID,
    WIFI_PASSWORD
  );
}

// ================================================================
// CONNECT MQTT
// ================================================================

void connectMQTT()
{
  if (
    WiFi.status() !=
    WL_CONNECTED
  )
  {
    return;
  }


  if (
    mqttClient.connected()
  )
  {
    return;
  }


  char clientID[40];


  uint64_t chipID =
    ESP.getEfuseMac();


  snprintf(
    clientID,
    sizeof(clientID),

    "MakerNova_RX_%04X",

    (uint16_t)(
      chipID &
      0xFFFF
    )
  );


  Serial.print(
    "Connecting Mosquitto..."
  );


  if (
    mqttClient.connect(
      clientID
    )
  )
  {
    Serial.println(
      "CONNECTED"
    );
  }
  else
  {
    Serial.print(
      "FAILED, state = "
    );

    Serial.println(
      mqttClient.state()
    );
  }
}

// ================================================================
// MAINTAIN NETWORK
// ================================================================

void maintainNetwork()
{
  // WiFi

  if (
    WiFi.status() !=
    WL_CONNECTED
  )
  {
    if (
      millis() -
      lastWiFiAttempt >=
      5000
    )
    {
      lastWiFiAttempt =
        millis();


      Serial.println(
        "WiFi reconnecting..."
      );


      WiFi.disconnect();


      WiFi.begin(
        WIFI_SSID,
        WIFI_PASSWORD
      );
    }


    return;
  }


  // MQTT

  if (
    !mqttClient.connected()
  )
  {
    if (
      millis() -
      lastMQTTAttempt >=
      3000
    )
    {
      lastMQTTAttempt =
        millis();

      connectMQTT();
    }
  }
  else
  {
    mqttClient.loop();
  }
}

// ================================================================
// ADD FLOAT TO JSON
//
// FIXED FOR ESP32 CORE 3.3.11.
//
// Does NOT use:
// String(float, uint8_t)
//
// ================================================================

void appendFloatJSON(
  String& payload,
  const char* key,
  float value,
  int decimals,
  bool comma
)
{
  payload += "\"";

  payload += key;

  payload += "\":";


  if (
    validFloat(value)
  )
  {
    char numberBuffer[32];


    snprintf(
      numberBuffer,
      sizeof(numberBuffer),
      "%.*f",
      decimals,
      (double)value
    );


    payload +=
      numberBuffer;
  }
  else
  {
    payload +=
      "null";
  }


  if (comma)
  {
    payload += ",";
  }
}

// ================================================================
// MQTT PUBLISH
// ================================================================

void publishMQTT()
{
  if (
    !mqttClient.connected()
  )
  {
    Serial.println(
      "MQTT offline"
    );

    return;
  }


  String payload;

  payload.reserve(
    700
  );


  payload =
    "{";


  // BME280

  appendFloatJSON(
    payload,
    "Temperature",
    data.temperature,
    2,
    true
  );


  appendFloatJSON(
    payload,
    "Humidity",
    data.humidity,
    2,
    true
  );


  appendFloatJSON(
    payload,
    "Pressure",
    data.pressure,
    2,
    true
  );


  appendFloatJSON(
    payload,
    "Altitude",
    data.altitude,
    2,
    true
  );


  // HC-SR04

  appendFloatJSON(
    payload,
    "Distance",
    data.distance,
    2,
    true
  );


  // MQ135 raw ADC
  //
  // Dashboard field is named AQI.
  // This is the raw MQ135 ADC value.

  payload +=
    "\"AQI\":";

  payload +=
    String(
      data.aqiRaw
    );

  payload +=
    ",";


  // MQ135 gases

  appendFloatJSON(
    payload,
    "CO2",
    data.CO2,
    2,
    true
  );


  appendFloatJSON(
    payload,
    "NH3",
    data.NH3,
    2,
    true
  );


  appendFloatJSON(
    payload,
    "NOx",
    data.NOx,
    2,
    true
  );


  appendFloatJSON(
    payload,
    "Alcohol",
    data.alcohol,
    2,
    true
  );


  appendFloatJSON(
    payload,
    "Benzene",
    data.benzene,
    2,
    true
  );


  appendFloatJSON(
    payload,
    "Smoke",
    data.smoke,
    2,
    true
  );


  // TCS34725

  payload +=
    "\"Color_Red\":";

  payload +=
    String(
      data.red
    );

  payload +=
    ",";


  payload +=
    "\"Color_Green\":";

  payload +=
    String(
      data.green
    );

  payload +=
    ",";


  payload +=
    "\"Color_Blue\":";

  payload +=
    String(
      data.blue
    );

  payload +=
    ",";


  // BNO085

  appendFloatJSON(
    payload,
    "Yaw",
    data.yaw,
    2,
    true
  );


  appendFloatJSON(
    payload,
    "Pitch",
    data.pitch,
    2,
    true
  );


  appendFloatJSON(
    payload,
    "Roll",
    data.roll,
    2,
    false
  );


  payload +=
    "}";


  Serial.println();
  Serial.println(
    "MQTT JSON:"
  );

  Serial.println(
    payload
  );


  bool published =
    mqttClient.publish(
      MQTT_TOPIC,
      payload.c_str(),
      true
    );


  if (published)
  {
    Serial.println(
      "MQTT -> PUBLISHED"
    );
  }
  else
  {
    Serial.println(
      "MQTT -> PUBLISH FAILED"
    );
  }
}

// ================================================================
// TFT FLOAT
// ================================================================

void printTFTFloat(
  float value,
  uint8_t decimals
)
{
  if (
    validFloat(value)
  )
  {
    tft.print(
      value,
      decimals
    );
  }
  else
  {
    tft.print(
      "--"
    );
  }
}

// ================================================================
// TFT DISPLAY
// ================================================================

void displaySensorData()
{
  tft.fillScreen(
    ILI9341_BLACK
  );


  // ------------------------------------------------
  // TITLE
  // ------------------------------------------------

  tft.setTextSize(
    2
  );

  tft.setTextColor(
    ILI9341_CYAN
  );

  tft.setCursor(
    5,
    5
  );

  tft.println(
    "MakerNova IoT"
  );


  tft.setTextSize(
    1
  );


  tft.setTextColor(
    ILI9341_WHITE
  );


  tft.setCursor(
    5,
    28
  );


  tft.print(
    "Packet: "
  );

  tft.print(
    data.packetNumber
  );


  tft.print(
    "  RSSI: "
  );

  tft.print(
    lastRSSI
  );

  tft.println(
    " dBm"
  );


  // ------------------------------------------------
  // BME280
  // ------------------------------------------------

  tft.setTextColor(
    ILI9341_YELLOW
  );


  tft.setCursor(
    5,
    50
  );


  tft.println(
    "BME280"
  );


  tft.setTextColor(
    ILI9341_WHITE
  );


  tft.setCursor(
    5,
    64
  );

  tft.print(
    "Temp: "
  );

  printTFTFloat(
    data.temperature,
    1
  );

  tft.println(
    " C"
  );


  tft.setCursor(
    5,
    77
  );

  tft.print(
    "Hum : "
  );

  printTFTFloat(
    data.humidity,
    1
  );

  tft.println(
    " %"
  );


  tft.setCursor(
    5,
    90
  );

  tft.print(
    "Pres: "
  );

  printTFTFloat(
    data.pressure,
    1
  );

  tft.println(
    " hPa"
  );


  tft.setCursor(
    5,
    103
  );

  tft.print(
    "Alt : "
  );

  printTFTFloat(
    data.altitude,
    1
  );

  tft.println(
    " m"
  );


  // ------------------------------------------------
  // HC-SR04
  // ------------------------------------------------

  tft.setTextColor(
    ILI9341_GREEN
  );


  tft.setCursor(
    5,
    124
  );


  tft.println(
    "HC-SR04"
  );


  tft.setTextColor(
    ILI9341_WHITE
  );


  tft.setCursor(
    5,
    138
  );


  tft.print(
    "Distance: "
  );


  printTFTFloat(
    data.distance,
    1
  );


  tft.println(
    " cm"
  );


  // ------------------------------------------------
  // BNO085
  // ------------------------------------------------

  tft.setTextColor(
    ILI9341_MAGENTA
  );


  tft.setCursor(
    5,
    160
  );


  tft.println(
    "BNO085"
  );


  tft.setTextColor(
    ILI9341_WHITE
  );


  tft.setCursor(
    5,
    174
  );


  tft.print(
    "Yaw  : "
  );


  printTFTFloat(
    data.yaw,
    1
  );


  tft.setCursor(
    5,
    187
  );


  tft.print(
    "Pitch: "
  );


  printTFTFloat(
    data.pitch,
    1
  );


  tft.setCursor(
    5,
    200
  );


  tft.print(
    "Roll : "
  );


  printTFTFloat(
    data.roll,
    1
  );


  // ------------------------------------------------
  // TCS34725
  // ------------------------------------------------

  tft.setTextColor(
    ILI9341_CYAN
  );


  tft.setCursor(
    170,
    50
  );


  tft.println(
    "TCS34725"
  );


  tft.setTextColor(
    ILI9341_WHITE
  );


  tft.setCursor(
    170,
    64
  );


  tft.print(
    "R: "
  );


  tft.println(
    data.red
  );


  tft.setCursor(
    170,
    77
  );


  tft.print(
    "G: "
  );


  tft.println(
    data.green
  );


  tft.setCursor(
    170,
    90
  );


  tft.print(
    "B: "
  );


  tft.println(
    data.blue
  );


  // ------------------------------------------------
  // MQ135
  // ------------------------------------------------

  tft.setTextColor(
    ILI9341_RED
  );


  tft.setCursor(
    170,
    115
  );


  tft.println(
    "MQ135"
  );


  tft.setTextColor(
    ILI9341_WHITE
  );


  tft.setCursor(
    170,
    129
  );


  tft.print(
    "Raw: "
  );


  tft.println(
    data.aqiRaw
  );


  tft.setCursor(
    170,
    142
  );


  tft.print(
    "CO2: "
  );


  printTFTFloat(
    data.CO2,
    1
  );


  tft.setCursor(
    170,
    155
  );


  tft.print(
    "NH3: "
  );


  printTFTFloat(
    data.NH3,
    1
  );


  tft.setCursor(
    170,
    168
  );


  tft.print(
    "Alcohol: "
  );


  printTFTFloat(
    data.alcohol,
    1
  );


  // ------------------------------------------------
  // MQTT
  // ------------------------------------------------

  tft.setCursor(
    170,
    202
  );


  if (
    mqttClient.connected()
  )
  {
    tft.setTextColor(
      ILI9341_GREEN
    );


    tft.print(
      "MQTT: ONLINE"
    );
  }
  else
  {
    tft.setTextColor(
      ILI9341_RED
    );


    tft.print(
      "MQTT: OFFLINE"
    );
  }
}

// ================================================================
// SERIAL OUTPUT
// ================================================================

void printReceivedData()
{
  Serial.println();
  Serial.println(
    "======================================"
  );

  Serial.println(
    "       VALID LORA PACKET"
  );

  Serial.println(
    "======================================"
  );


  Serial.print(
    "Packet      : "
  );

  Serial.println(
    data.packetNumber
  );


  Serial.print(
    "RSSI        : "
  );

  Serial.print(
    lastRSSI
  );

  Serial.println(
    " dBm"
  );


  Serial.print(
    "Temperature : "
  );

  Serial.println(
    data.temperature,
    2
  );


  Serial.print(
    "Humidity    : "
  );

  Serial.println(
    data.humidity,
    2
  );


  Serial.print(
    "Pressure    : "
  );

  Serial.println(
    data.pressure,
    2
  );


  Serial.print(
    "Altitude    : "
  );

  Serial.println(
    data.altitude,
    2
  );


  Serial.print(
    "Distance    : "
  );

  Serial.println(
    data.distance,
    2
  );


  Serial.print(
    "Yaw         : "
  );

  Serial.println(
    data.yaw,
    2
  );


  Serial.print(
    "Pitch       : "
  );

  Serial.println(
    data.pitch,
    2
  );


  Serial.print(
    "Roll        : "
  );

  Serial.println(
    data.roll,
    2
  );


  Serial.print(
    "RGB         : "
  );

  Serial.print(
    data.red
  );

  Serial.print(
    ", "
  );

  Serial.print(
    data.green
  );

  Serial.print(
    ", "
  );

  Serial.println(
    data.blue
  );


  Serial.print(
    "MQ135 Raw   : "
  );

  Serial.println(
    data.aqiRaw
  );


  Serial.println(
    "======================================"
  );
}

// ================================================================
// RECEIVE LORA
// ================================================================

void receiveLoRa()
{
  int packetSize =
    LoRa.parsePacket();


  if (
    packetSize <= 0
  )
  {
    return;
  }


  Serial.println();
  Serial.println(
    "LoRa -> PACKET DETECTED"
  );


  if (
    packetSize !=
    (int)sizeof(SensorData)
  )
  {
    Serial.print(
      "Invalid packet size: "
    );

    Serial.print(
      packetSize
    );

    Serial.print(
      " expected: "
    );

    Serial.println(
      sizeof(SensorData)
    );


    while (
      LoRa.available()
    )
    {
      LoRa.read();
    }


    return;
  }


  size_t bytesRead =
    LoRa.readBytes(
      (uint8_t*)&data,
      sizeof(data)
    );


  if (
    bytesRead !=
    sizeof(data)
  )
  {
    Serial.println(
      "Incomplete packet"
    );

    return;
  }


  if (
    data.magic !=
    PACKET_MAGIC
  )
  {
    Serial.println(
      "Invalid magic number"
    );

    return;
  }


  if (
    data.version !=
    PACKET_VERSION
  )
  {
    Serial.println(
      "Invalid packet version"
    );

    return;
  }


  lastRSSI =
    LoRa.packetRssi();


  receivedPackets++;


  Serial.println(
    "LoRa -> VALID PACKET"
  );


  printReceivedData();


  displaySensorData();


  if (
    mqttClient.connected()
  )
  {
    publishMQTT();
  }
  else
  {
    Serial.println(
      "MQTT offline - data shown only on TFT"
    );
  }
}

// ================================================================
// SETUP
// ================================================================

void setup()
{
  Serial.begin(
    115200
  );


  delay(
    1000
  );


  Serial.println();
  Serial.println(
    "======================================"
  );

  Serial.println(
    "       MAKERNOVA RECEIVER START"
  );

  Serial.println(
    "======================================"
  );


  // ------------------------------------------------
  // CHIP SELECTS
  // ------------------------------------------------

  pinMode(
    LORA_CS,
    OUTPUT
  );


  pinMode(
    TFT_CS,
    OUTPUT
  );


  digitalWrite(
    LORA_CS,
    HIGH
  );


  digitalWrite(
    TFT_CS,
    HIGH
  );


  // ------------------------------------------------
  // SHARED SPI
  // ------------------------------------------------

  SPI.begin(
    SPI_SCK,
    SPI_MISO,
    SPI_MOSI
  );


  // ------------------------------------------------
  // TFT
  // ------------------------------------------------

  Serial.println(
    "Initializing TFT..."
  );


  tft.begin();


  tft.setRotation(
    1
  );


  tft.fillScreen(
    ILI9341_BLACK
  );


  tft.setTextColor(
    ILI9341_GREEN
  );


  tft.setTextSize(
    2
  );


  tft.setCursor(
    20,
    70
  );


  tft.println(
    "MakerNova IoT"
  );


  tft.setTextSize(
    1
  );


  tft.setTextColor(
    ILI9341_WHITE
  );


  tft.setCursor(
    20,
    105
  );


  tft.println(
    "Waiting for LoRa..."
  );


  // ------------------------------------------------
  // LORA
  // ------------------------------------------------

  LoRa.setPins(
    LORA_CS,
    LORA_RST,
    LORA_DIO0
  );


  if (
    !LoRa.begin(
      LORA_FREQ
    )
  )
  {
    Serial.println(
      "LoRa -> NOT FOUND"
    );


    tft.setTextColor(
      ILI9341_RED
    );


    tft.setCursor(
      20,
      130
    );


    tft.println(
      "LoRa ERROR"
    );


    while (true)
    {
      delay(
        1000
      );
    }
  }


  LoRa.setSyncWord(
    LORA_SYNC_WORD
  );


  LoRa.setSpreadingFactor(
    LORA_SF
  );


  LoRa.setSignalBandwidth(
    LORA_BW
  );


  LoRa.setCodingRate4(
    LORA_CR
  );


  LoRa.enableCrc();


  Serial.println(
    "LoRa -> READY"
  );


  Serial.print(
    "Expected packet size: "
  );


  Serial.print(
    sizeof(SensorData)
  );


  Serial.println(
    " bytes"
  );


  // ------------------------------------------------
  // MQTT
  // ------------------------------------------------

  mqttClient.setServer(
    MQTT_SERVER,
    MQTT_PORT
  );


  mqttClient.setBufferSize(
    1024
  );


  mqttClient.setKeepAlive(
    30
  );


  // ------------------------------------------------
  // WIFI
  // ------------------------------------------------

  startWiFi();


  Serial.println();
  Serial.println(
    "RECEIVER READY"
  );
}

// LOOP

void loop()
{
  maintainNetwork();


  receiveLoRa();
}
