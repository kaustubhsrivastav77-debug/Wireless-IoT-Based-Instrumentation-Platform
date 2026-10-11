// ================================================================
//        MAKERNOVA IoT - FINAL TRANSMITTER - LONG RANGE
// ================================================================

#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>

#include <Adafruit_BNO08x.h>
#include <Adafruit_BME280.h>
#include <Adafruit_TCS34725.h>

#include <math.h>

// ================================================================
// PIN CONFIGURATION
// ================================================================

#define I2C_SDA 21
#define I2C_SCL 22

#define SPI_SCK   18
#define SPI_MISO  19
#define SPI_MOSI  23

// LoRa
#define LORA_CS    5
#define LORA_RST  -1
#define LORA_DIO0 -1

#define LORA_FREQ      433E6
#define LORA_SYNC_WORD 0x12

// LONG RANGE SETTINGS
#define LORA_SF       10
#define LORA_BW       125E3
#define LORA_CR       5
#define LORA_TX_POWER 17

// HC-SR04
#define TRIG_PIN 25
#define ECHO_PIN 34

// MQ135
#define MQ135_PIN 27

// ================================================================
// MQ135 SETTINGS
// ================================================================

#define ADC_MAX_VALUE 4095.0f
#define ADC_VOLTAGE   3.3f

#define MQ_VOLTAGE    5.0f
#define RL_VALUE      10.0f
#define R0_VALUE      62.326f

#define DIVIDER_RATIO (15.0f / 25.0f)

// ================================================================
// SENSOR OBJECTS
// ================================================================

Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

Adafruit_BME280 bme;

Adafruit_TCS34725 tcs(
  TCS34725_INTEGRATIONTIME_50MS,
  TCS34725_GAIN_4X
);

// ================================================================
// SENSOR STATUS
// ================================================================

bool bnoAvailable = false;
bool bmeAvailable = false;
bool tcsAvailable = false;
bool loraAvailable = false;

// ================================================================
// BNO085 VALUES
// ================================================================

float yaw = 0.0f;
float pitch = 0.0f;
float roll = 0.0f;

// ================================================================
// SENSOR PACKET
// ================================================================
// MUST MATCH RECEIVER EXACTLY
// PACKET SIZE = 78 BYTES
// ================================================================

struct __attribute__((packed)) SensorData
{
  uint16_t magic;

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

  float lux;
  float colorTemperature;

  float CO;
  float alcohol;
  float CO2;
  float toluene;
  float NH4;
  float acetone;
};

SensorData data = {0};

// ================================================================
// PACKET SETTINGS
// ================================================================

uint32_t packetCounter = 0;

unsigned long lastSendTime = 0;

// 2 seconds recommended for SF10
const unsigned long SEND_INTERVAL = 2000;

// ================================================================
// BNO085 QUATERNION -> EULER
// ================================================================

void quaternionToEuler(
  float qw,
  float qx,
  float qy,
  float qz,
  float &y,
  float &p,
  float &r
)
{
  float sinr_cosp =
    2.0f * (qw * qx + qy * qz);

  float cosr_cosp =
    1.0f - 2.0f * (qx * qx + qy * qy);

  r = atan2(
    sinr_cosp,
    cosr_cosp
  );

  float sinp =
    2.0f * (qw * qy - qz * qx);

  if (fabs(sinp) >= 1.0f)
  {
    p = copysign(
      PI / 2.0f,
      sinp
    );
  }
  else
  {
    p = asin(sinp);
  }

  float siny_cosp =
    2.0f * (qw * qz + qx * qy);

  float cosy_cosp =
    1.0f - 2.0f * (qy * qy + qz * qz);

  y = atan2(
    siny_cosp,
    cosy_cosp
  );
}

// ================================================================
// BNO085 REPORT
// ================================================================

bool enableBNOReport()
{
  if (!bno08x.enableReport(
        SH2_ROTATION_VECTOR,
        5000
      ))
  {
    Serial.println(
      "BNO085 -> Could not enable Rotation Vector"
    );

    return false;
  }

  return true;
}

// ================================================================
// BNO085 SETUP
// ================================================================

void setupBNO085()
{
  Serial.print("BNO085     : ");

  if (!bno08x.begin_I2C())
  {
    Serial.println("NOT FOUND");

    bnoAvailable = false;
    return;
  }

  if (!enableBNOReport())
  {
    Serial.println("REPORT ERROR");

    bnoAvailable = false;
    return;
  }

  bnoAvailable = true;

  Serial.println("READY");
}

// ================================================================
// READ BNO085
// ================================================================

void updateBNO085()
{
  if (!bnoAvailable)
    return;

  if (bno08x.wasReset())
  {
    Serial.println(
      "BNO085 -> Reset detected"
    );

    enableBNOReport();
  }

  if (bno08x.getSensorEvent(&sensorValue))
  {
    if (
      sensorValue.sensorId ==
      SH2_ROTATION_VECTOR
    )
    {
      quaternionToEuler(
        sensorValue.un.rotationVector.real,
        sensorValue.un.rotationVector.i,
        sensorValue.un.rotationVector.j,
        sensorValue.un.rotationVector.k,
        yaw,
        pitch,
        roll
      );

      yaw   *= 180.0f / PI;
      pitch *= 180.0f / PI;
      roll  *= 180.0f / PI;

      data.yaw   = yaw;
      data.pitch = pitch;
      data.roll  = roll;
    }
  }
}

// ================================================================
// HC-SR04
// ================================================================

float readDistance()
{
  digitalWrite(TRIG_PIN, LOW);

  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);

  delayMicroseconds(10);

  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration =
    pulseIn(
      ECHO_PIN,
      HIGH,
      30000
    );

  if (duration == 0)
  {
    return -1.0f;
  }

  float distance =
    duration * 0.0343f / 2.0f;

  if (
    distance < 2.0f ||
    distance > 450.0f
  )
  {
    return -1.0f;
  }

  return distance;
}

// ================================================================
// BME280 SETUP
// ================================================================

void setupBME280()
{
  Serial.print("BME280     : ");

  if (bme.begin(0x76, &Wire))
  {
    bmeAvailable = true;

    Serial.println(
      "READY @ 0x76"
    );

    return;
  }

  if (bme.begin(0x77, &Wire))
  {
    bmeAvailable = true;

    Serial.println(
      "READY @ 0x77"
    );

    return;
  }

  bmeAvailable = false;

  Serial.println("NOT FOUND");
}

// ================================================================
// READ BME280
// ================================================================

void readBME280()
{
  if (!bmeAvailable)
  {
    data.temperature = 0.0f;
    data.humidity = 0.0f;
    data.pressure = 0.0f;
    data.altitude = 0.0f;

    return;
  }

  data.temperature =
    bme.readTemperature();

  data.humidity =
    bme.readHumidity();

  data.pressure =
    bme.readPressure() / 100.0f;

  data.altitude =
    bme.readAltitude(1013.25f);
}

// ================================================================
// TCS34725 SETUP
// ================================================================

void setupTCS34725()
{
  Serial.print("TCS34725   : ");

  if (!tcs.begin())
  {
    tcsAvailable = false;

    Serial.println("NOT FOUND");

    return;
  }

  tcsAvailable = true;

  Serial.println("READY");
}

// ================================================================
// READ TCS34725
// ================================================================

void readTCS34725()
{
  if (!tcsAvailable)
  {
    data.red = 0;
    data.green = 0;
    data.blue = 0;
    data.clear = 0;

    data.lux = 0.0f;
    data.colorTemperature = 0.0f;

    return;
  }

  uint16_t r;
  uint16_t g;
  uint16_t b;
  uint16_t c;

  tcs.getRawData(
    &r,
    &g,
    &b,
    &c
  );

  data.red = r;
  data.green = g;
  data.blue = b;
  data.clear = c;

  data.lux =
    tcs.calculateLux(
      r,
      g,
      b
    );

  data.colorTemperature =
    tcs.calculateColorTemperature(
      r,
      g,
      b
    );
}

// ================================================================
// MQ135 EQUATION
// ================================================================

float calculatePPM(
  float ratio,
  float a,
  float b
)
{
  if (
    ratio <= 0.0f ||
    !isfinite(ratio)
  )
  {
    return 0.0f;
  }

  return
    a *
    pow(
      ratio,
      b
    );
}

// ================================================================
// READ MQ135
// ================================================================

void readMQ135()
{
  const uint8_t samples = 20;

  uint32_t adcTotal = 0;

  for (
    uint8_t i = 0;
    i < samples;
    i++
  )
  {
    adcTotal +=
      analogRead(MQ135_PIN);

    delay(2);
  }

  float adcValue =
    (float)adcTotal / samples;

  float adcVoltage =
    (
      adcValue /
      ADC_MAX_VALUE
    ) *
    ADC_VOLTAGE;

  float sensorVoltage =
    adcVoltage /
    DIVIDER_RATIO;

  if (
    sensorVoltage <= 0.01f ||
    sensorVoltage >= MQ_VOLTAGE
  )
  {
    data.CO = 0.0f;
    data.alcohol = 0.0f;
    data.CO2 = 0.0f;
    data.toluene = 0.0f;
    data.NH4 = 0.0f;
    data.acetone = 0.0f;

    return;
  }

  float Rs =
    RL_VALUE *
    (
      MQ_VOLTAGE -
      sensorVoltage
    ) /
    sensorVoltage;

  float ratio =
    Rs /
    R0_VALUE;

  data.CO =
    calculatePPM(
      ratio,
      605.18f,
      -3.937f
    );

  data.alcohol =
    calculatePPM(
      ratio,
      77.255f,
      -3.18f
    );

  data.CO2 =
    calculatePPM(
      ratio,
      110.47f,
      -2.862f
    );

  data.toluene =
    calculatePPM(
      ratio,
      44.947f,
      -3.445f
    );

  data.NH4 =
    calculatePPM(
      ratio,
      102.2f,
      -2.473f
    );

  data.acetone =
    calculatePPM(
      ratio,
      34.668f,
      -3.369f
    );
}

// ================================================================
// READ ALL SENSORS
// ================================================================

void readSensors()
{
  data.magic =
    0xBEEF;

  data.packetNumber =
    ++packetCounter;

  data.distance =
    readDistance();

  readBME280();

  updateBNO085();

  readTCS34725();

  readMQ135();
}

// ================================================================
// SERIAL OUTPUT
// ================================================================

void printSensorData()
{
  Serial.println();

  Serial.println(
    "================================================"
  );

  Serial.print(
    "PACKET #"
  );

  Serial.println(
    data.packetNumber
  );

  Serial.println(
    "================================================"
  );

  Serial.println("[ HC-SR04 ]");

  Serial.print("Distance       : ");

  if (data.distance < 0)
  {
    Serial.println("NO ECHO");
  }
  else
  {
    Serial.print(data.distance, 2);
    Serial.println(" cm");
  }

  Serial.println();
  Serial.println("[ BME280 ]");

  Serial.print("Temperature    : ");
  Serial.print(data.temperature, 2);
  Serial.println(" C");

  Serial.print("Humidity       : ");
  Serial.print(data.humidity, 2);
  Serial.println(" %");

  Serial.print("Pressure       : ");
  Serial.print(data.pressure, 2);
  Serial.println(" hPa");

  Serial.print("Altitude       : ");
  Serial.print(data.altitude, 2);
  Serial.println(" m");

  Serial.println();
  Serial.println("[ BNO085 ]");

  if (!bnoAvailable)
  {
    Serial.println(
      "Status         : NOT CONNECTED"
    );
  }
  else
  {
    Serial.print("Yaw            : ");
    Serial.print(data.yaw, 2);
    Serial.println(" deg");

    Serial.print("Pitch          : ");
    Serial.print(data.pitch, 2);
    Serial.println(" deg");

    Serial.print("Roll           : ");
    Serial.print(data.roll, 2);
    Serial.println(" deg");
  }

  Serial.println();
  Serial.println("[ TCS34725 ]");

  Serial.print("Red            : ");
  Serial.println(data.red);

  Serial.print("Green          : ");
  Serial.println(data.green);

  Serial.print("Blue           : ");
  Serial.println(data.blue);

  Serial.print("Clear          : ");
  Serial.println(data.clear);

  Serial.print("Lux            : ");
  Serial.println(data.lux, 2);

  Serial.print("Color Temp     : ");
  Serial.print(data.colorTemperature, 2);
  Serial.println(" K");

  Serial.println();
  Serial.println("[ MQ135 - Approximate ]");

  Serial.print("CO             : ");
  Serial.print(data.CO, 2);
  Serial.println(" ppm");

  Serial.print("Alcohol        : ");
  Serial.print(data.alcohol, 2);
  Serial.println(" ppm");

  Serial.print("CO2            : ");
  Serial.print(data.CO2, 2);
  Serial.println(" ppm");

  Serial.print("Toluene        : ");
  Serial.print(data.toluene, 2);
  Serial.println(" ppm");

  Serial.print("NH4            : ");
  Serial.print(data.NH4, 2);
  Serial.println(" ppm");

  Serial.print("Acetone        : ");
  Serial.print(data.acetone, 2);
  Serial.println(" ppm");
}

// ================================================================
// SEND LoRa
// ================================================================

void sendLoRa()
{
  if (!loraAvailable)
  {
    return;
  }

  LoRa.beginPacket();

  LoRa.write(
    (uint8_t *)&data,
    sizeof(data)
  );

  int result =
    LoRa.endPacket();

  if (result == 1)
  {
    Serial.print(
      "\nLoRa -> SENT | "
    );

    Serial.print(
      sizeof(data)
    );

    Serial.println(
      " bytes"
    );
  }
  else
  {
    Serial.println(
      "\nLoRa -> SEND FAILED"
    );
  }
}

// ================================================================
// SETUP
// ================================================================

void setup()
{
  Serial.begin(115200);

  delay(1000);

  Serial.println();

  Serial.println(
    "================================================"
  );

  Serial.println(
    " MAKERNOVA IoT - LONG RANGE TRANSMITTER"
  );

  Serial.println(
    "================================================"
  );

  pinMode(
    TRIG_PIN,
    OUTPUT
  );

  pinMode(
    ECHO_PIN,
    INPUT
  );

  pinMode(
    MQ135_PIN,
    INPUT
  );

  digitalWrite(
    TRIG_PIN,
    LOW
  );

  analogReadResolution(12);

  // ==============================================================
  // I2C
  // ==============================================================

  Serial.println();
  Serial.println(
    "Initializing I2C sensors..."
  );

  Wire.begin(
    I2C_SDA,
    I2C_SCL
  );

  delay(100);

  setupBNO085();
  setupBME280();
  setupTCS34725();

  // ==============================================================
  // SPI
  // ==============================================================

  Serial.println();

  Serial.println(
    "Initializing LoRa..."
  );

  pinMode(
    LORA_CS,
    OUTPUT
  );

  digitalWrite(
    LORA_CS,
    HIGH
  );

  SPI.begin(
    SPI_SCK,
    SPI_MISO,
    SPI_MOSI,
    LORA_CS
  );

  LoRa.setPins(
    LORA_CS,
    LORA_RST,
    LORA_DIO0
  );

  if (!LoRa.begin(LORA_FREQ))
  {
    Serial.println(
      "LoRa SX1278 : NOT FOUND"
    );

    loraAvailable =
      false;
  }
  else
  {
    // ============================================================
    // LONG RANGE LoRa SETTINGS
    // ============================================================

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

    LoRa.setTxPower(
      LORA_TX_POWER
    );

    LoRa.enableCrc();

    loraAvailable = true;

    Serial.println(
      "LoRa SX1278 : READY"
    );

    Serial.println(
      "Frequency   : 433 MHz"
    );

    Serial.println(
      "Sync Word   : 0x12"
    );

    Serial.println(
      "SF          : 10"
    );

    Serial.println(
      "Bandwidth   : 125 kHz"
    );

    Serial.println(
      "Coding Rate : 4/5"
    );

    Serial.println(
      "TX Power    : 17 dBm"
    );

    Serial.println(
      "CRC         : ON"
    );
  }

  Serial.println();

  Serial.print(
    "Packet Size : "
  );

  Serial.print(
    sizeof(SensorData)
  );

  Serial.println(
    " bytes"
  );

  Serial.println();

  Serial.println(
    "================================================"
  );

  Serial.println(
    "             TRANSMITTER READY"
  );

  Serial.println(
    "================================================"
  );

  lastSendTime =
    millis();
}

// ================================================================
// LOOP
// ================================================================

void loop()
{
  // BNO085 continuously updated
  updateBNO085();

  if (
    millis() -
    lastSendTime >=
    SEND_INTERVAL
  )
  {
    lastSendTime =
      millis();

    readSensors();

    printSensorData();

    sendLoRa();
  }
}
