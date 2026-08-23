#include <Wire.h>
#include <DHT.h>
#include <MPU6050.h>
#include <LoRa_E32.h>

#include "esp_camera.h"
#include "FS.h"
#include "SD_MMC.h"

#include <math.h>
#include <string.h>

// ============================================================
// ESP32-S3 CANSAT - FLIGHT READY VERSION
//
// OV3660 camera
// 1280x720 JPEG -> SD CSV Telemetry Logging
// MPU6050 (Safe Wakeup)
// BMP280 / BME280
// DHT11
// E32-433T30D (Transparent Transmission Mode)
// Passive buzzer (2-Minute Beacon)
//
// ============================================================


// ============================================================
// CAMERA GPIO
// ============================================================

#define PWDN_GPIO_NUM        -1
#define RESET_GPIO_NUM       -1
#define XCLK_GPIO_NUM        15
#define SIOD_GPIO_NUM         4
#define SIOC_GPIO_NUM         5
#define Y2_GPIO_NUM          11
#define Y3_GPIO_NUM           9
#define Y4_GPIO_NUM           8
#define Y5_GPIO_NUM          10
#define Y6_GPIO_NUM          12
#define Y7_GPIO_NUM          18
#define Y8_GPIO_NUM          17
#define Y9_GPIO_NUM          16
#define VSYNC_GPIO_NUM        6
#define HREF_GPIO_NUM         7
#define PCLK_GPIO_NUM        13


// ============================================================
// SD CARD
// ============================================================

#define SD_CLK_GPIO          39
#define SD_CMD_GPIO          38
#define SD_D0_GPIO           40


// ============================================================
// I2C
// ============================================================

#define I2C_SDA               2
#define I2C_SCL               1


// ============================================================
// DHT11
// ============================================================

#define DHT_PIN              42
#define DHT_TYPE             DHT11

DHT dht(DHT_PIN, DHT_TYPE);


// ============================================================
// MPU6050
// ============================================================

MPU6050 mpu(0x68);

bool mpu_ok = false;


// ============================================================
// BMP280 / BME280
// ============================================================

#define BMP280_ADDR_1        0x76
#define BMP280_ADDR_2        0x77

uint8_t bmpAddress = 0;
bool bmp_ok = false;

uint16_t dig_T1; int16_t  dig_T2; int16_t  dig_T3;
uint16_t dig_P1; int16_t  dig_P2; int16_t  dig_P3;
int16_t  dig_P4; int16_t  dig_P5; int16_t  dig_P6;
int16_t  dig_P7; int16_t  dig_P8; int16_t  dig_P9;


// ============================================================
// BUZZER (2-MINUTE RECOVERY BEACON)
// ============================================================

#define BUZZER_PIN            41
#define BUZZ_FREQUENCY        4000

#define BUZZER_ON_TIME        511
#define BUZZER_OFF_TIME       511
#define BUZZER_ARM_TIME       (2UL * 60UL * 1000UL) 

bool buzzerState = false;
unsigned long lastBuzzerToggle = 0;


// ============================================================
// LANDING DETECTION
// ============================================================

#define LANDING_ARM_TIME      (5UL * 60UL * 1000UL)
#define ALTITUDE_THRESHOLD    1.5f
#define STABILITY_TIME        (2UL * 60UL * 1000UL)

bool landingArmed = false;
bool landingDetected = false;
bool stabilityStarted = false;
unsigned long stabilityStartTime = 0;
float stabilityReferenceAltitude = 0.0f;


// ============================================================
// E32 UART (TRANSPARENT MODE)
// ============================================================

#define E32_RX_PIN             48
#define E32_TX_PIN             47

HardwareSerial E32Serial(1);
LoRa_E32 e32(&E32Serial, UART_BPS_RATE_9600);


// ============================================================
// SENSOR CONSTANTS & TIMERS
// ============================================================

#define G_TO_MS2               9.80665f
#define SEND_INTERVAL          2137UL
#define CAMERA_INTERVAL        100UL

unsigned long bootTime = 0;
unsigned long lastSend = 0;
unsigned long lastFrame = 0;


// ============================================================
// TELEMETRY PACKET
// ============================================================

struct __attribute__((packed)) DataPacket
{
  uint16_t packetID;
  float ax; float ay; float az; float a_total;
  float temperature; float humidity;
  float pressure; float altitude;
  uint8_t checksum;
};

DataPacket data;
uint16_t packetID = 0;


// ============================================================
// MPU CALIBRATION & CACHED VALUES
// ============================================================

float gx = 0.0f; float gy = 0.0f; float gz = 0.0f; float gMag = 16384.0f;
float lastTemp = 25.0f; float lastHumidity = 50.0f;
float lastPressure = 1013.25f; float lastAltitude = 0.0f;
float referencePressure = 1013.25f;


// ============================================================
// BMP280 LOW-LEVEL I2C
// ============================================================

bool bmpWriteRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(bmpAddress);
  Wire.write(reg); Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool bmpReadRegisters(uint8_t reg, uint8_t *buffer, uint8_t length) {
  Wire.beginTransmission(bmpAddress);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t received = Wire.requestFrom((int)bmpAddress, (int)length);
  if (received != length) return false;
  for (uint8_t i = 0; i < length; i++) buffer[i] = Wire.read();
  return true;
}

uint8_t bmpRead8(uint8_t reg) {
  uint8_t value = 0; bmpReadRegisters(reg, &value, 1); return value;
}
uint16_t bmpRead16(uint8_t reg) {
  uint8_t b[2]; if (!bmpReadRegisters(reg, b, 2)) return 0;
  return ((uint16_t)b[0] << 8) | b[1];
}
uint16_t bmpRead16LE(uint8_t reg) {
  uint8_t b[2]; if (!bmpReadRegisters(reg, b, 2)) return 0;
  return ((uint16_t)b[1] << 8) | b[0];
}
int16_t bmpReadS16LE(uint8_t reg) {
  return (int16_t)bmpRead16LE(reg);
}

// ============================================================
// BMP280 INITIALIZATION
// ============================================================

// ============================================================
// BMP280 INITIALIZATION (BYPASS CHIP ID CHECK)
// ============================================================

bool initBMP280() {
  uint8_t candidateAddresses[2] = { BMP280_ADDR_1, BMP280_ADDR_2 };
  
  for (int i = 0; i < 2; i++) {
    uint8_t addr = candidateAddresses[i];
    Wire.beginTransmission(addr);
    
    // If the sensor responds at this address, claim it!
    if (Wire.endTransmission() == 0) {
      bmpAddress = addr;
      uint8_t chipID = bmpRead8(0xD0);
      
      // Print the ID just so we can see what the clone is calling itself
      Serial.printf("Barometer found at 0x%02X! (Internal Chip ID: 0x%02X)\n", addr, chipID);
      
      // BYPASS: We completely removed the strict ID check here. 
      // As long as it answered us, we break the loop and proceed.
      break; 
    }
  }

  if (bmpAddress == 0) {
    Serial.println("BMP280 / BME280 not found"); 
    return false;
  }

  // Read calibration data
  dig_T1 = bmpRead16LE(0x88); dig_T2 = bmpReadS16LE(0x8A); dig_T3 = bmpReadS16LE(0x8C);
  dig_P1 = bmpRead16LE(0x8E); dig_P2 = bmpReadS16LE(0x90); dig_P3 = bmpReadS16LE(0x92);
  dig_P4 = bmpReadS16LE(0x94); dig_P5 = bmpReadS16LE(0x96); dig_P6 = bmpReadS16LE(0x98);
  dig_P7 = bmpReadS16LE(0x9A); dig_P8 = bmpReadS16LE(0x9C); dig_P9 = bmpReadS16LE(0x9E);
  
  // Configure the sensor
  uint8_t config = (0b011 << 5) | (0b010 << 2);
  uint8_t ctrl = (0b010 << 5) | (0b101 << 2) | 0b11;
  bmpWriteRegister(0xF5, config); 
  bmpWriteRegister(0xF4, ctrl);
  
  return true;
}

// ============================================================
// BMP280 TEMPERATURE + PRESSURE
// ============================================================

bool readBMP280(float &temperature, float &pressure) {
  uint8_t b[6];
  if (!bmpReadRegisters(0xF7, b, 6)) return false;
  int32_t adc_P = ((int32_t)b[0] << 12) | ((int32_t)b[1] << 4) | (b[2] >> 4);
  int32_t adc_T = ((int32_t)b[3] << 12) | ((int32_t)b[4] << 4) | (b[5] >> 4);
  if (adc_T == 0x80000 || adc_P == 0x80000) return false;
  int32_t var1, var2, t_fine;
  var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
  t_fine = var1 + var2;
  int32_t T = (t_fine * 5 + 128) >> 8;
  temperature = T / 100.0f;
  int64_t p_var1, p_var2;
  p_var1 = ((int64_t)t_fine) - 128000;
  p_var2 = p_var1 * p_var1 * (int64_t)dig_P6;
  p_var2 = p_var2 + ((p_var1 * (int64_t)dig_P5) << 17);
  p_var2 = p_var2 + (((int64_t)dig_P4) << 35);
  p_var1 = ((p_var1 * p_var1 * (int64_t)dig_P3) >> 8) + ((p_var1 * (int64_t)dig_P2) << 12);
  p_var1 = (((((int64_t)1) << 47) + p_var1) * (int64_t)dig_P1) >> 33;
  if (p_var1 == 0) return false;
  int64_t p = 1048576 - adc_P;
  p = (((p << 31) - p_var2) * 3125) / p_var1;
  p_var1 = ((int64_t)dig_P9 * (p >> 13) * (p >> 13)) >> 25;
  p_var2 = ((int64_t)dig_P8 * p) >> 19;
  p = ((p + p_var1 + p_var2) >> 8) + ((int64_t)dig_P7 << 4);
  pressure = p / 256.0f / 100.0f;
  return !isnan(pressure) && !isnan(temperature);
}

float calculateAltitude(float pressure) {
  if (pressure <= 0 || referencePressure <= 0) return 0.0f;
  return 44330.0f * (1.0f - powf(pressure / referencePressure, 0.19029495f));
}

// ============================================================
// CRC-8
// ============================================================

uint8_t crc8(const uint8_t *buffer, size_t length) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < length; i++) {
    crc ^= buffer[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x80) crc = (uint8_t)((crc << 1) ^ 0x07);
      else crc <<= 1;
    }
  }
  return crc;
}


// ============================================================
// CAMERA INITIALIZATION
// ============================================================

bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0; config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM; config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM; config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM; config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM; config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM; config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM; config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM; config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000; config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_HD; config.jpeg_quality = 10;
  config.fb_count = 2; config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  if (esp_camera_init(&config) != ESP_OK) return false;
  Serial.println("OV3660 camera OK"); return true;
}

bool initSD() {
  SD_MMC.setPins(SD_CLK_GPIO, SD_CMD_GPIO, SD_D0_GPIO);
  if (!SD_MMC.begin("/sdcard", true)) return false;
  if (SD_MMC.cardType() == CARD_NONE) return false;
  if (!SD_MMC.exists("/frames")) SD_MMC.mkdir("/frames");
  Serial.println("SD card OK"); return true;
}

void saveCameraFrame() {
  static uint32_t frameNumber = 0;
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;
  char filename[64];
  snprintf(filename, sizeof(filename), "/frames/frame_%06lu.jpg", (unsigned long)frameNumber);
  File file = SD_MMC.open(filename, FILE_WRITE);
  if (file) {
    if (file.write(fb->buf, fb->len) == fb->len) frameNumber++;
    file.close();
  }
  esp_camera_fb_return(fb);
}


// ============================================================
// SENSOR INITIALIZATION (SAFE MPU WAKEUP)
// ============================================================

void initSensors() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  bmp_ok = initBMP280();

  // Safe manual wakeup prevents I2C freezing bugs
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); // PWR_MGMT_1 Register
  Wire.write(0x00); // Write 0 to wake it up
  Wire.endTransmission();
  delay(10);

  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  if (ax != 0 || ay != 0 || az != 0) {
    mpu_ok = true; Serial.println("MPU6050 OK (Data check passed)");
  } else {
    mpu_ok = false; Serial.println("MPU6050 NOT FOUND");
  }

  dht.begin();
  Serial.println("DHT11 initialized");
}

void initializeAltitudeReference() {
  if (!bmp_ok) return;
  Serial.println("\nEstablishing launch pressure...");
  delay(1000);
  double sum = 0.0; int valid = 0;
  for (int i = 0; i < 20; i++) {
    float temperature, pressure;
    if (readBMP280(temperature, pressure)) { sum += pressure; valid++; }
    delay(50);
  }
  if (valid > 0) {
    referencePressure = sum / valid; lastPressure = referencePressure; lastAltitude = 0.0;
    Serial.printf("Launch pressure = %.2f hPa\n", referencePressure);
  }
}

void calibrateMPU() {
  if (!mpu_ok) return;
  Serial.println("MPU6050 calibration...");
  long sx = 0; long sy = 0; long sz = 0;
  for (int i = 0; i < 200; i++) {
    int16_t ax, ay, az; mpu.getAcceleration(&ax, &ay, &az);
    sx += ax; sy += ay; sz += az; delay(5);
  }
  gx = sx / 200.0f; gy = sy / 200.0f; gz = sz / 200.0f;
  gMag = sqrtf(gx * gx + gy * gy + gz * gz);
}

void readAcceleration() {
  if (!mpu_ok) { data.ax = data.ay = data.az = data.a_total = 0.0f; return; }
  int16_t ax_raw, ay_raw, az_raw;
  mpu.getAcceleration(&ax_raw, &ay_raw, &az_raw);
  data.ax = ((ax_raw - gx) / 16384.0f) * G_TO_MS2;
  data.ay = ((ay_raw - gy) / 16384.0f) * G_TO_MS2;
  data.az = ((az_raw - gz) / 16384.0f) * G_TO_MS2;
  float magnitude = sqrtf((float)ax_raw * ax_raw + (float)ay_raw * ay_raw + (float)az_raw * az_raw);
  data.a_total = (fabsf(magnitude - gMag) / 16384.0f) * G_TO_MS2;
}

void readDHT11() {
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  if (!isnan(temperature) && temperature > -40.0f && temperature < 80.0f) lastTemp = temperature;
  if (!isnan(humidity) && humidity >= 0.0f && humidity <= 100.0f) lastHumidity = humidity;
  data.temperature = lastTemp; data.humidity = lastHumidity;
}

void readBarometer() {
  if (!bmp_ok) { data.pressure = lastPressure; data.altitude = lastAltitude; return; }
  float temperature, pressure;
  if (readBMP280(temperature, pressure)) {
    if (pressure > 300.0f && pressure < 1200.0f) {
      lastPressure = pressure;
      float altitude = calculateAltitude(pressure);
      if (!isnan(altitude) && altitude > -500.0f && altitude < 9000.0f) lastAltitude = altitude;
    }
  }
  data.pressure = lastPressure; data.altitude = lastAltitude;
}


// ============================================================
// LANDING DETECTION
// ============================================================

void updateLandingDetection() {
  unsigned long now = millis();
  if (!landingArmed) {
    if (now - bootTime >= LANDING_ARM_TIME) { landingArmed = true; Serial.println("\nLANDING DETECTION ARMED"); }
    return;
  }
  if (landingDetected) return;
  float currentAltitude = data.altitude;
  if (!stabilityStarted) {
    stabilityStarted = true; stabilityStartTime = now; stabilityReferenceAltitude = currentAltitude; return;
  }
  if (fabsf(currentAltitude - stabilityReferenceAltitude) > ALTITUDE_THRESHOLD) {
    stabilityStarted = false; return;
  }
  if (now - stabilityStartTime >= STABILITY_TIME) {
    landingDetected = true; Serial.println("\nLANDING DETECTED");
  }
}


// ============================================================
// BUZZER BEACON TIMER
// ============================================================

void updateBuzzer() {
  unsigned long now = millis();
  
  // Wait 2 minutes before allowing buzzer to sound
  if (now - bootTime < BUZZER_ARM_TIME) {
    if (buzzerState) { noTone(BUZZER_PIN); buzzerState = false; }
    return;
  }

  // Active pulsing cycle
  if (buzzerState) {
    if (now - lastBuzzerToggle >= BUZZER_ON_TIME) {
      noTone(BUZZER_PIN); buzzerState = false; lastBuzzerToggle = now;
    }
  } else {
    if (now - lastBuzzerToggle >= BUZZER_OFF_TIME) {
      tone(BUZZER_PIN, BUZZ_FREQUENCY); buzzerState = true; lastBuzzerToggle = now;
    }
  }
}


// ============================================================
// E32 INITIALIZATION & TELEMETRY
// ============================================================

bool initE32() {
  E32Serial.begin(9600, SERIAL_8N1, E32_RX_PIN, E32_TX_PIN);
  delay(100);
  return e32.begin();
}

void sendTelemetry() {
  packetID++; data.packetID = packetID;
  data.checksum = crc8((uint8_t*)&data, sizeof(DataPacket) - 1);
  const uint8_t SYNC0 = 0xAA; const uint8_t SYNC1 = 0x55;
  const size_t frameSize = 2 + sizeof(DataPacket);
  uint8_t frame[frameSize];
  frame[0] = SYNC0; frame[1] = SYNC1;
  memcpy(frame + 2, &data, sizeof(DataPacket));

  ResponseStatus status = e32.sendMessage(frame, frameSize);
  Serial.printf("TX ID=%u | AX=%.2f AY=%.2f AZ=%.2f TOTAL=%.2f ALT=%.2f P=%.2f T=%.2f RH=%.2f | ",
    data.packetID, data.ax, data.ay, data.az, data.a_total,
    data.altitude, data.pressure, data.temperature, data.humidity);
  Serial.println(status.getResponseDescription());
}

void logTelemetryToSD() {
  File logFile = SD_MMC.open("/telemetry.csv", FILE_APPEND);
  if (logFile) {
    logFile.printf("%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,0x%02X\n",
      data.packetID, data.ax, data.ay, data.az, data.a_total,
      data.temperature, data.humidity, data.pressure, data.altitude, data.checksum);
    logFile.close();
  }
}


// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n==========================================");
  Serial.println(" ESP32-S3 CANSAT TRANSMITTER - FLIGHT READY");
  Serial.println("==========================================");

  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);

  initSensors();
  initializeAltitudeReference();
  calibrateMPU();

  if (!initCamera()) Serial.println("WARNING: camera unavailable");

  if (!initSD()) {
    Serial.println("WARNING: SD unavailable");
  } else {
    if (!SD_MMC.exists("/telemetry.csv")) {
      File logFile = SD_MMC.open("/telemetry.csv", FILE_WRITE);
      if (logFile) {
        logFile.println("Packet ID,Accel X,Accel Y,Accel Z,Accel Total,Temp (C),Humidity (%),Pressure (hPa),Altitude (m),Checksum");
        logFile.close(); Serial.println("Created telemetry.csv on SD card");
      }
    }
  }

  if (!initE32()) Serial.println("WARNING: E32 initialization failed");

  bootTime = millis();
  lastSend = millis();
  lastFrame = millis();
  Serial.println("\nSYSTEM READY");
}


// ============================================================
// LOOP (PRIORITY TASK SCHEDULER)
// ============================================================

void loop()
{
  unsigned long now = millis();

  // Fast, low-power tasks
  readAcceleration();
  updateLandingDetection();
  updateBuzzer();

  // MUTUAL EXCLUSION ZONE: Power Safety
  // Only trigger camera or radio if buzzer is currently silent
  if (!buzzerState) 
  {
    // PRIORITY 1: TELEMETRY
    if (now - lastSend >= SEND_INTERVAL)
    {
      lastSend = now;
      readDHT11();
      readBarometer();
      sendTelemetry();
      logTelemetryToSD();
      return; 
    }

    // PRIORITY 2: CAMERA
    if (now - lastFrame >= CAMERA_INTERVAL)
    {
      lastFrame = now;
      saveCameraFrame();
      return; 
    }
  }
}