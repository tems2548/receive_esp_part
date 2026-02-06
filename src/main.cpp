#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "driver/i2s.h"
#include <SPI.h>
#include "MCP41HVX1.h"
#include <ld2410.h>

#define RADAR Serial1
#define MONITOR Serial
// ld2410
#define RADAR_RX_PIN 16
#define RADAR_TX_PIN 17
#define LD2410_OUT_PIN 33

// Digipot
#define CS_PIN 21
MCP41HVX1 Digipot(CS_PIN);

// I2S AMP
#define I2S_PORT I2S_NUM_0
#define I2S_BCK 26
#define I2S_WS 25   // LRC
#define I2S_DATA 27 // DIN

// RTOS
QueueHandle_t audioQueue;
ld2410 radar;

uint32_t lastReading = 0;

IPAddress senderIP;
uint16_t senderPort;
bool senderKnown = false;
const char *ssid = "Audio-Net";
const char *password = "password123";
const int udpPort = 4210;
WiFiUDP udp;

struct __attribute__((packed)) AudioPacket
{
  uint16_t seq;
  uint16_t cmd[6];
  int16_t pcm[256];
};

void setupI2S()
{
  i2s_config_t cfg = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
      .sample_rate = 16000,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
      .intr_alloc_flags = 0,
      .dma_buf_count = 8,
      .dma_buf_len = 256,
      .use_apll = false};

  i2s_pin_config_t pin_cfg = {
      .bck_io_num = I2S_BCK,
      .ws_io_num = I2S_WS,
      .data_out_num = I2S_DATA,
      .data_in_num = -1};

  i2s_driver_install(I2S_PORT, &cfg, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_cfg);
  i2s_start(I2S_PORT);
}

// Task UDP
void TaskUDP(void *pvParameters)
{
  AudioPacket pkt;
  while (1)
  {
    int p = udp.parsePacket();

    // Save sender info
    senderIP = udp.remoteIP();
    senderPort = udp.remotePort();
    senderKnown = true;

    if (senderKnown) {

      udp.beginPacket(senderIP, senderPort);
      udp.write((uint8_t*)pkt.cmd, sizeof(pkt.cmd));
      udp.endPacket();

     // Serial.println("CMD sent back");
    }

    if (p == sizeof(pkt))
    {
      // read data
      udp.read((uint8_t *)&pkt, sizeof(pkt));

      // read cmd
      // MONITOR.println(pkt.cmd[0-5]);
      xQueueSend(audioQueue, &pkt, portMAX_DELAY);
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// Task I2S
void TaskI2S(void *pvParameters)
{
  MONITOR.println("TaskI2S START");
  vTaskDelay(pdMS_TO_TICKS(10)); // บังคับคืน CPU ครั้งแรก

  static AudioPacket pkt;
  size_t bw;

  while (1)
  {
    if (xQueueReceive(audioQueue, &pkt, portMAX_DELAY) == pdTRUE)
    {
      i2s_write(I2S_PORT, pkt.pcm, sizeof(pkt.pcm), &bw, portMAX_DELAY);
    }
  }
}
// Task I2S
void TaskDetection(void *pvParameters)
{
  // MONITOR.println("TaskI2S START");
  vTaskDelay(pdMS_TO_TICKS(10)); // บังคับคืน CPU ครั้งแรก
  while (1)
  {
    radar.read();

    if (radar.isConnected() && millis() - lastReading > 1000)
    {
      lastReading = millis();
      // GPIO Presence
      bool presenceGPIO = digitalRead(LD2410_OUT_PIN);

      if (presenceGPIO)
        Serial.print(F("GPIO Presence: YES | "));
      else
        Serial.print(F("GPIO Presence: NO  | "));

      // ส่วนเดิม ไม่เปลี่ยน
      if (radar.presenceDetected())
      {
        if (radar.stationaryTargetDetected())
        {
          Serial.print(F("Stationary target: "));
          Serial.print(radar.stationaryTargetDistance());
          Serial.print(F("cm energy:"));
          Serial.print(radar.stationaryTargetEnergy());
          Serial.print(' ');
        }

        if (radar.movingTargetDetected())
        {
          Serial.print(F("Moving target: "));
          Serial.print(radar.movingTargetDistance());
          Serial.print(F("cm energy:"));
          Serial.print(radar.movingTargetEnergy());
        }

        Serial.println();
      }
      else
      {
        Serial.println(F("No target"));
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void setup()
{
  MONITOR.begin(115200);

  // create Queue
  audioQueue = xQueueCreate(8, sizeof(AudioPacket));

  // setup sensor
  RADAR.begin(256000, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
  pinMode(LD2410_OUT_PIN, INPUT);

  // setup
  setupI2S();

  // UDP setup
  WiFi.begin(ssid, password);
  udp.begin(udpPort);

  // digipot setup
  SPI.begin(18, 19, 23, CS_PIN);

  // setup resistance value form MCP41HVX1 0 - 255
  //! DANGER don't set gain over 100 tab
  Digipot.WiperSetPosition(40);

  // setup sensor
  if (radar.begin(RADAR))
  {
    MONITOR.println(F("OK"));
    MONITOR.print(F("LD2410 firmware version: "));
    MONITOR.print(radar.firmware_major_version);
    MONITOR.print('.');
    MONITOR.print(radar.firmware_minor_version);
    MONITOR.print('.');
    MONITOR.println(radar.firmware_bugfix_version, HEX);
  }
  else
  {
    MONITOR.println(F("not connected"));
  }

  // Build TaskUDP
  xTaskCreatePinnedToCore(
      TaskUDP, // function
      "UDP",   // task name
      4096,    // stack size wifi/BLE/MQTT 4096-8192
      NULL,    // Parameters
      0,       // priority 3 > 2 > 1 > 0
      NULL,    // Task handle for suspent/resume or delete task
      0        // core 0
  );
  xTaskCreatePinnedToCore(
      TaskDetection,
      "Detection",
      4096,
      NULL,
      1,
      NULL,
      1);
  // Build TaskI2S
  xTaskCreatePinnedToCore(
      TaskI2S,
      "I2S",
      8192,
      NULL,
      2,
      NULL,
      1);

  taskYIELD();

  // check setup
  MONITOR.println("SETUP DONE");
  if (audioQueue != NULL)
  {
    MONITOR.println("Queue OK!");
  }
}

// FreeRTOS will manage task so don't use LOOP!!
void loop()
{
  // No Need
}