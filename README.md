# Automatic Audio Gain Adjustment System Based on Real-time Crowd Density

An **ESP32-based real-time audio receiver** over **WiFi (UDP)** with **I2S audio output**, **digital potentiometer gain control**, and **human presence detection** using the **LD2410 mmWave radar sensor**.  
Designed for **low-latency audio streaming** using **FreeRTOS multitasking**.

---

## Features

-  UDP-based real-time audio streaming
-  I2S audio output (16 kHz / 16-bit / Mono)
-  Digital volume control using MCP41HVX1
-  Human presence detection with LD2410 mmWave radar to change volume
-  FreeRTOS multitasking (UDP, Audio, Radar)
-  Bidirectional control/command channel

---

## Hardware Requirements

- ESP32 (WiFi + FreeRTOS)
- I2S Audio Amplifier (e.g. PAM8610)
- LD2410 mmWave Radar Sensor
- MCP41HVX1 Digital Potentiometer (50k ohm) 
- Speaker 
- Stable power supply

---

## Pin Configuration

### LD2410 Radar
| Signal | ESP32 GPIO |
|------|-----------|
| RX   | 16 |
| TX   | 17 |
| OUT  | 33 |

### MCP41HVX1 (SPI)
| Signal | ESP32 GPIO |
|------|-----------|
| CS   | 21 |
| SCK  | 18 |
| MISO | 19 |
| MOSI | 23 |

### I2S Audio
| Signal | ESP32 GPIO |
|------|-----------|
| BCK  | 26 |
| WS / LRC | 25 |
| DATA | 27 |

---

## WiFi Configuration

```text
SSID     : Audio-Net
Password : password123
UDP Port : 4210
```
---
## packet stuckture 
Each UDP packet must be exactly 524 bytes
- Sequence number         -----> 2 bytes
- Command / control words -----> 12 bytes
- PCM audio samples       -----> 512 bytes
```text
┌───────────────────────────────────────────────────────────┐
│                  AudioPacket (524 bytes)                  │
├───────────────┬───────────────────────────────────────────┤
│ seq (2 bytes) │ Packet sequence number (uint16_t)         │
├───────────────┼───────────────────────────────────────────┤
│ cmd[0] (2B)   │ Application-defined command / control     │
│ cmd[1] (2B)   │ Application-defined command / control     │
│ cmd[2] (2B)   │ Application-defined command / control     │
│ cmd[3] (2B)   │ Application-defined command / control     │
│ cmd[4] (2B)   │ Application-defined command / control     │
│ cmd[5] (2B)   │ Application-defined command / control     │
├───────────────┼───────────────────────────────────────────┤
│ pcm[0] (2B)   │ Signed PCM audio sample                   │
│ pcm[1] (2B)   │ Signed PCM audio sample                   │
│ ...           │ ...                                       │
│ pcm[254] (2B) │ Signed PCM audio sample                   │
│ pcm[255] (2B) │ Signed PCM audio sample                   │
└───────────────┴───────────────────────────────────────────┘

```
packet in code 
```C++
struct AudioPacket {
  uint16_t seq;        // Sequence number         
  uint16_t cmd[6];     // Command / control words 
  int16_t  pcm[256];   // PCM audio samples     
};
```
