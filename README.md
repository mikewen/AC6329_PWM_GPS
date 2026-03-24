# AC6329C Sensor Fusion Firmware (GPS + IMU)

This firmware integrates high-speed UART GPS parsing ($PQTMTAR, $GNRMC) and I2C IMU data (QMI8658 + MMC5603). It uses an ISR-to-Task semaphore architecture to handle 115200 baud data without loss on resource-constrained hardware.

## 🚀 System Architecture
- **MCU:** AC6329C (High-speed mode optimized)
- **UART:** 115200 Baud, 512-byte Ring Buffer
- **IMU:** QMI8658 (Accel/Gyro) & MMC5603 (Mag)
- **IIC Driver:** [mikewen/AC6329_IIC](https://github.com) 
- **Task Management:** ISR-driven semaphore triggers a dedicated RX Task to avoid blocking CPU.

---

## 📊 Data Protocol (Little-Endian)

All multi-byte values (u16, s16, u32, s32) are transmitted **Little-Endian**.

### 1. IMU Data (Header: `0xA1`)
**Size:** 20 Bytes | **Freq:** ~50Hz


| Byte | Field | Type | Description |
|:---|:---|:---|:---|
| 0 | Header | u8 | `0xA1` |
| 1 | Seq | u8 | Rolling counter (0-255) |
| 2-7 | Accel (X,Y,Z) | 3x s16 | Raw Accel |
| 8-13 | Gyro (X,Y,Z) | 3x s16 | Raw Gyro |
| 14-19| Mag (X,Y,Z) | 3x s16 | Raw Magnetometer |

### 2. GPS Orientation (Header: `0xA2`)
**Source:** `$PQTMTAR` | **Size:** 17 Bytes


| Byte | Field | Type | Scale (Multiplier) |
|:---|:---|:---|:---|
| 0 | Header | u8 | `0xA2` |
| 1-4 | Time | u32 | HHMMSS.sss * 1000 |
| 5 | Quality | u8 | 0:Inv, 4:RTK Fixed, 6:DR |
| 6-7 | Length | u16 | Baseline * 1000 |
| 8-9 | Pitch | s16 | Degrees * 100 |
| 10-11| Roll | s16 | Degrees * 100 |
| 12-13| Heading | u16 | Degrees * 100 (0-36000) |
| 14-15| Acc_Head | u16 | Accuracy * 1000 |
| 16 | UsedSV | u8 | Satellite Count |

### 3. GPS Position (Header: `0xA3`)
**Source:** `$GNRMC` | **Size:** 17 Bytes


| Byte | Field | Type | Scale (Multiplier) |
|:---|:---|:---|:---|
| 0 | Header | u8 | `0xA3` |
| 1-4 | Time | u32 | HHMMSS.sss * 1000 |
| 5-8 | Latitude | s32 | DDMM.MMMM * 10^6 |
| 9-12 | Longitude | s32 | DDDMM.MMMM * 10^6 |
| 13-14| Speed | u16 | Knots * 100 |
| 15-16| Course | u16 | Degrees * 100 |

---

## 📱 Android / Java Parser Snippet

```java
public void onBleDataReceived(byte[] data) {
    if (data.length < 17) return;
    
    ByteBuffer buffer = ByteBuffer.wrap(data).order(ByteOrder.LITTLE_ENDIAN);
    int header = data[0] & 0xFF;

    switch (header) {
        case 0xA1: // IMU Data
            short az = buffer.getShort(6);
            // Process IMU...
            break;

        case 0xA2: // GPS Orientation (PQTMTAR)
            // Divide by scale to get original units
            float pitch = buffer.getShort(8) / 100.0f;
            float heading = (buffer.getShort(12) & 0xFFFF) / 100.0f;
            Log.d("GPS", "Heading: " + heading + " Pitch: " + pitch);
            break;

        case 0xA3: // GPS Position (GNRMC)
            float rawLat = buffer.getInt(5) / 1000000.0f; // DDMM.MMMM
            float rawLon = buffer.getInt(9) / 1000000.0f; // DDDMM.MMMM
            
            // Helper to convert DDMM.MMMM to Decimal Degrees
            double lat = convertNmeaToDecimal(rawLat);
            double lon = convertNmeaToDecimal(rawLon);
            Log.d("GPS", "Lat: " + lat + " Lon: " + lon);
            break;
    }
}

private double convertNmeaToDecimal(float nmea) {
    int degrees = (int)(nmea / 100);
    float minutes = nmea - (degrees * 100);
    return degrees + (minutes / 60.0);
}
