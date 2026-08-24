// BMI088 library:
#ifndef BMI088_H
#define BMI088_H

#include <Arduino.h>
#include <Wire.h>

// --- Accelerometer registers ---
#define BMI088_ACC_CHIP_ID      0x00
#define BMI088_ACC_SOFTRESET    0x7E
#define BMI088_ACC_PWR_CONF     0x7C
#define BMI088_ACC_PWR_CTRL     0x7D
#define BMI088_ACC_CONF         0x40
#define BMI088_ACC_RANGE        0x41
#define BMI088_ACC_DATA         0x12
#define BMI088_ACC_TEMP         0x22

// --- Gyroscope registers ---
#define BMI088_GYR_CHIP_ID      0x00
#define BMI088_GYR_RANGE        0x0F
#define BMI088_GYR_BANDWIDTH    0x10
#define BMI088_GYR_LPM1         0x11
#define BMI088_GYR_SOFTRESET    0x14
#define BMI088_GYR_DATA         0x02
#define BMI088_GYR_TEMP         0x08


enum AccRange {
    ACC_RANGE_3G  = 0x00,
    ACC_RANGE_6G  = 0x01,
    ACC_RANGE_12G = 0x02,
    ACC_RANGE_24G = 0x03
};

enum AccODR {
    ACC_ODR_100HZ  = 0xA8,
    ACC_ODR_200HZ  = 0xA9,
    ACC_ODR_400HZ  = 0xAA,
    ACC_ODR_800HZ  = 0xAB,
    ACC_ODR_1600HZ = 0xAC
};

enum GyrRange {
    GYR_RANGE_125DPS  = 0x04,
    GYR_RANGE_250DPS  = 0x03,
    GYR_RANGE_500DPS  = 0x02,
    GYR_RANGE_1000DPS = 0x01,
    GYR_RANGE_2000DPS = 0x00
};

enum GyrODR {
    GYR_ODR_100HZ  = 0x04,
    GYR_ODR_200HZ  = 0x03,
    GYR_ODR_400HZ  = 0x02,
    GYR_ODR_1000HZ = 0x01,
    GYR_ODR_2000HZ = 0x00
};


class BMI088 {
public:
    BMI088(TwoWire &w = Wire, uint8_t addrA = 0x18, uint8_t addrG = 0x68)
      : wire(w), ACC_ADDR(addrA), GYR_ADDR(addrG) {}

    bool begin() {
        wire.begin();
        wire.setClock(400000);

        // Soft reset accel and gyro
        write8(ACC_ADDR, BMI088_ACC_SOFTRESET, 0xB6); delay(2);
        write8(GYR_ADDR, BMI088_GYR_SOFTRESET, 0xB6); delay(35);

        // Power on accel
        write8(ACC_ADDR, BMI088_ACC_PWR_CONF, 0x00); delay(1);
        write8(ACC_ADDR, BMI088_ACC_PWR_CTRL, 0x04); delay(1);

        // Default accel config: ODR 100 Hz, ±6g
        write8(ACC_ADDR, BMI088_ACC_CONF, 0xA8);
        write8(ACC_ADDR, BMI088_ACC_RANGE, 0x01);
        accScale = 6.0f / 32768.0f;

        // Default gyro config: normal mode, 2000 dps, ODR ~200 Hz
        write8(GYR_ADDR, BMI088_GYR_LPM1, 0x00);
        write8(GYR_ADDR, BMI088_GYR_BANDWIDTH, 0x02);
        write8(GYR_ADDR, BMI088_GYR_RANGE, 0x00);
        gyrScale = 2000.0f / 32768.0f;

        // ID check
        uint8_t acc_id = read8(ACC_ADDR, BMI088_ACC_CHIP_ID);
        uint8_t gyr_id = read8(GYR_ADDR, BMI088_GYR_CHIP_ID);
        return (acc_id == 0x1E && gyr_id == 0x0F);
    }

    void update() {
        // Read accel
        uint8_t ba[6]; readN(ACC_ADDR, BMI088_ACC_DATA, ba, 6);
        accX = to16(ba[0],ba[1]) * accScale;
        accY = to16(ba[2],ba[3]) * accScale;
        accZ = to16(ba[4],ba[5]) * accScale;

        // Read gyro
        uint8_t bg[6]; readN(GYR_ADDR, BMI088_GYR_DATA, bg, 6);
        gyrX = to16(bg[0],bg[1]) * gyrScale;
        gyrY = to16(bg[2],bg[3]) * gyrScale;
        gyrZ = to16(bg[4],bg[5]) * gyrScale;
    }

    
    void setAccRange(AccRange range) {
        write8(ACC_ADDR, BMI088_ACC_RANGE, range);
        switch(range) {
            case ACC_RANGE_3G:  accScale = 3.0f/32768.0f; break;
            case ACC_RANGE_6G:  accScale = 6.0f/32768.0f; break;
            case ACC_RANGE_12G: accScale = 12.0f/32768.0f; break;
            case ACC_RANGE_24G: accScale = 24.0f/32768.0f; break;
        }
    }

    void setAccODR(AccODR odr) {
        write8(ACC_ADDR, BMI088_ACC_CONF, odr);
    }

    void setGyroRange(GyrRange range) {
        write8(GYR_ADDR, BMI088_GYR_RANGE, range);
        switch(range) {
            case GYR_RANGE_125DPS:  gyrScale = 1.0f/262.144f; break;
            case GYR_RANGE_250DPS:  gyrScale = 1.0f/131.072f; break;
            case GYR_RANGE_500DPS:  gyrScale = 1.0f/65.536f;  break;
            case GYR_RANGE_1000DPS: gyrScale = 1.0f/32.768f;  break;
            case GYR_RANGE_2000DPS: gyrScale = 1.0f/16.384f;  break;
        }
    }

    void setGyroODR(GyrODR odr) {
        write8(GYR_ADDR, BMI088_GYR_BANDWIDTH, odr);
    }


    // --- getters (like MPU6050_light style) ---
    float getAccX() { return accX; }
    float getAccY() { return accY; }
    float getAccZ() { return accZ; }

    float getGyroX() { return gyrX; }
    float getGyroY() { return gyrY; }
    float getGyroZ() { return gyrZ; }

    float getAccTempC() {
        uint8_t bt[2]; readN(ACC_ADDR, BMI088_ACC_TEMP, bt, 2);
        int16_t raw = to16(bt[0], bt[1]);
        return (raw * 0.0039f) + 23.0f;  // datasheet conversion
    }

    float getGyroTempC() {
        uint8_t bt[2]; readN(GYR_ADDR, BMI088_GYR_TEMP, bt, 2);
        int16_t raw = to16(bt[0], bt[1]);
        return 23.0f + (raw / 512.0f);
    }

private:
    TwoWire &wire;
    uint8_t ACC_ADDR, GYR_ADDR;
    float accScale, gyrScale;

    float accX, accY, accZ;
    float gyrX, gyrY, gyrZ;

    void write8(uint8_t addr,uint8_t reg,uint8_t val) {
        wire.beginTransmission(addr);
        wire.write(reg);
        wire.write(val);
        wire.endTransmission();
    }

    uint8_t read8(uint8_t addr,uint8_t reg) {
        wire.beginTransmission(addr);
        wire.write(reg);
        wire.endTransmission(false);
        wire.requestFrom(addr,(uint8_t)1);
        return wire.available()?wire.read():0xFF;
    }

    void readN(uint8_t addr,uint8_t reg,uint8_t *buf,size_t n) {
        wire.beginTransmission(addr);
        wire.write(reg);
        wire.endTransmission(false);
        wire.requestFrom(addr,(uint8_t)n);
        for(size_t i=0;i<n && wire.available();i++) buf[i]=wire.read();
    }

    static int16_t to16(uint8_t l,uint8_t h) {
        return (int16_t)((h<<8)|l);
    }
};

#endif
