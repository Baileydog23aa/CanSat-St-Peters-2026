#ifndef PERSISTENT_STATE_H
#define PERSISTENT_STATE_H

#include <Arduino.h>

#define PSTATE_MAGIC 0xC0FFEEABUL
#define PSTATE_PATH  "/flight_state.bin"

// Everything needed to resume an in-progress flight after an unexpected
// reset WITHOUT re-running the at-rest calibration / range sweep, and
// without losing packet count or mission elapsed time (F1 / F2).
struct PersistentState {
  uint32_t magic;             // Checking that the read from Pstate is accurate
  uint8_t  flightActive;      // 1 while a flight is armed/in-progress and not yet safely closed out
  uint8_t  flightState;       // FlightState enum value
  uint8_t  accRangeIndex;     // index into ACC_RANGE_TABLE
  uint8_t  gyrRangeIndex;     // index into GYR_RANGE_TABLE
  uint16_t packetCount;
  uint32_t missionElapsedMs;  // accumulated mission time from prior boots
  float    groundPressurePa;  // zero-altitude reference pressure
  float    maxAltitudeM;      // apogee tracker
  float    gyroBiasX, gyroBiasY, gyroBiasZ;
  float    accelBiasX, accelBiasY, accelBiasZ;
  float    velX, velY, velZ;
  float    dispX, dispY, dispZ;
  float    roll, pitch, yaw;
  uint8_t  aerobrakeDeployed;
  uint8_t  parachuteDeployed;
  uint8_t  checksum;          // XOR of all preceding bytes
};

#endif
