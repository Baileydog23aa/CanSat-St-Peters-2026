#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Arduino.h>

// ---------------------------------------------------------------------
// This struct MUST stay byte-identical (same types, same order, same
// packing) to the Telemetry struct actually used to parse incoming bytes
// in the ground station .ino (the local struct declared inside loop(),
// which is the one E32.readBytes() fills in - note it uses int32_t for
// GPS, not the uint32_t used in the earlier/unused global struct).
// ---------------------------------------------------------------------
struct Telemetry {
  uint16_t Time;              // deciseconds of mission elapsed time (Time/10.0 = seconds)
  uint8_t  PacketCount;       // wraps at 255 - limitation of the ground struct's uint8_t field
  uint8_t  Mode;              // FlightState enum value (see FlightState.h)
  uint16_t Altitude;          // decimeters AGL -> Altitude/10.0 = meters (0.1 m resolution)
  uint16_t VerticalVelocity;  // deci-m/s, magnitude of vertical speed -> /10.0 = m/s
  int32_t  GPSLat;            // micro-degrees (deg * 1e6) -> /1000000.0 = decimal degrees
  int32_t  GPSLon;            // micro-degrees (deg * 1e6) -> /1000000.0 = decimal degrees
  uint16_t HDOPSats;          // encoded: HDOPSats = round(HDOP*10)*11 + numSats(0-10)
                               //   ground decodes: sats = HDOPSats % 11; HDOP = (HDOPSats/11)/10.0
  uint8_t  Voltage;           // deci-volts -> /10.0 = volts
  uint8_t  EnabledItems;      // bit0 Baro, bit1 IMU-Accel, bit2 IMU-Gyro, bit3 GPS, bit4 LittleFS, bit5 SD
  uint8_t  Checksum;          // XOR of all preceding bytes in this struct
} __attribute__((packed));

enum EnabledBits : uint8_t {
  BIT_BARO    = 0x01,
  BIT_IMU_ACC = 0x02,
  BIT_IMU_GYR = 0x04,
  BIT_GPS     = 0x08,
  BIT_FLASH   = 0x10, // LittleFS persistent-state store
  BIT_SD      = 0x20
};

enum FlightState : uint8_t {
  STATE_LAUNCH_WAIT       = 0,
  STATE_ASCENT            = 1,
  STATE_DESCENT_AEROBRAKE = 2,
  STATE_DESCENT_PARACHUTE = 3,
  STATE_LANDED            = 4
};

inline const char* flightStateName(uint8_t s) {
  switch (s) {
    case STATE_LAUNCH_WAIT:       return "LAUNCH_WAIT";
    case STATE_ASCENT:            return "ASCENT";
    case STATE_DESCENT_AEROBRAKE: return "DESCENT_AEROBRAKE";
    case STATE_DESCENT_PARACHUTE: return "DESCENT_PARACHUTE";
    case STATE_LANDED:            return "LANDED";
    default:                      return "UNKNOWN";
  }
}

#endif
