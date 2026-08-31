// =============================================================================
// CANSAT 2026 - FLIGHT COMPUTER (DUAL-CORE)
// Target: RP2040 (per your PCB), Arduino-Pico core (Earle Philhower)
//
// CORE ASSIGNMENT
//   Core 0 (setup()/loop())   - "everything else": BMI088 auto-ranging +
//                                integration, barometer altitude, flight
//                                state machine, deployment stubs, SD
//                                logging, LittleFS crash-resume, USB
//                                Serial debug print.
//   Core 1 (setup1()/loop1()) - GPS (L80-R) and the E32-433T-33D telemetry
//                                radio, both on SoftwareSerial, plus
//                                building/sending the telemetry packet at
//                                a max of 3 Hz.
//
// The two cores never share the I2C bus, SPI bus, or ADC directly - all of
// that stays on Core 0. Data crosses cores only through two small,
// mutex-protected snapshot structs (g_flight, g_comms) defined below.
//
// Library dependencies (install via Library Manager):
//   - MS5611 (by Rob Tillaart)                              [barometer, TE MS5611]
//   - TinyGPSPlus                                           [GPS/NMEA]
//   - SD, LittleFS, SoftwareSerial (bundled with Arduino-Pico core)
// =============================================================================

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <LittleFS.h>
#include <SoftwareSerial.h>
#include <MS5611.h>
#include <TinyGPSPlus.h>
#include "pico/mutex.h"
#include <SerialPIO.h>
#include <Servo.h>

#include "Config.h"
#include "Telemetry.h"
#include "PersistentState.h"
#include "BMI088.h"

// =============================================================================
// CROSS-CORE SHARED STATE
// =============================================================================

// Written by Core 0 every sample tick, read by Core 1 to build telemetry.
struct FlightSnapshot {
  uint8_t  flightState;
  float    altitudeAGL;
  float    verticalSpeedEst;
  float    batteryVoltage;
  uint32_t missionElapsedMs;
  uint8_t  enabledBitsCore0; // baro/imu/flash/sd bits; Core 1 ORs in the GPS bit
};

// Written by Core 1 (GPS fixes + packet count as it sends), read by Core 0
// for SD logging / Serial print / LittleFS persistence.
struct CommsSnapshot {
  double   gpsLat, gpsLon;
  uint8_t  gpsSats;
  float    gpsHdop;
  bool     gpsValid;
  uint16_t packetCountFull;
};

FlightSnapshot g_flight = {STATE_LAUNCH_WAIT, 0, 0, 0, 0, 0};
CommsSnapshot  g_comms  = {0, 0, 0, 99.9f, false, 0};
mutex_t g_flightMutex;
mutex_t g_commsMutex;

// One-shot handshake: Core 1 waits for Core 0 to finish LittleFS
// resume/calibration (and to publish any resumed packet count) before it
// starts sending telemetry.
volatile bool g_bootReady = false;

FlightSnapshot getFlightSnapshot() {
  mutex_enter_blocking(&g_flightMutex);
  FlightSnapshot copy = g_flight;
  mutex_exit(&g_flightMutex);
  return copy;
}
void setFlightSnapshot(const FlightSnapshot& s) {
  mutex_enter_blocking(&g_flightMutex);
  g_flight = s;
  mutex_exit(&g_flightMutex);
}
CommsSnapshot getCommsSnapshot() {
  mutex_enter_blocking(&g_commsMutex);
  CommsSnapshot copy = g_comms;
  mutex_exit(&g_commsMutex);
  return copy;
}
void setCommsGPS(double lat, double lon, uint8_t sats, float hdop, bool valid) {
  mutex_enter_blocking(&g_commsMutex);
  g_comms.gpsLat = lat; g_comms.gpsLon = lon;
  g_comms.gpsSats = sats; g_comms.gpsHdop = hdop; g_comms.gpsValid = valid;
  mutex_exit(&g_commsMutex);
}
void setCommsPacketCount(uint16_t pc) {
  mutex_enter_blocking(&g_commsMutex);
  g_comms.packetCountFull = pc;
  mutex_exit(&g_commsMutex);
}

uint8_t computeChecksum(const uint8_t* data, size_t len) {
  uint8_t c = 0;
  for (size_t i = 0; i < len; i++) c ^= data[i];
  return c;
}

// =============================================================================
// GLOBAL OBJECTS
// =============================================================================
BMI088 imu;
MS5611 baro(BARO_I2C_ADDR);
File sdLogFile;
PersistentState pstate;

SoftwareSerial GPS_SW_SERIAL(GPS_SW_RX_PIN, GPS_SW_TX_PIN);
SerialPIO TELEM_SW_SERIAL(TELEM_SW_RX_PIN, TELEM_SW_TX_PIN);
TinyGPSPlus gps; // only ever touched on Core 1
Servo Servo1;
Servo Servo2;
Servo Servo3;

// ---------------------------------------------------------------------------
// ACCEL / GYRO AUTO-RANGE TABLES (Core 0 only)
// ---------------------------------------------------------------------------
const AccRange ACC_RANGE_TABLE[4]   = { ACC_RANGE_3G, ACC_RANGE_6G, ACC_RANGE_12G, ACC_RANGE_24G };
const float    ACC_RANGE_MAX_G[4]   = { 3.0f, 6.0f, 12.0f, 24.0f };
const GyrRange GYR_RANGE_TABLE[5]   = { GYR_RANGE_125DPS, GYR_RANGE_250DPS, GYR_RANGE_500DPS, GYR_RANGE_1000DPS, GYR_RANGE_2000DPS };
const float    GYR_RANGE_MAX_DPS[5] = { 125.0f, 250.0f, 500.0f, 1000.0f, 2000.0f };

uint8_t accRangeIndex = 1; // start at +/-6G
uint8_t gyrRangeIndex = 4; // start at 2000 dps (safe for violent separation tumbling)
int8_t  accRangeCooldown = 0;
int8_t  gyrRangeCooldown = 0;
#define RANGE_CHANGE_COOLDOWN_SAMPLES 20

// ---------------------------------------------------------------------------
// SENSOR FUSION / INTEGRATION STATE (Core 0 only)
// ---------------------------------------------------------------------------
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
float accelBiasX = 0, accelBiasY = 0, accelBiasZ = 0; // includes local gravity component at pad orientation
float velX = 0, velY = 0, velZ = 0;
float dispX = 0, dispY = 0, dispZ = 0;
float roll = 0, pitch = 0, yaw = 0;
float prevAxMs2 = 0, prevAyMs2 = 0, prevAzMs2 = 0;

float groundPressurePa = 101325.0f;
float altitudeAGL = 0.0f;
float prevAltitudeAGL = 0.0f;
float verticalSpeedEst = 0.0f; // magnitude, m/s (from integrated accel, see updateIntegration)
float maxAltitudeM = 0.0f;

uint8_t flightState = STATE_LAUNCH_WAIT;
uint8_t descendingSampleCount = 0;
uint8_t calmSampleCount = 0;
bool    aerobrakeDeployed = false;
bool    parachuteDeployed = false;

bool baroOK = false, imuOK = false, sdOK = false, flashOK = false;

uint32_t missionStartOffsetMs = 0; // restored from LittleFS on resume (F2)

unsigned long lastSampleMs = 0;
unsigned long lastLogMs = 0;
unsigned long lastStateSaveMs = 0;
unsigned long lastMicros = 0;

bool resumedFlight = false;
int TELEM_INTERVAL_MS = TELEM_INTERVAL_MS_Flight;

bool ARMED = false;
float verticalSpeedBaro = 0.0f;
unsigned long lastAltUs = 0;

// =============================================================================
// CORE 0 SETUP: sensors, integration, SD/LittleFS, flight logic
// =============================================================================
void setup() {
  Serial.begin(DEBUG_BAUD);
  // while (!Serial);
  delay(200);
  Serial.println(F("=== CANSAT 2026 Flight Computer Boot (Core 0) ==="));

  mutex_init(&g_flightMutex);
  mutex_init(&g_commsMutex);

  Wire.begin();

  pinMode(PIN_AEROBRAKE_ACTUATOR, OUTPUT);
  pinMode(PIN_PARACHUTE_ACTUATOR, OUTPUT);
  Servo1.attach(Servo1Pin);
  Servo2.attach(Servo2Pin);
  Servo3.attach(Servo3Pin);
  analogReadResolution(12);

  Servo1.write(170);

  // --- LittleFS: check for an incomplete prior flight ---
  flashOK = LittleFS.begin();
  if (!flashOK) {
    Serial.println(F("LittleFS mount failed - formatting..."));
    LittleFS.format();
    flashOK = LittleFS.begin();
  }

  bool haveResumableState = flashOK && loadPersistentState();

  // --- IMU init ---
  imuOK = imu.begin();
  Serial.println(imuOK ? F("BMI088 OK") : F("BMI088 INIT FAILED"));

  // --- Barometer init (MS5611) ---
  baroOK = baro.begin();
  Serial.println(baroOK ? F("Barometer (MS5611) OK") : F("Barometer (MS5611) INIT FAILED"));

  // --- SD init ---
  sdOK = SD.begin(PIN_SD_CS);
  Serial.println(sdOK ? F("SD OK") : F("SD INIT FAILED"));
  if (sdOK) {
    bool newFile = !SD.exists(SD_LOG_FILENAME);
    sdLogFile = SD.open(SD_LOG_FILENAME, FILE_WRITE);
    if (sdLogFile && newFile) {
      sdLogFile.println(F("MissionTime_s,PacketCount,State,Altitude_m,VerticalSpeed_ms,"
                           "VelX,VelY,VelZ,DispX,DispY,DispZ,Roll_deg,Pitch_deg,Yaw_deg,"
                           "AccRange_g,GyrRange_dps,Lat,Lon,Sats,HDOP,BatteryV,GroundPressurePa"));
      sdLogFile.flush();
    }
  }

  uint16_t resumedPacketCount = 0;

  if (haveResumableState) {
    // ---------------------------------------------------------------------
    // RESUME PATH: a previous flight was in progress when we last saved.
    // Skip the at-rest calibration / range sweep entirely and go straight
    // to flight logic with the saved ranges, biases, and integrator state.
    // ---------------------------------------------------------------------
    resumedFlight = true;
    ARMED = true;
    Serial.println(F(">>> Incomplete flight detected in LittleFS - RESUMING without recalibration <<<"));

    accRangeIndex        = pstate.accRangeIndex;
    gyrRangeIndex         = pstate.gyrRangeIndex;
    flightState           = pstate.flightState;
    resumedPacketCount    = pstate.packetCount;
    missionStartOffsetMs  = pstate.missionElapsedMs;
    groundPressurePa      = pstate.groundPressurePa;
    maxAltitudeM           = pstate.maxAltitudeM;
    gyroBiasX = pstate.gyroBiasX; gyroBiasY = pstate.gyroBiasY; gyroBiasZ = pstate.gyroBiasZ;
    accelBiasX = pstate.accelBiasX; accelBiasY = pstate.accelBiasY; accelBiasZ = pstate.accelBiasZ;
    velX = pstate.velX; velY = pstate.velY; velZ = pstate.velZ;
    dispX = pstate.dispX; dispY = pstate.dispY; dispZ = pstate.dispZ;
    roll = pstate.roll; pitch = pstate.pitch; yaw = pstate.yaw;
    aerobrakeDeployed = pstate.aerobrakeDeployed;
    parachuteDeployed = pstate.parachuteDeployed;

    if (imuOK) {
      imu.setAccRange(ACC_RANGE_TABLE[accRangeIndex]);
      imu.setGyroRange(GYR_RANGE_TABLE[gyrRangeIndex]);
    }
  } else {
    // ---------------------------------------------------------------------
    // COLD START: fresh power-up on the pad. Run at-rest calibration.
    // ---------------------------------------------------------------------
    Serial.println(F("Cold start - running at-rest calibration..."));
    if (imuOK) {
      imu.setAccRange(ACC_RANGE_TABLE[accRangeIndex]);
      imu.setGyroRange(GYR_RANGE_TABLE[gyrRangeIndex]);
      calibrateAtRest();
    }
    if (baroOK) {
      groundPressurePa = averageGroundPressure();
    }
    flightState = STATE_LAUNCH_WAIT;
    resumedPacketCount = 0;
    missionStartOffsetMs = 0;
    maxAltitudeM = 0;
  }

  // Publish the (possibly resumed) packet count so Core 1 starts counting
  // from the right place, then release Core 1 to begin its setup1()/loop1().
  setCommsPacketCount(resumedPacketCount);
  publishFlightSnapshot();
  savePersistentState();
  g_bootReady = true;

  lastMicros = micros();
lastAltUs = lastMicros;
  lastSampleMs = lastLogMs = lastStateSaveMs = millis();

  Serial.println(F("=== Core 0 setup complete, entering flight loop ==="));
  if (!haveResumableState){
    TELEM_INTERVAL_MS = TELEM_INTERVAL_MS_Idle;
    while (!ARMED){
      unsigned long nowMs = millis();
      if (nowMs - lastSampleMs >= SAMPLE_INTERVAL_MS) {
        lastSampleMs = nowMs;
        sampleAndIntegrate();
        updateAltitude();
        publishFlightSnapshot();
      }
    }
    Serial.println("Arming CanSat");

    velX = velY = velZ = 0.0f;
    dispX = dispY = dispZ = 0.0f;
    altitudeAGL = 0.0f;
    prevAltitudeAGL = 0.0f;
    maxAltitudeM = 0.0f;
    
    Telemetry t;
    t.Time             = 0; // deciseconds
    t.PacketCount       = (uint8_t)(0);
    t.Mode              = 0;
    t.Altitude          = 0;
    t.VerticalVelocity  = 0;
    t.GPSLat            = (int32_t)(0 * 1000000.0);
    t.GPSLon            = (int32_t)(0 * 1000000.0);

    t.HDOPSats          = 0;

    t.Voltage           = 0;

    t.EnabledItems       = 0;

    TELEM_SW_SERIAL.write((uint8_t*)&t, sizeof(Telemetry));
    TELEM_INTERVAL_MS = TELEM_INTERVAL_MS_Flight;
  }
}

// =============================================================================
// CORE 0 LOOP
// =============================================================================
void loop() {
  unsigned long nowMs = millis();

  if (nowMs - lastSampleMs >= SAMPLE_INTERVAL_MS) {
    lastSampleMs = nowMs;
    sampleAndIntegrate();
    updateAltitude();
    updateFlightState();
    publishFlightSnapshot();
  }

  if (nowMs - lastLogMs >= LOG_INTERVAL_MS) {
    lastLogMs = nowMs;
    logToSD();
    printToSerial();
  }

  if (nowMs - lastStateSaveMs >= STATE_SAVE_INTERVAL_MS) {
    lastStateSaveMs = nowMs;
    savePersistentState();
  }
}

// Push the latest Core-0-owned flight data to Core 1 for telemetry.
void publishFlightSnapshot() {
  FlightSnapshot s;
  s.flightState        = flightState;
  s.altitudeAGL         = altitudeAGL;
  s.verticalSpeedEst    = verticalSpeedEst;
  s.batteryVoltage      = readBatteryVoltage();
  s.missionElapsedMs    = missionStartOffsetMs + millis();

  uint8_t enabled = 0;
  if (baroOK)  enabled |= BIT_BARO;
  if (imuOK)   enabled |= BIT_IMU_ACC | BIT_IMU_GYR;
  if (flashOK) enabled |= BIT_FLASH;
  if (sdOK)    enabled |= BIT_SD;
  s.enabledBitsCore0 = enabled;

  setFlightSnapshot(s);
}

// =============================================================================
// CALIBRATION (cold start only, Core 0)
// =============================================================================
void calibrateAtRest() {
  float sax = 0, say = 0, saz = 0, sgx = 0, sgy = 0, sgz = 0;
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    imu.update();
    sax += imu.getAccX(); say += imu.getAccY(); saz += imu.getAccZ();
    sgx += imu.getGyroX(); sgy += imu.getGyroY(); sgz += imu.getGyroZ();
    delay(10);
  }
  accelBiasX = sax / CALIBRATION_SAMPLES;
  accelBiasY = say / CALIBRATION_SAMPLES;
  accelBiasZ = saz / CALIBRATION_SAMPLES; // includes ~1g gravity component at pad orientation
  gyroBiasX = sgx / CALIBRATION_SAMPLES;
  gyroBiasY = sgy / CALIBRATION_SAMPLES;
  gyroBiasZ = sgz / CALIBRATION_SAMPLES;
  Serial.println(F("Calibration complete."));
}

// MS5611.read() triggers a conversion + reads the result; getPressure()
// then returns millibar (hPa), so it's converted to Pa here to keep the
// rest of the altitude math (which expects Pa) unchanged.
float readBaroPressurePa() {
  if (baro.read() != MS5611_READ_OK) {
    return groundPressurePa; // fall back to last known-good reference on a bad read
  }
  return baro.getPressure() * 100.0f;
}

float averageGroundPressure() {
  float sum = 0;
  const int N = 50;
  for (int i = 0; i < N; i++) {
    sum += readBaroPressurePa();
    delay(10);
  }
  return sum / N;
}

// =============================================================================
// AUTO-RANGING BMI088 (Core 0)
// =============================================================================
void autoRangeAccel(float ax, float ay, float az) {
  float peak = max(fabs(ax), max(fabs(ay), fabs(az)));
  float rangeG = ACC_RANGE_MAX_G[accRangeIndex];

  if (accRangeCooldown > 0) accRangeCooldown--;

  if (peak > 0.92f * rangeG && accRangeIndex < 3) {
    accRangeIndex++;
    imu.setAccRange(ACC_RANGE_TABLE[accRangeIndex]);
    accRangeCooldown = RANGE_CHANGE_COOLDOWN_SAMPLES;
  } else if (accRangeCooldown == 0 && accRangeIndex > 0 &&
             peak < 0.35f * ACC_RANGE_MAX_G[accRangeIndex - 1]) {
    accRangeIndex--;
    imu.setAccRange(ACC_RANGE_TABLE[accRangeIndex]);
    accRangeCooldown = RANGE_CHANGE_COOLDOWN_SAMPLES;
  }
}

void autoRangeGyro(float gx, float gy, float gz) {
  float peak = max(fabs(gx), max(fabs(gy), fabs(gz)));
  float rangeDps = GYR_RANGE_MAX_DPS[gyrRangeIndex];

  if (gyrRangeCooldown > 0) gyrRangeCooldown--;

  if (peak > 0.92f * rangeDps && gyrRangeIndex < 4) {
    gyrRangeIndex++;
    imu.setGyroRange(GYR_RANGE_TABLE[gyrRangeIndex]);
    gyrRangeCooldown = RANGE_CHANGE_COOLDOWN_SAMPLES;
  } else if (gyrRangeCooldown == 0 && gyrRangeIndex > 0 &&
             peak < 0.35f * GYR_RANGE_MAX_DPS[gyrRangeIndex - 1]) {
    gyrRangeIndex--;
    imu.setGyroRange(GYR_RANGE_TABLE[gyrRangeIndex]);
    gyrRangeCooldown = RANGE_CHANGE_COOLDOWN_SAMPLES;
  }
}

// =============================================================================
// SAMPLE + INTEGRATE (Core 0): accel -> velocity/displacement, gyro -> attitude
// =============================================================================
void sampleAndIntegrate() {
  if (!imuOK) return;

  unsigned long nowUs = micros();
  float dt = (nowUs - lastMicros) / 1000000.0f;
  lastMicros = nowUs;
  if (dt <= 0 || dt > 0.5f) dt = SAMPLE_INTERVAL_MS / 1000.0f; // guard first sample / rollover

  imu.update();
  float ax = imu.getAccX(), ay = imu.getAccY(), az = imu.getAccZ();
  float gx = imu.getGyroX(), gy = imu.getGyroY(), gz = imu.getGyroZ();

  autoRangeAccel(ax, ay, az);
  autoRangeGyro(gx, gy, gz);

  // --- Attitude: simple gyro integration (drifts without a magnetometer/accel
  //     fusion, but satisfies "integrate angular velocity to angular position"
  //     and gives a usable tilt/stability indicator for the descent phase). ---
  roll  += (gx - gyroBiasX) * dt;
  pitch += (gy - gyroBiasY) * dt;
  yaw   += (gz - gyroBiasZ) * dt;

  // --- Velocity / displacement: bias-removed accel, trapezoidal integration.
  //     NOTE: this is dead reckoning in the body frame with no orientation
  //     rotation applied, so it will drift steadily - it's a supplementary
  //     estimate, NOT the primary altitude source (the barometer is). ---
  float axMs2 = (ax - accelBiasX) * G_MS2;
  float ayMs2 = (ay - accelBiasY) * G_MS2;
  float azMs2 = (az - accelBiasZ) * G_MS2;

  velX += 0.5f * (axMs2 + prevAxMs2) * dt;
  velY += 0.5f * (ayMs2 + prevAyMs2) * dt;
  velZ += 0.5f * (azMs2 + prevAzMs2) * dt;

  dispX += velX * dt;
  dispY += velY * dt;
  dispZ += velZ * dt;

  prevAxMs2 = axMs2; prevAyMs2 = ayMs2; prevAzMs2 = azMs2;

  // Vertical speed estimate used for the telemetry "air speed" field
  verticalSpeedEst = sqrtf(velX * velX + velY * velY + velZ * velZ);
}

// =============================================================================
// ALTITUDE (barometer, primary source, Core 0)
// =============================================================================
void updateAltitude() {  
  if (!baroOK) return;

  unsigned long nowUs = micros();
  float dt = (nowUs - lastAltUs) / 1000000.0f;
  lastAltUs = nowUs;

  prevAltitudeAGL = altitudeAGL;
  float pressurePa = readBaroPressurePa();
  altitudeAGL = 44330.0f * (1.0f - powf(pressurePa / groundPressurePa, 0.19026f));
  if (altitudeAGL > maxAltitudeM) maxAltitudeM = altitudeAGL;

  if (dt > 0 && dt < 1.0f) {
    float rawRate = (altitudeAGL - prevAltitudeAGL) / dt;
    verticalSpeedBaro = 0.8f * verticalSpeedBaro + 0.2f * rawRate; // light low-pass
  }
}

// =============================================================================
// FLIGHT STATE MACHINE (Core 0)
// =============================================================================
void updateFlightState() {
  switch (flightState) {

    case STATE_LAUNCH_WAIT:
      if (altitudeAGL - prevAltitudeAGL > 0 && verticalSpeedEst > LIFTOFF_VVEL_THRESHOLD_MS) {
        flightState = STATE_ASCENT;
        Serial.println(F("[STATE] -> ASCENT"));
      }
      break;

    case STATE_ASCENT:
      if (altitudeAGL < prevAltitudeAGL) {
        descendingSampleCount++;
      } else {
        descendingSampleCount = 0;
      }
      if (descendingSampleCount >= APOGEE_DESCEND_CONFIRM_N) {
        flightState = STATE_DESCENT_AEROBRAKE;
        Serial.println(F("[STATE] -> DESCENT_AEROBRAKE (apogee/separation detected)"));
        if (!aerobrakeDeployed) {
          deployAerobrake();
          aerobrakeDeployed = true;
        }
      }
      break;

    case STATE_DESCENT_AEROBRAKE:
      if (altitudeAGL <= PARACHUTE_DEPLOY_ALT_M) {
        flightState = STATE_DESCENT_PARACHUTE;
        Serial.println(F("[STATE] -> DESCENT_PARACHUTE (150 m AGL)"));
        if (!parachuteDeployed) {
          deployParachute();
          parachuteDeployed = true;
        }
      }
      break;

    case STATE_DESCENT_PARACHUTE:
      if (altitudeAGL < LANDED_ALT_THRESHOLD_M/* && fabsf(verticalSpeedBaro) < LANDED_VVEL_THRESHOLD_MS*/) {
        calmSampleCount++;
      } else {
        calmSampleCount = 0;
      }
      if (calmSampleCount >= LANDED_CONFIRM_N) {
        flightState = STATE_LANDED;
        Serial.println(F("[STATE] -> LANDED"));
        savePersistentState(); // marks flightActive = 0, see fillPersistentStateFromLive()
      }
      break;

    case STATE_LANDED:
      TELEM_INTERVAL_MS = TELEM_INTERVAL_MS_Idle;
      // Idle. Could add periodic audio beacon trigger here (C4 requirement).
      break;
  }
}

// =============================================================================
// DEPLOYMENT MECHANISMS (Core 0)
// =============================================================================

// Called once, automatically, when the CANSAT is confirmed descending after
// apogee/rocket separation (mission guide M5).
void deployAerobrake() {
  // TODO: implement your aerobrake deployment mechanism here.
  // e.g. digitalWrite(PIN_AEROBRAKE_ACTUATOR, HIGH);
}

// Called once, automatically, when the CANSAT descends through
// PARACHUTE_DEPLOY_ALT_M (150 m AGL), per mission guide C3 / M6.
void deployParachute() {
  // TODO: implement your main parachute deployment mechanism here.
  // e.g. digitalWrite(PIN_PARACHUTE_ACTUATOR, HIGH);
  Servo1.write(90);
}

// =============================================================================
// BATTERY (Core 0 owns the ADC)
// =============================================================================
float readBatteryVoltage() {
  int raw = analogRead(PIN_BATT_ADC);
  float vAtPin = (raw / ADC_MAX_COUNTS) * ADC_REF_VOLTAGE;
  float ratio = (VDIV_TOP_OHMS + VDIV_BOTTOM_OHMS) / VDIV_BOTTOM_OHMS;
  return vAtPin * ratio;
}

// =============================================================================
// LOGGING (SD card CSV, full-resolution backup) + SERIAL PRINT (Core 0)
// =============================================================================
void logToSD() {
  if (!sdOK || !sdLogFile) return;
  CommsSnapshot comms = getCommsSnapshot();
  uint32_t missionElapsedMs = missionStartOffsetMs + millis();

  sdLogFile.print(missionElapsedMs / 1000.0, 3); sdLogFile.print(',');
  sdLogFile.print(comms.packetCountFull); sdLogFile.print(',');
  sdLogFile.print(flightStateName(flightState)); sdLogFile.print(',');
  sdLogFile.print(altitudeAGL, 2); sdLogFile.print(',');
  sdLogFile.print(verticalSpeedEst, 2); sdLogFile.print(',');
  sdLogFile.print(velX, 3); sdLogFile.print(','); sdLogFile.print(velY, 3); sdLogFile.print(','); sdLogFile.print(velZ, 3); sdLogFile.print(',');
  sdLogFile.print(dispX, 3); sdLogFile.print(','); sdLogFile.print(dispY, 3); sdLogFile.print(','); sdLogFile.print(dispZ, 3); sdLogFile.print(',');
  sdLogFile.print(roll, 2); sdLogFile.print(','); sdLogFile.print(pitch, 2); sdLogFile.print(','); sdLogFile.print(yaw, 2); sdLogFile.print(',');
  sdLogFile.print(ACC_RANGE_MAX_G[accRangeIndex], 1); sdLogFile.print(',');
  sdLogFile.print(GYR_RANGE_MAX_DPS[gyrRangeIndex], 1); sdLogFile.print(',');
  sdLogFile.print(comms.gpsLat, 6); sdLogFile.print(','); sdLogFile.print(comms.gpsLon, 6); sdLogFile.print(',');
  sdLogFile.print(comms.gpsSats); sdLogFile.print(','); sdLogFile.print(comms.gpsHdop, 1); sdLogFile.print(',');
  sdLogFile.print(readBatteryVoltage(), 2); sdLogFile.print(',');
  sdLogFile.println(groundPressurePa, 1);
  sdLogFile.flush();
}

void printToSerial() {
  CommsSnapshot comms = getCommsSnapshot();
  uint32_t missionElapsedMs = missionStartOffsetMs + millis();
  Serial.print(F("t=")); Serial.print(missionElapsedMs / 1000.0, 2);
  Serial.print(F("s  pkt=")); Serial.print(comms.packetCountFull);
  Serial.print(F("  state=")); Serial.print(flightStateName(flightState));
  Serial.print(F("  alt=")); Serial.print(altitudeAGL, 2); Serial.print(F("m"));
  Serial.print(F("  vSpeed=")); Serial.print(verticalSpeedEst, 2); Serial.print(F("m/s"));
  Serial.print(F("  vel=(")); Serial.print(velX, 2); Serial.print(','); Serial.print(velY, 2); Serial.print(','); Serial.print(velZ, 2); Serial.print(F(")"));
  Serial.print(F("  disp=(")); Serial.print(dispX, 2); Serial.print(','); Serial.print(dispY, 2); Serial.print(','); Serial.print(dispZ, 2); Serial.print(F(")"));
  Serial.print(F("  att=(r")); Serial.print(roll, 1); Serial.print(F(",p")); Serial.print(pitch, 1); Serial.print(F(",y")); Serial.print(yaw, 1); Serial.print(F(")"));
  Serial.print(F("  accRange=+-")); Serial.print(ACC_RANGE_MAX_G[accRangeIndex], 0); Serial.print(F("g"));
  Serial.print(F("  gyrRange=+-")); Serial.print(GYR_RANGE_MAX_DPS[gyrRangeIndex], 0); Serial.print(F("dps"));
  Serial.print(F("  gps=(")); Serial.print(comms.gpsLat, 5); Serial.print(','); Serial.print(comms.gpsLon, 5); Serial.print(F(") sats=")); Serial.print(comms.gpsSats);
  Serial.print(F(" hdop=")); Serial.print(comms.gpsHdop, 1);
  Serial.print(F("  batt=")); Serial.print(readBatteryVoltage(), 2); Serial.print(F("V"));
  Serial.println();
}

// =============================================================================
// LITTLEFS PERSISTENCE (Core 0 only - crash/reset resume)
// =============================================================================
void fillPersistentStateFromLive() {
  CommsSnapshot comms = getCommsSnapshot();

  pstate.magic             = PSTATE_MAGIC;
  pstate.flightActive      = (flightState != STATE_LANDED) ? 1 : 0;
  pstate.flightState       = flightState;
  pstate.accRangeIndex     = accRangeIndex;
  pstate.gyrRangeIndex     = gyrRangeIndex;
  pstate.packetCount       = comms.packetCountFull;
  pstate.missionElapsedMs  = missionStartOffsetMs + millis();
  pstate.groundPressurePa  = groundPressurePa;
  pstate.maxAltitudeM      = maxAltitudeM;
  pstate.gyroBiasX = gyroBiasX; pstate.gyroBiasY = gyroBiasY; pstate.gyroBiasZ = gyroBiasZ;
  pstate.accelBiasX = accelBiasX; pstate.accelBiasY = accelBiasY; pstate.accelBiasZ = accelBiasZ;
  pstate.velX = velX; pstate.velY = velY; pstate.velZ = velZ;
  pstate.dispX = dispX; pstate.dispY = dispY; pstate.dispZ = dispZ;
  pstate.roll = roll; pstate.pitch = pitch; pstate.yaw = yaw;
  pstate.aerobrakeDeployed = aerobrakeDeployed ? 1 : 0;
  pstate.parachuteDeployed = parachuteDeployed ? 1 : 0;
  pstate.checksum = computeChecksum((uint8_t*)&pstate, sizeof(PersistentState) - 1);
}

void savePersistentState() {
  if (!flashOK) return;
  fillPersistentStateFromLive();
  File f = LittleFS.open(PSTATE_PATH, "w");
  if (!f) return;
  f.write((uint8_t*)&pstate, sizeof(PersistentState));
  f.close();
}

// Returns true if a valid, in-progress flight state was loaded into `pstate`. (true if flight in progress)
bool loadPersistentState() {
  if (!LittleFS.exists(PSTATE_PATH)) return false;
  File f = LittleFS.open(PSTATE_PATH, "r");
  if (!f) return false;
  if (f.size() != sizeof(PersistentState)) { f.close(); return false; }
  f.read((uint8_t*)&pstate, sizeof(PersistentState));
  f.close();

  if (pstate.magic != PSTATE_MAGIC) return false;
  uint8_t chk = computeChecksum((uint8_t*)&pstate, sizeof(PersistentState) - 1);
  if (chk != pstate.checksum) return false;
  if (!pstate.flightActive) return false; // last flight closed out cleanly - do a normal cold start

  return true;
}

// =============================================================================
// CORE 1 SETUP: GPS + E32 telemetry, both on SoftwareSerial
// =============================================================================
unsigned long lastTelemMs = 0;
uint16_t core1PacketCountFull = 0;

void setup1() {
  // Wait for Core 0 to finish LittleFS resume/calibration and publish the
  // (possibly resumed) packet count before we start transmitting.
  while (!g_bootReady) {
    delay(5);
  }

  GPS_SW_SERIAL.begin(GPS_BAUD);
  TELEM_SW_SERIAL.begin(TELEM_BAUD);
  GPS_SW_SERIAL.listen(); // keep GPS as the actively-listening RX; telemetry is mostly TX-only

  if (PIN_E32_M0 >= 0) { pinMode(PIN_E32_M0, OUTPUT); digitalWrite(PIN_E32_M0, LOW); }
  if (PIN_E32_M1 >= 0) { pinMode(PIN_E32_M1, OUTPUT); digitalWrite(PIN_E32_M1, LOW); } // normal/transparent mode

  core1PacketCountFull = getCommsSnapshot().packetCountFull;

  lastTelemMs = millis();
}

// =============================================================================
// CORE 1 LOOP
// =============================================================================
void loop1() {
  // --- Feed GPS bytes continuously ---
  while (GPS_SW_SERIAL.available()) {
    gps.encode(GPS_SW_SERIAL.read());
  }

  double lat = 0, lon = 0;
  uint8_t sats = 0;
  float hdop = 99.9f;
  bool valid = gps.location.isValid();
  if (valid)                    { lat = gps.location.lat(); lon = gps.location.lng(); }
  if (gps.satellites.isValid()) sats = gps.satellites.value() > 10 ? 10 : gps.satellites.value();
  if (gps.hdop.isValid())       hdop = gps.hdop.hdop();

  setCommsGPS(lat, lon, sats, hdop, valid);

  // --- Telemetry, rate-limited to a max of 3 Hz ---
  unsigned long nowMs = millis();
  if (nowMs - lastTelemMs >= TELEM_INTERVAL_MS) {
    lastTelemMs = nowMs;
    sendTelemetry(lat, lon, sats, hdop, valid);
  }
  while (TELEM_SW_SERIAL.available()) {
    String ReceivedData = TELEM_SW_SERIAL.readStringUntil('\n');
    ReceivedData.trim();
    //Serial.println(ReceivedData);

    // Checking that the data received is for updating settings or aborting
    if (ReceivedData.startsWith("SET")) {
    } else if (ReceivedData.startsWith("ABORT")) {  // Sending an abort message if the flight is aborted since there is nothing to abort with HITL yet.
      Serial.println("ABORT FLIGHT");
    } else if (ReceivedData.startsWith("BEGIN_LOGGING")) {  // Sending an abort message if the flight is aborted since there is nothing to abort with HITL yet.
      Serial.println("Logging Begun");
    } else if (ReceivedData.startsWith("ARM")) {  // Sending an abort message if the flight is aborted since there is nothing to abort with HITL yet.
      ARMED = true;
    }

    // Serial.write(TELEM_SW_SERIAL.read());
  }
}

void sendTelemetry(double lat, double lon, uint8_t sats, float hdop, bool gpsValid) {
  FlightSnapshot fs = getFlightSnapshot();

  Telemetry t;
  t.Time             = (uint16_t)((fs.missionElapsedMs / 100UL) & 0xFFFF); // deciseconds
  core1PacketCountFull++;
  setCommsPacketCount(core1PacketCountFull);
  t.PacketCount       = (uint8_t)(core1PacketCountFull & 0xFF);
  t.Mode              = fs.flightState;
  t.Altitude          = (uint16_t)constrain(fs.altitudeAGL * 10.0f, 0, 65535);
  t.VerticalVelocity  = (uint16_t)constrain(fs.verticalSpeedEst * 10.0f, 0, 65535);
  t.GPSLat            = (int32_t)(lat * 1000000.0);
  t.GPSLon            = (int32_t)(lon * 1000000.0);

  uint16_t hdopEnc = (uint16_t)(roundf(hdop * 10.0f)) * 11 + sats;
  t.HDOPSats          = hdopEnc;

  t.Voltage           = (uint8_t)constrain(fs.batteryVoltage * 10.0f, 0, 255);

  uint8_t enabled = fs.enabledBitsCore0;
  if (gpsValid) enabled |= BIT_GPS;
  t.EnabledItems       = enabled;

  t.Checksum = computeChecksum((uint8_t*)&t, sizeof(Telemetry) - 1);

  TELEM_SW_SERIAL.write((uint8_t*)&t, sizeof(Telemetry));
}
