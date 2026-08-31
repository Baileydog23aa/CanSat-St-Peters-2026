// This script sends mock data for HITL (Hardware-In-The-Loop testing)
// This makes up random data and sends it

// How the rocket connects to the radio it is recieving from
#include <SoftwareSerial.h>
SoftwareSerial TLM(2, 3);

// How the data is received
struct Telemetry {
  uint16_t Time;
  uint8_t PacketCount;
  uint8_t Mode;
  uint16_t Altitude;
  int16_t VerticalVelocity;
  int32_t GPSLat;
  uint32_t GPSLon;
  uint16_t HDOPSats;
  uint8_t Voltage;
  uint8_t EnabledItems;
  uint8_t Checksum;
} __attribute__((packed));

uint8_t packetCount = 0;
int ReceivedPackets = 0;

// Lookup table for the setting names
const char* SettingNames[31] = {
  "minAccel",              // 0
  "AccelCutoff",           // 1
  "TransmitFreqIdle",      // 2
  "TransmitFreqBallistic", // 3
  "TransmitFreqLanded",    // 4
  "BurnoutAccel",          // 5
  "DrougeEnabled",         // 6
  "DrougeMode",            // 7
  "DrougeBackupMode",      // 8
  "MainMode",              // 9
  "MainBackupMode",        // 10
  "MainAlt",               // 11
  "LandedTolerance",       // 12
  "LandedCheckTime",       // 13
  "DrougePrimary",         // 14
  "DrougeBackup",          // 15
  "MainPrimary",           // 16
  "MainBackup",            // 17
  "DrougeDeployAngle",     // 18
  "DrougeNeutralAngle",    // 19
  "MainDeployAngle",       // 20
  "MainNeutralAngle",      // 21
  "DrougeBackupDeployAngle",   // 22
  "DrougeBackupNeutralAngle",  // 23
  "MainBackupDeployAngle",     // 24
  "MainBackupNeutralAngle",    // 25
  "SavingToFlash",         // 26
  "SavingToSD",            // 27
  "idleLogRate",           // 28
  "FlightLogRate",         // 29
  "BallisticLogRate"       // 30
};
long lastTransmit = 0;
String SettingValues[31];

void setup() {
  randomSeed(analogRead(0));
  
  // Beginning communication
  TLM.begin(9600);
  Serial.begin(115200);
  while (!Serial);

  // Sending that the startup is complete
  Serial.println("Startup Complete");
}

void loop() {
  // When data is received, it does this
  while (TLM.available()) {
    String ReceivedData = TLM.readStringUntil('\n');
    ReceivedData.trim();
    Serial.println(ReceivedData);

    // Checking that the data received is for updating settings or aborting
    if (ReceivedData.startsWith("SET")) {
      // Separating the setting name and number
      int settingIndex = ReceivedData.substring(4, 6).toInt();
      String newValue = ReceivedData.substring(7);

      // Saying what setting was updated and what it is updated to
      if (settingIndex >= 0 && settingIndex <= 30) {
        String oldValue = SettingValues[settingIndex];
        Serial.println("Updating setting " + String(SettingNames[settingIndex]) + " (" + String(settingIndex) + ") from: " + oldValue + " to: " + newValue);
        SettingValues[settingIndex] = newValue;
      } else {
        Serial.println("Received SET for unknown index: " + ReceivedData);
      }
    }else if (ReceivedData.startsWith("ABORT")){ // Sending an abort message if the flight is aborted since there is nothing to abort with HITL yet.
      Serial.println("ABORT FLIGHT");
    }
    
    Serial.write(TLM.read());
  }
  // /*
  if (millis() > lastTransmit + 330 && !TLM.available()){
    Telemetry t;

    // Time in 0.01 s units
    t.Time = (uint16_t)(millis() / 100);

    // Packet counter
    packetCount++;
    t.PacketCount = (uint8_t)packetCount;

    // Flight mode (0–4)
    t.Mode = (uint8_t)random(0, 11);

    // Altitude: 0–3000 m
    t.Altitude = (uint16_t)random(0, 30001);

    // Vertical velocity: -50 to +150 m/s
    t.VerticalVelocity = (uint16_t)random(-500, 1501) + 500;

    // GPS
    float lat = -37.787f + random(-1000, 1000) / 100000.0f;
    float lon = 175.279f + random(-1000, 1000) / 100000.0f;

    t.GPSLat = (int32_t)((lat) * 1000000.0f);
    t.GPSLon = (int32_t)((lon) * 1000000.0f);

    // HDOP 0.8–2.5, Satellites 8–18
    uint8_t sats = random(0, 12);
    uint8_t hdopTenths = random(0, 26);

    t.HDOPSats = (hdopTenths) * 11 + (sats);

    // Battery voltage 7.0–8.4 V
    t.Voltage = (uint8_t)random(70, 85);

    // All sensors enabled
    t.EnabledItems = 0b00111111;

    // Simple checksum
    t.Checksum = 0;
    uint8_t* ptr = (uint8_t*)&t;
    for (size_t i = 0; i < sizeof(Telemetry) - 1; i++) {
      t.Checksum ^= ptr[i];
    }

    // Sending the full packet
    TLM.write((uint8_t*)&t, sizeof(t));
    uint8_t Sats = (t.HDOPSats % 11);
    float HDOP = (t.HDOPSats / 11) / 10.0;
    String latestTelemetry = String(t.Time / 10.0, 1) + "," +
                        String(t.PacketCount) + "," +
                        String(ReceivedPackets) + "," +
                        String(t.Altitude / 10.0, 1) + "," +
                        String(t.VerticalVelocity/10.0, 1) + "," +
                        String(t.GPSLat/ 1000000.0, 6) + "," +
                        String(t.GPSLon/1000000.0, 6) + "," +
                        String(HDOP, 1) + "," + 
                        String(Sats) + "," +
                        String(t.Voltage / 10.0, 1) + "," +
                        String((t.EnabledItems & 0x01)?1:0) + "," + 
                        String((t.EnabledItems & 0x02)?1:0) + "," + 
                        String((t.EnabledItems & 0x04)?1:0) + "," + 
                        String((t.EnabledItems & 0x08)?1:0) + "," + 
                        String((t.EnabledItems & 0x16)?1:0) + "," + 
                        String((t.EnabledItems & 0x32)?1:0);
      // Serial.println(latestTelemetry);
      lastTransmit = millis();
  }
  // */
}