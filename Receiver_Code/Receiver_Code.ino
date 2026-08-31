#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID  "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_UUID     "beb5483e-36e1-4688-b7f5-ea07361b26a8"


// Packet information
String latestTelemetry = "";
struct Telemetry {
  uint16_t Time;
  uint8_t PacketCount;
  uint8_t Mode;
  uint16_t Altitude; // For flights over 6.5km, change to uint32_t
  uint16_t VerticalVelocity; // For flights over mach 19, use uint32_t
  uint32_t GPSLat;
  uint32_t GPSLon;
  uint16_t HDOPSats;
  uint8_t Voltage;
  uint8_t EnabledItems; // BaroEnabled, imuAccelEnabled, imuGyroEnabled, accelEnabled, flashEnabled, SDEnabled
  uint8_t Checksum;
} __attribute__((packed));
Telemetry t;
int ReceivedPackets = 0;

// Stuff for the screen
static const unsigned char PROGMEM logo_bmp[] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xe0, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x80, 0x06, 0x41, 0x00, 
	0x00, 0x00, 0x00, 0x07, 0x00, 0x08, 0x02, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x31, 0xc0, 0x00, 
	0x00, 0x00, 0x00, 0x78, 0x00, 0x67, 0x82, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x00, 0x5f, 0x18, 0x00, 
	0x00, 0x00, 0x01, 0xf8, 0x61, 0xb9, 0xc0, 0x00, 0x00, 0x00, 0x03, 0xfc, 0x7b, 0xec, 0x00, 0x00, 
	0x00, 0x00, 0x07, 0xf7, 0xde, 0xe0, 0x00, 0x00, 0x00, 0x0c, 0x07, 0xdf, 0xfb, 0x80, 0x00, 0x00, 
	0x00, 0x0c, 0x06, 0xff, 0xed, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xbf, 0x78, 0x00, 0x00, 
	0x00, 0x00, 0x0f, 0xfe, 0xff, 0x00, 0x70, 0x00, 0x03, 0x00, 0x3f, 0xfb, 0xfe, 0xe1, 0xe0, 0x00, 
	0x0f, 0xc0, 0xfe, 0xff, 0xff, 0xff, 0x80, 0x00, 0x02, 0x03, 0xe7, 0x9f, 0xf7, 0xfe, 0x00, 0x00, 
	0x00, 0x07, 0xe8, 0x6f, 0xdf, 0xf0, 0x00, 0x00, 0x00, 0x1f, 0xec, 0x5f, 0x7e, 0x00, 0x00, 0x00, 
	0x00, 0x3f, 0xf8, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4f, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0xfd, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0x7f, 0x00, 0x07, 0x00, 0x00, 0x00, 
	0x01, 0xff, 0xb8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0x80, 0x1e, 0x00, 0x00, 0x00, 0x00, 
	0x03, 0xf8, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define LOGO_HEIGHT   32
#define LOGO_WIDTH    64
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// For receiving the data
HardwareSerial E32(2);

// BLE stuff
BLECharacteristic *pCharacteristic;
// Checking if something was sent to the ESP32
class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) {
    String value = pChar->getValue().c_str();
    Serial.println("Received: " + value);
    
    // Echo back
    pChar->setValue(("Echo: " + value).c_str());
    pChar->notify();
  }
};

// Checking if a BLE client is connected
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    Serial.println("Client connected");
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->start();
  }
// Checking if a BLE client is disconnected
  void onDisconnect(BLEServer* pServer) {
    Serial.println("Client disconnected");
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->start();
    BLEDevice::startAdvertising();
  }
};

void setup() {
  // Basic setup
  Serial.begin(115200);
  pinMode(0, OUTPUT);
  digitalWrite(0, LOW);
  // Setting up display
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.display();
  display.clearDisplay();
  display.drawBitmap(
    (SCREEN_WIDTH  - LOGO_WIDTH)  / 2,   // x: centered horizontally
    (SCREEN_HEIGHT - LOGO_HEIGHT) / 2,   // y: centered vertically
    logo_bmp,
    LOGO_WIDTH,
    LOGO_HEIGHT,
    SSD1306_WHITE
  );
  display.display();
  // Beginning communication with rocket
  E32.begin(9600, SERIAL_8N1, 16, 17);

  // Beginning BLE communication
  BLEDevice::init("Rocket Ground Station");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pServer->setCallbacks(new MyServerCallbacks());

  pCharacteristic = pService->createCharacteristic(
    CHAR_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->start();
}

void loop() {
  // Checking if something was sent from the computer
  while (Serial.available()) {

    String message = Serial.readStringUntil('\n');

    E32.println(message);

    Serial.print("Sent: ");
    Serial.println(message);
  }
  // Getting information from rocket and sending to computer
  if (!Serial.available()){
    while(E32.available()>=sizeof(Telemetry)){
      struct Telemetry {
        uint16_t Time;
        uint8_t PacketCount;
        uint8_t Mode;
        uint16_t Altitude; // For flights over 6.5km, change to uint32_t
        uint16_t VerticalVelocity; // For flights over mach 19, use uint32_t
        int32_t GPSLat;
        int32_t GPSLon;
        uint16_t HDOPSats;
        uint8_t Voltage;
        uint8_t EnabledItems; // BaroEnabled, imuAccelEnabled, imuGyroEnabled, accelEnabled, flashEnabled, SDEnabled
        uint8_t Checksum;
      } __attribute__((packed));
      Telemetry t;
      ReceivedPackets ++;
      E32.readBytes((uint8_t*)&t,sizeof(Telemetry));
      uint8_t Sats = (t.HDOPSats % 11);
      float HDOP = (t.HDOPSats / 11) / 10.0;
      String latestTelemetry = String(t.Time / 10.0, 1) + "," +
                          String(t.PacketCount) + "," +
                          String(t.Mode) + "," + 
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
        Serial.println(latestTelemetry);
        // Sending data to BLE
      BLECharacteristic *pChar;
      delay(100);
      pCharacteristic->setValue(latestTelemetry.c_str());
      pCharacteristic->notify();
    }
  }
}
