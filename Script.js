// All of the data points that are received from the rocket and are displayed on the page
var GPS_LAT = -0.000000;
var GPS_LON = -0.000000;
var Time = 0.00;
var PacketCount = 0;
var flightState = 0;
var ReceivedPackets = 0;
var Altitude = 0.00;
var Velocity = 0.00;
var HDOP = 0.00;
var Sats = 0;
var Voltage = 0.00;
var MS5611Enabled = false;
var BMI088GYROEnabled = false;
var BMI088AccelEnabled = false;
var ADXL375Enabled = false;
var FlashEnabled = false;
var SDEnabled = false;
var Checksum = 0;
const SERVICE_UUID = '4fafc201-1fb5-459e-8fcc-c5c9c331914b';
const CHAR_UUID    = 'beb5483e-36e1-4688-b7f5-ea07361b26a8';


let LastMapUpdate = 0;

// Setting up serial connection
let port;
let writer;

//window.addEventListener('load', autoConnectBLE);
async function connectSerial() {
    port = await navigator.serial.requestPort();
    await port.open({ baudRate: 115200 });
    writer = port.writable.getWriter();
    const info = port.getInfo();
    alert("Serial Port Connected!");
    sessionStorage.setItem('portVendorId', info.usbVendorId);
    sessionStorage.setItem('portProductId', info.usbProductId);
}

async function autoConnect() {
    const ports = await navigator.serial.getPorts();
    const savedVendor = parseInt(sessionStorage.getItem('portVendorId'));
    const savedProduct = parseInt(sessionStorage.getItem('portProductId'));

    const rocketPort = ports.find(p => {
        const info = p.getInfo();
        return info.usbVendorId === savedVendor && info.usbProductId === savedProduct;
    });

    if (rocketPort) {
        port = rocketPort;
        if (port.readable === null) {
            await port.open({ baudRate: 115200 });
            writer = port.writable.getWriter();
        }
        startReading();
    }
}
autoConnect();
var ReadingSerial = false;
async function startReading() {
    console.log("Serial port opened:", port);
    const reader = port.readable.getReader();
    let buffer = '';

    while (port != null) {
        ReadingSerial = true;
        const { value, done } = await reader.read();
        if (done) break;
        
        buffer += new TextDecoder().decode(value);
        const lines = buffer.split('\n');
        buffer = lines.pop();
        
        var receivedData = "";
        var dataArray = [];
        lines.forEach(line => {
            console.log(line);
            receivedData = line.trim();
            dataArray = receivedData.split(',');

            Time = parseFloat(dataArray[0]);
            PacketCount = parseInt(dataArray[1]);
            flightState = parseInt(dataArray[2]);
            ReceivedPackets = parseInt(dataArray[3]);
            Altitude = parseFloat(dataArray[4]);
            Velocity = parseFloat(dataArray[5]);
            GPS_LAT = parseFloat(dataArray[6]);
            GPS_LON = parseFloat(dataArray[7]);
            HDOP = parseFloat(dataArray[8]);
            Sats = parseInt(dataArray[9]);
            Voltage = parseFloat(dataArray[10]);
            MS5611Enabled = dataArray[11] == 1 ? true : false;
            BMI088GYROEnabled = dataArray[12] == 1 ? true : false;
            BMI088AccelEnabled = dataArray[13] == 1 ? true : false;
            ADXL375Enabled = dataArray[14] == 1 ? true : false;
            FlashEnabled = dataArray[15] == 1 ? true : false;
            SDEnabled = dataArray[16] == 1 ? true : false;
            //Checksum = parseInt(dataArray[17]);
            update();
        });
    }
    ReadingSerial = false;
}

// For Bluetooth (Broken so it is removed) This is broken because I am using multiple pages
/*async function connectBLE(){

  try {
    const device = await navigator.bluetooth.requestDevice({
      acceptAllDevices: true,                    // ← show ALL BLE devices
      optionalServices: [SERVICE_UUID]           // ← still needed to access it
    });
    sessionStorage.setItem("bleDeviceName", device.name);

    const server         = await device.gatt.connect();
    const service        = await server.getPrimaryService(SERVICE_UUID);
    const characteristic = await service.getCharacteristic(CHAR_UUID);

    await characteristic.startNotifications();
    characteristic.addEventListener('characteristicvaluechanged', (e) => {
      const msg = new TextDecoder().decode(e.target.value);
      if (!ReadingSerial){
        var dataArray = [];
        dataArray = msg.split(',');

            Time = parseFloat(dataArray[0]);
            PacketCount = parseInt(dataArray[1]);
            ReceivedPackets = parseInt(dataArray[2]);
            flightState = parseInt(dataArray[3]);
            Altitude = parseFloat(dataArray[4]);
            Velocity = parseFloat(dataArray[5]);
            GPS_LAT = parseFloat(dataArray[6]);
            GPS_LON = parseFloat(dataArray[7]);
            HDOP = parseFloat(dataArray[8]);
            Sats = parseInt(dataArray[9]);
            Voltage = parseFloat(dataArray[10]);
            MS5611Enabled = dataArray[11] == 1 ? true : false;
            BMI088GYROEnabled = dataArray[12] == 1 ? true : false;
            BMI088AccelEnabled = dataArray[13] == 1 ? true : false;
            ADXL375Enabled = dataArray[14] == 1 ? true : false;
            FlashEnabled = dataArray[15] == 1 ? true : false;
            SDEnabled = dataArray[16] == 1 ? true : false;
            //Checksum = parseInt(dataArray[17]);
      }
      console.log('ESP32 says:', msg);
      update();
    });

    console.log('Connected to', device.name);

  } catch (err) {
    console.error('BLE failed:', err);
  }
}
async function autoConnectBLE() {
    try {
        const devices = await navigator.bluetooth.getDevices();
        console.log("Auto BLE starting");

        console.log("Devices:", devices);

        const savedName = sessionStorage.getItem("bleDeviceName");
        console.log("Saved name:", savedName);

        const device = devices.find(d => d.name === savedName);
        console.log("Device count:", devices.length);
        console.log("Device:", device);
        console.log("Connected:", device?.gatt?.connected);
        console.log("Found device:", device);

        if (!device) {
            console.log("No previously paired BLE device");
            return;
        }

        if (!device.gatt.connected) {
            let connected = false;

            for (let i = 0; i < 5 && !connected; i++) {
                try {
                    await device.gatt.connect();
                    connected = true;
                } catch (e) {
                    console.log("Retry", i + 1);
                    await new Promise(r => setTimeout(r, 1000));
                }
            }

        }

        const server = await device.gatt.connect();
        await new Promise(resolve => setTimeout(resolve, 1000));
        const service = await server.getPrimaryService(SERVICE_UUID);
        const characteristic =
            await service.getCharacteristic(CHAR_UUID);

        await characteristic.startNotifications();

       
        characteristic.addEventListener('characteristicvaluechanged', (e) => {
            const msg = new TextDecoder().decode(e.target.value);
            if (!ReadingSerial){
                var dataArray = [];
                dataArray = msg.split(',');

                    Time = parseFloat(dataArray[0]);
                    PacketCount = parseInt(dataArray[1]);
                    ReceivedPackets = parseInt(dataArray[2]);
                    flightState = parseInt(dataArray[3]);
                    Altitude = parseFloat(dataArray[4]);
                    Velocity = parseFloat(dataArray[5]);
                    GPS_LAT = parseFloat(dataArray[6]);
                    GPS_LON = parseFloat(dataArray[7]);
                    HDOP = parseFloat(dataArray[8]);
                    Sats = parseInt(dataArray[9]);
                    Voltage = parseFloat(dataArray[10]);
                    MS5611Enabled = dataArray[11] == 1 ? true : false;
                    BMI088GYROEnabled = dataArray[12] == 1 ? true : false;
                    BMI088AccelEnabled = dataArray[13] == 1 ? true : false;
                    ADXL375Enabled = dataArray[14] == 1 ? true : false;
                    FlashEnabled = dataArray[15] == 1 ? true : false;
                    SDEnabled = dataArray[16] == 1 ? true : false;
                    //Checksum = parseInt(dataArray[17]);
            }
            console.log('ESP32 says:', msg);
            update();
        });


    } catch (err) {
        console.log(err);
    }
}*/

// For GPS and the map
function decimalToDMSString(lat, lng) {
  function toDMS(decimal, isLat) {
    const dir = isLat ? (decimal >= 0 ? "N" : "S") : (decimal >= 0 ? "E" : "W");
    const abs = Math.abs(decimal);
    const deg = Math.floor(abs);
    const minFull = (abs - deg) * 60;
    const min = Math.floor(minFull);
    const sec = ((minFull - min) * 60).toFixed(1);
    return `${deg}°${min}'${sec}"${dir}`;
  }

  return `${toDMS(lat, true)} ${toDMS(lng, false)}`;
}

function encodeCoords(lat, lng) {
  const dms = decimalToDMSString(lat, lng);
  // UTF-8 encode before Base64 (handles ° symbol etc.)
  const encoded = btoa(unescape(encodeURIComponent(dms)));
  return encoded;
} // Embedding map coordinates

try{
    let viewing = false;
    document.querySelectorAll('#MenuBtn').forEach(btn => {
        btn.addEventListener('click', () => {
            viewing = !viewing;
            document.querySelector('.NavMenu').classList.toggle('viewing');
            document.querySelector('.NavBG').classList.toggle('viewing');
            document.querySelector('.NavBG_Gradient').classList.toggle('viewing');
            if (viewing == true) {
                document.querySelector('#MenuBtn img').src = "Images/ExitButton.png";
                document.querySelector('.exitBtn').classList.add('opening');
                document.querySelector('.exitBtn').classList.remove('closing');
            } else {
                document.querySelector('#MenuBtn img').src = "Images/MenuButton.png";
                document.querySelector('.exitBtn').classList.add('closing');
                document.querySelector('.exitBtn').classList.remove('opening');
            }
        });
    });
} catch (error) {} // NavMenu

// Updating the colours of the status indicators and updating the text
function update() {

    try {        
        GPS_Status = document.getElementById('GPS_Status');
        if (GPS_LAT !== 0 && GPS_LON !== 0 && HDOP !== 0 && Sats >= 4) {
            GPS_Status.innerHTML = "GO";
        } else {
            GPS_Status.innerHTML = "HOLD";
        }
        if (GPS_Status.innerHTML == "HOLD") {
            GPS_Status.style.color = "red";
        } else if (GPS_Status.innerHTML == "GO") {
            GPS_Status.style.color = "#00FF00";
        }
    } catch (error) {} // GPS Status Selector

    try {
        Flight_State = document.getElementById('Flight_State');
        if (flightState == 0) {
            Flight_State.innerHTML = "STARTUP";
        } else if (flightState == 1) {
            Flight_State.innerHTML = "PAD_IDLE";
        } else if (flightState == 2) {
            Flight_State.innerHTML = "PAD_ARMED";
        } else if (flightState == 3) {
            Flight_State.innerHTML = "ASCENT";
        } else if (flightState == 4) {
            Flight_State.innerHTML = "BURNOUT";
        } else if (flightState == 5) {
            Flight_State.innerHTML = "DESCENT";
        } else if (flightState == 6) {
            Flight_State.innerHTML = "DROUGE_DEPLOY";
        } else if (flightState == 7) {
            Flight_State.innerHTML = "DROUGE";
        } else if (flightState == 8) {
            Flight_State.innerHTML = "MAIN_DEPLOY";
        } else if (flightState == 9) {
            Flight_State.innerHTML = "MAIN";
        } else if (flightState == 10) {
            Flight_State.innerHTML = "BALLISTIC";
        } else if (flightState == 11) {
            Flight_State.innerHTML = "LANDED";
        }


        if (Flight_State.innerHTML == "STARTUP") {
            Flight_State.style.color = "red";
        } else if (Flight_State.innerHTML == "PAD_IDLE") {
            Flight_State.style.color = "orange";
        } else if (Flight_State.innerHTML == "PAD_ARMED") {
            Flight_State.style.color = "#00FF00";
        } else if (Flight_State.innerHTML == "ASCENT") {
            Flight_State.style.color = "#ffee00";
        } else if (Flight_State.innerHTML == "BURNOUT") {
            Flight_State.style.color = "#ff9900";
        } else if (Flight_State.innerHTML == "DESCENT") {
            Flight_State.style.color = "#00ffff";
        } else if (Flight_State.innerHTML == "DROUGE_DEPLOY") {
            Flight_State.style.color = "#0000ff";
        } else if (Flight_State.innerHTML == "DROUGE") {
            Flight_State.style.color = "#ff00ff";
        } else if (Flight_State.innerHTML == "MAIN_DEPLOY") {
            Flight_State.style.color = "#b300ff";
        } else if (Flight_State.innerHTML == "MAIN") {
            Flight_State.style.color = "#6600ff";
        } else if (Flight_State.innerHTML == "BALLISTIC") {
            Flight_State.style.color = "#ff0000";
        } else if (Flight_State.innerHTML == "LANDED") {
            Flight_State.style.color = "#00ff00";
        }
    } catch (error) {} // Flight State colours and updating text

    try {
        SD_State = document.getElementById('SD_State');
        if (SDEnabled == true) {
            SD_State.innerHTML = "GO";
        } else {
            SD_State.innerHTML = "DISABLED";
        }

        if (SD_State.innerHTML == "DISABLED") {
            SD_State.style.color = "red";
        } else if (SD_State.innerHTML == "GO") {
            SD_State.style.color = "#00FF00";
        }
    } catch (error) {} // SD State colours and updating text

    try {
        Flash_State = document.getElementById('Flash_State');
        if (FlashEnabled == true) {
            Flash_State.innerHTML = "GO";
        } else {
            Flash_State.innerHTML = "DISABLED";
        }

        if (Flash_State.innerHTML == "DISABLED") {
            Flash_State.style.color = "red";
        } else if (Flash_State.innerHTML == "GO") {
            Flash_State.style.color = "#00FF00";
        }
    } catch (error) {} // Flash State colours and updating text

    try {
        Data_Status = document.getElementById('Data_Status');
        if (SDEnabled == true && FlashEnabled == true) {
            Data_Status.innerHTML = "GO";
        } else {
            Data_Status.innerHTML = "HOLD";
        }
        if (Data_Status.innerHTML == "HOLD") {
            Data_Status.style.color = "red";
        } else if (Data_Status.innerHTML == "GO") {
            Data_Status.style.color = "#00FF00";
        }
    } catch (error) {} // Data Status Selector

    try {
        GyroActive = document.getElementById('GYRO');
        if (BMI088GYROEnabled == true) {
            GyroActive.innerHTML = "GO";
        } else {
            GyroActive.innerHTML = "HOLD";
        }
        if (GyroActive.innerHTML == "GO") {
            GyroActive.style.color = "#00FF00";
        } else if (GyroActive.innerHTML == "HOLD") {
            GyroActive.style.color = "red";
        }
    } catch (error) {} // Gyro State colours and updating text

    try {
        AccelActive = document.getElementById('ACCEL');
        if (BMI088AccelEnabled == true) {
            AccelActive.innerHTML = "GO";
        } else {
            AccelActive.innerHTML = "HOLD";
        }

        if (AccelActive.innerHTML == "GO") {
            AccelActive.style.color = "#00FF00";
        } else if (AccelActive.innerHTML == "HOLD") {
            AccelActive.style.color = "red";
        }
    } catch (error) {} // Accel State colours and updating text

    try {
        HighRangeAccelActive = document.getElementById('HIGH_RANGE_ACCEL');
        if (ADXL375Enabled == true) {
            HighRangeAccelActive.innerHTML = "GO";
        } else {
            HighRangeAccelActive.innerHTML = "HOLD";
        }

        if (HighRangeAccelActive.innerHTML == "GO") {
            HighRangeAccelActive.style.color = "#00FF00";
        } else if (HighRangeAccelActive.innerHTML == "HOLD") {
            HighRangeAccelActive.style.color = "red";
        }
    } catch (error) {} // High Range Accel State colours and updating text

    try {
        BAROActive = document.getElementById('BARO');
        if (MS5611Enabled == true) {
            BAROActive.innerHTML = "GO";
        } else {
            BAROActive.innerHTML = "HOLD";
        }

        if (BAROActive.innerHTML == "GO") {
            BAROActive.style.color = "#00FF00";
        } else if (BAROActive.innerHTML == "HOLD") {
            BAROActive.style.color = "red";
        }
    } catch (error) {}

    try {
        Flags_State = document.getElementById('Flags_Status');
        if (ADXL375Enabled == true && BMI088AccelEnabled == true && BMI088GYROEnabled == true && MS5611Enabled == true) {
            Flags_State.innerHTML = "GO";
        } else {
            Flags_State.innerHTML = "HOLD";
        }

        if (Flags_State.innerHTML == "GO") {
            Flags_State.style.color = "#00FF00";
        } else if (Flags_State.innerHTML == "HOLD") {
            Flags_State.style.color = "red";
        }
    } catch (error) {} // Flags status selector

    try{
        EmbedMap = document.getElementById('Map');
        let now = Date.now(); // milliseconds, monotonic, no wraparound
        if (now - LastMapUpdate >= 5000) { // throttle to once per second
            console.log("Updating map with coordinates: " + GPS_LAT.toFixed(6) + ", " + GPS_LON.toFixed(6));
            EmbedMap.src = "https://www.google.com/maps/embed?pb=!1m17!1m12!1m3!1d1750!2d" + GPS_LON.toFixed(6) + "!3d" + GPS_LAT.toFixed(6) + "!2m3!1f0!2f0!3f0!3m2!1i1024!2i768!4f13.1!3m2!1m1!2z"+ encodeCoords(GPS_LAT.toFixed(6), GPS_LON.toFixed(6)) +"!5e1!3m2!1sen!2snz!4v1779743037952!5m2!1sen!2snz";
            LastMapUpdate = now;
        }
    } catch (error) {} // Embedded Map

    try {
        HDOP_Text = document.getElementById('GPS_HDOP');
        HDOP_Text.innerHTML = HDOP.toFixed(2);
    } catch (error) {} // GPS HDOP updating text

    try {
        Sats_Text = document.getElementById('GPS_Sats');
        Sats_Text.innerHTML = Sats;
    } catch (error) {} // GPS Sats updating text

    try {
        Latitude_Text = document.getElementById('GPS_Latitude');
        Latitude_Text.innerHTML = GPS_LAT.toFixed(6);
    } catch (error) {} // GPS Latitude updating text

    try {
        Longitude_Text = document.getElementById('GPS_Longitude');
        Longitude_Text.innerHTML = GPS_LON.toFixed(6);
    } catch (error) {} // GPS Longitude updating text

    try {
        Altitude_Text = document.getElementById('Altitude');
        Altitude_Text.innerHTML = Altitude.toFixed(1) + " m";
    } catch (error) {} // Altitude updating text

    try {
        Velocity_Text = document.getElementById('Velocity');
        Velocity_Text.innerHTML = Velocity.toFixed(1) + " m/s";
    } catch (error) {} // Velocity updating text

    try {
        flightTime_Text = document.getElementById('Flight_Time');
        flightTime_Text.innerHTML = Time.toFixed(1) + " s";
    } catch (error) {} // Flight Time updating text

    try {
        packetCount_Text = document.getElementById('Packet_Count');
        packetCount_Text.innerHTML = PacketCount;
    } catch (error) {} // Packet Count updating text

    try {
        receivedPackets_Text = document.getElementById('Packets_Received');
        receivedPackets_Text.innerHTML = ReceivedPackets;
    } catch (error) {} // Received Packets updating text
}

// Sending for updating settings
async function send(str) {
    if (!writer) {
        console.error("No writer available — port not connected");
        return;
    }
    const data = new TextEncoder().encode("SET " + str + "\n");
    console.log("Sending:", data);
    await writer.write(data);
}
// Sending for functions
async function SendCommand(str) {
    if (!writer) {
        console.error("No writer available — port not connected");
        return;
    }
    const data = new TextEncoder().encode(str + "\n");
    console.log("Sending:", data);
    await writer.write(data);
}

// Settings functions sending when they are updated
function UpdateSetting0() {
    var launchAccel = document.getElementById("launchAccel").value;
    send(`00,${launchAccel}`);
    alert("Setting minAccel updated to " + launchAccel);
}
function UpdateSetting1() {
    var flightStateCheckTime = document.getElementById("flightStateCheckTime").value;
    send(`01,${flightStateCheckTime}`);
    alert("Setting flightStateCheckTime updated to " + flightStateCheckTime);
}
function UpdateSetting2() {
    var padIdleTransmitRate = document.getElementById("padIdleTransmitRate").value;
    send(`02,${padIdleTransmitRate}`);
    alert("Setting padIdleTransmitRate updated to " + padIdleTransmitRate);
}
function UpdateSetting3() {
    var ballisticTransmitRate = document.getElementById("ballisticTransmitRate").value;
    send(`03,${ballisticTransmitRate}`);
    alert("Setting ballisticTransmitRate updated to " + ballisticTransmitRate);
}
function UpdateSetting4() {
    var landedTransmitRate = document.getElementById("landedTransmitRate").value;
    send(`04,${landedTransmitRate}`);
    alert("Setting landedTransmitRate updated to " + landedTransmitRate);
}
function UpdateSetting5() {
    var burnoutAccel = document.getElementById("burnoutAccel").value;
    send(`05,${burnoutAccel}`);
    alert("Setting burnoutAccel updated to " + burnoutAccel);
}
function UpdateSetting7() {
    var droguePrimary = document.getElementById("droguePrimary").value;
    if (droguePrimary == 3) {
        send('06,0');
        alert("Setting droguePrimary updated to " + droguePrimary + " (drouge Parachue disabled)");
    } else {
        send('06,1');
        send(`07,${droguePrimary}`);
        alert("Setting droguePrimary updated to " + droguePrimary);
    }
    alert("Setting droguePrimary updated to " + droguePrimary);
}
function UpdateSetting8() {
    var drogueBackupMode = document.getElementById("drogueBackupMode").value;
    send(`08,${drogueBackupMode}`);
    alert("Setting drogueBackupMode updated to " + drogueBackupMode);
}
function UpdateSetting9() {
    var mainPrimaryMode = document.getElementById("mainPrimaryMode").value;
    send(`09,${mainPrimaryMode}`);
    alert("Setting mainPrimaryMode updated to " + mainPrimaryMode);
}
function UpdateSetting10() {
    var mainBackupMode = document.getElementById("mainBackupMode").value;
    send(`10,${mainBackupMode}`);
    alert("Setting mainBackupMode updated to " + mainBackupMode);
}
function UpdateSetting11() {
    var mainAlt = document.getElementById("mainDeploy").value;
    send(`11,${mainAlt}`);
    alert("Setting mainAlt updated to " + mainAlt);
}
function UpdateSetting12() {
    var landedTolerance = document.getElementById("landedTolerance").value;
    send(`12,${landedTolerance}`);
    alert("Setting landedTolerance updated to " + landedTolerance);
}
function UpdateSetting13() {
    var landedCheckTime = document.getElementById("landedCheckTime").value;
    send(`13,${landedCheckTime}`);
    alert("Setting landedCheckTime updated to " + landedCheckTime);
}
function UpdateSetting14() {
    var droguePrimary = document.getElementById("droguePrimary").value;
    send(`14,${droguePrimary}`);
    alert("Setting droguePrimary updated to " + droguePrimary);
}
function UpdateSetting15() {
    var drogueBackup = document.getElementById("drogueBackup").value;
    send(`15,${drogueBackup}`);
    alert("Setting drogueBackup updated to " + drogueBackup);
}
function UpdateSetting16() {
    var mainPrimary = document.getElementById("mainPrimary").value;
    send(`16,${mainPrimary}`);
    alert("Setting mainPrimary updated to " + mainPrimary);
}
function UpdateSetting17() {
    var mainBackup = document.getElementById("mainBackup").value;
    send(`17,${mainBackup}`);
    alert("Setting mainBackup updated to " + mainBackup);
}
function UpdateSetting26() {
    var savingToFlash = document.getElementById("savingToFlash").value;
    send(`26,${savingToFlash}`);
    alert("Setting savingToFlash updated to " + savingToFlash);
}
function UpdateSetting27() {
    var savingToSD = document.getElementById("savingToSD").value;
    send(`27,${savingToSD}`);
    alert("Setting savingToSD updated to " + savingToSD);
}
function UpdateSetting28() {
    var padIdleLogRate = document.getElementById("padIdleLogRate").value;
    send(`28,${padIdleLogRate}`);
    alert("Setting padIdleLogRate updated to " + padIdleLogRate);
}
function UpdateSetting29() {
    var flightLogRate = document.getElementById("flightLogRate").value;
    send(`29,${flightLogRate}`);
    alert("Setting flightLogRate updated to " + flightLogRate);
}
function UpdateSetting30() {
    var ballisticLogRate = document.getElementById("ballisticLogRate").value;
    send(`30,${ballisticLogRate}`);
    alert("Setting ballisticLogRate updated to " + ballisticLogRate);
}
function abortFlight() {
    SendCommand("ABORT");
    alert("Flight aborted");
}
function BeginLogging() {
    SendCommand("ARM");
    alert("CanSat Armed");
}