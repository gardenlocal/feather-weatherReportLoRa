#include <SensirionI2CScd4x.h>    // SCD40 I2C sensor library
#include <SPI.h>
#include <RH_RF95.h>

// FEATHER 32u4 LoRa I2C PIN
// https://learn.adafruit.com/adafruit-feather-32u4-radio-with-lora-radio-module/pinouts
// SDA 2
// SCL 3

// define Machien ID
const uint8_t DEVICE_ID = 1;

// define pinouts
const int PIN_LED     = 13;
const int PIN_CHG     = 6;    // READ CHARGE
const int PIN_GOOD    = 9;    // READ BATT-GOOD
const int PIN_SOLENOID = 10;  // SOLENOID CONTROL PIN OUT
const int PIN_LED_CHG = 11;   // CHARGE LED OUT
const int PIN_LED_GOOD = 12;  // BATT-GOOD LED OUT
const int PIN_SOIL_VCC = A4;  // SOIL VCC
const int PIN_SOIL_READ = 5;  // SOIL READ

// LoRa message/buffer setup
const int REQ_MESSAGE_SIZE = 2;
const int REQ_BUFFER_SIZE = REQ_MESSAGE_SIZE + 2;

const int REPLY_MESSAGE_SIZE = 4;
const int RECV_MAX_BUFFER_SIZE = 20;
uint8_t recvBuffer[RECV_MAX_BUFFER_SIZE];

// LoRa SPI / freq Setting
const int RFM95_CS    = 8;
const int RFM95_RST   = 4;
const int RFM95_INT   = 7;
const float RF95_FREQ = 900.0;

RH_RF95 rf95(RFM95_CS, RFM95_INT);

SensirionI2CScd4x scd4x;

uint64_t timerReadSensor;

// 32bit float share memory to 4byte bin
typedef union {
  float numFloat;
  byte numBin[4];
} floatingNumber;


// 16 bit uint16_t share memory with 2byte bin
typedef union {
  uint16_t numBigInt;
  byte numBin[2];
} uint16Number;

floatingNumber temperature, humidity;
uint16Number co2, soil;
bool isCharging = false;
bool bFogOn = false;

void pinSetup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_CHG, INPUT);
  pinMode(PIN_GOOD, INPUT);
  pinMode(PIN_SOLENOID, OUTPUT);
  pinMode(PIN_LED_CHG, OUTPUT);
  pinMode(PIN_LED_GOOD, OUTPUT);
  pinMode(PIN_SOIL_VCC, OUTPUT);
  pinMode(PIN_SOIL_READ, OUTPUT);

  digitalWrite(PIN_LED, LOW);
}

void printUint16Hex(uint16_t value) {
  Serial.print(value < 4096 ? "0" : "");
  Serial.print(value < 256 ? "0" : "");
  Serial.print(value < 16 ? "0" : "");
  Serial.print(value, HEX);
}

void printSerialNumber(uint16_t serial0, uint16_t serial1, uint16_t serial2) {
  Serial.print("Serial: 0x");
  printUint16Hex(serial0);
  printUint16Hex(serial1);
  printUint16Hex(serial2);
  Serial.println();
}

// print sensor value to serial monitor
void printWeatherData() {
  Serial.print("CO2:");
  Serial.print(co2.numBigInt);
  Serial.print("\t");
  Serial.print("Temperature:");
  Serial.print(temperature.numFloat);
  Serial.print("c, \t");
  Serial.print("Humidity:");
  Serial.print(humidity.numFloat);
  Serial.println("% \t");
  Serial.print("SOIL :");
  Serial.println(soil.numBigInt);
}

void getSoilData() {
  digitalWrite(PIN_SOIL_VCC, HIGH);
  soil.numBigInt = digitalRead(PIN_SOIL_READ);

  digitalWrite(PIN_SOIL_VCC, LOW);
}

void receivecMessage() {
  if (rf95.available()) {
    uint8_t recvBufferLen = sizeof(recvBuffer);

    if (rf95.recv(recvBuffer, &recvBufferLen)) {
      digitalWrite(PIN_LED, HIGH);

      if (recvBuffer[0] == '/') {
        if (recvBuffer[1] == DEVICE_ID) {   // only receiving device id
          if (recvBuffer[2] == 'F') {
            if (recvBuffer[3] == '1') bFogOn = true;
            else                      bFogOn = false;
            }
          }
        } else {  // if device id is not mine, quit function
          return;
      }
    }

    digitalWrite(PIN_LED, LOW);
  } else {
    // not recv
  }
}

void sendMessage() {
  uint8_t reply[17];
  reply[0] = '/';
  reply[1] = DEVICE_ID;               // device ID. int? byte? hmm...
  reply[2] = temperature.numBin[0];   // 32bit float bin
  reply[3] = temperature.numBin[1];
  reply[4] = temperature.numBin[2];
  reply[5] = temperature.numBin[3];
  reply[6] = humidity.numBin[0];      // 32bit float bin
  reply[7] = humidity.numBin[1];
  reply[8] = humidity.numBin[2];
  reply[9] = humidity.numBin[3];
  reply[10] = co2.numBin[0];          // 16bit int bin
  reply[11] = co2.numBin[1];
  reply[12] = soil.numBin[0];
  reply[13] = soil.numBin[1];
  if (isCharging) reply[14] = 1;      // chargning status bin
  else            reply[15] = 0;
  reply[16] = 0;

  rf95.send(reply, sizeof(reply));
  digitalWrite(PIN_LED, LOW);
  //  Serial.println("weatherData Sent!");
  printWeatherData();
}


void setup() {
  Serial.begin(115200);

  pinSetup();

  temperature.numFloat = 0.f;
  humidity.numFloat = 0.f;
  co2.numBigInt = 0;
  soil.numBigInt = 0;

  delay(1000);
  Serial.println("BOOTING....");

  initSCD40();    // init SCD I2C sensor
  initLoRa();     // init LoRa
  timerReadSensor = millis(); // init timer stamp
}

void loop() {
  uint16_t error;
  char errorMessage[256];

  if (millis() - timerReadSensor > 5000) {
    getSoilData();
    getWeatherData();
    printWeatherData();

    sendMessage();          // send radio message
    timerReadSensor = millis();
  }

  // checking batt status and LED out
  if (digitalRead(PIN_GOOD)) digitalWrite(PIN_LED_GOOD, LOW);
  else                      digitalWrite(PIN_LED_GOOD, HIGH);

  // checing batt charging status and LED out
  if (digitalRead(PIN_CHG))  {
    digitalWrite(PIN_LED_CHG, LOW);
    isCharging = false;
  }
  else {
    digitalWrite(PIN_LED_CHG, HIGH);
    isCharging = true;
  }

  // fog control
  if (bFogOn)    digitalWrite(PIN_SOLENOID, HIGH);
  else            digitalWrite(PIN_SOLENOID, LOW);

  //  Serial.print(digitalRead(PIN_GOOD));
  //  Serial.print(" - ");
  //  Serial.println(digitalRead(PIN_CHG));
}

void initSCD40() {
  Wire.begin();

  scd4x.begin(Wire);
  uint16_t error;
  char errorMessage[256];

  // stop potentially previously started measurement
  error = scd4x.stopPeriodicMeasurement();
  if (error) {
    Serial.print("Error trying to execute stopPeriodicMesurement(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }

  uint16_t serial0;
  uint16_t serial1;
  uint16_t serial2;
  error = scd4x.getSerialNumber(serial0, serial1, serial2);
  if (error) {
    Serial.print("Error trying to execute getSerialNumber");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  } else {
    printSerialNumber(serial0, serial1, serial2);
  }

  // start Measurement
  error = scd4x.startPeriodicMeasurement();
  if (error) {
    Serial.print("Error trying to execute startPeriodicMeasurement(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }

}

void initLoRa() { // init
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.println("Feather LoRa RX Test!");

  // manual LoRa rest
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }

  Serial.println("LoRa radio init OK!");

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed.");
    while (1);
  }

  Serial.print("Set Freq to : ");
  Serial.println(RF95_FREQ);

  // default transmitter power is 13dbm, using PA_BOOST
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
}

void getWeatherData() {
  uint16_t error;
  char errorMessage[256];

  // readMeasurement
  error = scd4x.readMeasurement(co2.numBigInt, temperature.numFloat, humidity.numFloat);
  if (error) {
    Serial.print("Error trying to execute readMeasurement(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  } else if (co2.numBigInt == 0) {
    Serial.println("Invalid sample detected, skipping.");
  } else {
    printWeatherData();
  }
  digitalWrite(PIN_LED, HIGH);
}
