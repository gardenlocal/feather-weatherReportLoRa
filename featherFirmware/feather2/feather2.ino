#include <SPI.h>
#include <RH_RF95.h>

// FEATHER 32u4 LoRa I2C PIN
// https://learn.adafruit.com/adafruit-feather-32u4-radio-with-lora-radio-module/pinouts
// SDA 2
// SCL 3

// define Machien ID
const uint8_t DEVICE_ID = 2;

// define pinouts
const int PIN_LED     = 13;
const int PIN_CHG     = 6;    // READ CHARGE
const int PIN_GOOD    = 9;    // READ BATT-GOOD
const int PIN_SOLENOID = 10;  // SOLENOID CONTROL PIN OUT
const int PIN_LED_SOLENOID = 11;   // CHARGE LED OUT
const int PIN_LED_GOOD = 12;  // BATT-GOOD LED OUT
//const int PIN_SOIL_VCC = A4;  // SOIL VCC
const int PIN_SOIL_READ = A5;  // SOIL READ

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
uint16Number soil;
bool isCharging = false;
bool bFogOn = false;

void pinSetup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_CHG, INPUT);
  pinMode(PIN_GOOD, INPUT);
  pinMode(PIN_SOLENOID, OUTPUT);
  pinMode(PIN_LED_SOLENOID, OUTPUT);
  pinMode(PIN_LED_GOOD, OUTPUT);
//  pinMode(PIN_SOIL_VCC, OUTPUT);
//  pinMode(PIN_SOIL_READ, OUTPUT);

  digitalWrite(PIN_LED, LOW);
}


// print sensor value to serial monitor
void printWeatherData() {
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
//  digitalWrite(PIN_SOIL_VCC, HIGH);
    soil.numBigInt = analogRead(PIN_SOIL_READ);
//  soil.numBigInt = millis();

//  digitalWrite(PIN_SOIL_VCC, LOW);
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
  uint8_t reply[15];
  reply[0] = '/';
  reply[1] = 'R'; // report packet
  reply[2] = DEVICE_ID;               // device ID. int? byte? hmm...
  reply[3] = temperature.numBin[0];   // 32bit float bin
  reply[4] = temperature.numBin[1];
  reply[5] = temperature.numBin[2];
  reply[6] = temperature.numBin[3];
  reply[7] = humidity.numBin[0];      // 32bit float bin
  reply[8] = humidity.numBin[1];
  reply[9] = humidity.numBin[2];
  reply[10] = humidity.numBin[3];
  reply[11] = soil.numBin[0];         // 16bit int bin
  reply[12] = soil.numBin[1];
  if (isCharging) reply[13] = 1;      // chargning status bin
  else            reply[13] = 0;
  reply[14] = 0;

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
  soil.numBigInt = 0;

  delay(1000);
  Serial.println("BOOTING....");

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
    bFogOn = !bFogOn;
  }

  // checking batt status and LED out
  if (digitalRead(PIN_GOOD)) digitalWrite(PIN_LED_GOOD, LOW);
  else                      digitalWrite(PIN_LED_GOOD, HIGH);

  if(digitalRead(PIN_CHG))  isCharging = false;
  else                      isCharging = true;


  // fog control
  if (bFogOn)    {
    digitalWrite(PIN_SOLENOID, HIGH);
    digitalWrite(PIN_LED_SOLENOID, HIGH);
  }
  else {
    digitalWrite(PIN_SOLENOID, LOW);
    digitalWrite(PIN_LED_SOLENOID, LOW);
  }

  //  Serial.print(digitalRead(PIN_GOOD));
  //  Serial.print(" - ");
  //  Serial.println(digitalRead(PIN_CHG));
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
  // dump demo
  temperature.numFloat = millis() / 1000;
  humidity.numFloat = millis() / 500;

}
