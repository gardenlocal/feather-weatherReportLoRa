#include <SPI.h>
#include <RH_RF95.h>
#include "DHT.h"

// define pinouts
const int PIN_LED     = 13;
const int PIN_DHT     = 5;

// DHT setup
#define DHTTYPE DHT22
DHT dht(PIN_DHT, DHTTYPE);

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

typedef union{
  float numFloat;
  byte numBin[4];
} floatingNumber;

floatingNumber temperature, humidity;

void setup() {
  Serial.begin(115200);

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  temperature.numFloat = 0.f;
  humidity.numFloat = 0.f;

  delay(1000);
  Serial.println("BOOTING....");

  dht.begin();
  initLoRa();
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(500);
  getWeatherData();
  printWeatherData();
}

void initLoRa(){ // init
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.println("Feather LoRa RX Test!");

  // manual LoRa rest
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while(!rf95.init()){
    Serial.println("LoRa radio init failed");
    while(1);
  }

  Serial.println("LoRa radio init OK!");

  if(!rf95.setFrequency(RF95_FREQ)){
    Serial.println("setFrequency failed.");
    while(1);
  }

  Serial.print("Set Freq to : ");
  Serial.println(RF95_FREQ);

  // default transmitter power is 13dbm, using PA_BOOST
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
}

void getWeatherData(){
  temperature.numFloat = dht.readTemperature();
  humidity.numFloat = dht.readHumidity();
}

void printWeatherData(){
  Serial.print(temperature.numFloat, 2);
  Serial.print("C,\t");
  Serial.print(humidity.numFloat, 2);
  Serial.println("%");
}


void receiveMessage(){
  if(rf95.available()){
    uint8_t recvBufferLen = sizeof(recvBuffer);

    if(rf95.recv(recvBuffer, &recvBufferLen)){
      digitalWrite(PIN_LED, HIGH);

      if(recvBuffer[0] == '/'){
        if(recvBuffer[1] == 'R'){
          getWeatherData();

          uint8_t reply[10];
          reply[0] = '/';
          reply[1] = temperature.numBin[0];
          reply[2] = temperature.numBin[1];
          reply[3] = temperature.numBin[2];
          reply[4] = temperature.numBin[3];
          reply[5] = humidity.numBin[0];
          reply[6] = humidity.numBin[1];
          reply[7] = humidity.numBin[2];
          reply[0] = humidity.numBin[3];
          reply[9] = 0;

          rf95.send(reply, sizeof(reply));
          digitalWrite(PIN_LED, LOW);
          Serial.println("weatherData Sent!");
          printWeatherData();
        }
      }
      digitalWrite(PIN_LED, LOW);
    } else {
      //recv failed
    }
  }
}
