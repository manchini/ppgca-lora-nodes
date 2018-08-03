#include <Arduino.h>

#include "lmic.h"
#include "hal/hal.h"
#include <SPI.h>
#include "DHT.h"
#include "config.h"


#define debug 1;
#define CFG_us915 1


void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16);}


static osjob_t sendjob;
char TTN_response[30];
const unsigned TX_INTERVAL = 3600;

#define DHTPIN 5
#define DHTTYPE DHT11
#define DHTPINON 4

#define analogInPin  A0
#define resistor1  1008
#define resistor2  302

long sensorValue = 0;
float denominator;

#define LPP_TEMPERATURE 103
#define LPP_HUMIDITY 104
#define LPP_ANALOG_INPUT 2

float temperature = 0;
float humidity = 0;


const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 9,
  .dio = {2, 6, 7},
};

int16_t val;
uint8_t cursor = 0;
byte buffer[11];

DHT dht(DHTPIN, DHTTYPE);

void resetLora(){
  LMIC_reset();


  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);


  LMIC_selectSubBand(1);

  LMIC_setLinkCheckMode(0);

  LMIC_setDrTxpow(DR_SF10,14);

}


void do_send(osjob_t* j) {

  resetLora();

  val = 0;
  cursor = 0;

  //Liga pino do dht22
  digitalWrite(DHTPINON, HIGH);
  delay(2000);
  temperature = dht.readTemperature();

  delay(2000);
  humidity = dht.readHumidity();

  digitalWrite(DHTPINON, LOW);

  // read the analog in value:

  sensorValue = 0;
  for(int i = 0; i <50;i++){
    sensorValue += analogRead(analogInPin);
    delay(10);
  }
  sensorValue = sensorValue/50;
  Serial.println(sensorValue);

  float voltage = sensorValue* 5.0  / 1023  ;
  voltage = voltage / denominator;
  Serial.println(voltage);


  //Monta Mensagem

  val = float(temperature*10);

  buffer[cursor++] = 0x01;
  buffer[cursor++] = LPP_TEMPERATURE;
  buffer[cursor++] = val >> 8;
  buffer[cursor++] = val;


  val = float(humidity*2);
  buffer[cursor++] = 0x02;
  buffer[cursor++] = LPP_HUMIDITY;
  buffer[cursor++] = val;

  val = float(voltage*100);
  buffer[cursor++] = 0x03;
  buffer[cursor++] = LPP_ANALOG_INPUT;
  buffer[cursor++] = val >> 8;
  buffer[cursor++] = val;


  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {

    Serial.println(F("OP_TXRXPEND, not sending"));

  } else {
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, buffer, sizeof(buffer)/sizeof(buffer[0]), 0);
    //Serial.println("Sending");
  }

}

void onEvent (ev_t ev) {
  switch(ev) {
    case EV_TXCOMPLETE:
    Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
    if (LMIC.txrxFlags & TXRX_ACK) {
      Serial.println(F("Received ack"));
    }

    if (LMIC.dataLen) {
      int i = 0;
      for ( i = 0 ; i < LMIC.dataLen ; i++ ){
        TTN_response[i] = LMIC.frame[LMIC.dataBeg+i];
      }
      TTN_response[i] = 0;
      Serial.println(TTN_response);
    }

    // Schedule next transmission
    os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);

    break;
    case EV_SCAN_TIMEOUT:
    Serial.println(F("EV_SCAN_TIMEOUT"));
    break;
    case EV_BEACON_FOUND:
    Serial.println(F("EV_BEACON_FOUND"));
    break;
    case EV_BEACON_MISSED:
    Serial.println(F("EV_BEACON_MISSED"));
    break;
    case EV_BEACON_TRACKED:
    Serial.println(F("EV_BEACON_TRACKED"));
    break;
    case EV_JOINING:
    Serial.println(F("EV_JOINING"));
    break;
    case EV_JOINED:
    Serial.println(F("EV_JOINED"));

    // Disable link check validation (automatically enabled
    // during join, but not supported by TTN at this time).
    LMIC_setLinkCheckMode(0);
    break;
    case EV_RFU1:
    Serial.println(F("EV_RFU1"));
    break;
    case EV_JOIN_FAILED:
    Serial.println(F("EV_JOIN_FAILED"));
    break;
    case EV_REJOIN_FAILED:
    Serial.println(F("EV_REJOIN_FAILED"));
    break;
    case EV_LOST_TSYNC:
    Serial.println(F("EV_LOST_TSYNC"));
    break;
    case EV_RESET:
    Serial.println(F("EV_RESET"));
    break;
    case EV_RXCOMPLETE:
    // data received in ping slot
    Serial.println(F("EV_RXCOMPLETE"));
    break;
    case EV_LINK_DEAD:
    Serial.println(F("EV_LINK_DEAD"));
    break;
    case EV_LINK_ALIVE:
    Serial.println(F("EV_LINK_ALIVE"));
    break;
    default:
    Serial.println(F("Unknown event"));
    break;
  }

}

void setup() {

  Serial.begin(115200);
  Serial.println(F("Starting"));

  pinMode(DHTPINON, OUTPUT);
  digitalWrite(DHTPINON, HIGH);
  delay(2000);
  dht.begin();

  denominator = (float)resistor2 / (resistor1 + resistor2);

  os_init();

  os_init();

  resetLora();

  do_send(&sendjob);

}


void loop() {
  os_runloop_once();


}
