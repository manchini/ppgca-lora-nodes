
#include <Arduino.h>
#include "lmic.h"
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include "DHTesp.h"
#include "AlphaSense.h"
#include "config.h"

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }


static osjob_t sendjob;
char TTN_response[3];
int16_t val;
uint8_t cursor = 0;
byte buffer[19];
#define TX_INTERVAL  600

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 5,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 17,
  .dio = {2, 4, 16},
};

#define DHTPIN 8
DHTesp dht;

#define LPP_TEMPERATURE 103
#define LPP_HUMIDITY 104
#define LPP_ANALOG_INPUT 2


AlphaSense sensorCO("CO",162470430, 332, 294, 345, 344, 0.470);
AlphaSense sensorSO2("SO2",164470002, 335, 362, 360, 368, 0.332);
AlphaSense sensorNO2("NO2",202501663, 230, 214, 229, 214, 0.228);

float temperature = 0;
float humidity = 0;
float CO;
float SO2;
float NO2;


void leSensores(){


  delay(1000);
  temperature = dht.getTemperature();
  //}
  delay(1000);
  humidity = dht.getHumidity();

  Serial.print("Temp: ");
  Serial.println(temperature);


  sensorCO.readValue(10,100);
  CO = sensorCO.algorithm_simple();

  sensorSO2.readValue(10,100);
  SO2 = sensorSO2.algorithm_simple();

  sensorNO2.readValue(10,100);
  NO2 = sensorNO2.algorithm_simple();

}

void resetLora(){
  LMIC_reset();


  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);


  LMIC_selectSubBand(0);

  LMIC_setLinkCheckMode(0);

  LMIC_setDrTxpow(DR_SF9,14);

}

void do_send(osjob_t* j) {

  resetLora();

  leSensores();

  val = 0;
  cursor = 0;


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

  val = float(CO*100/1000);
  buffer[cursor++] = 0x03;
  buffer[cursor++] = LPP_ANALOG_INPUT;
  buffer[cursor++] = val >> 8;
  buffer[cursor++] = val;

  val = float(SO2*100/1000);
  buffer[cursor++] = 0x04;
  buffer[cursor++] = LPP_ANALOG_INPUT;
  buffer[cursor++] = val >> 8;
  buffer[cursor++] = val;

  val = float(NO2*100);
  buffer[cursor++] = 0x05;
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
    Serial.print("dataLen: ");
    Serial.println(LMIC.dataLen);
    if (LMIC.dataLen) {
      for (int i = 0; i < LMIC.dataLen; i++) {
        TTN_response[i] = LMIC.frame[LMIC.dataBeg + i];
      }
    }
    // Schedule next transmission
    os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);

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

  Serial.println(F("Starting..."));

  sensorCO.init(0x48);
  sensorSO2.init(0x49);
  sensorNO2.init(0x4A);



  dht.setup(DHTPIN,DHTesp::DHT22);

  os_init();

  resetLora();

  do_send(&sendjob);

}

void loop() {
  os_runloop_once();

}
