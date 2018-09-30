
#include <Arduino.h>
#include "lmic.h"
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include "DHTesp.h"

#include "config.h"

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }


static osjob_t sendjob;
char TTN_response[3];
int16_t val;
uint8_t cursor = 0;
byte buffer[19];
#define TX_INTERVAL  30

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 5,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 17,
  .dio = {2, 4, 16},
};

#define DHTPIN 8
DHTesp dht;

float temperature = 0;
float humidity = 0;

#define LPP_TEMPERATURE 103
#define LPP_HUMIDITY 104
#define LPP_ANALOG_INPUT 2


Adafruit_ADS1115  adsCO(0x48);
Adafruit_ADS1115  adsSO2(0x49);
Adafruit_ADS1115   adsNO(0x4A);
float multiplier = 0.1875F;

float dif1=0 ;
float dif2 =0;

float co;
float SO2;
float NO;


void leSensores(){


  delay(1000);
  temperature = dht.getTemperature();
  //}
  delay(1000);
  humidity = dht.getHumidity();

  Serial.print("Temp: ");
  Serial.println(temperature);


  dif1=0 ;
  dif2 =0;

  int mvWe;
  int mvAe;

  int i = 0;
  for(i = 0 ; i < 10; i++){
    dif1 += adsCO.readADC_Differential_0_1();
    dif2 += adsCO.readADC_Differential_2_3();
    delay(200);
  }

  dif1 = dif1/10;
  dif2 = dif2/10;

  Serial.print("dif = ");
  Serial.print(dif1);
  Serial.print("dif2 = ");
  Serial.println(dif2);

  mvWe = dif1 * multiplier;
  mvAe = dif2 * multiplier;
  co = (mvWe+mvAe)/2/470.0;

  Serial.print("CO: WE ");
  Serial.print(mvWe);
  Serial.print(";AE ");
  Serial.print(mvAe);
  Serial.print(";CO ");
  Serial.println(co);

  dif1 =0;
  dif2 =0;
  i = 0;
  for(i = 0 ; i < 10; i++){
    dif1 += adsSO2.readADC_Differential_0_1();
    dif2 += adsSO2.readADC_Differential_2_3();
    delay(200);
  }

  dif1 = dif1/10;
  dif2 = dif2/10;

  Serial.print("dif = ");
  Serial.print(dif1);
  Serial.print(" dif2 = ");
  Serial.println(dif2);

  mvWe = dif1 * multiplier;
  mvAe = dif2 * multiplier;
  SO2 = (mvWe+mvAe)/2/332.0;

  Serial.print("SO2: WE ");
  Serial.print(mvWe);
  Serial.print(";AE ");
  Serial.print(mvAe);
  Serial.print(";SO2 ");
  Serial.println(SO2);


  dif1 =0;
  dif2 =0;
  i = 0;
  for(i = 0 ; i < 10; i++){
    dif1 += adsNO.readADC_Differential_0_1();
    dif2 += adsNO.readADC_Differential_2_3();
    delay(200);
  }

  dif1 = dif1/10;
  dif2 = dif2/10;

  Serial.print("dif = ");
  Serial.print(dif1);
  Serial.print(" dif2 = ");
  Serial.println(dif2);

  mvWe = dif1 * multiplier;
  mvAe = dif2 * multiplier;
  NO = (mvWe+mvAe)/2/228.0;

  Serial.print("NO: WE ");
  Serial.print(mvWe);
  Serial.print(";AE ");
  Serial.print(mvAe);
  Serial.print(";NO ");
  Serial.println(NO);
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

  val = float(co*100);
  buffer[cursor++] = 0x03;
  buffer[cursor++] = LPP_ANALOG_INPUT;
  buffer[cursor++] = val >> 8;
  buffer[cursor++] = val;

  val = float(SO2*100);
  buffer[cursor++] = 0x04;
  buffer[cursor++] = LPP_ANALOG_INPUT;
  buffer[cursor++] = val >> 8;
  buffer[cursor++] = val;

  val = float(NO*100);
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

  adsCO.setGain(GAIN_TWOTHIRDS);
  adsSO2.setGain(GAIN_TWOTHIRDS);
  adsNO.setGain(GAIN_TWOTHIRDS);

  adsCO.begin();
  adsSO2.begin();
  adsNO.begin();


  dht.setup(DHTPIN,DHTesp::DHT22);

  os_init();

  resetLora();

  do_send(&sendjob);

}

void loop() {
  os_runloop_once();

}
