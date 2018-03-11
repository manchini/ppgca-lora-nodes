#include <Arduino.h>

#include "lmic.h"
#include "hal/hal.h"
#include <SPI.h>

#define CFG_us915 1

static const u1_t PROGMEM APPEUI[8]={ };   // Chose LSB mode on the console and then copy it here.

void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}


static const u1_t PROGMEM DEVEUI[8]={ };   // LSB mode
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

static const u1_t PROGMEM APPKEY[16] = {}; // MSB mode
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16);}


const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 9,
  .dio = {2, 6, 7},
};

static osjob_t sendjob;
char TTN_response[30];
#define DIGITAL_IN 0


const unsigned TX_INTERVAL = 7;


void do_send(osjob_t* j) {

  byte buffer[1];
  buffer[0] = 6;

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    //LMIC_setTxData2(1, buffer, sizeof(buffer)/sizeof(buffer[0]), 0);
    LMIC_setTxData2(1, buffer, sizeof(buffer)/sizeof(buffer[0]), 0);
    //  Serial.println("Sending");
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

  //LMIC.freq = 915000000;
  Serial.begin(9600);
  Serial.println(F("Starting..."));

  os_init();
  LMIC_reset();
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);


  LMIC_setLinkCheckMode(0);
  //LMIC_setAdrMode(1);
  //LMIC.dn2Dr = DR_SF9;
  //LMIC_setDrTxpow(DR_SF9,14);

  do_send(&sendjob);

  LMIC.dn2Dr = DR_SF9;
  LMIC_setDrTxpow(DR_SF9,14);

}

void loop() {
  os_runloop_once();
}
