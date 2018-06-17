
#include <Arduino.h>
#include "lmic.h"
#include <hal/hal.h>
#include <SPI.h>

//arquivo de configuracao
#include "config.h"



void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }


static osjob_t sendjob;
char TTN_response[3];

int16_t val;
uint8_t cursor = 0;
byte buffer[4];

#define TX_INTERVAL  15

#define LPP_TEMPERATURE 103
#define LPP_HUMIDITY 104
#define LPP_ANALOG_INPUT 2




// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 14,
  .dio = {26, 33, 32}  // Pins for the Heltec ESP32 Lora board/ TTGO Lora32 with 3D metal antenna
};

void resetLora(){
  LMIC_reset();

  //autenticacao  ABP
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);

  //força utilizar apenas uma frequencia
  LMIC_selectSubBand(0);

//Desativa confirmacao msg
  LMIC_setLinkCheckMode(0);

  LMIC_setDrTxpow(DR_SF9,14);

}

void do_send(osjob_t* j) {


  val = 1;

  cursor = 0;
  buffer[cursor++] = 0x03;
  buffer[cursor++] = LPP_ANALOG_INPUT;
  buffer[cursor++] = val >> 8;
  buffer[cursor++] = val;


  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, não responde"));
  } else {
    LMIC_setTxData2(1, buffer, sizeof(buffer)/sizeof(buffer[0]), 0);

  }

}

void onEvent (ev_t ev) {
  //Comunicaçcao
  switch(ev) {
    case EV_TXCOMPLETE:
    Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));


    if (LMIC.txrxFlags & TXRX_ACK) {
      Serial.println(F("Received ack"));
    }
    //Tenta ler o retorno
    Serial.print("dataLen: ");
    Serial.println(LMIC.dataLen);
    if (LMIC.dataLen) {
      for (int i = 0; i < LMIC.dataLen; i++) {
        TTN_response[i] = LMIC.frame[LMIC.dataBeg + i];
      }
    }

    os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);

    break;
    //Demais eventos , mas não é necessários

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

  Serial.println(F("Iniciando..."));


  os_init();

  resetLora();

  do_send(&sendjob);

}

void loop() {
  os_runloop_once();

}
