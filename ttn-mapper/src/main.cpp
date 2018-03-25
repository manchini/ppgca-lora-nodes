
#include <Arduino.h>

#include "lmic.h"
#include "hal/hal.h"
#include <SPI.h>
#include <string.h>

#define CFG_us915 1

static const PROGMEM u1_t NWKSKEY[16] = {  };

static const u1_t PROGMEM APPSKEY[16] = { };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0X1 ; //  ; // <-- Change this address for every node!


void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }


const lmic_pinmap lmic_pins = {
  .nss = 5,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 17,
  .dio = {2, 4, 16},
};

static osjob_t sendjob;
char TTN_response[30];

const unsigned TX_INTERVAL = 3;

uint32_t LatitudeBinary, LongitudeBinary;
uint16_t altitudeGps;
uint8_t hdopGps;
uint8_t txBuffer[9];


// Check GPS and returns string if full line recorded, else false
boolean getGPS()
{
  Serial.flush();
  String stringGPS = "";

  while (Serial.available()){
    char c = Serial.read();
    if (c == '\r') {
      if(stringGPS.length()>=6){
        if(stringGPS.substring(0, 6) == "$GPGGA"){
          char buf[stringGPS.length()];
          stringGPS.toCharArray(buf, stringGPS.length());
          char *p = buf;
          char *str;
          int i =0;
          Serial.println(stringGPS);

          char *tmpLat;
          char tmpSN;
          char *tmpEWn;
          char tmpEW;
          char *tmpAltitude;
          char *tmpHdo;

          while ((str = strtok_r(p, ",", &p)) != NULL) {
            if(i==2){
              tmpLat = str;
            }else if(i==3){
              tmpSN = *str;
            }else if(i==4){
              tmpEWn = str;
            }else if(i==5){
              tmpEW = *str;
            }else if(i==8){
              tmpHdo = str;
            }else if(i==9){
              tmpAltitude = str;
            }
            i++;
          }
          //se não chegou até o 9 é porque tem erro
          if(i<9){
            Serial.println("Fail GPS");
            digitalWrite(12, HIGH);
            delay(1000);
            digitalWrite(12, LOW);
            return false;
          }

          //Monta buffer de envio
          float flat = atof(tmpLat)/100;
          if(tmpSN=='S'){
            flat = flat*-1;
          }

          float flon = atof(tmpEWn)/100;
          if(tmpEW=='W'){
            flon = flon*-1;
          }
          altitudeGps = atof(tmpAltitude);
          hdopGps = atof(tmpHdo);

          Serial.print("lat:");
          Serial.println(flat);
          Serial.print("lon:");
          Serial.println(flon);

          LatitudeBinary = (flat + 90) * 16777215 / 180;
          LongitudeBinary = (flon + 180) * 16777215 / 360;

          txBuffer[0] = ( LatitudeBinary >> 16 ) & 0xFF;
          txBuffer[1] = ( LatitudeBinary >> 8 ) & 0xFF;
          txBuffer[2] = LatitudeBinary & 0xFF;

          txBuffer[3] = ( LongitudeBinary >> 16 ) & 0xFF;
          txBuffer[4] = ( LongitudeBinary >> 8 ) & 0xFF;
          txBuffer[5] = LongitudeBinary & 0xFF;


          txBuffer[6] = ( altitudeGps >> 8 ) & 0xFF;
          txBuffer[7] = altitudeGps & 0xFF;

          hdopGps = hdopGps*10;
          txBuffer[8] = hdopGps & 0xFF;

          return true;
        }
      }
      stringGPS = "";
    } else{
      stringGPS = stringGPS+c;
    }
  }
  Serial.println("#");
  digitalWrite(12, HIGH);
  delay(1000);
  digitalWrite(12, LOW);
  return false;
}

void do_send(osjob_t* j) {

  if(!getGPS()){
    txBuffer[0] = 0;
    txBuffer[1] = 0;
    txBuffer[2] =0;
    txBuffer[3] = 0;
    txBuffer[4] = 0;
    txBuffer[5] = 0;
    txBuffer[6] = 0;
    txBuffer[7  ] = 0;
  }
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    //LMIC_setTxData2(1, buffer, sizeof(buffer)/sizeof(buffer[0]), 0);
    LMIC_setTxData2(1, txBuffer, sizeof(txBuffer)/sizeof(txBuffer[0]), 0);
    //  Serial.println("Sending");
  }
  //}
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

    digitalWrite(13, HIGH);
    delay(1000);
    digitalWrite(13, LOW);

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

  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);

  LMIC.freq = 904300000;
  Serial.begin(9600);
  Serial.println("Started");

  os_init();
  LMIC_reset();
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

  #ifdef PROGMEM
  // On AVR, these values are stored in flash and only copied to RAM
  // once. Copy them to a temporary buffer here, LMIC_setSession will
  // copy them into a buffer of its own again.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
  #else
  // If not running an AVR with PROGMEM, just use the arrays directly
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
  #endif


  LMIC_setLinkCheckMode(0);
  //LMIC_setAdrMode(1);
  //LMIC.dn2Dr = DR_SF9;
  LMIC_setDrTxpow(DR_SF10,14);
  LMIC_startJoining();
  do_send(&sendjob);

  LMIC.dn2Dr = DR_SF9;
  LMIC_setDrTxpow(DR_SF10,14);
}


void loop()
{
  os_runloop_once();

}
