#if defined(__AVR__)
#include <avr/pgmspace.h>
#include <Arduino.h>
#elif defined(ARDUINO_ARCH_ESP8266)
#include <ESP.h>
#elif defined(__MKL26Z64__)
#include <Arduino.h>
#else
#error Unknown architecture in aes.cpp
#endif

#include "lmic.h"
#include "hal/hal.h"
#include <SPI.h>
#include "DHT.h"

////////////////////////////

// Frame Counter
int count=0;
#define WAIT_SECS 3


// LoRaWAN Application identifier (AppEUI)
// Not used in this example
static const u1_t APPEUI[8] PROGMEM ={ };

// LoRaWAN DevEUI, unique device ID (LSBF)
// Not used in this example
static const u1_t DEVEUI[8] PROGMEM  = {};

// LoRaWAN NwkSKey, network session key
// Use this key for The Things Network
unsigned char NwkSkey[16] ={ };

// LoRaWAN AppSKey, application session key
// Use this key to get your data decrypted by The Things Network
unsigned char AppSkey[16] =		{ };

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace

#define msbf4_read(p)   (u4_t)((u4_t)(p)[0]<<24 | (u4_t)(p)[1]<<16 | (p)[2]<<8 | (p)[3])
unsigned char DevAddr[4] = {  };

// provide application router ID (8 bytes, LSBF)
void os_getArtEui (u1_t* buf) {
  memcpy(buf, APPEUI, 8);
}
// provide device ID (8 bytes, LSBF)
void os_getDevEui (u1_t* buf) {
  memcpy(buf, DEVEUI, 8);
}
// provide device key (16 bytes)
void os_getDevKey (u1_t* buf) {
  memcpy(buf, NwkSkey, 16);
}

int debug=1;
static osjob_t sendjob;

#define DHTPIN 8
#define DHTTYPE DHT22
#define LPP_TEMPERATURE 103
#define LPP_HUMIDITY 104


lmic_pinmap pins = {
  .nss = 6,
  .rxtx = 0,
  .rst = 5,
  .dio = {2, 3, 4},
};

DHT dht(DHTPIN, DHTTYPE);


void onEvent (ev_t ev) {
  //debug_event(ev);

  switch(ev) {
    // scheduled data sent (optionally data received)
    // note: this includes the receive window!
    case EV_TXCOMPLETE:
    // use this event to keep track of actual transmissions
    Serial.print("EV_TXCOMPLETE, time: ");
    Serial.println(millis() / 1000);
    if(LMIC.dataLen) { // data received in rx slot after tx
      //debug_buf(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
      Serial.println("Data Received");
    }
    break;
    default:
    break;
  }
}

void do_send(osjob_t* j) {
  Serial.println(LMIC.freq);

  int16_t val;

  byte buffer[7];
  uint8_t cursor = 0;

  float temperature = dht.readTemperature();
  //delay(2000);
  while(temperature==0 || isnan(temperature)){
    delay(1000);
    temperature = dht.readTemperature();
  }
  Serial.print("Temp:");
  Serial.println(temperature);

  float humidity =dht.readHumidity();

  while(humidity==0 || isnan(humidity)){
    delay(1000);
    humidity = dht.readHumidity();
  }
  Serial.print("hum:");
  Serial.println(humidity);

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

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, buffer, sizeof(buffer)/sizeof(buffer[0]), 0);
  }
// Schedule a timed job to run at the given timestamp (absolute system time)
  os_setTimedCallback(j, os_getTime()+sec2osticks(WAIT_SECS), do_send);

}

void setup() {
  Serial.begin(9600);
  Serial.println(F("Starting"));
  os_init();

  LMIC_reset();

  LMIC_setSession (0x1, msbf4_read(DevAddr), (uint8_t*)NwkSkey, (uint8_t*)AppSkey);

  // Disable data rate adaptation
  LMIC_setAdrMode(0);
  // Disable link check validation
  LMIC_setLinkCheckMode(0);
  // Disable beacon tracking
  LMIC_disableTracking ();
  // Stop listening for downstream data (periodical reception)
  LMIC_stopPingable();
  // Set data rate and transmit power (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7,14);
}


void loop() {

	do_send(&sendjob);
	while(1) {
		os_runloop_once();
		delay(100);
	}
}
