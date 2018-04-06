  #include <Arduino.h>

  #include "lmic.h"
  #include "hal/hal.h"
  #include <SPI.h>
  #include "DHT.h"

  #include "config.h"


  #define debug 0;
  #define CFG_us915 1 // alterar no arquivo de configuracao


  void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

  void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

  void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16);}


  const lmic_pinmap lmic_pins = {
    .nss = 6,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 5,
    .dio = {2, 3, 4},
  };

  static osjob_t sendjob;
  byte buffer[8];
  byte TTN_response[3];
  #define TX_INTERVAL  30
  bool joining = false;

  #define DHTPIN 7
  #define DHTTYPE DHT22
  DHT dht(DHTPIN, DHTTYPE);
  float temp;
  float hum;

  // mq7
  #define mq7pin 0//the AOUT pin of the CO sensor goes into analog pin A0 of the arduino
  unsigned int mq7Value;

  #define mpxPin 1
  unsigned long  mpxValue;
  float mpxAverage = 0;
  bool car = 1;
  int countTraffic=0;

  ///


  unsigned long lastUplinkMillis = 0;
  unsigned long currentMillis = 0;

  /*#define ledMsgPin  10*/
  #define ledCountPin  9


  void do_send(osjob_t* j) {

    temp = dht.readTemperature();
    hum = dht.readHumidity();

    mq7Value= analogRead(mq7pin);

    Serial.print(countTraffic);
    Serial.print(";");
    Serial.print( (int) (temp*100));
    Serial.print(";");
    Serial.print((int) (hum *10));
    Serial.print(";");
    Serial.println(mq7Value);

    //buffer
    buffer[0] = (countTraffic & 0xFF00) >> 8;
    buffer[1] = (countTraffic & 0x00FF);
    buffer[2] = (((int) (temp*100))& 0xFF00) >> 8;
    buffer[3] =  (((int) (temp*100))& 0x00FF);
    buffer[4] = (((int) (hum*10))& 0xFF00) >> 8;
    buffer[5] =  (((int) (hum*100))& 0x00FF);
    buffer[6] = (((int) (mq7Value*10))& 0xFF00) >> 8;
    buffer[7] =  (((int) (mq7Value*100))& 0x00FF);

    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
      Serial.println(F("OP_TXRXPEND, not sending"));
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

      if (LMIC.dataLen) {
        for (int i = 0; i < LMIC.dataLen; i++) {
          TTN_response[i] = LMIC.frame[LMIC.dataBeg + i];
        }

        if(TTN_response[0]){// retorno OK
          int retorno = (TTN_response[1] << 8)
          + TTN_response[2];

          countTraffic = countTraffic - retorno;
          if(countTraffic<0){
            countTraffic = 0;
          }
        }
      }
      /*digitalWrite(ledMsgPin, HIGH);
      delay(100);
      digitalWrite(ledMsgPin, LOW);
      */
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
      joining = true;

      break;
      case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      joining = false;

      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      /*  digitalWrite(ledMsgPin, HIGH);
      delay(100);
      digitalWrite(ledMsgPin, LOW);*/
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
    /*pinMode(ledMsgPin, OUTPUT);*/
    pinMode(ledCountPin, OUTPUT);

    dht.begin();

    os_init();
    LMIC_reset();

    LMIC.freq = 904300000;
    LMIC_setLinkCheckMode(0);
    //LMIC_setAdrMode(1);
    //LMIC.dn2Dr = DR_SF9;
    //LMIC_setDrTxpow(DR_SF9,14);
    //LMIC_startJoining();
    do_send(&sendjob);
    LMIC.dn2Dr = DR_SF9;
    LMIC_setDrTxpow(DR_SF9,14);


  }

  void loop() {
    os_runloop_once();
    currentMillis = millis();
    if(!joining){
      if(currentMillis<5000){ // avg pressao
        mpxAverage +=((float)mpxValue-mpxAverage)/10.f;
      }else{
        mpxValue = 0;
        for(int i = 0; i <10;i++){
          mpxValue += analogRead(mpxPin);
          delay(1);
        }
        mpxValue = mpxValue/10;

        if(mpxValue>mpxAverage +3 //){

          && mpxAverage-(int)mpxAverage!=0.0 ){//gamb de noiser
            car = !car;
            if(car){
              countTraffic++;
              digitalWrite(ledCountPin, HIGH);
              delay(100);
              digitalWrite(ledCountPin, LOW);
            }
              Serial.print("#");
              Serial.println(countTraffic);
              Serial.print(";");
              Serial.print(mpxAverage);
              Serial.print(";");
              Serial.println(mpxValue);
            mpxAverage =mpxValue;

          }else{
            mpxAverage +=((float)mpxValue-mpxAverage)/10.f;
            //Serial.print(mpxAverage);
            //Serial.print(";");
            //Serial.println(mpxValue);
          }
        }
      }
    }
