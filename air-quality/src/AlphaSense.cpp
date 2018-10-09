#include <Arduino.h>
#include <AlphaSense.h>


AlphaSense::AlphaSense(char* _gas, unsigned long _serial_no, unsigned int _we_zero_electronic,
  unsigned int _we_zero_total, unsigned int _ae_zero_electronic,
  unsigned int _ae_zero_total, float _sensitivity)
{
  strcpy(gas, _gas);
  serial_no = _serial_no;
  we_zero_electronic = _we_zero_electronic;
  we_zero_total = _we_zero_total;
  ae_zero_electronic = _ae_zero_electronic;
  ae_zero_total = _ae_zero_total;
  sensitivity = _sensitivity;
}

void AlphaSense::init(uint8_t i2cAddress) {
  ads = Adafruit_ADS1115(i2cAddress);
  ads.setGain(GAIN_TWOTHIRDS);
  ads.begin();

  Serial.print("Gas:");
  Serial.print(gas);
  Serial.print(";Serial:");
  Serial.print(serial_no);
  Serial.print(";we_zero_electronic:");
  Serial.print(we_zero_electronic);
  Serial.print(";we_zero_total:");
  Serial.print(we_zero_total);
  Serial.print(";ae_zero_electronic:");
  Serial.print(ae_zero_electronic);
  Serial.print(";ae_zero_total:");
  Serial.print(ae_zero_total);
  Serial.print(";sensitivity:");
  Serial.print(sensitivity);
  Serial.print(";i2cAddress:");
  Serial.print(i2cAddress);
  Serial.print(";GAIN_TWOTHIRDS:");
  Serial.print(GAIN_TWOTHIRDS);
  Serial.print(";ads_multiplier:");
  Serial.println(ads_multiplier);

}

void AlphaSense::readValue(int repeat, unsigned long _delay){

  float dif1=0 ;
  float dif2 =0;

  int i = 0;
  for(i = 0 ; i < repeat; i++){
    dif1 += ads.readADC_Differential_0_1();
    dif2 += ads.readADC_Differential_2_3();
    delay(_delay);
  }
  dif1 = dif1/repeat;
  dif2 = dif2/repeat;


  we_value = dif1 * ads_multiplier;
  ae_value = dif2 * ads_multiplier;

  /*Serial.print(";we:");
  Serial.print(we_value);
  Serial.print(";ae:");
  Serial.println(ae_value);*/
}

float AlphaSense::getWe(){
  return we_value;
}

float AlphaSense::getAe(){
  return ae_value;
}


float AlphaSense::algorithm_simple()
{

  float c = we_value - we_zero_total;
  if (c < 0)
  {
    c = 0;
  }

  float e = ae_value - ae_zero_total;
  if (e < 0)
  {
    e = 0;
  }

  float g = c - e;
  if (g < 0)
  {
    g = 0;
  }

  float h = g / sensitivity;


  Serial.print(gas);
  Serial.print(";");
  Serial.print(we_value);
  Serial.print(";");
  Serial.print(ae_value);
  Serial.print(";");
  Serial.println(h);

  return h;

}
