#include <Arduino.h>
#include <Adafruit_ADS1015.h>



class AlphaSense
{
public:
    char gas[5];
    unsigned long serial_no;
    unsigned int we_zero_electronic;
    unsigned int we_zero_total;
    unsigned int ae_zero_electronic;
    unsigned int ae_zero_total;
    float sensitivity;
    float ads_multiplier = 0.1875F;

    Adafruit_ADS1115  ads;

    float we_value;
    float ae_value;

    AlphaSense(char* gas, unsigned long _serial_no, unsigned int _we_zero_electronic, unsigned int _we_zero_total,
       unsigned int _ae_zero_electronic, unsigned int _ae_zero_total, float _sensitivity);

   void init(uint8_t i2cAddress);

   void readValue(int repeat, unsigned long delay);

   float getWe();
   float getAe();

   float algorithm_simple();
};
