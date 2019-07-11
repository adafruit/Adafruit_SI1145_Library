/*************************************************** 
  This is a library for the Si1145 UV/IR/Visible Light Sensor

  Designed specifically to work with the Si1145 sensor in the
  adafruit shop
  ----> https://www.adafruit.com/products/1777

  These sensors use I2C to communicate, 2 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "Adafruit_SI1145.h"

Adafruit_SI1145::Adafruit_SI1145() {
  _addr = SI1145_ADDR;
  _vis_dark = 256; // as per Si114x manual - previously ADC_OFFSET but now fixed at 256
  _ir_dark = 256;
}


boolean Adafruit_SI1145::begin(void) {
  Wire.begin();
 
  uint8_t id = read8(SI1145_REG_PARTID);
  if (id != 0x45) return false; // look for SI1145
  
  reset();
  

    /***********************************/
  // enable UVindex measurement coefficients!
  write8(SI1145_REG_UCOEFF0, 0x29);
  write8(SI1145_REG_UCOEFF1, 0x89);
  write8(SI1145_REG_UCOEFF2, 0x02);
  write8(SI1145_REG_UCOEFF3, 0x00);

  // enable UV sensor
  writeParam(SI1145_PARAM_CHLIST, SI1145_PARAM_CHLIST_ENUV |
  SI1145_PARAM_CHLIST_ENALSIR | SI1145_PARAM_CHLIST_ENALSVIS |
  SI1145_PARAM_CHLIST_ENPS1);
  // enable interrupt on every sample
  write8(SI1145_REG_INTCFG, SI1145_REG_INTCFG_INTOE);  
  write8(SI1145_REG_IRQEN, SI1145_REG_IRQEN_ALSEVERYSAMPLE);  

/****************************** Prox Sense 1 */

  // program LED current
  write8(SI1145_REG_PSLED21, 0x03); // 20mA for LED 1 only
  writeParam(SI1145_PARAM_PS1ADCMUX, SI1145_PARAM_ADCMUX_LARGEIR);
  // prox sensor #1 uses LED #1
  writeParam(SI1145_PARAM_PSLED12SEL, SI1145_PARAM_PSLED12SEL_PS1LED1);
  // fastest clocks, clock div 1
  writeParam(SI1145_PARAM_PSADCGAIN, 0);
  // take 511 clocks to measure
  writeParam(SI1145_PARAM_PSADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
  // in prox mode, high range
  writeParam(SI1145_PARAM_PSADCMISC, SI1145_PARAM_PSADCMISC_RANGE|
    SI1145_PARAM_PSADCMISC_PSMODE);

  writeParam(SI1145_PARAM_ALSIRADCMUX, SI1145_PARAM_ADCMUX_SMALLIR);  
  // fastest clocks, clock div 1
  writeParam(SI1145_PARAM_ALSIRADCGAIN, 0);
  // take 511 clocks to measure
  writeParam(SI1145_PARAM_ALSIRADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
  // in high range mode
  writeParam(SI1145_PARAM_ALSIRADCMISC, SI1145_PARAM_ALSIRADCMISC_RANGE);



  // fastest clocks, clock div 1
  writeParam(SI1145_PARAM_ALSVISADCGAIN, 0);
  // take 511 clocks to measure
  writeParam(SI1145_PARAM_ALSVISADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
  // in high range mode (not normal signal)
  writeParam(SI1145_PARAM_ALSVISADCMISC, SI1145_PARAM_ALSVISADCMISC_VISRANGE);


/************************/

  // measurement rate for auto
  write8(SI1145_REG_MEASRATE0, 0xFF); // 255 * 31.25uS = 8ms
  
  // auto run
  write8(SI1145_REG_COMMAND, SI1145_PSALS_AUTO);

  return true;
}

void Adafruit_SI1145::reset() {
  write8(SI1145_REG_MEASRATE0, 0);
  write8(SI1145_REG_MEASRATE1, 0);
  write8(SI1145_REG_IRQEN, 0);
  write8(SI1145_REG_IRQMODE1, 0);
  write8(SI1145_REG_IRQMODE2, 0);
  write8(SI1145_REG_INTCFG, 0);
  write8(SI1145_REG_IRQSTAT, 0xFF);

  write8(SI1145_REG_COMMAND, SI1145_RESET);
  delay(10);
  write8(SI1145_REG_HWKEY, 0x17);
  
  delay(10);
}


//////////////////////////////////////////////////////
// returns the visible gain
Adafruit_SI1145::Gain Adafruit_SI1145::readVisibleGain() {
  return (Adafruit_SI1145::Gain) readParam(SI1145_PARAM_ALSVISADCGAIN);
}

// adjust the visible gain
void Adafruit_SI1145::setVisibleGain(Adafruit_SI1145::Gain gain) {
 uint8_t g = (uint8_t) gain;
 writeParam(SI1145_PARAM_ALSVISADCGAIN, g);
}

// returns the IR gain
Adafruit_SI1145::Gain Adafruit_SI1145::readIRGain() {
  return (Adafruit_SI1145::Gain) readParam(SI1145_PARAM_ALSIRADCGAIN);
}

// adjust the IR gain
void Adafruit_SI1145::setIRGain(Adafruit_SI1145::Gain gain) {
 uint8_t g = (uint8_t) gain;
 writeParam(SI1145_PARAM_ALSIRADCGAIN, g);
}

// returns the UV index * 100 (divide by 100 to get the index)
uint16_t Adafruit_SI1145::readUV(void) {
 return read16(0x2C); 
}

// returns a calculated lux value as per SI144x AN523.6.
float Adafruit_SI1145::calculateLux(uint16_t vis, uint16_t ir) {
 uint8_t gain = readVisibleGain();
 float lux = ((5.41f * vis) + (-0.08f * ir)) / (1 + 2 * gain);
 return lux;
}

// returns visible+IR light levels
uint16_t Adafruit_SI1145::readVisible(void) {
 uint16_t vis = read16(SI1145_REG_ALSVISDATA0);
 if (vis < _vis_dark)
  _vis_dark = vis;
 vis -= _vis_dark;
 return vis;
}

// returns IR light levels
uint16_t Adafruit_SI1145::readIR(void) {
 uint16_t ir = read16(SI1145_REG_ALSIRDATA0);
 if (ir < _ir_dark)
  _ir_dark = ir;
 ir -= _ir_dark;
 return ir;
}

// returns "Proximity" - assumes an IR LED is attached to LED
uint16_t Adafruit_SI1145::readProx(void) {
 return read16(SI1145_REG_PS1DATA0);
}

/*********************************************************************/

uint8_t Adafruit_SI1145::writeParam(uint8_t p, uint8_t v) {
  //Serial.print("Param 0x"); Serial.print(p, HEX);
  //Serial.print(" = 0x"); Serial.println(v, HEX);
  
  write8(SI1145_REG_PARAMWR, v);
  write8(SI1145_REG_COMMAND, p | SI1145_PARAM_SET);
  return read8(SI1145_REG_PARAMRD);
}

uint8_t Adafruit_SI1145::readParam(uint8_t p) {
  write8(SI1145_REG_COMMAND, p | SI1145_PARAM_QUERY);
  return read8(SI1145_REG_PARAMRD);
}

/*********************************************************************/

uint8_t  Adafruit_SI1145::read8(uint8_t reg) {
  uint16_t val;
  Wire.beginTransmission(_addr);
  Wire.write((uint8_t)reg);
  Wire.endTransmission();

  Wire.requestFrom((uint8_t)_addr, (uint8_t)1);
  return Wire.read();
}

uint16_t Adafruit_SI1145::read16(uint8_t a) {
  uint16_t ret;

  Wire.beginTransmission(_addr); // start transmission to device 
  Wire.write(a); // sends register address to read from
  Wire.endTransmission(); // end transmission
  
  Wire.requestFrom(_addr, (uint8_t)2);// send data n-bytes read
  ret = Wire.read(); // receive DATA
  ret |= (uint16_t)Wire.read() << 8; // receive DATA

  return ret;
}

void Adafruit_SI1145::write8(uint8_t reg, uint8_t val) {

  Wire.beginTransmission(_addr); // start transmission to device 
  Wire.write(reg); // sends register address to write
  Wire.write(val); // sends value
  Wire.endTransmission(); // end transmission
}
