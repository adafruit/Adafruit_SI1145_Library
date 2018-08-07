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
#include "Adafruit_SI1145.h"

Adafruit_SI1145::Adafruit_SI1145() {
  _lastError = 0;
  _addr = SI1145_ADDR;
  ADCOffset = 0;
}


boolean Adafruit_SI1145::begin(bool autoMeasurements) {
  _lastError = 0;
  Wire.begin();
 
  uint8_t id = readPartId();
  if (id != 0x45 || _lastError) return false; // look for SI1145
  
  reset();
  if (_lastError) return false;

  /***********************************/
  // enable UVindex measurement coefficients!
  write8(SI1145_REG_UCOEFF0, 0x29);
  write8(SI1145_REG_UCOEFF1, 0x89);
  write8(SI1145_REG_UCOEFF2, 0x02);
  write8(SI1145_REG_UCOEFF3, 0x00);

  // enable sensors: uv, ir, vis, ps1 (channel0), ps2 (channel1)
  writeParam(SI1145_PARAM_CHLIST, SI1145_PARAM_CHLIST_ENUV |
  SI1145_PARAM_CHLIST_ENALSIR | SI1145_PARAM_CHLIST_ENALSVIS |
  SI1145_PARAM_CHLIST_ENPS1 | SI1145_PARAM_CHLIST_ENPS2 | SI1145_PARAM_CHLIST_ENPS3);

  // enable interrupt on every sample
  write8(SI1145_REG_INTCFG, SI1145_REG_INTCFG_INTOE);  
  write8(SI1145_REG_IRQEN, SI1145_REG_IRQEN_ALSEVERYSAMPLE);  

  /****************************** Prox Sense 1 (PS1) */
  // program LED current
  write8(SI1145_REG_PSLED21, 0x03); // 20mA for LED 1 only
  writeParam(SI1145_PARAM_PS1ADCMUX, SI1145_PARAM_ADCMUX_LARGEIR);
  // prox sensor #1 uses LED #1
  writeParam(SI1145_PARAM_PSLED12SEL, SI1145_PARAM_PSLED12SEL_PS1LED1);
  // fastest clocks, clock div 1
  writeParam(SI1145_PARAM_PSADCGAIN, SI114X_ADC_GAIN_DIV1);
  // take 511 clocks to measure
  writeParam(SI1145_PARAM_PSADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
  // in prox mode, high range
  writeParam(SI1145_PARAM_PSADCMISC, SI1145_PARAM_PSADCMISC_RANGE|
    SI1145_PARAM_PSADCMISC_PSMODE);

  /****************************** PS2 - reference for measurement */
  writeParam(SI1145_PARAM_PS2ADCMUX, SI1145_PARAM_ADCMUX_NO_PHOTODIODE);

  /****************************** PS3 - temperature */
  writeParam(SI1145_PARAM_PS3ADCMUX, SI1145_PARAM_ADCMUX_TEMPERATURE);

  /****************************** IR */
  writeParam(SI1145_PARAM_ALSIRADCMUX, SI1145_PARAM_ADCMUX_SMALLIR);
  // fastest clocks, clock div 1
  writeParam(SI1145_PARAM_ALSIRADCGAIN, SI114X_ADC_GAIN_DIV32);
  // take 511 clocks to measure
  writeParam(SI1145_PARAM_ALSIRADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
  // in high range mode
  writeParam(SI1145_PARAM_ALSIRADCMISC, SI1145_PARAM_ALSIRADCMISC_RANGE_HIGH);


  /****************************** Visible */
  // fastest clocks, clock div 1
  writeParam(SI1145_PARAM_ALSVISADCGAIN, SI114X_ADC_GAIN_DIV32);
  // take 511 clocks to measure
  writeParam(SI1145_PARAM_ALSVISADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
  // in high range mode (not normal signal)
  writeParam(SI1145_PARAM_ALSVISADCMISC, SI1145_PARAM_ALSVISADCMISC_VISRANGE_HIGH);

  /****************************** Read ADC negative offset */
  ADCOffset = uncompress8bto16b(readParam(SI1145_PARAM_ADC_OFFSET));

  /************************/

  if (autoMeasurements) {
    // measurement rate for auto
    write8(SI1145_REG_MEASRATE0, 0xFF); // 255 * 31.25uS = 8ms
    // auto run
    write8(SI1145_REG_COMMAND, SI1145_PSALS_AUTO);
  } else {
    // measurement rate none
    write8(SI1145_REG_MEASRATE0, 0x00);
    write8(SI1145_REG_MEASRATE1, 0x00);
    // stop all commands
    write8(SI1145_REG_COMMAND, SI1145_NOP);
  }

  return !_lastError;
}

uint8_t Adafruit_SI1145::reset() {
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

  return _lastError;
}

uint8_t Adafruit_SI1145::getLastError() {
  uint8_t res = _lastError;
  _lastError = 0;
  return res;
}

uint8_t Adafruit_SI1145::readPartId() {
  return read8(SI1145_REG_PARTID);
}

uint8_t Adafruit_SI1145::readRevId() {
  return read8(SI1145_REG_REVID);
}

uint8_t Adafruit_SI1145::readSeqId() {
  return read8(SI1145_REG_SEQID);
}

/* Expand 8 bit compressed value to 16 bit, see Silabs AN498 */
uint16_t Adafruit_SI1145::uncompress8bto16b(uint8_t x) {
  uint16_t result = 0;
  uint8_t exponent = 0;

  if (x < 8)
    return 0;

  exponent = (x & 0xf0) >> 4;
  result = 0x10 | (x & 0x0f);

  if (exponent >= 4)
    return result << (exponent - 4);
  return result >> (4 - exponent);
}

//////////////////////////////////////////////////////

// returns the UV index * 100 (divide by 100 to get the index)
uint16_t Adafruit_SI1145::readUV(void) {
 return read16(SI1145_REG_UVINDEX0);
}

// returns visible+IR light levels
uint16_t Adafruit_SI1145::readVisible(void) {
 return read16(SI1145_REG_ALSVISDATA0);
}

// returns IR light levels
uint16_t Adafruit_SI1145::readIR(void) {
 return read16(SI1145_REG_ALSIRDATA0);
}

// returns "Proximity" - assumes an IR LED is attached to LED
uint16_t Adafruit_SI1145::readProx(void) {
  return read16(SI1145_REG_PS1DATA0);
}

uint16_t Adafruit_SI1145::readPS2() {
  return read16(SI1145_REG_PS2DATA0);
}

uint16_t Adafruit_SI1145::readPS3() {
  return read16(SI1145_REG_PS3DATA0);
}

uint8_t Adafruit_SI1145::readVisibleGain() {
  return readParam(SI1145_PARAM_ALSVISADCGAIN);
}
 // adjust the visible gain
void Adafruit_SI1145::setVisibleGain(bool highRange, uint8_t gain) {
  writeParam(SI1145_PARAM_ALSVISADCGAIN, (gain & 0x07));
  writeParam(SI1145_PARAM_ALSVISADCMISC, highRange?SI1145_PARAM_ALSVISADCMISC_VISRANGE_HIGH:SI1145_PARAM_ALSVISADCMISC_VISRANGE_LOW);
}
 // returns the IR gain
uint8_t Adafruit_SI1145::readIRGain() {
  return readParam(SI1145_PARAM_ALSIRADCGAIN);
}
 // adjust the IR gain
void Adafruit_SI1145::setIRGain(bool highRange, uint8_t gain) {
 writeParam(SI1145_PARAM_ALSIRADCGAIN, (gain & 0x07));
 writeParam(SI1145_PARAM_ALSIRADCMISC, highRange?SI1145_PARAM_ALSIRADCMISC_RANGE_HIGH:SI1145_PARAM_ALSIRADCMISC_RANGE_LOW);
}

uint16_t Adafruit_SI1145::getADCOffset() const {
  return ADCOffset;
}

uint8_t Adafruit_SI1145::ExecuteCommand(uint8_t command) {
  write8(SI1145_REG_COMMAND, SI1145_NOP);
  // check permanent error
  uint8_t err = read8(SI1145_REG_RESPONSE);
  if(err)
    return err;

  uint8_t iResp = 0x00;
  write8(SI1145_REG_COMMAND, command);

  uint32_t tmr = millis();
  while (iResp == 0x00) {
    iResp = read8(SI1145_REG_RESPONSE);

    // 25ms timeout from datasheet
    if (tmr + 25 < millis())
      return 0xFF;
  }

  if ((iResp & 0xF0) == 0)
    return 0;
  else {
    // clear the error
    write8(SI1145_REG_COMMAND, SI1145_NOP);
    return iResp; // return error code
  }
}

uint8_t Adafruit_SI1145::takeForcedMeasurement() {
  return ExecuteCommand(SI1145_PSALS_FORCE);
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
    uint8_t res = Wire.endTransmission();
    if (res) {
      _lastError = res;
      return 0;
    }

    res = Wire.requestFrom((uint8_t)_addr, (uint8_t)1);
    if (res != 1) {
      _lastError = 100;
      return 0;
    }
    return Wire.read();
}

uint16_t Adafruit_SI1145::read16(uint8_t a) {
  uint16_t ret;

  Wire.beginTransmission(_addr); // start transmission to device 
  Wire.write(a); // sends register address to read from
  uint8_t res = Wire.endTransmission(); // end transmission
  // check transmission error
  if (res) {
    _lastError = res;
    return 0;
  }

  res = Wire.requestFrom(_addr, (uint8_t)2);// send data n-bytes read
  if (res != 2) {
    _lastError = 100;
    return 0;
  }
  ret = Wire.read(); // receive DATA
  ret |= (uint16_t)Wire.read() << 8; // receive DATA

  return ret;
}

void Adafruit_SI1145::write8(uint8_t reg, uint8_t val) {

  Wire.beginTransmission(_addr); // start transmission to device 
  Wire.write(reg); // sends register address to write
  Wire.write(val); // sends value
  uint8_t res = Wire.endTransmission(); // end transmission
  // check transmission error
  if (res) {
    _lastError = res;
  }
}
