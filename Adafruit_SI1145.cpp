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

// *******************************************************************
// calibration code from Si114x_functions.c from chip manufacturer
// *******************************************************************

#define FLT_TO_FX20(x)       ((x*1048576)+.5)
#define FX20_ONE             FLT_TO_FX20( 1.000000)
#define FX20_BAD_VALUE       0xffffffff

//                                            msb   lsb   align
//                                            i2c   i2c   ment
//                                            addr  addr
#define SIRPD_ADCHI_IRLED    (collect(buffer, 0x23, 0x22,  0))
#define SIRPD_ADCLO_IRLED    (collect(buffer, 0x22, 0x25,  1))
#define SIRPD_ADCLO_WHLED    (collect(buffer, 0x24, 0x26,  0))
#define VISPD_ADCHI_WHLED    (collect(buffer, 0x26, 0x27,  1))
#define VISPD_ADCLO_WHLED    (collect(buffer, 0x28, 0x29,  0))
#define LIRPD_ADCHI_IRLED    (collect(buffer, 0x29, 0x2a,  1))
#define LED_DRV65            (collect(buffer, 0x2b, 0x2c,  0))

// Structure Definition for calref array
struct cal_ref_t
{
    u32 sirpd_adchi_irled;
    u32 sirpd_adclo_irled;
    u32 sirpd_adclo_whled;
    u32 vispd_adchi_whled;
    u32 vispd_adclo_whled;
    u32 lirpd_adchi_irled;
    u32 ledi_65ma;
    u8  ucoef[4];
};

// Factory Calibration Reference Values
static struct cal_ref_t calref[2] =
{
    {
        FLT_TO_FX20( 4.021290),  // sirpd_adchi_irled
        FLT_TO_FX20(57.528500),  // sirpd_adclo_irled
        FLT_TO_FX20( 2.690010),  // sirpd_adclo_whled
        FLT_TO_FX20( 0.042903),  // vispd_adchi_whled
        FLT_TO_FX20( 0.633435),  // vispd_adclo_whled
        FLT_TO_FX20(23.902900),  // lirpd_adchi_irled
        FLT_TO_FX20(56.889300),  // ledi_65ma
        {0x7B, 0x6B, 0x01, 0x00} // default ucoef
    },
    {
        FLT_TO_FX20( 2.325484),  // sirpd_adchi_irled
        FLT_TO_FX20(33.541500),  // sirpd_adclo_irled
        FLT_TO_FX20( 1.693750),  // sirpd_adclo_whled
        FLT_TO_FX20( 0.026775),  // vispd_adchi_whled
        FLT_TO_FX20( 0.398443),  // vispd_adclo_whled
        FLT_TO_FX20(12.190900),  // lirpd_adchi_irled
        FLT_TO_FX20(56.558200),  // ledi_65ma
        {0xdb, 0x8f, 0x01, 0x00} // default ucoef
    }
};

// Converts the 12-bit factory test value from the Si114x and returns the
// fixed-point representation of this 12-bit factory test value.
static u32 decode(u32 input)
{
    s32  exponent, exponent_bias9;
    u32  mantissa;

    if (input==0) return 0.0;

    exponent_bias9 = (input & 0x0f00) >> 8;
    exponent       = exponent_bias9 - 9;

    mantissa       = input & 0x00ff; // fraction
    mantissa       |=        0x0100; // add in integer

    // representation in 12 bit integer, 20 bit fraction
    mantissa       = mantissa << (12+exponent);
    return mantissa;
}

// The buffer[] is assumed to point to a byte array that containst the
// factory calibration values after writing 0x12 to the command register
// This function takes the 12 bytes from the Si114x, then converts it
// to a fixed point representation, with the help of the decode() function
static u32 collect(u8 *buffer, u8 msb_addr, u8 lsb_addr, u8 alignment)
{
    u16 value;
    u8  msb_ind = msb_addr - 0x22;
    u8  lsb_ind = lsb_addr - 0x22;

    if (alignment == 0)
    {
        value =  buffer[msb_ind]<<4;
        value += buffer[lsb_ind]>>4;
    }
    else
    {
        value =  buffer[msb_ind]<<8;
        value += buffer[lsb_ind];
        value &= 0x0fff;
    }

    if (    ( value == 0x0fff )
         || ( value == 0x0000 ) ) return FX20_BAD_VALUE;
    else return decode( value );
}

// This performs a shift_left function. For convenience, a negative
// shift value will shift the value right. Value pointed will be
// overwritten.
static void shift_left(u32 *value_p, s8 shift)
{
    if (shift > 0)
        *value_p = *value_p<<shift ;
    else
        *value_p = *value_p>>(-shift) ;
}

// Aligns the value pointed by value_p to either the LEFT or the RIGHT
// the number of shifted bits is returned. The value in value_p is
// overwritten.
#define ALIGN_LEFT   1
#define ALIGN_RIGHT -1
static s8 align( u32 *value_p, s8 direction )
{
    s8  local_shift, shift ;
    u32 mask;

    // Check invalid value_p and *value_p, return without shifting if bad.
    if( value_p  == NULL )  return 0;
    if( *value_p == 0 )     return 0;

    // Make sure direction is valid
    switch( direction )
    {
        case ALIGN_LEFT:
            local_shift =  1 ;
            mask  = 0x80000000;
            break;

        case ALIGN_RIGHT:
            local_shift = -1 ;
            mask  = 0x00000001;
            break;

        default:
            // Invalid direction, return without shifting
            return 0;
    }

    shift = 0;
    while(1)
    {
        if (*value_p & mask ) break;
        shift++;
        shift_left( value_p, local_shift );
    }
    return shift;
}


// The fx20_round was designed to perform rounding to however many significant
// digits. However, for the factory calibration code, rounding to 16 always is
// sufficient. So, the FORCE_ROUND_16 define is provided just in case it would
// be necessary to dynamically choose how many bits to round to.
#define FORCE_ROUND_16 1

// fx20_round Rounds the u32 value pointed by ptr, by the number of bits specified by round.
//
// This compile switch used only to experiment with  various rounding precisions.
// The flexibility has a small performance price.
static void fx20_round
(
    u32 *value_p
    #if !FORCE_ROUND_16
    , s8 round
    #endif
)
{
    s8  shift;

    #if FORCE_ROUND_16
        // Use the following to force round = 16
        u32 mask1  = 0xffff8000;
        u32 mask2  = 0xffff0000;
        u32 lsb    = 0x00008000;
    #else
        // Use the following if you want to routine to be
        // capable of rounding to something other than 16.
        u32 mask1  = ((2<<(round))-1)<<(31-(round));
        u32 mask2  = ((2<<(round-1))-1)<<(31-(round-1));
        u32 lsb    = mask1-mask2;
    #endif

    shift = align( value_p, ALIGN_LEFT );
    if( ( (*value_p)&mask1 ) == mask1 )
    {
        *value_p = 0x80000000;
        shift -= 1;
    }
    else
    {
        *value_p += lsb;
        *value_p &= mask2;
    }

    shift_left( value_p, -shift );
}

// The fx20_divide and fx20_multiply uses this structure to pass
// values into it.
struct operand_t
{
   u32 op1;
   u32 op2;
};

// Returns a fixed-point (20-bit fraction) after dividing op1/op2
static u32 fx20_divide( struct operand_t *operand_p )
{
    s8  numerator_sh=0, denominator_sh=0;
    u32 result;
    u32 *numerator_p;
    u32 *denominator_p;

    if ( operand_p == NULL ) return FX20_BAD_VALUE;

    numerator_p   = &operand_p->op1;
    denominator_p = &operand_p->op2;

    if (   (*numerator_p   == FX20_BAD_VALUE)
        || (*denominator_p == FX20_BAD_VALUE)
        || (*denominator_p == 0             ) ) return FX20_BAD_VALUE;

    fx20_round  ( numerator_p   );
    fx20_round  ( denominator_p );
    numerator_sh   = align ( numerator_p,   ALIGN_LEFT  );
    denominator_sh = align ( denominator_p, ALIGN_RIGHT );

    result = *numerator_p / ( (u16)(*denominator_p) );
    shift_left( &result , 20-numerator_sh-denominator_sh );

    return result;
}

// Returns a fixed-point (20-bit fraction) after multiplying op1*op2
static u32 fx20_multiply( struct operand_t *operand_p )
{
    u32 result;
    s8  val1_sh, val2_sh;
    u32 *val1_p;
    u32 *val2_p;

    if( operand_p == NULL ) return FX20_BAD_VALUE;

    val1_p = &(operand_p->op1);
    val2_p = &(operand_p->op2);

    fx20_round( val1_p );
    fx20_round( val2_p );

    val1_sh = align( val1_p, ALIGN_RIGHT );
    val2_sh = align( val2_p, ALIGN_RIGHT );


    result = (u32)( ( (u32)(*val1_p) ) * ( (u32)(*val2_p) ) );
    shift_left( &result, -20+val1_sh+val2_sh );

    return result;
}

// Due to small differences in factory test setup, the reference calibration
// values may have slight variation. This function retrieves the calibration
// index stored in the Si114x so that it is possible to know which calibration
// reference values to use.
static s16 cal_index( u8 *buffer )
{
    s16 index;
    u8  size;

    // buffer[12] is the LSB, buffer[13] is the MSB
    index = ( s16 )( buffer[12] + ( (u16)( buffer[13] ) << 8 ) );
Serial1.printf("indx:%d\r\n", index);
    switch( index )
    {
        case -1:
            index = 0;
            break;
        default:
            index = -(2+index) ;
    }

    size = sizeof(calref)/sizeof(calref[0]);

    if ( index < size )
    {
        return  index;
    }
    else
    {
        return -1;
    }
}

// Returns the calibration ratio to be applied to VIS measurements
static u32 vispd_correction(u8 *buffer)
{

    struct operand_t op;
    u32              result;
    s16              index = cal_index( buffer );

    if ( index < 0 ) result = FX20_ONE;

    op.op1 = calref[ index ].vispd_adclo_whled;
    op.op2 = VISPD_ADCLO_WHLED;
    result = fx20_divide( &op );

    if ( result == FX20_BAD_VALUE ) result = FX20_ONE;

    return result;
}

// Returns the calibration ratio to be applied to IR measurements
static u32 irpd_correction(u8 *buffer)
{
    struct operand_t op;
    u32              result;
    s16              index = cal_index( buffer );

    if ( index < 0 ) result = FX20_ONE;

    // op.op1 = SIRPD_ADCLO_IRLED_REF; op.op2 = SIRPD_ADCLO_IRLED;
    op.op1 = calref[ index ].sirpd_adclo_irled;
    op.op2 = SIRPD_ADCLO_IRLED;
    result = fx20_divide( &op );

    if ( result == FX20_BAD_VALUE ) result = FX20_ONE;

    return result;
}

// Returns the ratio to correlate between x_RANGE=0 and x_RANGE=1
// It is typically 14.5, but may have some slight component-to-component
// differences.
static u32 adcrange_ratio(u8 *buffer)
{
    struct operand_t op;
    u32              result;

    op.op1 = SIRPD_ADCLO_IRLED  ; op.op2 = SIRPD_ADCHI_IRLED  ;
    result = fx20_divide( &op );

    if ( result == FX20_BAD_VALUE ) result = FLT_TO_FX20( 14.5 );

    return result;
}

// Returns the ratio to correlate between measurements made from large PD
// to measurements made from small PD.
static u32 irsize_ratio(u8 *buffer)
{
    struct operand_t op;
    u32              result;

    op.op1 = LIRPD_ADCHI_IRLED  ; op.op2 = SIRPD_ADCHI_IRLED  ;
    result = fx20_divide( &op );

    if ( result == FX20_BAD_VALUE ) result = FLT_TO_FX20(  6.0 );

    return  result;
}

// Returns the ratio to adjust for differences in IRLED drive strength. Note
// that this does not help with LED irradiance variation.
static u32 ledi_ratio(u8 *buffer)
{

    struct operand_t op;
    u32              result;
    s16              index = cal_index( buffer );

    if ( index < 0 ) result = FX20_ONE;

    // op.op1 = LED_DRV65_REF; op.op2 = LED_DRV65;
    op.op1 = calref[ index ].ledi_65ma;
    op.op2 = LED_DRV65;
    result = fx20_divide( &op );

    if ( result == FX20_BAD_VALUE ) result = FX20_ONE;

    return result;
}

// *******************************************************************
// end of calibration code
// *******************************************************************


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

  // calibration
  if (readCalibrationParameters()) {
    delay(10);

    /***********************************/
    // update calibrated UVindex measurement coefficients!
    SI114X_CAL_S *cparams = getCalibrationParameters();
    write8(SI1145_REG_UCOEFF0, cparams->ucoef_p[0]);
    write8(SI1145_REG_UCOEFF1, cparams->ucoef_p[1]);
    write8(SI1145_REG_UCOEFF2, cparams->ucoef_p[2]);
    write8(SI1145_REG_UCOEFF3, cparams->ucoef_p[3]);

    Serial1.printf("Calibr OK. [%d]\n", cparams->ucoef_p[0]);

  } else {
    delay(10);
    Serial1.printf("Calibr error\n");

    /***********************************/
    // enable UVindex measurement coefficients!
    write8(SI1145_REG_UCOEFF0, 0x29);
    write8(SI1145_REG_UCOEFF1, 0x89);
    write8(SI1145_REG_UCOEFF2, 0x02);
    write8(SI1145_REG_UCOEFF3, 0x00);
  }

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

bool Adafruit_SI1145::readCalibrationParameters() {
  uint8_t buffer[14] = {0};

  write8(SI1145_REG_UCOEFF0, 0x7B);
  write8(SI1145_REG_UCOEFF1, 0x6B);
  write8(SI1145_REG_UCOEFF2, 0x01);
  write8(SI1145_REG_UCOEFF3, 0x00);

  ExecuteCommand(SI1145_GET_CAL);

  if (readBytes(SI1145_REG_ALSVISDATA0, 12, buffer)) {
    Serial1.println("error cal read bytes!!!");
    return false;
  };

  ExecuteCommand(SI1145_GET_CAL_INDX);

  if (readBytes(SI1145_REG_PS1DATA0, 2, &buffer[12])) {
    Serial1.println("error cal indx read bytes!!!");
    return false;
  };

  Serial1.println("buffer:");
  Serial1.println(buffer[0]);
  Serial1.println(buffer[1]);
  Serial1.println(buffer[2]);
  Serial1.println(buffer[3]);
  Serial1.println(buffer[11]);
  Serial1.println(buffer[12]);

  s16  c_index = cal_index( buffer );
   if ( c_index != 0 && c_index != 1 ) {
     Serial1.printf("error cal index:%d\n", c_index);
      return false;
  }

  si114x_cal.vispd_correction = vispd_correction(buffer);
  si114x_cal.irpd_correction  = irpd_correction(buffer);
  si114x_cal.adcrange_ratio   = adcrange_ratio(buffer);
  si114x_cal.irsize_ratio     = irsize_ratio(buffer);
  si114x_cal.ledi_ratio       = ledi_ratio(buffer);
  si114x_cal.ucoef_p          = calref[c_index].ucoef;

  return true;
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

float Adafruit_SI1145::calcGain(uint16_t gain) {
  return (1 << (gain & 0x0007)) / ((gain & 0xFF00)?14.5:1.);
}

uint16_t Adafruit_SI1145::decGain(uint16_t g) {
  if(g == 0x8000)
    return 0xFFFF;

  g--;
  if (g < 0x0003) {
    g = 0x8007;
  }
  return g;
}

uint16_t Adafruit_SI1145::incGain(uint16_t g) {
  g++;
  if ((g & 0xFF) > 0x07){
    if (g & 0xFF00) {
      g = 0x0003; // low + x8
    } else {
       g = 0xFFFF;
    }
  }
  return g;
}

// calc gain from signal. signal must be measured in gain 0x8000
uint16_t Adafruit_SI1145::calcOptimalGainFromSignal(int signal) {
  uint16_t gain = 0x8000; // high + x1

  if(signal < 0) signal = 0;

  uint16_t g = gain;
  while(true) {
    // 60% from maximal 32767 readings
    float margin = (32767. * 0.6) / ((float)(1 << (g & 0x0007)) * ((g & 0xFF00)?1.:14.5));
    //DEBUG_PRINTLN(SF("gain=0x") + String(g, HEX) + SF(" margin=") + String(margin));
    if(signal < margin)
      gain = g;

    // iterator
    g = incGain(g);
    if (g == 0xFFFF)
      break;
  }

  return gain;
}

uint16_t Adafruit_SI1145::readVisibleGain() {
  return readParam(SI1145_PARAM_ALSVISADCGAIN) + readParam(SI1145_PARAM_ALSVISADCMISC)?0x8000:0x0000;
}

 // adjust the visible gain
void Adafruit_SI1145::setVisibleGain(uint16_t gain) {
  writeParam(SI1145_PARAM_ALSVISADCGAIN, (gain & 0x07));
  writeParam(SI1145_PARAM_ALSVISADCMISC, (gain & 0xFF00)?SI1145_PARAM_ALSVISADCMISC_VISRANGE_HIGH:SI1145_PARAM_ALSVISADCMISC_VISRANGE_LOW);
}

 // returns the IR gain
uint16_t Adafruit_SI1145::readIRGain() {
  return readParam(SI1145_PARAM_ALSIRADCGAIN) + readParam(SI1145_PARAM_ALSIRADCMISC)?0x8000:0x0000;
}

 // adjust the IR gain
void Adafruit_SI1145::setIRGain(uint16_t gain) {
 writeParam(SI1145_PARAM_ALSIRADCGAIN, (gain & 0x07));
 writeParam(SI1145_PARAM_ALSIRADCMISC, (gain & 0xFF00)?SI1145_PARAM_ALSIRADCMISC_RANGE_HIGH:SI1145_PARAM_ALSIRADCMISC_RANGE_LOW);
}

uint16_t Adafruit_SI1145::getADCOffset() const {
  return ADCOffset;
}

SI114X_CAL_S *Adafruit_SI1145::getCalibrationParameters() {
  return &si114x_cal;
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
    if (tmr + 40 < millis())
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

uint8_t Adafruit_SI1145::setMeassureChannels(uint8_t channels) {
  _lastError = 0;
  writeParam(SI1145_PARAM_CHLIST, channels);
  return _lastError;
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

uint8_t Adafruit_SI1145::readBytes(uint8_t a, uint8_t size, uint8_t buffer[]) {
  Wire.beginTransmission(_addr); // start transmission to device
  Wire.write(a); // sends register address to read from
  uint8_t res = Wire.endTransmission(); // end transmission
  // check transmission error
  if (res) {
    _lastError = res;
    return _lastError;
  }

  res = Wire.requestFrom(_addr, size);// send data n-bytes read
  if (res != size) {
    _lastError = 100;
    return _lastError;
  }

  for(int i = 0; i < size; i++)
    buffer[i] = Wire.read(); // receive DATA

  return 0;
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
