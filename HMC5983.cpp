/*
 * HMC5983.cpp - library header
 *
 * simple library for the HMC5983 sensor from Honeywell
 * adaptation form MBED from the Arduino library
 *
 * (c) 2014 Korneliusz Jarzebski, www.jarzebski.pl
 * (c) 2014 David Cuartielles, Arduino LLC
 * (c) 2016 Abel Romero, www.abelromero.com
 * (c) 2018 Arcadie Cracan
 */

// MISSING: EARTH DECLINATION ANGLE
// In other words, we are not making any compensation for the earth's north pole location vs the magnetic measurement

#include "HMC5983.h"

/*
HMC5983::HMC5983(PinName sda, PinName scl) : i2c_(*reinterpret_cast<I2C*>(i2cRaw))
{
    // Placement new to avoid additional heap memory allocation.
    new(i2cRaw) I2C(sda, scl);

    init();
}

HMC5983::HMC5983(I2C &i2c): i2c_(i2c)
{
    init();
}

void HMC5983::Config(TwoWire *i2c_);
*/

//
HMC5983::HMC5983( TwoWire * wire)
{
  i2c_ = wire;
  m_Scale = 1;
}

bool HMC5983::Begin()
{
    const float initial_m[9] = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    
    i2c_ -> begin();
    i2c_ -> setClock(400000);
    const float initial_b[3] = {0.0f, 0.0f, 0.0f};
    if ((fastRegister8(HMC5983_REG_IDENT_A) != 0x48)
            || (fastRegister8(HMC5983_REG_IDENT_B) != 0x34)
            || (fastRegister8(HMC5983_REG_IDENT_C) != 0x33)) {
    Serial.printf("Begin: %x, %x, %x\n", fastRegister8(HMC5983_REG_IDENT_A), fastRegister8(HMC5983_REG_IDENT_B), fastRegister8(HMC5983_REG_IDENT_C));
        return false;
    }

    // Set Gain Range
    setRange(HMC5983_RANGE_8_1GA);
    // Set DataRate 220Hz ~4.5ms
    setDataRate(HMC5983_DATARATE_220HZ);
    // Set number of samples to average
    setSampleAverages(HMC5983_SAMPLEAVERAGE_2);
    // Set Mode
    setMeasurementMode(HMC5983_CONTINOUS);
    setCalibrationMatrices(initial_m, initial_b);
    H[0] = 0;
    H[1] = 0;
    H[2] = 0;
    // Setup DRDY int
//  if (ISR_callback != NULL) {
//    pinMode(3, INPUT_PULLUP);
//    attachInterrupt(digitalPinToInterrupt(3), ISR_callback, FALLING);
//  }
    return true;
}

uint8_t HMC5983::getStatus() {
  
  uint8_t drdy = 0;
  uint8_t data[1];
  read(HMC5983_ADDRESS, HMC5983_STATUS, data, 1);
  drdy = data[0] << 6;
  return drdy;
  
}


/*
From datasheet for the HMC5983
Below is an example of a (power-on) initialization process for “continuous-measurement mode” via I2C interface:
1. Write CRA (00) – send 0x3C 0x00 0x70 (8-average, 15 Hz default or any other rate, normal measurement)
2. Write CRB (01) – send 0x3C 0x01 0xA0 (Gain=5, or any other desired gain)
3. For each measurement query:
  Write Mode (02) – send 0x3C 0x02 0x01 (Single-measurement mode)
  Wait 6 ms or monitor status register or DRDY hardware interrupt pin
  Send 0x3D 0x06 (Read all 6 bytes. If gain is changed then this data set is using previous gain)
  Convert three 16-bit 2’s compliment hex values to decimal values and assign to X, Z, Y, respectively.
(Self addition:)
4. Convert the magnetic information into a compass value
REGARDING THE CALCULATION OF THE ACTUAL HEADING VALUE
From AN-203 http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
The magnetic compass heading can be determined (in degrees) from the magnetometer's x and y readings by using the
following set of equations:
  Direction (y>0) = 90 - [arcTAN(x/y)]*180/PI
  Direction (y<0) = 270 - [arcTAN(x/y)]*180/PI
  Direction (y=0, x<0) = 180.0
  Direction (y=0, x>0) = 0.0
*/

void HMC5983::setRange(hmc5983_range_t range)
{
	if(range == HMC5983_RANGE_0_88GA)
	{
	m_Scale = 0.73;
	}
	else if(range == HMC5983_RANGE_1_3GA)
	{
		m_Scale = 0.92;
	}
	else if(range == HMC5983_RANGE_1_9GA)
	{
		m_Scale = 1.22;
	}
	else if(range == HMC5983_RANGE_2_5GA)
	{
		m_Scale = 1.52;
	}
	else if(range == HMC5983_RANGE_4GA)
	{
		m_Scale = 2.27;
	}
	else if(range == HMC5983_RANGE_4_7GA)
	{
		m_Scale = 2.56;
	}
	else if(range == HMC5983_RANGE_5_6GA)
	{
		m_Scale = 3.03;
	}
	else if(range == HMC5983_RANGE_8_1GA)
	{
		m_Scale = 4.35;
	}
	else
		m_Scale = 1;


    writeRegister8(HMC5983_REG_CONFIG_B, range << 5);
}

hmc5983_range_t HMC5983::getRange(void)
{
    return (hmc5983_range_t)((readRegister8(HMC5983_REG_CONFIG_B) >> 5));
}

void HMC5983::setMeasurementMode(hmc5983_mode_t mode)
{
    uint8_t value;

    value = readRegister8(HMC5983_REG_MODE);

    value &= 0b11111100;
    value |= mode;
    Serial.println(value, BIN);
    writeRegister8(HMC5983_REG_MODE, value);
}

hmc5983_mode_t HMC5983::getMeasurementMode(void)
{
    uint8_t value;

    value = readRegister8(HMC5983_REG_MODE);
    value &= 0b00000011;

    return (hmc5983_mode_t)value;
}

void HMC5983::setDataRate(hmc5983_dataRate_t dataRate)
{
    uint8_t value;

    value = readRegister8(HMC5983_REG_CONFIG_A);
    value &= 0b11100011;
    value |= (dataRate << 2);

    writeRegister8(HMC5983_REG_CONFIG_A, value);
}

hmc5983_dataRate_t HMC5983::getDataRate(void)
{
    uint8_t value;

    value = readRegister8(HMC5983_REG_CONFIG_A);
    value &= 0b00011100;
    value >>= 2;

    return (hmc5983_dataRate_t)value;
}

void HMC5983::setSampleAverages(hmc5983_sampleAverages_t sampleAverages)
{
    uint8_t value;

    value = readRegister8(HMC5983_REG_CONFIG_A);
    value &= 0b10011111;
    value |= (sampleAverages << 5);

    writeRegister8(HMC5983_REG_CONFIG_A, value);
}

hmc5983_sampleAverages_t HMC5983::getSampleAverages(void)
{
    uint8_t value;

    value = readRegister8(HMC5983_REG_CONFIG_A);
    value &= 0b01100000;
    value >>= 5;

    return (hmc5983_sampleAverages_t)value;
}

// Write byte to register
void HMC5983::writeRegister8(uint8_t reg, uint8_t value)
{
    //char cmd[2];

    //cmd[0] = reg;
    //cmd[1] = value;
    write(HMC5983_ADDRESS, reg, value);
}

// Read byte to register
uint8_t HMC5983::fastRegister8(uint8_t reg)
{
    uint8_t value;

    //write(HMC5983_ADDRESS, (char *)&reg, 1);
    //read(HMC5983_ADDRESS, (char *)&value, 1);
    read(HMC5983_ADDRESS, reg, &value, 1);
    return value;
}

// Read byte from register
uint8_t HMC5983::readRegister8(uint8_t reg)
{
    uint8_t value;

    //write(HMC5983_ADDRESS, (char *)&reg, 1);
    read(HMC5983_ADDRESS, reg, &value, 1);
    
    return value;

//  Wire.requestFrom(HMC5983_ADDRESS, 1);
//  while(!Wire.available()) {};
//  value = Wire.read();
}

// Read word from register
int16_t HMC5983::readRegister16(uint8_t reg)
{
    int16_t value;
    uint8_t resp[2];

    //write(HMC5983_ADDRESS, (char *)&reg, 1);
    read(HMC5983_ADDRESS, reg, resp, 2);

//  Wire.requestFrom(HMC5983_ADDRESS, 2);
//  while(!Wire.available()) {};

    value = (uint8_t)resp[0] << 8 | (uint8_t)resp[1];

    return value;
}

void HMC5983::getHeadingRegs()
{
    // the values for X, Y & Z must be read in X, Z & Y order.
    uint8_t data[6];
    //writeRegister8(HMC5983_OUT_X_MSB, 0); // Select MSB X register
    read(HMC5983_ADDRESS, HMC5983_OUT_X_MSB, data, 6);
    uint8_t X_MSB = data[0];
    uint8_t X_LSB = data[1];
    uint8_t Z_MSB = data[2];
    uint8_t Z_LSB = data[3];
    uint8_t Y_MSB = data[4];
    uint8_t Y_LSB = data[5];

    // compose byte for X, Y, Z's LSB & MSB 8bit registers
    H[0] = (uint16_t(X_MSB) << 8) | X_LSB;
    H[1] = (uint16_t(Y_MSB) << 8) | Y_LSB;
    H[2] = (uint16_t(Z_MSB) << 8) | Z_LSB;
}

void HMC5983::getMagRaw(int16_t *values) {
    // the values for X, Y & Z must be read in X, Z & Y order.
    uint8_t data[6];
    //writeRegister8(HMC5983_OUT_X_MSB, 0); // Select MSB X register
    read(HMC5983_ADDRESS, HMC5983_OUT_X_MSB, data, 6);
    uint8_t X_MSB = data[0];
    uint8_t X_LSB = data[1];
    uint8_t Z_MSB = data[2];
    uint8_t Z_LSB = data[3];
    uint8_t Y_MSB = data[4];
    uint8_t Y_LSB = data[5];

    // compose byte for X, Y, Z's LSB & MSB 8bit registers
    values[0] = (uint16_t(X_MSB) << 8) | X_LSB;
    values[1] = (uint16_t(Y_MSB) << 8) | Y_LSB;
    values[2] = (uint16_t(Z_MSB) << 8) | Z_LSB;
}  

void HMC5983::getMagScaled(float *values) {
    int16_t magRaw[3];
    
    getMagRaw(magRaw);
    
    values[0] = magRaw[0] * m_Scale * 0.1;  //convert to microTelsa from milliGauss
    values[1] = magRaw[1] * m_Scale * 0.1;
    values[2] = magRaw[2] * m_Scale * 0.1;
}  


void HMC5983::computeCalibratedHeadingVector(float *v)
{
    float Hub[3];

    int i, j;

    // remove bias
    for (i = 0; i < 3; i++) {
        Hub[i] = H[i] - _b[i];
        v[i] = 0.0f;
    }

    // apply matrix transformation
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            v[i] += _m[3*i+j]*Hub[j];
        }
    }
}

void HMC5983::setCalibrationMatrices(const float *m, const float *b)
{
    int i, j;

    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            _m[3*i+j] = m[3*i+j];
        }
        _b[i] = b[i];
    }
}

float HMC5983::readHeading(bool calibrate)
{
    // declare the heading variable we'll be returning
    float result = 0;
    float cH[3];
    int i;

    getHeadingRegs();
    for (i = 0; i < 3; i++) {
        cH[i] = H[i] * m_Scale;
    }

    if (calibrate) {
        computeCalibratedHeadingVector(cH);
    }
    // this is the correct way, fixed from original David's work.
    // corrected following datasheet and his own comments...xD
    // even corrected from datasheet, the 90-270 angle is a bit confusing, but the 360º are captured.
    if (cH[1] > 0.0f) result = 90.0f - (float)atan(cH[0] / cH[1]) * 180.0f / M_PI;
    if (cH[1] < 0.0f) result = 270.0f - (float)atan(cH[0] / cH[1]) * 180.0f / M_PI;
    if (cH[1] == 0.0f && cH[0] < 0.0f) result = 180.0f;
    if (cH[1] == 0.0f && cH[0] > 0.0f) result = 0.0f;

    return result;
}

void HMC5983::readHeadingVector(float *v, bool calibrate)
{
    getHeadingRegs();
    if (calibrate) {
        computeCalibratedHeadingVector(v);
    }
    else {
        v[0] = H[0];
        v[1] = H[1];
        v[2] = H[2];
    }
}


    
bool HMC5983::read(uint8_t dev_, uint8_t reg, uint8_t * data, uint8_t count) {
  i2c_->beginTransmission(dev_);
  i2c_->write(reg);
  i2c_->endTransmission(false);
  bytes_rx_ = i2c_->requestFrom(static_cast<uint8_t>(dev_), count);
  if (bytes_rx_ == count) {
    for (size_t i = 0; i < count; i++) {
      data[i] = i2c_->read();
    }
    return true;
  } else {
    return false;
  }
}

bool HMC5983::write(uint8_t dev_, uint8_t reg, uint8_t data)
{
  uint8_t ret_val;
  
  i2c_->beginTransmission(dev_);
  i2c_->write(reg);
  i2c_->write(data);
  i2c_->endTransmission();
    
  delay(10);
  read(dev_, reg, &ret_val, sizeof(ret_val));
  if (data == ret_val) {
    return true;
  } else {
    return false;
  }
}