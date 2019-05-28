/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file pt100_spi.cpp
 *
 * SPI interface for PT100 (MAX31865)
 */

#include <px4_config.h>

#include "pt100.h"
#include <drivers/device/spi.h>

#include "board_config.h"

/* SPI protocol address bits */
#define DIR_READ			(1<<7)  //for set
#define DIR_WRITE			~(1<<7) //for clear


#pragma pack(push,1)
struct spi_data_s {
	uint8_t addr;
	struct pt100::data_s data;
};

struct spi_reg_s {
	uint8_t addr;
	struct pt100::reg_s data;
};
#pragma pack(pop)

class PT100_SPI: public device::SPI, public pt100::IPT100
{
public:
	PT100_SPI(uint8_t bus, uint32_t device);
	~PT100_SPI();

	int init();

	uint8_t readFault(void);
	void clearFault(void);
	void enableBias(bool b);
	void autoConvert(bool b);
	void setWires(max31865_numwires_t wires );
	float  temperature(float RTDnominal, float refResistor);
	uint16_t readRTD (void);
	uint8_t readRegister8(uint8_t addr);
	uint16_t readRegister16(uint8_t addr);
	int writeRegister8(uint8_t addr, uint8_t data);

private:
	spi_reg_s _reg;
	spi_data_s _data;
};

pt100::IPT100 *pt100_spi_interface(uint8_t busnum, uint8_t device)
{
	return new PT100_SPI(busnum, device);
}

PT100_SPI::PT100_SPI(uint8_t bus, uint32_t device) :
	SPI("PT100_SPI", nullptr, PX4_SPI_BUS_EXTERNAL1, PX4_SPIDEV_EXTERNAL1_1, SPIDEV_MODE3, 500 * 1000)
{
}

PT100_SPI::~PT100_SPI()
{

}

int PT100_SPI::init()
{
	return SPI::init();
};

uint8_t PT100_SPI::readFault(void) {
  return readRegister8(MAX31856_FAULTSTAT_REG);
}

void PT100_SPI::clearFault(void) {
  uint8_t t = readRegister8(MAX31856_CONFIG_REG);
  t &= ~0x2C;
  t |= MAX31856_CONFIG_FAULTSTAT;
  writeRegister8(MAX31856_CONFIG_REG, t);
}

void PT100_SPI::enableBias(bool b) {
  uint8_t t = readRegister8(MAX31856_CONFIG_REG);
  if (b) {
    t |= MAX31856_CONFIG_BIAS;       // enable bias
  } else {
    t &= ~MAX31856_CONFIG_BIAS;       // disable bias
  }
  writeRegister8(MAX31856_CONFIG_REG, t);
}

void PT100_SPI::autoConvert(bool b) {
  uint8_t t = readRegister8(MAX31856_CONFIG_REG);
  if (b) {
    t |= MAX31856_CONFIG_MODEAUTO;       // enable autoconvert
  } else {
    t &= ~MAX31856_CONFIG_MODEAUTO;       // disable autoconvert
  }
  writeRegister8(MAX31856_CONFIG_REG, t);
}

void PT100_SPI::setWires(max31865_numwires_t wires ) {
  uint8_t t = readRegister8(MAX31856_CONFIG_REG);
  if (wires == MAX31865_3WIRE) {
    t |= MAX31856_CONFIG_3WIRE;
  } else {
    // 2 or 4 wire
    t &= ~MAX31856_CONFIG_3WIRE;
  }
  writeRegister8(MAX31856_CONFIG_REG, t);
}

float  PT100_SPI::temperature(float RTDnominal, float refResistor) {
  // http://www.analog.com/media/en/technical-documentation/application-notes/AN709_0.pdf

  float Z1, Z2, Z3, Z4, Rt, temp;

  Rt = readRTD();
  Rt /= 32768;
  Rt *= refResistor;

  // Serial.print("\nResistance: "); Serial.println(Rt, 8);

  Z1 = -RTD_A;
  Z2 = RTD_A * RTD_A - (4 * RTD_B);
  Z3 = (4 * (float)RTD_B) / RTDnominal;
  Z4 = 2 * RTD_B;

  temp = Z2 + (Z3 * Rt);
  temp = ((float)sqrt(temp) + Z1) / Z4;

  if (temp >= 0) return temp;

  // ugh.
  Rt /= RTDnominal;
  Rt *= 100;      // normalize to 100 ohm

  float rpoly = Rt;

  temp = -242.02;
  temp += (float)2.2228 * rpoly;
  rpoly *= Rt;  // square
  temp += (float)2.5859e-3 * rpoly;
  rpoly *= Rt;  // ^3
  temp -= (float)4.8260e-6 * rpoly;
  rpoly *= Rt;  // ^4
  temp -= (float)2.8183e-8 * rpoly;
  rpoly *= Rt;  // ^5
  temp += (float)1.5243e-10 * rpoly;

  return temp;
}

uint16_t PT100_SPI::readRTD (void) {
  clearFault();
  enableBias(true);
  usleep(10000);
  uint8_t t = readRegister8(MAX31856_CONFIG_REG);
  t |= MAX31856_CONFIG_1SHOT;
  writeRegister8(MAX31856_CONFIG_REG, t);
  usleep(65000);

  uint16_t rtd = readRegister16(MAX31856_RTDMSB_REG);

  // remove fault
  rtd >>= 1;

  return rtd;
}

/**********************************************/
uint8_t PT100_SPI::readRegister8(uint8_t addr) {
	_reg.addr = (uint8_t)(addr | DIR_READ); //set MSB bit

	if (transfer((uint8_t *)&_reg, (uint8_t *)&_reg, 2) == OK) {
		return _reg.data.f;

	} else {
		return 0;
	}

}

uint16_t PT100_SPI::readRegister16(uint8_t addr) {

	_data.addr = (uint8_t)(addr | DIR_READ); //set MSB bit

	if (transfer((uint8_t *)&_data, (uint8_t *)&_data, 3) == OK) {
		return _data.data.t_lsb;

	} else {
		return 0;
	}
}


int PT100_SPI::writeRegister8(uint8_t addr, uint8_t data) {
	uint8_t cmd[2] = { (uint8_t)(addr & DIR_WRITE), data}; //clear MSB bit
	return transfer(&cmd[0], nullptr, 2);
}
