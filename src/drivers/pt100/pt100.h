/****************************************************************************
 *
 *   Copyright (C) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file pt100.h
 *
 * Shared defines for the pt100 driver.
 */
#pragma once

#define MAX31856_CONFIG_REG            0x00
#define MAX31856_CONFIG_BIAS           0x80
#define MAX31856_CONFIG_MODEAUTO       0x40
#define MAX31856_CONFIG_MODEOFF        0x00
#define MAX31856_CONFIG_1SHOT          0x20
#define MAX31856_CONFIG_3WIRE          0x10
#define MAX31856_CONFIG_24WIRE         0x00
#define MAX31856_CONFIG_FAULTSTAT      0x02
#define MAX31856_CONFIG_FILT50HZ       0x01
#define MAX31856_CONFIG_FILT60HZ       0x00

#define MAX31856_RTDMSB_REG           0x01
#define MAX31856_RTDLSB_REG           0x02
#define MAX31856_HFAULTMSB_REG        0x03
#define MAX31856_HFAULTLSB_REG        0x04
#define MAX31856_LFAULTMSB_REG        0x05
#define MAX31856_LFAULTLSB_REG        0x06
#define MAX31856_FAULTSTAT_REG        0x07

#define MAX31865_FAULT_HIGHTHRESH     0x80
#define MAX31865_FAULT_LOWTHRESH      0x40
#define MAX31865_FAULT_REFINLOW       0x20
#define MAX31865_FAULT_REFINHIGH      0x10
#define MAX31865_FAULT_RTDINLOW       0x08
#define MAX31865_FAULT_OVUV           0x04


#define RTD_A 3.9083e-3
#define RTD_B -5.775e-7

typedef enum max31865_numwires {
  MAX31865_2WIRE = 0,
  MAX31865_3WIRE = 1,
  MAX31865_4WIRE = 0
} max31865_numwires_t;

namespace pt100
{
#pragma pack(push,1)
struct reg_s {
	uint8_t f;
}; //calibration data

struct data_s {
	uint8_t t_msb;
	uint8_t t_lsb;
}; // data
#pragma pack(pop)

class IPT100
{
public:

	virtual ~IPT100() = default;

	virtual int init() = 0;
	virtual uint8_t readFault(void) = 0;
	virtual void clearFault(void) = 0;
	virtual void enableBias(bool b) = 0;
	virtual void autoConvert(bool b) = 0;
	virtual void setWires(max31865_numwires_t wires ) = 0;
	virtual float  temperature(float RTDnominal, float refResistor) = 0;
	virtual uint16_t readRTD (void) = 0;
	virtual uint8_t readRegister8(uint8_t addr) = 0;
	virtual uint16_t readRegister16(uint8_t addr) = 0;
	virtual int writeRegister8(uint8_t addr, uint8_t data) = 0;
};


} /* namespace */


/* interface factories */
extern pt100::IPT100 *pt100_spi_interface(uint8_t busnum, uint8_t device);
typedef pt100::IPT100 *(*PT100_constructor)(uint8_t, uint8_t);
