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
 * @file max31865.h
 *
 * Shared defines for the MAX31865 driver.
 */
#pragma once

#include <drivers/device/ringbuffer.h>
#include <drivers/device/spi.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_meteo.h>
#include <drivers/device/integrator.h>
#include <perf/perf_counter.h>

typedef enum max31865_numwires {
  MAX31865_2WIRE = 0,
  MAX31865_3WIRE = 1,
  MAX31865_4WIRE = 0
} max31865_numwires_t;

class MAX31865 : public device::SPI
{
public:
	MAX31865(int bus, const char *path, uint32_t device);
	virtual ~MAX31865();

	virtual int		init();

	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	void			print_info();

protected:
	virtual int		probe();

private:
	uint16_t			_product{0};	/** product code */

	struct hrt_call		_call {};
	unsigned			_call_interval{0};


	unsigned			_report_ticks; // 0 - no cycling, otherwise period of sending a report
	unsigned			_max_mesure_ticks; //ticks needed to measure

	ringbuffer::RingBuffer	*_reports;

	bool			_collect_phase;

	orb_advert_t		_meteo_topic;
	int					_orb_class_instance;
	int					_class_instance;

	perf_counter_t		_sample_perf;
	perf_counter_t		_bad_transfers;


	perf_counter_t		_controller_latency_perf;


	/**
	 * Start automatic measurement.
	 */
	void		start();

	/**
	 * Stop automatic measurement.
	 */
	void		stop();

	/**
	 * Reset chip.
	 *
	 * Resets the chip and measurements ranges, but not scale and offset.
	 */
	int			reset();

	/**
	 * Static trampoline from the hrt_call context; because we don't have a
	 * generic hrt wrapper yet.
	 *
	 * Called by the HRT in interrupt context at the specified rate if
	 * automatic polling is enabled.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void		measure_trampoline(void *arg);

	/**
	 * Fetch measurements from the sensor and update the report buffers.
	 */
	int			measure();

	int 		collect();

	uint8_t		read_reg8(uint8_t reg);
	uint16_t		read_reg16(uint8_t reg);
	void			write_reg(uint8_t reg, uint8_t val);
	uint8_t 		readFault(void);
	void 			clearFault(void);
	void 			enableBias(bool b);
	void 			autoConvert(bool b);
	void 			setWires(max31865_numwires_t wires );
	float  			temperature(float RTDnominal, float refResistor);
	uint16_t 		readRTD (void);

	/**
	 * Measurement self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	int 			self_test();


	MAX31865(const MAX31865 &);
	MAX31865 operator=(const MAX31865 &);
};


