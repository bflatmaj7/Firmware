/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file can.cpp
 *
 * Base class for devices attached via the CAN bus.
 *
 */

#include "CAN.hpp"

namespace device
{

CAN::CAN(const char *name, const char *devname) :
	CDev(name, devname)
{
	DEVICE_DEBUG("CAN::CAN name = %s devname = %s", name, devname);
	// fill in _device_id fields for a I2C device
	_device_id.devid_s.bus_type = DeviceBusType_CAN;
	// devtype needs to be filled in by the driver
	_device_id.devid_s.devtype = 0;
}

CAN::~CAN()
{
	if (_dev) {
//		px4_canbus_uninitialize(_dev);
		_dev = nullptr;
	}
}


int
CAN::init()
{
	int ret = PX4_ERROR;
//	unsigned bus_index;

	// attach to the i2c bus
//	_dev = px4_canbus_initialize(get_device_bus());

	if (_dev == nullptr) {
		DEVICE_DEBUG("failed to init CAN");
		ret = -ENOENT;
		goto out;
	}


	// call the probe function to check whether the device is present
	ret = probe();

	if (ret != OK) {
		DEVICE_DEBUG("probe failed");
		goto out;
	}

	// do base class init, which will create device node, etc
	ret = CDev::init();

	if (ret != OK) {
		DEVICE_DEBUG("cdev init failed");
		goto out;
	}

//	// tell the world where we are
//	DEVICE_LOG("on CAN bus %d at 0x%02x (bus: %u KHz, max: %u KHz)",
//		   get_device_bus(), get_device_address(), _bus_clocks[bus_index] / 1000, _frequency / 1000);

out:

	if ((ret != OK) && (_dev != nullptr)) {
//		px4_canbus_uninitialize(_dev);
		_dev = nullptr;
	}

	return ret;
}


} // namespace device
