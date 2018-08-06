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
CAN::init(const char *port)
{
	int ret = PX4_ERROR;

	/* Check if we have already initialized */

	if (!initialized) {
		/* Call stm32_caninitialize() to get an instance of the CAN interface */

		_dev = px4_caninitialize(1);
		px4_arch_configgpio(GPIO_CAN1_RX);
		px4_arch_configgpio(GPIO_CAN1_TX);
		px4_arch_configgpio(GPIO_CAN2_RX | GPIO_PULLUP);
		px4_arch_configgpio(GPIO_CAN2_TX);

		if (_dev == NULL) {
			DEVICE_DEBUG("can init failed");
			goto out;
		}

		/* Register the CAN driver at "/dev/can0" */

		ret = can_register(port, _dev);

		if (ret < 0) {
			DEVICE_DEBUG("can init failed");
			goto out;
		}

		/* Now we are initialized */

		initialized = true;
	}

	return OK;
	if (ret != OK) {
		DEVICE_DEBUG("can init failed");
		goto out;
	}

out:

	if ((ret != OK) && (_dev != nullptr)) {
		_dev = nullptr;
	}

	return ret;
}

int
CAN::receive(int fd, FAR uint8_t *data){
	int ret = PX4_OK;
//	ret = can_read(fd,data,255);
	return ret;
}


int
CAN::transmit(int fd, char * buf,size_t len){
//	int nbytes;
//	size_t msgsize;

//	struct px4_can_msg_t txmsg;
//
//    txmsg.cm_hdr.ch_id     = 99;
//    txmsg.cm_hdr.ch_rtr    = false;
//    txmsg.cm_hdr.ch_dlc    = len;
//
//    for (unsigned i = 0; i < len; i++)
//	{
//	  txmsg.cm_data[i] = buf[i];
//	}

	/* Send the TX message */

//	msgsize = CAN_MSGLEN(len);
//	nbytes = write(fd, buf,len);
//
//	if (nbytes<=0){
//		return PX4_ERROR;
//	}

	return OK;
}


} // namespace device
