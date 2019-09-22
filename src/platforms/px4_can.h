/****************************************************************************
 *
 *   Copyright (c) 2015 Mark Charlebois. All rights reserved.
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
 * @file px4_can.h
 *
 * Includes device headers depending on the build target
 */

#pragma once


#if defined(__PX4_ROS)

#error "Devices not supported in ROS"

#elif defined (__PX4_NUTTX)
__BEGIN_DECLS

/*
 * Building for NuttX
 */
#include <px4_config.h>
#include <sys/ioctl.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>
#include <nuttx/can/can.h>
#include <nuttx/irq.h>
#include <nuttx/wqueue.h>
#include <chip.h>
#include <arch/board/board.h>
#include <arch/chip/chip.h>
#include "up_internal.h"
#include "up_arch.h"

#define px4_can_msg_t can_msg_s
typedef struct can_dev_s px4_can_dev_t;
__END_DECLS

#elif defined(__PX4_POSIX)
#include <stdint.h>

// NOTE - This is a copy of the NuttX i2c_msg_s structure

//typedef struct {
//	uint32_t frequency;         /* I2C frequency */
//	uint16_t addr;              /* Slave address (7- or 10-bit) */
//	uint16_t flags;             /* See I2C_M_* definitions */
//	uint8_t *buffer;        /* Buffer to be transferred */
//	ssize_t length;             /* Length of the buffer in bytes */
//} px4_can_msg_t;

// NOTE - This is a copy of the NuttX i2c_ops_s structure
typedef struct {
	const struct px4_can_ops_t *ops; /* I2C vtable */
} px4_can_dev_t;


#else
#error "No target platform defined"
#endif

