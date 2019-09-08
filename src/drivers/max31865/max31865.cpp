/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_getopt.h>
#include <px4_workqueue.h>

#include <drivers/device/spi.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <vector>

#include <perf/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_pt100.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/pt100.h>
#include <uORB/topics/debug_key_value.h>

#include <board_config.h>

#define MAX31865_BUS 		PX4_SPI_BUS_EXTERNAL1
#define MAX31865_DEVICE_PATH	"/dev/max31865"

#define DIR_READ				0x00
#define DIR_WRITE				0x80

//  MAX31865 registers

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


typedef enum max31865_numwires {
  MAX31865_2WIRE = 0,
  MAX31865_3WIRE = 1,
  MAX31865_4WIRE = 0
} max31865_numwires_t;

#define RTD_A 3.9083e-3
#define RTD_B -5.775e-7

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif


class MAX31865 : public device::SPI
{
public:
	MAX31865(int bus, const char *path, uint32_t device);
	virtual ~MAX31865();

	virtual int 		init();

	virtual ssize_t		read(device::file_t *filp, char *buffer, size_t buflen);
	virtual int			ioctl(device::file_t *filp, int cmd, unsigned long arg);

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void				print_info();

protected:
	virtual int			probe();

private:
	float				_temperature;
	int                 _conversion_interval;
	work_s				_work;
	ringbuffer::RingBuffer  *_reports;
	bool				_sensor_ok;
	int				_measure_ticks;
	int				_class_instance;
	int				_orb_class_instance;


	orb_advert_t		_pt100_topic;

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;


	uint8_t 			readFault(void);
	void	 			clearFault(void);
	void				enableBias(bool b);
	void				autoConvert(bool b);
	void				setWires(max31865_numwires_t wires );
	float				convert_temp(uint16_t rtd);
	uint8_t				read_reg8(uint8_t reg);
	uint16_t			read_reg16(uint8_t reg);
	int					write_reg(uint8_t reg, uint8_t val);

	/**
	* Initialise the automatic measurement state machine and start it.
	*
	* @note This function is called at open and error time.  It might make sense
	*       to make it more aggressive about resetting the bus in case of errors.
	*/
	void				start();

	/**
	* Stop the automatic measurement state machine.
	*/
	void				stop();


	/**
	* Perform a poll cycle; collect from the previous measurement
	* and start a new one.
	*/
	void				cycle();
	int					measure();
	int					collect();

	/**
	* Static trampoline from the workq context; because we don't have a
	* generic workq wrapper yet.
	*
	* @param arg		Instance pointer for the driver that is polling.
	*/
	static void			cycle_trampoline(void *arg);


};

extern "C" { __EXPORT int max31865_main(int argc, char *argv[]); }

MAX31865::MAX31865(int bus, const char *path, uint32_t device) :
			SPI("MAX31865", path, bus, device, SPIDEV_MODE1, 500 * 1000),
	_temperature(-1.0f),
	_conversion_interval(-1),
	_reports(nullptr),
	_sensor_ok(false),
	_measure_ticks(0),
	_class_instance(-1),
	_orb_class_instance(-1),
	_pt100_topic(nullptr),
	_sample_perf(perf_alloc(PC_ELAPSED, "max31865_read")),
	_comms_errors(perf_alloc(PC_COUNT, "max31865_com_err"))
{
	/* enable debug() calls */
	_debug_enabled = false;

	/* work_cancel in the dtor will explode if we don't do this... */
	memset(&_work, 0, sizeof(_work));
}

MAX31865::~MAX31865()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	if (_pt100_topic != nullptr) {
		orb_unadvertise(_pt100_topic);
	}

	if (_class_instance != -1) {
		unregister_class_devname(PT100_BASE_DEVICE_PATH, _class_instance);
	}

	/* free perf counters */
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
MAX31865::init()
{
	int ret = PX4_ERROR;

	_temperature = 0.0f;
	_conversion_interval = 100000;

	/* do SPI init (and probe) first */
	if (SPI::init() != OK) {
		return ret;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(pt100_s));


	if (_reports == nullptr) {
		return ret;
	}

	_class_instance = register_class_devname(PT100_BASE_DEVICE_PATH);

	/* get a publish handle on the range finder topic */
	struct pt100_s ds_report = {};

	_pt100_topic = orb_advertise_multi(ORB_ID(pt100), &ds_report,
			 &_orb_class_instance, ORB_PRIO_HIGH);

	if (_pt100_topic == nullptr) {
		DEVICE_LOG("failed to create pt100 object. Did you start uOrb?");
	}

	setWires(MAX31865_4WIRE);
	enableBias(false);
	autoConvert(false);
	clearFault();
	// Select altitude register
	int ret2 = measure();

	if (ret2 == 0) {
		ret = OK;
		_sensor_ok = true;
	}

	return ret;
}


int
MAX31865::probe()
{
	return measure();
}

int
MAX31865::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_measure_ticks = 0;
				return OK;

			/* external signalling (DRDY) not supported */
			case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_ticks = USEC2TICK(_conversion_interval);

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();

					}

					return OK;
				}

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* convert hz to tick interval via microseconds */
					int ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if (ticks < USEC2TICK(_conversion_interval)) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_measure_ticks = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_measure_ticks == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}

		return (1000 / _measure_ticks);

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			ATOMIC_ENTER;

			if (!_reports->resize(arg)) {
				ATOMIC_LEAVE;
				return -ENOMEM;
			}

			ATOMIC_LEAVE;

			return OK;
		}

	case SENSORIOCRESET:
		/* XXX implement this */
		return -EINVAL;

	default:
		/* give it to the superclass */
		return SPI::ioctl(filp, cmd, arg);
	}
}

uint8_t MAX31865::readFault(void) {
  return read_reg8(MAX31856_FAULTSTAT_REG);
}

void MAX31865::clearFault(void) {
  uint8_t t = read_reg8(MAX31856_CONFIG_REG);
  t &= ~0x2C;
  t |= MAX31856_CONFIG_FAULTSTAT;
  write_reg(MAX31856_CONFIG_REG, t);
}

void MAX31865::enableBias(bool b) {
  uint8_t t = read_reg8(MAX31856_CONFIG_REG);
  if (b) {
    t |= MAX31856_CONFIG_BIAS;       // enable bias
  } else {
    t &= ~MAX31856_CONFIG_BIAS;       // disable bias
  }
  write_reg(MAX31856_CONFIG_REG, t);
}

void MAX31865::autoConvert(bool b) {
  uint8_t t = read_reg8(MAX31856_CONFIG_REG);
  if (b) {
    t |= MAX31856_CONFIG_MODEAUTO;       // enable autoconvert
  } else {
    t &= ~MAX31856_CONFIG_MODEAUTO;       // disable autoconvert
  }
  write_reg(MAX31856_CONFIG_REG, t);
}

void MAX31865::setWires(max31865_numwires_t wires ) {
  uint8_t t = read_reg8(MAX31856_CONFIG_REG);
  if (wires == MAX31865_3WIRE) {
    t |= MAX31856_CONFIG_3WIRE;
  } else {
    // 2 or 4 wire
    t &= ~MAX31856_CONFIG_3WIRE;
  }
  write_reg(MAX31856_CONFIG_REG, t);
}

float MAX31865::convert_temp(uint16_t rtd) {
  // http://www.analog.com/media/en/technical-documentation/application-notes/AN709_0.pdf

  float Z1, Z2, Z3, Z4, Rt, temp;
  float RTDnominal = 100;
  float refResistor = 430; // Adafruit 470;// MikroOE

  Rt = (float)rtd;
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

uint8_t
MAX31865::read_reg8(uint8_t reg)
{
	uint8_t cmd[2] = { (uint8_t)(reg | DIR_READ), 0}; //set MSB bit
	transfer(&cmd[0], &cmd[0], 2);

	return cmd[1];
}

uint16_t
MAX31865::read_reg16(uint8_t reg)
{
	uint8_t cmd[2] = { reg, 0 };
	cmd[0] = (uint8_t)(reg | DIR_READ);
	transfer(cmd, cmd, sizeof(cmd));

	// return cmd[0] + cmd[1]
	return (cmd[0] << 8) + cmd[1];
}

int
MAX31865::write_reg(uint8_t reg, uint8_t val)
{
	int ret;
	uint8_t cmd[2];
	cmd[0] = (uint8_t)(reg | DIR_WRITE);
	cmd[1] = val;
	ret = transfer(cmd, cmd, sizeof(cmd));
	return ret;
}

ssize_t
MAX31865::read(device::file_t *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct pt100_s);
	struct pt100_s *rbuf = reinterpret_cast<struct pt100_s *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_measure_ticks > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_reports->get(rbuf)) {
				ret += sizeof(*rbuf);
				rbuf++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */
	do {
		_reports->flush();

		/* trigger a measurement */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		/* wait for it to complete */
		usleep(_conversion_interval);

		/* run the collection phase */
		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* state machine will have generated a report, copy it out */
		if (_reports->get(rbuf)) {
			ret = sizeof(*rbuf);
		}

	} while (0);

	return ret;
}

int
MAX31865::measure()
{
	int ret;

	clearFault();
	enableBias(true);
	usleep(10000);
	uint8_t t = read_reg8(MAX31856_CONFIG_REG);
	t |= MAX31856_CONFIG_1SHOT;
	ret = write_reg(MAX31856_CONFIG_REG, t);

	if (OK != ret) {
		PX4_WARN("spi measure failed");
		perf_count(_comms_errors);
		DEVICE_DEBUG("spi::transfer returned %d", ret);
		return ret;
	}

	ret = OK;

	return ret;
}

int
MAX31865::collect()
{
	int	ret = -EIO;

	/* read from the sensor */
	perf_begin(_sample_perf);

//	uint8_t cmd[2] = { MAX31856_RTDMSB_REG, 0};
//	cmd[0] = (uint8_t)(MAX31856_RTDMSB_REG | DIR_READ);;
//	ret = transfer(cmd, nullptr, 1);
//	ret = transfer(nullptr, cmd, sizeof(cmd));
	uint8_t cmd[3] = { (uint8_t)(MAX31856_RTDMSB_REG | DIR_READ), 0,0}; //set MSB bit
	ret = transfer(&cmd[0], &cmd[0], 3);

//	ret = read_reg8(MAX31856_CONFIG_REG);

	if (ret < 0) {
		PX4_WARN("error reading from sensor: %d", ret);
		DEVICE_DEBUG("error reading from sensor: %d", ret);
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

    uint16_t rtd = (cmd[1] << 8) + cmd[2];
    rtd >>= 1;

    float temperature = convert_temp(rtd);

	struct pt100_s report;
	report.timestamp = hrt_absolute_time();
	report.pt100 = temperature;
	/* TODO: set proper ID */
	//report.id = 333;

	/* publish it, if we are the primary */
	if (_pt100_topic != nullptr) {
		orb_publish(ORB_ID(pt100), _pt100_topic, &report);
//		PX4_WARN("pt100 published");
	}

	_reports->force(&report);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	ret = OK;


	perf_end(_sample_perf);
	return ret;
}

void
MAX31865::start()
{
	/* reset the report ring and state machine */
	_reports->flush();

	/* set register to '0' */
	measure();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&MAX31865::cycle_trampoline, this, USEC2TICK(_conversion_interval));
}

void
MAX31865::stop()
{
	work_cancel(HPWORK, &_work);
}

void
MAX31865::cycle_trampoline(void *arg)
{
	MAX31865 *dev = (MAX31865 *)arg;

	dev->cycle();
}

void
MAX31865::cycle()
{

	/* trigger a measurement */
	if (OK != measure()) {
		DEVICE_DEBUG("measure error");
		return;
	}

	/* wait for it to complete */
	usleep(65000);

	/* Collect results */
	if (OK != collect()) {
		DEVICE_DEBUG("collection error");
		/* if error restart the measurement state machine */
		start();
		return;
	}

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(HPWORK,
		   &_work,
		   (worker_t)&MAX31865::cycle_trampoline,
		   this,
		   USEC2TICK(_conversion_interval));

}

void
MAX31865::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	_reports->print_info("report queue");
}

/**
 * Local functions in support of the shell command.
 */
namespace max31865
{

MAX31865	*g_dev;

void	start();
void	stop();
void	test();
void	reset();
void	info();

/**
 * Start the driver.
 */
void
start()
{
	int fd = -1;

	if (g_dev != nullptr) {
		errx(1, "already started");
	}

	/* create the driver */
	g_dev = new MAX31865(PX4_SPI_BUS_EXTERNAL1, MAX31865_DEVICE_PATH, PX4_SPIDEV_EXTERNAL1_1);

	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = open(MAX31865_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		::close(fd);
		goto fail;
	}

	::close(fd);
	exit(0);

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Stop the driver
 */
void stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		errx(1, "driver not running");
	}

	exit(0);
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{
	struct pt100_s report;
	ssize_t sz;
	int ret;

	int fd = open(MAX31865_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'hyt271 start' if the driver is not running", MAX31865_DEVICE_PATH);
	}

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		err(1, "immediate read failed");
	}

	print_message(report);

	/* start the sensor polling at 2Hz */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
		errx(1, "failed to set 2Hz poll rate");
	}

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != 1) {
			errx(1, "timed out waiting for sensor data");
		}

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			err(1, "periodic read failed");
		}

		print_message(report);
	}

	/* reset the sensor polling to default rate */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT)) {
		errx(1, "failed to set default poll rate");
	}

	::close(fd);
	errx(0, "PASS");
}
/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(MAX31865_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "failed ");
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		err(1, "driver reset failed");
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "driver poll restart failed");
	}

	::close(fd);
	exit(0);
}


/**
 * Print a little info about the driver.
 */
void
info()
{
	if (g_dev == nullptr) {
		errx(1, "driver not running");
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

} /* namespace */


int
max31865_main(int argc, char *argv[])
{
	// check for optional arguments
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;


	while ((ch = px4_getopt(argc, argv, "R:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		default:
			PX4_WARN("Unknown option!");
		}
	}

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[myoptind], "start")) {
		max31865::start();
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[myoptind], "stop")) {
		max31865::stop();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[myoptind], "test")) {
		max31865::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[myoptind], "reset")) {
		max31865::reset();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[myoptind], "info") || !strcmp(argv[myoptind], "status")) {
		max31865::info();
	}

	PX4_ERR("unrecognized command, try 'start', 'test', 'reset' or 'info'");
	return PX4_ERROR;
}
