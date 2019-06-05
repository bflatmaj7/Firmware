/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file hyt271.cpp
 *
 * @author Norman Wildmann <norman.wildmann@dlr.de>
 *
 * Driver for the HYT271 humidity and temperature sensor.
 * Default I2C address 0x28 is used.
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_getopt.h>
#include <px4_workqueue.h>

#include <drivers/device/i2c.h>

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
#include <drivers/drv_meteo.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/meteo.h>
#include <uORB/topics/mhp.h>
#include <uORB/topics/pt100.h>
#include <uORB/topics/debug_key_value.h>

#include <board_config.h>

/* Configuration Constants */
#ifdef PX4_I2C_BUS_EXPANSION3
#define HYT271_BUS 		PX4_I2C_BUS_EXPANSION3
#else
#define HYT271_BUS 		PX4_I2C_BUS_EXPANSION
#endif
#define HYT271_BASEADDR 	0x28
#define HYT271_DEVICE_PATH	"/dev/hyt271"


#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

//struct debug_key_value_s dbg;
//orb_advert_t 		pub_dbg;

class HYT271 : public device::I2C
{
public:
	HYT271(int bus = HYT271_BUS,
	      int address = HYT271_BASEADDR);
	virtual ~HYT271();

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
	float				_humidity;
	int                 _conversion_interval;
	work_s				_work;
	ringbuffer::RingBuffer  *_reports;
	bool				_sensor_ok;
	int				_measure_ticks;
	int				_class_instance;
	int				_orb_class_instance;
	int				_orb_sensor_pt100_fd;
	int				_orb_sensor_baro_fd;

	orb_advert_t		_meteo_topic;

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;



	/**
	* Test whether the device supported by the driver is present at a
	* specific address.
	*
	* @param address	The I2C bus address to probe.
	* @return			True if the device is present.
	*/
	int					probe_address(uint8_t address);

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

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int hyt271_main(int argc, char *argv[]);

HYT271::HYT271(int bus, int address) :
	I2C("HYT271", HYT271_DEVICE_PATH, bus, address, 400000),
	_temperature(-1.0f),
	_humidity(-1.0f),
	_conversion_interval(-1),
	_reports(nullptr),
	_sensor_ok(false),
	_measure_ticks(0),
	_class_instance(-1),
	_orb_class_instance(-1),
	_orb_sensor_baro_fd(-1),
	_meteo_topic(nullptr),
	_sample_perf(perf_alloc(PC_ELAPSED, "hyt271_read")),
	_comms_errors(perf_alloc(PC_COUNT, "hyt271_com_err"))

{
	/* enable debug() calls */
	_debug_enabled = false;

	/* work_cancel in the dtor will explode if we don't do this... */
	memset(&_work, 0, sizeof(_work));
}

HYT271::~HYT271()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	if (_meteo_topic != nullptr) {
		orb_unadvertise(_meteo_topic);
	}


	if (_class_instance != -1) {
		unregister_class_devname(METEO_BASE_DEVICE_PATH, _class_instance);
	}

	/* free perf counters */
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
HYT271::init()
{
	int ret = PX4_ERROR;

	_temperature = 0.0f;
	_humidity = 0.0f;
	_conversion_interval = 1000000;

	/* do I2C init (and probe) first */
	if (I2C::init() != OK) {
		return ret;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(meteo_s));

	set_device_address(HYT271_BASEADDR);

	if (_reports == nullptr) {
		return ret;
	}

	_class_instance = register_class_devname(METEO_BASE_DEVICE_PATH);

	/* get a publish handle on the range finder topic */
	struct meteo_s ds_report = {};

	_meteo_topic = orb_advertise_multi(ORB_ID(meteo), &ds_report,
			 &_orb_class_instance, ORB_PRIO_HIGH);

	if (_meteo_topic == nullptr) {
		DEVICE_LOG("failed to create meteo object. Did you start uOrb?");
	}

	// Select altitude register
	int ret2 = measure();

	if (ret2 == 0) {
		ret = OK;
		_sensor_ok = true;
	}

	return ret;
}

int
HYT271::probe()
{
	return measure();
}


int
HYT271::ioctl(device::file_t *filp, int cmd, unsigned long arg)
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
		return I2C::ioctl(filp, cmd, arg);
	}
}

ssize_t
HYT271::read(device::file_t *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct meteo_s);
	struct meteo_s *rbuf = reinterpret_cast<struct meteo_s *>(buffer);
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
HYT271::measure()
{
	int ret;

	uint8_t cmd = 0;
	ret = transfer(&cmd, 1, nullptr, 0);

	if (OK != ret) {
		perf_count(_comms_errors);
		DEVICE_DEBUG("i2c::transfer returned %d", ret);
		return ret;
	}

	ret = OK;

	return ret;
}

int
HYT271::collect()
{
	int	ret = -EIO;

	/* read from the sensor */
	uint8_t val[4] = {0, 0, 0, 0};
	perf_begin(_sample_perf);

	ret = transfer(nullptr, 0, &val[0], 4);


	if (ret < 0) {
		DEVICE_DEBUG("error reading from sensor: %d", ret);
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

    float humidity = ((val[0] & 0x3f) << 8 | val[1]) * (100.0 / 0x3fff);
    float temperature = (val[2] << 8 | (val[3] & 0xfc)) * (165.0 / 0xfffc) - 40;

	_orb_sensor_pt100_fd = orb_subscribe(ORB_ID(pt100));
	struct pt100_s pt100_msg;

	float pt100;
	if (orb_copy(ORB_ID(pt100), _orb_sensor_pt100_fd, &pt100_msg) == PX4_OK) {
		pt100 = pt100_msg.pt100;
	}
	else{
		pt100 = 0;
	}

	_orb_sensor_baro_fd = orb_subscribe(ORB_ID(mhp));
	struct mhp_s baro_msg;

	float ps;
	if (orb_copy(ORB_ID(mhp), _orb_sensor_baro_fd, &baro_msg) == PX4_OK) {
		ps = baro_msg.dpS/100;
	}
	else{
		ps = 1000;
	}

    float kappa = 0.28585657;
    float E_hc = 6.107*pow(10,(7.45*(double)temperature/(235.0+(double)temperature))); //saturation vapor pressure
    float e_hu = (double)humidity / 100.0 * (double)E_hc; // hPa
    float m_hu = 621.97 * (double)e_hu / ((double)ps - (double)e_hu); //mixing ratio
    float t_vir = ((double)temperature + 273.15) * (1.0 + (0.61 * (double)m_hu / 1000.0)) ; // virtual temperature
    float t_pot_v = (double)t_vir* pow((1000.0 / (double)ps),kappa); // virtual potential temperature
//    float Rd_par = 287.0;  // gas constant dry air [J/K/kg]
//    float rho  = (double)ps * 100.0 / ((double)Rd_par * (double)t_vir);
//    float a_hu = (double)e_hu / (461.0 * ((double)temperature + 273.15)) * 100.0 * 1000.0; // absolute humidity g/m^3
    float q_hu = (double)m_hu/1000 / ((double)m_hu/1000+1); // absolute humidity g/m^3


	struct meteo_s report;
	report.timestamp = hrt_absolute_time();
	report.humidity = humidity;
	report.temperature = temperature;
	report.t_pot_v = t_pot_v;
	report.q_hu = q_hu;
	report.pt100 = pt100;
	/* TODO: set proper ID */
	//report.id = 333;

	/* publish it, if we are the primary */
	if (_meteo_topic != nullptr) {
		orb_publish(ORB_ID(meteo), _meteo_topic, &report);
//		PX4_WARN("meteo published");
	}

	_reports->force(&report);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	ret = OK;

	if (_orb_sensor_baro_fd != -1) {
		orb_unsubscribe(_orb_sensor_baro_fd);
	}

	perf_end(_sample_perf);
	return ret;
}

void
HYT271::start()
{
	/* reset the report ring and state machine */
	_reports->flush();

	/* set register to '0' */
	measure();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&HYT271::cycle_trampoline, this, USEC2TICK(_conversion_interval));
}

void
HYT271::stop()
{
	work_cancel(HPWORK, &_work);
}

void
HYT271::cycle_trampoline(void *arg)
{
	HYT271 *dev = (HYT271 *)arg;

	dev->cycle();
}

void
HYT271::cycle()
{

	/* trigger a measurement */
	if (OK != measure()) {
		DEVICE_DEBUG("measure error");
		return;
	}

	/* wait for it to complete */
	usleep(100000);

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
		   (worker_t)&HYT271::cycle_trampoline,
		   this,
		   USEC2TICK(_conversion_interval));

}

void
HYT271::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	_reports->print_info("report queue");
}

/**
 * Local functions in support of the shell command.
 */
namespace hyt271
{

HYT271	*g_dev;

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
	g_dev = new HYT271(HYT271_BUS);

	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = open(HYT271_DEVICE_PATH, O_RDONLY);

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
	struct meteo_s report;
	ssize_t sz;
	int ret;

	int fd = open(HYT271_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'hyt271 start' if the driver is not running", HYT271_DEVICE_PATH);
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
	int fd = open(HYT271_DEVICE_PATH, O_RDONLY);

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
hyt271_main(int argc, char *argv[])
{
	// check for optional arguments
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

//	strncpy(dbg.key, "debug_test", sizeof(dbg.key));
//	dbg.value = 0.0f;
//	pub_dbg = orb_advertise(ORB_ID(debug_key_value), &dbg);

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
		hyt271::start();
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[myoptind], "stop")) {
		hyt271::stop();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[myoptind], "test")) {
		hyt271::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[myoptind], "reset")) {
		hyt271::reset();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[myoptind], "info") || !strcmp(argv[myoptind], "status")) {
		hyt271::info();
	}

	PX4_ERR("unrecognized command, try 'start', 'test', 'reset' or 'info'");
	return PX4_ERROR;
}
