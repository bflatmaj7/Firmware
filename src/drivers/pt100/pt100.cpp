/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file pt100.cpp
 * Driver for the MAX36185 PT100 temperature sensor.
 */

#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <getopt.h>
#include <px4_log.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <arch/board/board.h>
#include <board_config.h>
#include "pt100.h"

#include <drivers/device/device.h>
#include <drivers/drv_meteo.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/ringbuffer.h>

#include <perf/perf_counter.h>
#include <systemlib/err.h>

#include <uORB/topics/meteo.h>

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

/* Configuration Constants */
enum PT100_BUS {
	PT100_BUS_SPI_EXTERNAL=0
};

/*
 * PT100 internal constants and data structures.
 */

class PT100 : public device::CDev
{
public:
	PT100(pt100::IPT100 *interface, const char *path);
	virtual ~PT100();

	virtual int		init();

	virtual ssize_t	read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

private:
	pt100::IPT100		*_interface;

	bool                _running;

	uint8_t				_curr_ctrl;

	struct work_s		_work;
	unsigned			_report_ticks; // 0 - no cycling, otherwise period of sending a report
	unsigned			_max_mesure_ticks; //ticks needed to measure

	ringbuffer::RingBuffer	*_reports;

	bool			_collect_phase;

	orb_advert_t		_meteo_topic;
	int					_orb_class_instance;
	int					_class_instance;

	perf_counter_t		_sample_perf;
	perf_counter_t		_measure_perf;
	perf_counter_t		_comms_errors;

	float			_T; /* in degC */


	/* periodic execution helpers */
	void			start_cycle();
	void			stop_cycle();
	void			cycle(); //main execution
	static void		cycle_trampoline(void *arg);

	int		measure(); //start measure
	int		collect(); //get results and publish
};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int pt100_main(int argc, char *argv[]);

PT100::PT100(pt100::IPT100 *interface, const char *path) :
	CDev("PT100", path),
	_interface(interface),
	_running(false),
	_report_ticks(0),
	_reports(nullptr),
	_collect_phase(false),
	_meteo_topic(nullptr),
	_orb_class_instance(-1),
	_class_instance(-1),
	_sample_perf(perf_alloc(PC_ELAPSED, "pt100_read")),
	_measure_perf(perf_alloc(PC_ELAPSED, "pt100_measure")),
	_comms_errors(perf_alloc(PC_COUNT, "pt100_comms_errors"))
{
	// work_cancel in stop_cycle called from the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));
}

PT100::~PT100()
{
	/* make sure we are truly inactive */
	stop_cycle();

	if (_class_instance != -1) {
		unregister_class_devname(METEO_BASE_DEVICE_PATH, _class_instance);
	}

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	if (_meteo_topic != nullptr) {
		orb_unadvertise(_meteo_topic);
	}

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_measure_perf);
	perf_free(_comms_errors);

	delete _interface;
}

int
PT100::init()
{
	int ret;

	ret = CDev::init();

	if (ret != OK) {
		PX4_WARN("CDev init failed");
		return ret;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(meteo_s));

	if (_reports == nullptr) {
		PX4_WARN("can't get memory for reports");
		ret = -ENOMEM;
		return ret;
	}

	/* register alternate interfaces if we have to */
	_class_instance = register_class_devname(METEO_BASE_DEVICE_PATH);

	/* reset sensor */
	_interface->setWires(MAX31865_4WIRE);
	_interface->enableBias(false);
	_interface->autoConvert(false);
	_interface->clearFault();
//	uint8_t fault = _interface->readFault();
//	PX4_WARN("fault read: %d", fault);

	usleep(10000);

	/* do a first measurement cycle to populate reports with valid data */
	struct meteo_s mrp;
	_reports->flush();

	if (measure()) {
		PX4_WARN("measure error");
		return -EIO;
	}

	usleep(TICK2USEC(_max_mesure_ticks));

	if (collect()) {
		PX4_WARN("collect error");
		return -EIO;
	}

	_reports->get(&mrp);

	_meteo_topic = orb_advertise_multi(ORB_ID(meteo), &mrp,
			 &_orb_class_instance, ORB_PRIO_HIGH);

	if (_meteo_topic == nullptr) {
		PX4_WARN("failed to create meteo publication");
		return -ENOMEM;
	}

	return OK;

}

ssize_t
PT100::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct meteo_s);
	struct meteo_s *mrp = reinterpret_cast<struct meteo_s *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_report_ticks > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_reports->get(mrp)) {
				ret += sizeof(*mrp);
				mrp++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */

	_reports->flush();

	if (measure()) {
		return -EIO;
	}

	usleep(TICK2USEC(_max_mesure_ticks));

	if (collect()) {
		return -EIO;
	}

	if (_reports->get(mrp)) { //get new generated report
		ret = sizeof(*mrp);
	}

	return ret;
}

int
PT100::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {

			unsigned ticks = 0;

			switch (arg) {

			case SENSOR_POLLRATE_MANUAL:
				stop_cycle();
				_report_ticks = 0;
				return OK;

			case SENSOR_POLLRATE_EXTERNAL:
			case 0:
				return -EINVAL;

			case SENSOR_POLLRATE_MAX:

			/* FALLTHROUGH */
			case SENSOR_POLLRATE_DEFAULT:
				ticks = _max_mesure_ticks;

			/* FALLTHROUGH */
			default: {
					if (ticks == 0) {
						ticks = USEC2TICK(USEC_PER_SEC / arg);
					}

					/* do we need to start internal polling? */
					bool want_start = (_report_ticks == 0);

					/* check against maximum rate */
					if (ticks < _max_mesure_ticks) {
						return -EINVAL;
					}

					_report_ticks = ticks;

					if (want_start) {
						start_cycle();
					}

					return OK;
				}
			}

			break;
		}

	case SENSORIOCGPOLLRATE:
		if (_report_ticks == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}

		return (USEC_PER_SEC / USEC_PER_TICK / _report_ticks);

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			irqstate_t flags = px4_enter_critical_section();

			if (!_reports->resize(arg)) {
				px4_leave_critical_section(flags);
				return -ENOMEM;
			}

			px4_leave_critical_section(flags);
			return OK;
		}

	case SENSORIOCRESET:
		/*
		 * Since we are initialized, we do not need to do anything, since the
		 * PROM is correctly read and the part does not need to be configured.
		 */
		return OK;

	default:
		break;
	}

	return CDev::ioctl(filp, cmd, arg);
}

void
PT100::start_cycle()
{

	/* reset the report ring and state machine */
	_collect_phase = false;
	_running = true;
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&PT100::cycle_trampoline, this, 1);
}

void
PT100::stop_cycle()
{
	_running = false;
	work_cancel(HPWORK, &_work);
}

void
PT100::cycle_trampoline(void *arg)
{
	PT100 *dev = reinterpret_cast<PT100 *>(arg);

	dev->cycle();
}

void
PT100::cycle()
{
	if (_collect_phase) {
		collect();
		unsigned wait_gap = _report_ticks - _max_mesure_ticks;

		if ((wait_gap != 0) && (_running)) {
			work_queue(HPWORK, &_work, (worker_t)&PT100::cycle_trampoline, this,
				   wait_gap); //need to wait some time before new measurement
			return;
		}

	}

	measure();

	if (_running) {
		work_queue(HPWORK, &_work, (worker_t)&PT100::cycle_trampoline, this, _max_mesure_ticks);
	}

}

int
PT100::measure()
{
	_collect_phase = true;

	perf_begin(_measure_perf);

	/* start measure */
//	int ret = _interface->readRTD();

//	if (ret != OK) {
//		perf_count(_comms_errors);
//		perf_cancel(_measure_perf);
//		return -EIO;
//	}

	perf_end(_measure_perf);

	return OK;
}

int
PT100::collect()
{
	_collect_phase = false;

	perf_begin(_sample_perf);

	struct meteo_s report;
	/* this should be fairly close to the end of the conversion, so the best approximation of the time */
	report.timestamp = hrt_absolute_time();

	uint16_t temp = _interface->readRTD();

	//convert data to temperature
//	uint16_t temp =  _interface->temperature(100,430);

	report.temperature = temp;

	/* publish it */
	if (!(_pub_blocked)) {
		/* publish it */
		orb_publish(ORB_ID(meteo), _meteo_topic, &report);
	}

	_reports->force(&report);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	perf_end(_sample_perf);

	return OK;
}

void
PT100::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %u ticks\n", _report_ticks);
	_reports->print_info("report queue");
}

/**
 * Local functions in support of the shell command.
 */
namespace pt100
{

/*
  list of supported bus configurations
 */
struct pt100_bus_option {
	enum PT100_BUS busid;
	const char *devpath;
	PT100_constructor interface_constructor;
	uint8_t busnum;
	uint32_t device;
	bool external;
	PT100 *dev;
} bus_options[] = {
	{ PT100_BUS_SPI_EXTERNAL, "/dev/pt100_spi", &pt100_spi_interface, PX4_SPI_BUS_EXTERNAL1, PX4_SPIDEV_EXTERNAL1_1, true},
};
#define NUM_BUS_OPTIONS (sizeof(bus_options)/sizeof(bus_options[0]))

bool	start_bus(struct pt100_bus_option &bus);
struct pt100_bus_option &find_bus(enum PT100_BUS busid);
void	start(enum PT100_BUS busid);
void	test(enum PT100_BUS busid);
void	reset(enum PT100_BUS busid);
void	info();
void	usage();


/**
 * Start the driver.
 */
bool
start_bus(struct pt100_bus_option &bus)
{

	if (bus.dev != nullptr) {
		PX4_ERR("bus option already started");
		return false;
//		exit(1);
	}

	pt100::IPT100 *interface = bus.interface_constructor(bus.busnum, bus.device);

	if (interface->init() != OK) {
		delete interface;
		PX4_WARN("no device on bus %u", (unsigned)bus.busid);
		return false;
	}

	bus.dev = new PT100(interface, bus.devpath);

	if (bus.dev == nullptr) {
		PX4_ERR("dev error");
		return false;
	}

	if (OK != bus.dev->init()) {
		delete bus.dev;
		bus.dev = nullptr;
		PX4_ERR("dev init error");
		return false;
	}

	int fd = open(bus.devpath, O_RDONLY);

	/* set the poll rate to default, starts automatic data collection */
	if (fd == -1) {
		PX4_ERR("can't open spi device");
//		exit(1);
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		close(fd);
		PX4_ERR("failed setting default poll rate");
//		exit(1);
	}

	close(fd);
	return true;
}


/**
 * Start the driver.
 *
 * This function call only returns once the driver
 * is either successfully up and running or failed to start.
 */
void
start(enum PT100_BUS busid)
{
	uint8_t i;
	bool started = false;

	for (i = 0; i < NUM_BUS_OPTIONS; i++) {
		if (bus_options[i].dev != NULL) {
			PX4_WARN("dev not null");
			// this device is already started
			continue;
		}

		if (bus_options[i].busid != busid) {
			PX4_WARN("busid wrong");
			// not the one that is asked for
			continue;
		}

		started |= start_bus(bus_options[i]);
	}

	if (!started) {
		PX4_WARN("bus option number is %d", i);
		PX4_ERR("driver start failed");
		exit(1);
	}

	// one or more drivers started OK
	exit(0);
}


/**
 * find a bus structure for a busid
 */
struct pt100_bus_option &find_bus(enum PT100_BUS busid)
{
	for (uint8_t i = 0; i < NUM_BUS_OPTIONS; i++) {
		if ((busid == bus_options[i].busid) && bus_options[i].dev != NULL) {
			return bus_options[i];
		}
	}

	PX4_ERR("bus %u not started", (unsigned)busid);
	exit(1);
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test(enum PT100_BUS busid)
{
	struct pt100_bus_option &bus = find_bus(busid);
	struct meteo_s report;
	ssize_t sz;
	int ret;

	int fd;

	fd = open(bus.devpath, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("open failed (try 'pt100 start' if the driver is not running)");
		exit(1);
	}

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		PX4_ERR("immediate read failed");
		exit(1);
	}

	print_message(report);

	/* set the queue depth to 10 */
	if (OK != ioctl(fd, SENSORIOCSQUEUEDEPTH, 10)) {
		PX4_ERR("failed to set queue depth");
		exit(1);
	}

	/* start the sensor polling at 2Hz */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
		PX4_ERR("failed to set 2Hz poll rate");
		exit(1);
	}

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != 1) {
			PX4_ERR("timed out waiting for sensor data");
			exit(1);
		}

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			PX4_ERR("periodic read failed");
			exit(1);
		}

		print_message(report);
	}

	close(fd);
	PX4_ERR("PASS");
	exit(0);
}

/**
 * Reset the driver.
 */
void
reset(enum PT100_BUS busid)
{
	struct pt100_bus_option &bus = find_bus(busid);
	int fd;

	fd = open(bus.devpath, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("failed ");
		exit(1);
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		PX4_ERR("driver reset failed");
		exit(1);
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		PX4_ERR("driver poll restart failed");
		exit(1);
	}

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	for (uint8_t i = 0; i < NUM_BUS_OPTIONS; i++) {
		struct pt100_bus_option &bus = bus_options[i];

		if (bus.dev != nullptr) {
			PX4_WARN("%s", bus.devpath);
			bus.dev->print_info();
		}
	}

	exit(0);
}

void
usage()
{
	PX4_WARN("missing command: try 'start', 'info', 'test', 'test2', 'reset'");
	PX4_WARN("options:");
}

} // namespace

int
pt100_main(int argc, char *argv[])
{
	enum PT100_BUS busid = PT100_BUS_SPI_EXTERNAL;

	const char *verb = argv[optind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		pt100::start(busid);
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test")) {
		pt100::test(busid);
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset")) {
		pt100::reset(busid);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		pt100::info();
	}

	PX4_ERR("unrecognized command, try 'start', 'test', 'reset' or 'info'");
	exit(1);
}
