/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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

#include <termios.h>

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

#include <drivers/drv_hrt.h>
#include <drivers/drv_mhp.h>
#include <drivers/device/device.h>
#include <drivers/device/ringbuffer.h>

#include <perf/perf_counter.h>
#include <systemlib/err.h>

#include <uORB/uORB.h>
#include <uORB/topics/subsystem_info.h>
#include <uORB/topics/mhp.h>

#include <board_config.h>

#define PSC5B_DEFAULT_PORT	"/dev/ttyS3"

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

typedef enum _Psc5bMsgStatus{
	MSG_EMPTY = 0,
	MSG_COMPLETE
} Psc5bMsgStatus;

typedef struct{
	uint16_t id;
	uint8_t dlc;
	uint8_t data[10];
	Psc5bMsgStatus status;

} PSC5B_MESSAGE;


enum PSC5B_PARSE_STATE {
	PSC5B_PARSE_STATE0_ID1 = 0,
	PSC5B_PARSE_STATE1_ID2 = 1,
//	PSC5B_PARSE_STATE2_DLC = 2,
	PSC5B_PARSE_STATE3_DATA = 2
};

#define PSC5B_CANID1 0x1111
#define PSC5B_CANID2 0x1211

int psc5b_parser(char c, char *parserbuf,  unsigned *parserbuf_index, PSC5B_PARSE_STATE *state,PSC5B_MESSAGE *msg);

class PSC5B : public device::CDev
{
public:
	PSC5B(const char *port = PSC5B_DEFAULT_PORT);
	virtual ~PSC5B();

	virtual int 		init();

	virtual ssize_t		read(device::file_t *filp, char *buffer, size_t buflen);
	virtual int			ioctl(device::file_t *filp, int cmd, unsigned long arg);

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void				print_info();

protected:

private:
	char 				_port[20];
	PSC5B_MESSAGE	 	_msg;
	float				_dp0;
	float				_dp1;
	float				_dp2;
	float				_dp3;
	float				_dp4;
	float				_dpS;
	float				_aoa;
	float				_aos;
	float				_tas;
	int                 _conversion_interval;
	work_s				_work;
	ringbuffer::RingBuffer		*_reports;
	int				_measure_ticks;
	bool				_collect_phase;
	int				_fd;
	char				_linebuf[255];
	unsigned			_linebuf_index;
	enum PSC5B_PARSE_STATE		_parse_state;
	hrt_abstime			_last_read;
	int				_class_instance;
	int				_orb_class_instance;

	orb_advert_t		_mhp_topic;

	unsigned			_consecutive_fail_count;

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;

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
	void			cycle();
	int				collect();
	int				handle_msg(PSC5B_MESSAGE *msg);
	int				calc_flow();

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
extern "C" __EXPORT int psc5b_main(int argc, char *argv[]);

PSC5B::PSC5B(const char *port) :
	CDev("PSC5B", MHP0_DEVICE_PATH),
	_dp0(-1.0f),
	_dp1(-1.0f),
	_dp2(-1.0f),
	_dp3(-1.0f),
	_dp4(-1.0f),
	_dpS(-1.0f),
	_aoa(-1.0f),
	_aos(-1.0f),
	_tas(-1.0f),
	_conversion_interval(-1),
	_reports(nullptr),
	_measure_ticks(0),
	_collect_phase(false),
	_fd(-1),
	_linebuf_index(0),
	_parse_state(PSC5B_PARSE_STATE0_ID1),
	_last_read(0),
	_class_instance(-1),
	_orb_class_instance(-1),
	_mhp_topic(nullptr),
	_consecutive_fail_count(0),
	_sample_perf(perf_alloc(PC_ELAPSED, "psc5b_read")),
	_comms_errors(perf_alloc(PC_COUNT, "psc5b_com_err"))

{
	/* store port name */
	strncpy(_port, port, sizeof(_port));
	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';

	/* open fd */
	_fd = ::open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_fd < 0) {
		warnx("FAIL: psc5b fd");
	}

	struct termios uart_config;

	int termios_state;

	/* fill the struct for the new configuration */
	tcgetattr(_fd, &uart_config);

	/* clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;

	/* no parity, one stop bit */
	uart_config.c_cflag &= ~(CSTOPB | PARENB);

	unsigned speed = B9600;

	/* set baud rate */
	if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
		warnx("ERR CFG: %d ISPD", termios_state);
	}

	if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
		warnx("ERR CFG: %d OSPD\n", termios_state);
	}

	if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
		warnx("ERR baud %d ATTR", termios_state);
	}

	// disable debug() calls
	_debug_enabled = false;

	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));
}

PSC5B::~PSC5B()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	if (_class_instance != -1) {
		unregister_class_devname(MHP_BASE_DEVICE_PATH, _class_instance);
	}

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
PSC5B::init()
{

	_dp0 = 0.0f;
	_dp1 = 0.0f;
	_dp2 = 0.0f;
	_dp3 = 0.0f;
	_dp4 = 0.0f;
	_dpS = 0.0f;
	_aoa = 0.0f;
	_aos = 0.0f;
	_tas = 0.0f;
	_conversion_interval = 100000;

	int ret = 0;

	do { /* create a scope to handle exit conditions using break */

		/* do regular cdev init */
		ret = CDev::init();

		if (ret != OK) { break; }

		/* allocate basic report buffers */
		_reports = new ringbuffer::RingBuffer(2, sizeof(mhp_s));

		if (_reports == nullptr) {
			warnx("mem err");
			ret = -1;
			break;
		}

		_class_instance = register_class_devname(MHP_BASE_DEVICE_PATH);

		/* get a publish handle on the range finder topic */
		struct mhp_s ds_report = {};

		_mhp_topic = orb_advertise_multi(ORB_ID(mhp), &ds_report,
					 &_orb_class_instance, ORB_PRIO_HIGH);

		if (_mhp_topic == nullptr) {
			DEVICE_LOG("failed to create ins object. Did you start uOrb?");
		}

	} while (0);

	/* close the fd */
	::close(_fd);
	_fd = -1;

	return OK;
}

int
PSC5B::ioctl(device::file_t *filp, int cmd, unsigned long arg)
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
		return CDev::ioctl(filp, cmd, arg);
	}
}

ssize_t
PSC5B::read(device::file_t *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct mhp_s);
	struct mhp_s *rbuf = reinterpret_cast<struct mhp_s *>(buffer);
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
PSC5B::collect()
{
	int	ret = 1;

	perf_begin(_sample_perf);

	/* clear buffer if last read was too long ago */
	int64_t read_elapsed = hrt_elapsed_time(&_last_read);

	/* the buffer for read chars is buflen minus null termination */
	char readbuf[sizeof(_linebuf)];
	unsigned readlen = sizeof(readbuf) - 1;
	ret = ::read(_fd, &readbuf[0], readlen);

	if (ret > 0) {
		int byte_count = ret;
		for (int i = 0; i < byte_count; i++) {
			if (OK == psc5b_parser(readbuf[i], _linebuf, &_linebuf_index, &_parse_state, &_msg)) {
				if (_msg.status == MSG_COMPLETE){
					ret = handle_msg(&_msg);
					_msg.status = MSG_EMPTY;
				}
//				char test_str[7];
//				sprintf(test_str,"s: %d",_parse_state);
//				::write(_fd,test_str,7);
			}
		}

		_last_read = hrt_absolute_time();

		perf_end(_sample_perf);

	} else if (ret == 0) {
//		PX4_WARN("no data received.");
		return -EAGAIN;
	} else if (ret < 0) {
//		::write(_fd,"test_nor",8);
		perf_count(_comms_errors);
		perf_end(_sample_perf);
			/* only throw an error if we time out */
		if (read_elapsed > (_conversion_interval * 2)) {
//			PX4_WARN("time out");
			return ret;
			} else {
//			PX4_WARN("serial read failed");
			return ret;
		}

	}

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	return ret;
}

int
PSC5B::handle_msg(PSC5B_MESSAGE *msg)
{

	switch (msg->id){
	case PSC5B_CANID1:
		_dp0 = msg->data[0]<<8 | msg->data[1];
		_dp1 = msg->data[2]<<8 | msg->data[3];
		_dp2 = msg->data[4]<<8 | msg->data[5];
		_dp3 = msg->data[6]<<8 | msg->data[7];
		break;
	case PSC5B_CANID2:
		_dp4 = msg->data[0]<<8 | msg->data[1];
		_dpS = (msg->data[2]<<8 | msg->data[3])*5;
		calc_flow();
		break;
	default:
		break;
	}

	struct mhp_s report;

	report.timestamp = hrt_absolute_time();
	report.dp0 = _dp0;
	report.dp1 = _dp1;
	report.dp2 = _dp2;
	report.dp3 = _dp3;
	report.dp4 = _dp4;
	report.dpS = _dpS;
	report.aoa = _aoa;
	report.aos = _aos;
	report.tas = _tas;
	/* TODO: set proper ID */
	//report.id = 0;

	/* publish it */
	orb_publish(ORB_ID(mhp), _mhp_topic, &report);

	_reports->force(&report);

	return OK;

}

int
PSC5B::calc_flow()
{
	double dP = 1.0/4.0 * (double)(_dp1 + _dp2 + _dp3 + _dp4) ;
    // Calculate coefficients k_a and k_b
	double k_a = (double)(_dp1-_dp3)/((double)_dp0-dP) ; // alpha
	double k_b = (double)(_dp2-_dp4)/((double)_dp0-dP) ; // beta

    // Calculate the dimensionless pressure coefficients
	double Si_a=0.0;
	double Si_b=0.0;
	double Si_q=0.0;
	double Si_p=0.0;
	for(int i=0;i<=poly_order;i++)
	{
		double Sj_a=0.0;
		double Sj_b=0.0;
		double Sj_q=0.0;
		double Sj_p=0.0;
		for(int j=0;j<=poly_order;j++)
		  {
			Sj_a = Sj_a + poly_alpha[(i+1)*j] * pow(k_b,j);
			Sj_b = Sj_b + poly_beta[(i+1)*j] * pow(k_b,j);
			Sj_q = Sj_q + poly_kq[(i+1)*j] * pow(k_b,j);
			Sj_p = Sj_p + poly_kp[(i+1)*j] * pow(k_b,j);
	 //		    cout << Sj_a << "  " << a[i][j] << "  " << pow(k_b[k],j) << "  " << k_b[k] << endl;
			//cout << Sj_b << "  " << b[i][j] << "  " << pow(k_b[k],j) << endl;
			//cout << Sj_p << "  " << pt[i][j] << "  " << pow(k_b[k],j) << endl;
		  }
		Si_a = Si_a + Sj_a * pow(k_a,i);
		Si_b = Si_b + Sj_b * pow(k_a,i);
		Si_q = Si_q + Sj_q * pow(k_a,i);
		Si_p = Si_p + Sj_p * pow(k_a,i);
	}
	if(sqrt(pow(k_a,2))<1.5 && sqrt(pow(k_b,2))<1.5)
	  {
		_aoa = Si_a ; // angle of attack [deg]
		_aos = Si_b ; // sideslip angle [deg]
		_tas = ((Si_q * ((double)_dp0-dP)) + (double)_dp0) / 100.0 ;  // dynamic pressure [hPa]
//		data[index.p_s_c].value[k] = (P[k] + p0[k] - Si_p * (p0[k]-dP[k])) / 100.0; // static pressure [hPa]
	  }
	else  // dummy values by no airspeed
	  {
		_aoa = -99.0;  // angle of attack [deg]
		_aos = -99.0 ; // sideslip angle [deg]
		_tas = -99.0 ;  // dynamic pressure [hPa]
//		data[index.p_s_c].value[k] = data[index.p_s1].value[k] ;  // static pressure [hPa]
	  }


	return 0;
}


void
PSC5B::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&PSC5B::cycle_trampoline, this, 1);
}

void
PSC5B::stop()
{
	work_cancel(HPWORK, &_work);
}

void
PSC5B::cycle_trampoline(void *arg)
{
	PSC5B *dev = static_cast<PSC5B *>(arg);

	dev->cycle();
}

void
PSC5B::cycle()
{
	/* fds initialized? */
	if (_fd < 0) {
		/* open fd */
		_fd = ::open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);
	}

	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		collect();

		/* next phase is measurement */
		_collect_phase = false;

		/*
		 * Is there a collect->measure gap?
		 */
		if (_measure_ticks > USEC2TICK(_conversion_interval)) {

			/* schedule a fresh cycle call when we are ready to measure again */
			work_queue(HPWORK,
				   &_work,
				   (worker_t)&PSC5B::cycle_trampoline,
				   this,
				   _measure_ticks - USEC2TICK(_conversion_interval));

			return;
		}
	}

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(HPWORK,
		   &_work,
		   (worker_t)&PSC5B::cycle_trampoline,
		   this,
		   USEC2TICK(_conversion_interval));

}

void
PSC5B::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	_reports->print_info("report queue");
}

/**
 * Local functions in support of the shell command.
 */
namespace psc5b
{

PSC5B	*g_dev;

void	start(const char *port);
void	stop();
void	test();
void	reset();
void	info();

/**
 * Start the driver.
 */
void
start(const char *port)
{
	int fd;

	if (g_dev != nullptr) {
		errx(1, "already started");
	}

	/* create the driver */
	g_dev = new PSC5B(port);

	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = open(MHP0_DEVICE_PATH, 0);

	if (fd < 0) {
		warnx("device open fail");
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		goto fail;
	}

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
	struct mhp_s report;
	ssize_t sz;

	int fd = open(MHP0_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'psc5b start' if the driver is not running", MHP0_DEVICE_PATH);
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
		int ret = poll(&fds, 1, 2000);

		if (ret != 1) {
			warnx("timed out");
			break;
		}

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			warnx("read failed: got %d vs exp. %d", sz, sizeof(report));
			break;
		}

		print_message(report);
	}

	/* reset the sensor polling to the default rate */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT)) {
		errx(1, "ERR: DEF RATE");
	}

	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(MHP0_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "failed ");
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		err(1, "driver reset failed");
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "driver poll restart failed");
	}

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

int psc5b_parser(char c, char *parserbuf,  unsigned *parserbuf_index, PSC5B_PARSE_STATE *state,PSC5B_MESSAGE *msg)
{
	int ret = 0;

	switch (*state) {
	case PSC5B_PARSE_STATE0_ID1:
		*state = PSC5B_PARSE_STATE1_ID2;
		msg->id = c;
		*parserbuf_index=0;

		break;

	case PSC5B_PARSE_STATE1_ID2:
		msg->id |= c<<8;
		if (msg->id==PSC5B_CANID1 || msg->id==PSC5B_CANID2){
//			*state = PSC5B_PARSE_STATE2_DLC;
			*state = PSC5B_PARSE_STATE3_DATA;
		}
		else{
			*state = PSC5B_PARSE_STATE0_ID1;
			return PX4_ERROR;
		}

		break;

//	case PSC5B_PARSE_STATE2_DLC:
//		msg->dlc = c;
//		if (msg->dlc==8){
//			*state = PSC5B_PARSE_STATE3_DATA;
//		}
//		else{
//			*state = PSC5B_PARSE_STATE0_ID1;
//			return PX4_ERROR;
//		}
//		break;

	case PSC5B_PARSE_STATE3_DATA:
		if (*parserbuf_index<9)
		{
			msg->data[*parserbuf_index] = c;
			(*parserbuf_index)++;
		}
		else
		{
			msg->data[*parserbuf_index] = c;
			*state = PSC5B_PARSE_STATE0_ID1;
			msg->status = MSG_COMPLETE;
			return PX4_OK;
		}
		break;
	}

	return ret;
}

int
psc5b_main(int argc, char *argv[])
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
		if (argc > myoptind + 1) {
			psc5b::start(argv[myoptind + 1]);

		} else {
			psc5b::start(PSC5B_DEFAULT_PORT);
		}
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[myoptind], "stop")) {
		psc5b::stop();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[myoptind], "test")) {
		psc5b::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[myoptind], "reset")) {
		psc5b::reset();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[myoptind], "info") || !strcmp(argv[myoptind], "status")) {
		psc5b::info();
	}

	PX4_ERR("unrecognized command, try 'start', 'test', 'reset' or 'info'");
	return PX4_ERROR;
}

