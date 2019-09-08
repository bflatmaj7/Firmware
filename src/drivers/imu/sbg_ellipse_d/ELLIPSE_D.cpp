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
#include <drivers/drv_imu.h>
#include <drivers/device/device.h>
#include <drivers/device/ringbuffer.h>

#include <perf/perf_counter.h>
#include <systemlib/err.h>

#include <uORB/uORB.h>
#include <uORB/topics/subsystem_info.h>
#include <uORB/topics/ins.h>
#include <uORB/topics/vehicle_gps_position.h>

#include <drivers/imu/sbg_ellipse_d/ELLIPSE_D.h>

#include <board_config.h>

// designated serial port TELEM2 on Pixhawk
#define ELLIPSE_D_DEFAULT_PORT		"/dev/ttyS2"

class ELLIPSE_D : public device::CDev
{
public:
	ELLIPSE_D(const char *port = ELLIPSE_D_DEFAULT_PORT);
	virtual ~ELLIPSE_D();

	virtual int 			init();

	virtual ssize_t			read(device::file_t *filp, char *buffer, size_t buflen);
	virtual int			ioctl(device::file_t *filp, int cmd, unsigned long arg);

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void				print_info();

private:
	char 				_port[20];
	ELLIPSE_MESSAGE 	_msg;
	unsigned			_clk_time;
	unsigned			_imu_time;
	unsigned			_mag_time;
	unsigned			_eul_time;
	unsigned			_nav_time;
	unsigned			_gps_hdt_time;
	unsigned			_gps_vel_time;
	unsigned			_gps_pos_time;
	short				_clk_year;
	char				_clk_month;
	char				_clk_day;
	unsigned			_clk_tow;
	float				_ax;
	float				_ay;
	float				_az;
	float				_gx;
	float				_gy;
	float				_gz;
	float				_dvelx;
	float				_dvely;
	float				_dvelz;
	float				_dangx;
	float				_dangy;
	float				_dangz;
	float				_mx;
	float				_my;
	float				_mz;
	float				_roll;
	float				_pitch;
	float				_yaw;
	float				_roll_acc;
	float				_pitch_acc;
	float				_yaw_acc;
	float				_vns;
	float				_vew;
	float				_vud;
	float				_vns_acc;
	float				_vew_acc;
	float				_vud_acc;
	float				_lat;
	float				_lon;
	float				_alt;
	float				_lat_acc;
	float				_lon_acc;
	float				_alt_acc;
	float				_alt_ellipsoid;
	float				_gps_vns;
	float				_gps_vew;
	float				_gps_vud;
	float				_gps_vns_acc;
	float				_gps_vew_acc;
	float				_gps_vud_acc;
	float				_gps_course;
	float				_gps_course_acc;
	float				_gps_heading;
	float				_gps_heading_acc;
	float				_gps_pitch;
	float				_gps_pitch_acc;
	unsigned			_sol_status;
	short 				_imu_status;
	unsigned			_gps_vel_status;
	short				_gps_hdt_status;
	int                 _conversion_interval;
	work_s				_work;
	ringbuffer::RingBuffer		*_reports;
	int					_measure_ticks;
	bool				_collect_phase;
	int					_fd;
	char				_linebuf[SBG_ECOM_MAX_BUFFER_SIZE];
	unsigned			_linebuf_index;
	enum ELLIPSE_D_PARSE_STATE		_parse_state;
	hrt_abstime			_last_read;
	int					_class_instance;
	int					_orb_class_instance;

	orb_advert_t			_ins_topic;
	struct vehicle_gps_position_s	_report_gps_pos;				///< uORB topic for gps position
	orb_advert_t			_report_gps_pos_pub;				///< uORB pub for gps position
	int					_gps_orb_instance;				///< uORB multi-topic instance

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
	int				handle_msg(ELLIPSE_MESSAGE *msg);
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
extern "C" __EXPORT int ellipse_d_main(int argc, char *argv[]);


ELLIPSE_D::ELLIPSE_D(const char *port) :
	CDev("ELLIPSE_D", IMU0_DEVICE_PATH),
	_clk_time(0),
	_imu_time(0),
	_mag_time(0),
	_eul_time(0),
	_nav_time(0),
	_gps_hdt_time(0),
	_gps_vel_time(0),
	_gps_pos_time(0),
	_clk_year(0),
	_clk_month(0),
	_clk_day(0),
	_clk_tow(0),
	_ax(-99),
	_ay(-99),
	_az(-99),
	_gx(-99),
	_gy(-99),
	_gz(-99),
	_dvelx(-99),
	_dvely(-99),
	_dvelz(-99),
	_dangx(-99),
	_dangy(-99),
	_dangz(-99),
	_mx(-99),
	_my(-99),
	_mz(-99),
	_roll(-99),
	_pitch(-99),
	_yaw(-99),
	_roll_acc(-99),
	_pitch_acc(-99),
	_yaw_acc(-99),
	_vns(-99),
	_vew(-99),
	_vud(-99),
	_vns_acc(-99),
	_vew_acc(-99),
	_vud_acc(-99),
	_lat(-99),
	_lon(-99),
	_alt(-99),
	_lat_acc(-99),
	_lon_acc(-99),
	_alt_acc(-99),
	_alt_ellipsoid(-99),
	_gps_vns(-99),
	_gps_vew(-99),
	_gps_vud(-99),
	_gps_vns_acc(-99),
	_gps_vew_acc(-99),
	_gps_vud_acc(-99),
	_gps_course(-99),
	_gps_course_acc(-99),
	_gps_heading(-99),
	_gps_heading_acc(-99),
	_gps_pitch(-99),
	_gps_pitch_acc(-99),
	_sol_status(0),
	_imu_status(0),
	_gps_vel_status(0),
	_gps_hdt_status(0),
	_conversion_interval(-1),
	_reports(nullptr),
	_measure_ticks(0),
	_collect_phase(false),
	_fd(-1),
	_linebuf_index(0),
	_parse_state(ELLIPSE_D_PARSE_STATE0_SYNC1),
	_last_read(0),
	_class_instance(-1),
	_orb_class_instance(-1),
	_ins_topic(nullptr),
	_report_gps_pos{},
	_report_gps_pos_pub(nullptr),
	_gps_orb_instance(-1),
	_consecutive_fail_count(0),
	_sample_perf(perf_alloc(PC_ELAPSED, "ellipse_d_read")),
	_comms_errors(perf_alloc(PC_COUNT, "ellipse_d_com_err"))
{
	/* store port name */
	strncpy(_port, port, sizeof(_port));
	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';

	/* open fd */
	_fd = ::open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_fd < 0) {
		warnx("FAIL: ellipse_d fd");
	}

	struct termios uart_config;

	int termios_state;

	/* fill the struct for the new configuration */
	tcgetattr(_fd, &uart_config);

	/* clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;

	/* no parity, one stop bit */
	uart_config.c_cflag &= ~(CSTOPB | PARENB);

	unsigned speed = B115200;

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

ELLIPSE_D::~ELLIPSE_D()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	if (_class_instance != -1) {
		unregister_class_devname(IMU_BASE_DEVICE_PATH, _class_instance);
	}

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}


int ELLIPSE_D::init()
{

	_ax = 0.0f;
	_ay = 0.0f;
	_az = 0.0f;
	_gx = 0.0f;
	_gy = 0.0f;
	_gz = 0.0f;
	_mx = 0.0f;
	_my = 0.0f;
	_mz = 0.0f;
	_roll = 0.0f;
	_pitch = 0.0f;
	_yaw = 0.0f;
	_vns = 0.0f;
	_vew = 0.0f;
	_vud = 0.0f;
	_lat = 0.0f;
	_lon = 0.0f;
	_alt = 0.0f;
	_alt_ellipsoid = 0.0f;
	_sol_status = 0;
	_imu_status = 0;
	_conversion_interval =	10000;
	/* status */
	int ret = 0;

	do { /* create a scope to handle exit conditions using break */

		/* do regular cdev init */
		ret = CDev::init();

		if (ret != OK) { break; }

		/* allocate basic report buffers */
		_reports = new ringbuffer::RingBuffer(2, sizeof(ins_s));

		if (_reports == nullptr) {
			warnx("mem err");
			ret = -1;
			break;
		}

		_class_instance = register_class_devname(IMU_BASE_DEVICE_PATH);

		/* get a publish handle on the range finder topic */
		struct ins_s ds_report = {};

		_ins_topic = orb_advertise_multi(ORB_ID(ins), &ds_report,
					 &_orb_class_instance, ORB_PRIO_HIGH);

		if (_ins_topic == nullptr) {
			DEVICE_LOG("failed to create ins object. Did you start uOrb?");
		}

	} while (0);

	/* close the fd */
	::close(_fd);
	_fd = -1;

	return OK;
}

int
ELLIPSE_D::ioctl(device::file_t *filp, int cmd, unsigned long arg)
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
					signed ticks = USEC2TICK(1000000 / arg);

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
ELLIPSE_D::read(device::file_t *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct ins_s);
	struct ins_s *rbuf = reinterpret_cast<struct ins_s *>(buffer);
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
ELLIPSE_D::collect()
{
	int	ret = 1;

	perf_begin(_sample_perf);

	/* clear buffer if last read was too long ago */
	int64_t read_elapsed = hrt_elapsed_time(&_last_read);

	/* the buffer for read chars is buflen minus null termination */
	char readbuf[sizeof(_linebuf)];
	unsigned readlen = sizeof(readbuf) - 1;
	ret = ::read(_fd, &readbuf[0], readlen);
//	::write(_fd,"test_123",8);

	if (ret > 0) {
		int byte_count = ret;
		for (int i = 0; i < byte_count; i++) {
			if (OK == ellipse_d_parser(readbuf[i], _linebuf, &_linebuf_index, &_parse_state, &_msg)) {
				if (_msg.status == MSG_COMPLETE){
					ret = handle_msg(&_msg);
					_msg.status = MSG_EMPTY;
				}
//				char test_str[4];
//				sprintf(test_str,"%d",_parse_state);
//				::write(_fd,test_str,4);
			}
		}

		_last_read = hrt_absolute_time();

		perf_end(_sample_perf);

	} else if (ret == 0) {
//		PX4_WARN("no data received.");
		return -EAGAIN;
	} else if (ret < 0) {
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
ELLIPSE_D::handle_msg(ELLIPSE_MESSAGE *msg)
{

	switch (msg->id){
	case SBG_ECOM_LOG_UTC_TIME:
		_clk_time = (*(unsigned *)&msg->data[0]);
		_clk_year = (*(short *)&msg->data[6]);
		_clk_month = msg->data[8];
		_clk_day = msg->data[9];
		_clk_tow = (*(unsigned *)&msg->data[17]);
		break;
	case SBG_ECOM_LOG_IMU_DATA:
		_imu_time = (*(unsigned *)&msg->data[0]);
		_imu_status = (*(short *)&msg->data[4]);
		_ax = (*(float *)&msg->data[6]);
		_ay = (*(float *)&msg->data[10]);
		_az = (*(float *)&msg->data[14]);
		_gx = (*(float *)&msg->data[18]);
		_gy = (*(float *)&msg->data[22]);
		_gz = (*(float *)&msg->data[26]);
		_dvelx = (*(float *)&msg->data[34]);
		_dvely = (*(float *)&msg->data[38]);
		_dvelz = (*(float *)&msg->data[42]);
		_dangx = (*(float *)&msg->data[46]);
		_dangy = (*(float *)&msg->data[50]);
		_dangz = (*(float *)&msg->data[54]);
		break;
	case SBG_ECOM_LOG_MAG:
		_mag_time = (*(unsigned *)&msg->data[0]);
		_mx = (*(float *)&msg->data[6]);
		_my = (*(float *)&msg->data[10]);
		_mz = (*(float *)&msg->data[14]);
		break;
	case SBG_ECOM_LOG_GPS1_HDT:
		_gps_hdt_time = (*(unsigned *)&msg->data[0]);
		_gps_hdt_status = *(short *)&msg->data[4];
		_gps_heading = (*(float *)&msg->data[10]);
		_gps_heading_acc = (*(float *)&msg->data[14]);
		_gps_pitch = (*(float *)&msg->data[18]);
		_gps_pitch_acc = (*(float *)&msg->data[22]);
		break;
	case SBG_ECOM_LOG_GPS1_VEL:
		_gps_vel_time = (*(unsigned *)&msg->data[0]);
		_gps_vel_status = *(unsigned *)&msg->data[4];
		_gps_vns = (*(float *)&msg->data[12]);
		_gps_vew = (*(float *)&msg->data[16]);
		_gps_vud = (*(float *)&msg->data[20]);
		_gps_vns_acc = (*(float *)&msg->data[24]);
		_gps_vew_acc = (*(float *)&msg->data[28]);
		_gps_vud_acc = (*(float *)&msg->data[32]);
		_gps_course = (*(float *)&msg->data[36]);
		_gps_course_acc = (*(float *)&msg->data[40]);
		break;
	case SBG_ECOM_LOG_EKF_EULER:
		_eul_time = (*(unsigned *)&msg->data[0]);
		_roll = (*(float *)&msg->data[4])*180/PI;
		_pitch = (*(float *)&msg->data[8])*180/PI;
		_yaw = (*(float *)&msg->data[12])*180/PI;
		_roll_acc = (*(float *)&msg->data[16])*180/PI;
		_pitch_acc = (*(float *)&msg->data[20])*180/PI;
		_yaw_acc = (*(float *)&msg->data[24])*180/PI;

		struct ins_s report;

		report.timestamp = hrt_absolute_time();
		report.clk_time = _clk_time;
		report.year = _clk_year;
		report.month = _clk_month;
		report.day = _clk_day;
		report.tow = _clk_tow;
		report.imu_time = _imu_time;
		report.ax = _ax;
		report.ay = _ay;
		report.az = _az;
		report.gx = _gx;
		report.gy = _gy;
		report.gz = _gz;
		report.dvelx = _dvelx;
		report.dvely = _dvely;
		report.dvelz = _dvelz;
		report.dangx = _dangx;
		report.dangy = _dangy;
		report.dangz = _dangz;
		report.mag_time = _mag_time;
		report.mx = _mx;
		report.my = _my;
		report.mz = _mz;
		report.eul_time = _eul_time;
		report.roll = _roll;
		report.pitch = _pitch;
		report.yaw = _yaw;
		report.roll_acc = _roll_acc;
		report.pitch_acc = _pitch_acc;
		report.yaw_acc = _yaw_acc;
		report.nav_time = _nav_time;
		report.vns = _vns;
		report.vew = _vew;
		report.vud = _vud;
		report.vns_acc = _vns_acc;
		report.vew_acc = _vew_acc;
		report.vud_acc = _vud_acc;
		report.gps_hdt_time = _gps_hdt_time;
		report.gps_heading = _gps_heading;
		report.gps_pitch = _gps_pitch;
		report.gps_course = _gps_course;
		report.gps_heading_acc = _gps_heading_acc;
		report.gps_pitch_acc = _gps_pitch_acc;
		report.gps_vel_time = _gps_vel_time;
		report.gps_vns = _gps_vns;
		report.gps_vew = _gps_vew;
		report.gps_vud = _gps_vud;
		report.gps_vns_acc = _gps_vns_acc;
		report.gps_vew_acc = _gps_vew_acc;
		report.gps_vud_acc = _gps_vud_acc;
		report.sol_status = _sol_status;
		report.imu_status = _imu_status;
		report.gps_hdt_status = _gps_hdt_status;
		report.gps_vel_status = _gps_vel_status;
		/* TODO: set proper ID */
		//report.id = 0;
		/* publish it */
		orb_publish(ORB_ID(ins), _ins_topic, &report);

		_reports->force(&report);

		break;
	case SBG_ECOM_LOG_EKF_NAV:
		_nav_time = (*(unsigned *)&msg->data[0]);
		_vns = *(float *)&msg->data[4];
		_vew = *(float *)&msg->data[8];
		_vud = *(float *)&msg->data[12];
		_vns_acc = *(float *)&msg->data[16];
		_vew_acc = *(float *)&msg->data[20];
		_vud_acc = *(float *)&msg->data[24];
		_lat = (int)((*(double *)&msg->data[28])*1e7);
		_lon = (int)((*(double *)&msg->data[36])*1e7);
		_alt = (int)((*(double *)&msg->data[44])*1000);
		_lat_acc = *(float *)&msg->data[56];
		_lon_acc = *(float *)&msg->data[60];
		_alt_acc = *(float *)&msg->data[64];
		_alt_ellipsoid = (int)((*(double *)&msg->data[44])*1000)+(int)((*(float *)&msg->data[52])*1000);
		_sol_status = *(unsigned *)&msg->data[68];

		break;
	case SBG_ECOM_LOG_GPS1_POS:
		_gps_pos_time = (*(unsigned *)&msg->data[0]);
		_report_gps_pos = {};
		_report_gps_pos.timestamp = hrt_absolute_time();
		_report_gps_pos.time_utc_usec =  (*(int *)&msg->data[8])* 1000ULL;
		_report_gps_pos.lat = (int)((*(double *)&msg->data[12])*1e7);
		_report_gps_pos.lon = (int)((*(double *)&msg->data[20])*1e7);
		_report_gps_pos.alt = (int)((*(double *)&msg->data[28])*1000);
		_report_gps_pos.alt_ellipsoid = (int)((*(double *)&msg->data[28])*1000)+(int)((*(float *)&msg->data[36])*1000);
		_report_gps_pos.s_variance_m_s = 0.5f;
		_report_gps_pos.c_variance_rad = 0.1f;
		_report_gps_pos.eph = (*(float *)&msg->data[40]);
		_report_gps_pos.epv = (*(float *)&msg->data[48]);
		_report_gps_pos.hdop = (*(float *)&msg->data[40]);
		_report_gps_pos.vdop = (*(float *)&msg->data[48]);
		_report_gps_pos.vel_n_m_s = _gps_vns;
		_report_gps_pos.vel_e_m_s = _gps_vew;
		_report_gps_pos.vel_d_m_s = _gps_vud;
		_report_gps_pos.satellites_used = msg->data[52];
		if ((msg->data[4]&0x1F)==0){
			_report_gps_pos.fix_type = 3;
		}else{
			_report_gps_pos.fix_type = 0;
		}
//		_report_gps_pos.fix_type = msg->data[4]&0x1F;

		orb_publish_auto(ORB_ID(vehicle_gps_position), &_report_gps_pos_pub, &_report_gps_pos, &_gps_orb_instance,
				 ORB_PRIO_DEFAULT);
		break;
	default:
		break;
	}

	return OK;

}

void
ELLIPSE_D::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&ELLIPSE_D::cycle_trampoline, this, 1);

}

void
ELLIPSE_D::stop()
{
	work_cancel(HPWORK, &_work);
}

void
ELLIPSE_D::cycle_trampoline(void *arg)
{
	ELLIPSE_D *dev = static_cast<ELLIPSE_D *>(arg);

	dev->cycle();
}

void
ELLIPSE_D::cycle()
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
//		int collect_ret = collect();

//		if (collect_ret == -EAGAIN) {
//			/* reschedule to grab the missing bits, time to transmit 8 bytes @ 9600 bps */
//			work_queue(HPWORK,
//				   &_work,
//				   (worker_t)&ELLIPSE_D::cycle_trampoline,
//				   this,
//				   USEC2TICK(1042 * 8));
//			return;
//		}
//
//		if (OK != collect_ret) {
//
//			/* we know the sensor needs about four seconds to initialize */
//			if (hrt_absolute_time() > 5 * 1000 * 1000LL && _consecutive_fail_count < 5) {
//				DEVICE_LOG("collection error #%u", _consecutive_fail_count);
//			}
//
//			_consecutive_fail_count++;
//
//			/* restart the measurement state machine */
//			start();
//			return;
//
//		} else {
//			/* apparently success */
//			_consecutive_fail_count = 0;
//		}

		/* next phase is measurement */
		_collect_phase = false;

		/*
		 * Is there a collect->measure gap?
		 */
		if (_measure_ticks > USEC2TICK(_conversion_interval)) {

			/* schedule a fresh cycle call when we are ready to measure again */
			work_queue(HPWORK,
				   &_work,
				   (worker_t)&ELLIPSE_D::cycle_trampoline,
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
		   (worker_t)&ELLIPSE_D::cycle_trampoline,
		   this,
		   USEC2TICK(_conversion_interval));
}

void
ELLIPSE_D::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %d ticks\n", _measure_ticks);
	_reports->print_info("report queue");
}




/**
 * Local functions in support of the shell command.
 */
namespace ellipse_d
{

ELLIPSE_D	*g_dev;

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
	g_dev = new ELLIPSE_D(port);

	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = open(IMU0_DEVICE_PATH, 0);

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
	struct ins_s report;
	ssize_t sz;

	int fd = open(IMU0_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'ellipse_d start' if the driver is not running", IMU0_DEVICE_PATH);
	}

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		err(1, "immediate read failed");
	}

	print_message(report);

	/* start the sensor polling at 2 Hz rate */
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
	int fd = open(IMU0_DEVICE_PATH, O_RDONLY);

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

} // namespace


int ellipse_d_parser(char c, char *parserbuf, unsigned *parserbuf_index, enum ELLIPSE_D_PARSE_STATE *state, ELLIPSE_MESSAGE *msg)
{
	int ret = 0;

	switch (*state) {
	case ELLIPSE_D_PARSE_STATE0_SYNC1:
		if (c == SBG_ECOM_SYNC_1) {
			*state = ELLIPSE_D_PARSE_STATE1_SYNC2;
			(*parserbuf_index) = 0;
		}

		break;

	case ELLIPSE_D_PARSE_STATE1_SYNC2:
		if (c == SBG_ECOM_SYNC_2) {
			*state = ELLIPSE_D_PARSE_STATE2_MSG;
		}
		else
		{
			*state = ELLIPSE_D_PARSE_STATE0_SYNC1;
			return PX4_ERROR;
		}

		break;

	case ELLIPSE_D_PARSE_STATE2_MSG:
		*state = ELLIPSE_D_PARSE_STATE3_CLASS;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;
		msg->id = c;

		break;

	case ELLIPSE_D_PARSE_STATE3_CLASS:
		*state = ELLIPSE_D_PARSE_STATE4_LEN1;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;
		msg->cl = c;

		break;

	case ELLIPSE_D_PARSE_STATE4_LEN1:
		*state = ELLIPSE_D_PARSE_STATE5_LEN2;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;
		msg->length = c;

		break;

	case ELLIPSE_D_PARSE_STATE5_LEN2:
		*state = ELLIPSE_D_PARSE_STATE6_DATA;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;
		msg->length |= c<<8;
		if (msg->length>SBG_ECOM_MAX_PAYLOAD_SIZE){
			*state = ELLIPSE_D_PARSE_STATE0_SYNC1;
			msg->length = 0;
			return PX4_ERROR;
		}

		break;

	case ELLIPSE_D_PARSE_STATE6_DATA:
		if ((*parserbuf_index)<(msg->length+4))
		{
			msg->data[(*parserbuf_index)-4] = c;
			parserbuf[*parserbuf_index] = c;
			(*parserbuf_index)++;

			break;
		}
		else
		{
			*state = ELLIPSE_D_PARSE_STATE7_CRC1;
		}

	case ELLIPSE_D_PARSE_STATE7_CRC1:
		*state = ELLIPSE_D_PARSE_STATE8_CRC2;
		msg->crc = c;

		break;

	case ELLIPSE_D_PARSE_STATE8_CRC2:
		msg->crc |= c<<8;
		uint16_t computedCrc;
		computedCrc = calcCRC(parserbuf, *parserbuf_index);
		if (msg->crc == computedCrc)
		{
			*state = ELLIPSE_D_PARSE_STATE9_ETX;
		}
		else
		{
			*state = ELLIPSE_D_PARSE_STATE0_SYNC1;
			return PX4_ERROR;
		}
//		*state = ELLIPSE_D_PARSE_STATE9_ETX;

		break;

	case ELLIPSE_D_PARSE_STATE9_ETX:
		*state = ELLIPSE_D_PARSE_STATE0_SYNC1;
		if (c==SBG_ECOM_ETX)
		{
			msg->status = MSG_COMPLETE;
		}

		break;
	}

	return ret;
}


/**
 * @brief Calculate CRC checksum for the specified buffer according to the polynom given in the IG500N protocol specification
 * @param pBuffer Pointer to the buffer
 * @param bufferSize Number of bytes in the buffer
 * @retval crc Calculated CRC checksum
 */
uint16_t calcCRC(const char *pBuffer, uint16_t bufferSize)
{
	uint16_t poly = 0x8408;
	uint16_t crc = 0;
	uint8_t carry;
	uint8_t i_bits;
	uint16_t j;
	for (j=0; j<bufferSize; j++)
	{
		crc = crc ^ pBuffer[j];
		for (i_bits=0; i_bits<8; i_bits++)
		{
			carry = crc & 1;
			crc = crc / 2;
			if (carry)
			{
				crc = crc^poly;
			}
		}
	}
	return crc;
}

int
ellipse_d_main(int argc, char *argv[])
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
			ellipse_d::start(argv[myoptind + 1]);

		} else {
			ellipse_d::start(ELLIPSE_D_DEFAULT_PORT);
		}
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[myoptind], "stop")) {
		ellipse_d::stop();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[myoptind], "test")) {
		ellipse_d::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[myoptind], "reset")) {
		ellipse_d::reset();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[myoptind], "info") || !strcmp(argv[1], "status")) {
		ellipse_d::info();
	}

	PX4_ERR("unrecognized command, try 'start', 'test', 'reset' or 'info'");
	return PX4_ERROR;
}
