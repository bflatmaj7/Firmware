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

#include <drivers/drv_hrt.h>
#include <matrix/matrix/math.hpp>
#include <px4_module.h>
#include <px4_module_params.h>
#include <px4_workqueue.h>
#include <parameters/param.h>
#include <perf/perf_counter.h>
#include <uORB/topics/ins.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/mhp.h>
#include <uORB/topics/meteo.h>
#include <uORB/topics/wind_estimate.h>

#define SCHEDULE_INTERVAL	100000	/**< The schedule interval in usec (10 Hz) */

using matrix::Dcmf;
using matrix::Quatf;
using matrix::Vector2f;
using matrix::Vector3f;

class MhpWindModule : public ModuleBase<MhpWindModule>, public ModuleParams
{
public:

	MhpWindModule();

	~MhpWindModule();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	// run the main loop
	void cycle();

	int print_status() override;

private:
	static struct work_s	_work;

	orb_advert_t 	_wind_est_pub{nullptr};			/**< wind estimate topic */

	int _mhp_sub{-1};
	int _ins_sub{-1};
	int _meteo_sub{-1};
	int _param_sub{-1};

	perf_counter_t _perf_elapsed{};
	perf_counter_t _perf_interval{};

	static void	cycle_trampoline(void *arg);
	int 		start();

	void		update_params();

	bool 		subscribe_topics();
};

work_s	MhpWindModule::_work = {};

MhpWindModule::MhpWindModule():
	ModuleParams(nullptr)
{
	_ins_sub = orb_subscribe(ORB_ID(ins));
	_mhp_sub = orb_subscribe(ORB_ID(mhp));
	_meteo_sub = orb_subscribe(ORB_ID(meteo));
	_param_sub = orb_subscribe(ORB_ID(parameter_update));

	// initialise parameters
	update_params();

	_perf_elapsed = perf_alloc_once(PC_ELAPSED, "mhp_wind elapsed");
	_perf_interval = perf_alloc_once(PC_INTERVAL, "mhp_wind interval");
}

MhpWindModule::~MhpWindModule()
{
	orb_unsubscribe(_ins_sub);
	orb_unsubscribe(_mhp_sub);
	orb_unsubscribe(_meteo_sub);
	orb_unsubscribe(_param_sub);

	orb_unadvertise(_wind_est_pub);

	perf_free(_perf_elapsed);
	perf_free(_perf_interval);
}

int
MhpWindModule::task_spawn(int argc, char *argv[])
{
	/* schedule a cycle to start things */
	work_queue(LPWORK, &_work, (worker_t)&MhpWindModule::cycle_trampoline, nullptr, 0);

	// wait until task is up & running
	if (wait_until_running() < 0) {
		_task_id = -1;

	} else {
		_task_id = task_id_is_work_queue;
		return PX4_OK;
	}

	return PX4_ERROR;
}

void
MhpWindModule::cycle_trampoline(void *arg)
{
	MhpWindModule *dev = reinterpret_cast<MhpWindModule *>(arg);

	// check if the trampoline is called for the first time
	if (!dev) {
		dev = new MhpWindModule();

		if (!dev) {
			PX4_ERR("alloc failed");
			return;
		}

		_object = dev;
	}

	if (dev) {
		dev->cycle();
	}
}

void
MhpWindModule::cycle()
{
	perf_count(_perf_interval);
	perf_begin(_perf_elapsed);
	const float PI = 3.14159265;
	const float D2R = PI/180;
	const float R2D = 180/PI;

	bool param_updated;
	orb_check(_param_sub, &param_updated);

	if (param_updated) {
		update_params();
	}

	bool ins_valid = false;
	bool mhp_valid = false;
	bool meteo_valid = false;

	const hrt_abstime time_now_usec = hrt_absolute_time();

	// validate required conditions for the filter to fuse measurements

	ins_s ins = {};

	if (orb_copy(ORB_ID(ins), _ins_sub, &ins) == PX4_OK) {
		ins_valid = (time_now_usec - ins.timestamp < 1000 * 1000) && (ins.timestamp > 0);
	}

	mhp_s mhp = {};

	if (orb_copy(ORB_ID(mhp), _mhp_sub, &mhp) == PX4_OK) {
		mhp_valid = (time_now_usec - mhp.timestamp < 1000 * 1000) && (mhp.timestamp > 0);
	}

	meteo_s meteo = {};

	if (orb_copy(ORB_ID(meteo), _meteo_sub, &meteo) == PX4_OK) {
		meteo_valid = (time_now_usec - meteo.timestamp < 1000 * 1000) && (meteo.timestamp > 0);
	}


	// if all data is valid, publish wind measurement
	wind_estimate_s wind_est = {};
	if (ins_valid && mhp_valid && meteo_valid) {

		float droll = -0.3;  // in deg
		float dpitch = 1.92;  // in deg
		float dyaw = -1.26;  // in deg
		float dtas = 1.01;

		// correct the attitude angles and tas using the correction factors
		float tas = mhp.tas * dtas;
		float pitch = (ins.pitch + dpitch) * D2R ;
		float thead = (ins.yaw + dyaw) * D2R ;
		float roll = (ins.roll + droll) * D2R ;

		// The tas vector is calculated in x, y and z-direction (body-fixed
		// coordinate system using alpha and (corrected) beta
		float alpha = mhp.aoa * D2R;
		float beta = mhp.aos * D2R;
		float D = sqrt(1.0 +pow((tan(alpha)),2.0) + pow((tan(beta)),2.0) );
		D = 1 / D;
		float tas_bx = tas * D;
		float tas_by = tas * D * (float)tan(beta);
		float tas_bz = tas * D * (float)tan(alpha); // positive downwards!

		// The true airspeed vector is transformed from body-fixed
		// into geodetic coordinate system using the transformation matrix

		float tas_gx = ( tas_bx * ((float)cos(pitch) * (float)cos(thead))) +	( tas_by * ((float)sin(roll) * (float)sin(pitch) * (float)cos(thead) - (float)cos(roll) * (float)sin(thead))) + ( tas_bz * ((float)cos(roll) * (float)sin(pitch) * (float)cos(thead) + (float)sin(roll) * (float)sin(thead))) ;
		float tas_gy = ( tas_bx * ((float)cos(pitch) * (float)sin(thead))) +	( tas_by * ((float)sin(roll) * (float)sin(pitch) * (float)sin(thead) + (float)cos(roll) * (float)cos(thead))) + ( tas_bz * ((float)cos(roll) * (float)sin(pitch) * (float)sin(thead) - (float)sin(roll) * (float)cos(thead))) ;
		float tas_gz = ( tas_bx * (float)(-1.0 * sin(pitch))) + ( tas_by * ((float)sin(roll) * (float)cos(pitch))) + ( tas_bz * ((float)cos(roll) * (float)cos(pitch))) ;

		// the true airspeed components (geodetic coord.) are substracted from
		// the measured groundspeed components (geodetic coord.) to become
		// the meteorological wind vector.

		float u = ins.vew - tas_gy; // Positive in east direction
		float v = ins.vns - tas_gx; // Positive in north direction
		float w = -1 * (ins.vud - tas_gz); // Positive Upwards

		float ff = sqrt( (u*u) + (v*v) ); // windspeed
		float dd = (float)atan2(u, v) + PI;
//		if(v< 0)	dd = dd - PI;
//		dd = dd - 2*PI*(float)floor(dd/(2*PI)); // dd between 0 and 2* PI
		dd = dd * R2D; // in degree

		wind_est.timestamp = time_now_usec;
		wind_est.windspeed_north = v;
		wind_est.windspeed_east = u;
		wind_est.windspeed_up = w;
		wind_est.windspeed_abs = ff;
		wind_est.winddirection = dd;

		int instance;
		orb_publish_auto(ORB_ID(wind_estimate), &_wind_est_pub, &wind_est, &instance, ORB_PRIO_DEFAULT);
	}

//	wind_est.timestamp = time_now_usec;
//	wind_est.windspeed_north = 1;
//	wind_est.windspeed_east = 2;
//	wind_est.windspeed_up = 3;
//	wind_est.windspeed_abs = 4;
//	wind_est.winddirection = 5;
//
//	int instance;
//	orb_publish_auto(ORB_ID(wind_estimate), &_wind_est_pub, &wind_est, &instance, ORB_PRIO_DEFAULT);

	perf_end(_perf_elapsed);

	if (should_exit()) {
		exit_and_cleanup();

	} else {
		/* schedule next cycle */
		work_queue(LPWORK, &_work, (worker_t)&MhpWindModule::cycle_trampoline, this, USEC2TICK(SCHEDULE_INTERVAL));
	}
}

void MhpWindModule::update_params()
{
	updateParams();

}

int MhpWindModule::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		int ret = MhpWindModule::task_spawn(argc, argv);

		if (ret) {
			return ret;
		}
	}

	return print_usage("unknown command");
}

int MhpWindModule::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module runs a wind speed retrieval using multi-hole probe measurements of the flow angles and true airspeed as well
as precise attitude and GPS velocity measurements. True airspeed is measured using dynamic pressure, temperature and humidity measurements.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mhp_wind-measurement", "mhp_wind");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int MhpWindModule::print_status()
{
	perf_print_counter(_perf_elapsed);
	perf_print_counter(_perf_interval);

	return 0;
}

extern "C" __EXPORT int mhp_wind_measurement_main(int argc, char *argv[]);

int
mhp_wind_measurement_main(int argc, char *argv[])
{
	return MhpWindModule::main(argc, argv);
}
