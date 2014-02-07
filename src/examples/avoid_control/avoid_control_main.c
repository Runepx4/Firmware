/****************************************************************************
 *
 *   Copyright (c)  2013 PX4 Development Team. All rights reserved.
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
 * @file avoid_control_main.c
 * Sense & Avoid application example for PX4 autopilot
 *
 * @author Rune Brogaard <ryb@force.dk>
 */

#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <mavlink/mavlink_log.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>

#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/danger.h> //RB topic

#include "avoid_control_params.h"

static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */

/**
 * daemon management function.
 */
__EXPORT int avoid_control_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int avoid_control_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason)
		warnx("%s\n", reason);
	errx(1, "usage: avoid_control {start|stop|status} [-p <additional params>]\n\n");
}

/**
 * The avoid_control app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int avoid_control_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("avoid_control already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		daemon_task = task_spawn_cmd("avoid_control",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_DEFAULT,
					 4096,
					 avoid_control_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");
		} else {
			warnx("\tnot started\n");
		}
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}


int avoid_control_thread_main(int argc, char *argv[]) {

	warnx("[avoid_control] starting\n");

	thread_running = true;
	static int mavlink_fd;
	mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
	mavlink_log_info(mavlink_fd, "[avoid_control] started");

	/* structures */
	struct vehicle_control_mode_s control_mode;
	memset(&control_mode, 0, sizeof(control_mode));
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	struct vehicle_attitude_setpoint_s att_sp;
	memset(&att_sp, 0, sizeof(att_sp));
	struct manual_control_setpoint_s manual;
	memset(&manual, 0, sizeof(manual));
	struct danger_s dan;
	memset(&dan, 0, sizeof(dan));

	/* subscribe to danger message */
	int danger_sub = orb_subscribe(ORB_ID(danger));

	/* subscribe to attitude, motor setpoints and system state */
	int control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	int manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	/* subscribe to parameter message */
	int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));

	/* publish setpoint */
	orb_advert_t att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &att_sp);

	/* parameters init*/
	struct avoid_control_params params;
	struct avoid_control_param_handles param_handles;
	parameters_init(&param_handles);
	parameters_update(&param_handles, &params);

	/* register the perf counter */
	perf_counter_t mc_err_perf = perf_alloc(PC_COUNT, "avoid_control_err");
	perf_counter_t mc_loop_perf = perf_alloc(PC_ELAPSED, "avoid_control_runtime");
	perf_counter_t mc_interval_perf = perf_alloc(PC_INTERVAL, "avoid_control_interval");

	static bool sensors_ready = false;
	static bool status_changed = false;
	static bool commincation_lost = false;


	while (!thread_should_exit) {

		/* wait for first attitude msg to be sure all data are available */
		if (sensors_ready)
		{
			/* polling */
			struct pollfd fds[2] = {
				{ .fd = danger_sub, .events = POLLIN }, // danger message from onboard PC
				{ .fd = parameter_update_sub,   .events = POLLIN }
				};

			/* wait for a danger message update, check for exit condition every 500 ms */
			int ret = poll(fds, 2, 300);

			if (ret < 0)
			{
				/* poll error, count it in perf */
				perf_count(mc_err_perf);
			}
			else if (ret == 0)
			{
				if (!commincation_lost)
				{
					mavlink_log_info(mavlink_fd,"[AvoidCtrl] communication lost!");
					commincation_lost = true;
				}
				/* no return value */
				//Change to manual control in case onboard communication with the linux_pc fails:
				bool updated; //bool used to check if new information has been published to a subscription
				//if RC inputs changed, get local copy of rc-inputs into the local buffer manual:
				orb_check(manual_sub, &updated);
				if (updated) orb_copy(ORB_ID(manual_control_setpoint), manual_sub, &manual);

				att_sp.pitch_body = manual.pitch;
				att_sp.roll_body = manual.roll;
				att_sp.yaw_body = manual.yaw;
				att_sp.thrust = manual.throttle;
				att_sp.timestamp = hrt_absolute_time();

			}
			else
			{
				/* parameter update available? */
				if (fds[1].revents & POLLIN)
				{
					/* read from param to clear updated flag */
					struct parameter_update_s update;
					orb_copy(ORB_ID(parameter_update), parameter_update_sub, &update);

					parameters_update(&param_handles, &params);
					mavlink_log_info(mavlink_fd,"[AvoidCtrl] parameters updated.");
				}

				/* only run controller if "danger" changed */
				if (fds[0].revents & POLLIN)
				{
					perf_begin(mc_loop_perf);
					// obtained data for the first file descriptor
					struct danger_s dan;
					// copy danger data into local buffer
					orb_copy(ORB_ID(danger), danger_sub, &dan);

					if (commincation_lost)
					{
						mavlink_log_info(mavlink_fd,"[AvoidCtrl] communication restored..");
						commincation_lost = false;
					}

					/* Do not use data from the laserscanner if the scanner is not within the horizontal limits */
					if ((att.pitch > params.limit_pitch) && (att.roll > params.limit_roll)) {
						 dan.avoid_dir = 0.0;
						 dan.avoid_level = 0;
						 dan.danger_dir = 0;
						 dan.danger_dist = 4000;
						 dan.danger_level = 0;
					 }

					bool updated; //bool used to check if new information has been published to a subscription

					/* if control_mode changed, get a local copy of the control mode */
					orb_check(control_mode_sub, &updated);
					if (updated) orb_copy(ORB_ID(vehicle_control_mode), control_mode_sub, &control_mode);

					//if attitude setpoint changed, get local copy of attitude setpoint into the local buffer att_sp:
					orb_check(att_sp_sub, &updated);
					if (updated) orb_copy(ORB_ID(vehicle_attitude_setpoint), att_sp_sub, &att_sp);

					//if RC inputs changed, get local copy of rc-inputs into the local buffer manual:
					orb_check(manual_sub, &updated);
					if (updated) orb_copy(ORB_ID(manual_control_setpoint), manual_sub, &manual);

					//if attitude changed, get local copy of attitude into the local buffer att:
					orb_check(att_sub, &updated);
					if (updated) orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);

					if (!control_mode.flag_control_manual_enabled)
					{
						/* limit roll and pitch corrections */
						float pitch_corr = -(sinf(dan.avoid_dir) * ((float)dan.avoid_level/255.0)) * params.limit_pitch;
						float roll_corr = (cosf(dan.avoid_dir) * ((float)dan.avoid_level/255.0)) * params.limit_roll;

						//float pitch_sp_ctl = manual.pitch / params.rc_scale_pitch;// [-1..0..1]
						//float roll_sp_ctl = manual.roll / params.rc_scale_roll; // [-1..0..1]

						float pitch_body = pitch_corr + manual.pitch;
						float roll_body  = roll_corr + manual.roll;

						/* limit roll and pitch outputs */
						if((pitch_body <= params.rc_scale_pitch) && (pitch_body >= -params.rc_scale_pitch))
							{
							att_sp.pitch_body = pitch_body;
							}
						else
						{
							if(pitch_body > params.rc_scale_pitch) att_sp.pitch_body = params.rc_scale_pitch;
							if(pitch_body < -params.rc_scale_pitch)	att_sp.pitch_body = -params.rc_scale_pitch;
						}
						if((roll_body <= params.rc_scale_roll) && (roll_body >= -params.rc_scale_roll))
						{
							att_sp.roll_body = roll_body;
						}
						else
						{
							if(roll_body > params.rc_scale_roll) att_sp.roll_body = params.rc_scale_roll;
							if(roll_body < -params.rc_scale_roll) att_sp.roll_body = -params.rc_scale_roll;
						}

						//att_sp.yaw_body = dan.avoid_dir;
						att_sp.yaw_body = manual.yaw;
						att_sp.thrust = manual.throttle;
						att_sp.timestamp = hrt_absolute_time();

					}

					/* measure in what intervals the controller runs */
					perf_count(mc_interval_perf);
					perf_end(mc_loop_perf);
				}

			}

			// publish new attitude setpoint
			orb_publish(ORB_ID(vehicle_attitude_setpoint), att_sp_pub, &att_sp);
		}
		else
		{
			/* sensors not ready waiting for first attitude msg */

			/* polling */
			struct pollfd fds[1] = {
				{ .fd = danger_sub, .events = POLLIN },
			};

			/* wait for a attitude msg, check for exit condition every 5 s */
			int ret = poll(fds, 1, 5000);

			if (ret < 0)
			{
				/* poll error, count it in perf */
				perf_count(mc_err_perf);
			}
			else if (ret == 0)
			{
				/* no return value, ignore */
				mavlink_log_info(mavlink_fd,"[AvoidCtrl] wating for first danger msg...");
			}
			else
			{
				if (fds[0].revents & POLLIN)
				{
					sensors_ready = true;
					mavlink_log_info(mavlink_fd,"[AvoidCtrl] initialized.");
				}
			}
		}

	}

	warnx("[avoid_control] exiting.\n");
	mavlink_log_info(mavlink_fd,"[AvoidCtrl] ending now...");

	thread_running = false;
	close(parameter_update_sub);
	close(att_sub);
	close(danger_sub);
	close(control_mode_sub);
	close(att_sp_pub);

	perf_print_counter(mc_loop_perf);
	perf_free(mc_loop_perf);

	fflush(stdout);
	return 0;
}
