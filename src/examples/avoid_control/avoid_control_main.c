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

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/danger.h> //RB topic


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
	orb_set_interval(danger_sub, 500);

	/* subscribe to attitude, motor setpoints and system state */
	int control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	int manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));


	/* publish setpoint */
	orb_advert_t att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &att_sp);

	/* subscribe to sensor_combined topic */
	//int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
	//orb_set_interval(sensor_sub_fd, 500);


	while (!thread_should_exit) {

			int error_counter = 0;

			/* one could wait for multiple topics with this technique, just using one here */
			struct pollfd fds[] = {
				{ .fd = danger_sub,   .events = POLLIN },
				/* there could be more file descriptors here, in the form like:
				 * { .fd = other_sub_fd,   .events = POLLIN },
				 */
			};

			for (int i = 0; i < 10; i++) {
				/* wait for danger update of 1 file descriptor for 500 ms (0.5 second) */
				int poll_ret = poll(fds, 1, 500);

				/* handle the poll result */
				if (poll_ret == 0) {
					/* this means none of our providers is giving us data */
					printf("[avoid_control] Got no data within a second\n");
				} else if (poll_ret < 0) {
					/* this is seriously bad - should be an emergency */
					if (error_counter < 10 || error_counter % 50 == 0) {
						/* use a counter to prevent flooding (and slowing us down) */
						printf("[avoid_control] ERROR return value from poll(): %d\n"
							, poll_ret);
					}
					error_counter++;
				} else {

					if (fds[0].revents & POLLIN) {
						/* obtained data for the first file descriptor */
						struct danger_s dan;
						/* copy danger data into local buffer */
						orb_copy(ORB_ID(danger), danger_sub, &dan);
						printf("[avoid_control] Danger:\t%8.4f\t%8.4f\t%8.4f\n",
							(double)dan.sensor_id,
							(double)dan.avoid_dir,
							(double)dan.danger_level);

						//Copy attitude setpoint into local buffer:
						orb_copy(ORB_ID(vehicle_attitude_setpoint), att_sp_sub, &att_sp);

						att_sp.yaw_body = dan.avoid_dir;
						att_sp.thrust = i/10;

						/* publish new attitude setpoint */
						orb_publish(ORB_ID(vehicle_attitude_setpoint), att_sp_pub, &att_sp);
					}
					/* there could be more file descriptors here, in the form like:
					 * if (fds[1..n].revents & POLLIN) {}
					 */
				}
			}

		sleep(10);
	}

	warnx("[avoid_control] exiting.\n");

	thread_running = false;

	return 0;
}
