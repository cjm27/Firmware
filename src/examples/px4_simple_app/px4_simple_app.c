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
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/adc_report.h>


#include "params.h"


__EXPORT int px4_simple_app_main(int argc, char *argv[]);

int px4_simple_app_main(int argc, char *argv[])
{
	PX4_INFO("FOIL");

	float param = 0;
	param_get(param_find("FOIL_AIL_P"), &param);
	PX4_INFO("PARAM:t%8.4f", (double)param);

	// /* subscribe to sensor_combined topic */
	// int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
	// /* limit the update rate to 5 Hz */
	// orb_set_interval(sensor_sub_fd, 200);

	// //subscribe to global position
	// int global_position_fd = orb_subscribe(ORB_ID(vehicle_global_position));
	// orb_set_interval(global_position_fd, 200);

	//subscribe to RC
	// int input_rc_fd = orb_subscribe(ORB_ID(input_rc));
	// orb_set_interval(input_rc_fd, 200);

	//subscribe to manual control setpoint
	int manual_control_setpoint_fd = orb_subscribe(ORB_ID(manual_control_setpoint));
	orb_set_interval(manual_control_setpoint_fd, 200);

	//Advertise Manual Control Setpoint
	// struct manual_control_setpoint_s manual_control;
	// memset(&manual_control, 0, sizeof(manual_control));
	// orb_advert_t manual_control_pub = orb_advertise(ORB_ID(manual_control_setpoint), &manual_control);

	//ADC
	int adc_sub_fd = orb_subscribe(ORB_ID(adc_report));
	orb_set_interval(adc_sub_fd, 200);

	// struct adc_report_s adc;
	// memset(&adc, 0, sizeof(adc));
	// orb_advert_t adc_pub = orb_advertise(ORB_ID(adc_report), &adc);

	//advertise RC input topic
	// struct input_rc_s input;
	// memset(&input, 0, sizeof(input));
	// orb_advert_t input_pub = orb_advertise(ORB_ID(input_rc), &input);

	/* advertise attitude topic */
	// struct vehicle_attitude_s att;
	// memset(&att, 0, sizeof(att));
	// orb_advert_t att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);
	
	// //baro sensor
	// struct sensor_baro_s baro;
	// memset(&baro, 0, sizeof(baro));
	// orb_advert_t baro_pub = orb_advertise(ORB_ID(sensor_baro), &baro);

	// struct vehicle_global_position_s pos;
	// memset(&pos, 0, sizeof(pos));
	// orb_advert_t pos_pub = orb_advertise(ORB_ID(vehicle_global_position), &pos);


	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {
		{ .fd = manual_control_setpoint_fd,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};

	int error_counter = 0;

	for (int i = 0; i < 10; i++) {
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = px4_poll(fds, 1, 1000);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a second");

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {

			if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				//struct sensor_combined_s raw;
				/* copy sensors raw data into local buffer */
				//orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
				// PX4_INFO("Accelerometer:\t%8.4f\t%8.4f\t%8.4f",
				// 	 (double)raw.accelerometer_m_s2[0],
				// 	 (double)raw.accelerometer_m_s2[1],
				// 	 (double)raw.accelerometer_m_s2[2]);


				/* set att and publish this information for other apps
				 the following does not have any meaning, it's just an example
				*/
				// att.q[0] = raw.accelerometer_m_s2[0];
				// att.q[1] = raw.accelerometer_m_s2[1];
				// att.q[2] = raw.accelerometer_m_s2[2];

				// orb_publish(ORB_ID(vehicle_attitude), att_pub, &att);
				

				
				// struct vehicle_global_position_s rawPos;
				// orb_copy(ORB_ID(vehicle_global_position), global_position_fd, &rawPos);
				// // PX4_INFO("Altitude:\t%8.4f",
				// // 	 (double)rawPos.alt);

				struct manual_control_setpoint_s raw_manual_control_setpoint;
				orb_copy(ORB_ID(manual_control_setpoint), manual_control_setpoint_fd, &raw_manual_control_setpoint);
				// PX4_INFO("MANUAL:\t%8.4f\t%8.4f\t%8.4f\t%8.4f",
				// 	 (double)raw_manual_control_setpoint.x,
				// 	 (double)raw_manual_control_setpoint.y,
				// 	 (double)raw_manual_control_setpoint.z,
				// 	 (double)raw_manual_control_setpoint.r);


				//manual_control.x = i * 0.01;
				//orb_publish(ORB_ID(manual_control_setpoint), manual_control_pub, &manual_control);

				struct adc_report_s raw_adc;
				orb_copy(ORB_ID(adc_report), adc_sub_fd, &raw_adc);
				PX4_INFO("MANUAL:\t%8.4f",
					 (double)raw_adc.channel_value[4]);

				// struct input_rc_s raw_input;
				// orb_copy(ORB_ID(input_rc), input_rc_fd, &raw_input);
				// // PX4_INFO("RC:\t%8.4f\t%8.4f\t%8.4f",
				// // 	 (double)raw_input.values[0],
				// // 	 (double)raw_input.values[1],
				// // 	 (double)raw_input.values[2]);

				// input.values[0] = i;
				// orb_publish(ORB_ID(input_rc), input_pub, &input);


				// baro.pressure = i;
				// orb_publish(ORB_ID(sensor_baro), baro_pub, &baro);

				// pos.alt = rawPos.alt + (float)1.0;
				// orb_publish(ORB_ID(vehicle_global_position), pos_pub, &pos);
				
			}

			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */
		}
	}

	PX4_INFO("exiting");

	return 0;
}
