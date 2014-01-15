/****************************************************************************
 *
 *   Copyright (C) 2012-2013 Rune Brogaard. All rights reserved.
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
 * @file danger.h
 *
 * Definition of danger topic
 */

#ifndef TOPIC_DANGER_H_
#define TOPIC_DANGER_H_

#include "../uORB.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @addtogroup topics
 * @{
 */

/**
 * Danger
 */
struct danger_s {
	uint64_t time_usec; //< Timestamp (UNIX) microseconds since system boot, needed to integrate
	float danger_dir; //< Direction of the danger in rad
	float avoid_dir; //< Avoid dirrection in rad
	uint16_t danger_dist; //< Distance to danger in mm
	uint8_t sensor_id; //< Sensor ID
	uint8_t danger_level; //< Level of danger
	uint8_t avoid_level; //< Avoid level/weight
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(danger);

#endif
