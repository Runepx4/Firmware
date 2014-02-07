/****************************************************************************
 *
 *   Copyright (C) 2008-2014 FORCE Technology. All rights reserved.
 *   Author: Rune Brogaard <ryb@force.dk>
 *
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

/*
 * @file avoid_control_params.h
 * 
 * Parameters for avoid controller
 */

#include <systemlib/param/param.h>

struct avoid_control_params {
	float speed_p;
	float limit_pitch;
	float limit_roll;
	float trim_roll;
	float trim_pitch;

	float rc_scale_pitch;
	float rc_scale_roll;
	float rc_scale_yaw;
};

struct avoid_control_param_handles {
	param_t speed_p;
	param_t limit_pitch;
	param_t limit_roll;
	param_t trim_roll;
	param_t trim_pitch;

	param_t rc_scale_pitch;
	param_t rc_scale_roll;
	param_t rc_scale_yaw;
};

/**
 * Initialize all parameter handles and values
 *
 */
int parameters_init(struct avoid_control_param_handles *h);

/**
 * Update all parameters
 *
 */
int parameters_update(const struct avoid_control_param_handles *h, struct avoid_control_params *p);
