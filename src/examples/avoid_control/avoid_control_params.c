/****************************************************************************
 *
 *   Copyright (C) 2008-2014 FORCE Technology. All rights reserved.
 *   Author: Rune Brogaard <ryb@force.dk>
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
 * @file avoid_control_params.c
 * 
 */

#include "avoid_control_params.h"

/* controller parameters */
PARAM_DEFINE_FLOAT(AVDC_S_P, 0.1f);
PARAM_DEFINE_FLOAT(AVDC_L_PITCH, 0.4f);
PARAM_DEFINE_FLOAT(AVDC_L_ROLL, 0.4f);

int parameters_init(struct avoid_control_param_handles *h)
{
	/* PID parameters */
	h->speed_p	 			=	param_find("AVDC_S_P");
	h->limit_pitch 			=	param_find("AVDC_L_PITCH");
	h->limit_roll 			=	param_find("AVDC_L_ROLL");
	h->trim_roll 			=	param_find("TRIM_ROLL");
	h->trim_pitch 			=	param_find("TRIM_PITCH");

	h->rc_scale_pitch    =   param_find("RC_SCALE_PITCH");
	h->rc_scale_roll    =   param_find("RC_SCALE_ROLL");
	h->rc_scale_yaw      =   param_find("RC_SCALE_YAW");


	return OK;
}

int parameters_update(const struct avoid_control_param_handles *h, struct avoid_control_params *p)
{
	param_get(h->speed_p, &(p->speed_p));
	param_get(h->limit_pitch, &(p->limit_pitch));
	param_get(h->limit_roll, &(p->limit_roll));
	param_get(h->trim_roll, &(p->trim_roll));
	param_get(h->trim_pitch, &(p->trim_pitch));

	param_get(h->rc_scale_pitch, &(p->rc_scale_pitch));
	param_get(h->rc_scale_roll, &(p->rc_scale_roll));
	param_get(h->rc_scale_yaw, &(p->rc_scale_yaw));

	return OK;
}
