/****************************************************************************
 *
 * Copyright 2020 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/
/****************************************************************************
 *
 *   Copyright (C) 2020 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <tinyara/config.h>

#include <sys/types.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <tinyara/timer.h>
#include <tinyara/irq.h>
#include "timer_api.h"


#if 0
#ifdef CONFIG_ARCH_IRQPRIO
#include <tinyara/arch.h>
#endif

#include "chip.h"
#include "amebad_gpt.h"
#endif

#ifdef CONFIG_TIMER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/
struct amebad_gpt_info_s {
	uint32_t	base;
	int		irq_id;
};

struct amebad_gpt_lowerhalf_s {
	const struct timer_ops_s	*ops;       /* Lowerhalf operations */
	gtimer_t					*obj;
	uint32_t					irq_id;
	struct amebad_gpt_info_s 	*gpt_info;
//	gpt_config_t				config;
	bool						started;   /* True: Timer has been started */
	tccb_t					callback;
	void						*arg;       /* Argument passed to upper half callback */
	uint32_t					gpt_timeout;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static int amebad_timer_handler(int irq, void *context, void *arg);
static int amebad_gpt_start(struct timer_lowerhalf_s *lower);
static int amebad_gpt_stop(struct timer_lowerhalf_s *lower);
static int amebad_gpt_getstatus(struct timer_lowerhalf_s *lower,
							struct timer_status_s *status);
static int amebad_gpt_settimeout(struct timer_lowerhalf_s *lower,
								uint32_t timeout);
static void amebad_gpt_setcallback(struct timer_lowerhalf_s *lower,
								CODE tccb_t callback, void *arg);
static int amebad_gpt_ioctl(struct timer_lowerhalf_s *lower, int cmd,
						unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/
static const struct timer_ops_s g_timer_ops = {
	.start			= amebad_gpt_start,
	.stop			= amebad_gpt_stop,
	.getstatus		= amebad_gpt_getstatus,
	.settimeout	= amebad_gpt_settimeout,
	.setcallback	= amebad_gpt_setcallback,
	.ioctl			= NULL,
};

static struct amebad_gpt_lowerhalf_s g_gpt0_lowerhalf = {
	.ops         = &g_timer_ops,
};

static struct amebad_gpt_lowerhalf_s g_gpt1_lowerhalf = {
	.ops         = &g_timer_ops,
};

static struct amebad_gpt_lowerhalf_s g_gpt2_lowerhalf = {
	.ops         = &g_timer_ops,
};

static struct amebad_gpt_lowerhalf_s g_gpt3_lowerhalf = {
	.ops         = &g_timer_ops,
};

static struct amebad_gpt_info_s g_gpt0_chipinfo = {
	.base   = TIM0,
	.irq_id = TIMER0_IRQ,
};

static struct amebad_gpt_info_s g_gpt1_chipinfo = {
	.base   = TIM1,
	.irq_id = TIMER1_IRQ,
};

static struct amebad_gpt_info_s g_gpt2_chipinfo = {
	.base   = TIM2,
	.irq_id = TIMER2_IRQ,
};

static struct amebad_gpt_info_s g_gpt3_chipinfo = {
	.base   = TIM3,
	.irq_id = TIMER3_IRQ,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/****************************************************************************
 * Name: amebad_gpt_handler
 *
 * Description:
 *   timer interrupt handler
 *
 * Input Parameters:
 *
 * Returned Values:
 *
 ****************************************************************************/
static int amebad_gpt_handler(int irq, void *context, void *arg)
{
	struct amebad_gpt_lowerhalf_s *priv = (struct amebad_gpt_lowerhalf_s *)arg;
	uint32_t next_interval_us = 0;
	printf("Enter amebad_gpt_handler\n");

	if (priv->callback(&next_interval_us, priv->arg)) {
		if (next_interval_us > 0) {			
			gtimer_reload (priv->obj, next_interval_us);
			priv->gpt_timeout = next_interval_us;
		}
	} else {
		amebad_gpt_stop(priv);
	}

	return OK;
}

/****************************************************************************
 * Name: amebad_gpt_start
 *
 * Description:
 *   Start the timer, resetting the time to the current timeout,
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/
static int amebad_gpt_start(struct timer_lowerhalf_s *lower)
{
	struct amebad_gpt_lowerhalf_s *priv = (struct amebad_gpt_lowerhalf_s *)lower;
	DEBUGASSERT(priv);
printf("Enter amebad_gpt_start\n");
	if (!priv->started) {
		gtimer_start (priv->obj);
		priv->started = true;

		tmrvdbg("Timer %d is started, callback %s\n", priv->obj->timer_id, priv->callback == NULL ? "none" : "set");
		return OK;
	}

	/* Return EBUSY to indicate that the timer was already running */
	tmrdbg("Error!! Timer %d was already running\n", priv->obj->timer_id);
	return -EBUSY;
}

/****************************************************************************
 * Name: amebad_gpt_stop
 *
 * Description:
 *   Stop the timer
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/
static int amebad_gpt_stop(struct timer_lowerhalf_s *lower)
{
	struct amebad_gpt_lowerhalf_s *priv = (struct amebad_gpt_lowerhalf_s *)lower;
printf("Enter amebad_gpt_stop\n");

	if (priv->started) {
		gtimer_stop (priv->obj);
		priv->started = false;

		tmrvdbg("Timer %d is stopped\n", priv->obj->timer_id);
		return OK;
	}

	/* Return ENODEV to indicate that the timer was not running */

	tmrdbg("Error!! Timer %d is not running\n", priv->obj->timer_id);
	return -ENODEV;
}

/****************************************************************************
 * Name: amebad_gpt_getstatus
 *
 * Description:
 *   get GPT status
 *
 * Input Parameters:
 *   lower   - A pointer the publicly visible representation of the
 *             "lower-half" driver state structure.
 *   status  - A pointer saved current GPT status
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/
static int amebad_gpt_getstatus(struct timer_lowerhalf_s *lower,
							struct timer_status_s *status)
{
	struct amebad_gpt_lowerhalf_s *priv = (struct amebad_gpt_lowerhalf_s *)lower;
	uint32_t ticks;
//	gpt_clock_source_t clock_source = priv->config.clockSource;
	printf("Enter amebad_gpt_getstatus\n");

	/* Return the status bit */
	
	status->flags = 0;
	if (priv->started) {
		status->flags |= TCFLAGS_ACTIVE;
	}

	if (priv->callback) {
		status->flags |= TCFLAGS_HANDLER;
	}
	
	/* Return the actual timeout in microseconds */
	ticks = gtimer_read_tick(priv->obj);
	status->timeout = ticks *1000000 /32768;
	
	printf("priv->gpt_timeout =%d \n",priv->gpt_timeout);
	printf("status->timeout =%d \n",status->timeout);
	
	/* Get the time remaining until the timer expires (in microseconds). */
	status->timeleft =  priv->gpt_timeout -status->timeout;
printf("priv->gpt_timeout = %d; status->timeout = %d \n",priv->gpt_timeout,status->timeout);

	return OK;
}

/****************************************************************************
 * Name: amebad_gpt_settimeout
 *
 * Description:
 *   Set a new timeout value (and reset the timer)
 *
 * Input Parameters:
 *   lower   - A pointer the publicly visible representation of the
 *             "lower-half" driver state structure.
 *   timeout - The new timeout value in microseconds.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/
static int amebad_gpt_settimeout(struct timer_lowerhalf_s *lower, uint32_t timeout)
{
	struct amebad_gpt_lowerhalf_s *priv = (struct amebad_gpt_lowerhalf_s *)lower;
	printf("Enter amebad_gpt_settimeout\n");
	printf("timeout=%d\n",timeout);

	if (priv->started) {
		return -EPERM;
	}

	gtimer_reload (priv->obj, timeout);
	priv->gpt_timeout = timeout;

	return OK;
}

/****************************************************************************
 * Name: amebad_gpt_setcallback
 *
 * Description:
 *   Call this user provided timeout callback.
 *
 * Input Parameters:
 *   lower    - A pointer the publicly visible representation of the
 *              "lower-half" driver state structure.
 *   callback - The new timer expiration function pointer. If this
 *              function pointer is NULL, then the reset-on-expiration
 *              behavior is restored,
 *  arg       - Argument that will be provided in the callback
 *
 * Returned Values:
 *   The previous timer expiration function pointer or NULL is there was
 *   no previous function pointer.
 *
 ****************************************************************************/
static void amebad_gpt_setcallback(struct timer_lowerhalf_s *lower, tccb_t callback, void *arg)
{
	struct amebad_gpt_lowerhalf_s *priv = (struct amebad_gpt_lowerhalf_s *)lower;
	irqstate_t flags = irqsave();

	/* Save the new callback */
	priv->callback = callback;
	priv->arg      = arg;

// If callback  function pointer is NULL, then the reset-on-expiration behavior is restored,
	if (callback != NULL && priv->started) {
		amebad_gpt_setisr(priv->gpt_info, amebad_gpt_handler, priv);
		gtimer_start_one_shout(priv->obj, priv->gpt_timeout, (void*)amebad_gpt_handler, priv);
	} else {
		amebad_gpt_stop(priv);
		amebad_gpt_setisr(priv->gpt_info, NULL, NULL);
	}

	irqrestore(flags);
}

#if 0
/****************************************************************************
 * Name: amebad_gpt_ioctl
 *
 * Description:
 *   Any ioctl commands that are not recognized by the "upper-half" driver
 *   are forwarded to the lower half driver through this method.
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *   cmd   - The ioctl command value
 *   arg   - The optional argument that accompanies the 'cmd'.  The
 *           interpretation of this argument depends on the particular
 *           command.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/
static int amebad_gpt_ioctl(struct timer_lowerhalf_s *lower, int cmd, unsigned long arg)
{
	struct amebad_gpt_lowerhalf_s *priv = (struct amebad_gpt_lowerhalf_s *)lower;
	int ret = -EINVAL;
	gpt_clock_source_t clock_source = priv->config.clockSource;

	tmrvdbg("GPT IOCTL cmd: %d, arg: %ld\n", cmd, arg);

	switch (cmd) {
	case TCIOC_SETDIV:
		if (clock_source == kGPT_ClockSource_Osc) {
			if (arg <= GPT_OSCDIV_MAXVAL) {
				imxrt_gpt_setoscclockdivider(priv->gpt->base, arg);
				priv->config.divider = arg;
				ret = OK;
			} else {
				tmrdbg("Invalid Value of OSC Divider (%d)\n", arg);
			}
		} else {
			if (arg <= GPT_DIV_MAXVAL) {
				amebad_gpt_setclockdivider(priv->gpt->base, arg);
				priv->config.divider = arg;
				ret = OK;
			} else {
				tmrdbg("Invalid Value of Divider (%d)\n", arg);
			}
		}
		break;
	case TCIOC_GETDIV:
		if (clock_source == kGPT_ClockSource_Osc) {
			ret = (int)amebad_gpt_getoscclockdivider(priv->gpt->base);
		} else {
			ret = (int)amebad_gpt_getclockdivider(priv->gpt->base);
		}
		break;
	case TCIOC_SETFREERUN:
		if ((bool)arg == true) {
			amebad_gpt_setfreerunmode(priv->gpt->base);
		} else {
			amebad_gpt_unsetfreerunmode(priv->gpt->base);
		}
		ret = OK;
		break;
#ifdef CONFIG_ARCH_IRQPRIO
	case TCIOC_SETIRQPRIO:
		ret = up_prioritize_irq(priv->gpt->irq_id, arg);
		break;
#endif
	case TCIOC_SETCLKSRC:
		if (arg > kGPT_ClockSource_Off && arg <= kGPT_ClockSource_Osc) {
			amebad_gpt_setclocksource(priv->gpt->base, (gpt_clock_source_t)arg);
			priv->config.clockSource = (gpt_clock_source_t)arg;
			ret = OK;
		}
		break;
	default:
		tmrdbg("Invalid cmd %d\n", cmd);
		break;
	}
	return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/
int amebad_timer_initialize(const char *devpath, int timer)
{
	struct amebad_gpt_lowerhalf_s *priv = NULL;
	priv->gpt_info = NULL;

	switch (timer) {
	case TIMER0:
		priv = &g_gpt0_lowerhalf;
		priv->gpt_info = &g_gpt0_chipinfo;
		break;

	case TIMER1:
		priv = &g_gpt1_lowerhalf;
		priv->gpt_info = &g_gpt1_chipinfo;
		break;

	case TIMER2:
		priv = &g_gpt2_lowerhalf;
		priv->gpt_info = &g_gpt2_chipinfo;
		break;

	case TIMER3:
		priv = &g_gpt3_lowerhalf;
		priv->gpt_info = &g_gpt3_chipinfo;
		break;
	}

	if (!priv) {
		return -ENODEV;
	}

	/* Initialize the elements of lower half state structure */

	priv->started  = false;
	priv->callback = NULL;
	gtimer_init(priv->obj, timer);
		
	if (priv->gpt_info == NULL) {
		return -EINVAL;
	}

	/*
	 * Register the timer driver as /dev/timerX.  The returned value from
	 * timer_register is a handle that could be used with timer_unregister().
	 * REVISIT: The returned handle is discard here.
	 */
	if (!timer_register(devpath, (struct timer_lowerhalf_s *)priv)) {
		/*
		 * The actual cause of the failure may have been a failure to allocate
		 * perhaps a failure to register the timer driver (such as if the
		 * 'depath' were not unique).  We know here but we return EEXIST to
		 * indicate the failure (implying the non-unique devpath).
		 */
		return -EEXIST;
	}

	return OK;
}

int amebad_gpt_setisr(struct amebad_gpt_info_s *priv, xcpt_t handler, void *arg)
{
	irq_attach(priv->irq_id, handler, arg);

	/* Enable SysTick interrupts */
	up_enable_irq(priv->irq_id);

	return OK;
}

#endif /* CONFIG_TIMER */

