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
#include <stdint.h>
#include <errno.h>

#include <tinyara/kmalloc.h>
#include <tinyara/watchdog.h>
#include <tinyara/irq.h>

#include "wdt_api.h"
#include "rtl8721d_wdg.h"


#ifdef CONFIG_WATCHDOG

static int amebad_wdg_start(FAR struct watchdog_lowerhalf_s *lower);
static int amebad_wdg_stop(FAR struct watchdog_lowerhalf_s *lower);
static int amebad_wdg_keepalive(FAR struct watchdog_lowerhalf_s *lower);
static int amebad_wdg_getstatus(FAR struct watchdog_lowerhalf_s *lower, FAR struct watchdog_status_s *status);
static int amebad_wdg_settimeout(FAR struct watchdog_lowerhalf_s *lower, uint32_t timeout);
static xcpt_t amebad_wdg_capture(FAR struct watchdog_lowerhalf_s *lower, xcpt_t handler);
static int amebad_wdg_ioctl(FAR struct watchdog_lowerhalf_s *lower, int cmd, unsigned long arg);
void my_watchdog_irq_handler(uint32_t id);


const struct watchdog_ops_s g_wdgops = {
	.start = amebad_wdg_start,
	.stop = amebad_wdg_stop,
	.keepalive = amebad_wdg_keepalive,
	.getstatus = amebad_wdg_getstatus,
	.settimeout = amebad_wdg_settimeout,
	.capture = amebad_wdg_capture,
	.ioctl = amebad_wdg_ioctl,
};

/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * well-known watchdog_lowerhalf_s structure.
 */

struct amebad_wdg_lowerhalf_s {
	FAR const struct watchdog_ops_s *ops;
	u32 NewState;
	WDG_InitTypeDef WDG_InitStruct;
	u32 timeout_ms;
	bool enableInterrupt;
	xcpt_t handler;
	u32 statusflag; // 1 for wdg started, 0 for wdg stoped
};

void my_watchdog_irq_handler(uint32_t id) {

    printf("watchdog barks!! \n");

  //  watchdog_stop();

}

static int amebad_wdg_start(FAR struct watchdog_lowerhalf_s *lower)
{
	struct amebad_wdg_lowerhalf_s *priv = (struct amebad_wdg_lowerhalf_s *)lower;
	DEBUGASSERT(priv);
	printf("amebad_wdg_start \n");
	WDG_Cmd(ENABLE);
	priv->statusflag = 1;

	return OK;
}

static int amebad_wdg_stop(FAR struct watchdog_lowerhalf_s *lower)
{
	struct amebad_wdg_lowerhalf_s *priv = (struct amebad_wdg_lowerhalf_s *)lower;
	DEBUGASSERT(priv);
	printf("amebad_wdg_stop \n");

	WDG_Cmd(DISABLE);
	priv->statusflag = 0;

	return OK;
}

static int amebad_wdg_keepalive(FAR struct watchdog_lowerhalf_s *lower)
{
	struct amebad_wdg_lowerhalf_s *priv = (struct amebad_wdg_lowerhalf_s *)lower;
	DEBUGASSERT(priv);
	printf("amebad_wdg_keepalive \n");

	WDG_Refresh();

	return OK;
}

static int amebad_wdg_getstatus(FAR struct watchdog_lowerhalf_s *lower, FAR struct watchdog_status_s *status)
{
#if 0 	// reture watchdog flag, timeout and timeleft value
	
#define WDFLAGS_ACTIVE   (1 << 0)	/* 1=The watchdog timer is running */
#define WDFLAGS_RESET    (1 << 1)	/* 1=Reset when the watchog timer expires */
#define WDFLAGS_CAPTURE  (1 << 2)	/* 1=Call the user function when the watchdog timer expires */
#define WDFLAGS_WAIT     (1 << 3)	/* When watchdog is suspended */

	uint32_t flags;				/* See WDFLAGS_* definitions above */
	uint32_t timeout;			/* The current timeout setting (in milliseconds) */
	uint32_t timeleft;			/* Time left until the watchdog expiration
							 * (in milliseconds) */
#endif

	struct amebad_wdg_lowerhalf_s *priv = (struct amebad_wdg_lowerhalf_s *)lower;
	DEBUGASSERT(priv);
	printf("amebad_wdg_getstatus \n");

	status->flags = WDFLAGS_RESET;

	/* Return ACTIVE if watchdog timer is started */

	if (priv->statusflag == 1) {
		status->flags |= WDFLAGS_ACTIVE;
	}

	if (priv->handler) {
		status->flags |= WDFLAGS_CAPTURE;
	}

	if (priv->statusflag == 0) {
		status->flags |= WDFLAGS_WAIT;
	}
	
	/* Get timeout in ms */
	status->timeout = priv->timeout_ms;
	
	/* Get the timeleft before watchdog timeout in ms */
#if 0
	u32 CountId = priv->WDG_InitStruct.CountProcess;
	u16 DivFactor = priv->WDG_InitStruct.DivFacProcess;
	
	printf("CountId=%d \n",CountId);
	printf("DivFactor=%d \n",DivFactor);
	u32 Count = (0x00000001 << (CountId+1))-1; //Wdg timer remaining count
	u32 wdt_frequency  = 32768 / (priv->WDG_InitStruct.DivFacProcess + 1); 
	printf("Count=%d \n",Count);
	printf("wdt_frequency=%d \n",wdt_frequency);
	status->timeleft =  Count / wdt_frequency;
#endif
	printf("WDG_REG_BASE=%X \n",WDG_REG_BASE);
	u16 DivFactor =  WDG_REG_BASE & 0x0000FFFF;
	u8 CountId = (WDG_REG_BASE >> 25) & 0x00000001;
	printf("DivFactor=%X \n",DivFactor);
	printf("CountId=%X \n",CountId);
	u32 Count = (0x00000001 << (CountId+1))-1; //Wdg timer remaining count
	u16 CurrentTimeoutMs = DivFactor * (Count * 3) /100;
	printf("Count=%d\n",Count);
	printf("CurrentTimeoutMs=%d \n",CurrentTimeoutMs);	
	status->timeleft = status->timeout - CurrentTimeoutMs;
	u32 temp = 0;
	temp = HAL_READ32(SYSTEM_CTRL_BASE_LP, REG_LP_KM0_CTRL);
	printf("temp=%X \n",temp);	
	
	printf("status->timeleft=%d \n",status->timeleft);
	return OK;
}

static int amebad_wdg_settimeout(FAR struct watchdog_lowerhalf_s *lower, uint32_t timeout)
{
	struct amebad_wdg_lowerhalf_s *priv = (struct amebad_wdg_lowerhalf_s *)lower;
	DEBUGASSERT(priv);
	printf("amebad_wdg_settimeout \n");

	/* Watchdog Init */
	u32 CountProcess;
	u32 DivFacProcess;
	
	WDG_Scalar(timeout, &CountProcess, &DivFacProcess);

	priv->WDG_InitStruct.CountProcess = CountProcess;
	priv->WDG_InitStruct.DivFacProcess = DivFacProcess;

	WDG_Init(&priv ->WDG_InitStruct);

	/* Set to new timeout. */
	priv->timeout_ms = timeout;
	
	return OK;
}

/****************************************************************************
 * Name: amebad_wdg_capture
 *
 * Description:
 *   Don't reset on watchdog timer timeout; instead, call this user provider
 *   timeout handler.  NOTE:  Providing handler==NULL will restore the reset
 *   behavior.
 *
 * Input Parameters:
 *   lower      - A pointer the publicly visible representation of the "lower-half"
 *                driver state structure.
 *   newhandler - The new watchdog expiration function pointer.  If this
 *                function pointer is NULL, then the reset-on-expiration
 *                behavior is restored,
 *
 * Returned Values:
 *   The previous watchdog expiration function pointer or NULL is there was
 *   no previous function pointer, i.e., if the previous behavior was
 *   reset-on-expiration (NULL is also returned if an error occurs).
 *
 ****************************************************************************/

static xcpt_t amebad_wdg_capture(FAR struct watchdog_lowerhalf_s *lower, xcpt_t handler)
{
	struct amebad_wdg_lowerhalf_s *priv = (struct amebad_wdg_lowerhalf_s *)lower;
	xcpt_t oldhandler;

	DEBUGASSERT(priv);
	printf("amebad_wdg_capture \n");

	oldhandler = priv->handler;
	priv->enableInterrupt = true;

	/* Set the new handler. */
	priv->handler = handler;
	printf("handler = %d \n",&handler);
	printf("handler = %x \n",&handler);
	if (handler) {
		printf("Before watchdog_irq_init \n");
		watchdog_irq_init((void *)handler, 0);
	//	watchdog_irq_init(my_watchdog_irq_handler, 0);
	//	WDG_IrqInit(my_watchdog_irq_handler, 0);//irq_attach( ) is in InterruptRegister( )
		} else {
			WDG_IrqClear();
		}
	return oldhandler;
}

static int amebad_wdg_ioctl(FAR struct watchdog_lowerhalf_s *lower, int cmd, unsigned long arg)
{
	struct amebad_wdg_lowerhalf_s *priv = (struct amebad_wdg_lowerhalf_s *)lower;
	DEBUGASSERT(priv);
	printf("amebad_wdg_ioctl \n");

#if 0
	switch (cmd) {

		case WDIOC_START: {
			printf("WDIOC_START \n");
			amebad_wdg_start (priv);
		}
		break;

		case WDIOC_STOP: {
			printf("WDIOC_STOP \n");
			amebad_wdg_stop (priv);
		}
		break;
		
		case WDIOC_GETSTATUS: {
			printf("WDIOC_GETSTATUS \n");
			amebad_wdg_getstatus (priv, arg);
		}
		break;

		case WDIOC_SETTIMEOUT: {
			printf("WDIOC_SETTIMEOUT \n");
			amebad_wdg_settimeout (priv, arg);
		}
		break;

		case WDIOC_CAPTURE: {
			printf("WDIOC_CAPTURE \n");
			amebad_wdg_capture (priv, arg);
		}
		break;

		default: {
			printf("cmd = %d \n",cmd);
			llvdbg("invalid cmd for wdg ioctl.\n");
		}
		break;
	}
#endif

	return OK;
}

/****************************************************************************
 * Name: amebad_wdg_initialize
 *
 * Description:
 *   Initialize and register the amebad watchdog. Watchdog timer is registered
 *   as 'devpath'.
 *
 ****************************************************************************/
int amebad_wdg_initialize(const char *devpath, uint32_t timeout_ms)
{
	struct amebad_wdg_lowerhalf_s *priv = NULL;
	priv = (struct amebad_wdg_lowerhalf_s *)kmm_malloc(sizeof(struct amebad_wdg_lowerhalf_s));
printf("amebad_wdg_initialize \n");
	if (!priv) {
		llvdbg("wdg_lowerhalf_s allocation error.\n");
		return -ENOMEM;
	}

	/* Set the priv to default */
	priv->ops = &g_wdgops;
	priv->handler = NULL;

	/* Watchdog Init */
	u32 CountProcess;
	u32 DivFacProcess;
	
	WDG_Scalar(timeout_ms, &CountProcess, &DivFacProcess);

	priv->WDG_InitStruct.CountProcess = CountProcess;
	priv->WDG_InitStruct.DivFacProcess = DivFacProcess;

	WDG_Init(&priv ->WDG_InitStruct);

	/* Register the watchdog driver as /dev/wdgX. */
	if (!watchdog_register(devpath, (FAR struct watchdog_lowerhalf_s *)priv)) {
		/*
		 * The actual cause of the failure may have been a failure to allocate
		 * perhaps a failure to register the timer driver (such as if the
		 * 'depath' were not unique).  We know here but we return EEXIST to
		 * indicate the failure (implying the non-unique devpath).
		 */
		kmm_free(priv);
		return -EEXIST;
	}

	return OK;
}
#endif //CONFIG_WATCHDOG
