/****************************************************************************
 *
 * Copyright 2021 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
 *
 ****************************************************************************/
#include <tinyara/config.h>
#include <stdlib.h>
#include <debug.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <errno.h>
#include <net/if.h>
#include <tinyara/lwnl/lwnl.h>
#include <tinyara/net/if/ble.h>
#include "ble_manager_event.h"
#include "ble_manager_msghandler.h"
#include "ble_manager_log.h"
#include "ble_manager_message.h"

/*  only ble msg handler thread can use this variable */
static blemgr_msg_s g_msg = {BLE_EVT_NONE, BLE_MANAGER_FAIL, NULL, NULL};

static inline void LWNL_SET_MSG(blemgr_msg_s *msg, blemgr_req_e event,
								ble_result_e result, void *param, sem_t *signal)
{
	msg->event = event;
	msg->result = result;
	msg->param = param;
	msg->signal = signal;
}

static void _lwnl_call_event(int fd, lwnl_cb_status status, int len)
{
	switch (status.evt) {
	case LWNL_EVT_BLE_CONNECTED:
		LWNL_SET_MSG(&g_msg, BLE_EVT_CONNECTED, BLE_MANAGER_FAIL, NULL, NULL);
		break;
	case LWNL_EVT_BLE_DISCONNECTED:
		LWNL_SET_MSG(&g_msg, BLE_EVT_DISCONNECTED, BLE_MANAGER_FAIL, NULL, NULL);
		break;
	case LWNL_EVT_BLE_SCAN_START:
		LWNL_SET_MSG(&g_msg, BLE_EVT_SCAN_STARTED, BLE_MANAGER_FAIL, NULL, NULL);
		break;
	case LWNL_EVT_BLE_SCAN_STOP:
		LWNL_SET_MSG(&g_msg, BLE_EVT_SCAN_STOPPED, BLE_MANAGER_FAIL, NULL, NULL);
		break;
	default:
		BLE_LOG_ERROR("Bad status received (%d)\n", status.evt);
		BLE_ERR;
		return;
	}
	return;
}

/**
 * Public
 */
int lwnl_fetch_ble_event(int fd, void *buf, int buflen)
{
	#define LWNL_CB_HEADER_LEN (sizeof(lwnl_cb_status) + sizeof(uint32_t))
	lwnl_cb_status status;
	uint32_t len;
	char type_buf[LWNL_CB_HEADER_LEN] = {0,};
	ble_handler_msg *hmsg = (ble_handler_msg *)buf;

	/*  lwnl guarantees that type_buf will read LWNL_CB_HEADER_LEN if it succeeds
	* So it doesn't need to consider partial read
	*/
	int nbytes = read(fd, (char *)type_buf, LWNL_CB_HEADER_LEN);
	if (nbytes < 0) {
		BLE_LOG_ERROR("Failed to receive (nbytes=%d)\n", nbytes);
		BLE_ERR;
		return -1;
	} else if (nbytes < LWNL_CB_HEADER_LEN) {
		BLE_LOG_ERROR("Invalid data received (nbytes=%d)\n", nbytes);
		BLE_ERR;
		return -2;
	}

	memcpy(&status, type_buf, sizeof(lwnl_cb_status));
	memcpy(&len, type_buf + sizeof(lwnl_cb_status), sizeof(uint32_t));

	BLE_LOG_VERBOSE("dev state(%d) length(%d)\n", status.evt, len);
	if (status.type == LWNL_DEV_BLE) {
		_lwnl_call_event(fd, status, len);
		hmsg->msg = &g_msg;
	} else {
		/* Remove wifi data in socket. */
		char *t_buf = (char *)malloc(len);
		nbytes = read(fd, (char *)t_buf, len);
		BLE_LOG_INFO("read unused data nbytes : %d / len : %d\n", nbytes, len);
		free(t_buf);
		hmsg->msg = NULL;
	}

	hmsg->signal = NULL;

	return 0;
}
