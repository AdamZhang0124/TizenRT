/****************************************************************************
 *
 * Copyright 2017 Samsung Electronics All Rights Reserved.
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

/// @file tc_watchdog.c
/// @brief Test Case Example for watchdog driver
#include <tinyara/watchdog.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include "tc_internal.h"
#include <errno.h>

/**
 * @brief Closes all open files
 */
static inline void close_fds(int *fds, int count)
{
	for (; count >= 0; count--) {
		close(fds[count]);
	}
}

/**
* @fn                   :tc_driver_watchdog_open_close
* @brief                :Test the watchdog driver open and close
* @scenario             :Test the watchdog driver open and close
* API's covered         :open, close
* Preconditions         :none
* Postconditions        :none
* @return               :void
*/
static void tc_driver_watchdog_open_close(void)
{
	int fd = 0;
	int ret = 0;
printf("OC 1\n");
	fd = open("/dev/watchdog0", O_RDWR);
	TC_ASSERT_GT("watchdog_open", fd, 0);
printf("OC 2\n");
	ret = close(fd);
	TC_ASSERT_EQ("watchdog_close", ret, OK);
printf("OC 3\n");
	/* Negative test cases */
	int count = 0;
	int fds[255] = { 0, };
	for (count = 0; count < 255; count++) {
		fds[count] = open("/dev/watchdog0", O_RDWR);
		if (fds[count] < 0) {
			TC_ASSERT_EQ_CLEANUP("watchdog_open", errno, EMFILE, close_fds(fds, --count));
			break;
		}
	}
printf("OC 4\n");
	close_fds(fds, --count);
printf("OC 5\n");
	TC_SUCCESS_RESULT();
}

/**
* @fn                   :tc_driver_watchdog_read_write
* @brief                :Test the watchdog driver read and write
* @scenario             :Test the watchdog driver read and write
* API's covered         :read, write
* Preconditions         :none
* Postconditions        :none
* @return               :void
*/
static void tc_driver_watchdog_read_write(void)
{
	int fd = 0;
	int ret = 0;
printf("WR 1\n");
	fd = open("/dev/watchdog0", O_RDWR);
	TC_ASSERT_GT("watchdog_open", fd, 0);
printf("WR 2\n");
	ret = read(fd, NULL, 0);
	TC_ASSERT_EQ_CLEANUP("watchdog_read", ret, 0, close(fd));
printf("WR 3\n");
	ret = write(fd, NULL, 1);
	TC_ASSERT_EQ_CLEANUP("watchdog_write", ret, 0, close(fd));
printf("WR 4\n");
	close(fd);
printf("WR 5\n");
	TC_SUCCESS_RESULT();
}

/**
* @fn                   :tc_driver_watchdog_ioctl
* @brief                :Test the watchdog driver ioctl
* @scenario             :Test the watchdog driver ioctl
* API's covered         :ioctl
* Preconditions         :none
* Postconditions        :none
* @return               :void
*/
static void tc_driver_watchdog_ioctl(void)
{
	int fd = 0;
	int ret = 0;
printf("iotcl 1\n");
	fd = open("/dev/watchdog0", O_RDWR);
	TC_ASSERT_GT("watchdog_open", fd, 0);
printf("iotcl 2\n");
	ret = ioctl(fd, WDIOC_SETTIMEOUT, 1000);
	TC_ASSERT_EQ_CLEANUP("watchdog_ioctl", ret, OK, close(fd));
printf("iotcl 3 WDIOC_START=%d\n",WDIOC_START);
	ret = ioctl(fd, WDIOC_START, 0);
	TC_ASSERT_EQ_CLEANUP("watchdog_ioctl", ret, OK, close(fd));
printf("iotcl 4 WDIOC_GETSTATUS=%d\n",WDIOC_GETSTATUS);
	FAR struct watchdog_status_s status;
	ret = ioctl(fd, WDIOC_GETSTATUS, (unsigned long)&status);
	TC_ASSERT_EQ_CLEANUP("watchdog_ioctl", ret, OK, close(fd));
	TC_ASSERT_EQ_CLEANUP("watchdog_ioctl", status.flags & WDFLAGS_ACTIVE, WDFLAGS_ACTIVE, close(fd));
	TC_ASSERT_EQ_CLEANUP("watchdog_ioctl", status.timeout, 1000, close(fd));
printf("iotcl 5 WDIOC_GETSTATUS=%d\n",WDIOC_GETSTATUS);
	/* Negative testcase for WDIOC_GETSTATUS */
	ret = ioctl(fd, WDIOC_GETSTATUS, 0UL);
	TC_ASSERT_LT_CLEANUP("watchdog_ioctl", ret, 0, close(fd));
printf("iotcl 6 WDIOC_CAPTURE\n");
	FAR struct watchdog_capture_s cap;
	ret = ioctl(fd, WDIOC_CAPTURE, (unsigned long)&cap);
	TC_ASSERT_EQ_CLEANUP("watchdog_ioctl", ret, OK, close(fd));
printf("iotcl 7 WDIOC_CAPTURE\n");
	/* Negative testcase for WDIOC_CAPTURE */
	ret = ioctl(fd, WDIOC_CAPTURE, 0UL);
	TC_ASSERT_LT_CLEANUP("watchdog_ioctl", ret, 0, close(fd));
printf("iotcl 8 WDIOC_KEEPALIVE\n");
	ret = ioctl(fd, WDIOC_KEEPALIVE, 0UL);
	TC_ASSERT_EQ_CLEANUP("watchdog_ioctl", ret, OK, close(fd));
printf("iotcl 9 WDIOC_STOP\n");
	ret = ioctl(fd, WDIOC_STOP, 0UL);
	TC_ASSERT_EQ_CLEANUP("watchdog_ioctl", ret, OK, close(fd));
printf("iotcl 10 -1\n");
	ret = ioctl(fd, -1, 0UL);
	TC_ASSERT_EQ_CLEANUP("watchdog_ioctl", ret, OK, close(fd));
printf("iotcl 11\n");
	close(fd);
printf("iotcl 12\n");
	TC_SUCCESS_RESULT();
}

xcpt_t  test_handler(int irq, FAR void *context, FAR void *arg)
{
	printf("Enter test_handler!!!!!!!!!!!!!!!!!!!!\n");
	return OK;
}

static void tc_amebad_watchdog(void)
{
printf("tc_amebad_watchdog\n");
	int fd = 0;
	int ret = 0;
printf("iotcl 1 open\n");
	fd = open("/dev/watchdog0", O_RDWR);
	TC_ASSERT_GT("watchdog_open", fd, 0);
#if 0
printf("iotcl 2\n");
	ret = ioctl(fd, WDIOC_SETTIMEOUT, 15000);
	TC_ASSERT_EQ_CLEANUP("watchdog_ioctl", ret, OK, close(fd));
printf("iotcl 3 WDIOC_START=%d\n",WDIOC_START);
	ret = ioctl(fd, WDIOC_START, 0);
	TC_ASSERT_EQ_CLEANUP("watchdog_ioctl", ret, OK, close(fd));
printf("iotcl 4 WDIOC_GETSTATUS=%d\n",WDIOC_GETSTATUS);
	FAR struct watchdog_status_s status;
	ret = ioctl(fd, WDIOC_GETSTATUS, (unsigned long)&status);
	TC_ASSERT_EQ_CLEANUP("watchdog_ioctl", ret, OK, close(fd));
wait_ms(1000);
	ret = ioctl(fd, WDIOC_GETSTATUS, (unsigned long)&status);
	TC_ASSERT_EQ_CLEANUP("watchdog_ioctl", ret, OK, close(fd));
wait_ms(1000);
	ret = ioctl(fd, WDIOC_GETSTATUS, (unsigned long)&status);
	TC_ASSERT_EQ_CLEANUP("watchdog_ioctl", ret, OK, close(fd));
wait_ms(1000);
	ret = ioctl(fd, WDIOC_GETSTATUS, (unsigned long)&status);
	TC_ASSERT_EQ_CLEANUP("watchdog_ioctl", ret, OK, close(fd));
wait_ms(1000);
	ret = ioctl(fd, WDIOC_GETSTATUS, (unsigned long)&status);
	TC_ASSERT_EQ_CLEANUP("watchdog_ioctl", ret, OK, close(fd));
#endif
	printf("WDIOC_CAPTURE\n");
	FAR struct watchdog_capture_s *capture;
//printf("--------------newhandler = %d ; oldhandler = %d\n",(int)capture->newhandler, (int)capture->oldhandler);
	capture->newhandler = test_handler(1, NULL, NULL); //------set test handler
	printf("Aft test_handler\n");
	ret = ioctl(fd, WDIOC_CAPTURE, (unsigned long)capture);
	TC_ASSERT_EQ_CLEANUP("watchdog_ioctl", ret, OK, close(fd));

	printf("WDIOC_START=%d\n",WDIOC_START);
	ret = ioctl(fd, WDIOC_START, 0);
	TC_ASSERT_EQ_CLEANUP("watchdog_ioctl", ret, OK, close(fd));
		

	TC_SUCCESS_RESULT();
}


/****************************************************************************
 * Name: watchdog driver test
 ****************************************************************************/
void watchdog_main(void)
{
//	tc_driver_watchdog_open_close();
//	tc_driver_watchdog_read_write();
//	tc_driver_watchdog_ioctl();
	tc_amebad_watchdog();
	return;
}
