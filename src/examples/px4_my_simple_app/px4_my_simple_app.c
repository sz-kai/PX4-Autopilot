/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
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
 * @file px4_my_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */


#if 0
__EXPORT int px4_my_simple_app_main(int argc, char *argv[]);

int px4_my_simple_app_main(int argc, char *argv[])
{
	PX4_INFO("Hello Sky! test");
	return OK;
}
#endif

#if 0
int px4_my_simple_app_main(int argc, char *argv[])
{
	if (argc < 2) {
		PX4_INFO("Usage: px4_simple_app {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {
		// 执行启动逻辑
		PX4_INFO("App started");

	} else if (!strcmp(argv[1], "stop")) {
		// 执行停止逻辑
		PX4_INFO("App stopped");
	}

	return OK;
}
#endif

#if 1

/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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

// #include <px4_platform_common/px4_config.h>
#include <stdio.h>
#include <string.h>

#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>

__EXPORT void px4_my_simple_app_main(int argc, char *argv[]);

void px4_my_simple_app_main(int argc, char *argv[])
{
	PX4_INFO("Hello Sky!");


	/* 1. 订阅数据,获取文件描述符 */
	int sensor_combined_sub_fd = orb_subscribe(ORB_ID(sensor_combined));

	if (sensor_combined_sub_fd < 0) {
		PX4_ERR("orb_subscribe failed - sensor_combined");
		return;
	}

	px4_pollfd_struct_t fds[] = {
		{.fd = sensor_combined_sub_fd, .events = POLLIN},
		/* 可以同时监视多个主题 */
		/* {.fd = ..., .events = POLLIN} */
	};

	/* 2. 广播主题 */
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	orb_advert_t att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);

	/* 3. 等待订阅数据更新 */
	int counter = 0;

	for (counter = 0; counter < 10; counter++) { // 读取10次数据
		int poll_ret = px4_poll(fds, 1, 1000);

		if (poll_ret == 0) {
			PX4_WARN("No data within a second");
			return;


			
		} else if (poll_ret < 0) {
			PX4_ERR("px4_poll failed");
			return;

		} else {
			if (fds[0].revents & POLLIN) {
				/* 4. 读取数据 */
				struct sensor_combined_s raw;
				orb_copy(ORB_ID(sensor_combined), sensor_combined_sub_fd, &raw);
				PX4_INFO("Accelerometer:\t%8.4f\t%8.4f\t%8.4f",
					 (double)raw.accelerometer_m_s2[0],
					 (double)raw.accelerometer_m_s2[1],
					 (double)raw.accelerometer_m_s2[2]);
				/* 5. 发布姿态数据 */
				att.q[0] = raw.gyro_rad[0];
				att.q[1] = raw.gyro_rad[1];
				att.q[2] = raw.gyro_rad[2];
				orb_publish(ORB_ID(vehicle_attitude), att_pub, &att);
			}

		}
	}
}

#endif
