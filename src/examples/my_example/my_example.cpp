#include <px4_platform_common/posix.h>
#include <px4_platform_common/log.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <mathlib/math/Limits.hpp>

#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>

void print_usage();
int command_run(int ntimes);
int command_publish_attitude(int ntimes);

extern "C" __EXPORT int my_example_main(int argc, char* argv[])
{
	if (argc < 2) {
		fprintf(stderr, "Error input\n");
		print_usage();
		return ERROR;
	}
	if (strncmp(argv[1], "help", sizeof("help")) == 0) {
		print_usage();
		return OK;
	} else if (strncmp(argv[1], "run", sizeof("run")) == 0 && argc > 2) {
		int ntimes = math::min(atoi(argv[2]), 50);
		return command_run(ntimes);
	} else if (strncmp(argv[1], "publish", sizeof("publish")) == 0 && argc > 2) {
		int ntimes = math::min(atoi(argv[2]), 50);
		return command_publish_attitude(ntimes);
	}

	fprintf(stderr, "Invalid command: ");
	for (int i = 1; i < argc; ++i) {
		fprintf(stderr, "%s ", argv[i]);
	}
	fprintf(stderr, "\n");
	print_usage();
	return ERROR;
}

void print_usage()
{
	fprintf(stderr,
		"My example\n"
		"useage: my_example <command> [options]\n"
		"commands:\n"
		"\thelp\n"
		"\trun <ntimes>\n"
		"\tpublish <ntimes>\n"
	);
}

int command_run(int ntimes)
{
	PX4_INFO("Hello px4 example: run %d times", ntimes);
	int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
	px4_pollfd_struct_t fds[] =  {
		{.fd = sensor_sub_fd, .events = POLLIN },
	};

	for(int i = 1; i <= ntimes; ++i) {
		int poll_ret = px4_poll(fds, 1, 1000);
		if (poll_ret <= 0) {
			PX4_ERR("px4_poll error: %d", poll_ret);
			break;
		}
		if (fds[0].revents & POLLIN) {
			sensor_combined_s raw;
			orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
			printf(
				"Read sensor [%d]\n"
				"\tAccelerometer: \t%8.4f\t%8.4f\t%8.4f\n"
				"\tGyroscope:	\t%8.4f\t%8.4f\t%8.4f\n",
				i,
			    	(double)raw.accelerometer_m_s2[0],
			    	(double)raw.accelerometer_m_s2[1],
			    	(double)raw.accelerometer_m_s2[2],
				(double)raw.gyro_rad[0],
				(double)raw.gyro_rad[1],
				(double)raw.gyro_rad[2]
			);
			px4_sleep(1);
		}
	}
	PX4_INFO("Goodbye px4 example");
	return OK;
}

int command_publish_attitude(int ntimes)
{
	PX4_INFO("Hello PX4 example: publish attitude");

	auto sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
	orb_set_interval(sensor_sub_fd, 200);

	auto att = vehicle_attitude_s{};
	auto att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);

	px4_pollfd_struct_t fds[] = {
		{.fd = sensor_sub_fd, .events = POLLIN},
	};

	uint error_counter = 0;
	for (int i = 0; i < ntimes; ++i) {
		int poll_ret = px4_poll(fds, 1, 1000);
		if (poll_ret == 0) {
			PX4_ERR("No data within a second");
		} else if (poll_ret < 0) {
			if (error_counter < 10 || error_counter % 50 == 0) {
				PX4_ERR("Error return value from px4_poll(): %d", poll_ret);
			}
			++error_counter;
		} else {
			if (fds[0].revents & POLLIN) {
				sensor_combined_s raw;
				orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
				printf("publish: %llu\n", raw.timestamp);
				att.q[0] = raw.accelerometer_m_s2[0];
				att.q[1] = raw.accelerometer_m_s2[1];
				att.q[2] = raw.accelerometer_m_s2[2];
				// att.q[0] = 3.14f;
				// att.q[1] = 6.28f;
				// att.q[2] = 9.66f;
				// att.q[3] = 1234.f;
				orb_publish(ORB_ID(vehicle_attitude), att_pub, &att);
			}
		}
	}
	PX4_INFO("Goodbye my example: publish");
	return OK;
}
