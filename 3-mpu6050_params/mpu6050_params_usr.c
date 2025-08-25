#define _GNU_SOURCE
#include<stdio.h>
#include<stdint.h>
#include<unistd.h>
#include<fcntl.h>
#include<errno.h>
#include<string.h>
#include<sys/ioctl.h>
#include<time.h>
#include<stdlib.h>

#include "mpu6050_params.h"

#ifndef __u32
#define __u32 uint32_t
#endif

struct mpu6050_sample{
	uint64_t ts_ns;
	int16_t ax, ay, az;
	int16_t gx, gy, gz;
	int16_t temp;
} __attribute__((aligned(8)));

static void sleep_ms(int ms)
{
	struct timespec ts = { .tv_sec = ms/1000, .tv_nsec = (long)(ms % 1000) * 1000000L};
	nanosleep(&ts, NULL);
}

static int read_line(char *buf, size_t sz)
{
	if(!fgets(buf, (int)sz, stdin)) return -1;
	size_t n = strlen(buf);
	if(n && buf[n - 1] == '\n') buf[n - 1] = '\0';
	return 0;
}

static int ask_u32(const char *prompt, __u32 *out)
{
	char line[128];
	printf("%s", prompt);
	fflush(stdout);
	if(read_line(line, sizeof(line)) < 0) return -1;
	char *end = NULL;
	unsigned long v = strtoul(line, &end, 10);
	if(end == line || *end) return -1;
	*out = (__u32)v;
	return 0;
}

static int ensure_open(const char *dev, int *fd)
{
	if(*fd > 0) return 0;
	*fd = open(dev, O_RDONLY);
	if(*fd < 0){
		perror("open");
		fprintf(stderr, "failed to open\n");
		return -1;
	}
	return 0;
}

int main(void)
{
	const char *dev = "/dev/mpu6050_params";
	int fd = -1;

	if(ensure_open(dev, &fd) < 0) return 1;

	__u32 v; int rc;
	v = 3; rc = ioctl(fd, MPU_IOC_SET_LPF, &v); if(rc < 0) fprintf(stderr, "Warn : SET_LPF");
	v = 100; rc = ioctl(fd, MPU_IOC_SET_ODR, &v); if(rc < 0) fprintf(stderr, "Warn : SET_ODR");
	v = 2; rc = ioctl(fd, MPU_IOC_SET_FS_A, &v); if(rc < 0) fprintf(stderr, "Warn : SET_FS_A");
	v = 250; rc = ioctl(fd, MPU_IOC_SET_FS_G, &v); if(rc < 0) fprintf(stderr, "Warn : SET_FS_G");
	sleep_ms(20);

	puts("---------------------------------------------------");

	while(1){
		printf(
				"\n[Menu]\n"
				" 1) read (1 sample)\n"
				" 2) ioctl SET_ODR (Hz)\n"
				" 3) ioctl_SET_LPF (0...6)\n"
				" 4) ioctl_SET_FS_A (2/4/8/16 g)\n"
				" 5) ioctl_SET_FS_G (250/500/1000/2000 dps)\n"
				" q) quit \n"
				" Select : ");
		fflush(stdout);

		char sel[16];
		if(read_line(sel, sizeof(sel)) < 0) break;
		if(sel[0] == 'q' || sel[0] == 'Q') break;

		switch(sel[0]){
			case '1' : 
				for(int i = 0; i < 10; i++){
					struct mpu6050_sample s;
					ssize_t n = read(fd, &s, sizeof(s));

					if(n < 0){
						if(errno == EINTR) continue;
						perror("read");
						break;
					}
					if(n == 0){
						close(fd);
						fd = open("/dev/mpu6050_params", O_RDONLY);
						if(fd < 0) {perror("reopen"); break; }
						usleep(5000);
						continue;
					}
					if(n != sizeof(s)){
						fprintf(stderr, "short read\n");
						break;
					}
					printf("%llu ns accel=[%6d %6d %6d] gyro=[%6d %6d %6d]\n", (unsigned long long)s.ts_ns, s.ax, s.ay, s.az, s.gx, s.gy, s.gz);
					usleep(10000);
				}
				break;
			case '2':
				__u32 odr;
				if(ask_u32("Enter ODR(HZ): ", &odr) < 0) {puts("invalid"); break;}
				rc = ioctl(fd, MPU_IOC_SET_ODR, &odr);
				if(rc < 0) fprintf(stderr, "SET_ODR failed\n");
				else puts("SET_ODR ok");
				break;

			case '3':
				__u32 dlpf;
				if(ask_u32("Enter DLPF (0...6): ", &dlpf) < 0 || dlpf > 6) {puts("invalid"); break;}
				rc = ioctl(fd, MPU_IOC_SET_LPF, &dlpf);
				if(rc < 0) fprintf(stderr, "SET_LPF failed\n");
				else puts("SET_LPF ok");
				break;

			case '4':
				__u32 g;
				if(ask_u32("Enter accel FS (2/4/8/16 g): ", &g) < 0) {puts("invalid"); break; }
				if(!(g == 2 || g == 4 || g == 8 || g == 16)) {puts("invalid"); break; }
				rc = ioctl(fd, MPU_IOC_SET_FS_A, &g);
				if(rc < 0) fprintf(stderr, "SET_FS_A_failed\n");
				else puts("SET_FS_A ok");
				break;

			case '5':
				__u32 dps;
				if(ask_u32("Enter gyro FS (250/500/1000/2000 dps): ", &dps) < 0) {puts("invalid"); break;}
				if(!(dps == 250 || dps == 500 || dps == 1000 || dps == 2000)) {puts("invalid"); break;}
				rc = ioctl(fd, MPU_IOC_SET_FS_G, &dps);
				if(rc < 0) fprintf(stderr, "SET_FS_G failed\n");
				else puts("SET_FS_G ok");
				break;

			default:
				puts("unknown sel");
				break;
		}
	}

	if(fd >= 0) close(fd);
	puts("tlqkf");
	return 0;
}

