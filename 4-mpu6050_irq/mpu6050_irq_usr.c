// user_2.c
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
#include<sys/poll.h>

#include "mpu6050_irq.h"

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
		fprintf(stderr, "failed to open %s\n", dev);
		return -1;
	}
	return 0;
}

static uint64_t now_ns(void){
	struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec*1000000000ull + ts.tv_nsec;
}

/* 간단 벤치: poll + read 로 N개 샘플 수집, 평균 간격/레이턴시 출력 */
static void bench_irq(int fd, int N, int timeout_ms)
{
	struct pollfd pfd = { .fd = fd, .events = POLLIN };
	struct mpu6050_sample s;
	uint64_t *lat = malloc(sizeof(uint64_t)*N);
	uint64_t *intv = malloc(sizeof(uint64_t)*N);
	if(!lat || !intv){ fprintf(stderr,"oom\n"); goto out; }

	int got = 0;
	uint64_t last_ts = 0;
	double sum_lat = 0.0, sum_intv = 0.0;

	while(got < N){
		int pr = poll(&pfd, 1, timeout_ms);
		if(pr < 0){
			if(errno == EINTR) continue;
			perror("poll"); break;
		}
		if(pr == 0){
			fprintf(stderr, "timeout while waiting data\n");
			break;
		}
		if(pfd.revents & POLLIN){
			ssize_t n = read(fd, &s, sizeof(s));
			if(n != sizeof(s)){
				if(n < 0 && errno == EINTR) continue;
				perror("read"); break;
			}
			uint64_t tnow = now_ns();
			uint64_t l = (tnow >= s.ts_ns) ? (tnow - s.ts_ns) : 0;
			lat[got] = l;
			sum_lat += (double)l;

			if(got > 0){
				uint64_t d = s.ts_ns - last_ts;
				intv[got] = d;
				sum_intv += (double)d;
			}
			last_ts = s.ts_ns;
			got++;
		}
	}

	if(got > 1){
		double mean_lat_us = (sum_lat / (double)got) / 1000.0;
		double mean_intv_us = (sum_intv / (double)(got-1)) / 1000.0;
		double hz = 1e6 / mean_intv_us;
		printf("[BENCH] N=%d  mean_interval=%.2f us (%.2f Hz)  mean_latency=%.2f us\n",
			got, mean_intv_us, hz, mean_lat_us);
	} else {
		printf("[BENCH] not enough samples (got=%d)\n", got);
	}

out:
	free(lat); free(intv);
}

int main(void)
{
	const char *dev = "/dev/" DRV_NAME;
	int fd = -1;

	if(ensure_open(dev, &fd) < 0) return 1;

	__u32 v; int rc;
	/* 초기 설정(경고만) */
	v = 3;   rc = ioctl(fd, MPU_IOC_SET_LPF, &v);  if(rc < 0) fprintf(stderr, "Warn : SET_LPF\n");
	v = 100; rc = ioctl(fd, MPU_IOC_SET_ODR, &v);  if(rc < 0) fprintf(stderr, "Warn : SET_ODR\n");
	v = 2;   rc = ioctl(fd, MPU_IOC_SET_FS_A, &v); if(rc < 0) fprintf(stderr, "Warn : SET_FS_A\n");
	v = 250; rc = ioctl(fd, MPU_IOC_SET_FS_G, &v); if(rc < 0) fprintf(stderr, "Warn : SET_FS_G\n");

	puts("MPU6050 interactive menu (dev: /dev/" DRV_NAME ")");
	puts("---------------------------------------------------");

	while(1){
		printf(
			"\n[Menu]\n"
			" 1) read (10 samples, 10ms interval)  [polling path도 동작]\n"
			" 2) ioctl SET_ODR (Hz)\n"
			" 3) ioctl SET_LPF (0...6)\n"
			" 4) ioctl SET_FS_A (2/4/8/16 g)\n"
			" 5) ioctl SET_FS_G (250/500/1000/2000 dps)\n"
			" 6) bench IRQ (N samples)\n"
			" 7) ioctl SET_IRQ_EN (1:on / 0:off)\n"
			" q) quit \n"
			" Select : ");
		fflush(stdout);

		char sel[16];
		if(read_line(sel, sizeof(sel)) < 0) break;
		if(sel[0] == 'q' || sel[0] == 'Q') break;

		switch(sel[0]){
		case '1': {
			for(int i = 0; i < 10; i++){
				struct mpu6050_sample s;
				ssize_t n = read(fd, &s, sizeof(s));
				if(n < 0){
					if(errno == EINTR){ i--; continue; }
					perror("read");
					break;
				}
				if(n != sizeof(s)){ fprintf(stderr, "short read\n"); break; }
				printf("%llu ns accel=[%6d %6d %6d] gyro=[%6d %6d %6d]\n",
					(unsigned long long)s.ts_ns, s.ax, s.ay, s.az, s.gx, s.gy, s.gz);
				usleep(10000);
			}
			break;
		}
		case '2': {
			__u32 odr;
			if(ask_u32("Enter ODR(Hz): ", &odr) < 0){ puts("invalid"); break; }
			rc = ioctl(fd, MPU_IOC_SET_ODR, &odr);
			puts(rc < 0 ? "SET_ODR failed" : "SET_ODR ok");
			break;
		}
		case '3': {
			__u32 dlpf;
			if(ask_u32("Enter DLPF (0...6): ", &dlpf) < 0 || dlpf > 6){ puts("invalid"); break; }
			rc = ioctl(fd, MPU_IOC_SET_LPF, &dlpf);
			puts(rc < 0 ? "SET_LPF failed" : "SET_LPF ok");
			break;
		}
		case '4': {
			__u32 g;
			if(ask_u32("Enter accel FS (2/4/8/16 g): ", &g) < 0){ puts("invalid"); break; }
			if(!(g == 2 || g == 4 || g == 8 || g == 16)){ puts("invalid"); break; }
			rc = ioctl(fd, MPU_IOC_SET_FS_A, &g);
			puts(rc < 0 ? "SET_FS_A failed" : "SET_FS_A ok");
			break;
		}
		case '5': {
			__u32 dps;
			if(ask_u32("Enter gyro FS (250/500/1000/2000 dps): ", &dps) < 0){ puts("invalid"); break; }
			if(!(dps == 250 || dps == 500 || dps == 1000 || dps == 2000)){ puts("invalid"); break; }
			rc = ioctl(fd, MPU_IOC_SET_FS_G, &dps);
			puts(rc < 0 ? "SET_FS_G failed" : "SET_FS_G ok");
			break;
		}
		case '6': {
			__u32 N;
			if(ask_u32("Enter N (samples): ", &N) < 0 || N < 2){ puts("invalid"); break; }
			bench_irq(fd, (int)N, 2000);
			break;
		}
		case '7': {
			__u32 en;
			if(ask_u32("Enable IRQ? (1:on / 0:off): ", &en) < 0){ puts("invalid"); break; }
			rc = ioctl(fd, MPU_IOC_SET_IRQ_EN, &en);
			puts(rc < 0 ? "SET_IRQ_EN failed" : "SET_IRQ_EN ok");
			break;
		}
		default:
			puts("unknown sel");
			break;
		}
	}

	if(fd >= 0) close(fd);
	return 0;
}
