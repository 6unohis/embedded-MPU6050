#define _GNU_SOURCE
#include<stdio.h>
#include<stdlib.h>
#include<stdint.h>
#include<unistd.h>
#include<errno.h>
#include<fcntl.h>
#include<signal.h>
#include<string.h>
#include<time.h>
#include<sys/ioctl.h>
#include<linux/i2c-dev.h>

static volatile int running = 1;
static void on_sight(int sig) {(void)sig; running = 0;}
static int i2c_write8(int fd, uint8_t reg, uint8_t val)
{
	uint8_t buf[2] = {reg, val};
	if(write(fd, buf, 2) != 2) return -1;
	return 0;
}

static int i2c_read8(int fd, uint8_t reg, uint8_t *val)
{
	if(write(fd, &reg, 1) != 1) return -1;
	if(read(fd, val, 1) != 1) return -1;
	return 0;
}

static int i2c_readn(int fd, uint8_t reg, uint8_t *buf, size_t n)
{
	if(write(fd, &reg, 1) != 1) return -1;
	if(read(fd, buf, n) != (ssize_t)n) return -1;
	return 0;
}

static int16_t be16(const uint8_t *p){return (int16_t)((p[0] << 8) | p[1] );}

int main(int argc, char ** argv){
	if(argc < 3){
		fprintf(stderr, "Usage : sudo %s <i2c-dev> <addr-hex>\n", argv[0]);
		fprintf(stderr, "Example : sudo %s /dev/i2c-1 0x68\n", argv[0]);
		return 1;
	}

	const char *i2cdev = argv[1];
	int addr = (int)strtol(argv[2], NULL, 0);

	const uint8_t REG_WHO_AM_I     = 0x75;
	const uint8_t REG_PWR_MGMT_1   = 0x6B;
	const uint8_t REG_SMPLRT_DIV   = 0x19;
	const uint8_t REG_CONFIG       = 0x1A;
	const uint8_t REG_GYRO_CFG     = 0x1B;
	const uint8_t REG_ACCEL_CFG    = 0x1C;
	const uint8_t REG_ACCEL_XOUT_H = 0x3B;

	const float   ACC_LSB_PER_G    = 16384.0f;
	const float   GYR_LSB_PER_DPS  = 131.0f;

	int fd = open(i2cdev, O_RDWR);
	if(fd < 0){
		perror("open i2cdev");
		return 1;
	}

	if(ioctl(fd, I2C_SLAVE, addr) < 0){
		perror("ioctl I2C_SLAVE");
		close(fd);
		return 1;
	}

	uint8_t who = 0;
	if(i2c_read8(fd, REG_WHO_AM_I, &who) < 0){
		perror("read WHO_AM_I");
		close(fd);
		return 1;
	}
	printf("[INFO] WHO_AM_I = 0x%02X (expect 0x68)\n", who);
	if(who != 0X68){
		fprintf(stderr, "[WARN] Unexpected WHO_AM_I. Check address\n");
	}

	if(i2c_write8(fd, REG_PWR_MGMT_1, 0x00) < 0){
		perror("write PWR_MGMT_1");
		close(fd);
		return 1;
	}

	usleep(1000 * 100);

	(void)i2c_write8(fd, REG_SMPLRT_DIV, 0x07);
	(void)i2c_write8(fd, REG_CONFIG,     0x03);
	(void)i2c_write8(fd, REG_GYRO_CFG,   0x00);
	(void)i2c_write8(fd, REG_ACCEL_CFG,  0x00);
	usleep(1000 * 10);

	signal(SIGINT, on_sight);
	printf("[INFO] Reading at ~ 100Hz. Press Ctrl + c to stop.\n");

	struct timespec ts = {0};
	ts.tv_sec = 0;
	ts.tv_nsec = 10 * 1000 * 1000;

	while(running){
		uint8_t buf[14];
		if(i2c_readn(fd, REG_ACCEL_XOUT_H, buf, sizeof(buf)) < 0){
			perror("read data burst");
			break;
		}

		int16_t ax_raw = be16(&buf[0]);
		int16_t ay_raw = be16(&buf[2]);
		int16_t az_raw = be16(&buf[4]);
		int16_t temp_raw = be16(&buf[6]);
		int16_t gx_raw = be16(&buf[8]);
		int16_t gy_raw = be16(&buf[10]);
		int16_t gz_raw = be16(&buf[12]);

		float ax_g = ax_raw / ACC_LSB_PER_G;
		float ay_g = ay_raw / ACC_LSB_PER_G;
		float az_g = az_raw / ACC_LSB_PER_G;

		float temp_c = (temp_raw / 340.0f) + 36.53f;

		float gx_dps = gx_raw / GYR_LSB_PER_DPS;
		float gy_dps = gy_raw / GYR_LSB_PER_DPS;
		float gz_dps = gz_raw / GYR_LSB_PER_DPS;

		printf("ACC[g] x=%7.3f y=%7.3f z=%7.3f | GYR[dps] x=%7.2f y=%7.2f z=%7.2f | T=%.2fC\r", ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, temp_c);
		fflush(stdout);

		nanosleep(&ts, NULL);
	}

	printf("\n[INFO] stopped.\n");

	close(fd);
	return 0;
}