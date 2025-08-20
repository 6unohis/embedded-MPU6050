// mpu6050_cat.c
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdint.h>

int main(void){
    int fd = open("/dev/mpu6050_min", O_RDONLY);
    if(fd < 0){ perror("open"); return 1; }
    uint8_t buf[14];
    while(1){
        ssize_t n = read(fd, buf, sizeof(buf));
        if(n < 0){ perror("read"); break; }
        if(n == 14){
            int16_t ax = (buf[0]<<8)|buf[1];
            int16_t ay = (buf[2]<<8)|buf[3];
            int16_t az = (buf[4]<<8)|buf[5];
            int16_t tg = (buf[6]<<8)|buf[7];   
            int16_t gx = (buf[8]<<8)|buf[9];
            int16_t gy = (buf[10]<<8)|buf[11];
            int16_t gz = (buf[12]<<8)|buf[13];
            printf("ACC(%6d,%6d,%6d) TEMPraw(%6d) GYR(%6d,%6d,%6d)\n",
                   ax, ay, az, tg, gx, gy, gz);
        }
        usleep(50*1000);
    }
    close(fd);
    return 0;
}
