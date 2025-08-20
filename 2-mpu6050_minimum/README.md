# Minimal MPU6050 Linux Driver

## 개요
`mpu6050` 자이로 센서 최소 동작 드라이버를 구현한다.

모듈 파라미터, 스케일 변환, IIO 인터페이스 등을 제외하고 단순 `read`를 통해 raw data를 가져오도록 구현한 캐릭터 디바이스(`/dev/mpu6050-min`) 드라이버이다.

- 디바이스 트리 오버레이를 통해 I2C1 버스(0x68)에 센서 등록
- 드라이버 모듈 자동 바인딩
- `/dev/mpu6050-min`디바이스 생성

## 준비
- Raspberry Pi 4
- MPU6050 센서

|MPU6050|Raspberry Pi|
|-|-|
|VCC|5V|
|GND|GND|
|SCL|GPIO2|
|SDA|GPIO3|

- 커널 헤더 및 `dtc`설치
```bash
$ sudo apt-get install raspberrypi-kernel-headers device-tree-compiler build-essential
```

## 설치 및 빌드
### 1. 오버레이 컴파일 & 적용
```
~/2-mpu6050_minimum $ sudo dtc -@ -I dts -O dtb -o /boot/firmware/overlays/mpu6050-overlay.dtbo mpu6050-siyoung-overlay.dts
```
`/boot/firmware/config.txt`에 다음 줄 추가
```
dtoverlay=mpu6050-overlay
```

### 2. 드라이버 및 사용자 코드 빌드
```
~/2-mpu6050_minimum $ make
```

정상 빌드 완료 시 `mpu6050_min_drv.ko`모듈 생성

### 3. 모듈 설치 및 의존성 갱신
```
~/2-mpu6050_minimum $ mkdir /lib/modules/$(uname -r)/extra
~/2-mpu6050_minimum $ sudo cp mpu6050_min_drv.ko /lib/modules/$(uname -r)/extra
~/2-mpu6050_minimum $ sudo depmod -a
```

## 사용법
### 1. 모듈 로드 및 디바이스 노드 확인
```bash
lsmod | grep -i mpu
```
> 예상 결과: `mpu6050_min_drv   12288   0`

```bash
ls -lah /dev/mpu*
```
> 예상 결과: `crw------- 1 root ... /dev/mpu6050_min`

```bash
readlink /sys/bus/i2c/devices/1-0068/driver
```
> 예상 결과: `../../../../../../bus/i2c/drivers/mpu6050_min`

```bash
dmesg | grep mpu
```
> 예상 결과
>
> `[...] mpu6050_min_drv: loading out-of-tree module taints kernel.`
>
> `[...] mpu6050_min 1-0068: ready: /dev/mpu6050_min (WHO_AM_I=0x68)` 

### 2. 사용자 코드 실행
```
~/2-mpu6050_minimum $ sudo ./mpu6050_min_user
```
> 예상 결과
>
> `ACC(..., ..., ...) TEMPraw(...) GYR(..., ..., ...)`
>
> `ACC(..., ..., ...) TEMPraw(...) GYR(..., ..., ...)`