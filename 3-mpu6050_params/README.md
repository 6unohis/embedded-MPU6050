# MPU6050 Linux Driver based ioctl

## 개요
`mpu6050` 자이로 센서 드라이버를 구현한다. 해당 드라이버에서는 `odr`, `lpf`, `gyro & accel full scale` 설정 값들을 `ioctl` 방식으로 설정할 수 있도록 한다.

- DT 오버레이 -> 모듈 자동 바인딩 -> 디바이스 생성은 이전과 동일
- 설정값 변경 `ioctl` 함수 구현
- 사용자 프로그램에서 메뉴 형식을 통해 동작 확인

## 설치 및 빌드
### 1. 오버레이 컴파일 & 적용
```
~/3-mpu6050_params $ sudo dtc -@ -I dts -O dtb -o /boot/firmware/overlays/mpu6050-params-overlay.dtbo mpu6050-siyoung-overlay.dts
```
`/boot/firmware/config.txt`에 다음 줄 추가
```
dtoverlay=mpu6050-params-overlay
```

### 2. 드라이버 및 사용자 코드 빌드
```
~/3-mpu6050_params $ make
```

정상 빌드 완료 시 `mpu6050_params_drv.ko`모듈 생성

### 3. 모듈 설치 및 의존성 갱신
```
~/3-mpu6050_params $ mkdir /lib/modules/$(uname -r)/extra
~/3-mpu6050_params $ sudo cp mpu6050_params_drv.ko /lib/modules/$(uname -r)/extra
~/3-mpu6050_params $ sudo depmod -a
```

## 사용법
모듈 로드 및 디바이스 노드 확인 등은 이전 `mpu6050_minimum`과 동일하게 실행할 수 있다.

## 진행 예정 사항
이전까지 진행한 내용들은 polling 방식으로, `ioctl`이나 `read` 호출 시 드라이버 내부에서는 레지스터들을 확인하며 데이터가 준비되었는지를 계속 확인한다.

구현은 단순하지만 CPU 자원도 낭비이고 latency는 우리가 설정하는 타이머 간격에 종속된다는 단점이 있다.

그러나 mpu6050은 데이터를 주기적으로 뱉는 스트림 장치로 인터럽트 방식으로 발전시킨다면 즉각적인 데이터 확인도 가능할 테고 polling 방식에서 놓친 데이터들을 최대한 정확하게 얻을 수 있을 것이다.

이런 IRQ 기반 동작은 CPU의 활용률을 높이고 latency 감소에도 효과적일 것으로 예상된다. 샘플 드롭 역시 감소할 것이며 CPU는 busy loop 방식의 polling을 진행하지 않으니 저전력에 유리할 것으로 예상된다.