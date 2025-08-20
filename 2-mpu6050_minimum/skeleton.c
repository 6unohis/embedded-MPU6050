#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>

#define DRV_NAME        "mpu6050"
#define MPU_I2C_ADDR     0x68

#define REG_PWR_MGMT_1   0x6B
#define REG_WHO_AM_I     0x75
#define REG_ACCEL_XOUT_H 0x3B

/* 
1.
사용자 공간과의 인터페이스를 위한 char device 
I2C 방식의 mpu6050에서 실제 데이터를 호출하기 위한 i2c_client
*/
struct mpu6050_min{
    struct i2c_client *client;
    struct miscdevice  miscdev;
};

/*
3.
read 실제 구현
mpu6050_min의 client를 통해 접근할 수 있다고 가정하고 시작
*/
static ssize_t mpu6050_min_read(struct file *fp, char __user *buf, size_t len, loff_t *pos)
{
    /* 3-1. i2c로 접근하기 위해 상위구조체 호출 */
    struct mpu6050_min *chip = container_of(fp->private_data, struct mpu6050_min, miscdev);

    u8 sample[14];
    
    /* 3-2. 상위구조체->i2c_clinet로 접근 */
    i2c_smbus_read_i2c_block_data(chip->client, REG_ACCEL_XOUT_H, sizeof(sample), sample);

    /* 3-3. sample에 저장했으니 copy_to_user로 전달 */
    copy_to_user(buf, sample, sizeof(sample));
}

/*
2. 
사용자 공간에서 실제로 접근하는 장치는 /dev/... 장치니 read만 정의
open의 경우 래퍼가 존재
release의 경우 miscdev에 대한 동적 할당이 없으니 스킵
*/
struct file_operations fops = {
    .owner = THIS_MODULE,
    .read  = mpu6050_min_read
};

/* 5. 
부팅 시 오버레이를 파싱하며 I2C 디바이스를 탐색
해당되는 디바이스 노드에 대한 i2c_client를 생성하고 등록된 모든 i2c_driver의 of_match_table 비교
매칭 성공 시 probe 호출
따라서 probe에서는 i2c_client와 디바이스 노드 mpu6050_min을 연결하기 위해 공간 할당, 연결, 초기화 과정 필요
*/

static int mpu6050_min_probe(struct i2c_client *client)
{
    /* 5-1. i2c_client가 mpu6050이 맞는지 검증 */
    if(client->addr != MPU_I2C_ADDR){
        return -ENODEV;
    }

    /* 5-2. 디바이스 노드 mpu6050_min 생성 & client 연결 가능 */
    struct mpu6050_min *m;
    m = devm_kzalloc(&client->dev, sizeof(*m), GFP_KERNEL);
    m->client = client;

    /* 5-3. miscdev 초기화 */
    ...
    m->miscdev.fops = &fops;

    /* 5-4. miscdev 초기화 완료 시에만 커널에 등록 */
    misc_register(&m->miscdev);

    /* 5-5. dev->client 등록했으니 client->dev 등록 */
    i2c_set_clientdata(client, m);
}

static void mpu6050_min_remove(struct i2c_client *clinet)
{

}

/* 4. I2C 드라이버 등록 및 해제 */
static const struct of_device_id mpu6050_min_of_match[] = {
    { .compatible =  },   
    { }
};
MODULE_DEVICE_TABLE(of, mpu6050_min_of_match);

static const struct i2c_device_id mpu6050_min_ids[] = {
    {  },
    { }
};
MODULE_DEVICE_TABLE(i2c, mpu6050_min_ids);

static struct i2c_driver mpu6050_min_driver = {
    .driver = {
        .name = DRV_NAME,
        .of_match_table = mpu6050_min_of_match,
    },
    .probe    = mpu6050_min_probe,
    .remove   = mpu6050_min_remove,
    .id_table = mpu6050_min_ids,
};

module_i2c_driver(mpu6050_min_driver);