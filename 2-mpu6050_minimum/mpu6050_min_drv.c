#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>

#define DRV_NAME        "mpu6050_min"
#define MPU_I2C_ADDR    0x68

#define REG_PWR_MGMT_1      0x6B
#define REG_WHO_AM_I        0x75
#define REG_ACCEL_XOUT_H    0x3B   

struct mpu6050_min {
    struct i2c_client *client;
    struct miscdevice  miscdev;
};

static int mpu6050_read_block(struct i2c_client *client, u8 reg, u8 *buf, int len)
{
    int ret = i2c_smbus_read_i2c_block_data(client, reg, len, buf);
    if (ret == len) return 0;
    return ret < 0 ? ret : -EIO;
}

static int mpu6050_wakeup(struct i2c_client *client)
{
    int ret = i2c_smbus_write_byte_data(client, REG_PWR_MGMT_1, 0x00);
    if (ret < 0) return ret;
    msleep(5);
    return 0;
}

static ssize_t mpu6050_min_read(struct file *f, char __user *ubuf, size_t len, loff_t *ppos)
{
    struct mpu6050_min *chip = container_of(f->private_data, struct mpu6050_min, miscdev);
    u8 frame[14];
    int ret;

    if (len < sizeof(frame))
        return -EINVAL;

    ret = mpu6050_read_block(chip->client, REG_ACCEL_XOUT_H, frame, sizeof(frame));
    if (ret)
        return ret;

    if (copy_to_user(ubuf, frame, sizeof(frame)))
        return -EFAULT;

    return sizeof(frame);
}

static const struct file_operations mpu6050_min_fops = {
    .owner = THIS_MODULE,
    .read  = mpu6050_min_read,
};

static int mpu6050_min_probe(struct i2c_client *client)
{
    struct mpu6050_min *chip;
    int who, ret;

    if (client->addr != MPU_I2C_ADDR) {
        dev_err(&client->dev, "unexpected addr 0x%02x (expected 0x68)\n", client->addr);
        return -ENODEV;
    }

    chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
    if (!chip) return -ENOMEM;

    chip->client = client;

    ret = mpu6050_wakeup(client);
    if (ret) {
        dev_err(&client->dev, "wake-up failed: %d\n", ret);
        return ret;
    }

    who = i2c_smbus_read_byte_data(client, REG_WHO_AM_I);
    if (who < 0) {
        dev_err(&client->dev, "WHO_AM_I read failed: %d\n", who);
        return who;
    }
    if ((who & 0x7E) != 0x68 && who != 0x68) {
        dev_warn(&client->dev, "unexpected WHO_AM_I=0x%02x (continuing anyway)\n", who);
    }

    chip->miscdev.minor = MISC_DYNAMIC_MINOR;
    chip->miscdev.name  = DRV_NAME;
    chip->miscdev.fops  = &mpu6050_min_fops;

    ret = misc_register(&chip->miscdev);
    if (ret) {
        dev_err(&client->dev, "misc_register failed: %d\n", ret);
        return ret;
    }

    i2c_set_clientdata(client, chip);
    dev_info(&client->dev, "ready: /dev/%s (WHO_AM_I=0x%02x)\n", DRV_NAME, who);
    return 0;
}

static void mpu6050_min_remove(struct i2c_client *client)
{
    struct mpu6050_min *chip = i2c_get_clientdata(client);
    misc_deregister(&chip->miscdev);
}

static const struct of_device_id mpu6050_min_of_match[] = {
    { .compatible = "siyoung,mpu6050" },   
    { }
};
MODULE_DEVICE_TABLE(of, mpu6050_min_of_match);

static const struct i2c_device_id mpu6050_min_ids[] = {
    { "mpu6050_min", 0 },
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

MODULE_AUTHOR("you");
MODULE_DESCRIPTION("Minimal MPU6050 driver (no params)");
MODULE_LICENSE("GPL");
