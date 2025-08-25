// SPDX-License-Identifier: GPL-2.0
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/ioctl.h>
#include <linux/ktime.h>
#include <linux/bitops.h>

#include "mpu6050_params.h"

struct mpu6050_sample {
	u64 ts_ns;
	s16 ax, ay, az;
	s16 gx, gy, gz;
	s16 temp;
} __aligned(8);

struct mpu6050_params{
	struct i2c_client    *client;
	struct mpu6050_sample sample;
	struct miscdevice     miscdev;
	struct mutex          lock;
};

static int param_gyro_fs  = 0; // 0:250,1:500,2:1000,3:2000(dps) 
static int param_accel_fs = 0; // 0:2,  1:4,  2:8,   3:16  (g)   
static int param_dlpf     = 3; // 0..6                 
static int param_smpldiv  = 9; // SMPLRT_DIV (ODR = base/(1+div))    

module_param(param_gyro_fs,  int, 0444);
module_param(param_accel_fs, int, 0444);
module_param(param_dlpf,     int, 0444);
module_param(param_smpldiv,  int, 0444);

MODULE_PARM_DESC(param_gyro_fs,  "Gyro full-scale: 0:250,1:500,2:1000,3:2000 dps");
MODULE_PARM_DESC(param_accel_fs, "Accel full-scale: 0:2g,1:4g,2:8g,3:16g");
MODULE_PARM_DESC(param_dlpf,     "DLPF cfg (0..6)");
MODULE_PARM_DESC(param_smpldiv,  "SMPLRT_DIV (0..255)");

#define GYRO_FS_SHIFT   3
#define ACCEL_FS_SHIFT  3
#define FS_MASK         (0x3 << 3)

static int read_block(struct i2c_client *client, u8 reg, u8 *buf, int len)
{
	int ret = i2c_smbus_read_i2c_block_data(client, reg, len, buf);
	if (ret == len) return 0;
	return ret < 0 ? ret : -EIO;
}

static int update_bits(struct i2c_client *client, u8 reg, u8 mask, u8 val)
{
	int old = i2c_smbus_read_byte_data(client, reg);
	u8 newv;
	int ret;

	if (old < 0) return old;
	newv = ((u8)old & ~mask) | (val & mask);
	if (newv == (u8)old) return 0;
	ret = i2c_smbus_write_byte_data(client, reg, newv);
	return ret < 0 ? ret : 0;
}

static int current_base_rate_hz(struct i2c_client *client)
{
	int cfg = i2c_smbus_read_byte_data(client, REG_CONFIG);
	if (cfg < 0) return cfg;
	return ((cfg & 0x07) == 0) ? 8000 : 1000;
}

static int set_sleep(struct i2c_client *client, bool en)
{
	return update_bits(client, REG_PWR_MGMT_1, BIT(6), en ? BIT(6) : 0);
}

static int set_clk_source_pll(struct i2c_client *client)
{
	return update_bits(client, REG_PWR_MGMT_1, 0x07, 0x01);
}

static int mpu6050_params_wakeup(struct i2c_client *client)
{
	int ret = set_sleep(client, false);
	if (ret) return ret;
	msleep(10);
	ret = set_clk_source_pll(client);
	if (ret) return ret;
	msleep(5);
	return 0;
}

static int mpu_open(struct inode *ino, struct file *f)
{
	struct mpu6050_params *m = container_of(f->private_data, struct mpu6050_params, miscdev);
	return 0;
}

static ssize_t mpu6050_params_read(struct file *f, char __user *buf, size_t len, loff_t *pos)
{
	struct mpu6050_params *m = container_of(f->private_data, struct mpu6050_params, miscdev);
	u8 raw[14];
	int ret;

	if (*pos) 
		return 0;

	if (len < sizeof(m->sample))
		return -EINVAL;

	mutex_lock(&m->lock);
	ret = read_block(m->client, REG_ACCEL_XOUT_H, raw, sizeof(raw));
	if (!ret) {
		m->sample.ax    = (s16)((raw[0]  << 8) | raw[1]);
		m->sample.ay    = (s16)((raw[2]  << 8) | raw[3]);
		m->sample.az    = (s16)((raw[4]  << 8) | raw[5]);
		m->sample.gx    = (s16)((raw[8]  << 8) | raw[9]);
		m->sample.gy    = (s16)((raw[10] << 8) | raw[11]);
		m->sample.gz    = (s16)((raw[12] << 8) | raw[13]);
		m->sample.temp  = (s16)((raw[6]  << 8) | raw[7]);
		m->sample.ts_ns = ktime_get_ns();
	}
	mutex_unlock(&m->lock);
	if (ret) 
		return ret;

	if (copy_to_user(buf, &m->sample, sizeof(m->sample)))
		return -EFAULT;

	*pos = sizeof(m->sample);
	return sizeof(m->sample);
}

static int mpu_set_odr(struct i2c_client *c, u32 odr_hz, bool dlpf_on)
{
	int base, div;

	if (!odr_hz) 
		return -EINVAL;
	base = dlpf_on ? 1000 : 8000;

	div = (base + odr_hz/2) / odr_hz;
	if (div <= 0)  
		div = 1;     
	if (div > 256) 
		div = 256; 

	div -= 1;

	return i2c_smbus_write_byte_data(c, REG_SMPLRT_DIV, (u8)div);
}

static int mpu_set_lpf(struct i2c_client *c, u32 dlpf_cfg /*0..6*/)
{
	if (dlpf_cfg > 6) return -EINVAL;
	return update_bits(c, REG_CONFIG, 0x07, (u8)(dlpf_cfg & 0x7));
}

static int mpu_set_fs_accel(struct i2c_client *c, u32 g_fullscale /*2,4,8,16*/)
{
	u8 sel;
	switch (g_fullscale) {
	case 2:  sel = 0; break;
	case 4:  sel = 1; break;
	case 8:  sel = 2; break;
	case 16: sel = 3; break;
	default: return -EINVAL;
	}
	return update_bits(c, REG_ACCEL_CONFIG, FS_MASK, (u8)(sel << ACCEL_FS_SHIFT));
}

static int mpu_set_fs_gyro(struct i2c_client *c, u32 dps_fullscale /*250,500,1000,2000*/)
{
	u8 sel;
	switch (dps_fullscale) {
	case 250:  sel = 0; break;
	case 500:  sel = 1; break;
	case 1000: sel = 2; break;
	case 2000: sel = 3; break;
	default: return -EINVAL;
	}
	return update_bits(c, REG_GYRO_CONFIG, FS_MASK, (u8)(sel << GYRO_FS_SHIFT));
}

static long mpu6050_params_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	struct mpu6050_params *m = container_of(f->private_data, struct mpu6050_params, miscdev);
	struct i2c_client *client = m->client;

	if (_IOC_TYPE(cmd) != MPU_IOC_MAGIC || _IOC_NR(cmd) > MPU6050_IOC_MAXNR)
		return -ENOTTY;

	switch (cmd) {
	case MPU_IOC_SET_ODR: {
		u32 odr;
		int base, ret;

		if (copy_from_user(&odr, (void __user *)arg, sizeof(odr)))
			return -EFAULT;
		if (!odr) 
			return -EINVAL;

		mutex_lock(&m->lock);
		base = current_base_rate_hz(client);
		if (base < 0) { 
			mutex_unlock(&m->lock); 
			return base; 
		}
		ret = mpu_set_odr(client, odr, base == 1000);
		mutex_unlock(&m->lock);
		return ret;
	}
	case MPU_IOC_SET_LPF: {
		u32 dlpf;
		int ret, base;

		if (copy_from_user(&dlpf, (void __user *)arg, sizeof(dlpf)))
			return -EFAULT;
		if (dlpf > 6) 
			return -EINVAL;

		mutex_lock(&m->lock);
		ret = mpu_set_lpf(client, dlpf);
		mutex_unlock(&m->lock);
		return ret;
	}
	case MPU_IOC_SET_FS_A: {
		u32 fs;
		int ret;

		if (copy_from_user(&fs, (void __user *)arg, sizeof(fs)))
			return -EFAULT;

		mutex_lock(&m->lock);
		ret = mpu_set_fs_accel(client, fs);
		mutex_unlock(&m->lock);
		return ret;
	}
	case MPU_IOC_SET_FS_G: {
		u32 fs;
		int ret;

		if (copy_from_user(&fs, (void __user *)arg, sizeof(fs)))
			return -EFAULT;

		mutex_lock(&m->lock);
		ret = mpu_set_fs_gyro(client, fs);
		mutex_unlock(&m->lock);
		return ret;
	}
	default:
		return -ENOTTY;
	}
}

static const struct file_operations fops = {
	.owner          = THIS_MODULE,
	.open           = mpu_open,
	.read           = mpu6050_params_read,
	.unlocked_ioctl = mpu6050_params_ioctl,
};

static int mpu_probe(struct i2c_client *client)
{
	struct mpu6050_params *m;
	int who, ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_I2C_BLOCK)) {
		dev_err(&client->dev, "SMBus byte/block not supported\n");
		return -EOPNOTSUPP;
	}

	if (client->addr != 0x68 && client->addr != 0x69)
		return -ENODEV;
	if (client->addr != MPU_I2C_ADDR)
		dev_warn(&client->dev, "addr 0x%02x (expected 0x68)\n", client->addr);

	m = devm_kzalloc(&client->dev, sizeof(*m), GFP_KERNEL);
	if (!m) return -ENOMEM;

	m->client = client;
	mutex_init(&m->lock);

	ret = mpu6050_params_wakeup(client);
	if (ret) {
		dev_err(&client->dev, "wakeup failed: %d\n", ret);
		return ret;
	}

	who = i2c_smbus_read_byte_data(client, REG_WHO_AM_I);
	if (who < 0) {
		dev_err(&client->dev, "WHO_AM_I read failed: %d\n", who);
		return who;
	}
	if ((who & 0x7E) != 0x68 && who != 0x68)
		dev_warn(&client->dev, "unexpected WHO_AM_I=0x%02x\n", who);

	if (param_dlpf < 0) param_dlpf = 0;
	if (param_dlpf > 6) param_dlpf = 6;
	if (param_smpldiv < 0) param_smpldiv = 0;
	if (param_smpldiv > 255) param_smpldiv = 255;
	if (param_accel_fs < 0) param_accel_fs = 0;
	if (param_accel_fs > 3) param_accel_fs = 0;
	if (param_gyro_fs  < 0) param_gyro_fs  = 0;
	if (param_gyro_fs  > 3) param_gyro_fs  = 0;

	mutex_lock(&m->lock);
	ret = mpu_set_lpf(client, (u32)param_dlpf);
	if (ret) { 
		mutex_unlock(&m->lock); 
		return ret; 
	}
	
	ret = i2c_smbus_write_byte_data(client, REG_SMPLRT_DIV, (u8)param_smpldiv);
	if (ret) {
		mutex_unlock(&m->lock);
		return ret; 
	}
	
	ret = update_bits(client, REG_GYRO_CONFIG,  FS_MASK, (u8)(param_gyro_fs  << GYRO_FS_SHIFT));
	if (ret) { 
		mutex_unlock(&m->lock); 
		return ret; 
	}
	
	ret = update_bits(client, REG_ACCEL_CONFIG, FS_MASK, (u8)(param_accel_fs << ACCEL_FS_SHIFT));
	mutex_unlock(&m->lock);
	if (ret) 
		return ret;

	m->miscdev.minor = MISC_DYNAMIC_MINOR;
	m->miscdev.name  = DRV_NAME;
	m->miscdev.fops  = &fops;

	ret = misc_register(&m->miscdev);
	if (ret) {
		dev_err(&client->dev, "misc_register failed: %d\n", ret);
		return ret;
	}

	i2c_set_clientdata(client, m);
	dev_info(&client->dev, "ready: /dev/%s (addr=0x%02x, WHO_AM_I=0x%02x, DLPF=%d, DIV=%d)\n", m->miscdev.name, client->addr, who, param_dlpf, param_smpldiv);
	return 0;
}

static void mpu_remove(struct i2c_client *client)
{
	struct mpu6050_params *m = i2c_get_clientdata(client);
	misc_deregister(&m->miscdev);
}

static const struct of_device_id mpu_of_match[] = {
	{ .compatible = "siyoung,mpu6050_params" },
	{ }
};
MODULE_DEVICE_TABLE(of, mpu_of_match);

static const struct i2c_device_id mpu_ids[] = {
	{ DRV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mpu_ids);

static struct i2c_driver mpu_driver = {
	.driver = {
		.name = DRV_NAME,
		.of_match_table = mpu_of_match,
	},
	.probe    = mpu_probe,
	.remove   = mpu_remove,
	.id_table = mpu_ids,
};

module_i2c_driver(mpu_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MPU6050 driver w.ioctl, params");
MODULE_AUTHOR("you");
