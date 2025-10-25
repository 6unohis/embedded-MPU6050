// mpu6050_params_drv.c
// SPDX-License-Identifier: GPL-2.0
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/ioctl.h>
#include <linux/compat.h>
#include <linux/ktime.h>
#include <linux/bitops.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>
#include <linux/poll.h>
#include <linux/spinlock.h>
#include <linux/wait.h>

#include "mpu6050_irq.h"

struct mpu6050_sample {
	u64 ts_ns;
	s16 ax, ay, az;
	s16 gx, gy, gz;
	s16 temp;
} __aligned(8);

#define FIFO_DEPTH 256

struct mpu6050_params{
	struct i2c_client    *client;
	struct mpu6050_sample sample;
	struct miscdevice     miscdev;
	struct mutex          lock;

	/* IRQ 모드 관련 */
	bool                  use_irq;
	int                   irq;
	DECLARE_KFIFO(fifo, struct mpu6050_sample, FIFO_DEPTH);
	wait_queue_head_t     wq;
	spinlock_t            fifo_lock;
	atomic_t              overrun_cnt;
};

/* ---- module parameters (초기 설정값) ---- */
static int param_gyro_fs  = 0; /* 0:±250,1:±500,2:±1000,3:±2000(dps) */
static int param_accel_fs = 0; /* 0:±2,  1:±4,  2:±8,   3:±16  (g)   */
static int param_dlpf     = 3; /* 0..6 (권장:3=~42Hz)                */
static int param_smpldiv  = 9; /* SMPLRT_DIV (ODR = base/(1+div))    */
static int param_use_irq  = 1; /* 1=IRQ(기본), 0=polling             */

module_param(param_gyro_fs,  int, 0444);
module_param(param_accel_fs, int, 0444);
module_param(param_dlpf,     int, 0444);
module_param(param_smpldiv,  int, 0444);
module_param(param_use_irq,  int, 0644);

MODULE_PARM_DESC(param_gyro_fs,  "Gyro full-scale: 0:250,1:500,2:1000,3:2000 dps");
MODULE_PARM_DESC(param_accel_fs, "Accel full-scale: 0:2g,1:4g,2:8g,3:16g");
MODULE_PARM_DESC(param_dlpf,     "DLPF cfg (0..6)");
MODULE_PARM_DESC(param_smpldiv,  "SMPLRT_DIV (0..255)");
MODULE_PARM_DESC(param_use_irq,  "1: IRQ mode, 0: polling mode");

/* ---- 비트/마스크 ---- */
#define GYRO_FS_SHIFT   3
#define ACCEL_FS_SHIFT  3
#define FS_MASK         (0x3 << 3)

/* ---- 공통 헬퍼 ---- */
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

/* 현재 DLPF에 따른 내부 기준 샘플레이트(Hz) 반환: DLPF=0이면 8k, 그 외 1k */
static int current_base_rate_hz(struct i2c_client *client)
{
	int cfg = i2c_smbus_read_byte_data(client, REG_CONFIG);
	if (cfg < 0) return cfg;
	return ((cfg & 0x07) == 0) ? 8000 : 1000;
}

/* ---- 전원/클럭 ---- */
static int set_sleep(struct i2c_client *client, bool en)
{
	return update_bits(client, REG_PWR_MGMT_1, BIT(6), en ? BIT(6) : 0);
}

static int set_clk_source_pll(struct i2c_client *client)
{
	/* PWR_MGMT_1.CLKSEL = 1 (PLL X gyro) */
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

/* ---- IRQ 핸들러 ---- */
#define DATA_RDY_EN        BIT(0)
#define INT_RD_CLEAR       BIT(4)
#define LATCH_INT_EN       BIT(5)

static irqreturn_t mpu_irq_thread(int irq, void *dev_id)
{
	struct mpu6050_params *m = dev_id;
	u8 raw[14];
	struct mpu6050_sample s;
	unsigned long flags;

	/* 데이터 준비됨: 14바이트 블록 읽기 */
	if (read_block(m->client, REG_ACCEL_XOUT_H, raw, sizeof(raw)))
		return IRQ_HANDLED;

	s.ax    = (s16)((raw[0]  << 8) | raw[1]);
	s.ay    = (s16)((raw[2]  << 8) | raw[3]);
	s.az    = (s16)((raw[4]  << 8) | raw[5]);
	s.temp  = (s16)((raw[6]  << 8) | raw[7]);
	s.gx    = (s16)((raw[8]  << 8) | raw[9]);
	s.gy    = (s16)((raw[10] << 8) | raw[11]);
	s.gz    = (s16)((raw[12] << 8) | raw[13]);
	s.ts_ns = ktime_get_ns();

	spin_lock_irqsave(&m->fifo_lock, flags);
	if (!kfifo_is_full(&m->fifo))
		kfifo_in(&m->fifo, &s, 1);
	else
		atomic_inc(&m->overrun_cnt);
	spin_unlock_irqrestore(&m->fifo_lock, flags);

	wake_up_interruptible(&m->wq);
	return IRQ_HANDLED;
}

/* ---- file ops ---- */
static int mpu_open(struct inode *ino, struct file *f)
{
	/* misc_open이 f->private_data = &miscdev 로 세팅 → 우리 구조체로 변경 */
	struct mpu6050_params *m =
		container_of(f->private_data, struct mpu6050_params, miscdev);
	/* 스트리밍 장치로 쓰기 위해 pos 초기화(EOF 방지) */
	f->f_pos = 0;
	return 0;
}

static ssize_t mpu6050_params_read(struct file *f, char __user *buf, size_t len, loff_t *pos)
{
	struct mpu6050_params *m = container_of(f->private_data, struct mpu6050_params, miscdev);
	u8 raw[14];
	int ret;

	if (len < sizeof(m->sample))
		return -EINVAL;

	if (m->use_irq) {
		struct mpu6050_sample s;
		unsigned long flags;

		/* 데이터 없으면 대기(논블록이면 EAGAIN) */
		if (kfifo_is_empty(&m->fifo)) {
			if (f->f_flags & O_NONBLOCK) return -EAGAIN;
			if (wait_event_interruptible(m->wq, !kfifo_is_empty(&m->fifo)))
				return -ERESTARTSYS;
		}

		spin_lock_irqsave(&m->fifo_lock, flags);
		if (!kfifo_out(&m->fifo, &s, 1)) {
			spin_unlock_irqrestore(&m->fifo_lock, flags);
			return -EIO;
		}
		spin_unlock_irqrestore(&m->fifo_lock, flags);

		if (copy_to_user(buf, &s, sizeof(s)))
			return -EFAULT;

		return sizeof(s);
	}

	/* polling path: 즉시 I2C 읽기 (스트리밍, EOF 없음) */
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
	if (ret) return ret;

	if (copy_to_user(buf, &m->sample, sizeof(m->sample)))
		return -EFAULT;

	return sizeof(m->sample);
}

static __poll_t mpu6050_params_poll(struct file *f, poll_table *wait)
{
	struct mpu6050_params *m = container_of(f->private_data, struct mpu6050_params, miscdev);
	__poll_t mask = 0;

	if (m->use_irq) {
		poll_wait(f, &m->wq, wait);
		if (!kfifo_is_empty(&m->fifo))
			mask |= POLLIN | POLLRDNORM;
	} else {
		/* polling 모드에선 read가 즉시 가능하다고 가정 */
		mask |= POLLIN | POLLRDNORM;
	}
	return mask;
}

/* ---- 설정 함수들 ---- */

/* ODR 설정: DLPF 상태에 따라 base=1k/8k, SMPLRT_DIV = base/odr - 1 (반올림) */
static int mpu_set_odr(struct i2c_client *c, u32 odr_hz, bool dlpf_on)
{
	int base, div;

	if (!odr_hz) return -EINVAL;
	base = dlpf_on ? 1000 : 8000;

	div = (base + odr_hz/2) / odr_hz;
	if (div <= 0)  div = 1;     /* odr > base */
	if (div > 256) div = 256;   /* odr 너무 낮음 */
	div -= 1;

	return i2c_smbus_write_byte_data(c, REG_SMPLRT_DIV, (u8)div);
}

/* DLPF 설정: CONFIG[2:0] */
static int mpu_set_lpf(struct i2c_client *c, u32 dlpf_cfg /*0..6*/)
{
	if (dlpf_cfg > 6) return -EINVAL;
	return update_bits(c, REG_CONFIG, 0x07, (u8)(dlpf_cfg & 0x7));
}

/* ACCEL full-scale: ACCEL_CONFIG[4:3] */
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

/* GYRO full-scale: GYRO_CONFIG[4:3] */
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

/* ---- ioctl ---- */
static long mpu6050_params_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	struct mpu6050_params *m =
		container_of(f->private_data, struct mpu6050_params, miscdev);
	struct i2c_client *client = m->client;

	if (_IOC_TYPE(cmd) != MPU_IOC_MAGIC || _IOC_NR(cmd) > MPU6050_IOC_MAXNR)
		return -ENOTTY;

	switch (cmd) {
	case MPU_IOC_SET_ODR: {
		u32 odr;
		int base, ret;

		if (copy_from_user(&odr, (void __user *)arg, sizeof(odr)))
			return -EFAULT;
		if (!odr) return -EINVAL;

		mutex_lock(&m->lock);
		base = current_base_rate_hz(client);
		if (base < 0) { mutex_unlock(&m->lock); return base; }
		ret = mpu_set_odr(client, odr, base == 1000);
		mutex_unlock(&m->lock);
		return ret;
	}
	case MPU_IOC_SET_LPF: {
		u32 dlpf;
		int ret;

		if (copy_from_user(&dlpf, (void __user *)arg, sizeof(dlpf)))
			return -EFAULT;
		if (dlpf > 6) return -EINVAL;

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
	case MPU_IOC_SET_IRQ_EN: {
		u32 en;
		int ret = 0;

		if (copy_from_user(&en, (void __user *)arg, sizeof(en)))
			return -EFAULT;

		if (!!en == m->use_irq)
			return 0; /* 변화 없음 */

		if (en) {
			/* FIFO/WQ 초기화 후 INT enable */
			unsigned long flags;
			spin_lock_irqsave(&m->fifo_lock, flags);
			kfifo_reset(&m->fifo);
			spin_unlock_irqrestore(&m->fifo_lock, flags);
			atomic_set(&m->overrun_cnt, 0);
			/* INT enable */
			ret = i2c_smbus_write_byte_data(client, REG_INT_ENABLE, DATA_RDY_EN);
			if (!ret) m->use_irq = true;
		} else {
			/* INT disable */
			ret = i2c_smbus_write_byte_data(client, REG_INT_ENABLE, 0x00);
			m->use_irq = false;
		}
		return ret;
	}
	default:
		return -ENOTTY;
	}
}

#ifdef CONFIG_COMPAT
static long mpu6050_params_compat_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	return mpu6050_params_ioctl(f, cmd, (unsigned long)compat_ptr(arg));
}
#endif

static const struct file_operations fops = {
	.owner          = THIS_MODULE,
	.open           = mpu_open,
	.read           = mpu6050_params_read,
	.poll           = mpu6050_params_poll,
	.unlocked_ioctl = mpu6050_params_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl   = mpu6050_params_compat_ioctl,
#endif
};

/* ---- I2C probe/remove ---- */
static int mpu_probe(struct i2c_client *client)
{
	struct mpu6050_params *m;
	int who, ret;

	/* 어댑터 기능 체크 */
	if (!i2c_check_functionality(client->adapter,
		I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_I2C_BLOCK)) {
		dev_err(&client->dev, "SMBus byte/block not supported\n");
		return -EOPNOTSUPP;
	}

	/* 주소 허용: 0x68/0x69 */
	if (client->addr != 0x68 && client->addr != 0x69)
		return -ENODEV;
	if (client->addr != MPU_I2C_ADDR)
		dev_warn(&client->dev, "addr 0x%02x (expected 0x68)\n", client->addr);

	m = devm_kzalloc(&client->dev, sizeof(*m), GFP_KERNEL);
	if (!m) return -ENOMEM;

	m->client = client;
	mutex_init(&m->lock);
	spin_lock_init(&m->fifo_lock);
	init_waitqueue_head(&m->wq);
	INIT_KFIFO(m->fifo);
	atomic_set(&m->overrun_cnt, 0);
	m->use_irq = !!param_use_irq;

	ret = mpu6050_params_wakeup(client);
	if (ret) {
		dev_err(&client->dev, "wakeup failed: %d\n", ret);
		return ret;
	}

	/* WHO_AM_I 확인 */
	who = i2c_smbus_read_byte_data(client, REG_WHO_AM_I);
	if (who < 0) {
		dev_err(&client->dev, "WHO_AM_I read failed: %d\n", who);
		return who;
	}
	if ((who & 0x7E) != 0x68 && who != 0x68)
		dev_warn(&client->dev, "unexpected WHO_AM_I=0x%02x\n", who);

	/* 모듈 파라미터 기반 초기 설정 */
	if (param_dlpf < 0) param_dlpf = 0;
	if (param_dlpf > 6) param_dlpf = 6;
	if (param_smpldiv < 0) param_smpldiv = 0;
	if (param_smpldiv > 255) param_smpldiv = 255;
	if (param_accel_fs < 0) param_accel_fs = 0;
	if (param_accel_fs > 3) param_accel_fs = 0;
	if (param_gyro_fs  < 0) param_gyro_fs  = 0;
	if (param_gyro_fs  > 3) param_gyro_fs  = 0;

	mutex_lock(&m->lock);
	/* LPF */
	ret = mpu_set_lpf(client, (u32)param_dlpf);
	if (ret) { mutex_unlock(&m->lock); return ret; }
	/* ODR: 직접 DIV 쓰기 (기본은 1kHz base 가정; DLPF=0이면 8k) */
	ret = i2c_smbus_write_byte_data(client, REG_SMPLRT_DIV, (u8)param_smpldiv);
	if (ret) { mutex_unlock(&m->lock); return ret; }
	/* 풀스케일 */
	ret = update_bits(client, REG_GYRO_CONFIG,  FS_MASK, (u8)(param_gyro_fs  << GYRO_FS_SHIFT));
	if (ret) { mutex_unlock(&m->lock); return ret; }
	ret = update_bits(client, REG_ACCEL_CONFIG, FS_MASK, (u8)(param_accel_fs << ACCEL_FS_SHIFT));
	mutex_unlock(&m->lock);
	if (ret) return ret;

	/* IRQ 요청(선택): DT에서 .irq가 들어와 있어야 함 */
	m->irq = client->irq;
	if (m->use_irq) {
		if (m->irq <= 0) {
			dev_warn(&client->dev, "no IRQ in DT; falling back to polling\n");
			m->use_irq = false;
		} else {
			int flags = IRQF_ONESHOT; /* 트리거 타입은 DT에서 가져감 */
			ret = devm_request_threaded_irq(&client->dev, m->irq,
					NULL, mpu_irq_thread, flags, DRV_NAME, m);
			if (ret) {
				dev_err(&client->dev, "irq request failed: %d\n", ret);
				return ret;
			}
			/* INT enable */
			ret = i2c_smbus_write_byte_data(client, REG_INT_ENABLE, DATA_RDY_EN);
			if (ret) {
				dev_err(&client->dev, "INT_ENABLE failed: %d\n", ret);
				return ret;
			}
		}
	} else {
		/* polling 모드: INT 끔 */
		i2c_smbus_write_byte_data(client, REG_INT_ENABLE, 0x00);
	}

	/* misc 등록 */
	m->miscdev.minor = MISC_DYNAMIC_MINOR;
	m->miscdev.name  = DRV_NAME;
	m->miscdev.fops  = &fops;

	ret = misc_register(&m->miscdev);
	if (ret) {
		dev_err(&client->dev, "misc_register failed: %d\n", ret);
		return ret;
	}

	i2c_set_clientdata(client, m);
	dev_info(&client->dev, "ready: /dev/%s (addr=0x%02x, WHO_AM_I=0x%02x, DLPF=%d, DIV=%d, IRQ=%s)\n",
		 m->miscdev.name, client->addr, who, param_dlpf, param_smpldiv,
		 m->use_irq ? "on" : "off");
	return 0;
}

static void mpu_remove(struct i2c_client *client)
{
	struct mpu6050_params *m = i2c_get_clientdata(client);
	/* INT off */
	i2c_smbus_write_byte_data(client, REG_INT_ENABLE, 0x00);
	misc_deregister(&m->miscdev);
}

static const struct of_device_id mpu_of_match[] = {
	{ .compatible = "siyoung,mpu6050_irq" },
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
MODULE_DESCRIPTION("MPU6050 driver (ioctl + IRQ/poll toggle + kfifo + poll)");
MODULE_AUTHOR("siyoung");
