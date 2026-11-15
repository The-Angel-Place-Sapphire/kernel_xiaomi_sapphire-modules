/*
 *  w1_slg.c - Aisinochip SLG driver
 *
 * Copyright (c) Shanghai Aisinochip Electronics Techology Co., Ltd.
 *
 * Sapphire (khaje) version: the one-wire bus is bit-banged directly on the
 * TLMM GPIO66 registers (as the stock sapphire slg_verify.ko does), the
 * device is only registered when no "main_suppiler" authenticator exists and
 * the SLG ROM ID is recognised, and the auth_ops carry probe_done/get_chip_ok.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/crc16.h>
#include <linux/uaccess.h>
#include <linux/scatterlist.h>
#include <linux/of_gpio.h>
#include <linux/random.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/ktime.h>
#include <linux/timekeeping.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <crypto/akcipher.h>
#include <crypto/rng.h>
#include <crypto/algapi.h>
#include <crypto/hash.h>
#include <linux/gpio/consumer.h>
#include "battery_auth_class.h"
#include "slglib/uECC.h"

#define CRC16_INIT      0

//#define W1_FAMILY_SLG               0xAC

#define SLG_MEMORY_READ             0xCA
#define SLG_ECDSA_SIGN              0x2A
#define SLG_DC_DECREASE             0x24
#define SLG_DC_GET                  0xC0
#define SLG_ROMID_GET               0xB2
#define SLG_MANDI_GET               0x4F
#define SLG_POWEROFF_MODE           0x58
#define SLG_ECDSA_DISABLE           0xFC
#define SLG_CHIP_KILL               0xFE

#define RESPONSE_SUCCESS            0xAA

#define W1_BUS_ERROR                -201
#define COMMAND_ERROR               -202
#define PARAM_ERROR                 -203
#define RECV_CRC_ERROR              -204
#define RECV_LENGTH_ERROR           -205

#define SLG_NOT_EXIST               -210
#define SHASH_ALLOC_FAILED          -211
#define AKCIPHER_ALLOC_FAILED       -212
#define AKCIPHER_REQUEST_FAILED     -213

#define ECC_SIGN_WAIT_MAX           80
#define ECC_VERIFY_WAIT_MAX         150
#define MEMORY_WRITE_WAIT           100
#define GENERAL_WAIT_MAX            10

#define SLG_CMD_RETRY_MAX           10

static int cmd_need_wait;

struct result_data_struct {
	unsigned char result_buf[128];
	int result_len;
	int status;
};

struct slg_data {
	struct platform_device *pdev;
	struct device *dev;
	const char *auth_name;
	struct auth_device *auth_dev;
	uint8_t batt_id;
	int probe_done;
	bool romid_verified;
};

static struct slg_data *g_info;

DEFINE_MUTEX(slg_cmd_lock);	/* stock: zeroed, never initialised */
static wait_queue_head_t slg_wait;
static DEFINE_SPINLOCK(w1_lock);

/* khaje TLMM GPIO66 (one-wire data line) */
#define GPIO66_CFG_REG_PHYS             0x542000
#define GPIO_CFG_OUTPUT                 0x3c0
#define GPIO_CFG_INPUT                  0x1c0
#define GPIO_OUT_HIGH                   0x2
#define GPIO_OUT_LOW                    0x0

static void __iomem *gpio_66_cfg;
static void __iomem *gpio_66_inout;

#define ONE_WIRE_CONFIG_OUT     (*(volatile u32 __force *)(gpio_66_cfg) = (GPIO_CFG_OUTPUT))
#define ONE_WIRE_CONFIG_IN      (*(volatile u32 __force *)(gpio_66_cfg) = (GPIO_CFG_INPUT))
#define ONE_WIRE_OUT_HIGH					\
	do {							\
		(*(volatile u32 __force *)(gpio_66_inout) = (GPIO_OUT_HIGH));	\
		wmb();						\
		(*(volatile u32 __force *)(gpio_66_inout) = (GPIO_OUT_HIGH));	\
		wmb();						\
	} while (0)
#define ONE_WIRE_OUT_LOW					\
	do {							\
		(*(volatile u32 __force *)(gpio_66_inout) = (GPIO_OUT_LOW));	\
		wmb();						\
		(*(volatile u32 __force *)(gpio_66_inout) = (GPIO_OUT_LOW));	\
		wmb();						\
	} while (0)
#define ONE_WIRE_GPIO_READ      ((*(volatile u32 __force *)gpio_66_inout) & 0x1)

#define WRITE_ONE_OUTPUT_H              (15000L)

#define WRITE_ZERO_OUTPUT_L             (10000L)
#define WRITE_ZERO_OUTPUT_H             (7000L)

#define READ_BIT_OUTPUT_L               (500L)
#define READ_BIT_END                    (16000L)

#define W1_BUS_RESET_DELAY              (70)
#define W1_BUS_RESET_COUNT              3

#define W1_SKIP_ROM		0xCC

static inline void w1_delay_ns(u64 ns)
{
	u64 t = ktime_get_boottime_ns();

	while (ktime_get_boottime_ns() - t < ns)
		;
}

/*write/read ops*/
static void w1_write_bit(u8 bit)
{
	unsigned long flags = 0;

	ONE_WIRE_CONFIG_OUT;
	spin_lock_irqsave(&w1_lock, flags);

	ONE_WIRE_OUT_LOW;
	if (bit) {
		ONE_WIRE_OUT_HIGH;
		w1_delay_ns(WRITE_ONE_OUTPUT_H);
	} else {
		//keep to low
		w1_delay_ns(WRITE_ZERO_OUTPUT_L);
		ONE_WIRE_OUT_HIGH;
		w1_delay_ns(WRITE_ZERO_OUTPUT_H);
	}

	spin_unlock_irqrestore(&w1_lock, flags);
}

static u8 w1_read_bit(void)
{
	u8 result;
	unsigned long flags = 0;

	/* sample timing is critical here */
	ONE_WIRE_CONFIG_OUT;
	spin_lock_irqsave(&w1_lock, flags);

	ONE_WIRE_OUT_LOW;
	w1_delay_ns(READ_BIT_OUTPUT_L);

	ONE_WIRE_CONFIG_IN;
	result = ONE_WIRE_GPIO_READ;
	w1_delay_ns(READ_BIT_END);

	spin_unlock_irqrestore(&w1_lock, flags);

	return result;
}

static void w1_write_8(u8 byte)
{
	int i;

	for (i = 0; i < 8; ++i)
		w1_write_bit((byte >> i) & 0x1);
}

static void w1_write_block(const u8 * buf, int len)
{
	int i;

	for (i = 0; i < len; ++i)
		w1_write_8(buf[i]);
}

static u8 w1_read_8(void)
{
	int i;
	int res = 0;

	for (i = 0; i < 8; ++i)
		res |= (w1_read_bit() << i);

	return res;
}

static u8 w1_read_block(u8 * buf, int len)
{
	int i;
	int ret;

	for (i = 0; i < len; ++i)
		buf[i] = w1_read_8();
	ret = len;

	return ret;
}

static void wait_slave_release(void)
{
	int counter;
	int result;

	ONE_WIRE_CONFIG_IN;
	for (counter = 0; counter < 200; counter++) {
		result = ONE_WIRE_GPIO_READ;
		if (result == 1) {
			break;
		}

		udelay(1);
	}
}

static int w1_reset_bus(void)
{
	int result = 1;
	int counter = 0;
	int retry = W1_BUS_RESET_COUNT;
	int poweroff = 0;
      RESET_AGAIN:
	ONE_WIRE_CONFIG_OUT;
	ONE_WIRE_OUT_LOW;
	//delay 70us for 62.5kbps
	udelay(W1_BUS_RESET_DELAY);
	ONE_WIRE_OUT_HIGH;
	ONE_WIRE_CONFIG_IN;

	for (counter = 0; counter < 100; counter++) {
		result = ONE_WIRE_GPIO_READ;
		if (result == 0) {
			break;
		}
	}
	udelay(1);

	if (result == 0) {
		//there is a slave, wait the bus release
		wait_slave_release();
	} else {
		retry--;
		if (retry > 0) {
			msleep(2);
			goto RESET_AGAIN;
		} else {
			if (poweroff == 0) {
				printk
				    ("[slg] slg reset failed, poweroff slg and retry again\n");
				poweroff = 1;
				ONE_WIRE_CONFIG_OUT;
				ONE_WIRE_OUT_LOW;
				// delay 30ms let slg poweroff
				msleep(30);
				ONE_WIRE_OUT_HIGH;
				// delay 10ms let slg poweron
				msleep(10);
				retry = W1_BUS_RESET_COUNT;
				goto RESET_AGAIN;
			}
		}
	}

	return result;
}

static int w1_reset_select_slave(void)
{
	if (w1_reset_bus())
		return -1;

	w1_write_8(W1_SKIP_ROM);

	return 0;
}

void *hex2asc(unsigned char *dest, unsigned char *src, unsigned int len)
{
	unsigned int i;
	unsigned char *p;

	p = dest;
	if (len % 2)
		*dest++ = (*src++ & 0x0F) + 0x30;
	for (i = 0; i < (len / 2); i++) {
		*dest++ = ((*src & 0xF0) >> 4) + 0x30;
		*dest++ = (*src++ & 0x0F) + 0x30;
	}
	while (p != dest) {
		if (*p >= 0x3A)
			*p += 7;
		p++;
	}
	return ((unsigned char *) dest);
}

void printf_hex(unsigned char *output, int output_len)
{
	char buffer[1024];

	if (output_len == 0) {
		return;
	} else if (output_len > (sizeof(buffer) / 2)) {
		output_len = (sizeof(buffer) / 2);
	}

	memset(buffer, 0x00, sizeof(buffer));

	hex2asc(buffer, output, output_len << 1);

	pr_info("w1 data: %s\n", buffer);
}

/*
 * Send and Receive data by W1 bus
 *
 * @param input: Send data buffer
 * @param input_len: Send data length
 * @param output: Received data buffer
 * @param output_len: Received data length
 * @return: 0=Success; others=failure, see Error code
 *
 */
static int bus_send_recv(unsigned char *input, int input_len,
			 unsigned char *output, int *output_len)
{
	unsigned char buffer[512] = {0};
	unsigned char recv[512];
	unsigned short calc_crc;
	unsigned short recv_crc;
	int index, len;
	int ret;
	unsigned short retry = SLG_CMD_RETRY_MAX;

	if (input_len > 500) {
		return PARAM_ERROR;
	}

	memset(buffer, 0, sizeof(buffer));
	memset(recv, 0, sizeof(recv));

	memcpy(buffer, input, input_len);
	index = input_len;

	calc_crc = crc16(CRC16_INIT, buffer, index);
	calc_crc ^= 0xFFFF;
	buffer[index++] = calc_crc >> 8;
	buffer[index++] = calc_crc & 0xFF;

	mutex_lock(&slg_cmd_lock);

      RETRY:
	if (w1_reset_select_slave()) {
		pr_err("w1_reset_select_slave failed\n");
		ret = W1_BUS_ERROR;
		goto END;
	}
	w1_write_block(buffer, index);

	mdelay(cmd_need_wait);

	memset(recv, 0, sizeof(recv));
	w1_read_block(recv, 1);

	len = recv[0];
	if (len <= 200) {
		w1_read_block(&recv[1], len + 2);
	} else {
		len = 0;
	}

	pr_info("w1 send data:\n");
	printf_hex(buffer, index);
	pr_info("w1 read length data: %02X\n", recv[0]);

	if (len != 0) {
		pr_info("w1 recv data:\n");
		printf_hex(recv, len + 3);

		calc_crc = crc16(CRC16_INIT, recv, len + 1);
		calc_crc ^= 0xFFFF;

		recv_crc = (recv[len + 1] << 8) + recv[len + 2];

		pr_info("w1 recv crc %X, clac crc: %X\n", recv_crc,
		      calc_crc);

		if (recv_crc == calc_crc) {
			pr_info("w1 recv success\n");
			ret = 0;
			memcpy(output, &recv[1], len);
			*output_len = len;

			if (recv[1] == 0x22) {
				ret = RECV_CRC_ERROR;
			}
		} else {
			ret = RECV_CRC_ERROR;
		}
	} else {
		ret = RECV_LENGTH_ERROR;
	}

      END:
	if (((ret == RECV_CRC_ERROR) || (ret == RECV_LENGTH_ERROR))
	    && (retry > 0)) {
		printk("[slg] command send failed retry %d\n",
		       SLG_CMD_RETRY_MAX - retry + 1);
		wait_event_timeout(slg_wait, 0, msecs_to_jiffies(100));
		retry--;
		goto RETRY;
	}

	mutex_unlock(&slg_cmd_lock);

	return ret;
}

/*
 * Read a page of memory data from SLG
 *
 * @param index: The memory page index
 * @param output: Memory data buffer
 * @param output_len: Memory data length
 * @return: 0=Success; others=failure, see Error code
 */
int slg_memory_read(int index, unsigned char *output, int *output_len)
{
	unsigned char send[8] = {0};
	unsigned char recv[64];
	int ret;
	int recv_len = 0;

	cmd_need_wait = GENERAL_WAIT_MAX;
	*output_len = 0;

	pr_info("index = %d\n", index);

	send[0] = SLG_MEMORY_READ;
	send[1] = 1;
	send[2] = index;

	memset(recv, 0xFF, sizeof(recv));
	ret = bus_send_recv(send, 3, recv, &recv_len);

	if (ret == 0) {
		if (recv[0] != RESPONSE_SUCCESS) {
			ret = -recv[0];
		} else {
			memcpy(output, &recv[1], recv_len - 1);
			*output_len = recv_len - 1;
		}
	}

	return ret;
}

/*
 * Do ECDSA sign the data by SLG
 *
 * @param input: The additional data to sign
 * @param input_len: The length of the additional data
 * @param output: The signature data, r and s
 * @param output_len: The length of the signature data
 * @return: 0=Success; others=failure, see Error code
 */
int slg_ecdsa_sign(unsigned char *input, int input_len,
		   unsigned char *output, int *output_len)
{
	unsigned char send[128] = {0};
	unsigned char recv[128];
	int ret;
	int recv_len = 0;
	int index = 0;

	cmd_need_wait = ECC_SIGN_WAIT_MAX;
	*output_len = 0;

	send[index++] = SLG_ECDSA_SIGN;
	send[index++] = input_len;
	memcpy(&send[index], input, input_len);
	index += input_len;

	memset(recv, 0xFF, sizeof(recv));
	ret = bus_send_recv(send, index, recv, &recv_len);

	if (ret == 0) {
		if (recv[0] != RESPONSE_SUCCESS) {
			ret = -recv[0];
		} else {
			memcpy(output, &recv[1], recv_len - 1);
			*output_len = recv_len - 1;
		}
	}

	return ret;
}

/*
 * Get the Decrement Counter
 *
 * @param counter: The initial value of the DC
 * @return: postive=The DC value; negative=failure, see Error code
 */
int slg_dc_get(void)
{
	unsigned char send[8] = {0};
	unsigned char recv[8];
	int ret;
	int recv_len;

	cmd_need_wait = GENERAL_WAIT_MAX;

	send[0] = SLG_DC_GET;
	send[1] = 0x00;

	memset(recv, 0xFF, sizeof(recv));
	ret = bus_send_recv(send, 2, recv, &recv_len);

	if (ret == 0) {
		if (recv[0] != RESPONSE_SUCCESS) {
			ret = -recv[0];
		} else {
			ret =
			    (recv[1] << 24) + (recv[2] << 16) +
			    (recv[3] << 8) + (recv[4]);
		}
	}

	return ret;
}

/*
 * Do decrease the Decrement Counter
 *
 * @return: 0=Success; others=failure, see Error code
 */
int slg_dc_decrease(void)
{
	unsigned char send[8] = {0};
	unsigned char recv[8];
	int ret;
	int recv_len;

	cmd_need_wait = GENERAL_WAIT_MAX;

	send[0] = SLG_DC_DECREASE;
	send[1] = 0x00;

	memset(recv, 0xFF, sizeof(recv));
	ret = bus_send_recv(send, 2, recv, &recv_len);

	if (ret == 0) {
		if (recv[0] != RESPONSE_SUCCESS) {
			ret = -recv[0];
		}
	}

	return ret;
}

/*
 * Get ROM ID
 *
 * @param output: ROM ID data buffer
 * @param output_len: 8 bytes
 * @return: 0=Success; others=failure, see Error code
 */
int slg_romid_get(unsigned char *output, int *output_len)
{
	unsigned char send[4] = {0};
	unsigned char recv[64];
	int ret;
	int recv_len;

	cmd_need_wait = GENERAL_WAIT_MAX;
	*output_len = 0;

	send[0] = SLG_ROMID_GET;
	send[1] = 0x00;

	memset(recv, 0xFF, sizeof(recv));
	ret = bus_send_recv(send, 2, recv, &recv_len);

	if (ret == 0) {
		if (recv[0] != RESPONSE_SUCCESS) {
			ret = -recv[0];
		} else {
			memcpy(output, &recv[1], 8);
			*output_len = 8;
		}
	}

	return ret;
}

/*
 * Get Manufacture ID
 *
 * @param output: manufacture data buffer
 * @param output_len: 2 bytes
 * @return: 0=Success; others=failure, see Error code
 */
int slg_manid_get(unsigned char *output, int *output_len)
{
	unsigned char send[4] = {0};
	unsigned char recv[64];
	int ret;
	int recv_len;

	cmd_need_wait = GENERAL_WAIT_MAX;
	*output_len = 0;

	send[0] = SLG_ROMID_GET;
	send[1] = 0x00;

	memset(recv, 0xFF, sizeof(recv));
	ret = bus_send_recv(send, 2, recv, &recv_len);

	if (ret == 0) {
		if (recv[0] != RESPONSE_SUCCESS) {
			ret = -recv[0];
		} else {
			memcpy(output, &recv[1 + 8], 2);
			*output_len = 2;
		}
	}

	return ret;
}

/*
 * Set SLG to Poweroff Mode
 *
 * @return: 0=Success; others=failure, see Error code
 */
int slg_poweroff_mode(void)
{
	unsigned char send[8] = {0};
	unsigned char recv[64];
	int ret;
	int recv_len;

	cmd_need_wait = GENERAL_WAIT_MAX;

	send[0] = 0x58;
	send[1] = 0x02;
	send[2] = 0x5A;
	send[3] = 0xA5;

	memset(recv, 0xFF, sizeof(recv));
	ret = bus_send_recv(send, 4, recv, &recv_len);

	if (ret == 0) {
		if (recv[0] != RESPONSE_SUCCESS) {
			ret = -recv[0];
		}
	}

	return ret;
}

/*
 * Disable SLG ECDSA function
 *
 * @return: postive=Success; negative=failure, see Error code
 */
int slg_ecdsa_disable(void)
{
	unsigned char ecdsa_disable[] =
	    { 0x44, 0x69, 0x73, 0x5F, 0x45, 0x43, 0x44, 0x53, 0x41 };
	unsigned char send[64] = {0};
	unsigned char recv[64];
	int ret;
	int recv_len;
	int index = 0;

	cmd_need_wait = GENERAL_WAIT_MAX;

	send[index++] = SLG_ECDSA_DISABLE;
	send[index++] = sizeof(ecdsa_disable);
	memcpy(&send[index], ecdsa_disable, sizeof(ecdsa_disable));
	index += sizeof(ecdsa_disable);

	memset(recv, 0xFF, sizeof(recv));
	ret = bus_send_recv(send, index, recv, &recv_len);

	if (ret == 0) {
		if (recv[0] != RESPONSE_SUCCESS) {
			ret = -recv[0];
		}
	}

	return ret;
}

/*
 * Kill SLG chip
 *
 * @return: postive=Success; negative=failure, see Error code
 */
int slg_chip_kill(void)
{
	unsigned char chip_kill[] =
	    { 0x44, 0x69, 0x73, 0x44, 0x65, 0x76, 0x69, 0x63, 0x65 };
	unsigned char send[64] = {0};
	unsigned char recv[64];
	int ret;
	int recv_len;
	int index = 0;

	cmd_need_wait = GENERAL_WAIT_MAX;

	send[index++] = SLG_CHIP_KILL;
	send[index++] = sizeof(chip_kill);
	memcpy(&send[index], chip_kill, sizeof(chip_kill));
	index += sizeof(chip_kill);

	memset(recv, 0xFF, sizeof(recv));
	ret = bus_send_recv(send, index, recv, &recv_len);

	if (ret == 0) {
		if (recv[0] != RESPONSE_SUCCESS) {
			ret = -recv[0];
		}
	}

	return ret;
}

/*
 * Compute SHA256
 *
 * @param data: messgee data buffer
 * @param datalen: messgee data length
 * @param digest: the result of SHA256
 * @return: 0=Success; others=failure, see Error code
 */
static int sha256_compute(const unsigned char *data, unsigned int datalen,
			  unsigned char *digest)
{
	struct crypto_shash *alg;
	char *hash_alg_name = "sha256-generic";
	int ret;

	struct sdesc {
		struct shash_desc shash;
		char ctx[];
	};

	struct sdesc *sdesc;
	int size;

	alg = crypto_alloc_shash(hash_alg_name, 0, 0);
	if (IS_ERR(alg)) {
		pr_err("can't alloc alg %s\n", hash_alg_name);
		return SHASH_ALLOC_FAILED;
	}

	size = sizeof(struct shash_desc) + crypto_shash_descsize(alg);
	sdesc = kmalloc(size, GFP_KERNEL);
	if (!sdesc) {
		crypto_free_shash(alg);
		return -ENOMEM;
	}

	sdesc->shash.tfm = alg;

	ret = crypto_shash_digest(&sdesc->shash, data, datalen, digest);

	kfree(sdesc);

	crypto_free_shash(alg);

	return ret;
}

/*
 * Verify ECDSA signature
 *
 * @param pub_key: public key buffer, 32 bytes x and 32 bytes y
 * @param msg: messgee data buffer
 * @param msg_len: messgee data length
 * @param sig: the message signature, 32 bytes r and 32 bytes s
 * @return: 0=Success; others=failure, see Error code
 */
static int ecdsa_verify(uint8_t * pub_key, uint8_t * msg, int msg_len,
			uint8_t * sig)
{
	uint8_t msg_digest[32];
	const struct uECC_Curve_t* curve_para;
	int ret;
	curve_para = uECC_secp256r1();

	ret = sha256_compute(msg, msg_len, msg_digest);
	if (ret) {
		pr_err("%s sha256 message fail: %d\n", __func__, ret);
		return ret;
	}

	if (0 == uECC_verify(pub_key, msg_digest, 32, sig, curve_para)) {
		pr_err("%s ecc verify fail: %d\n", __func__);
		return -1;
	}

	return 0;
}

//To be signed PAGE index
static const int PAGE_INDEX = 61;

//device signer R's page number
static const int DEVICE_SIGNER_R = 59;

//device signer S's page number
static const int DEVICE_SIGNER_S = 60;

//device public key X's page number
static const int DEVICE_PUBLIC_KEY_X = 61;

//device public key Y's page number
static const int DEVICE_PUBLIC_KEY_Y = 62;

//battery id page
static const int BATT_ID = 0;

static const uint8_t MI_CA_PUBLIC_X[] = {
	0xc0, 0xe2, 0x5d, 0x5f, 0xa6, 0x9d, 0x5a, 0x3f,
	0x95, 0x63, 0xf2, 0x70, 0x03, 0x33, 0x77, 0x24,
	0x3a, 0x8e, 0x49, 0xb8, 0x65, 0x07, 0x73, 0x76,
	0xa2, 0x85, 0x9b, 0xba, 0x5a, 0x10, 0x41, 0x12
};

static const uint8_t MI_CA_PUBLIC_Y[] = {
	0x2a, 0x4e, 0xac, 0x7c, 0xc9, 0x61, 0x86, 0x28,
	0x5c, 0xf1, 0xc0, 0x34, 0x09, 0x4d, 0x32, 0x9d,
	0xa2, 0x0f, 0x54, 0xa3, 0x0b, 0x21, 0x5b, 0xd8,
	0xf1, 0xf6, 0xf3, 0x17, 0x0c, 0x60, 0xc7, 0x8a
};


static int slg_check_chip_ok(void)
{
	uint8_t romid[8];
	int len;
	int ret;
	int i;

	if (!g_info->romid_verified) {
		ret = slg_romid_get(romid, &len);
		if (ret != 0) {
			pr_err("%s %d failed: %d\n", __func__, __LINE__, ret);
			g_info->probe_done = 0;
			return ret;
		}
		for (i = 0; i < len; i++)
			pr_info("%s romid[%d]: 0x%02x\n", __func__, i, romid[i]);

		/* SLG ROM ID: family code 0xAC, 0x04, 0xF? */
		if (romid[7] == 0xAC && romid[1] == 0x04 &&
		    (romid[2] & 0xF0) == 0xF0) {
			g_info->romid_verified = true;
			g_info->probe_done = true;
		} else {
			g_info->romid_verified = false;
			g_info->probe_done = false;
		}
	}

	pr_err("%s g_info->romid_verified = %d\n", __func__,
	       g_info->romid_verified);

	return g_info->romid_verified;
}

static int slg_authenticate_battery(void)
{
	uint8_t romid[8];
	uint8_t manid[2];
	uint8_t device_pubkey[64];
	uint8_t signer[64];
	uint8_t page_data[32];
	uint8_t hrng[32];
	uint8_t tbs[128];
	uint8_t buffer[128];
	uint8_t ca_pubkey[64];
	uint8_t batt_id[32];
	int len;
	int index;
	int ret;
	int i, j;

	pr_info("%s %d battery authenticate begin\n", __func__, __LINE__);
	ret = slg_romid_get(romid, &len);
	if (ret != 0) {
		pr_err("%s %d failed: %d\n", __func__, __LINE__, ret);
		return ret;
	}
	for (i = 0; i < len; i++)
		pr_err("%s romid[%d]: 0x%02x\n", __func__, i, romid[i]);

	ret = slg_manid_get(manid, &len);
	if (ret != 0) {
		pr_err("%s %d failed: %d\n", __func__, __LINE__, ret);
		return ret;
	}
	for (j = 0; j < len; j++)
		pr_err("%s manid[%d]: 0x%02x\n", __func__, j, manid[j]);

	ret = slg_memory_read(DEVICE_PUBLIC_KEY_X, device_pubkey, &len);
	if (ret != 0) {
		pr_err("%s %d failed: %d\n", __func__, __LINE__, ret);
		return ret;
	}
	ret = slg_memory_read(DEVICE_PUBLIC_KEY_Y, &device_pubkey[32],
			      &len);
	if (ret != 0) {
		pr_err("%s %d failed: %d\n", __func__, __LINE__, ret);
		return ret;
	}
	ret = slg_memory_read(DEVICE_SIGNER_R, signer, &len);
	if (ret != 0) {
		pr_err("%s %d failed: %d\n", __func__, __LINE__, ret);
		return ret;
	}
	ret = slg_memory_read(DEVICE_SIGNER_S, &signer[32], &len);
	if (ret != 0) {
		pr_err("%s %d failed: %d\n", __func__, __LINE__, ret);
		return ret;
	}
	memcpy(ca_pubkey, MI_CA_PUBLIC_X, sizeof(MI_CA_PUBLIC_X));
	memcpy(&ca_pubkey[32], MI_CA_PUBLIC_Y, sizeof(MI_CA_PUBLIC_Y));
	//step 1. verify device cert
	index = 0;
	memcpy(&tbs[index], romid, sizeof(romid));
	index += sizeof(romid);
	memcpy(&tbs[index], manid, sizeof(manid));
	index += sizeof(manid);
	memcpy(&tbs[index], device_pubkey, sizeof(device_pubkey));
	index += sizeof(device_pubkey);

	ret = ecdsa_verify(ca_pubkey, tbs, index, signer);
	if (ret != 0) {
		pr_err("%s step 1 failed: %d\n", __func__, ret);
		return ret;
	}
	ret = slg_memory_read(PAGE_INDEX, page_data, &len); //step 2. verify device signer
	if (ret != 0) {
		pr_err("%s %d failed: %d\n", __func__, __LINE__, ret);
		return ret;
	}
	index = 0;
	get_random_bytes(hrng, 32);
	tbs[index++] = PAGE_INDEX;
	memcpy(&tbs[index], hrng, sizeof(hrng));
	index += 32;
	ret = slg_ecdsa_sign(tbs, index, buffer, &len);
	if (ret != 0) {
		pr_err("%s %d failed: %d\n", __func__, __LINE__, ret);
		return ret;
	}

	index = 0;
	memcpy(&tbs[index], romid, sizeof(romid));
	index += sizeof(romid);
	memcpy(&tbs[index], page_data, sizeof(page_data));
	index += sizeof(page_data);
	tbs[index++] = PAGE_INDEX;
	memcpy(&tbs[index], manid, sizeof(manid));
	index += sizeof(manid);
	memcpy(&tbs[index], hrng, sizeof(hrng));
	index += 32;
	ret = ecdsa_verify(device_pubkey, tbs, index, buffer);
	if (ret != 0) {
		pr_err("%s step 2 failed: %d\n", __func__, ret);
		return ret;
	}
	ret = slg_dc_decrease();
	if (ret != 0) {
		pr_err("%s %d decrease counter decrease failed: %d\n", __func__, __LINE__, ret);
		return ret;
	}

	ret = slg_memory_read(BATT_ID, batt_id, &len);
	if (ret != 0)
		pr_err("%s %d read page 0 failed: %d\n", __func__, __LINE__, ret);

	if (batt_id[0] == 0x47 || batt_id[0] == 0x45)
		g_info->batt_id = 0;
	else if (batt_id[0] == 0x4e && batt_id[1] == 0x56)
		g_info->batt_id = 1;
	else
		g_info->batt_id  = 0xff;

	pr_err("%s battery_id is %d \n", __func__, g_info->batt_id);

	pr_err("%s %d battery authenticate success\n", __func__, __LINE__);

	return ret;
}

static int slg_start_auth_battery(struct auth_device *auth_dev)
{
	int ret;

	pr_info("%s auth_battery \n", __func__);

	ret = slg_authenticate_battery();
	if (ret)
		return -1;

	return 0;
}

static int slg_get_batt_id(struct auth_device *auth_dev, u8 * id)
{
	*id = g_info->batt_id;
	pr_info("%s batt_id is %d \n", __func__, g_info->batt_id);

	return 0;
}

static int slg_probe_done(struct auth_device *auth_dev)
{
	pr_info("%s g_info->probe_done is %d \n", __func__, g_info->probe_done);

	return g_info->probe_done;
}

static int slg_get_chip_ok(struct auth_device *auth_dev)
{
	pr_info("%s chip ok \n", __func__);

	return slg_check_chip_ok();
}

struct auth_ops slg_auth_ops = {
	.auth_battery = slg_start_auth_battery,
	.get_battery_id = slg_get_batt_id,
	.probe_done = slg_probe_done,
	.get_chip_ok = slg_get_chip_ok,
};

static int slg_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct slg_data *info;

	if (get_batt_auth_by_name("main_suppiler")) {
		pr_err("slg device detect skip");
		return 0;
	}

	info =
	    devm_kzalloc(&(pdev->dev), sizeof(struct slg_data),
			 GFP_KERNEL);
	if (!info) {
		pr_err("%s alloc mem fail\n", __func__);
		return -ENOMEM;
	}
	info->probe_done = 0;

	if ((!pdev->dev.of_node
	     || !of_device_is_available(pdev->dev.of_node)))
		return -ENODEV;

	info->dev = &(pdev->dev);
	info->pdev = pdev;
	platform_set_drvdata(pdev, info);
	spin_lock_init(&w1_lock);

	ret = of_property_read_string(pdev->dev.of_node,
		"auth_name", &info->auth_name);
	if (ret < 0) {
		pr_err("%s can not find auth name(%d), use default name\n",
		       __func__, ret);
		info->auth_name = "second_supplier";
	}

	g_info = info;
	g_info->probe_done = 0;
	g_info->romid_verified = false;

	gpio_66_cfg = ioremap(GPIO66_CFG_REG_PHYS, 0x8);
	if (!gpio_66_cfg) {
		dev_err(&pdev->dev, "<%s>: map gpio-66 cfg register failed",
			__func__);
		return -EINVAL;
	}
	gpio_66_inout = gpio_66_cfg + 0x4;

	init_waitqueue_head(&slg_wait);

	slg_check_chip_ok();
	if (!g_info->romid_verified) {
		pr_err("%s slg device not found\n", __func__);
		return 0;
	}
	pr_info("%s slg device detected\n", __func__);

	info->auth_dev = auth_device_register(info->auth_name, NULL, info,
					      &slg_auth_ops);
	if (!info->auth_dev || IS_ERR(info->auth_dev)) {
		pr_err("%s failed to register auth device\n", __func__);
		return PTR_ERR(info->auth_dev);
	}

	pr_err("slg probe successfully!\n");

	return 0;
}

static int slg_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id slg_of_ids[] = {
	{.compatible = "acl,slg"},
	{},
};

static struct platform_driver slg_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "acl,slg",
		   .of_match_table = slg_of_ids,
		   },
	.probe = slg_probe,
	.remove = slg_remove,
};

static int __init slg_init(void)
{
	pr_info("%s enter\n", __func__);
	return platform_driver_register(&slg_driver);
}

static void __exit slg_exit(void)
{
	pr_err("%s enter\n", __func__);

	platform_driver_unregister(&slg_driver);
}

module_init(slg_init);
module_exit(slg_exit);

MODULE_LICENSE("GPL");
