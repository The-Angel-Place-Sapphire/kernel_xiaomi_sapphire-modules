/*===========================================================================
*
=============================================================================
EDIT HISTORY

when           who     what, where, why
--------       ---     -----------------------------------------------------------
03/25/2020           Inital Release
=============================================================================*/

/*---------------------------------------------------------------------------
* Include Files
* -------------------------------------------------------------------------*/
/*
 * sapphire (SM6225 "khaje") version: the 1-wire line is driven through the
 * TLMM registers of GPIO66 directly (ioremap of the GPIO66 cfg/in_out
 * registers) with ktime based busy-wait timing, commands are serialized by
 * ds_cmd_lock and retried up to DS_CMD_RETRY times, the chip is powered via
 * GPIO 463 when needed, and the battery id is taken from page 0 at probe.
 * Reconstructed from the stock sapphire ds28e30_verify.ko on top of the
 * Huaqin battery_secrete/ds28e30.c (linastorvaldz gold / picasso09 moon).
 */

#include <linux/slab.h>		/* kfree() */
#include <linux/module.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/string.h>

#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/power_supply.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/gpio/consumer.h>
#include <linux/regmap.h>
#include <linux/random.h>
#include <linux/io.h>
#include <linux/ktime.h>
#include <linux/timekeeping.h>
#include <linux/wait.h>
#include <linux/mutex.h>
#include <linux/printk.h>

#include "ds28e30.h"
#include "sha256_hmac.h"
#include "deep_cover_coproc.h"
#include "battery_auth_class.h"

//common define
#define ds_info(fmt, ...)	pr_info("[ds28e30] " fmt, ##__VA_ARGS__)
#define ds_dbg(fmt, ...)	pr_debug("[ds28e30] " fmt, ##__VA_ARGS__)
#define ds_err(fmt, ...)	pr_err("[ds28e30] " fmt, ##__VA_ARGS__)
#define ds_log			ds_info

/* khaje TLMM GPIO66 (1-wire data) and the battery-ID power GPIO */
#define DS_GPIO66_CFG_PHYS		0x542000
#define DS_GPIO_CFG_OUT			0x3c0
#define DS_GPIO_CFG_IN			0x5c0
#define DS_GPIO_OUT_HIGH		0x2
#define DS_GPIO_OUT_LOW			0x0
#define DS_POWER_GPIO			463

#define DS_CMD_RETRY			20
#define DS_DETECT_RETRY			10
#define DS_RETRY_WAIT_MS		20

// int OWSkipROM(void);
int pagenumber;

//maxim define
unsigned short CRC16;
const short oddparity[16] =
    { 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0 };
unsigned char last_result_byte = RESULT_SUCCESS;

//define system-level publick key, authority public key  and certificate constant variables
unsigned char SystemPublicKeyX[32];
unsigned char SystemPublicKeyY[32];
unsigned char AuthorityPublicKey_X[32];
unsigned char AuthorityPublicKey_Y[32];
unsigned char Page_Certificate_R[32];
unsigned char Page_Certificate_S[32];
unsigned char Certificate_Constant[16];
unsigned char Expected_CID[2];
unsigned char Expected_MAN_ID[2];
unsigned char challenge[32] =
    { 55, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00,
	00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 66
};

// int pagenumber = 0;

unsigned char session_seed[32] = {
	0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
	0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
	0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
	0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA
};

unsigned char S_secret[32] = {
	0x0C, 0x99, 0x2B, 0xD3, 0x95, 0xDB, 0xA0, 0xB4,
	0xEF, 0x07, 0xB3, 0xD8, 0x75, 0xF3, 0xC7, 0xAE,
	0xDA, 0xC4, 0x41, 0x2F, 0x48, 0x93, 0xB5, 0xD9,
	0xE1, 0xE5, 0x4B, 0x20, 0x9B, 0xF3, 0x77, 0x39
};

int auth_ANON = 1;
int auth_BDCONST = 1;

//define constant for generating certificate
unsigned char MI_Certificate_Constant[16] = { 0x25, 0x1C, 0x51, 0x79, 0x91, 0x92, 0x67, 0x58, 0xAF, 0xCA, 0xD5, 0x44, 0x22, 0xA2, 0x98, 0xB9 };	//for MI

unsigned char MI_SystemPublicKeyX[32] = { 0xE4, 0xDD, 0xAB, 0x00, 0x57, 0xA2, 0x11, 0xBC,	//32-byte system-level public key X
	0x4E, 0x7F, 0xE1, 0x70, 0xF9, 0xB0, 0x98, 0xB3,
	0x76, 0xE9, 0x84, 0x7B, 0xEE, 0x7F, 0x69, 0xCF,
	0x98, 0xC8, 0xBB, 0xD7, 0xC1, 0x22, 0xB9, 0xA3
};

unsigned char MI_SystemPublicKeyY[32] = { 0x3A, 0xD6, 0xEE, 0x23, 0x40, 0x12, 0xB9, 0x9E,	//32-byte system-level public key Y
	0x61, 0x1C, 0x73, 0xC6, 0xED, 0x94, 0x47, 0x71,
	0x2A, 0x66, 0x7A, 0x9D, 0x1A, 0xAC, 0x07, 0xC4,
	0xF9, 0x27, 0xCB, 0xF4, 0xFB, 0x22, 0x41, 0x3C
};

unsigned char MI_AuthorityPublicKey_X[32] =
    { 0x7B, 0xD7, 0x7A, 0xC6, 0xCF, 0xEF, 0xD3, 0xC1,
	0xAC, 0x60, 0x44, 0x23, 0x1C, 0xCE, 0xE2, 0x66,
	0x05, 0xD3, 0xDE, 0x85, 0x06, 0xBD, 0x17, 0xF9,
	0xA7, 0x14, 0x0B, 0x0D, 0x32, 0x7F, 0x1F, 0x16
};

unsigned char MI_AuthorityPublicKey_Y[32] =
    { 0x7D, 0xA2, 0x2B, 0xED, 0xBD, 0x13, 0xFB, 0x94,
	0x4B, 0x1F, 0x0A, 0x1B, 0x0D, 0x33, 0xE8, 0x91,
	0xAF, 0x37, 0x75, 0xD8, 0x4A, 0x5C, 0x54, 0x90,
	0x5A, 0x64, 0xC7, 0x36, 0x37, 0x33, 0xC5, 0xFC
};

// keys in byte array format, used by software compute functions
// char private_key[32];
char public_key_x[32];
char public_key_y[32];

//define testing number
#define Testing_Item_Number  19

//useful define
unsigned char MANID[2] = { MI_MAN_ID_LSB, MI_MAN_ID_MSB };
unsigned char HardwareVersion[2] = { 0x00 };

unsigned char TestingItemResult[Testing_Item_Number];	//maximal testing items

//define testing item result
#define Family_Code_Result    0
#define Custom_ID_Result 1	//custom ID is special for each mobile maker
#define Unique_ID_Result  2
#define MAN_ID_Result  3
#define Status_Result     4
#define Page0_Result  5
#define Page1_Result  6
#define Page2_Result  7
#define Page3_Result  8
#define CounterValue_Result 9
#define Verification_Signature_Result  10
#define Verification_Certificate_Result  11
#define Program_Page0_Result  12
#define Program_Page1_Result  13
#define Program_Page2_Result  14
#define Program_Page3_Result  15
#define DecreasingCounterValue_Result 16
#define Device_publickey_Result  17
#define Device_certificate_Result  18

//mi add
// unsigned char flag_mi_romid = 0;
unsigned char flag_mi_status = 0;
unsigned char flag_mi_page0_data = 0;
unsigned char flag_mi_page1_data = 0;
unsigned char flag_mi_counter = 0;
unsigned char flag_mi_auth_result = 0;
unsigned char mi_romid[8] = { 0x00 };
unsigned char mi_status[12] = { 0x00 };	//0,1,2,3,4,5,6,7,28,29,36
unsigned char mi_page0_data[32] = { 0x00 };
unsigned char mi_page1_data[32] = { 0x00 };
unsigned char mi_counter[16] = { 0x00 };

int mi_auth_result = 0x00;
unsigned int attr_trytimes = 1;

/* page 0 (battery vendor) data, read once at probe */
unsigned char Page0Data[32];
static uint8_t batt_id = 0xff;

DEFINE_MUTEX(ds_cmd_lock);	/* stock: zeroed, never initialised */
static wait_queue_head_t ds28e30_wait;

static void __iomem *gpio_66_cfg;
static void __iomem *gpio_66_inout;

struct ds_data {
	struct platform_device *pdev;
	struct device *dev;
	const char *auth_name;
	struct auth_device *auth_dev;

	bool batt_verified;
	struct delayed_work authentic_work;
	int version;
	bool romid_verified;
	int page0_val;
	int probe_done;
	raw_spinlock_t io_lock;
};

static struct ds_data *g_info;

#define DS_REG_WRITE(val, reg)		do { *(volatile unsigned int __force *)(reg) = ((val)); wmb(); } while (0)

#define ONE_WIRE_CONFIG_OUT		(*(volatile unsigned int __force *)gpio_66_cfg = DS_GPIO_CFG_OUT)
#define ONE_WIRE_CONFIG_IN		(*(volatile unsigned int __force *)gpio_66_cfg = DS_GPIO_CFG_IN)
#define ONE_WIRE_OUT_HIGH		do { DS_REG_WRITE(DS_GPIO_OUT_HIGH, gpio_66_inout); \
					     DS_REG_WRITE(DS_GPIO_OUT_HIGH, gpio_66_inout); } while (0)
#define ONE_WIRE_OUT_LOW		do { DS_REG_WRITE(DS_GPIO_OUT_LOW, gpio_66_inout); \
					     DS_REG_WRITE(DS_GPIO_OUT_LOW, gpio_66_inout); } while (0)
#define ONE_WIRE_GPIO_READ		(*(volatile unsigned int __force *)gpio_66_inout & 0x1)

#define DS_WAIT_MS(ms)	wait_event_timeout(ds28e30_wait, 0, msecs_to_jiffies(ms))

/* write/read ops */
static void Delay_ns(unsigned int T)
{
	u64 start = ktime_get_boottime_ns();

	while (ktime_get_boottime_ns() - start < T)
		;
}

static void Delay_us(unsigned int T)
{
	Delay_ns(T * 1000);
}

static unsigned char ow_reset(void)
{
	unsigned char presence = 0xFF;
	unsigned long flags;
	u64 start, now;

	raw_spin_lock_irqsave(&g_info->io_lock, flags);

	ONE_WIRE_CONFIG_OUT;
	ONE_WIRE_OUT_LOW;
	Delay_us(48);		// 48
	ONE_WIRE_OUT_HIGH;
	ONE_WIRE_CONFIG_IN;

	start = ktime_get_boottime_ns();
	do {
		now = ktime_get_boottime_ns();
		presence = ONE_WIRE_GPIO_READ;
	} while (presence && (now - start) <= 10000);

	Delay_us(50);

	raw_spin_unlock_irqrestore(&g_info->io_lock, flags);

	return presence;
}

static unsigned char read_bit(void)
{
	unsigned int vamm;

	ONE_WIRE_CONFIG_OUT;
	ONE_WIRE_OUT_LOW;
	ONE_WIRE_CONFIG_IN;
	vamm = ONE_WIRE_GPIO_READ;
	Delay_us(5);
	ONE_WIRE_OUT_HIGH;
	ONE_WIRE_CONFIG_OUT;
	Delay_us(6);

	return vamm;
}

void write_bit(char bitval)
{
	ONE_WIRE_CONFIG_OUT;
	ONE_WIRE_OUT_LOW;
	Delay_ns(500);
	if (bitval != 0)
		ONE_WIRE_OUT_HIGH;
	Delay_us(10);
	ONE_WIRE_OUT_HIGH;
	Delay_us(6);
}

static unsigned char read_byte(void)
{
	unsigned char i;
	unsigned char value = 0;
	unsigned long flags;

	raw_spin_lock_irqsave(&g_info->io_lock, flags);
	for (i = 0; i < 8; i++) {
		if (read_bit())
			value |= 0x01 << i;	// reads byte in, one byte at a time and then shifts it left
	}
	raw_spin_unlock_irqrestore(&g_info->io_lock, flags);

	return value;
}

static void write_byte(char val)
{
	unsigned char i;
	unsigned char temp;
	unsigned long flags;

	raw_spin_lock_irqsave(&g_info->io_lock, flags);
	// writes byte, one bit at a time
	for (i = 0; i < 8; i++) {
		temp = val >> i;	// shifts val right 'i' spaces
		temp &= 0x01;	// copy that bit to temp
		write_bit(temp);	// write bit in temp into
	}
	raw_spin_unlock_irqrestore(&g_info->io_lock, flags);
}

unsigned char crc_low_first(unsigned char *ptr, unsigned char len)
{
	unsigned char i;
	unsigned char crc = 0x00;

	while (len--) {
		crc ^= *ptr++;
		for (i = 0; i < 8; ++i) {
			if (crc & 0x01)
				crc = (crc >> 1) ^ 0x8c;
			else
				crc = (crc >> 1);
		}
	}

	return (crc);
}

short Read_RomID(unsigned char *RomID)
{
	unsigned char i;
	unsigned char crc = 0x00;

	if ((ow_reset()) != 0) {
		ds_err("Failed to reset ds28e30!\n");
		ow_reset();
		return ERROR_NO_DEVICE;
	}

	write_byte(CMD_READ_ROM);
	Delay_us(10);
	for (i = 0; i < 8; i++)
		RomID[i] = read_byte();

	print_hex_dump(KERN_ERR, "[ds28e30] RomID: ", DUMP_PREFIX_NONE,
		       16, 1, RomID, 8, false);

	crc = crc_low_first(RomID, 7);

	if (crc == RomID[7]) {
		if (RomID[0] != FAMILY_CODE) {
			ds_err("wrong family code\n");
			return DS_FALSE;
		}
		memcpy(mi_romid, RomID, 8);
		return DS_TRUE;
	} else {
		ow_reset();
		ds_err("crc error in Read_RomID\n");
		return DS_FALSE;
	}
}

unsigned short docrc16(unsigned short data)
{
	data = (data ^ (CRC16 & 0xff)) & 0xff;
	CRC16 >>= 8;

	if (oddparity[data & 0xf] ^ oddparity[data >> 4])
		CRC16 ^= 0xc001;

	data <<= 6;
	CRC16 ^= data;
	data <<= 1;
	CRC16 ^= data;

	return CRC16;
}

//---------------------------------------------------------------------------
/// @internal
///
/// Sent/receive standard flow command 
///
/// @param[in] write_buf
/// Buffer with write contents (preable payload)
/// @param[in] write_len
/// Total length of data to write in 'write_buf'
/// @param[in] delay_ms
/// Delay in milliseconds after command/preable.  If == 0 then can use 
/// repeated-start to re-access the device for read of result byte. 
/// @param[in] expect_read_len
/// Expected result read length 
/// @param[out] read_buf
/// Buffer to hold data read from device. It must be at least 255 bytes long. 
/// @param[out] read_len
/// Pointer to an integer to contain the length of data read and placed in read_buf
/// Preloaded with expected read length for 1-Wire mode. If (0) but expected_read=TRUE
/// then the first byte read is the length of data to read. 
///
///  @return
///  TRUE - command successful @n
///  FALSE - command failed
///
/// @endinternal
///
static __always_inline void ds28e30_power_up(void)
{
	if (!gpiod_get_raw_value(gpio_to_desc(DS_POWER_GPIO))) {
		gpiod_direction_output_raw(gpio_to_desc(DS_POWER_GPIO), 1);
		ds_info("power up ds28e30\n");
		DS_WAIT_MS(DS_RETRY_WAIT_MS);
	}
}

int standard_cmd_flow(unsigned char *write_buf, int write_len,
		      int delay_ms, int expect_read_len,
		      unsigned char *read_buf, int *read_len)
{
	unsigned char pkt[256] = { 0 };
	int pkt_len = 0;
	int i;
	int reset_retry = 0, crc1_retry = 0, crc2_retry = 0;

	mutex_lock(&ds_cmd_lock);

	do {
		ds28e30_power_up();

		// Reset/presence
		if ((ow_reset()) != 0) {
			reset_retry++;
			ds_err("reset device failed, retry cnt: %d\n", reset_retry);
			DS_WAIT_MS(DS_RETRY_WAIT_MS);
			continue;
		}

		// Rom Command: Skip ROM
		write_byte(CMD_SKIP_ROM);

		// set result byte to no response
		last_result_byte = RESULT_FAIL_NONE;

		// construct write block, start with XPC command
		pkt_len = 0;
		pkt[pkt_len++] = CMD_START;

		// add length
		pkt[pkt_len++] = write_len;

		// write (first byte will be sub-command)
		memcpy(&pkt[pkt_len], write_buf, write_len);
		pkt_len += write_len;

		ds_info("w1w: %02x\n", CMD_SKIP_ROM);
		print_hex_dump(KERN_ERR, "[ds28e30] w1w: ", DUMP_PREFIX_NONE,
			       16, 1, pkt, pkt_len, false);

		// send packet to DS28E30
		for (i = 0; i < pkt_len; i++)
			write_byte(pkt[i]);

		// read two CRC bytes
		pkt[pkt_len++] = read_byte();
		pkt[pkt_len++] = read_byte();
		ds_info("w1r: %02x %02x\n", pkt[pkt_len - 2], pkt[pkt_len - 1]);

		// check CRC16
		CRC16 = 0;
		for (i = 0; i < pkt_len; i++)
			docrc16(pkt[i]);

		if (CRC16 != 0xB001) {
			crc1_retry++;
			ds_err("first crc error, retry cnt: %d\n", crc1_retry);
			DS_WAIT_MS(DS_RETRY_WAIT_MS);
			continue;
		}

		// check for strong pull-up
		if (delay_ms > 0) {
			// send release byte, start strong pull-up
			write_byte(0xAA);
			ds_info("w1w: %02x\n", 0xAA);
			// optional delay
			DS_WAIT_MS(delay_ms);
		}

		// read FF and the length byte
		pkt[0] = read_byte();
		pkt[1] = read_byte();
		*read_len = pkt[1];
		ds_info("w1r: %02x %02x\n", pkt[0], pkt[1]);

		// read the rest of the response and the CRC
		for (i = 0; i < expect_read_len + 2; i++)
			read_buf[i] = read_byte();
		print_hex_dump(KERN_ERR, "[ds28e30] w1r: ", DUMP_PREFIX_NONE,
			       16, 1, read_buf, expect_read_len + 2, false);

		// check CRC16
		CRC16 = 0;
		docrc16(*read_len);
		for (i = 0; i < (expect_read_len + 2); i++)
			docrc16(read_buf[i]);

		if (*read_len != expect_read_len) {
			*read_len = expect_read_len;
			ds_err("receive length error\n");
		}

		if (CRC16 == 0xB001)
			break;

		crc2_retry++;
		ds_err("second crc error, retry cnt: %d\n", crc2_retry);
		DS_WAIT_MS(DS_RETRY_WAIT_MS);
	} while (reset_retry < DS_CMD_RETRY && crc1_retry < DS_CMD_RETRY &&
		 crc2_retry < DS_CMD_RETRY);

	mutex_unlock(&ds_cmd_lock);

	return (reset_retry < DS_CMD_RETRY && crc1_retry < DS_CMD_RETRY &&
		crc2_retry < DS_CMD_RETRY);
}

int ds28e30_read_ROMNO_MANID_HardwareVersion()
{
	unsigned char i, temp = 0, buf[10], pg = 0, flag;

	/* N17 code for HQ-292265 by tongjiacheng at 20230522 start */
	if (mi_romid[0] != FAMILY_CODE) {
		mi_romid[0] = 0x00;
		Read_RomID(mi_romid);	//search DS28E30
	}
	/* N17 code for HQ-292265 by tongjiacheng at 20230522 end */
	else {
		for (i = 0; i < 6; i++)
			temp |= mi_romid[1 + i];	//check if the device is power up at the first time

		if (temp == 0)	//power up the device, then read ROMID again
		{
			ds28e30_cmd_readStatus(pg, buf, MANID, HardwareVersion);	//page number=0
			mi_romid[0] = 0x00;
			Read_RomID(mi_romid);	//read ROMID from DS28E30
			flag = ds28e30_cmd_readStatus(0x80 | pg, buf, MANID, HardwareVersion);	//page number=0
			return flag;
		} else {
			flag = ds28e30_cmd_readStatus(0x80 | pg, buf, MANID, HardwareVersion);	//page number=0
			return flag;
		}
	}
	return DS_FALSE;
}

//---------------------------------------------------------------------------
//setting expected MAN_ID, protection status, counter value, system-level public key, authority public key and certificate constants
void ConfigureDS28E30Parameters(void)
{
	//MI device
	Expected_CID[0] = MI_CID_LSB;
	Expected_CID[1] = MI_CID_MSB;
	Expected_MAN_ID[0] = MI_MAN_ID_LSB;
	Expected_MAN_ID[1] = MI_MAN_ID_MSB;
	//memcpy( Expected_PageProtectionStatus, MI_PageProtectionStatus ,11);
	/* N17 code for HQ-293392 by tongjiacheng at 20230504 start */
	memcpy(Certificate_Constant, MI_Certificate_Constant, sizeof(MI_Certificate_Constant));
	memcpy(SystemPublicKeyX, MI_SystemPublicKeyX, sizeof(MI_SystemPublicKeyX));
	memcpy(SystemPublicKeyY, MI_SystemPublicKeyY, sizeof(MI_SystemPublicKeyY));
	memcpy(AuthorityPublicKey_X, MI_AuthorityPublicKey_X, sizeof(MI_AuthorityPublicKey_X));
	memcpy(AuthorityPublicKey_Y, MI_AuthorityPublicKey_Y, sizeof(MI_AuthorityPublicKey_Y));
	/* N17 code for HQ-293392 by tongjiacheng at 20230504 en */
}

//---------------------------------------------------------------------------
/// Verify certificate of devices like DS28C36/DS28C36/DS28E38/DS28E30.
///
/// @param[in] sig_r
/// Buffer for R portion of certificate signature (MSByte first)
/// @param[in] sig_s
/// Buffer for S portion of certificate signature (MSByte first)
/// @param[in] pub_x
/// Public Key x to verify
/// @param[in] pub_y
/// Public Key y to verify
/// @param[in] SLAVE_ROMID
/// device's 64-bit ROMID (LSByte first)
/// @param[in] SLAVE_MANID
/// Maxim defined as manufacturing ID
/// @param[in] system_level_pub_key_x
/// 32-byte buffer container the system level public key x
/// @param[in] system_level_pub_key_y
/// 32-byte buffer container the system level public key y
///
///  @return
///  DS_TRUE - certificate valid @n
///  DS_FALSE - certificate not valid
///
int verifyECDSACertificateOfDevice(unsigned char *sig_r,
				   unsigned char *sig_s,
				   unsigned char *pub_key_x,
				   unsigned char *pub_key_y,
				   unsigned char *SLAVE_ROMID,
				   unsigned char *SLAVE_MANID,
				   unsigned char *system_level_pub_key_x,
				   unsigned char *system_level_pub_key_y)
{
	unsigned char buf[32];

	// setup software ECDSA computation
	deep_cover_coproc_setup(0, 0, 0, 0);

	// create customization field
	// 16 zeros (can be set to other customer specific value)
	memcpy(buf, Certificate_Constant, 16);
	// ROMID
	memcpy(&buf[16], SLAVE_ROMID, 8);
	// MANID
	memcpy(&buf[24], SLAVE_MANID, 2);
	return deep_cover_verifyECDSACertificate(sig_r, sig_s, pub_key_x,
						 pub_key_y, buf, 26,
						 system_level_pub_key_x,
						 system_level_pub_key_y);
}

//--------------------------------------------------------------------------
/// 'Compute and Read Page Authentication' command
///
/// @param[in] pg - page number to compute auth on
/// @param[in] anon - anonymous flag (1) for anymous
/// @param[in] challenge
/// buffer length must be at least 32 bytes containing the challenge
/// @param[out] data
/// buffer length must be at least 64 bytes to hold ECDSA signature
///
/// @return
/// DS_TRUE - command successful @n
/// DS_FALSE - command failed
///
int ds28e30_cmd_computeReadPageAuthentication(int pg, int anon,
					      unsigned char *challenge,
					      unsigned char *sig)
{
	unsigned char write_buf[200];
	int write_len;
	unsigned char read_buf[255];
	int read_len;
	/*
	   Reset
	   Presence Pulse
	   <ROM Select>
	   TX: XPC Command (66h)
	   TX: Length byte 34d 
	   TX: XPC sub-command A5h (Compute and Read Page Authentication)
	   TX: Parameter (page)
	   TX: Challenge (32d bytes)
	   RX: CRC16 (inverted of XPC command, length, sub-command, parameter, and challenge)
	   TX: Release Byte
	   <Delay TBD>
	   RX: Dummy Byte
	   RX: Length byte (65d)
	   RX: Result Byte
	   RX: Read ECDSA Signature (64 bytes, ‘s’ and then ‘r’, MSByte first, [same as ES10]), 
	   signature 00h's if result byte is not AA success
	   RX: CRC16 (inverted, length byte, result byte, and signature)
	   Reset or send XPC command (66h) for a new sequence
	 */

	// construct the write buffer
	write_len = 0;
	write_buf[write_len++] = CMD_COMP_READ_AUTH;
	write_buf[write_len] = pg & 0x7f;
	if (anon)
		write_buf[write_len] |= 0xE0;
	write_len++;
	write_buf[write_len++] = 0x03;	//authentication parameter
	memcpy(&write_buf[write_len], challenge, 32);
	write_len += 32;

	// preload read_len with expected length
	read_len = 65;

	// default failure mode 
	last_result_byte = RESULT_FAIL_NONE;

	// if(ds28e30_standard_cmd_flow(write_buf, DELAY_DS28E30_ECDSA_GEN_TGES, read_buf, &read_len, write_len))
	if (standard_cmd_flow
	    (write_buf, write_len, DELAY_DS28E30_ECDSA_GEN_TGES, read_len,
	     read_buf, &read_len)) {
		// get result byte
		last_result_byte = read_buf[0];
		// check result
		if (read_len == 65) {
			if (read_buf[0] == RESULT_SUCCESS) {
				memcpy(sig, &read_buf[1], 64);
				return DS_TRUE;
			}
		}
	}

	ow_reset();
	// no payload in read buffer or failed command
	return DS_FALSE;
}

//---------------------------------------------------------------------------
/// High level function to do a full challenge/response ECDSA operation
/// on specified page
///
/// @param[in] pg
/// page to do operation on
/// @param[in] anon
/// flag to indicate in anonymous mode (1) or not anonymous (0)
/// @param[in] mempage
/// buffer with memory page contents, required for verification of ECDSA signature
/// @param[in] challenge
/// buffer containing challenge, must be 32 bytes
/// @param[out] sig_r
/// buffer for r portion of signature, must be 32 bytes
/// @param[out] sig_s
/// buffer for s portion of signature, must be 32 bytes
///
/// @return
/// DS_TRUE - command successful @n
/// DS_FALSE - command failed
///
int ds28e30_computeAndVerifyECDSA_NoRead(int pg, int anon,
					 unsigned char *mempage,
					 unsigned char *challenge,
					 unsigned char *sig_r,
					 unsigned char *sig_s)
{
	unsigned char signature[64], message[256];
	int msg_len;
	unsigned char *pubkey_x, *pubkey_y;

	// compute and read auth command
	if (!ds28e30_cmd_computeReadPageAuthentication
	    (pg, anon, challenge, signature)) {
		return DS_FALSE;
	}
	// put the signature in the return buffers, signature is 's' and then 'r', MSByte first
	memcpy(sig_s, signature, 32);
	memcpy(sig_r, &signature[32], 32);

	// construct the message to hash for signature verification
	// ROM NO | Page Data | Challenge (Buffer) | Page# | MANID

	// ROM NO
	msg_len = 0;
	if (anon)
		memset(&message[msg_len], 0xFF, 8);
	else
		memcpy(&message[msg_len], mi_romid, 8);
	msg_len += 8;
	// Page Data
	memcpy(&message[msg_len], mempage, 32);
	msg_len += 32;
	// Challenge (Buffer)
	memcpy(&message[msg_len], challenge, 32);
	msg_len += 32;
	// Page#
	message[msg_len++] = pg;
	// MANID
	memcpy(&message[msg_len], MANID, 2);
	msg_len += 2;

	pubkey_x = public_key_x;
	pubkey_y = public_key_y;

	// verify Signature and return result
	return deep_cover_verifyECDSASignature(message, msg_len, pubkey_x,
					       pubkey_y, sig_r, sig_s);
}

//---------------------------------------------------------------------------
//-------- ds28e30 High level functions 
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
/// High level function to do a full challenge/response ECDSA operation 
/// on specified page 
///
/// @param[in] pg
/// page to do operation on
/// @param[in] anon
/// flag to indicate in anonymous mode (1) or not anonymous (0)
/// @param[out] mempage
/// buffer to return the memory page contents
/// @param[in] challenge
/// buffer containing challenge, must be 32 bytes
/// @param[out] sig_r
/// buffer for r portion of signature, must be 32 bytes 
/// @param[out] sig_s
/// buffer for s portion of signature, must be 32 bytes 
///
/// @return
/// DS_TRUE - command successful @n
/// DS_FALSE - command failed
///
int ds28e30_computeAndVerifyECDSA(int pg, int anon, unsigned char *mempage,
				  unsigned char *challenge,
				  unsigned char *sig_r,
				  unsigned char *sig_s)
{
	// read destination page
	// if (!ds28e30_cmd_readMemory(pg, mempage))
	if (TestingItemResult[Page0_Result] == DS_FALSE)
		return DS_FALSE;

	return ds28e30_computeAndVerifyECDSA_NoRead(pg, anon, mempage,
						    challenge, sig_r,
						    sig_s);
}

int AuthenticateDS28E30(void)
{
	unsigned char buf[128];
	unsigned char DevicePublicKey_X[32], DevicePublicKey_Y[32];
	unsigned char sig_r[32], sig_s[32];

	ds_info("-------- authenticate ds28e30 start --------\n");

	/* ROMID/MAN_ID, status, page0 and counter were checked at probe */
	TestingItemResult[CounterValue_Result] = DS_TRUE;
	TestingItemResult[Family_Code_Result] = DS_TRUE;
	ConfigureDS28E30Parameters();
	TestingItemResult[Custom_ID_Result] = DS_TRUE;
	TestingItemResult[Unique_ID_Result] = DS_TRUE;
	TestingItemResult[MAN_ID_Result] = DS_TRUE;
	TestingItemResult[Status_Result] = DS_TRUE;
	TestingItemResult[Page0_Result] = DS_TRUE;

	/**3、verifying Device's digital signatre**/
	ds_info("******** get device public kery ********\n");
	TestingItemResult[Device_publickey_Result] = DS_FALSE;
	TestingItemResult[Device_publickey_Result] =
	    ds28e30_cmd_readDevicePublicKey(buf);
	if (TestingItemResult[Device_publickey_Result] != DS_TRUE) {
		ds_err("read public key failed\n");
		goto out;
	}
	memcpy(DevicePublicKey_X, buf, 32);	//reserve device public key X
	memcpy(DevicePublicKey_Y, &buf[32], 32);	//reserve device public key Y
	memcpy(public_key_x, DevicePublicKey_X, 32);
	memcpy(public_key_y, DevicePublicKey_Y, 32);
	print_hex_dump(KERN_ERR, "[ds28e30] Page28Data: ", DUMP_PREFIX_NONE,
		       16, 1, buf, 32, false);
	print_hex_dump(KERN_ERR, "[ds28e30] Page29Data: ", DUMP_PREFIX_NONE,
		       16, 1, &buf[32], 32, false);

	ds_info("******** verify ECDSA ********\n");
	TestingItemResult[Verification_Signature_Result] = DS_FALSE;
	deep_cover_coproc_setup(0, 0, 0, 0);
	TestingItemResult[Verification_Signature_Result] =
	    ds28e30_computeAndVerifyECDSA(0, 0, Page0Data, challenge, sig_r, sig_s);
	if (TestingItemResult[Verification_Signature_Result] != DS_TRUE) {
		ds_err("computeAndVerifyECDSA failed\n");
		goto out;
	}

	/**4、verifying Device's certificate**/
	ds_info("******** get certificate ********\n");
	TestingItemResult[Device_certificate_Result] = DS_FALSE;
	if (ds28e30_cmd_readMemory(PG_CERTIFICATE_R, Page_Certificate_R) != DS_TRUE) {
		ds_err("read page4 failed\n");
		goto out;
	}
	if (ds28e30_cmd_readMemory(PG_CERTIFICATE_S, Page_Certificate_S) != DS_TRUE) {
		ds_err("read page5 failed\n");
		goto out;
	}
	TestingItemResult[Device_certificate_Result] = DS_TRUE;
	print_hex_dump(KERN_ERR, "[ds28e30] page4Data: ", DUMP_PREFIX_NONE,
		       16, 1, Page_Certificate_R, 32, false);
	print_hex_dump(KERN_ERR, "[ds28e30] page5Data: ", DUMP_PREFIX_NONE,
		       16, 1, Page_Certificate_S, 32, false);

	ds_info("******** verify certificate ********\n");
	TestingItemResult[Verification_Certificate_Result] = DS_FALSE;
	TestingItemResult[Verification_Certificate_Result] =
	    verifyECDSACertificateOfDevice(Page_Certificate_R, Page_Certificate_S,
					   DevicePublicKey_X, DevicePublicKey_Y,
					   mi_romid, MANID, SystemPublicKeyX,
					   SystemPublicKeyY);

out:
	if ((TestingItemResult[Verification_Signature_Result] == DS_TRUE) &&
	    (TestingItemResult[Verification_Certificate_Result] == DS_TRUE))
		mi_auth_result = DS_TRUE;
	else
		mi_auth_result = DS_FALSE;

	ds_info("mi_auth_result = %d, battery id: %x\n", mi_auth_result, batt_id);
	ds_info("-------- authenticate ds28e30 end --------\n");

	return mi_auth_result;
}

/*
* 'Read Status' command
*
*  @param[in] pg
*  page to read protection
*  @param[out] pr_data
*  pointer to unsigned char buffer of length 6 for page protection data
*  @param[out] manid
*  pointer to unsigned char buffer of length 2 for manid (manufactorur ID)
*
*  @return
*  DS_TRUE - command successful @n
*  DS_FALSE - command failed
*/
int ds28e30_cmd_readStatus(int pg, unsigned char *pr_data, unsigned char *manid, unsigned char *hardware_version)	//-----------------------
{
	unsigned char write_buf[10];
	unsigned char read_buf[128];
	int read_len = 2, write_len;
	int PageNumbers[] = { 0, 1, 2, 3, 4, 5, 6, 7, 28, 29, 36, 106 };
	int i;

	/* 
	   Reset
	   Presence Pulse
	   <ROM Select>
	   TX: XPC Command (66h)
	   TX: Length byte 1d 
	   TX: XPC sub-command AAh (Read Status)
	   RX: CRC16 (inverted of XPC command, length, and sub-command)
	   TX: Release Byte
	   <Delay TBD>
	   RX: Dummy Byte
	   RX: Length Byte (11d)
	   RX: Result Byte
	   RX: Read protection values (6 Bytes), MANID (2 Bytes), ROM VERSION (2 bytes)
	   RX: CRC16 (inverted, length byte, protection values, MANID, ROM_VERSION)
	   Reset or send XPC command (66h) for a new sequence
	 */

	// construct the write buffer
	write_len = 0;
	write_buf[write_len++] = CMD_READ_STATUS;
	write_buf[write_len++] = pg;

	// preload read_len with expected length
	if (pg & 0x80)
		read_len = 5;

	// default failure mode 
	last_result_byte = RESULT_FAIL_NONE;


	// if(ds28e30_standard_cmd_flow(write_buf, DELAY_DS28E30_EE_READ_TRM, read_buf, &read_len, write_len))
	if (standard_cmd_flow
	    (write_buf, write_len, DELAY_DS28E30_EE_READ_TRM, read_len,
	     read_buf, &read_len)) {
		// get result byte
		last_result_byte = read_buf[0];
		// should always be 2 or 5 length for status data
		if (read_len == 2 || read_len == 5) {
			if (read_buf[0] == RESULT_SUCCESS
			    || read_buf[0] == RESULT_FAIL_DEVICEDISABLED) {
				if (read_len == 2) {
					memcpy(pr_data, &read_buf[1], 8);
					*pr_data = read_buf[1];
					pr_data[0] = read_buf[1];
					for (i = 0;
					     i < ARRAY_SIZE(PageNumbers);
					     i++) {
						if (pg == PageNumbers[i]) {
							if (i <
							    ARRAY_SIZE
							    (mi_status)) {
								flag_mi_status
								    = 1;
								mi_status
								    [i] =
								    read_buf
								    [1];
							}
						}
					}
				} else {
					memcpy(manid, &read_buf[1], 2);
					memcpy(hardware_version,
					       &read_buf[3], 2);
					memcpy(MANID, &read_buf[1], 2);
					memcpy(HardwareVersion,
					       &read_buf[3], 2);
				/* N17 code for HQ-295595 by tongjiacheng at 20230509 start*/
					ds_dbg("page%d manid:\n",
						pg & 0x7f);
					ds_dbg("%02x %02x\n",
						manid[0], manid[1]);
					ds_dbg
					    ("page%d hardware_version:\n",
					     pg & 0x7f);
					ds_dbg("%02x %02x\n",
						hardware_version[0],
						hardware_version[1]);
				/* N17 code for HQ-295595 by tongjiacheng at 20230509 end*/
				}
				// ds_info("[llt--------------true----------------] ds28e30_cmd_readStatus--\n");
				return DS_TRUE;
			}
		}
	}

	ow_reset();
	// no payload in read buffer or failed command
	return DS_FALSE;
}

////////////////////////////////////////////////////////////////////////////////
/*!
* @brief maxim_authentic_check()
* @detailed
* This function check maxim authentic result
* @return int DS_FALSE/ DS_TRUE/ ERROR_UNMATCH_MAC
*/
////////////////////////////////////////////////////////////////////////////////
int maxim_authentic_check(void)
{
	if (flag_mi_auth_result)
		return mi_auth_result;

	return DS_FALSE;
}

////////////////////////////////////////////////////////////////////////////////
/*!
* @brief maxim_authentic_start(void)
* @detailed
* This function start maxim authentic
* @return bool true if success
*/
////////////////////////////////////////////////////////////////////////////////

//---------------------------------------------------------------------------
//-------- ds28e30 Memory functions 
//---------------------------------------------------------------------------

//--------------------------------------------------------------------------
/// 'Write Memory' command
///
/// @param[in] pg
/// page number to write
/// @param[in] data
/// buffer must be at least 32 bytes
///
///  @return
///  DS_TRUE - command successful @n
///  DS_FALSE - command failed
///
int ds28e30_cmd_writeMemory(int pg, unsigned char *data)
{
	unsigned char write_buf[50];
	int write_len;
	unsigned char read_buf[255];
	int read_len;

	/*
	   Reset
	   Presence Pulse
	   <ROM Select>
	   TX: XPC Command (66h)
	   TX: Length byte 34d
	   TX: XPC sub-command 96h (Write Memory)
	   TX: Parameter
	   TX: New page data (32d bytes)
	   RX: CRC16 (inverted of XPC command, length, sub-command, parameter)
	   TX: Release Byte
	   <Delay TBD>
	   RX: Dummy Byte
	   RX: Length Byte (1d)
	   RX: Result Byte
	   RX: CRC16 (inverted of length and result byte)
	   Reset or send XPC command (66h) for a new sequence
	 */

	// construct the write buffer
	write_len = 0;
	write_buf[write_len++] = CMD_WRITE_MEM;
	write_buf[write_len++] = pg;
	memcpy(&write_buf[write_len], data, 32);
	write_len += 32;

	// preload read_len with expected length
	read_len = 1;

	// default failure mode
	last_result_byte = RESULT_FAIL_NONE;

	// if(ds28e30_standard_cmd_flow(write_buf, DELAY_DS28E30_EE_WRITE_TWM, read_buf, &read_len, write_len))
	if (standard_cmd_flow
	    (write_buf, write_len, DELAY_DS28E30_EE_WRITE_TWM, read_len,
	     read_buf, &read_len)) {
		// get result byte
		last_result_byte = read_buf[0];
		// check result
		if (read_len == 1)
			return (read_buf[0] == RESULT_SUCCESS);
	}
	//ow_reset();
	// no payload in read buffer or failed command
	return DS_FALSE;
}

//--------------------------------------------------------------------------
/// 'Read Memory' command
///
/// @param[in] pg
/// page number to read
/// @param[out] data
/// buffer length must be at least 32 bytes to hold memory read
///
///  @return
///  DS_TRUE - command successful @n
///  DS_FALSE - command failed
///
int ds28e30_cmd_readMemory(int pg, unsigned char *data)
{
	unsigned char write_buf[10];
	int write_len;
	unsigned char read_buf[255];
	int read_len;

	/*
	   Reset
	   Presence Pulse
	   <ROM Select>
	   TX: XPC Command (66h)
	   TX: Length byte 2d
	   TX: XPC sub-command 69h (Read Memory)
	   TX: Parameter (page)
	   RX: CRC16 (inverted of XPC command, length, sub-command, and parameter)
	   TX: Release Byte
	   <Delay TBD>
	   RX: Dummy Byte
	   RX: Length (33d)
	   RX: Result Byte
	   RX: Read page data (32d bytes)
	   RX: CRC16 (inverted, length byte, result byte, and page data)
	   Reset or send XPC command (66h) for a new sequence
	 */

	// construct the write buffer
	write_len = 0;
	write_buf[write_len++] = CMD_READ_MEM;
	write_buf[write_len++] = pg;

	// preload read_len with expected length
	read_len = 33;

	// default failure mode
	last_result_byte = RESULT_FAIL_NONE;

	// if(ds28e30_standard_cmd_flow(write_buf, DELAY_DS28E30_EE_READ_TRM, read_buf, &read_len, write_len))
	if (standard_cmd_flow
	    (write_buf, write_len, DELAY_DS28E30_EE_READ_TRM, read_len,
	     read_buf, &read_len)) {
		// get result byte
		last_result_byte = read_buf[0];
		// check result
		if (read_len == 33) {
			if (read_buf[0] == RESULT_SUCCESS) {
				memcpy(data, &read_buf[1], 32);

				if (pg == 0) {
					flag_mi_page0_data = 1;
					memcpy(mi_page0_data, data, 16);
				}
				if (pg == 1) {
					flag_mi_page1_data = 1;
					memcpy(mi_page1_data, data, 16);
				}
				if (pg == 106) {
					flag_mi_counter = 1;
					memcpy(mi_counter, data, 16);
				}
				// ds_info("[llt---------------true---------------] DS28E30_cmd_readMemory--\n");
				return DS_TRUE;
			}
		}
	}
	//ow_reset();
	// ds_info("[llt--------------false----------------] DS28E30_cmd_readMemory--\n");
	// no payload in read buffer or failed command
	return DS_FALSE;
}

//--------------------------------------------------------------------------
/// 'Set Page Protection' command
///
/// @param[in] pg
/// page to set protection
/// @param[in] prot
/// protection value to set
///
///  @return
///  DS_TRUE - command successful @n
///  DS_FALSE - command failed
///
int ds28e30_cmd_setPageProtection(int pg, unsigned char prot)
{
	unsigned char write_buf[10];
	int write_len;
	unsigned char read_buf[255];
	int read_len;

	/*
	   Reset
	   Presence Pulse
	   <ROM Select>
	   TX: XPC Command (66h)
	   TX: Length byte 3d 
	   TX: XPC sub-command C3h (Set Protection)
	   TX: Parameter (page)
	   TX: Parameter (protection)
	   RX: CRC16 (inverted of XPC command, length, sub-command, parameters)
	   TX: Release Byte
	   <Delay TBD>
	   RX: Dummy Byte
	   RX: Length Byte (1d)
	   RX: Result Byte
	   RX: CRC16 (inverted, length byte and result byte)
	   Reset or send XPC command (66h) for a new sequence
	 */

	// construct the write buffer
	write_len = 0;
	write_buf[write_len++] = CMD_SET_PAGE_PROT;
	write_buf[write_len++] = pg;
	write_buf[write_len++] = prot;

	// preload read_len with expected length
	read_len = 1;

	// default failure mode 
	last_result_byte = RESULT_FAIL_NONE;

	// if(ds28e30_standard_cmd_flow(write_buf, DELAY_DS28E30_EE_WRITE_TWM, read_buf, &read_len, write_len))
	if (standard_cmd_flow
	    (write_buf, write_len, DELAY_DS28E30_EE_WRITE_TWM, read_len,
	     read_buf, &read_len)) {
		// get result byte
		last_result_byte = read_buf[0];
		// check result
		if (read_len == 1)
			return (read_buf[0] == RESULT_SUCCESS);
	}

	ow_reset();
	// no payload in read buffer or failed command
	return DS_FALSE;
}

//--------------------------------------------------------------------------
/// 'Decrement Counter' command
///
///  @return
///  DS_TRUE - command successful @n
///  DS_FALSE - command failed
///
int ds28e30_cmd_decrementCounter(void)
{
	int write_len;
	unsigned char write_buf[10];
	unsigned char read_buf[255];
	int read_len;

	/*
	   Presence Pulse
	   <ROM Select>
	   TX: XPC Command (66h)
	   TX: Length byte 1d 
	   TX: XPC sub-command C9h (Decrement Counter)
	   RX: CRC16 (inverted of XPC command, length, sub-command)
	   TX: Release Byte
	   <Delay TBD>
	   RX: Dummy Byte
	   RX: Length Byte (1d)
	   RX: Result Byte
	   RX: CRC16 (inverted, length byte and result byte)
	   Reset or send XPC command (66h) for a new sequence
	 */

	// construct the write buffer
	write_len = 0;
	write_buf[write_len++] = CMD_DECREMENT_CNT;

	// preload read_len with expected length
	read_len = 1;

	// default failure mode 
	last_result_byte = RESULT_FAIL_NONE;

	// if(ds28e30_standard_cmd_flow(write_buf, DELAY_DS28E30_EE_WRITE_TWM+50, read_buf, &read_len, write_len))
	if (standard_cmd_flow
	    (write_buf, write_len, DELAY_DS28E30_EE_WRITE_TWM + 50,
	     read_len, read_buf, &read_len)) {
		// get result byte
		last_result_byte = read_buf[0];
		// check result byte
		if (read_len == 1)
			return (read_buf[0] == RESULT_SUCCESS);
	}

	ow_reset();
	// no payload in read buffer or failed command
	return DS_FALSE;
}

//--------------------------------------------------------------------------
/// 'Device Disable' command
///
/// @param[in] release_sequence
/// 8 byte release sequence to disable device
///
///  @return
///  DS_TRUE - command successful @n
///  DS_FALSE - command failed
///
int ds28e30_cmd_DeviceDisable(unsigned char *release_sequence)
{
	unsigned char write_buf[10];
	int write_len;
	unsigned char read_buf[255];
	int read_len;

	/*
	   Reset
	   Presence Pulse
	   <ROM Select>
	   TX: XPC Command (66h)
	   TX: Length byte 9d 
	   TX: XPC sub-command 33h (Disable command)
	   TX: Release Sequence (8 bytes)
	   RX: CRC16 (inverted of XPC command, length, sub-command, and release sequence)
	   TX: Release Byte
	   <Delay TBD>
	   RX: Dummy Byte
	   RX: Length Byte (1d)
	   RX: Result Byte
	   RX: CRC16 (inverted, length byte and result byte)
	   Reset or send XPC command (66h) for a new sequence
	 */

	// construct the write buffer
	write_len = 0;
	write_buf[write_len++] = CMD_DISABLE_DEVICE;
	memcpy(&write_buf[write_len], release_sequence, 8);
	write_len += 8;

	// preload read_len with expected length
	read_len = 1;

	// default failure mode 
	last_result_byte = RESULT_FAIL_NONE;

	// if(ds28e30_standard_cmd_flow(write_buf, DELAY_DS28E30_EE_WRITE_TWM, read_buf, &read_len, write_len))
	if (standard_cmd_flow
	    (write_buf, write_len, DELAY_DS28E30_EE_WRITE_TWM, read_len,
	     read_buf, &read_len)) {
		// get result byte
		last_result_byte = read_buf[0];
		// check result
		if (read_len == 1)
			return (read_buf[0] == RESULT_SUCCESS);
	}

	ow_reset();
	// no payload in read buffer or failed command
	return DS_FALSE;
}

//--------------------------------------------------------------------------
/// 'authenticated Write memory' command
///
/// @param[in] pg
/// page number to write
/// @param[in] data
/// buffer must be at least 32 bytes
/// @param[in] certificate sig_r
/// @param[in] certificate sig_s

///
///  @return
///  DS_TRUE - command successful @n
///  DS_FALSE - command failed
///
int ds28e30_cmd_authendicatedECDSAWriteMemory(int pg, unsigned char *data,
					      unsigned char *sig_r,
					      unsigned char *sig_s)
{
	unsigned char write_buf[128];
	int write_len;
	unsigned char read_buf[16];
	int read_len;

	/*
	   Reset
	   Presence Pulse
	   <ROM Select>
	   TX: XPC Command (66h)
	   TX: Length byte 98d
	   TX: XPC sub-command 89h (authenticated Write Memory)
	   TX: Parameter
	   TX: New page data (32d bytes)
	   TX: Certificate R&S (64 bytes)
	   RX: CRC16 (inverted of XPC command, length, sub-command, parameter, page data, certificate R&S)
	   TX: Release Byte
	   <Delay TBD>
	   RX: Dummy Byte
	   RX: Length Byte (1d)
	   RX: Result Byte
	   RX: CRC16 (inverted of length and result byte)
	   Reset or send XPC command (66h) for a new sequence
	 */

	// construct the write buffer
	write_len = 0;
	write_buf[write_len++] = CMD_AUTHENTICATE_WRITE;
	write_buf[write_len++] = pg & 0x03;
	memcpy(&write_buf[write_len], data, 32);
	write_len += 32;
	memcpy(&write_buf[write_len], sig_r, 32);
	write_len += 32;
	memcpy(&write_buf[write_len], sig_s, 32);
	write_len += 32;


	// preload read_len with expected length
	read_len = 1;

	// default failure mode
	last_result_byte = RESULT_FAIL_NONE;

	// if(ds28e30_standard_cmd_flow(write_buf, DELAY_DS28E30_EE_WRITE_TWM + DELAY_DS28E30_Verify_ECDSA_Signature_tEVS, read_buf, &read_len, write_len))
	if (standard_cmd_flow
	    (write_buf, write_len,
	     DELAY_DS28E30_EE_WRITE_TWM +
	     DELAY_DS28E30_Verify_ECDSA_Signature_tEVS, read_len, read_buf,
	     &read_len)) {
		// get result byte
		last_result_byte = read_buf[0];
		// check result
		if (read_len == 1)
			return (read_buf[0] == RESULT_SUCCESS);
	}

	ow_reset();
	// no payload in read buffer or failed command
	return DS_FALSE;
}

//--------------------------------------------------------------------------
/// 'Read device public key' command
///
/// @param[in]
/// no param required
/// @param[out] data
/// buffer length must be at least 64 bytes to hold device public key read
///
///  @return
///  DS_TRUE - command successful @n
///  DS_FALSE - command failed
///
int ds28e30_cmd_readDevicePublicKey(unsigned char *data)
{
	if ((ds28e30_cmd_readMemory(PG_DS28E30_PUB_KEY_X, data)) ==
	    DS_FALSE) {
		ds_err("read page28 failed\n");
		return DS_FALSE;
	}
	if ((ds28e30_cmd_readMemory(PG_DS28E30_PUB_KEY_Y, data + 32)) ==
	    DS_FALSE) {
		ds_err("read page29 failed\n");
		return DS_FALSE;
	}
	return DS_TRUE;
}


//--------------------------------------------------------------------------
/// write EEPROM page 1 as an example
///
int ds28e30_func_write_pg1(int pg, unsigned char *data,
			   unsigned char *sig_r, unsigned char *sig_s)
{
	short i;
	unsigned char buf[128];
	unsigned char flag = DS_TRUE;

	TestingItemResult[Program_Page1_Result] = DS_FALSE;
	memset(buf, 0x55, 32);	//set page 1 to all 0x55, just for testing, displace it with your own page 1 data
	for (i = 0; i < WRITE_PG1_RETRY; i++) {
		flag = ds28e30_cmd_writeMemory(PG_USER_EEPROM_1, buf);	//User page 1
		TestingItemResult[Program_Page1_Result] = flag;
		if (flag == DS_FALSE)
			continue;
		flag = ds28e30_cmd_readMemory(PG_USER_EEPROM_1, &buf[32]);	//User page 1
		if ((memcmp(buf, &buf[32], 32)) != 0) {
			TestingItemResult[Program_Page1_Result] = DS_FALSE;
			continue;
		} else {
			TestingItemResult[Program_Page1_Result] = DS_TRUE;
			break;
		}
	}

	return DS_TRUE;
}

//---------------------------------------------------------------------------
//-------- ds28e30 Helper functions
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
/// Return last result byte. Useful if a function fails.
///
/// @return
/// Result byte
///
unsigned char ds28e30_getLastResultByte(void)
{
	return last_result_byte;
}

////////////////////////////////////////////////////////////////////////////////
/*
* retry interface for read romid 
*/
//////////////////////////////////////////////////////////////////////////////// 
int ds28e30_Read_RomID_retry(unsigned char *RomID)
{
	int i, pg;
	unsigned char data[50];

	pg = 0;
	for (i = 0; i < GET_ROM_ID_RETRY; i++) {
		if (ds28e30_cmd_readStatus
		    (pg, data, MANID, HardwareVersion) == DS_TRUE) {
			if (Read_RomID(RomID) == DS_TRUE) {
				return DS_TRUE;
			}
		}
	}

	return DS_FALSE;
}

////////////////////////////////////////////////////////////////////////////////
/*
* retry interface for read page data
*/
////////////////////////////////////////////////////////////////////////////////
int ds28e30_get_page_data_retry(int page, unsigned char *data)
{
	int i;

	if (page >= MAX_PAGENUM)
		return DS_FALSE;

	for (i = 0; i < GET_USER_MEMORY_RETRY; i++) {
		if (ds28e30_cmd_readMemory(PG_USER_EEPROM_0, data) ==
		    DS_TRUE) {
			ds_dbg("mi_page0_data data:\n");
			ds_dbg("%02x %02x %02x %02x %02x %02x %02x %02x",
			       mi_page0_data[0], mi_page0_data[1],
			       mi_page0_data[2], mi_page0_data[3],
			       mi_page0_data[4], mi_page0_data[5],
			       mi_page0_data[6], mi_page0_data[7]);
			ds_dbg("%02x %02x %02x %02x %02x %02x %02x %02x",
			       mi_page0_data[8], mi_page0_data[9],
			       mi_page0_data[10], mi_page0_data[11],
			       mi_page0_data[12], mi_page0_data[13],
			       mi_page0_data[14], mi_page0_data[15]);
			ds_dbg("mi_page1_data data:\n");
			ds_dbg("%02x %02x %02x %02x %02x %02x %02x %02x",
			       mi_page1_data[0], mi_page1_data[1],
			       mi_page1_data[2], mi_page1_data[3],
			       mi_page1_data[4], mi_page1_data[5],
			       mi_page1_data[6], mi_page1_data[7]);
			ds_dbg("%02x %02x %02x %02x %02x %02x %02x %02x",
			       mi_page1_data[8], mi_page1_data[9],
			       mi_page1_data[10], mi_page1_data[11],
			       mi_page1_data[12], mi_page1_data[13],
			       mi_page1_data[14], mi_page1_data[15]);
			return DS_TRUE;
		}
	}

	return DS_FALSE;
}

int ds28e30_get_page_status_retry(unsigned char *data)
{
	int i, pg;

	pg = 0;
	for (i = 0; i < GET_BLOCK_STATUS_RETRY; i++) {
		if (ds28e30_cmd_readStatus
		    (pg, data, MANID, HardwareVersion) == DS_TRUE)
			return DS_TRUE;
	}

	return DS_FALSE;
}

// #endif//end LLT_MODIFY-----------------------------------------------------------------------------------------------------------------------------------------------

/* sysfs group */
static ssize_t ds28e30_ds_readstatus_status_read(struct device *dev, struct device_attribute
						 *attr, char *buf)
{
	int result;
	unsigned char status[16] = { 0x00 };
	int i = 0;
	int count = 0;


	for (i = 0; i < attr_trytimes; i++) {
		result = ds28e30_get_page_status_retry(status);

		if (result == DS_TRUE) {
			count++;
			ds_log("DS28E30_cmd_readStatus success!\n");
		} else {
			ds_log("DS28E30_cmd_readStatus fail!\n");
		}
		ds_dbg
		    ("Status = %02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x\n",
		     status[0], status[1], status[2], status[3], status[4],
		     status[5], status[6], status[7], status[8], status[9],
		     status[10], status[11], status[12], status[13],
		     status[14], status[15]);
		Delay_us(1000);
	}
	ds_log("test done\nsuccess time : %d\n", count);
	return scnprintf(buf, PAGE_SIZE,
			 "Success = %d\nStatus = %02x,%02x,%02x,%02x,%02x,%02x,%102x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x\n",
			 count, status[0], status[1], status[2], status[3],
			 status[4], status[5], status[6], status[7],
			 status[8], status[9], status[10], status[11],
			 status[12], status[13], status[14], status[15]);
}

static ssize_t ds28e30_ds_romid_status_read(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	short status;
	unsigned char RomID[10] = { 0x00 };
	int i = 0;
	int count = 0;

	for (i = 0; i < attr_trytimes; i++) {
		status = ds28e30_Read_RomID_retry(RomID);

		if (status == DS_TRUE) {
			count++;
			ds_log("Read_RomID success!\n");
		} else {
			ds_log("Read_RomID fail!\n");
		}
		ds_dbg("RomID = %02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x\n",
		       RomID[0], RomID[1], RomID[2], RomID[3],
		       RomID[4], RomID[5], RomID[6], RomID[7]);
		Delay_us(1000);
	}
	ds_log("test done\nsuccess time : %d\n", count);
	return scnprintf(buf, PAGE_SIZE,
			 "Success = %d\nRomID = %02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x\n",
			 count, RomID[0], RomID[1], RomID[2], RomID[3],
			 RomID[4], RomID[5], RomID[6], RomID[7]);
}

static ssize_t ds28e30_ds_pagenumber_status_read(struct device *dev, struct device_attribute
						 *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%02x\n", pagenumber);
}

static ssize_t ds28e30_ds_pagenumber_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	int buf_int;

	if (sscanf(buf, "%d", &buf_int) != 1)
		return -EINVAL;

	ds_dbg("new pagenumber = %d\n", buf_int);

	if ((buf_int >= 0) && (buf_int <= 3))
		pagenumber = buf_int;

	return count;
}

static ssize_t ds28e30_ds_pagedata_status_read(struct device *dev, struct device_attribute
					       *attr, char *buf)
{
	int result;
	unsigned char pagedata[16] = { 0x00 };
	int i = 0;
	int count = 0;

	for (i = 0; i < attr_trytimes; i++) {
		result = ds28e30_get_page_data_retry(pagenumber, pagedata);

		if (result == DS_TRUE) {
			count++;
			ds_log("DS28E30_cmd_readMemory success!\n");
		} else {
			ds_log("DS28E30_cmd_readMemory fail!\n");
		}
		ds_dbg
		    ("pagedata = %02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x\n",
		     pagedata[0], pagedata[1], pagedata[2], pagedata[3],
		     pagedata[4], pagedata[5], pagedata[6], pagedata[7],
		     pagedata[8], pagedata[9], pagedata[10], pagedata[11],
		     pagedata[12], pagedata[13], pagedata[14],
		     pagedata[15]);
		Delay_us(1000);
	}
	ds_log("test done\nsuccess time : %d\n", count);
	return scnprintf(buf, PAGE_SIZE,
			 "Success = %d\npagedata = %02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x\n",
			 count, pagedata[0], pagedata[1], pagedata[2],
			 pagedata[3], pagedata[4], pagedata[5],
			 pagedata[6], pagedata[7], pagedata[8],
			 pagedata[9], pagedata[10], pagedata[11],
			 pagedata[12], pagedata[13], pagedata[14],
			 pagedata[15]);
}

static ssize_t ds28e30_ds_pagedata_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	int result;
	unsigned char pagedata[16] = { 0x00 };

	if (sscanf
	    (buf,
	     "%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx",
	     &pagedata[0], &pagedata[1], &pagedata[2], &pagedata[3],
	     &pagedata[4], &pagedata[5], &pagedata[6], &pagedata[7],
	     &pagedata[8], &pagedata[9], &pagedata[10], &pagedata[11],
	     &pagedata[12], &pagedata[13], &pagedata[14],
	     &pagedata[15]) != 16)
		return -EINVAL;

	ds_dbg
	    ("new data = %02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x\n",
	     pagedata[0], pagedata[1], pagedata[2], pagedata[3],
	     pagedata[4], pagedata[5], pagedata[6], pagedata[7],
	     pagedata[8], pagedata[9], pagedata[10], pagedata[11],
	     pagedata[12], pagedata[13], pagedata[14], pagedata[15]);

	result = ds28e30_cmd_writeMemory(pagenumber, pagedata);
	if (result == DS_TRUE)
		ds_log("DS28E30_cmd_writeMemory success!\n");
	else
		ds_log("DS28E30_cmd_writeMemory fail!\n");

	return count;
}

static ssize_t ds28e30_ds_time_status_read(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", attr_trytimes);
}

static ssize_t ds28e30_ds_time_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	int buf_int;

	if (sscanf(buf, "%d", &buf_int) != 1)
		return -EINVAL;

	ds_log("new trytimes = %d\n", buf_int);

	if (buf_int > 0)
		attr_trytimes = buf_int;

	return count;
}

static ssize_t ds28e30_ds_session_seed_status_read(struct device *dev, struct device_attribute
						   *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE,
			 "%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,\n%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,\n%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,\n%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,\n",
			 session_seed[0], session_seed[1], session_seed[2],
			 session_seed[3], session_seed[4], session_seed[5],
			 session_seed[6], session_seed[7], session_seed[8],
			 session_seed[9], session_seed[10],
			 session_seed[11], session_seed[12],
			 session_seed[13], session_seed[14],
			 session_seed[15], session_seed[16],
			 session_seed[17], session_seed[18],
			 session_seed[19], session_seed[20],
			 session_seed[21], session_seed[22],
			 session_seed[23], session_seed[24],
			 session_seed[25], session_seed[26],
			 session_seed[27], session_seed[28],
			 session_seed[29], session_seed[30],
			 session_seed[31]);
}

static ssize_t ds28e30_ds_session_seed_store(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
	if (sscanf
	    (buf,
	     "%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx",
	     &session_seed[0], &session_seed[1], &session_seed[2],
	     &session_seed[3], &session_seed[4], &session_seed[5],
	     &session_seed[6], &session_seed[7], &session_seed[8],
	     &session_seed[9], &session_seed[10], &session_seed[11],
	     &session_seed[12], &session_seed[13], &session_seed[14],
	     &session_seed[15], &session_seed[16], &session_seed[17],
	     &session_seed[18], &session_seed[19], &session_seed[20],
	     &session_seed[21], &session_seed[22], &session_seed[23],
	     &session_seed[24], &session_seed[25], &session_seed[26],
	     &session_seed[27], &session_seed[28], &session_seed[29],
	     &session_seed[30], &session_seed[31]) != 32)
		return -EINVAL;

	return count;
}

static ssize_t ds28e30_ds_challenge_status_read(struct device *dev, struct device_attribute
						*attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE,
			 "%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,\n%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,\n%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,\n%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,\n",
			 challenge[0], challenge[1], challenge[2],
			 challenge[3], challenge[4], challenge[5],
			 challenge[6], challenge[7], challenge[8],
			 challenge[9], challenge[10], challenge[11],
			 challenge[12], challenge[13], challenge[14],
			 challenge[15], challenge[16], challenge[17],
			 challenge[18], challenge[19], challenge[20],
			 challenge[21], challenge[22], challenge[23],
			 challenge[24], challenge[25], challenge[26],
			 challenge[27], challenge[28], challenge[29],
			 challenge[30], challenge[31]);
}

static ssize_t ds28e30_ds_challenge_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	if (sscanf
	    (buf,
	     "%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx",
	     &challenge[0], &challenge[1], &challenge[2], &challenge[3],
	     &challenge[4], &challenge[5], &challenge[6], &challenge[7],
	     &challenge[8], &challenge[9], &challenge[10], &challenge[11],
	     &challenge[12], &challenge[13], &challenge[14],
	     &challenge[15], &challenge[16], &challenge[17],
	     &challenge[18], &challenge[19], &challenge[20],
	     &challenge[21], &challenge[22], &challenge[23],
	     &challenge[24], &challenge[25], &challenge[26],
	     &challenge[27], &challenge[28], &challenge[29],
	     &challenge[30], &challenge[31]) != 32)
		return -EINVAL;

	return count;
}

static ssize_t ds28e30_ds_S_secret_status_read(struct device *dev, struct device_attribute
					       *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE,
			 "%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,\n%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,\n%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,\n%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,\n",
			 S_secret[0], S_secret[1], S_secret[2],
			 S_secret[3], S_secret[4], S_secret[5],
			 S_secret[6], S_secret[7], S_secret[8],
			 S_secret[9], S_secret[10], S_secret[11],
			 S_secret[12], S_secret[13], S_secret[14],
			 S_secret[15], S_secret[16], S_secret[17],
			 S_secret[18], S_secret[19], S_secret[20],
			 S_secret[21], S_secret[22], S_secret[23],
			 S_secret[24], S_secret[25], S_secret[26],
			 S_secret[27], S_secret[28], S_secret[29],
			 S_secret[30], S_secret[31]);
}

static ssize_t ds28e30_ds_S_secret_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	if (sscanf
	    (buf,
	     "%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx,%2hhx",
	     &S_secret[0], &S_secret[1], &S_secret[2], &S_secret[3],
	     &S_secret[4], &S_secret[5], &S_secret[6], &S_secret[7],
	     &S_secret[8], &S_secret[9], &S_secret[10], &S_secret[11],
	     &S_secret[12], &S_secret[13], &S_secret[14], &S_secret[15],
	     &S_secret[16], &S_secret[17], &S_secret[18], &S_secret[19],
	     &S_secret[20], &S_secret[21], &S_secret[22], &S_secret[23],
	     &S_secret[24], &S_secret[25], &S_secret[26], &S_secret[27],
	     &S_secret[28], &S_secret[29], &S_secret[30],
	     &S_secret[31]) != 32)
		return -EINVAL;

	return count;
}

static ssize_t ds28e30_ds_auth_ANON_status_read(struct device *dev, struct device_attribute
						*attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%02x\n", auth_ANON);
}

static ssize_t ds28e30_ds_auth_ANON_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	if (sscanf(buf, "%d", &auth_ANON) != 1)
		return -EINVAL;

	return count;
}

static ssize_t ds28e30_ds_auth_BDCONST_status_read(struct device *dev, struct device_attribute
						   *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%02x\n", auth_BDCONST);
}

static ssize_t ds28e30_ds_auth_BDCONST_store(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
	if (sscanf(buf, "%d", &auth_BDCONST) != 1)
		return -EINVAL;

	return count;
}

static ssize_t ds28e30_ds_Auth_Result_status_read(struct device *dev, struct device_attribute
						  *attr, char *buf)
{
	int result;

	result = AuthenticateDS28E30();
	if (result == DS_TRUE)
		return scnprintf(buf, PAGE_SIZE,
				 "Authenticate success!!!\n");
	else
		return scnprintf(buf, PAGE_SIZE,
				 "Authenticate failed : other reason.\n");
}

static ssize_t ds28e30_ds_page0_data_read(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	int ret;
	unsigned char page0_data[50];
	ret = ds28e30_get_page_data_retry(0, page0_data);
	return scnprintf(buf, PAGE_SIZE,
			 "Page0 data = %02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x\n",
			 page0_data[0], page0_data[1], page0_data[2],
			 page0_data[3], page0_data[4], page0_data[5],
			 page0_data[6], page0_data[7], page0_data[8],
			 page0_data[9], page0_data[10], page0_data[11],
			 page0_data[12], page0_data[13], page0_data[14],
			 page0_data[15]);
	if (ret != DS_TRUE)
		return -EAGAIN;
}

static ssize_t ds28e30_ds_page1_data_read(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	int ret;
	unsigned char page1_data[50];
	ret = ds28e30_get_page_data_retry(1, page1_data);
	return scnprintf(buf, PAGE_SIZE,
			 "Page1 data = %02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x\n",
			 page1_data[0], page1_data[1], page1_data[2],
			 page1_data[3], page1_data[4], page1_data[5],
			 page1_data[6], page1_data[7], page1_data[8],
			 page1_data[9], page1_data[10], page1_data[11],
			 page1_data[12], page1_data[13], page1_data[14],
			 page1_data[15]);
	if (ret != DS_TRUE)
		return -EAGAIN;
}

static ssize_t ds28e30_ds_verify_model_name_read(struct device *dev, struct device_attribute
						 *attr, char *buf)
{
	int ret;
	ret = ds28e30_Read_RomID_retry(mi_romid);
	if (ret == DS_TRUE)
		return scnprintf(buf, PAGE_SIZE, "ds28e30");
	else
		return scnprintf(buf, PAGE_SIZE, "unknown");
}

static ssize_t ds28e30_ds_chip_ok_read(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	int ret;
	int chip_ok_status;
	ret = ds28e30_Read_RomID_retry(mi_romid);
	if (((mi_romid[0] == FAMILY_CODE) && (mi_romid[6] == MI_CID_MSB)
	     && ((mi_romid[5] & 0xf0) == MI_CID_LSB)))
		chip_ok_status = true;
	else
		chip_ok_status = false;
	return scnprintf(buf, PAGE_SIZE, "%d\n", chip_ok_status);
}

static ssize_t ds28e30_ds_cycle_count_read(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	int ret;
	unsigned int cycle_count;
	unsigned char pagedata[16] = { 0x00 };

	ret = ds28e30_get_page_data_retry(DC_PAGE, pagedata);
	if (ret == DS_TRUE) {
		cycle_count = (pagedata[2] << 16) + (pagedata[1] << 8)
		    + pagedata[0];
		cycle_count = DC_INIT_VALUE - cycle_count;
	} else {
		cycle_count = 0;
	}
	return scnprintf(buf, PAGE_SIZE, "%d\n", cycle_count);
}

int DS28E30_cmd_decrementCounter(void)
{
	unsigned char write_buf[255];
	unsigned char read_buf[255];
	int write_len = 0;
	int read_len = 1;

	last_result_byte = RESULT_FAIL_NONE;
	/*
	   ?<Start, device address write>
	   ?TX: Decrement Counter Command
	   ?<Stop>
	   ?<Delay>
	   ?<Start, device address read>
	   ?RX: Length (SMBus) [always 1]
	   ?RX: Result byte
	   ?<Stop>
	 */

	write_buf[write_len++] = 1;
	write_buf[write_len++] = CMD_DECREMENT_CNT;

	if (standard_cmd_flow
	    (write_buf, write_len, DELAY_DS28E30_EE_WRITE_TWM + 50,
	     read_len, read_buf, &read_len)) {
		if (read_len == 1) {
			last_result_byte = read_buf[0];
			if (read_buf[0] == RESULT_SUCCESS)
				return DS_TRUE;
		}
	}

	ow_reset();
	return DS_FALSE;
}

static ssize_t ds28e30_ds_cycle_count_write(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	int ret;

	ret = DS28E30_cmd_decrementCounter();

	return ret;
}

static DEVICE_ATTR(ds_readstatus, S_IRUGO,
		   ds28e30_ds_readstatus_status_read, NULL);
static DEVICE_ATTR(ds_romid, S_IRUGO, ds28e30_ds_romid_status_read, NULL);
static DEVICE_ATTR(ds_pagenumber, S_IRUGO | S_IWUSR | S_IWGRP,
		   ds28e30_ds_pagenumber_status_read,
		   ds28e30_ds_pagenumber_store);
static DEVICE_ATTR(ds_pagedata, S_IRUGO | S_IWUSR | S_IWGRP,
		   ds28e30_ds_pagedata_status_read,
		   ds28e30_ds_pagedata_store);
static DEVICE_ATTR(ds_time, S_IRUGO | S_IWUSR | S_IWGRP,
		   ds28e30_ds_time_status_read, ds28e30_ds_time_store);
static DEVICE_ATTR(ds_session_seed, S_IRUGO | S_IWUSR | S_IWGRP,
		   ds28e30_ds_session_seed_status_read,
		   ds28e30_ds_session_seed_store);
static DEVICE_ATTR(ds_challenge, S_IRUGO | S_IWUSR | S_IWGRP,
		   ds28e30_ds_challenge_status_read,
		   ds28e30_ds_challenge_store);
static DEVICE_ATTR(ds_S_secret, S_IRUGO | S_IWUSR | S_IWGRP,
		   ds28e30_ds_S_secret_status_read,
		   ds28e30_ds_S_secret_store);
static DEVICE_ATTR(ds_auth_ANON, S_IRUGO | S_IWUSR | S_IWGRP,
		   ds28e30_ds_auth_ANON_status_read,
		   ds28e30_ds_auth_ANON_store);
static DEVICE_ATTR(ds_auth_BDCONST, S_IRUGO | S_IWUSR | S_IWGRP,
		   ds28e30_ds_auth_BDCONST_status_read,
		   ds28e30_ds_auth_BDCONST_store);
static DEVICE_ATTR(ds_Auth_Result, S_IRUGO,
		   ds28e30_ds_Auth_Result_status_read, NULL);
static DEVICE_ATTR(ds_page0_data, S_IRUGO,
		   ds28e30_ds_page0_data_read, NULL);
static DEVICE_ATTR(ds_page1_data, S_IRUGO,
		   ds28e30_ds_page1_data_read, NULL);
static DEVICE_ATTR(ds_verify_model_name, S_IRUGO,
		   ds28e30_ds_verify_model_name_read, NULL);
static DEVICE_ATTR(ds_chip_ok, S_IRUGO, ds28e30_ds_chip_ok_read, NULL);
static DEVICE_ATTR(ds_cycle_count, S_IRUGO | S_IWUSR | S_IWGRP,
		   ds28e30_ds_cycle_count_read,
		   ds28e30_ds_cycle_count_write);

static struct attribute *ds_attributes[] = {
	&dev_attr_ds_readstatus.attr,
	&dev_attr_ds_romid.attr,
	&dev_attr_ds_pagenumber.attr,
	&dev_attr_ds_pagedata.attr,
	&dev_attr_ds_time.attr,
	&dev_attr_ds_session_seed.attr,
	&dev_attr_ds_challenge.attr,
	&dev_attr_ds_S_secret.attr,
	&dev_attr_ds_auth_ANON.attr,
	&dev_attr_ds_auth_BDCONST.attr,
	&dev_attr_ds_Auth_Result.attr,
	&dev_attr_ds_page0_data.attr,
	&dev_attr_ds_page1_data.attr,
	&dev_attr_ds_verify_model_name.attr,
	&dev_attr_ds_chip_ok.attr,
	&dev_attr_ds_cycle_count.attr,
	NULL,
};

static const struct attribute_group ds_attr_group = {
	.attrs = ds_attributes,
};

static int ds28e30_start_auth_battery(struct auth_device *auth_dev)
{
	if (AuthenticateDS28E30() == DS_TRUE)
		return 0;

	return -1;
}

static int ds28e30_get_batt_id(struct auth_device *auth_dev, u8 * id)
{
	*id = batt_id;

	return 0;
}

static int ds28e30_probe_done(struct auth_device *auth_dev)
{
	ds_info("%s g_info->probe_done is %d \n", __func__, g_info->probe_done);

	return g_info->probe_done;
}

struct auth_ops ds28e30_auth_ops = {
	.auth_battery = ds28e30_start_auth_battery,
	.get_battery_id = ds28e30_get_batt_id,
	.probe_done = ds28e30_probe_done,
};

/*
 * Check that a DS28E30 answers on the bus: reset/presence, then a
 * Read Status (page 0) XPC header whose CRC16 must check out.
 */
int detect_ds28e30(void)
{
	unsigned char pkt[8] = { 0 };
	int pkt_len;
	int i, retry;

	for (retry = 0; retry < DS_DETECT_RETRY; retry++) {
		ds28e30_power_up();

		if ((ow_reset()) != 0) {
			ds_err("reset device failed\n");
			DS_WAIT_MS(DS_RETRY_WAIT_MS);
			continue;
		}

		write_byte(CMD_SKIP_ROM);
		ds_info("w1w: %02x\n", CMD_SKIP_ROM);

		pkt_len = 0;
		pkt[pkt_len++] = CMD_START;
		pkt[pkt_len++] = 2;
		pkt[pkt_len++] = CMD_READ_STATUS;
		pkt[pkt_len++] = 0;
		for (i = 0; i < pkt_len; i++)
			write_byte(pkt[i]);
		ds_info("w1w: %02x %02x %02x %02x\n",
			pkt[0], pkt[1], pkt[2], pkt[3]);

		pkt[pkt_len++] = read_byte();
		pkt[pkt_len++] = read_byte();
		ds_info("w1r: %02x %02x\n", pkt[4], pkt[5]);

		CRC16 = 0;
		for (i = 0; i < pkt_len; i++)
			docrc16(pkt[i]);

		if (CRC16 == 0xB001)
			return DS_TRUE;

		ds_err("crc error\n");
		DS_WAIT_MS(DS_RETRY_WAIT_MS);
	}

	return DS_FALSE;
}

/* battery vendor from page 0: 'G' (Guangyu) = 0, "NV" (NVT) = 1 */
void ds28e30_get_battery_id(void)
{
	unsigned char buf[64] = { 0 };
	int retry = 0;

	ds_info("get battery id from device\n");

	do {
		if (ds28e30_cmd_readMemory(PG_USER_EEPROM_0, Page0Data) != DS_TRUE) {
			retry++;
			ds_info("readMemory page0 failed, retry cnt: %d\n", retry);
			if (retry == DS_DETECT_RETRY)
				break;
			continue;
		}

		memcpy(buf, Page0Data, 32);
		print_hex_dump(KERN_ERR, "[ds28e30] Page0Data: ", DUMP_PREFIX_NONE,
			       16, 1, buf, 32, false);

		if (Page0Data[0] == 'G')
			batt_id = 0;
		else if (Page0Data[0] == 'N' && Page0Data[1] == 'V')
			batt_id = 1;
		else {
			batt_id = 0xff;
			ds_info("invalid batt_id: %x\n", batt_id);
			retry++;
			continue;
		}

		memcpy(mi_page0_data, Page0Data, 32);
		ds_info("batt_id: %x\n", batt_id);
		break;
	} while (retry != DS_DETECT_RETRY);
}

static int ds28e30_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct ds_data *info;
	unsigned char romid[8];

	ds_info("%s probe 10131900\n", __func__);

	info = devm_kzalloc(&(pdev->dev), sizeof(struct ds_data), GFP_KERNEL);
	if (!info) {
		ds_err("%s alloc mem fail\n", __func__);
		return -ENOMEM;
	}

	if ((!pdev->dev.of_node
	     || !of_device_is_available(pdev->dev.of_node)))
		return -ENODEV;

	info->pdev = pdev;
	info->dev = &(pdev->dev);
	platform_set_drvdata(pdev, info);

	ret = of_property_read_string(pdev->dev.of_node,
		"auth_name", &info->auth_name);
	if (ret < 0) {
		ds_err("%s can not find auth name(%d)\n", __func__, ret);
		info->auth_name = "main_suppiler";
	}

	g_info = info;

	gpio_66_cfg = ioremap(DS_GPIO66_CFG_PHYS, 8);
	if (!gpio_66_cfg) {
		dev_err(&pdev->dev, "<%s>: map gpio-66 cfg register failed", __func__);
		return -EINVAL;
	}
	gpio_66_inout = gpio_66_cfg + 4;

	raw_spin_lock_init(&info->io_lock);
	init_waitqueue_head(&ds28e30_wait);

	if (detect_ds28e30() != DS_TRUE) {
		ds_err("%s ds28e30 device not found\n", __func__);
		return 0;
	}

	ds_err("%s ds28e30 device detected\n", __func__);

	/* ds28e30 need read status first */
	ds28e30_Read_RomID_retry(romid);
	ds28e30_get_battery_id();

	ret = sysfs_create_group(&(info->dev->kobj), &ds_attr_group);
	if (ret) {
		ds_err("%s failed to register sysfs(%d)\n", __func__, ret);
		sysfs_remove_group(&(info->dev->kobj), &ds_attr_group);
		return ret;
	}

	info->auth_dev = auth_device_register(info->auth_name, NULL, info,
					      &ds28e30_auth_ops);
	if (IS_ERR_OR_NULL(info->auth_dev)) {
		ds_err("%s failed to register auth device\n", __func__);
		return PTR_ERR(info->auth_dev);
	}

	info->probe_done = true;
	ds_err("%s probe successfully!\n", __func__);

	return 0;
}

static int ds28e30_remove(struct platform_device *pdev)
{
	kfree(platform_get_drvdata(pdev));

	return 0;
}


static const struct of_device_id ds28e30_of_ids[] = {
	{.compatible = "maxim,ds28e30"},
	{},
};

static struct platform_driver ds28e30_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "maxim,ds28e30",
		   .of_match_table = ds28e30_of_ids,
		   },
	.probe = ds28e30_probe,
	.remove = ds28e30_remove,
};

static int __init ds28e30_init(void)
{
	ds_err("%s enter\n", __func__);
	return platform_driver_register(&ds28e30_driver);
}

static void __exit ds28e30_exit(void)
{
	ds_err("%s enter\n", __func__);

	platform_driver_unregister(&ds28e30_driver);
}

module_init(ds28e30_init);
module_exit(ds28e30_exit);

MODULE_LICENSE("GPL");
