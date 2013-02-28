/*
 * drivers/media/video/msm/ov5640_globaloptics.c
 *
 * Refer to drivers/media/video/msm/mt9d112.c
 * For IC OV5640 of Module GLOBALOPTICS: 5.0Mp, 1/4-Inch System-On-A-Chip (SOC) CMOS Digital Image Sensor
 *
 * Copyright (C) 2009-2010 ZTE Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Created by zhang.shengjie@zte.com.cn
 */
/*-----------------------------------------------------------------------------------------
  when          who                   what, where, why                    comment tag
  --------      -- ----     -------------------------------------    --------------------------
  2011-11-25    wangtao    add fix wb for flash open on black              ZTE_WT_20111125
  2011-08-09   yeganlin   Kenxu add to solve capture brightness not        ZTE_YGL_20110809
                          same random 
  2011-07-18   guo.yl     add flash LED Auto mode flag judgement           GYL_CAM_20110718
  2011-07-11   ygl        Ken modified for SNR test                        ZTE_YGL_CAM_20110711
  2011-06-21   wangtao    kenxu add for reduce noise under dark condition  WT_CAM_20110621
                        neil add color matrix    
  2011-05-17   wangtao    bright is different from snap                    WT_CAM_20110517
  2011-05-06   wangtao   resolve bug of wb cannt save                      WT_CAM_20110506 
  2011-05-03   wangtao   resolve bug of af sleep ov neil                   WT_CAM_20110503 
  2011-04-21   wangtao    optimize contrast /staturation register write avoid direct write  WT_CAM_20110421
  2011-04-18   wangtao      resolve sensor bright flash                   ZTE_CAM_WT_20110418
  2011-04-13   lijing       add sensor configuration                      ZTE_CAM_LJ_20110413
  2011-03-30   lijing       merge settings from skate                     ZTE_LJ_CAM_20110330
  2011-03-10   wangtao      add ov5640 5.0MP effect / wb / bright         WT_CAM_20110127
                            reduce snap time by neil ov  
  2011-01-27   wangtao      add ov5640 5.0MP af setting by set register   WT_CAM_20110127
------------------------------------------------------------------------------------------*/

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include "ov5640.h"
#include <linux/slab.h>
#include <mach/camera.h>
/*-----------------------------------------------------------------------------------------
 *
 * MACRO DEFINITION
 *
 *----------------------------------------------------------------------------------------*/
/*
 * To implement the parallel init process
 */
#define OV5640_PROBE_WORKQUEUE

#if defined(OV5640_PROBE_WORKQUEUE)
#include <linux/workqueue.h>
static struct platform_device *pdev_wq = NULL;
static struct workqueue_struct *ov5640_wq = NULL;
static void ov5640_workqueue(struct work_struct *work);
static DECLARE_WORK(ov5640_cb_work, ov5640_workqueue);
#endif /* defined(OV5640_PROBE_WORKQUEUE) */

/*
 * OV5640 Registers and their values
 */
/* Sensor I2C Board Name */
#define OV5640_I2C_BOARD_NAME "ov5640"

/* Sensor I2C Bus Number (Master I2C Controller: 0) */
#define OV5640_I2C_BUS_ID  (0)

/* Sensor I2C Slave Address */
#define OV5640_SLAVE_WR_ADDR 0x78 /* replaced by "msm_i2c_devices.addr" */
#define OV5640_SLAVE_RD_ADDR 0x79 /* replaced by "msm_i2c_devices.addr" */

/* Sensor I2C Device ID */
#define REG_OV5640_MODEL_ID    0x300A
#define OV5640_MODEL_ID        0x5640

/* SOC Registers */
#define REG_OV5640_SENSOR_RESET     0x001A

/* CAMIO Input MCLK (MHz) */
#define OV5640_CAMIO_MCLK  24000000 // from 12Mhz to 24Mhz

/* GPIO For Lowest-Power mode (SHUTDOWN mode) */
#define OV5640_GPIO_SHUTDOWN_CTL   32

/* GPIO For Sensor Clock Switch */


/* 
 * modify for mclk switch for msm7627_joe
 */
#if defined(CONFIG_MACH_RAISE)
#define OV5640_GPIO_SWITCH_CTL     39
#define OV5640_GPIO_SWITCH_VAL     1
#elif defined(CONFIG_MACH_R750) || defined(CONFIG_MACH_JOE)
#define OV5640_GPIO_SWITCH_CTL     39
#define OV5640_GPIO_SWITCH_VAL     0
#elif defined (CONFIG_MACH_V9PLUS)
#define OV5640_GPIO_SWITCH_CTL     181
#define OV5640_GPIO_SWITCH_VAL     0
#else
#define OV5640_GPIO_SWITCH_VAL     1
#endif


//#define CAPTURE_FRAMERATE 375
//#define PREVIEW_FRAMERATE 1500

#define CAPTURE_FRAMERATE 750
#define PREVIEW_FRAMERATE 1500

static unsigned int ov5640_preview_exposure;
static uint16_t ov5640_gain;
static unsigned short ov5640_preview_maxlines;

//ZTE_YGL_CAM_20111221,modifed for SNR
uint16_t YAVG; //kenxu add for improve noise.
uint16_t WB_T; //neil add for detect wb temperature
int preview_sysclk, preview_HTS;

//yanwei add the interface of touch AF begin
#define OV5640_AF_WINDOW_FULL_WIDTH  80//64
#define OV5640_AF_WINDOW_FULL_HEIGHT 60//48

uint16_t x_ratio = 0;
uint16_t y_ratio = 0;
int32_t ov5640_TouchAF_x = -1;
int32_t ov5640_TouchAF_y = -1;
//yanwei add the interface of touch AF end
extern uint32_t  flash_led_enable;
/* 
 * Auto Exposure Gain Factor (disused)
 * #define AE_GAIN_FACTOR  50    // 2.2: underexposure, 200: overexposure, 50: normal exposure
 */

/*-----------------------------------------------------------------------------------------
 *
 * TYPE DECLARATION
 *
 *----------------------------------------------------------------------------------------*/
struct ov5640_work_t {
    struct work_struct work;
};

static struct ov5640_work_t *ov5640_sensorw;
static struct i2c_client *ov5640_client;

struct ov5640_ctrl_t {
    const struct msm_camera_sensor_info *sensordata;
};

/*-----------------------------------------------------------------------------------------
 *
 * GLOBAL VARIABLE DEFINITION
 *
 *----------------------------------------------------------------------------------------*/
static struct ov5640_ctrl_t *ov5640_ctrl;
/*
static uint32_t g_preview_exposure;
static uint32_t g_preview_line_width;
static uint16_t g_preview_gain,g_preview_gain_low,g_preview_gain_high;
static uint32_t g_preview_frame_rate = 0;
*/
#if 0 //ZTE_ZT_CAM_20101203
static uint16_t temp_l,temp_m,temp_h;
#endif

//DECLARE_MUTEX(ov5640_sem);

/*-----------------------------------------------------------------------------------------
 *
 * FUNCTION DECLARATION
 *
 *----------------------------------------------------------------------------------------*/
static int ov5640_sensor_init(const struct msm_camera_sensor_info *data);
static int ov5640_sensor_config(void __user *argp);
static int ov5640_sensor_release(void);

static int32_t ov5640_i2c_add_driver(void);
static void ov5640_i2c_del_driver(void);

extern int32_t msm_camera_power_backend(enum msm_camera_pwr_mode_t pwr_mode);
extern int msm_camera_clk_switch(const struct msm_camera_sensor_info *data,
                                         uint32_t gpio_switch,
                                         uint32_t switch_val);

/*
 * Get FTM flag to adjust 
 * the initialize process 
 * of camera
 */
#ifdef CONFIG_ZTE_PLATFORM
#ifdef CONFIG_ZTE_FTM_FLAG_SUPPORT
extern int zte_get_ftm_flag(void);
#endif
#endif

/*
 * add flash LED black  mode flag judgement,ZTE_WT_20111125
 */

//modify for preview abnormal in mono mode after snapshot  by lijing ZTE_CAM_LJ_20120627]
static int8_t  zte_effect=0;
static int8_t  zte_sat=2;
static int8_t  zte_contrast=2;
static int8_t  zte_sharpness=2;
static int8_t  zte_brightness=3;
 #ifdef ZTE_FIX_WB_ENABLE
extern  uint32_t flash_led_enable;
static int8_t  zte_wb_mode=0;
static int     zte_disable_wb_auto_flag=0; 
static int     zte_flash_on_fix_wb(void);
static int 	   zte_flash_on_fix_wb_setreg(void);
#endif

/*
  * add Auto mode for flash LED, ZTE_CAM_LJ_20120616
  */
extern  void zte_flash_auto_flag_set_value(int);
 
/*-----------------------------------------------------------------------------------------
 *
 * FUNCTION DEFINITION
 *
 *----------------------------------------------------------------------------------------*/
/*
 * Hard standby of sensor
 * on: =1, enter into hard standby
 * on: =0, exit from hard standby
 *
 * Hard standby mode is set by register of REG_OV5640_STANDBY_CONTROL.
 */
static int ov5640_hard_standby(const struct msm_camera_sensor_info *dev, uint32_t on)
{
    int rc;

    CDBG("%s: entry\n", __func__);

    rc = gpio_request(dev->sensor_pwd, "ov5640");
    if (0 == rc)
    {
        /* ignore "rc" */
        rc = gpio_direction_output(dev->sensor_pwd, on);

        /* time delay for the entry into standby */ 
	mdelay(10);
    }

    gpio_free(dev->sensor_pwd);

    return rc;
}

/*
 * Hard reset: RESET_BAR pin (active LOW)
 * Hard reset has the same effect as the soft reset.
 */
static int __attribute__((unused))ov5640_hard_reset(const struct msm_camera_sensor_info *dev)
{
    int rc = 0;

    CDBG("%s: entry\n", __func__);

    rc = gpio_request(dev->sensor_reset, "ov5640");
    if (0 == rc)
    {
        /* ignore "rc" */
        rc = gpio_direction_output(dev->sensor_reset, 1);

        /* time delay for asserting RESET */
	mdelay(10);
        /* ignore "rc" */
        rc = gpio_direction_output(dev->sensor_reset, 0);
        /*
          * RESET_BAR pulse width: Min 70 EXTCLKs
          * EXTCLKs: = MCLK (i.e., OV5640_CAMIO_MCLK)
          */
	mdelay(10);
        /* ignore "rc" */
        rc = gpio_direction_output(dev->sensor_reset, 1);
        /*
          * Time delay before first serial write: Min 100 EXTCLKs
          * EXTCLKs: = MCLK (i.e., OV5640_CAMIO_MCLK)
          */ 
	mdelay(10);
    }

    gpio_free(dev->sensor_reset);

    return rc;
}

static int32_t ov5640_i2c_txdata(unsigned short saddr,
                                       unsigned char *txdata,
                                       int length)
{
    struct i2c_msg msg[] = {
        {
            .addr  = saddr,
            .flags = 0,
            .len   = length,
            .buf   = txdata,
        },
    };

    if (i2c_transfer(ov5640_client->adapter, msg, 1) < 0)
    {
        pr_err("%s: failed!\n", __func__);
        return -EIO;
    }

    return 0;
}

static int32_t ov5640_i2c_write(unsigned short saddr,
                                    unsigned short waddr,
                                    unsigned short wdata,
                                    enum ov5640_width_t width)
{
    int32_t rc = -EFAULT;
    unsigned char buf[3];

    memset(buf, 0, sizeof(buf));

    switch (width)
    {
        case WORD_LEN:
        {
            buf[0] = (waddr & 0xFF00) >> 8;
            buf[1] = (waddr & 0x00FF);
#if 0 // 16-bit data width 
            buf[2] = (wdata & 0xFF00) >> 8;
            buf[3] = (wdata & 0x00FF);
            rc = ov5640_i2c_txdata(saddr, buf, 4);
#else // 8-bit data width for OV sensor  
            buf[2] = (wdata & 0x00FF);
            rc = ov5640_i2c_txdata(saddr, buf, 3);
#endif
        }
        break;

        case BYTE_LEN:
        {
            buf[0] = (waddr & 0xFF00) >> 8;
            buf[1] = (waddr & 0x00FF);
            buf[2] = (wdata & 0x00FF);
	        rc = ov5640_i2c_txdata(saddr, buf, 3);
        }
        break;

        default:
        {
        }
        break;
    }

    if (rc < 0)
    {
        pr_err("%s: waddr = 0x%x, wdata = 0x%x, failed!\n", __func__, waddr, wdata);
    }

    return rc;
}

static int32_t ov5640_i2c_write_table(struct ov5640_i2c_reg_conf const *reg_conf_tbl,
                                             int len)
{
    uint32_t i;
    int32_t rc = 0;

    for (i = 0; i < len; i++)
    {
        rc = ov5640_i2c_write(ov5640_client->addr,
                               reg_conf_tbl[i].waddr,
                               reg_conf_tbl[i].wdata,
                               reg_conf_tbl[i].width);
        if (rc < 0)
        {
        
            break;
        }

        if (reg_conf_tbl[i].mdelay_time != 0)
        {
            /* time delay for writing I2C data */
            mdelay(reg_conf_tbl[i].mdelay_time);
        }
    }

    return rc;
}

static int ov5640_i2c_rxdata(unsigned short saddr,
                                   unsigned char *rxdata,
                                   int length)
{
    struct i2c_msg msgs[] = {
        {
            .addr  = saddr,
            .flags = 0,
            .len   = 2,
            .buf   = rxdata,
        },
        {
            .addr  = saddr,
            .flags = I2C_M_RD,
            .len   = length,
            .buf   = rxdata,
        },
    };

    if (i2c_transfer(ov5640_client->adapter, msgs, 2) < 0)
    {
        pr_err("%s: failed!\n", __func__);
        return -EIO;
    }

    return 0;
}

static int32_t ov5640_i2c_read(unsigned short saddr,
                                     unsigned short raddr,
                                     unsigned short *rdata,
                                     enum ov5640_width_t width)
{
    int32_t rc = 0;
    unsigned char buf[4];

    if (!rdata)
    {
        pr_err("%s: rdata points to NULL!\n", __func__);
        return -EIO;
    }

    memset(buf, 0, sizeof(buf));

    switch (width)
    {
        case WORD_LEN:
        {
            buf[0] = (raddr & 0xFF00) >> 8;
            buf[1] = (raddr & 0x00FF);

            rc = ov5640_i2c_rxdata(saddr, buf, 2);
            if (rc < 0)
            {
                return rc;
            }

            *rdata = buf[0] << 8 | buf[1];
        }
        break;

        case BYTE_LEN:
        {
            buf[0] = (raddr & 0xFF00) >> 8;
            buf[1] = (raddr & 0x00FF);

            rc = ov5640_i2c_rxdata(saddr, buf, 2);
            if (rc < 0)
            {
                return rc;
            }

            *rdata = buf[0]| 0x0000;
        }
        break;

        default:
        {
        }
        break;
    }

    if (rc < 0)
    {
        pr_err("%s: failed!\n", __func__);
    }

    return rc;
} 
static int32_t ov5640_i2c_read_byte(unsigned short raddr, unsigned short *rdata)
{
	int rc = 0;
	unsigned char buf[2];
	//pr_err("+ov5640_i2c_read_byte\n");
	memset(buf, 0, sizeof(buf));

	buf[0] = (raddr & 0xFF00)>>8;
	buf[1] = (raddr & 0x00FF);

	rc = ov5640_i2c_rxdata(ov5640_client->addr, buf, 1);
	if (rc < 0) {
		pr_err("ov5640_i2c_read_byte failed!\n");
		return rc;
	}

	*rdata = buf[0];

	//pr_err("-ov5640_i2c_read_byte\n");
	return rc;
}

static int32_t ov5640_i2c_write_b_sensor(unsigned int waddr, unsigned short bdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[3];
	memset(buf, 0, sizeof(buf));
	buf[0] = (waddr & 0xFF00) >> 8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = bdata;
	//pr_err("i2c_write_b addr = 0x%x, val = 0x%x\n", waddr, bdata);
	rc = ov5640_i2c_txdata(ov5640_client->addr, buf, 3);
	if (rc < 0)
		pr_err("i2c_write_b failed, addr = 0x%x, val = 0x%x!\n",
			waddr, bdata);
	return rc;
}
static int32_t ov5640_set_lens_roll_off(void)
{
   // return 0;
    int32_t rc = 0;
	return rc;
    CDBG("%s: entry\n", __func__);

    rc = ov5640_i2c_write_table(ov5640_regs.rftbl, ov5640_regs.rftbl_size);

    return rc;
}

static long ov5640_set_brightness(int8_t brightness)
{
    long rc = 0;
    uint16_t tmp_reg = 0;

    pr_err("%s: entry: brightness=%d\n", __func__, brightness);

    switch(brightness)
    {
        case CAMERA_BRIGHTNESS_0:
        {
            tmp_reg = 0;
            rc = ov5640_i2c_read(ov5640_client->addr, 0x5001, &tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg &= 0x00FF;
            tmp_reg |= 0x0080;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5001, tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5587, 0x0030, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            //WT_CAM_20110411 write 5580
            rc = ov5640_i2c_read(ov5640_client->addr, 0x5580, &tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg |= 0x0004;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5580, tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg = 0;
            rc = ov5640_i2c_read(ov5640_client->addr, 0x5588, &tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg &= 0x00FF;
            tmp_reg |= 0x0008;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5588, tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
        }
        break;

        case CAMERA_BRIGHTNESS_1:
        {
            tmp_reg = 0;
            rc = ov5640_i2c_read(ov5640_client->addr, 0x5001, &tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg &= 0x00FF;
            tmp_reg |= 0x0080;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5001, tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5587, 0x0020, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            //WT_CAM_20110411 write 5580
           rc = ov5640_i2c_read(ov5640_client->addr, 0x5580, &tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg |= 0x0004;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5580, tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg = 0;
            rc = ov5640_i2c_read(ov5640_client->addr, 0x5588, &tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg &= 0x00FF;
            tmp_reg |= 0x0008;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5588, tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
        }
        break;

        case CAMERA_BRIGHTNESS_2:
        {
            tmp_reg = 0;
            rc = ov5640_i2c_read(ov5640_client->addr, 0x5001, &tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg &= 0x00FF;
            tmp_reg |= 0x0080;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5001, tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5587, 0x0010, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            //WT_CAM_20110411 write 5580
            rc = ov5640_i2c_read(ov5640_client->addr, 0x5580, &tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg |= 0x0004;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5580, tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }            
            tmp_reg = 0;
            rc = ov5640_i2c_read(ov5640_client->addr, 0x5588, &tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg &= 0x00FF;
            tmp_reg |= 0x0008;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5588, tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }  
        }
        break;

        case CAMERA_BRIGHTNESS_3:
        {
        	  tmp_reg = 0;
            rc = ov5640_i2c_read(ov5640_client->addr, 0x5001, &tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg &= 0x00FF;
            tmp_reg |= 0x0080;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5001, tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5587, 0x0000, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            //WT_CAM_20110411 write 5580
            rc = ov5640_i2c_read(ov5640_client->addr, 0x5580, &tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg |= 0x0004;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5580, tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg = 0;
            rc = ov5640_i2c_read(ov5640_client->addr, 0x5588, &tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg &= 0x00F7;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5588, tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
                  
        }
        break;

        case CAMERA_BRIGHTNESS_4:
        {
        	   tmp_reg = 0;
            rc = ov5640_i2c_read(ov5640_client->addr, 0x5001, &tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg &= 0x00FF;
            tmp_reg |= 0x0080;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5001, tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5587, 0x0010, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            //WT_CAM_20110411 write 5580
           rc = ov5640_i2c_read(ov5640_client->addr, 0x5580, &tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg |= 0x0004;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5580, tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg = 0;
            rc = ov5640_i2c_read(ov5640_client->addr, 0x5588, &tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg &= 0x00F7;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5588, tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            } 
        }
        break;

        case CAMERA_BRIGHTNESS_5:
        {
            tmp_reg = 0;
            rc = ov5640_i2c_read(ov5640_client->addr, 0x5001, &tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg &= 0x00FF;
            tmp_reg |= 0x0080;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5001, tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5587, 0x0020, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            //WT_CAM_20110411 write 5580
            rc = ov5640_i2c_read(ov5640_client->addr, 0x5580, &tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg |= 0x0004;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5580, tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg = 0;
            rc = ov5640_i2c_read(ov5640_client->addr, 0x5588, &tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg &= 0x00F7;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5588, tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
        }
        break;

        case CAMERA_BRIGHTNESS_6:
        {
            tmp_reg = 0;
            rc = ov5640_i2c_read(ov5640_client->addr, 0x5001, &tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg &= 0x00FF;
            tmp_reg |= 0x0080;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5001, tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5587, 0x0030, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            //WT_CAM_20110411 write 5580
            rc = ov5640_i2c_read(ov5640_client->addr, 0x5580, &tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg |= 0x0004;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5580, tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg = 0;
            rc = ov5640_i2c_read(ov5640_client->addr, 0x5588, &tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg &= 0x00F7;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5588, tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }  
        }
        break;

        default:
        {
            pr_err("%s: parameter error!\n", __func__);
            return -EFAULT;
        }            
    }

    return rc;
}

static long ov5640_set_contrast(int8_t contrast_val)
{
    long rc = 0;
    uint16_t tmp_reg = 0;

    pr_err("%s: entry: contrast_val=%d\n", __func__, contrast_val);
	if (zte_effect == CAMERA_EFFECT_OFF) 
	{
    switch(contrast_val)
    {
        case CAMERA_CONTRAST_0:
        {
            //WT_CAM_20110421
            tmp_reg = 0;
            rc = ov5640_i2c_read(ov5640_client->addr, 0x5001, &tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }

            /*
             * ZTE_LJ_CAM_20101026
             * fix bug of no preview image after changing effect mode repeatedly
             */
            tmp_reg &= 0x00FF;
            tmp_reg |= 0x0080;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5001, tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg = 0;
            rc = ov5640_i2c_read(ov5640_client->addr, 0x5580, &tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg &= 0x00FF;
            tmp_reg |= 0x0004;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5580, tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }            
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5586, 0x0010, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }            
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5585, 0x0010, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }            
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5588, 0x0001, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }            
        }
        break;

        case CAMERA_CONTRAST_1:
        {
            //WT_CAM_20110421
            tmp_reg = 0;
            rc = ov5640_i2c_read(ov5640_client->addr, 0x5001, &tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }

            /*
             * ZTE_LJ_CAM_20101026
             * fix bug of no preview image after changing effect mode repeatedly
             */
            tmp_reg &= 0x00FF;
            tmp_reg |= 0x0080;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5001, tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg = 0;
            rc = ov5640_i2c_read(ov5640_client->addr, 0x5580, &tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg &= 0x00FF;
            tmp_reg |= 0x0004;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5580, tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }            
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5586, 0x0018, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }            
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5585, 0x0018, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }            
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5588, 0x0001, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }            
        }
        break;

        case CAMERA_CONTRAST_2:
        {
            //WT_CAM_20110421
            tmp_reg = 0;
            rc = ov5640_i2c_read(ov5640_client->addr, 0x5001, &tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }

            /*
             * ZTE_LJ_CAM_20101026
             * fix bug of no preview image after changing effect mode repeatedly
             */
            tmp_reg &= 0x00FF;
            tmp_reg |= 0x0080;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5001, tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg = 0;
            rc = ov5640_i2c_read(ov5640_client->addr, 0x5580, &tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg &= 0x00FF;
            tmp_reg |= 0x0004;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5580, tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }            
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5586, 0x0020, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }            
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5585, 0x0000, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }            
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5588, 0x0001, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }            
        }
        break;

        case CAMERA_CONTRAST_3:
        {
            //WT_CAM_20110421
            tmp_reg = 0;
            rc = ov5640_i2c_read(ov5640_client->addr, 0x5001, &tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }

            /*
             * ZTE_LJ_CAM_20101026
             * fix bug of no preview image after changing effect mode repeatedly
             */
            tmp_reg &= 0x00FF;
            tmp_reg |= 0x0080;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5001, tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg = 0;
            rc = ov5640_i2c_read(ov5640_client->addr, 0x5580, &tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg &= 0x00FF;
            tmp_reg |= 0x0004;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5580, tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }            
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5586, 0x0024, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }            
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5585, 0x0010, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }            
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5588, 0x0001, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }            
        }
        break;

        case CAMERA_CONTRAST_4:
        {
            //WT_CAM_20110421
            tmp_reg = 0;
            rc = ov5640_i2c_read(ov5640_client->addr, 0x5001, &tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }

            /*
             * ZTE_LJ_CAM_20101026
             * fix bug of no preview image after changing effect mode repeatedly
             */
            tmp_reg &= 0x00FF;
            tmp_reg |= 0x0080;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5001, tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg = 0;
            rc = ov5640_i2c_read(ov5640_client->addr, 0x5580, &tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg &= 0x00FF;
            tmp_reg |= 0x0004;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5580, tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }            
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5586, 0x002c, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }            
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5585, 0x001c, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }            
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5588, 0x0001, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }            
        }        
        break;

        default:
        {
            pr_err("%s: parameter error!\n", __func__);
            return -EFAULT;
        }            
    }
		}
    return rc;
}

static long ov5640_set_saturation(int8_t saturation_val)
{
    long rc = 0;
    uint16_t tmp_reg = 0;



    pr_err("%s: entry: saturation_val=%d\n", __func__, saturation_val);
	if (zte_effect == CAMERA_EFFECT_OFF)
	{ 
    switch(saturation_val)
    {
        case CAMERA_SATURATION_0:
        {
        	
            //WT_CAM_20110421
            tmp_reg = 0;
            rc = ov5640_i2c_read(ov5640_client->addr, 0x5001, &tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }

	/*
	* ZTE_LJ_CAM_20101026
	* fix bug of no preview image after changing effect mode repeatedly
	*/
	tmp_reg &= 0x00FF;
	tmp_reg |= 0x0080;
	rc = ov5640_i2c_write(ov5640_client->addr, 0x5001, tmp_reg, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}
	rc = ov5640_i2c_write(ov5640_client->addr, 0x5583, 0x0020, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}            

	rc = ov5640_i2c_write(ov5640_client->addr, 0x5584, 0x0010, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}
	tmp_reg = 0;
	rc = ov5640_i2c_read(ov5640_client->addr, 0x5580, &tmp_reg, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}
	tmp_reg &= 0x00FF;
	tmp_reg |= 0x0002;
	rc = ov5640_i2c_write(ov5640_client->addr, 0x5580, tmp_reg, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}    
	tmp_reg = 0;

	rc = ov5640_i2c_read(ov5640_client->addr, 0x5588, &tmp_reg, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}
	tmp_reg &= 0x00FF;
	tmp_reg |= 0x0040;
	rc = ov5640_i2c_write(ov5640_client->addr, 0x5588, tmp_reg, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}              
	}
	break;

        case CAMERA_SATURATION_1:
        {
            //WT_CAM_20110421
            tmp_reg = 0;
            rc = ov5640_i2c_read(ov5640_client->addr, 0x5001, &tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }

	/*
	* ZTE_LJ_CAM_20101026
	* fix bug of no preview image after changing effect mode repeatedly
	*/
	tmp_reg &= 0x00FF;
	tmp_reg |= 0x0080;
	rc = ov5640_i2c_write(ov5640_client->addr, 0x5001, tmp_reg, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}
	rc = ov5640_i2c_write(ov5640_client->addr, 0x5583, 0x0038, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}            

	rc = ov5640_i2c_write(ov5640_client->addr, 0x5584, 0x0020, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}            
	tmp_reg = 0;
	rc = ov5640_i2c_read(ov5640_client->addr, 0x5580, &tmp_reg, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}
	tmp_reg &= 0x00FF;
	tmp_reg |= 0x0002;
	rc = ov5640_i2c_write(ov5640_client->addr, 0x5580, tmp_reg, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}  
	tmp_reg = 0;
	rc = ov5640_i2c_read(ov5640_client->addr, 0x5588, &tmp_reg, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}
	tmp_reg &= 0x00FF;
	tmp_reg |= 0x0040;
	rc = ov5640_i2c_write(ov5640_client->addr, 0x5588, tmp_reg, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}      
	}
	break;

	case CAMERA_SATURATION_2:
	{
	//WT_CAM_20110421
	tmp_reg = 0;
	rc = ov5640_i2c_read(ov5640_client->addr, 0x5001, &tmp_reg, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}

	/*
	* ZTE_LJ_CAM_20101026
	* fix bug of no preview image after changing effect mode repeatedly
	*/
	tmp_reg &= 0x00FF;
	tmp_reg |= 0x0080;
	rc = ov5640_i2c_write(ov5640_client->addr, 0x5001, tmp_reg, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}
	rc = ov5640_i2c_write(ov5640_client->addr, 0x5583, 0x0040, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}            
	rc = ov5640_i2c_write(ov5640_client->addr, 0x5584, 0x001e, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}            
	tmp_reg = 0;
	rc = ov5640_i2c_read(ov5640_client->addr, 0x5580, &tmp_reg, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}
	tmp_reg &= 0x00FF;
	tmp_reg |= 0x0002;

	rc = ov5640_i2c_write(ov5640_client->addr, 0x5580, tmp_reg, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	} 
	tmp_reg = 0;
	rc = ov5640_i2c_read(ov5640_client->addr, 0x5588, &tmp_reg, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}
	tmp_reg &= 0x00FF;
	tmp_reg &= 0x00bf;
	rc = ov5640_i2c_write(ov5640_client->addr, 0x5588, tmp_reg, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}      
	}
	break;

	case CAMERA_SATURATION_3:
	{
	//WT_CAM_20110421
	tmp_reg = 0;
	rc = ov5640_i2c_read(ov5640_client->addr, 0x5001, &tmp_reg, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}

	/*
	* ZTE_LJ_CAM_20101026
	* fix bug of no preview image after changing effect mode repeatedly
	*/
	tmp_reg &= 0x00FF;
	tmp_reg |= 0x0080;
	rc = ov5640_i2c_write(ov5640_client->addr, 0x5001, tmp_reg, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}
	rc = ov5640_i2c_write(ov5640_client->addr, 0x5583, 0x0050, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}            

	rc = ov5640_i2c_write(ov5640_client->addr, 0x5584, 0x0038, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}
	tmp_reg = 0;
	rc = ov5640_i2c_read(ov5640_client->addr, 0x5580, &tmp_reg, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}
	tmp_reg &= 0x00FF;
	tmp_reg |= 0x0002;
	rc = ov5640_i2c_write(ov5640_client->addr, 0x5580, tmp_reg, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}  
	tmp_reg = 0;
	rc = ov5640_i2c_read(ov5640_client->addr, 0x5588, &tmp_reg, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}
	tmp_reg &= 0x00FF;
	tmp_reg |= 0x0040;
	rc = ov5640_i2c_write(ov5640_client->addr, 0x5588, tmp_reg, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}      
	}
	break;

	case CAMERA_SATURATION_4:
	{
	//WT_CAM_20110421
	tmp_reg = 0;
	rc = ov5640_i2c_read(ov5640_client->addr, 0x5001, &tmp_reg, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}

	/*
	* ZTE_LJ_CAM_20101026
	* fix bug of no preview image after changing effect mode repeatedly
	*/
	tmp_reg &= 0x00FF;
	tmp_reg |= 0x0080;
	rc = ov5640_i2c_write(ov5640_client->addr, 0x5001, tmp_reg, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}
	rc = ov5640_i2c_write(ov5640_client->addr, 0x5583, 0x0060, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}            

	rc = ov5640_i2c_write(ov5640_client->addr, 0x5584, 0x0048, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}
	tmp_reg = 0;
	rc = ov5640_i2c_read(ov5640_client->addr, 0x5580, &tmp_reg, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}
	tmp_reg &= 0x00FF;
	tmp_reg |= 0x0002;
	rc = ov5640_i2c_write(ov5640_client->addr, 0x5580, tmp_reg, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}  
	tmp_reg = 0;
	rc = ov5640_i2c_read(ov5640_client->addr, 0x5588, &tmp_reg, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}
	tmp_reg &= 0x00FF;
	tmp_reg |= 0x0040;
	rc = ov5640_i2c_write(ov5640_client->addr, 0x5588, tmp_reg, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}      
	}        
	break;

	default:
	{
	pr_err("%s: parameter error!\n", __func__);
	return -EFAULT;
	}            
	}
		}

    return rc;
}
/*
  * add Auto mode for flash LED, ZTE_CAM_LJ_20120616
  */
long ov5640_flash_auto_mode_flag_judge(void)
{
 long rc = 0;
  
uint16_t g_preview_gain_flash;
uint16_t ov5640_auto_flash_threshold = 0xF0;
       
    rc = ov5640_i2c_read(ov5640_client->addr, 0x350b, &g_preview_gain_flash, BYTE_LEN);

    if (rc < 0)
    {
        return rc;
    }
	
  if( g_preview_gain_flash >= ov5640_auto_flash_threshold )
  	{//zte_flash_auto_flag=1
       zte_flash_auto_flag_set_value(1);
  }
  else
  	{//zte_flash_auto_flag=0
  	zte_flash_auto_flag_set_value(0);
  }
 return 0;  

}

//===============AE start=================
int XVCLK = 2400;	// real clock/10000

static int ov5640_get_sysclk(void)
{
//	int8_t buf[2];
	unsigned short  buf[2];
	 // calculate sysclk
	 int Multiplier, PreDiv, VCO, SysDiv, Pll_rdiv, Bit_div2x = 1, sclk_rdiv, sysclk;

	 int sclk_rdiv_map[] = {
		 1, 2, 4, 8};

	ov5640_i2c_read_byte(0x3034, buf);
	buf[1] = buf[0] & 0x0f;
	 if (buf[1] == 8 || buf[1] == 10) {
		 Bit_div2x = buf[1] / 2;
	 }

	ov5640_i2c_read_byte(0x3035, buf);
	 SysDiv = buf[0] >>4;
	 if(SysDiv == 0) {
		 SysDiv = 16;
	 }

	ov5640_i2c_read_byte(0x3036, buf);
	 Multiplier = buf[0];

	ov5640_i2c_read_byte(0x3037, buf);
	 PreDiv = buf[0] & 0x0f;
	 Pll_rdiv = ((buf[0] >> 4) & 0x01) + 1;

	ov5640_i2c_read_byte(0x3108, buf);
	 buf[1] = buf[0] & 0x03;
	 sclk_rdiv = sclk_rdiv_map[buf[1]]; 

	 VCO = XVCLK * Multiplier / PreDiv;

	 sysclk = VCO / SysDiv / Pll_rdiv * 2 / Bit_div2x / sclk_rdiv;

	 return sysclk;
}

static int ov5640_get_HTS(void)
{
	//int8_t buf[2];
	unsigned short buf[2];
	 // read HTS from register settings
	 int HTS;

	ov5640_i2c_read_byte(0x380c, buf);
	ov5640_i2c_read_byte(0x380d, &buf[1]);
	 HTS=buf[0];
	 HTS = (HTS<<8) + buf[1];

	 return HTS;
}

static int ov5640_get_VTS(void)
{
	//int8_t buf[2];
	unsigned short  buf[2];
	 // read VTS from register settings
	 int VTS;

	ov5640_i2c_read_byte(0x380e, buf);
	printk("%s: 0x380e=0x%x\n", __func__, buf[0]);
	ov5640_i2c_read_byte(0x380f, &buf[1]);
	printk("%s: 0x380e=0x%x\n", __func__, buf[1]);
	VTS = buf[0];
	VTS = VTS<<8;
	VTS += (unsigned char)buf[1];
	printk("%s: VTS=0x%x\n", __func__, VTS);

	 return VTS;
}

static int ov5640_set_VTS(int VTS)
{
	unsigned short buf[2];
	 // write VTS to registers
	 

	 //temp = VTS & 0xff;
	 buf[0] = VTS & 0xff;
	 printk("%s: VTS & oxff = 0x%x\n", __func__, buf[0]);
	 ov5640_i2c_write_b_sensor(0x380f, buf[0]);

	 buf[0] = VTS>>8;
	 printk("%s: VTS>>8 = 0x%x\n", __func__, buf[0]);
	 ov5640_i2c_write_b_sensor(0x380e, buf[0]);

	 return 0;
}

/*
static int ov5640_get_shutter(void)
{
	 // read shutter, in number of line period
	 int shutter;
	 //int8_t buf[2];
	 unsigned short  buf[2];
	 int temp;

	ov5640_i2c_read_byte(0x3500, buf);
	 shutter = (buf[0] & 0x0f);
	 printk("%s: 0x3500=0x%x\n", __func__, buf[0]);
	 ov5640_i2c_read_byte(0x3501, buf);
	 shutter = (shutter<<8) + buf[0];
	 printk("%s: 0x3501=0x%x\n", __func__, buf[0]);
	 ov5640_i2c_read_byte(0x3502, buf);
	 temp = buf[0];
	 shutter = (shutter<<4) + (temp>>4);
	 printk("%s: 0x3502=0x%x\n", __func__, buf[0]);

	 return shutter;
}
*/
static int ov5640_set_shutter(int shutter)
{
	 // write shutter, in number of line period
	unsigned short buf[2];
	 int temp;
	 
	 shutter = shutter & 0xffff;

	 temp = shutter & 0x0f;
	 buf[0] = temp<<4;
	 printk("%s: shutter&0x0f <<4 0x%x\n", __func__, buf[0]);
	 ov5640_i2c_write_b_sensor(0x3502, buf[0]);

	 temp = shutter & 0xfff;
	 buf[0] = temp>>4;
	 printk("%s: shutter&0xfff >>4 0x%x\n", __func__, buf[0]);
	 ov5640_i2c_write_b_sensor(0x3501,buf[0]);

	 temp = shutter>>12;
	 buf[0] = temp;
	 printk("%s: shutter>>12 0x%x\n", __func__, buf[0]);
	 ov5640_i2c_write_b_sensor(0x3500, buf[0]);

	 return 0;
}

/*
static int ov5640_get_gain16(void)
{
	 // read gain, 16 = 1x
	 int gain16;
	 //int8_t buf[2];
	 unsigned short buf[2];

	ov5640_i2c_read_byte(0x350a, buf);
	 //buf[0] = buf[0] & 0x03;
	 gain16 = buf[0] & 0x03;
	 printk("%s: 0x350a=0x%x\n", __func__, buf[0]);
	 //ov5640_i2c_read_byte(0x350b, &buf[1], 1, BYTE_LEN);
	 ov5640_i2c_read_byte(0x350b, buf);
	 printk("%s: 0x350b=0x%x\n", __func__, (int8_t)buf[0]);
	 printk("%s: 0x350b=%d\n", __func__, buf[0]);

	 gain16 = (gain16<<8) + buf[0];
	 printk("%s: gain16=0x%x\n", __func__, gain16);

	 return gain16;
}
*/
static int ov5640_set_gain16(int gain16)
{
	 // write gain, 16 = 1x
	 int16_t buf[2];
	 //unsigned short buf[2];
	 
	 gain16 = gain16 & 0x3ff;

	 buf[0] = gain16 & 0xff;
	 ov5640_i2c_write_b_sensor(0x350b, buf[0]);

	 buf[0] = gain16>>8;
	 ov5640_i2c_write_b_sensor(0x350a, buf[0]);

	 return 0;
}

	static int ov5640_get_light_frequency(void)
	{
	// get banding filter value
	int light_frequency = 0;
	unsigned short buf[2];

	ov5640_i2c_read_byte(0x3c01, buf);

	 if (buf[0] & 0x80) {
		 // manual
		 ov5640_i2c_read_byte(0x3c00, &buf[1]);
		 if (buf[1] & 0x04) {
			 // 50Hz
			 light_frequency = 50;
		 }
		 else {
			 // 60Hz
			 light_frequency = 60;
		 }
	 }
	 else {
		 // auto
		 ov5640_i2c_read_byte(0x3c0c, &buf[1]);
		 if (buf[1] & 0x01) {
			 // 50Hz
			 light_frequency = 50;
		 }
		 else {
			 // 60Hz
		 }
	 }
	 return light_frequency;
}


	static void ov5640_set_bandingfilter(void)
	{
	int16_t buf[2];
	//unsigned short buf[2];
	int preview_VTS;
	int band_step60, max_band60, band_step50, max_band50;

	 // read preview PCLK
	 preview_sysclk = ov5640_get_sysclk();

	 // read preview HTS
	 preview_HTS = ov5640_get_HTS();

	 // read preview VTS
	 preview_VTS = ov5640_get_VTS();

	 // calculate banding filter
	 // 60Hz
	 band_step60 = preview_sysclk * 100/preview_HTS * 100/120;
	 buf[0] = band_step60 >> 8;
	 ov5640_i2c_write_b_sensor(0x3a0a, buf[0]);
	 buf[0] = band_step60 & 0xff;
	 ov5640_i2c_write_b_sensor(0x3a0b, buf[0]);

	 max_band60 = (int)((preview_VTS-4)/band_step60);
	 buf[0] = (int8_t)max_band60;
	 ov5640_i2c_write_b_sensor(0x3a0d, buf[0]);

	 // 50Hz
	 band_step50 = preview_sysclk * 100/preview_HTS; 
	 buf[0] = (int8_t)(band_step50 >> 8);
	 ov5640_i2c_write_b_sensor(0x3a08, buf[0]);
	buf[0] = (int8_t)(band_step50 & 0xff);
	 ov5640_i2c_write_b_sensor(0x3a09, buf[0]);

	 max_band50 = (int)((preview_VTS-4)/band_step50);
	 buf[0] = (int8_t)max_band50;
	 ov5640_i2c_write_b_sensor(0x3a0e,buf[0]);
}
	/*
	static int ov5640_set_AE_target(int target)
	{
	int16_t buf[2];
	//unsigned short buf[2];
	// stable in high
	int fast_high, fast_low;
	AE_low = target * 23 / 25;	// 0.92
	AE_high = target * 27 / 25;	// 1.08

	 fast_high = AE_high<<1;
	 if(fast_high>255)
		 fast_high = 255;

	 fast_low = AE_low>>1;

	buf[0] = (int8_t)AE_high;
	ov5640_i2c_write_b_sensor(0x3a0f, buf[0]);
	buf[0] = (int8_t)AE_low;
	 ov5640_i2c_write_b_sensor(0x3a10, buf[0]);
	 buf[0] = (int8_t)AE_high;
	 ov5640_i2c_write_b_sensor(0x3a1b, buf[0]);
	  buf[0] = (int8_t)AE_low;
	 ov5640_i2c_write_b_sensor(0x3a1e, buf[0]);
	 buf[0] = (int8_t)fast_high;
	 ov5640_i2c_write_b_sensor(0x3a11, buf[0]);
	 buf[0] = (int8_t)fast_low;
	 ov5640_i2c_write_b_sensor(0x3a1f, buf[0]);

	 return 0;
}

*/

	static long ov5640_hw_ae_parameter_record(void)
	{
	int rc = 0;
	uint16_t UV, temp;
	uint16_t ret_l,ret_m,ret_h, LencQ_H, LencQ_L;
      pr_err("%s: entry\n", __func__);
	ov5640_i2c_write_b_sensor(0x3503, 0x07);
	ov5640_i2c_write_b_sensor(0x5196, 0x23); //freeze awb kenxu update @20120516

	//================debug start========================
	ov5640_i2c_read_byte(0x3c01, &temp);
	pr_err("0x3c01=0x%x\n", temp);
	ov5640_i2c_read_byte(0x3c00, &temp);
	pr_err("0x3c00=0x%x\n", temp);
	ov5640_i2c_read_byte(0x3a08, &temp);
	pr_err("0x3a08=0x%x\n", temp);
	ov5640_i2c_read_byte(0x3a09, &temp);
	pr_err("0x3a09=0x%x\n", temp);

	ov5640_i2c_read_byte(0x3a0a, &temp);
	pr_err("0x3a0a=0x%x\n", temp);
	ov5640_i2c_read_byte(0x3a0b, &temp);
	pr_err("0x3a0b=0x%x\n", temp);
	ov5640_i2c_read_byte(0x3a0d, &temp);
	pr_err("0x3a0d=0x%x\n", temp);
	ov5640_i2c_read_byte(0x3a0e, &temp);
	pr_err("0x3a0e=0x%x\n", temp);


	ov5640_i2c_read_byte(0x3034, &temp);
	pr_err("0x3034=0x%x\n", temp);
	ov5640_i2c_read_byte(0x3035, &temp);
	pr_err("0x3035=0x%x\n", temp);
	ov5640_i2c_read_byte(0x3036, &temp);
	pr_err("0x3036=0x%xn", temp);
	ov5640_i2c_read_byte(0x3037, &temp);
	pr_err("0x3037=0x%x\n", temp);


	ov5640_i2c_read_byte(0x3108, &temp);
	pr_err("0x3108=0x%x\n", temp);
	ov5640_i2c_read_byte(0x3824, &temp);
	pr_err("0x3824=0x%x\n", temp);
	ov5640_i2c_read_byte(0x460c, &temp);
	pr_err("0x460c=0x%x\n", temp);
	ov5640_i2c_read_byte(0x300e, &temp);
	pr_err("0x300e=0x%x\n", temp);


	ov5640_i2c_read_byte(0x380c, &temp);
	pr_err("0x380c=0x%x\n", temp);
	ov5640_i2c_read_byte(0x380d, &temp);
	pr_err("0x380d=0x%x\n", temp);
	ov5640_i2c_read_byte(0x380e, &temp);
	pr_err("0x380e=0x%x\n", temp);
	ov5640_i2c_read_byte(0x380f, &temp);
	pr_err("0x380f=0x%x\n", temp);


	ov5640_i2c_read_byte(0x5588, &temp);
	pr_err("0x5588=0x%x\n", temp);
	ov5640_i2c_read_byte(0x5583, &temp);
	pr_err("0x5583=0x%x\n", temp);
	ov5640_i2c_read_byte(0x5584, &temp);
	pr_err("0x5584=0x%x\n", temp);

	ov5640_i2c_read_byte(0x5580, &temp);
	pr_err("0x5580=0x%x\n", temp);
	ov5640_i2c_read_byte(0x5001, &temp);
	pr_err("0x5001=0x%x\n", temp);
	ov5640_i2c_read_byte(0x558c, &temp);
	pr_err("0x558c=0x%x\n", temp);
	ov5640_i2c_read_byte(0x5384, &temp);
	pr_err("0x5384=0x%x\n", temp);
	ov5640_i2c_read_byte(0x5385, &temp);
	pr_err("0x5385=0x%x\n", temp);
	ov5640_i2c_read_byte(0x5386, &temp);
	pr_err("0x5386=0x%x\n", temp);
	ov5640_i2c_read_byte(0x5387, &temp);
	pr_err("0x5387=0x%x\n", temp);
	ov5640_i2c_read_byte(0x5388, &temp);
	pr_err("0x5388=0x%x\n", temp);
	ov5640_i2c_read_byte(0x5389, &temp);
	pr_err("0x5389=0x%x\n", temp);
	ov5640_i2c_read_byte(0x538a, &temp);
	pr_err("0x538a=0x%x\n", temp);
	ov5640_i2c_read_byte(0x538b, &temp);
	pr_err("0x538b=0x%x\n", temp);

	//================debug end========================


#if 1
	//modify for preview abnormal in mono mode after snapshot  by lijing ZTE_CAM_LJ_20120627
	if(CAMERA_EFFECT_OFF == zte_effect) {
		//keep saturation same for preview and capture
		pr_err("lijing:effect off \n");
		ov5640_i2c_read_byte(0x558c, &UV);
		ov5640_i2c_read_byte(0x5588, &temp);
		temp = temp | 0x40;
		ov5640_i2c_write_b_sensor(0x5588, temp); //Manual UV
		ov5640_i2c_write_b_sensor(0x5583, UV);
		ov5640_i2c_write_b_sensor(0x5584, UV);
		printk("preview_UV=%d\n", UV);
	}
    #endif

	//keep Lenc same for preview and capture
	ov5640_i2c_read_byte(0x350a, &LencQ_H);
	ov5640_i2c_read_byte(0x350b, &LencQ_L);
	ov5640_i2c_write_b_sensor(0x5054, LencQ_H);
	ov5640_i2c_write_b_sensor(0x5055, LencQ_L);	
	ov5640_i2c_write_b_sensor(0x504d, 0x02);	//Manual mini Q	

    //get preview exp & gain
    ret_h = ret_m = ret_l = 0;
    ov5640_preview_exposure = 0;
    ov5640_i2c_read_byte(0x3500, &ret_h);
    ov5640_i2c_read_byte(0x3501, &ret_m);
    ov5640_i2c_read_byte(0x3502, &ret_l);
    ov5640_preview_exposure = (ret_h << 12) + (ret_m << 4) + (ret_l >> 4);
//    printk("preview_exposure=%d\n", ov5640_preview_exposure);

    ret_h = ret_m = ret_l = 0;
    ov5640_preview_maxlines = 0;
    ov5640_i2c_read_byte(0x380e, &ret_h);
    ov5640_i2c_read_byte(0x380f, &ret_l);
    ov5640_preview_maxlines = (ret_h << 8) + ret_l;
//    printk("Preview_Maxlines=%d\n", ov5640_preview_maxlines);

    //Read back AGC Gain for preview
    ov5640_gain = 0;
    ov5640_i2c_read_byte(0x350b, &ov5640_gain);
//    printk("Gain,0x350b=0x%x\n", ov5640_gain);

	//ZTE_YGL_CAM_20111221,modifed for SNR
	YAVG = 0;
	ov5640_i2c_read_byte(0x56A1, &YAVG);
	WB_T = 0;
	ov5640_i2c_read_byte(0x51d0, &WB_T);
	pr_err("YAVG=0x%x\n", YAVG);
	pr_err("WB_T=0x%x\n", WB_T);

 // read preview PCLK	 
 preview_sysclk = ov5640_get_sysclk();	 
 // read preview HTS	 
 preview_HTS = ov5640_get_HTS();
	
  #ifdef ZTE_FIX_WB_ENABLE
	zte_flash_on_fix_wb();
#endif

	return rc;
	}

/*
 * Auto Focus Trigger
 * WT_CAM_20110127 set af register to enable af function 
 */
static int32_t ov5640_af_trigger(void)
	{
	int32_t rc;
	int32_t rc_3028;

	uint16_t af_ack=0x0002;
	uint32_t i;
	uint16_t /*af_status_3024,af_status_3025, af_status_3026, af_status_3027,*/af_status_3028;
	CDBG("%s: entry\n", __func__);
	pr_err("yanwei ov5640_af_trigger: x_ratio=%d,y_ratio=%d,ov5640_TouchAF_x=%d,ov5640_TouchAF_y=%d\n", x_ratio,y_ratio,ov5640_TouchAF_x,ov5640_TouchAF_y);

	ov5640_flash_auto_mode_flag_judge();
	rc = msm_camera_flash_led_enable();
	pr_err("yanwei %s: entry pre_flash\n", __func__);
	if(ov5640_TouchAF_x >= 0 && ov5640_TouchAF_y >= 0) {

	//set to idle
	rc = ov5640_i2c_write(ov5640_client->addr, 0x3023, 0x0001, BYTE_LEN);
	if (rc < 0)
	{
	pr_err("%s: failed, rc=%d!\n", __func__, rc);
	goto done;
	}
	rc = ov5640_i2c_write(ov5640_client->addr, 0x3022, 0x0008, BYTE_LEN);
	if (rc < 0)
	{
	pr_err("%s: failed, rc=%d!\n",__func__, rc);        
	goto done;
	}
	mdelay(15);
		
	rc = ov5640_i2c_write(ov5640_client->addr, 0x3024, (int)ov5640_TouchAF_x, BYTE_LEN);
	pr_err("yanwei ov5640_TouchAF_x=%d,ov5640_TouchAF_y=%d\n", ov5640_TouchAF_x,ov5640_TouchAF_y);			
	if (rc < 0)
	{
	pr_err("%s: failed, rc=%d!\n", __func__, rc);
	goto done;
	}
	rc = ov5640_i2c_write(ov5640_client->addr, 0x3025, (int)ov5640_TouchAF_y, BYTE_LEN);
	if (rc < 0)
	{
	pr_err("%s: failed, rc=%d!\n",__func__, rc);        
	goto done;
	}

	rc = ov5640_i2c_write(ov5640_client->addr, 0x3023, 0x0001, BYTE_LEN);
	if (rc < 0)
	{
	pr_err("%s: failed, rc=%d!\n", __func__, rc);
	goto done;
	}
	rc = ov5640_i2c_write(ov5640_client->addr, 0x3022, 0x0081, BYTE_LEN);
	if (rc < 0)
	{
	pr_err("%s: failed, rc=%d!\n",__func__, rc);        
	goto done;
	}
	for (i = 0; (i < 200) && (0x0000 != af_ack); ++i)
	{
	af_ack = 0x0002;
	rc = ov5640_i2c_read(ov5640_client->addr, 0x3023, &af_ack, BYTE_LEN);
	if (rc < 0)
	{
	pr_err("%s: rc < 0\n", __func__);
	goto done;
	} 
	
	mdelay(15);
		
	pr_err("Trig _1 Auto Focus  i  = %d ",i);
	}
	}

	//Use Trig Auto Focus command to start auto focus	

	rc = ov5640_i2c_write(ov5640_client->addr, 0x3023, 0x0001, BYTE_LEN);
	if (rc < 0)
	{
	pr_err("%s: failed, rc=%d!\n", __func__, rc);
	goto done;
	}

	rc = ov5640_i2c_write(ov5640_client->addr, 0x3022, 0x0003, BYTE_LEN);
	if (rc < 0)
	{
	pr_err("%s: failed, rc=%d!\n",__func__, rc);        
	goto done;
	}

	/*
	* af_status: = 0x00,  AF is done successfully
	*            != 0x00, AF is being done
	*
	* i: < 100, time delay for tuning AF
	*    >= 100, time out for tuning AF
	*/
	af_ack = 0x0002;//set af_ack value for compare its value from get 

	for (i = 0; (i < 200) && (0x0000 != af_ack); ++i)
	{
	af_ack = 0x0002;
	rc = ov5640_i2c_read(ov5640_client->addr, 0x3023, &af_ack, BYTE_LEN);

	if (rc < 0)
	{
	pr_err("%s: rc < 0\n", __func__);
	goto done;
	}         
	mdelay(15);
	//pr_err("Trig Auto Focus  i  = %d ",i);
	}

	rc_3028 = ov5640_i2c_read(ov5640_client->addr, 0x3028, &af_status_3028, BYTE_LEN);
	pr_err("af_status_3028=%d,\n", af_status_3028);
	//yanwei add ae code for touch af or snapshot af
      if((ov5640_TouchAF_x != 40)&&(ov5640_TouchAF_y != 30)){
	ov5640_i2c_write_b_sensor(0x3503, 0x00);
	pr_err("0x3503=0x00");
	  }
      else{
	ov5640_i2c_write_b_sensor(0x3503, 0x07);
	pr_err("0x3503=0x07");	 
       	}
        rc = ov5640_hw_ae_parameter_record();
        if (rc < 0)
        {
            return rc;
        }
	rc=msm_camera_flash_led_disable();
	pr_err("%s: yanwei msm_camera_flash_led_disable\n", __func__);

	if(0 != af_status_3028) {
	pr_err("Touch AF succeeded! %d,\n", af_status_3028);
	rc = 0;
	goto done;
	}else{
	pr_err("Touch AF failed: %s: AF af_status_3028=%x,\n", __func__, af_status_3028);
	rc = 0;
	//rc = -EFAULT;
	goto done;
	}

	done:
	ov5640_TouchAF_x = -1;
	ov5640_TouchAF_y = -1;
	return rc;
	}
 
   

static long ov5640_set_wb(int8_t wb_mode)
{
    long rc = 0;
    uint16_t tmp_reg = 0;

    pr_err("%s: entry: wb_mode=%d\n", __func__, wb_mode);

    switch(wb_mode)
    {
        case CAMERA_WB_MODE_AWB:
        {

            rc = ov5640_i2c_write(ov5640_client->addr, 0x3406, 0x0000, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
        }
        break;

        case CAMERA_WB_MODE_SUNLIGHT:
        {
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3406 ,0x0001, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3400 ,0x0006, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3401 ,0x001c, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3402 ,0x0004, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3403 ,0x0000, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3404 ,0x0005, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3405 ,0x0020, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
          
        }
        break;

        case CAMERA_WB_MODE_INCANDESCENT:
        {
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3406 ,0x0001, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3400 ,0x0004, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3401 ,0x0010, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3402 ,0x0004, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3403 ,0x0000, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3404 ,0x0008, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3405 ,0x0040, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
        }
        break;
        
        case CAMERA_WB_MODE_FLUORESCENT:
        {
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3406 ,0x0001, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3400 ,0x0005, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3401 ,0x0048, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3402 ,0x0004, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3403 ,0x0000, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3404 ,0x0007, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3405 ,0x00c0, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
        }
        break; 

        case CAMERA_WB_MODE_CLOUDY:
        {
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3406 ,0x0001, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3400 ,0x0006, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3401 ,0x0048, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3402 ,0x0004, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3403 ,0x0000, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3404 ,0x0004, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3405 ,0x00d3, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
        }
        break;

        case CAMERA_WB_MODE_NIGHT:
        {
            tmp_reg = 0;
            rc = ov5640_i2c_read(ov5640_client->addr, 0x3a00, &tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg &= 0x00FF;
            tmp_reg |= 0x0004;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3a00, tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3a02 ,0x000b, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }

            rc = ov5640_i2c_write(ov5640_client->addr, 0x3a03 ,0x0088, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3a14 ,0x000b, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }

            rc = ov5640_i2c_write(ov5640_client->addr, 0x3a15 ,0x0088, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            /*
               * Change preview FPS from 1500 to 375
               */
//            g_preview_frame_rate  = 375;
        }
        break;

        default:
        {
            pr_err("%s: parameter error!\n", __func__);
            return -EFAULT;
        }     
    }

    if(wb_mode != CAMERA_WB_MODE_NIGHT)
    {
    	     /*
            tmp_reg = 0;
            rc = ov5640_i2c_read(ov5640_client->addr, 0x3a00, &tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg &= 0x00FB;    
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3a00, tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            */
    }

    return rc;
}
    
/* ZTE_YGL_CAM_20111230
  * add the interface of touch AF for froyo
  */
static int32_t ov5640_set_aec_rio(aec_rio_cfg position)
{
    int32_t rc = 0;

    pr_err("ov5640_set_aec_rio: position.preview_width=%d,position.preview_height=%d\n", position.preview_width,position.preview_height);
    //pr_err("ov5640_set_aec_rio: position.x=%d,position.y=%d\n", position.x,position.y);

    x_ratio = (position.preview_width/80);//OV5640_AF_WINDOW_FULL_WIDTH 
    y_ratio = (position.preview_height/60);//OV5640_AF_WINDOW_FULL_HEIGHT
    if(x_ratio != 0 && y_ratio != 0){
    	ov5640_TouchAF_x = (position.x / x_ratio);
    	ov5640_TouchAF_y = (position.y / y_ratio);
    }
	
    pr_err("ov5640_set_aec_rio: x_ratio=%d,y_ratio=%d\n", x_ratio,y_ratio);
    return rc;
}  
#if 0
 /* ZTE_YGL_CAM_20101214
  * add the interface of touch AF for froyo
  */
static int32_t ov5640_set_aec_rio(aec_rio_cfg position)
{
    
    int32_t rc = 0;
    uint16_t af_status;
    uint32_t i;
    uint16_t window_full_width;
    uint16_t window_full_height;
    uint16_t x_ratio = 0;
    uint16_t y_ratio = 0;
  
	return rc;
    /*
     * window_full_width/window_full_height: 0~63
     */
    window_full_width = OV5640_AF_WINDOW_FULL_WIDTH;
    window_full_height = OV5640_AF_WINDOW_FULL_HEIGHT;
    x_ratio = position.preview_width/window_full_width;
    y_ratio = position.preview_height/window_full_height;
	
    if (x_ratio == 0 || y_ratio == 0)
    {
       return rc;
    }
	
    /* 
    * 1. set as idle mode
    */
    rc = ov5640_i2c_write(ov5640_client->addr, 0x3024, 0x08, BYTE_LEN);
    if (rc < 0)
    {
       return rc;
    }
	
    /* 
    * 2. read reg 0x3027, if =0, it means idle state
    */
    af_status = 0x0000;
    rc = ov5640_i2c_read(ov5640_client->addr, 0x3027, &af_status, BYTE_LEN);
    if (rc < 0)
    {
       return rc;
    }
    for (i = 0; (i < 100) && (0x0000 != af_status); ++i)
    {
        af_status = 0x0000;
        rc = ov5640_i2c_read(ov5640_client->addr, 0x3027, &af_status, BYTE_LEN);
        if (rc < 0)
        {
           return rc;
        }        

        mdelay(10);
    }
	
    /* 
    * 3. set windows position
    */
    rc = ov5640_i2c_write(ov5640_client->addr, 0x3025, 0x11, BYTE_LEN);
    if (rc < 0)
    {
       return rc;
    }
    rc = ov5640_i2c_write(ov5640_client->addr, 0x5084, (int32_t)(position.x / x_ratio), BYTE_LEN);
    if (rc < 0)
    {
       return rc;
    }
    rc = ov5640_i2c_write(ov5640_client->addr, 0x5085, (int32_t)(position.y / y_ratio), BYTE_LEN);
    if (rc < 0)
    {
       return rc;
    }
    rc = ov5640_i2c_write(ov5640_client->addr, 0x3024, 0x10, BYTE_LEN);
    if (rc < 0)
    {
       return rc;
    }
	
    /* 
    * 4. single foucs trigger
    */
    rc = ov5640_i2c_write(ov5640_client->addr, 0x3024, 0x03, BYTE_LEN);
    if (rc < 0)
    {
       return rc;
    }

    af_status = 0x0000;
    for (i = 0; (i < 1000) && (0x0002 != af_status); ++i)
    {
        af_status = 0x0000;
        rc = ov5640_i2c_read(ov5640_client->addr, 0x3027, &af_status, BYTE_LEN);
        if (rc < 0)
        {
           return rc;
        }        

        mdelay(15);
    }

    return rc;
}
#endif

/*
 * ISO Setting
 */
static int32_t ov5640_set_iso(int8_t iso_val)
{
    int32_t rc = 0;

    CDBG("%s: entry: iso_val=%d\n", __func__, iso_val);

    switch (iso_val)
    {
        case CAMERA_ISO_SET_AUTO:
        {
            //WT_CAM_20110428 iso value
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3A18 ,0x0000, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            #if 1 // 主观测试版本 2011-06-16 ken
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3A19 ,0x00f8, BYTE_LEN);	
            #else
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3A19 ,0x0040, BYTE_LEN);
            #endif
            if (rc < 0)
            {
               return rc;
            }
        }
        break;

        case CAMERA_ISO_SET_HJR:
        {
            //add code here     
        }
        break;

        case CAMERA_ISO_SET_100:
        {
    	    rc = ov5640_i2c_write(ov5640_client->addr, 0x3503 ,0x0002, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }	
    	    rc = ov5640_i2c_write(ov5640_client->addr, 0x350b ,0x0000, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }		  
	mdelay(100);	
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3A18 ,0x0000, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3A19 ,0x0020, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3503 ,0x0000, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
        }
        break;

        case CAMERA_ISO_SET_200:
        {
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3503 ,0x0002, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }	
    	    rc = ov5640_i2c_write(ov5640_client->addr, 0x350b ,0x0000, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }		  
	mdelay(100);
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3A18 ,0x0000, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3A19 ,0x0040, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3503 ,0x0000, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }		
        }
        break;

        case CAMERA_ISO_SET_400:
        {
             rc = ov5640_i2c_write(ov5640_client->addr, 0x3503 ,0x0002, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }	
    	    rc = ov5640_i2c_write(ov5640_client->addr, 0x350b ,0x0000, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }		  
	mdelay(100);
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3A18 ,0x0000, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3A19 ,0x0080, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3503 ,0x0000, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }	
        }
        break;

        case CAMERA_ISO_SET_800:
        {
    	    rc = ov5640_i2c_write(ov5640_client->addr, 0x3503 ,0x0002, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }	
    	    rc = ov5640_i2c_write(ov5640_client->addr, 0x350b ,0x0000, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }		  
		mdelay(100);	
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3A18 ,0x0000, BYTE_LEN);//ZTE_YGL_CAM_20110711,modifed for SNR
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3A19 ,0x00f8, BYTE_LEN);//ZTE_YGL_CAM_20110711,modifed for SNR
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3503 ,0x0000, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }	
        }
        break;

        default:
        {
            pr_err("%s: parameter error!\n", __func__);
            rc = -EFAULT;
        }     
    }

	return rc;
} 

/*
 * Antibanding Setting
 */
static int32_t  ov5640_set_antibanding(int8_t antibanding)
{
    int32_t rc = 0;
    int16_t tmp_reg = 0;

    pr_err("%s: entry: antibanding=%d\n", __func__, antibanding);

    switch (antibanding)
    {
        case CAMERA_ANTIBANDING_SET_OFF:
        {
            pr_err("%s: CAMERA_ANTIBANDING_SET_OFF NOT supported!\n", __func__);
        }
        break;

        case CAMERA_ANTIBANDING_SET_60HZ:
        {
            tmp_reg = 0;
            rc = ov5640_i2c_read(ov5640_client->addr, 0x3c01, &tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg |= 0x0080;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3C01, tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
                return rc;
            }

            
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3C00, 0x0000, BYTE_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3A0A, 0x0000, BYTE_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3A0B, 0x00f6, BYTE_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3A0D, 0x0004, BYTE_LEN);
            if (rc < 0)
            {
                return rc;
            }
        }            
        break;

        case CAMERA_ANTIBANDING_SET_50HZ:
        {
        	  
            tmp_reg = 0;
            rc = ov5640_i2c_read(ov5640_client->addr, 0x3c01, &tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg |= 0x0080;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3C01, tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3C00, 0x0004, BYTE_LEN);
            if (rc < 0)
            {
                return rc;
            }
             
	pr_err("50hz");
	}
	break;

	case CAMERA_ANTIBANDING_SET_AUTO:
	{
	tmp_reg = 0;
	rc = ov5640_i2c_read(ov5640_client->addr, 0x3c01, &tmp_reg, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}
	tmp_reg |= 0x0080;
	rc = ov5640_i2c_write(ov5640_client->addr, 0x3C01, tmp_reg, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}
	rc = ov5640_i2c_write(ov5640_client->addr, 0x3C00, 0x0004, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}     

	pr_err("50hz");

#if 0 //kenxu mask for 50hz default @20120525
	rc = ov5640_i2c_write(ov5640_client->addr, 0x3622, 0x0001, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}
	rc = ov5640_i2c_write(ov5640_client->addr, 0x3635, 0x001c, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}
	rc = ov5640_i2c_write(ov5640_client->addr, 0x3634, 0x0040, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}
	rc = ov5640_i2c_write(ov5640_client->addr, 0x3C00, 0x0000, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}
	rc = ov5640_i2c_write(ov5640_client->addr, 0x3C01, 0x0034, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}
	rc = ov5640_i2c_write(ov5640_client->addr, 0x3C04, 0x0028, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}
	rc = ov5640_i2c_write(ov5640_client->addr, 0x3C05, 0x0098, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}
	rc = ov5640_i2c_write(ov5640_client->addr, 0x3C06, 0x0000, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}
	rc = ov5640_i2c_write(ov5640_client->addr, 0x3C07, 0x0008, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}
	rc = ov5640_i2c_write(ov5640_client->addr, 0x3C08, 0x0000, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}
	rc = ov5640_i2c_write(ov5640_client->addr, 0x3C09, 0x001c, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}
	rc = ov5640_i2c_write(ov5640_client->addr, 0x300c, 0x0022, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}
	rc = ov5640_i2c_write(ov5640_client->addr, 0x3C0A, 0x009c, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}
	rc = ov5640_i2c_write(ov5640_client->addr, 0x3C0B, 0x0040, BYTE_LEN);
	if (rc < 0)
	{
	return rc;
	}
#endif
	}
	break;

        default:
        {
            pr_err("%s: parameter error!\n", __func__);
            rc = -EFAULT;
        }     
    }

	return rc;
} 

/*
 * Sharpness Setting
 */
static int32_t ov5640_set_sharpness(int8_t sharpness)
{
    int32_t rc = 0;

    CDBG("%s: entry: sharpness=%d\n", __func__, sharpness);
	if (zte_effect == CAMERA_EFFECT_OFF) 
	{       
    switch (sharpness)
    {
        case CAMERA_SHARPNESS_0:
        {
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5308 ,0x0065, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5302 ,0x0018, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5303 ,0x0000, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
        }
        break;

        case CAMERA_SHARPNESS_1:
        {
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5308 ,0x0065, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5302 ,0x0010, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5303 ,0x0000, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
        }
        break;

        case CAMERA_SHARPNESS_2:
        {
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5308 ,0x0025, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5302 ,0x0010, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5303 ,0x0008, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
        }
        break;
        
        case CAMERA_SHARPNESS_3:
        {
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5308 ,0x0065, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5302 ,0x0008, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5303 ,0x0000, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
        }
        break; 

        case CAMERA_SHARPNESS_4:
        {
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5308 ,0x0065, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5302 ,0x0002, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5303 ,0x0000, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
        }
        break;        
        
        default:
        {
            pr_err("%s: parameter error!\n", __func__);
            rc = -EFAULT;
        }     
    }
		}
	return rc;
} 

/* ZTE_ZT_CAM_20101026_04
 * add the interface of exposure compensation for foryo
 */
static long ov5640_set_exposure_compensation(int8_t exposure)
{
    long rc = 0;

    pr_err("%s: entry: exposure=%d\n", __func__, exposure);

    switch(exposure)
    {
        case CAMERA_EXPOSURE_0:
        {
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3a0f, 0x0018, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3a10, 0x0010, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3a11, 0x0050, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3a1b, 0x0018, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3a1e, 0x0010, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3a1f, 0x0004, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }        
        }
        break;
        
        case CAMERA_EXPOSURE_1:
        {	
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3a0f, 0x0020, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3a10, 0x0018, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3a11, 0x0050, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3a1b, 0x0020, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3a1e, 0x0018, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3a1f, 0x0008, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }       
        }
        break;

        case CAMERA_EXPOSURE_2:
        {
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3a0f, 0x0038, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3a10, 0x0030, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3a11, 0x0060, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3a1b, 0x0038, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3a1e, 0x0030, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3a1f, 0x0014, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
        }
        break;

        case CAMERA_EXPOSURE_3:
        {
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3a0f, 0x0048, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3a10, 0x0040, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3a11, 0x0070, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3a1b, 0x0048, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3a1e, 0x0040, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3a1f, 0x0020, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }        
        }
        break;

        case CAMERA_EXPOSURE_4:
        {
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3a0f, 0x0058, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3a10, 0x0050, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3a11, 0x0080, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3a1b, 0x0058, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3a1e, 0x0050, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3a1f, 0x0020, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }       
        }
        break;
        
        default:
        {
            pr_err("%s: parameter error!\n", __func__);
            return -EFAULT;
        }        
    }

    return rc;
}

static long ov5640_reg_init(void)
{
    long rc;

    pr_err("%s: entry\n", __func__);

    /* PLL Setup */
    rc = ov5640_i2c_write_table(ov5640_regs.plltbl, ov5640_regs.plltbl_size);
    if (rc < 0)
    {
        return rc;
    }

    /* Configure sensor for Preview mode and Snapshot mode */
    rc = ov5640_i2c_write_table(ov5640_regs.prev_snap_reg_settings, ov5640_regs.prev_snap_reg_settings_size);
    if (rc < 0)
    {
        return rc;
    }

    /* Configure for Noise Reduction */
    rc = ov5640_i2c_write_table(ov5640_regs.noise_reduction_reg_settings, ov5640_regs.noise_reduction_reg_settings_size);
    if (rc < 0)
    {
        return rc;
    }

    /* Configure for Refresh Sequencer */
    rc = ov5640_i2c_write_table(ov5640_regs.stbl, ov5640_regs.stbl_size);
    if (rc < 0)
    {
        return rc;
    }

    /* Configuration for AF */
    rc = ov5640_i2c_write_table(ov5640_regs.autofocus_reg_settings, ov5640_regs.autofocus_reg_settings_size);
    if (rc < 0)
    {
        return rc;
    }

    /* Configuration for Lens Roll */
    rc = ov5640_set_lens_roll_off();
    if (rc < 0)
    {
        return rc;
    }

    return 0;
}


#ifdef ZTE_FIX_WB_ENABLE
static int zte_flash_on_fix_wb_setreg()
{
    //3406 01 disable wb-auto
	//3406 00 enable wb-auto
	int rc = 0;
	pr_err("%s\r\n",__func__);
	rc = ov5640_i2c_write(ov5640_client->addr, 0x3406, 0x01, BYTE_LEN); 
	if (rc < 0)
	{
		return rc;
	} 
	zte_disable_wb_auto_flag=1;
	
	rc = ov5640_i2c_write(ov5640_client->addr, 0x3400, 0x07, BYTE_LEN); //0x0a
	if (rc < 0)
	{
		return rc;
	} 
   rc = ov5640_i2c_write(ov5640_client->addr, 0x3401, 0x5C, BYTE_LEN);
	if (rc < 0)
	{
		return rc;
	} 
   rc = ov5640_i2c_write(ov5640_client->addr, 0x3402, 0x04, BYTE_LEN); //0x05
	if (rc < 0)
	{
		return rc;
	} 
   rc = ov5640_i2c_write(ov5640_client->addr, 0x3403, 0x50, BYTE_LEN);
	if (rc < 0)
	{
		return rc;
	} 
   rc = ov5640_i2c_write(ov5640_client->addr, 0x3404, 0x05, BYTE_LEN);
	if (rc < 0)
	{
		return rc;
	} 
   rc = ov5640_i2c_write(ov5640_client->addr, 0x3405, 0xC0, BYTE_LEN);
	if (rc < 0)
	{
		return rc;
	} 	
    
	return rc;
}

/*
*wb_auto is on ,flash is on ,bright <0x30 ,then will fix wb 
*wb_auto is on ,flash is auto  ,bright <0x10 ,then will fix wb 
*/
static int  zte_flash_on_fix_wb(void)
{
	int rc = 0;
	  
	uint16_t bright_val=0;

	   
    rc = ov5640_i2c_read(ov5640_client->addr, 0x56a1, &bright_val, BYTE_LEN);
    if (rc < 0)
    {
        return rc;
    }

	
	pr_err("0x56a1 g_bright_val =0x%x  flash_led_enable=%d zte_wb_mode=%d",
	bright_val,flash_led_enable,zte_wb_mode);

    if((zte_wb_mode == 1 )&&((flash_led_enable == 1)&&(bright_val < 0x30)))
	{
		zte_flash_on_fix_wb_setreg();
		pr_err("flash on ");
	}
	
	if((zte_wb_mode == 1 )&&((flash_led_enable == 2)&&(bright_val < 0x10)))
	{
		zte_flash_on_fix_wb_setreg();
		pr_err("flash auto ");
	}
	
	return 0;  


}
#endif

static long ov5640_hw_ae_transfer(void)
{
    int rc = 0;
   //calculate capture exp & gain
   
	 int preview_shutter, preview_gain16;
	 int capture_gain16;
	 int capture_sysclk, capture_HTS, capture_VTS;
	 int light_frequency, capture_bandingfilter, capture_max_band;
	 long int capture_gain16_shutter,capture_shutter;
	 unsigned short average;
	 preview_shutter = ov5640_preview_exposure;	 
	 // read preview gain
        preview_gain16 = ov5640_gain;
	 printk("%s: preview_shutter=0x%x, preview_gain16=0x%x\n", __func__, preview_shutter, preview_gain16);
	/*
	  * add Auto mode for flash LED, ZTE_CAM_LJ_20120616
	  */	 
	//ov5640_flash_auto_mode_flag_judge();
	// read capture VTS
	 capture_VTS = ov5640_get_VTS();
	 capture_HTS = ov5640_get_HTS();
	 capture_sysclk = ov5640_get_sysclk();
	 printk("%s: capture_VTS=0x%x, capture_HTS=0x%x, capture_sysclk=0x%x\n", __func__, capture_VTS, capture_HTS, capture_sysclk);

	 // get average	  
	 ov5640_i2c_read_byte(0x56a1,&average);	 

	 // calculate capture banding filter
	 light_frequency = ov5640_get_light_frequency();
	 if (light_frequency == 60) {
		 // 60Hz
		 capture_bandingfilter = capture_sysclk * 100 / capture_HTS * 100 / 120;
	 }
	 else {
		 // 50Hz
		 capture_bandingfilter = capture_sysclk * 100 / capture_HTS;
	 }
	 capture_max_band = (int)((capture_VTS - 4)/capture_bandingfilter);
	 printk("%s: light_frequency=0x%x, capture_bandingfilter=0x%x, capture_max_band=0x%x\n", __func__, light_frequency, capture_bandingfilter, capture_max_band);
      # if 0
	 // calculate capture shutter/gain16
	 if (average > AE_low && average < AE_high) {
		 // in stable range
		 capture_shutter = preview_shutter * capture_sysclk/preview_sysclk * preview_HTS/capture_HTS * AE_Target / average;
		//capture_gain16_shutter = preview_gain16 * preview_shutter /preview_sysclk * capture_sysclk /capture_HTS * preview_HTS * AE_Target / average;
	 }
	 else {
		 capture_shutter = preview_shutter * capture_sysclk/preview_sysclk * preview_HTS/capture_HTS;
		// capture_gain16_shutter = preview_gain16 * preview_shutter /preview_sysclk * capture_sysclk/capture_HTS* preview_HTS;
	 }
      #endif
	  	 capture_shutter = preview_shutter * capture_sysclk/preview_sysclk * preview_HTS/capture_HTS;
	        capture_gain16_shutter = preview_gain16 * capture_shutter;
	  
	 printk("%s:  preview_gain16=%d, preview_shutter=%d   capture_gain16_shutter=%ld\n", __func__, preview_gain16, preview_shutter,capture_gain16_shutter);
	 printk("%s: capture_sysclk=%d, preview_sysclk=%d, preview_HTS=%d\n", __func__, capture_sysclk, preview_sysclk, preview_HTS);

	 // gain to shutter
	 if(capture_gain16_shutter < (capture_bandingfilter * 16)) {
		 // shutter < 1/100
		 capture_shutter = capture_gain16_shutter/16;
		 if(capture_shutter <1)
			 capture_shutter = 1;

		 capture_gain16 = capture_gain16_shutter/capture_shutter;
		 if(capture_gain16 < 16)
			 capture_gain16 = 16;
	 printk("shutter < 1/100\n");
		 
	 }
	 else {
		 if(capture_gain16_shutter > (capture_bandingfilter*capture_max_band*16)) {
			 // exposure reach max
			 capture_shutter = capture_bandingfilter*capture_max_band;
			 capture_gain16 = capture_gain16_shutter / capture_shutter;
			 	 printk(" exposure reach max\n");
		 }
		 else {
			 // 1/100 < capture_shutter =< max, capture_shutter = n/100
			 capture_shutter = ((int)(capture_gain16_shutter/16/capture_bandingfilter)) * capture_bandingfilter;
			 capture_gain16 = capture_gain16_shutter / capture_shutter;
			  printk("1/100 < capture_shutter =< max, capture_shutter = n/100\n");
		 }
	 }

#if 0 // 主观测试版本 2011-12-21 ken
        //kenxu add for reduce noise under dark condition
        if(iCapture_Gain > 32) //gain > 2x, gain / 2, exposure * 2;
        {
          iCapture_Gain = iCapture_Gain / 2;
          ulCapture_Exposure = ulCapture_Exposure * 2;
        }
#endif
    printk("WB_T=%d\n", WB_T);
    if(WB_T == 0X02)
  	{
  	  rc = ov5640_i2c_write_table(ov5640_regs.lens_shading_D65_tbl,ov5640_regs.lens_shading_D65_tbl_size);
  	}
    else if (WB_T == 0X06)
  	{
  	 rc = ov5640_i2c_write_table(ov5640_regs.lens_shading_A_tbl,ov5640_regs.lens_shading_A_tbl_size);
  	}

         printk("%s: capture_gain16=0x%x\n", __func__, capture_gain16);
   // if (flash_led_enable == 0)
    	{
	    if(capture_gain16 > 48) //reach to max gain 16x, change blc to pass SNR test under low light
         {  
         printk("%s: YAVG=0x%x\n", __func__, YAVG);

      	    if(YAVG <=15) //10lux
      	 {
      	  capture_shutter = capture_shutter * 2;
          rc = ov5640_i2c_write_b_sensor(0x4009, 0x50);
          if (rc < 0)
          {
            return rc;
          }
      	}
      	else if(YAVG <=23)//50lux
        {
          capture_shutter = capture_shutter * 4/3;
          rc = ov5640_i2c_write_b_sensor(0x4009, 0x30);
          if (rc < 0)
          {
            return rc;
          }
        }            
     }
      	    printk("capture_shutter=%ld\n", capture_shutter);
    	}

	 // write capture gain
	 ov5640_set_gain16(capture_gain16);

	// write capture shutter
	if (capture_shutter > (capture_VTS - 4)) {
	capture_VTS = capture_shutter + 4;
	ov5640_set_VTS(capture_VTS);
	}
	ov5640_set_shutter(capture_shutter);  
	mdelay(150);//150
	return rc;
	}

static long ov5640_set_effect(int32_t mode, int32_t effect)
{
    uint16_t __attribute__((unused)) reg_addr;
    uint16_t __attribute__((unused)) reg_val;
    uint16_t tmp_reg = 0;
    long rc = 0;

    pr_err("%s: entry: mode=%d, effect=%d\n", __func__, mode, effect);

    switch (mode)
    {
        case SENSOR_PREVIEW_MODE:
        {
            /* Context A Special Effects */
            /* add code here
                 e.g.
                 reg_addr = 0xXXXX;
               */
        }
        break;

        case SENSOR_SNAPSHOT_MODE:
        {
            /* Context B Special Effects */
            /* add code here
                 e.g.
                 reg_addr = 0xXXXX;
               */
        }
        break;

        default:
        {
            /* add code here
                 e.g.
                 reg_addr = 0xXXXX;
               */
        }
        break;
    }

    switch (effect)
    {
        case CAMERA_EFFECT_OFF:
        {

            tmp_reg = 0;
            rc = ov5640_i2c_read(ov5640_client->addr, 0x5001, &tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }

            /*
             * ZTE_LJ_CAM_20101026
             * fix bug of no preview image after changing effect mode repeatedly
             */
            tmp_reg &= 0x00FF;
            tmp_reg |= 0x0080;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5001, tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5583, 0x0040, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
	rc = ov5640_i2c_write(ov5640_client->addr, 0x5584, 0x001e, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg = 0;
            rc = ov5640_i2c_read(ov5640_client->addr, 0x5580, &tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg &= 0x00FF;
            tmp_reg &= 0x0006;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5580, tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }   
        }
        break;

        case CAMERA_EFFECT_MONO:
        {
            tmp_reg = 0;
            rc = ov5640_i2c_read(ov5640_client->addr, 0x5001, &tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg &= 0x00FF;
            tmp_reg |= 0x0080;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5001, tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5583, 0x0080, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5584, 0x0080, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg = 0;
            rc = ov5640_i2c_read(ov5640_client->addr, 0x5580, &tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg &= 0x0087;
            tmp_reg |= 0x0018;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5580, tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }	
        }
        break;
        
        case CAMERA_EFFECT_NEGATIVE:
        {
            tmp_reg = 0;
            rc = ov5640_i2c_read(ov5640_client->addr, 0x5001, &tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg &= 0x00FF;
            tmp_reg |= 0x0080;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5001, tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg = 0;
            rc = ov5640_i2c_read(ov5640_client->addr, 0x5580, &tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg &= 0x0087;
            tmp_reg |= 0x0040;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5580, tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }	
        }
        break;          
        
        case CAMERA_EFFECT_SEPIA:
        {
            tmp_reg = 0;
            rc = ov5640_i2c_read(ov5640_client->addr, 0x5001, &tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg &= 0x00FF;
            tmp_reg |= 0x0080;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5001, tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5583, 0x0040, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5584, 0x00a0, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg = 0;
            rc = ov5640_i2c_read(ov5640_client->addr, 0x5580, &tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg &= 0x0087;
            tmp_reg |= 0x0018;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5580, tmp_reg, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }	
        }
        break;
        
        //case CAMERA_EFFECT_BULISH:    
       // case CAMERA_EFFECT_GREENISH:
       // case CAMERA_EFFECT_REDDISH:
        {
            pr_err("%s: not supported!\n", __func__);
            rc = 0;
        }
        break;      
        
        default:
        {
            pr_err("%s: parameter error!\n", __func__);
            return -EFAULT;
        }
    }

    return rc;
}

static long ov5640_set_sensor_mode(int32_t mode)
{
    long rc = 0;
   // uint16_t af_ack;
	uint16_t temp;
    //int i;
    /*
      * In order to avoid reentry of SENSOR_PREVIEW_MODE by
      * Qualcomm Camera HAL, e.g. vfe_set_dimension/vfe_preview_config,
      * 
      * enter into SENSOR_PREVIEW_MODE only when ov5640_previewmode_entry_cnt=0
      */
    static uint32_t ov5640_previewmode_entry_cnt = 0;


    switch (mode)
    {
        case SENSOR_PREVIEW_MODE:
		{
		ov5640_i2c_write_table(ov5640_regs.snapshot2preview_tbl, ov5640_regs.snapshot2preview_size);
#ifdef ZTE_FIX_WB_ENABLE
		if(zte_disable_wb_auto_flag)
		{
		rc = ov5640_i2c_write(ov5640_client->addr, 0x3406, 0x0000, BYTE_LEN); //denable wb_auto
		zte_disable_wb_auto_flag=0;
		pr_err("flash fix awb zte_enable_wb_auto_flag");
		}
#endif



#if 0
	return 0;
	/*
	* Enter Into SENSOR_PREVIEW_MODE
	* Only When ov5640_previewmode_entry_cnt=0
	*/
	if (0 != ov5640_previewmode_entry_cnt)
	{
	CDBG("%s: reentry of SENSOR_PREVIEW_MODE: entry count=%d\n", __func__,
	ov5640_previewmode_entry_cnt);
	break;
	}
#endif

	rc = ov5640_i2c_write_table(ov5640_regs.snapshot2preview_tbl, ov5640_regs.snapshot2preview_size);
	if (rc < 0)
	{
	return rc;
	}  

	ov5640_i2c_read_byte(0x5588, &temp);
	temp = temp & 0xbf;
	ov5640_i2c_write_b_sensor(0x5588, temp); //Auto UV

	ov5640_set_bandingfilter();
	//ov5640_i2c_write_b_sensor(0x503d, 0x80); //color bar
	//ov5640_i2c_write_b_sensor(0x4741, 0x00); //color bar


//Reset MCU
	rc = ov5640_i2c_write(ov5640_client->addr, 0x3000, 0x0020, BYTE_LEN);
	if (rc < 0)
	{
	pr_err("%s: failed, rc=%ld!\n", __func__, rc);
	return rc;
	}
	mdelay(10);
	rc = ov5640_i2c_write(ov5640_client->addr, 0x3000, 0x0000, BYTE_LEN);
	if (rc < 0)
	{
	pr_err("%s: failed, rc=%ld!\n", __func__, rc);
	return rc;
	}     

		       //modify for preview abnormal in mono mode after snapshot  by lijing ZTE_CAM_LJ_20120627
	ov5640_set_effect(0,zte_effect);
	ov5640_set_contrast(zte_contrast);
	ov5640_set_sharpness(zte_sharpness);
	ov5640_set_saturation(zte_sat);
	ov5640_set_brightness(zte_brightness);	
#ifdef ZTE_FIX_WB_ENABLE
		if(zte_disable_wb_auto_flag)
		{
		rc = ov5640_i2c_write(ov5640_client->addr, 0x3406, 0x0000, BYTE_LEN); //denable wb_auto
		zte_disable_wb_auto_flag=0;
		pr_err("flash fix awb zte_enable_wb_auto_flag");
		}
#endif
	/* Exit from AF mode */
	rc = ov5640_i2c_write(ov5640_client->addr, 0x3023, 0x0001, BYTE_LEN);
	if (rc < 0)
	{
	pr_err("%s: failed, rc=%ld!\n", __func__, rc);
	return rc;
	}
	rc = ov5640_i2c_write(ov5640_client->addr, 0x3022, 0x0008, BYTE_LEN);
	if (rc < 0)
	{
	pr_err("%s: failed, rc=%ld!\n", __func__, rc);
	return rc;
	}   
            /*
               * Enter Into SENSOR_PREVIEW_MODE
               * Only When ov5640_previewmode_entry_cnt=0
               */
            ov5640_previewmode_entry_cnt++;
        }
        break;

        case SENSOR_SNAPSHOT_MODE:
        {
            /*
               * Enter Into SENSOR_PREVIEW_MODE
               * Only When ov5640_previewmode_entry_cnt=0
               */
            ov5640_previewmode_entry_cnt = 0;
            pr_err("yanwei SENSOR_SNAPSHOT_MODE");
#if 1 //enable exposure algorithm
		rc=msm_camera_flash_led_disable();	
	     pr_err("yanwei %s: msm_camera_flash_led_disable\n", __func__);	
    	     rc = ov5640_i2c_write_table(ov5640_regs.preview2snapshot_tbl, ov5640_regs.preview2snapshot_size);

            rc = ov5640_hw_ae_transfer();
            if (rc < 0)
            {
                return rc;
            }            
#else //disable exposure algorithm
            rc = ov5640_i2c_write_table(ov5640_regs.preview2snapshot_tbl, ov5640_regs.preview2snapshot_size);
            if (rc < 0)
            {
                return rc;
            }        
            CDBG("5MP snapshot!!!\n");
            mdelay(3000);         
#endif            
        }
        break;

        default:
        {
            pr_err("%s: parameter error!\n", __func__);
            return -EFAULT;
        }
    }

    return 0;
}

/*
 * Power-up Process
 */
static long ov5640_power_up(void)
{
    CDBG("%s: not supported!\n", __func__);
    return 0;
}

/*
 * Power-down Process
 */
static long ov5640_power_down(void)
{
    CDBG("%s: not supported!\n", __func__);
    return 0;
}

/*
 * Set lowest-power mode (SHUTDOWN mode)
 *
 * OV5640_GPIO_SHUTDOWN_CTL: 0, to quit lowest-power mode, or
 *                            1, to enter lowest-power mode
 */
#if defined(CONFIG_MACH_RAISE)
static int __attribute__((unused))ov5640_power_shutdown(uint32_t on)
#elif defined(CONFIG_MACH_MOONCAKE)
static int __attribute__((unused)) ov5640_power_shutdown(uint32_t on)
#elif defined(CONFIG_MACH_JOE)
static int __attribute__((unused)) ov5640_power_shutdown(uint32_t on)
#else
static int __attribute__((unused)) ov5640_power_shutdown(uint32_t on)
#endif /* defined(CONFIG_MACH_RAISE) */
{
    int rc;

    CDBG("%s: entry\n", __func__);

    rc = gpio_request(OV5640_GPIO_SHUTDOWN_CTL, "ov5640");
    if (0 == rc)
    {
        /* ignore "rc" */
        rc = gpio_direction_output(OV5640_GPIO_SHUTDOWN_CTL, on);

        /* time delay for shutting sensor down */
	mdelay(10);
    }

    gpio_free(OV5640_GPIO_SHUTDOWN_CTL);

    return rc;
}

#if !defined(CONFIG_SENSOR_ADAPTER)
static int ov5640_sensor_init_probe(const struct msm_camera_sensor_info *data)
{
    uint16_t model_id = 0;    
    int rc = 0;

    pr_err("%s: entry\n", __func__);


    rc = ov5640_i2c_read(ov5640_client->addr, REG_OV5640_MODEL_ID, &model_id, WORD_LEN);
    if (rc < 0)
    {
        goto init_probe_fail;
    }

    pr_err("%s: model_id = 0x%x\n", __func__, model_id);

    /*
      * set sensor id for EM (Engineering Mode)
      */
#ifdef CONFIG_SENSOR_INFO
    msm_sensorinfo_set_sensor_id(model_id);
#else
    // do nothing here
#endif  

    /* Check if it matches it with the value in Datasheet */
    if (model_id != OV5640_MODEL_ID)
    {
        rc = -EFAULT;
        goto init_probe_fail;
    }

    rc = ov5640_reg_init();
    if (rc < 0)
    {
        goto init_probe_fail;
    }

    return rc;

init_probe_fail:
    pr_err("%s: rc = %d, failed!\n", __func__, rc);
    return rc;
}
#else
static int ov5640_sensor_i2c_probe_on(void)
{
    int rc;
    struct i2c_board_info info;
    struct i2c_adapter *adapter;
    struct i2c_client *client;

    rc = ov5640_i2c_add_driver();
    if (rc < 0)
    {
        pr_err("%s: add i2c driver failed!\n", __func__);
        return rc;
    }

    memset(&info, 0, sizeof(struct i2c_board_info));
    info.addr = OV5640_SLAVE_WR_ADDR >> 1;
    strlcpy(info.type, OV5640_I2C_BOARD_NAME, I2C_NAME_SIZE);

    adapter = i2c_get_adapter(OV5640_I2C_BUS_ID);
    if (!adapter)
    {
        pr_err("%s: get i2c adapter failed!\n", __func__);
        goto i2c_probe_failed;
    }

    client = i2c_new_device(adapter, &info);
    i2c_put_adapter(adapter);
    if (!client)
    {
        pr_err("%s: add i2c device failed!\n", __func__);
        goto i2c_probe_failed;
    }

    ov5640_client = client;

    return 0;

i2c_probe_failed:
    ov5640_i2c_del_driver();
    return -ENODEV;
}

static void ov5640_sensor_i2c_probe_off(void)
{
    i2c_unregister_device(ov5640_client);
    ov5640_i2c_del_driver();
}

static int ov5640_sensor_dev_probe(const struct msm_camera_sensor_info *pinfo)
{
    uint16_t model_id;
    uint32_t switch_on;
    int rc;

    pr_err("%s: entry\n", __func__);

    /*
      * Deassert Sensor Reset
      * Ignore "rc"
      */

    rc = gpio_request(pinfo->sensor_reset, "ov5640");
    rc = gpio_direction_output(pinfo->sensor_reset, 0);
    gpio_free(pinfo->sensor_reset);

    /* time delay for deasserting RESET */
	mdelay(10);

    /*
      * Enter Into Hard Standby
      */
    switch_on = 1;
    rc = ov5640_hard_standby(pinfo, switch_on);
    if (rc < 0)
    {
        goto dev_probe_fail;
    }

    /*
      * Power VREG on
      */
    rc = msm_camera_power_backend(MSM_CAMERA_PWRUP_MODE);
    if (rc < 0)
    {
        goto dev_probe_fail;
    }

    /*
     * Camera clock switch for both frontend and backend sensors, i.e., MT9V113 and OV5640
     *
     * For MT9V113: 0.3Mp, 1/11-Inch System-On-A-Chip (SOC) CMOS Digital Image Sensor
     * For OV5640: 5.0Mp, 1/4-Inch System-On-A-Chip (SOC) CMOS Digital Image Sensor
     *
     * switch_val: 0, to switch clock to frontend sensor, i.e., MT9V113, or
     *             1, to switch clock to backend sensor, i.e., OV5640
     */
#if defined(CONFIG_MACH_RAISE)
    rc = msm_camera_clk_switch(pinfo, OV5640_GPIO_SWITCH_CTL, OV5640_GPIO_SWITCH_VAL);
    if (rc < 0)
    {
        goto dev_probe_fail;
    }
#elif defined(CONFIG_MACH_MOONCAKE)
    /* Do nothing */
#elif defined(CONFIG_MACH_R750) || defined(CONFIG_MACH_JOE)
    rc = msm_camera_clk_switch(pinfo, OV5640_GPIO_SWITCH_CTL, OV5640_GPIO_SWITCH_VAL);
    if (rc < 0)
    {
        goto dev_probe_fail;
    }
#else
    /* Do nothing */
#endif /* defined(CONFIG_MACH_RAISE) */

    /* Input MCLK */
    msm_camio_clk_rate_set(OV5640_CAMIO_MCLK);

    /* time delay for enabling MCLK */
	mdelay(10);

    /*
      * Exit From Hard Standby
      */
    switch_on = 0;
    rc = ov5640_hard_standby(pinfo, switch_on);
    if (rc < 0)
    {
        goto dev_probe_fail;
    }

    /*
      * Assert Sensor Reset
      * Ignore "rc"
      */
    rc = gpio_request(pinfo->sensor_reset, "ov5640");
    rc = gpio_direction_output(pinfo->sensor_reset, 1);
    gpio_free(pinfo->sensor_reset);

    /* time delay for asserting RESET */
	mdelay(10);

    model_id = 0;
    rc = ov5640_i2c_read(ov5640_client->addr, REG_OV5640_MODEL_ID, &model_id, BYTE_LEN);
    if (rc < 0)
    {
        goto dev_probe_fail;
    }

	pr_err("zt debug: %s : model_id %x\n", __func__, model_id);//zhangtao
    CDBG("%s: model_id = 0x%x\n", __func__, model_id);

    /*
      * set sensor id for EM (Engineering Mode)
      */
#ifdef CONFIG_SENSOR_INFO
    msm_sensorinfo_set_sensor_id(model_id);
#else
    // do nothing here
#endif

    /* Check if it matches it with the value in Datasheet */
    if (model_id != OV5640_MODEL_ID)
    {
        rc = -EFAULT;
        goto dev_probe_fail;
    }

    return 0;

dev_probe_fail:
    pr_err("%s: rc = %d, failed!\n", __func__, rc);
    return rc;
}
#endif

static int ov5640_sensor_probe_init(const struct msm_camera_sensor_info *data)
{
#if !defined(CONFIG_SENSOR_ADAPTER)
    uint32_t switch_on; 
#endif
    int rc = 0;

    CDBG("%s: entry\n", __func__);

printk("zt debug: %s\n", __func__);//zhangtao
    if (!data || strcmp(data->sensor_name, "ov5640"))
    {
        pr_err("%s: invalid parameters!\n", __func__);
        rc = -ENODEV;
        goto probe_init_fail;
    }

    ov5640_ctrl = kzalloc(sizeof(struct ov5640_ctrl_t), GFP_KERNEL);
    if (!ov5640_ctrl)
    {
        pr_err("%s: kzalloc failed!\n", __func__);
        rc = -ENOMEM;
        goto probe_init_fail;
    }

    ov5640_ctrl->sensordata = data;

#if !defined(CONFIG_SENSOR_ADAPTER)
    /*
      * Deassert Sensor Reset
      * Ignore "rc"
      */
    rc = gpio_request(ov5640_ctrl->sensordata->sensor_reset, "ov5640");
    rc = gpio_direction_output(ov5640_ctrl->sensordata->sensor_reset, 0);
    gpio_free(ov5640_ctrl->sensordata->sensor_reset);

    /* time delay for deasserting RESET */
	mdelay(10);

    /* Enter Into Hard Standby */
    switch_on = 1;
    rc = ov5640_hard_standby(ov5640_ctrl->sensordata, switch_on);
    if (rc < 0)
    {
        pr_err("set standby failed!\n");
        goto probe_init_fail;
    }

    /*
      * Power VREG on
      */
    rc = msm_camera_power_backend(MSM_CAMERA_PWRUP_MODE);
    if (rc < 0)
    {
        pr_err("%s: camera_power_backend failed!\n", __func__);
        goto probe_init_fail;
    }

    /*
     * Camera clock switch for both frontend and backend sensors, i.e., MT9V113 and OV5640
     *
     * For MT9V113: 0.3Mp, 1/11-Inch System-On-A-Chip (SOC) CMOS Digital Image Sensor
     * For OV5640: 5.0Mp, 1/4-Inch System-On-A-Chip (SOC) CMOS Digital Image Sensor
     *
     * switch_val: 0, to switch clock to frontend sensor, i.e., MT9V113, or
     *             1, to switch clock to backend sensor, i.e., OV5640
     */
#if defined(CONFIG_MACH_RAISE)
    rc = msm_camera_clk_switch(ov5640_ctrl->sensordata, OV5640_GPIO_SWITCH_CTL, OV5640_GPIO_SWITCH_VAL);
    if (rc < 0)
    {
        pr_err("%s: camera_clk_switch failed!\n", __func__);
        goto probe_init_fail;
    }
#elif defined(CONFIG_MACH_MOONCAKE)
    /* Do nothing */
#elif defined(CONFIG_MACH_JOE)
    /* 
    * add for mclk switch for msm7627_joe
    */
    rc = msm_camera_clk_switch(ov5640_ctrl->sensordata, OV5640_GPIO_SWITCH_CTL, OV5640_GPIO_SWITCH_VAL);
    if (rc < 0)
    {
        pr_err("%s: camera_clk_switch failed!\n", __func__);
        goto probe_init_fail;
    }
#else
    /* Do nothing */
#endif /* defined(CONFIG_MACH_RAISE) */

/*
 * for v9plus,gpio 181 is configured as mclk switch
 * 0: back camera 1: front camera
 */
#if defined (CONFIG_MACH_V9PLUS)
    rc = msm_camera_clk_switch(ov5640_ctrl->sensordata, OV5640_GPIO_SWITCH_CTL, OV5640_GPIO_SWITCH_VAL);
    if (rc < 0)
    {
        pr_err("%s: camera_clk_switch failed!\n", __func__);
        goto probe_init_fail;
    }
#else
    // do nothing
#endif
    /* Input MCLK */
    msm_camio_clk_rate_set(OV5640_CAMIO_MCLK);

    /* time delay for enabling MCLK */
	mdelay(10);

    /* Exit From Hard Standby */
    switch_on = 0;
    rc = ov5640_hard_standby(ov5640_ctrl->sensordata, switch_on);
    if (rc < 0)
    {
        pr_err("set standby failed!\n");
        goto probe_init_fail;
    }

    /*
      * Assert Sensor Reset
      * Ignore "rc"
      */
    rc = gpio_request(ov5640_ctrl->sensordata->sensor_reset, "ov5640");
    rc = gpio_direction_output(ov5640_ctrl->sensordata->sensor_reset, 1);
    gpio_free(ov5640_ctrl->sensordata->sensor_reset);

    /* time delay for asserting RESET */
	mdelay(10);

    /* Sensor Register Setting */
    rc = ov5640_sensor_init_probe(ov5640_ctrl->sensordata);
    if (rc < 0)
    {
        pr_err("%s: sensor_init_probe failed!\n", __func__);
        goto probe_init_fail;
    }
#else
    rc = ov5640_sensor_dev_probe(ov5640_ctrl->sensordata);
    if (rc < 0)
    {
        pr_err("%s: ov5640_sensor_dev_probe failed!\n", __func__);
        goto probe_init_fail;
    }

    /* Sensor Register Setting */
    rc = ov5640_reg_init();
    if (rc < 0)
    {
        pr_err("%s: ov5640_reg_init failed!\n", __func__);
        goto probe_init_fail;
    }
#endif

    /* Enter Into Hard Standby */
    rc = ov5640_sensor_release();
    if (rc < 0)
    {
        pr_err("%s: sensor_release failed!\n", __func__);
        goto probe_init_fail;
    }

    return 0;

probe_init_fail:
    /*
      * To power sensor down
      * Ignore "rc"
      */
    msm_camera_power_backend(MSM_CAMERA_PWRDWN_MODE);
    if(ov5640_ctrl)
    {
        kfree(ov5640_ctrl);
    }
    return rc;
}

static int ov5640_sensor_init(const struct msm_camera_sensor_info *data)
{
    uint32_t switch_on; 
    int rc;

    CDBG("%s: entry\n", __func__);

    if (!data || strcmp(data->sensor_name, "ov5640"))
    {
        pr_err("%s: invalid parameters!\n", __func__);
        rc = -ENODEV;
        goto sensor_init_fail;
    }

    /*
     * for v9plus,gpio 181 is configured as mclk switch
     * 0: back camera 1: front camera
     */
#if defined (CONFIG_MACH_V9PLUS)
    rc = msm_camera_clk_switch(ov5640_ctrl->sensordata, OV5640_GPIO_SWITCH_CTL, 0);
    if (rc < 0)
    {
        pr_err("%s: camera_clk_switch failed!\n", __func__);
        goto sensor_init_fail;
    }
#else
    //do nothing
#endif
    msm_camio_camif_pad_reg_reset();

    /* time delay for resetting CAMIF's PAD */ 
	mdelay(10);
    /* Input MCLK */
    msm_camio_clk_rate_set(OV5640_CAMIO_MCLK);

    /* time delay for enabling MCLK */ 
	mdelay(10);

    /* Exit From Hard Standby */   
    switch_on = 0;
    rc = ov5640_hard_standby(ov5640_ctrl->sensordata, switch_on);
    if (rc < 0)
    {
        pr_err("set standby failed!\n");
        rc = -EFAULT;
        goto sensor_init_fail;
    }
//#if defined (CONFIG_HI704_KERR)
#if 1
    rc = ov5640_i2c_write(ov5640_client->addr, 0x3017, 0x00FF, BYTE_LEN);
	if (rc < 0)
	{
	    pr_err("%s: failed 1, rc=%d!\n", __func__, rc);
	    return rc;
	}
	rc = ov5640_i2c_write(ov5640_client->addr, 0x3018, 0x00FF, BYTE_LEN);
	if (rc < 0)
	{
	    pr_err("%s: failed 2, rc=%d!\n", __func__, rc);
	    return rc;
	}
#endif
    /*
      * To avoid green display when preview mode is resumed
      * after snapshot or review.
      * The action of pulling STANDBY GPIO down is the key which
      * generates the problem mentioned above,
      * So time delay is added to avoid this problem.
      *
      * ZTE_YGL_CAM_20101018, ZTE_YGL_CAM_20101123
      * Decrease time delay from 700ms to 400ms in order to pass CTS
      */	  
	//mdelay(400);

    return 0;

sensor_init_fail:
    return rc; 
}

static int ov5640_sensor_config(void __user *argp)
{
    struct sensor_cfg_data cfg_data;
    long rc = 0;

    pr_err("%s: entry\n", __func__);

    if (copy_from_user(&cfg_data, (void *)argp, sizeof(struct sensor_cfg_data)))
    {
        pr_err("%s: copy_from_user failed!\n", __func__);
        return -EFAULT;
    }

    /* down(&ov5640_sem); */

    pr_err("%s: cfgtype = %d, mode = %d ,antibanding =%d\n", __func__, cfg_data.cfgtype, cfg_data.mode,cfg_data.cfg.antibanding);

    switch (cfg_data.cfgtype)
    {
        case CFG_SET_MODE:
        {
            rc = ov5640_set_sensor_mode(cfg_data.mode);
        }
        break;

        case CFG_SET_EFFECT:
        {
            rc = ov5640_set_effect(cfg_data.mode, cfg_data.cfg.effect);
	    zte_effect = cfg_data.cfg.effect;
        }
        break;

        case CFG_PWR_UP:
        {
            rc = ov5640_power_up();
        }
        break;

        case CFG_PWR_DOWN:
        {
            rc = ov5640_power_down();
        }
        break;

        case CFG_SET_WB:
        {
            rc = ov5640_set_wb(cfg_data.cfg.wb_mode);
			#ifdef ZTE_FIX_WB_ENABLE
			zte_wb_mode=cfg_data.cfg.wb_mode;
			#endif
        }
        break;

        case CFG_SET_BRIGHTNESS:
        {
            rc = ov5640_set_brightness(cfg_data.cfg.brightness);
	     zte_brightness = cfg_data.cfg.brightness;
        }
        break;
        
        case CFG_SET_CONTRAST:
        {
            rc = ov5640_set_contrast(cfg_data.cfg.contrast);
	     zte_contrast = cfg_data.cfg.contrast;
        }
        break;
        
        case CFG_SET_SATURATION:
        {
            rc = ov5640_set_saturation(cfg_data.cfg.saturation);
	    zte_sat = cfg_data.cfg.saturation;
        }
        break;

        case CFG_SET_AF:
        {
            rc = ov5640_af_trigger();
        }
        break;
        /* ZTE_YGL_CAM_20101230
         * add the interface of touch AF for froyo
         */
        case CFG_SET_AEC_RIO:
        {
            rc = ov5640_set_aec_rio(cfg_data.cfg.aec_rio);
        }
            break;
			
        case CFG_SET_ISO:
        {
            rc = ov5640_set_iso(cfg_data.cfg.iso_val);
        }
        break;

        case CFG_SET_ANTIBANDING:
        {
        	  pr_err("CFG_SET_ANTIBANDING");
            rc = ov5640_set_antibanding(cfg_data.cfg.antibanding);
        }
        break;

        case CFG_SET_SHARPNESS:
        {
            rc = ov5640_set_sharpness(cfg_data.cfg.sharpness);
	    zte_sharpness = cfg_data.cfg.sharpness;
        }
        break;

        case CFG_SET_EXPOSURE_COMPENSATION://ZTE_ZT_CAM_20101026_04
        {
            rc = ov5640_set_exposure_compensation(cfg_data.cfg.exposure);
        }
        break;
        
        default:
        {
            pr_err("%s: parameter error!\n", __func__);
            rc = -EFAULT;
        }
        break;
    }

    /* up(&ov5640_sem); */

    return rc;
}

static int ov5640_sensor_release(void)
{
    uint32_t switch_on;
    int rc = 0;

    CDBG("%s: entry\n", __func__);

    /* down(&ov5640_sem); */

    /*
      * MCLK is disabled by 
      * msm_camio_clk_disable(CAMIO_VFE_CLK)
      * in msm_camio_disable
      */
//      #if defined (CONFIG_HI704_KERR)

#if 1
	rc = ov5640_i2c_write(ov5640_client->addr, 0x3017, 0x0000, BYTE_LEN);
	if (rc < 0)
	{
	    pr_err("%s: failed, rc=%d!\n", __func__, rc);
	    return rc;
	}
	rc = ov5640_i2c_write(ov5640_client->addr, 0x3018, 0x0000, BYTE_LEN);
	if (rc < 0)
	{
	    pr_err("%s: failed, rc=%d!\n", __func__, rc);
	    return rc;
	}       
#endif
    /* Enter Into Hard Standby */
    /* ignore rc */
    switch_on = 1;
    rc = ov5640_hard_standby(ov5640_ctrl->sensordata, switch_on);

    //gpio_free(0);
	/* up(&ov5640_sem); */

    return rc;
}

static int ov5640_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int rc = 0;

    CDBG("%s: entry\n", __func__);

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        rc = -ENOTSUPP;
        goto probe_failure;
    }

    ov5640_sensorw = kzalloc(sizeof(struct ov5640_work_t), GFP_KERNEL);
    if (!ov5640_sensorw)
    {
        rc = -ENOMEM;
        goto probe_failure;
    }

    i2c_set_clientdata(client, ov5640_sensorw);

    ov5640_client = client;

    return 0;

probe_failure:
    kfree(ov5640_sensorw);
    ov5640_sensorw = NULL;
    pr_err("%s: rc = %d, failed!\n", __func__, rc);
    return rc;
}

static int __exit ov5640_i2c_remove(struct i2c_client *client)
{
    struct ov5640_work_t *sensorw = i2c_get_clientdata(client);

    CDBG("%s: entry\n", __func__);

    free_irq(client->irq, sensorw);   
    kfree(sensorw);

    ov5640_client = NULL;
    ov5640_sensorw = NULL;

    return 0;
}

static const struct i2c_device_id ov5640_id[] = {
    { "ov5640", 0},
    { },
};

static struct i2c_driver ov5640_driver = {
    .id_table = ov5640_id,
    .probe  = ov5640_i2c_probe,
    .remove = __exit_p(ov5640_i2c_remove),
    .driver = {
        .name = "ov5640",
    },
};

static int32_t ov5640_i2c_add_driver(void)
{
    int32_t rc = 0;

    rc = i2c_add_driver(&ov5640_driver);
    if (IS_ERR_VALUE(rc))
    {
        goto init_failure;
    }

    return rc;

init_failure:
    pr_err("%s: rc = %d, failed!\n", __func__, rc);
    return rc;
}

static void ov5640_i2c_del_driver(void)
{
    i2c_del_driver(&ov5640_driver);
}

void ov5640_exit(void)
{
    CDBG("%s: entry\n", __func__);
    ov5640_i2c_del_driver();
}

int ov5640_sensor_probe(const struct msm_camera_sensor_info *info,
                               struct msm_sensor_ctrl *s)
{
    int rc = 0;

    CDBG("%s: entry\n", __func__);


#if !defined(CONFIG_SENSOR_ADAPTER)
    rc = ov5640_i2c_add_driver();
    if (rc < 0)
    {
        goto probe_failed;
    }
#endif

#if 1

    rc = gpio_request(3, "mt9m114");
    if (rc < 0)
    {
        pr_err("%s: set mt9m114 reset to 0!\n", __func__);
        //goto probe_failed;
	return rc;	
    }
    pr_err("%s: set reset to 0\n", __func__);
    rc = gpio_direction_output(3, 0);
	gpio_free(3);
    /* up(&ov5640_sem); */

#endif

    rc = ov5640_sensor_probe_init(info);
    if (rc < 0)
    {
        pr_err("%s: ov5640_sensor_probe_init failed!\n", __func__);
        goto probe_failed;
    }

	/*
	 * add sensor configuration
	 * ZTE_CAM_LJ_20110413
	 */ 
    s->s_mount_angle = 0;
    s->s_camera_type = BACK_CAMERA_2D;

    s->s_init       = ov5640_sensor_init;
    s->s_config     = ov5640_sensor_config;
    s->s_release    = ov5640_sensor_release;

    return 0;

probe_failed:
    pr_err("%s: rc = %d, failed!\n", __func__, rc);

#if !defined(CONFIG_SENSOR_ADAPTER)
    ov5640_i2c_del_driver();
#else
    // Do nothing
#endif

    return rc;
}

static int __ov5640_probe_internal(struct platform_device *pdev)
{
#if defined(CONFIG_SENSOR_ADAPTER)
 	int rc;
#else
    // Do nothing
#endif

/*
 * Get FTM flag to adjust 
 * the initialize process 
 * of camera
 */
#ifdef CONFIG_ZTE_PLATFORM
#ifdef CONFIG_ZTE_FTM_FLAG_SUPPORT
    if(zte_get_ftm_flag())
    {
        return 0;
    }
#endif
#endif


#if !defined(CONFIG_SENSOR_ADAPTER)
/*
 * ZTE_CAMERA_LJ_20111121
 * add camera adapter for v9plus
 */
#if defined(CONFIG_CAMERA_ADAPTER)
    return msm_camera_drv_start(pdev, ov5640_sensor_probe,0);
#else
    return msm_camera_drv_start(pdev, ov5640_sensor_probe);
#endif
#else
    rc = msm_camera_dev_start(pdev,
                              ov5640_sensor_i2c_probe_on,
                              ov5640_sensor_i2c_probe_off,
                              ov5640_sensor_dev_probe);
    if (rc < 0)
    {
        pr_err("%s: msm_camera_dev_start failed!\n", __func__);
        goto probe_failed;
    }

    rc = msm_camera_drv_start(pdev, ov5640_sensor_probe);
    if (rc < 0)
    {
        goto probe_failed;
    }

    return 0;

probe_failed:
    pr_err("%s: rc = %d, failed!\n", __func__, rc);
    /* 
      * ZTE_JIA_CAM_20101209
      * to avoid standby current exception problem
      *
      * ignore "rc"
      */
    msm_camera_power_backend(MSM_CAMERA_PWRDWN_MODE);
    return rc;
#endif
}

#if defined(OV5640_PROBE_WORKQUEUE)
/* To implement the parallel init process */
static void ov5640_workqueue(struct work_struct *work)
{
    int32_t rc;

    /*
     * ignore "rc"
     */

    rc = __ov5640_probe_internal(pdev_wq);
}

static int32_t ov5640_probe_workqueue(void)
{
    int32_t rc;


    ov5640_wq = create_singlethread_workqueue("ov5640_wq");

    if (!ov5640_wq)
    {
        pr_err("%s: ov5640_wq is NULL!\n", __func__);
        return -EFAULT;
    }

    /*
      * Ignore "rc"
      * "queue_work"'s rc:
      * 0: already in work queue
      * 1: added into work queue
      */   
    rc = queue_work(ov5640_wq, &ov5640_cb_work);

    return 0;
}

static int __ov5640_probe(struct platform_device *pdev)
{
    int32_t rc;

    pdev_wq = pdev;


    rc = ov5640_probe_workqueue();

    return rc;
}
#else
static int __ov5640_probe(struct platform_device *pdev)
{
    return __ov5640_probe_internal(pdev);
}
#endif /* OV5640_PROBE_WORKQUEUE */

static struct platform_driver msm_camera_driver = {
    .probe = __ov5640_probe,
    .driver = {
        .name = "msm_camera_ov5640",
        .owner = THIS_MODULE,
    },
};

static int __init ov5640_init(void)
{
    printk("zt debug: %s\n", __func__);//zhangtao
    pr_err(": %s\n", __func__);//zhangtao
    return platform_driver_register(&msm_camera_driver);
}

module_init(ov5640_init);

