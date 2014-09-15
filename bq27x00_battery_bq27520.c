/*
 * bqGauge battery driver
 *
 * Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 * Copyright (C) 2010-2011 Lars-Peter Clausen <lars@metafoo.de>
 * Copyright (C) 2011 Pali Rohár <pali.rohar@gmail.com>
 *
 * Based on a previous work by Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

/*
 * Datasheets:
 * http://focus.ti.com/docs/prod/folders/print/bq27000.html
 * http://focus.ti.com/docs/prod/folders/print/bq27500.html
 * http://www.ti.com/product/bq27411-g1
 * http://www.ti.com/product/bq27421-g1
 * http://www.ti.com/product/bq27425-g1
 * http://www.ti.com/product/bq27441-g1
 */

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>

#define BQ27520_UPDATER
#undef  BQ27520_UPDATER

#ifdef BQ27520_UPDATER
#include "bqfs_cmd_type.h"
#include "bqfs_image.h"
#endif 

//#include <linux/power/bq27x00_battery.h>

#define DRIVER_VERSION            "1.3.0"


#define I2C_RETRY_CNT    3
#define BQGAUGE_I2C_ROM_ADDR    (0x16 >> 1)
#define BQGAUGE_I2C_DEV_ADDR    (0xAA >> 1)



struct bqGauge_device_info;

struct bqGauge_Device{
    // interface to report property request from host
    void(*updater)          (struct bqGauge_device_info *di);
    int (*read_fw_ver)      (struct bqGauge_device_info *di);
    int (*read_status)      (struct bqGauge_device_info *di);
    int (*read_fcc)         (struct bqGauge_device_info *di);
    int (*read_designcap)   (struct bqGauge_device_info *di);
    int (*read_rsoc)        (struct bqGauge_device_info *di);
    int (*read_temperature) (struct bqGauge_device_info *di);
    int (*read_cyclecount)  (struct bqGauge_device_info *di);
    int (*read_timetoempty) (struct bqGauge_device_info *di);
    int (*read_timetofull)  (struct bqGauge_device_info *di);
    int (*read_health)      (struct bqGauge_device_info *di);
    int (*read_voltage)     (struct bqGauge_device_info *di);
    int (*read_current)     (struct bqGauge_device_info *di);
    int (*read_capacity_level)(struct bqGauge_device_info *di);
};


enum bqGauge_chip {BQ27520,BQ27320};

static DEFINE_MUTEX(battery_mutex);

struct bqGauge_reg_cache{
    int temperature;
    int voltage;
    int currentI;
    int time_to_empty;
    int time_to_full;
    int charge_full;
    int charge_design_full;
    int cycle_count;
    int rsoc;
    int flags;
    int health;
};

struct bqGauge_device_info {
    struct  device        *dev;
    struct  bqGauge_Device *gauge;

    int     id;
    enum    bqGauge_chip    chip;
 
    int     fw_ver;// format : AABBCCDD: AABB version, CCDD build number
    int     df_ver;

    struct  bqGauge_reg_cache cache;
 
    unsigned long last_update;
    
    struct  delayed_work work;
    struct  power_supply bat;

    struct  mutex lock;

};

static data enum power_supply_property bqGauge_battery_props[] = {
    POWER_SUPPLY_PROP_STATUS,
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_CURRENT_NOW,
    POWER_SUPPLY_PROP_CAPACITY,
    POWER_SUPPLY_PROP_TEMP,
    POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
    POWER_SUPPLY_PROP_CHARGE_FULL,
    POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
    POWER_SUPPLY_PROP_CYCLE_COUNT,
    POWER_SUPPLY_PROP_HEALTH,
};

static unsigned int poll_interval = 60;
module_param(poll_interval, uint, 0644);
MODULE_PARM_DESC(poll_interval, "battery poll interval in seconds - " \
                "0 disables polling");


// common routines for bq I2C gauge                
static int bq_read_i2c_byte(struct bqGauge_device_info *di, u8 reg)
{
    struct i2c_client *client = to_i2c_client(di->dev);
    struct i2c_msg msg[2];
    unsigned char data;
    int ret;
    int i = 0;

    if (!client->adapter)
        return -ENODEV;

    msg[0].addr = client->addr;
    msg[0].flags = 0;
    msg[0].buf = &reg;
    msg[0].len = sizeof(reg);
    msg[1].addr = client->addr;
    msg[1].flags = I2C_M_RD;
    msg[1].buf = &data;
    msg[1].len = 1;

    mutex_lock(&battery_mutex);
    for(i = 0; i < I2C_RETRY_CNT; i++){
        ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
        if(ret >= 0) break;
        msleep(5);
    }
    mutex_unlock(&battery_mutex);
    if (ret < 0)
        return ret;

    ret = data;

    return ret;
}

static int bq_read_i2c_word(struct bqGauge_device_info *di, u8 reg)
{
    struct i2c_client *client = to_i2c_client(di->dev);
    struct i2c_msg msg[2];
    unsigned char data[2];
    int ret;
    int i = 0;

    if (!client->adapter)
        return -ENODEV;

    msg[0].addr = client->addr;
    msg[0].flags = 0;
    msg[0].buf = &reg;
    msg[0].len = sizeof(reg);
    msg[1].addr = client->addr;
    msg[1].flags = I2C_M_RD;
    msg[1].buf = data;
    msg[1].len = 2;

    mutex_lock(&battery_mutex);
    for(i = 0; i < I2C_RETRY_CNT; i++){
        ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
        if(ret >= 0) break;
        msleep(5);
    }
    mutex_unlock(&battery_mutex);
    if (ret < 0)
        return ret;

    ret = get_unaligned_le16(data);;

    return ret;
}

static int bq_write_i2c_byte(struct bqGauge_device_info *di, u8 reg, unsigned char value)
{
    struct i2c_client *client = to_i2c_client(di->dev);
    struct i2c_msg msg;
    unsigned char data[4];
    int ret;
    int i = 0;

    if (!client->adapter)
        return -ENODEV;

    data[0] = reg;
    data[1] = value;
    
    msg.len = 2;
    msg.buf = data;
    msg.addr = client->addr;
    msg.flags = 0;

    mutex_lock(&battery_mutex);
    for(i = 0; i < I2C_RETRY_CNT; i++){
        ret = i2c_transfer(client->adapter, &msg, 1);
        if(ret >= 0) break;
        msleep(5);
    }
    mutex_unlock(&battery_mutex);    
    if (ret < 0)
        return ret;

    return 0;
}


static int bq_write_i2c_word(struct bqGauge_device_info *di, u8 reg, int value)
{
    struct i2c_client *client = to_i2c_client(di->dev);
    struct i2c_msg msg;
    unsigned char data[4];
    int ret;
    int i = 0;

    if (!client->adapter)
        return -ENODEV;

    data[0] = reg;
    put_unaligned_le16(value, &data[1]);
    
    msg.len = 3;
    msg.buf = data;
    msg.addr = client->addr;
    msg.flags = 0;

    mutex_lock(&battery_mutex);
    for(i = 0; i < I2C_RETRY_CNT; i++){
        ret = i2c_transfer(client->adapter, &msg, 1);
        if(ret >= 0) break;
        msleep(5);
    }
    mutex_unlock(&battery_mutex);    
    if (ret < 0)
        return ret;

    return 0;
}


static int bq_read_i2c_blk(struct bqGauge_device_info *di, u8 reg, u8 *data, u8 len)
{

    struct i2c_client *client = to_i2c_client(di->dev);
    struct i2c_msg msg[2];
    int ret;
    int i = 0;

    if (!client->adapter)
        return -ENODEV;

    msg[0].addr = client->addr;
    msg[0].flags = 0;
    msg[0].buf = &reg;
    msg[0].len = 1;

    msg[1].addr = client->addr;
    msg[1].flags = I2C_M_RD;
    msg[1].buf = data;
    msg[1].len = len;

    mutex_lock(&battery_mutex);
    for(i = 0; i < I2C_RETRY_CNT; i++){
        ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
        if(ret >= 0) break;
        msleep(5);
    }
    mutex_unlock(&battery_mutex);    

    if (ret < 0)
        return ret;

    return ret;
}

static int bq_write_i2c_blk(struct bqGauge_device_info *di, u8 reg, u8 *data, u8 sz)
{
    struct i2c_client *client = to_i2c_client(di->dev);
    struct i2c_msg msg;
    int ret;
    int i = 0;
    u8 buf[200];

    if (!client->adapter)
        return -ENODEV;

    buf[0] = reg;
    memcpy(&buf[1], data, sz);

    msg.buf = buf;
    msg.addr = client->addr;
    msg.flags = 0;
    msg.len = sz + 1;

    mutex_lock(&battery_mutex);
    for(i = 0; i < I2C_RETRY_CNT; i++){
        ret = i2c_transfer(client->adapter, &msg, 1);
        if(ret >= 0) break;
        msleep(5);
    }
    mutex_unlock(&battery_mutex);
    if (ret < 0)
        return ret;

    return 0;
}

/* tool, support function */
static u8 checksum(u8 *data, u8 len)
{
    u16 sum = 0;
    int i;

    for (i = 0; i < len; i++)
        sum += data[i];

    sum &= 0xFF;

    return 0xFF - sum;
}




/* bq27520 device stuff */
#define BQ27520_REG_CONTRL          0x00
#define BQ27520_REG_FLAGS           0x0A
#define BQ27520_REG_FCC             0x12
#define BQ27520_REG_RSOC            0x20
#define BQ27520_REG_DESIGNCAP       0x2E
#define BQ27520_REG_TEMP            0x06
#define BQ27520_REG_CYCLECNT        0x1E
#define BQ27520_REG_TTE             0x16
#define BQ27520_REG_VOLT            0x08
#define BQ27520_REG_CURRENT         0x14
#define BQ27520_REG_OPSTATUS        0x2C

#define BQ27520_REG_DFCLASS         0x3E
#define BQ27520_REG_DFBLOCK         0x3F
#define BQ27520_REG_BLOCKDATA       0x40

#define BQ27520_REG_BLKCHKSUM       0x60
#define BQ27520_REG_BLKDATCTL       0x61

#define BQ27520_SECURITY_SEALED     0x03
#define BQ27520_SECURITY_UNSEALED   0x02
#define BQ27520_SECURITY_FA         0x01
#define BQ27520_SECURITY_MASK       0x03


#define BQ27520_UNSEAL_KEY          0x36720414
#define BQ27520_FA_KEY              0xFFFFFFFF


#define BQ27520_FLAG_DSG            BIT(0)  
#define BQ27520_FLAG_OTC            BIT(15)
#define BQ27520_FLAG_OTD            BIT(14)
#define BQ27520_FLAG_FC             BIT(9)  

#define BQ27520_SUBCMD_FLAGS        0x0000
#define BQ27520_SUBCMD_FWVER        0x0002
#define BQ27520_SUBCMD_ENTER_ROM    0x0F00
#define BQ27520_SUBCMD_SEAL         0x0020

static int bq27520_read_fw_version(struct bqGauge_device_info *di)
{
    int ret;
    u8 buf[36];
    
    ret = bq_write_i2c_word(di,BQ27520_REG_CONTRL,BQ27520_SUBCMD_FWVER);
    if(ret < 0){
        dev_err(di->dev,"Failed to send read fw version command\n");
        return ret;
    }
    mdelay(2);
    ret = bq_read_i2c_word(di,BQ27520_REG_CONTRL);
    if(ret < 0){
        dev_err(di->dev,"Failed to read read fw version \n");
        return ret;
    }
 
    return ret;
  
}

static int bq27520_read_current(struct bqGauge_device_info *);

static int bq27520_read_status(struct bqGauge_device_info *di)
{
    int flags;
    int status;
    short curr;
    
    curr = bq27520_read_current(di);
    mdelay(2);
    flags = bq_read_i2c_word(di, BQ27520_REG_FLAGS);
    if(flags < 0){
        dev_err(di->dev,"Failed to read battery status register:%d\n", flags);
        flags = di->cache.flags;
    }
    di->cache.flags = flags;
    
    if(flags & BQ27520_FLAG_FC)
        status = POWER_SUPPLY_STATUS_FULL;
    else if (flags & BQ27520_FLAG_DSG)
        status = POWER_SUPPLY_STATUS_DISCHARGING;
    else if (curr > 0)
        status = POWER_SUPPLY_STATUS_CHARGING;
    else
        status = POWER_SUPPLY_STATUS_NOT_CHARGING;
        
    return status;
}


static int bq27520_read_fcc(struct bqGauge_device_info *di)
{
    int ret;
    ret = bq_read_i2c_word(di, BQ27520_REG_FCC);
    if(ret < 0){
        dev_err(di->dev,"Failed to read FullChargeCapacity register:%d\n", ret);
        ret = di->cache.charge_full;
    }
    di->cache.charge_full = ret;

    return ret * 1000;
}

static int bq27520_read_designcapacity(struct bqGauge_device_info *di)
{
    int ret;
    ret = bq_read_i2c_word(di, BQ27520_REG_DESIGNCAP);
    if(ret < 0){
        dev_err(di->dev,"Failed to read FullChargeCapacity register:%d\n", ret);
        ret = di->cache.charge_design_full;
    }
    di->cache.charge_design_full = ret;

    return ret * 1000;
}

static int bq27520_read_rsoc(struct bqGauge_device_info *di)
{
    int ret;
    ret = bq_read_i2c_word(di, BQ27520_REG_RSOC);
    if(ret < 0){
        dev_err(di->dev,"Failed to read RSOC register:%d\n", ret);
        ret = di->cache.rsoc;
    }
    di->cache.rsoc = ret;

    return ret;

}

static int bq27520_read_temperature(struct bqGauge_device_info *di)
{
    int ret;
    ret = bq_read_i2c_word(di, BQ27520_REG_TEMP);
    if(ret < 0){
        dev_err(di->dev,"Failed to read TEMP register:%d\n", ret);
        ret = di->cache.temperature;
    }
    di->cache.temperature = ret;

    return ret;

}

static int bq27520_read_cyclecount(struct bqGauge_device_info *di)
{
    int ret;
    ret = bq_read_i2c_word(di, BQ27520_REG_CYCLECNT);
    if(ret < 0){
        dev_err(di->dev,"Failed to read CycleCount register:%d\n", ret);
        ret = di->cache.cycle_count;
    }
    di->cache.cycle_count = ret;

    return ret;

}

static int bq27520_read_timetoempty(struct bqGauge_device_info *di)
{
    int ret;
    ret = bq_read_i2c_word(di, BQ27520_REG_TTE);
    if(ret < 0){
        dev_err(di->dev,"Failed to read TimeToEmpty register:%d\n", ret);
        ret = di->cache.time_to_empty;
    }
    di->cache.time_to_empty = ret;

    return ret;

}

static int bq27520_read_health(struct bqGauge_device_info *di)
{
    int flags;
    int status;
    flags = bq_read_i2c_word(di, BQ27520_REG_FLAGS);
    if(flags < 0){
        dev_err(di->dev,"Failed to read BatteryStatus:%d\n", flags);
        flags = di->cache.flags;
    }
    di->cache.flags = flags;
    if(flags & (BQ27520_FLAG_OTC | BQ27520_FLAG_OTD))
        status = POWER_SUPPLY_HEALTH_OVERHEAT;
    else
        status = POWER_SUPPLY_HEALTH_GOOD;
        
    return status;
}

static int bq27520_read_voltage(struct bqGauge_device_info *di)
{

    int ret;
    ret = bq_read_i2c_word(di, BQ27520_REG_VOLT);
    if(ret < 0){
        dev_err(di->dev,"Failed to read Voltage register:%d\n", ret);
        ret = di->cache.voltage;
    }
    di->cache.voltage = ret;
    
    return (ret * 1000);

}

static int bq27520_read_current(struct bqGauge_device_info *di)
{
    int ret;
    ret = bq_read_i2c_word(di, BQ27520_REG_CURRENT);
    if(ret < 0){
        dev_err(di->dev,"Failed to read Curent register:%d\n", ret);
        ret = di->cache.currentI;
    }
    di->cache.currentI = ret;
    
    return (int)((s16)ret) * 1000;
}

static int bq27520_read_capacity_level(struct bqGauge_device_info *di)
{
    int level;
    int rsoc;
    int flags;
    
    rsoc = bq27520_read_rsoc(di);
    flags = bq27520_read_status(di);
    if(rsoc > 95)
        level = POWER_SUPPLY_CAPACITY_LEVEL_HIGH;
    else if (flags & 0x02) // SYSDOWN  
        level = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
    else if (flags & 0x04)
        level = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
    else
        level = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
    
    return level;
}

#ifdef BQ27520_UPDATER
static void bq27520_update_bqfs(struct bqGauge_device_info *di);
#endif

struct bqGauge_Device bqGauge_27520 = {
#ifdef BQ27520_UPDATER
    .updater            = bq27520_update_bqfs,
#else   
    .updater            = NULL,
#endif
    .read_fw_ver        = bq27520_read_fw_version,
    .read_status        = bq27520_read_status,
    .read_fcc           = bq27520_read_fcc,
    .read_designcap     = bq27520_read_designcapacity,
    .read_rsoc          = bq27520_read_rsoc,
    .read_health        = bq27520_read_health,
    .read_voltage       = bq27520_read_voltage,
    .read_current       = bq27520_read_current,    
    .read_temperature   = bq27520_read_temperature,
    .read_cyclecount    = bq27520_read_cyclecount,
    .read_timetoempty   = bq27520_read_timetoempty,
    .read_timetofull    = NULL,
    .read_capacity_level= bq27520_read_capacity_level,
 };

 
//function works with assumption that deivce is not sealed
static int bq27520_read_df(struct bqGauge_device_info * di,u8 classid, u8 offset, u8* buf, u8 len)
{
    int ret;
    u8 tmp_buf[40];
    int i;
    int crc;
    u8 crc_calc = 0;
    
    if (offset % 32 + len > 32) return -1; // less than one block boundary one time

    ret = bq_write_i2c_byte(di,BQ27520_REG_BLKDATCTL, 0);
    if(ret < 0) return ret;
    mdelay(2);
    ret = bq_write_i2c_byte(di,BQ27520_REG_DFCLASS, classid);
    if(ret < 0) return ret;
    mdelay(2);
    ret = bq_write_i2c_byte(di,BQ27520_REG_DFBLOCK, offset / 32);
    if(ret < 0) return ret;
    
    mdelay(2);
    ret = bq_read_i2c_blk(di,BQ27520_REG_BLOCKDATA,tmp_buf, 32);
    if(ret < 0) return ret;
    
    crc = bq_read_i2c_byte(di, BQ27520_REG_BLKCHKSUM);
    crc_calc = checksum(tmp_buf,32);
    if(crc != crc_calc) return -2;
    
    for(i = 0; i < len; i++){
        buf[i] =  tmp_buf[offset % 32 + i];
    }
    return len;
} 


static int bq27520_write_df(struct bqGauge_device_info * di,u8 classid, u8 offset, u8* buf, u8 len)
{
    int ret;
    u8 tmp_buf[40];
    int i;
    int crc;
    u8 crc_calc = 0;
    
    if (offset % 32 + len > 32) return -1; // less than one block one time

    ret = bq_write_i2c_byte(di,BQ27520_REG_BLKDATCTL, 0);
    if(ret < 0) return ret;
    mdelay(2);
    ret = bq_write_i2c_byte(di,BQ27520_REG_DFCLASS, classid);
    if(ret < 0) return ret;
    mdelay(2);
    ret = bq_write_i2c_byte(di,BQ27520_REG_DFBLOCK, offset / 32);
    if(ret < 0) return ret;
    
    mdelay(2);
    ret = bq_read_i2c_blk(di,BQ27520_REG_BLOCKDATA,tmp_buf, 32);
    if(ret < 0) return ret;
    
    mdelay(2);
    crc = bq_read_i2c_byte(di, BQ27520_REG_BLKCHKSUM);
    crc_calc = checksum(tmp_buf,32);
    if(crc != crc_calc) return -2;
    //update with new value
    for(i = 0; i < len; i++)
        tmp_buf[offset % 32 + i] = buf[i];
    // calculate new crc 
    crc_calc = checksum(tmp_buf,32);
    mdelay(2);
    ret = bq_write_i2c_blk(di,BQ27520_REG_BLOCKDATA,tmp_buf,32);
    if(ret < 0) return ret;
    
    mdelay(2);
    ret = bq_write_i2c_byte(di,BQ27520_REG_BLKCHKSUM,crc_calc);
    return ret;
} 
 
 
#ifdef BQ27520_UPDATER 
/* the following routines are for bqfs/dffs update purpose, can be removed if not used*/
static int bq27520_check_seal_state(struct bqGauge_device_info * di)
{
    int status;
    bq_write_i2c_word(di, BQ27520_REG_CONTRL, BQ27520_SUBCMD_FLAGS);
    mdelay(2);
    status = bq_read_i2c_word(di, BQ27520_REG_CONTRL);
    if(status < 0) return status;
    
    if(((u16)status & 0x6000) == 0) //FA and SS neither set
        status = BQ27520_SECURITY_FA;
    else if(((u16)status & 0x2000) == 0) // SS not set
        status = BQ27520_SECURITY_UNSEALED;
    else    
        status = BQ27520_SECURITY_SEALED;

    return status;
}

static int bq27520_unseal(struct bqGauge_device_info * di)
{
    int ret;
    
    bq_write_i2c_word(di, BQ27520_REG_CONTRL, BQ27520_UNSEAL_KEY & 0xFFFF);
    mdelay(2);
    bq_write_i2c_word(di, BQ27520_REG_CONTRL, (BQ27520_UNSEAL_KEY >> 16)& 0xFFFF);
    mdelay(5);
    
    ret = bq27520_check_seal_state(di);
    if(ret == BQ27520_SECURITY_UNSEALED || ret == BQ27520_SECURITY_FA)
        return 1;
    else
        return 0;
}

static int bq27520_unseal_full_access(struct bqGauge_device_info * di)
{
    int ret;
    
    bq_write_i2c_word(di, BQ27520_REG_CONTRL, BQ27520_FA_KEY & 0xFFFF);
    mdelay(2);
    bq_write_i2c_word(di, BQ27520_REG_CONTRL, (BQ27520_FA_KEY >> 16)& 0xFFFF);
    mdelay(5);
    
    ret = bq27520_check_seal_state(di);
    if(ret == BQ27520_SECURITY_FA)
        return 1;
    else
        return 0;

}

static bool bqGauge_check_rom_mode(struct bqGauge_device_info * di)
{
    struct i2c_client *client = to_i2c_client(di->dev);
    int ret;

    client->addr = BQGAUGE_I2C_ROM_ADDR;
    ret = bq_read_i2c_byte(di,0x66);
    mdelay(2);
    client->addr = BQGAUGE_I2C_DEV_ADDR;//restore address
    if(ret < 0 ){ // not in rom mode 
        return false;
    }
    return true;
}

static bool bq27520_enter_rom_mode(struct bqGauge_device_info * di)
{
    int ret;
    
    ret = bq_write_i2c_word(di,BQ27520_REG_CONTRL,BQ27520_SUBCMD_ENTER_ROM);
    mdelay(2);
    if(ret < 0) return false;
    
    return bqGauge_check_rom_mode(di);
}

#define BQ27520_DEVICE_NAME_CLASSID     48
#define BQ27520_DEVICE_NAME_OFFSET      17
#define BQ27520_DEVICE_NAME_LENGTH      7

static bool bq27520_check_update_necessary(struct bqGauge_device_info * di)
{
    // this is application specific, return true if need update firmware or data flash
    u8 buf[40];
    int ret;

    ret = bq27520_read_df(di,BQ27520_DEVICE_NAME_CLASSID,BQ27520_DEVICE_NAME_OFFSET, buf, BQ27520_DEVICE_NAME_LENGTH);
    if( ret != BQ27520_DEVICE_NAME_LENGTH)
        return false;
    if(strncmp(buf,"VER0329",BQ27520_DEVICE_NAME_LENGTH) == 0)//same
        return false;
    else    
        return true;
}

static bool bq27520_mark_as_updated(struct bqGauge_device_info * di)
{
    // this is application specific
    int ret;
    ret = bq27520_write_df(di,BQ27520_DEVICE_NAME_CLASSID,BQ27520_DEVICE_NAME_OFFSET, "VER0329", BQ27520_DEVICE_NAME_LENGTH);
    if(ret < 0) 
        return false;
    else    
        return true;
}


static bool bq27520_update_execute_cmd(struct bqGauge_device_info *di, const bqfs_cmd_t *cmd)
{
    int ret;
    uint8_t tmp_buf[CMD_MAX_DATA_SIZE];

    switch (cmd->cmd_type) {
    case CMD_R:
        ret = bq_read_i2c_blk(di, cmd->reg, (u8 *)&cmd->data.bytes, cmd->data_len);
        if( ret < 0) return false;
        return true;

    case CMD_W:
        ret = bq_write_i2c_blk(di,cmd->reg,(u8 *)&cmd->data.bytes, cmd->data_len);
        if(ret < 0) return false;
        return true;
    case CMD_C:
        if (bq_read_i2c_blk(di,cmd->reg,tmp_buf,cmd->data_len) < 0)
            return false;//read fail
        if (memcmp(tmp_buf, cmd->data.bytes, cmd->data_len)) {
            dev_dbg(di->dev, "\nCommand C failed at line %d:\n",
                cmd->line_num);
            return false;
        }
        return true;

    case CMD_X:
        mdelay(cmd->data.delay);
        return true;

    default:
        dev_err(di->dev, "Unsupported command at line %d\n",
            cmd->line_num);
        return false;
    }
}


static void bq27520_update_bqfs(struct bqGauge_device_info *di)
{
    struct i2c_client *client = to_i2c_client(di->dev);
    u16 i;

    if(bqGauge_check_rom_mode(di)) goto update;// already in rom mode
    // check if needed update 
    if(!bq27520_check_update_necessary(di)) return;

    if (bq27520_check_seal_state(di) != BQ27520_SECURITY_FA){
        if(!bq27520_unseal(di)) 
            return;
        mdelay(10);
        if(!bq27520_unseal_full_access(di)) 
            return;
    }

    if(!bq27520_enter_rom_mode(di)) return;
    
update:    
    client->addr = BQGAUGE_I2C_ROM_ADDR;
    dev_info(di->dev,"Updating");
    for(i = 0; i < ARRAY_SIZE(bqfs_image); i++){
        dev_info(di->dev,".");
        if(!bq27520_update_execute_cmd(di,&bqfs_image[i])){
            dev_err(di->dev,"%s:Failed at command:%d\n",__func__,i);
            return;
        }
    }
    dev_info(di->dev,"Done!\n");

    client->addr = BQGAUGE_I2C_DEV_ADDR;    
    // mark as updated
    bq27520_mark_as_updated(di);
    
    return ;
}

#endif

/* bqfs/dffs update routines end */

static void bqGauge_refresh(struct bqGauge_device_info *di)
{
    static int rsoc_prev = -1;
    
    if(!di->gauge) return;

    if(di->gauge->read_status){
        di->gauge->read_status(di);
        mdelay(2);
    }
    
    if(di->gauge->read_current){
        di->gauge->read_current(di);
        mdelay(2);
    }

    if(di->gauge->read_voltage){
        di->gauge->read_voltage(di);
        mdelay(2);
    }
        
    if(di->gauge->read_temperature){
        di->gauge->read_temperature(di);
        mdelay(2);
    }
      
    if(di->gauge->read_timetoempty){
        di->gauge->read_timetoempty(di);
        mdelay(2);
    }

    if(di->gauge->read_timetofull){
        di->gauge->read_timetofull(di);
        mdelay(2);
    }
        
    if(di->gauge->read_fcc){
        di->gauge->read_fcc(di);
        mdelay(2);
    }   
    
    if(di->gauge->read_rsoc){
        di->gauge->read_rsoc(di);
        mdelay(2);
    }   
    
    if(di->cache.rsoc != rsoc_prev){
        rsoc_prev = di->cache.rsoc;
        power_supply_changed(&di->bat);
    }

    di->last_update = jiffies;
}

static void bqGauge_battery_poll(struct work_struct *work)
{
    struct bqGauge_device_info *di =
        container_of(work, struct bqGauge_device_info, work.work);


    bqGauge_refresh(di);

    if (poll_interval > 0) {
        /* The timer does not have to be accurate. */
        set_timer_slack(&di->work.timer, poll_interval * HZ / 4);
        schedule_delayed_work(&di->work, poll_interval * HZ);
    }
}


#define to_bqGauge_device_info(x) container_of((x), \
                struct bqGauge_device_info, bat);

static int bqGauge_get_property(struct power_supply *psy,
                    enum power_supply_property psp,
                    union power_supply_propval *val)
{
    struct bqGauge_device_info *di = to_bqGauge_device_info(psy);

    mutex_lock(&di->lock);
    if (time_is_before_jiffies(di->last_update + 5 * HZ)) {
        cancel_delayed_work_sync(&di->work);
        bqGauge_battery_poll(&di->work.work);
    }
    mutex_unlock(&di->lock);

    if(di->gauge == NULL)
        return -ENODEV;
        
    switch (psp) {
    case POWER_SUPPLY_PROP_STATUS:
        if(di->gauge->read_status)
            val->intval = di->gauge->read_status(di);
        else
            val->intval = -EINVAL;
        break;
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        if(di->gauge->read_voltage)
            val->intval = di->gauge->read_voltage(di);
        else
            val->intval = -EINVAL;
        break;
    case POWER_SUPPLY_PROP_PRESENT:
        if(di->gauge->read_voltage)
            val->intval = di->gauge->read_voltage(di) > 0 ? 1 : 0;
        else
            val->intval = -EINVAL;
        break;
    case POWER_SUPPLY_PROP_CURRENT_NOW:
        if(di->gauge->read_current)
            val->intval = -di->gauge->read_current(di);
        else
            val->intval = -EINVAL;
        break;
    case POWER_SUPPLY_PROP_CAPACITY:
        if(di->gauge->read_rsoc)
            val->intval = di->gauge->read_rsoc(di);
        else
            val->intval = -EINVAL;
        break;
    case POWER_SUPPLY_PROP_TEMP:
        if(di->gauge->read_temperature)
            val->intval = di->gauge->read_temperature(di) - 2731;
        else
            val->intval = -EINVAL;
        break;
    case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
        if(di->gauge->read_timetoempty)
            val->intval = di->gauge->read_timetoempty(di);
        else
            val->intval = -EINVAL;
        break;
    case POWER_SUPPLY_PROP_CHARGE_FULL:
        if(di->gauge->read_fcc)
            val->intval = di->gauge->read_fcc(di);
        else
            val->intval = -EINVAL;
        break;
    case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
        if(di->gauge->read_designcap)
            val->intval = di->gauge->read_designcap(di);
        else
            val->intval = -EINVAL;
        break;
    case POWER_SUPPLY_PROP_CYCLE_COUNT:
        if(di->gauge->read_cyclecount)
            val->intval = di->gauge->read_cyclecount(di);
        else
            val->intval = -EINVAL;
        break;
    case POWER_SUPPLY_PROP_HEALTH:
        if(di->gauge->read_health)
            val->intval = di->gauge->read_health(di);
        else
            val->intval = -EINVAL;
        break;
    default:
        return -EINVAL;
    }

    return 0;
}

static void bqGauge_external_power_changed(struct power_supply *psy)
{
    struct bqGauge_device_info *di = to_bqGauge_device_info(psy);

    cancel_delayed_work_sync(&di->work);
    schedule_delayed_work(&di->work, 0);
}

static void  set_properties_array(struct bqGauge_device_info *di,
    enum power_supply_property *props, int num_props)
{
    int tot_sz = num_props * sizeof(enum power_supply_property);

    di->bat.properties = devm_kzalloc(di->dev, tot_sz, GFP_KERNEL);

    if (di->bat.properties) {
        memcpy(di->bat.properties, props, tot_sz);
        di->bat.num_properties = num_props;
    } else {
        di->bat.num_properties = 0;
    }
}

static int  bqGauge_powersupply_init(struct bqGauge_device_info *di)
{
    int ret;

    di->bat.type = POWER_SUPPLY_TYPE_BATTERY;

    set_properties_array(di, bqGauge_battery_props,
            ARRAY_SIZE(bqGauge_battery_props));
            
    di->bat.get_property = bqGauge_get_property;
    di->bat.external_power_changed = bqGauge_external_power_changed;

    INIT_DELAYED_WORK(&di->work, bqGauge_battery_poll);
    mutex_init(&di->lock);

    ret = power_supply_register(di->dev, &di->bat);
    if (ret) {
        dev_err(di->dev, "failed to register battery: %d\n", ret);
        return ret;
    }

    dev_info(di->dev, "support ver. %s enabled\n", DRIVER_VERSION);

    bqGauge_refresh(di);

    return 0;
}

static void bqGauge_powersupply_unregister(struct bqGauge_device_info *di)
{
    /*
     * power_supply_unregister call bqGauge_battery_get_property which
     * call bqGauge_battery_poll.
     * Make sure that bqGauge_battery_poll will not call
     * schedule_delayed_work again after unregister (which cause OOPS).
     */
    poll_interval = 0;

    cancel_delayed_work_sync(&di->work);

    power_supply_unregister(&di->bat);

    mutex_destroy(&di->lock);
}


/* i2c specific code */
#ifdef CONFIG_BATTERY_BQ27X00_I2C

/* If the system has several batteries we need a different name for each
 * of them...
 */

static ssize_t show_firmware_version(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct bqGauge_device_info *di = dev_get_drvdata(dev);
    int ver = 0;

    if(di->gauge && di->gauge->read_fw_ver)
        ver = di->gauge->read_fw_ver(di);

    return sprintf(buf, "%04x\n", ver);
}

static ssize_t show_device_regs(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct bqGauge_device_info *di = dev_get_drvdata(dev);
    
    bqGauge_refresh(di);

    return sprintf(buf, "volt:%d, current: %d, Temperature: %d, RSOC: %d, FCC: %d, TTE: %d, Flags: %02X\n", 
                        di->cache.voltage,(s16)di->cache.currentI, di->cache.temperature,di->cache.rsoc, di->cache.charge_full,di->cache.time_to_empty,di->cache.flags);    

}


static DEVICE_ATTR(fw_version, S_IRUGO, show_firmware_version, NULL);
static DEVICE_ATTR(show_regs, S_IRUGO, show_device_regs,NULL);


static struct attribute *bqGauge_attributes[] = {
    &dev_attr_fw_version.attr,
    &dev_attr_show_regs.attr,
    NULL
};

static const struct attribute_group bqGauge_attr_group = {
    .attrs = bqGauge_attributes,
};

static DEFINE_IDR(battery_id);

static int  bqGauge_battery_probe(struct i2c_client *client,
                 const struct i2c_device_id *id)
{
    char *name;
    struct bqGauge_device_info *di;
    int retval = 0;
    int num;

    /* Get new ID for the new battery device */
//    retval = idr_alloc(&battery_id, client,0,0,GFP_KERNEL);
    retval = idr_pre_get(&battery_id,GFP_KERNEL);
    if(retval == 0)
        return -ENOMEM;
    mutex_lock(&battery_mutex);
    retval = idr_get_new(&battery_id,client,&num);
    mutex_unlock(&battery_mutex);
    if (retval < 0)
        return retval;

    name = kasprintf(GFP_KERNEL, "%s-%d", id->name, retval);
    if (!name) {
        dev_err(&client->dev, "failed to allocate device name\n");
        retval = -ENOMEM;
        goto batt_failed_1;
    }

    di = kzalloc(sizeof(*di), GFP_KERNEL);
    if (!di) {
        dev_err(&client->dev, "failed to allocate device info data\n");
        retval = -ENOMEM;
        goto batt_failed_2;
    }

    di->id = num;
    di->dev = &client->dev;
    di->chip = id->driver_data;
    di->bat.name = name;
    
    if (di->chip == BQ27520)
        di->gauge = &bqGauge_27520;
    else {
        dev_err(&client->dev,
            "Unexpected gas gague: %d\n", di->chip);
            di->gauge = NULL;    
    }


    if(di->gauge && di->gauge->read_fw_ver){
        di->fw_ver =  di->gauge->read_fw_ver(di);
    }
    else    
        di->fw_ver = 0x00;
        
    dev_info(&client->dev, "Gas Guage fw version is 0x%04x\n", di->fw_ver);

    retval = bqGauge_powersupply_init(di);
    if (retval)
        goto batt_failed_3;


    i2c_set_clientdata(client, di);
    
    if(di->gauge && di->gauge->updater)
        di->gauge->updater(di);

    /* Schedule a polling after about 1 min */
    schedule_delayed_work(&di->work, 60 * HZ);
        
    retval = sysfs_create_group(&client->dev.kobj, &bqGauge_attr_group);
    if (retval)
        dev_err(&client->dev, "could not create sysfs files\n");

    return 0;

batt_failed_3:
    kfree(di);
batt_failed_2:
    kfree(name);
batt_failed_1:
    mutex_lock(&battery_mutex);
    idr_remove(&battery_id, retval);
    mutex_unlock(&battery_mutex);

    return retval;
}

static int bqGauge_battery_remove(struct i2c_client *client)
{
    struct bqGauge_device_info *di = i2c_get_clientdata(client);

    bqGauge_powersupply_unregister(di);

    kfree(di->bat.name);

    mutex_lock(&battery_mutex);
    idr_remove(&battery_id, di->id);
    mutex_unlock(&battery_mutex);

    kfree(di);

    return 0;
}

static int bqGauge_battery_suspend(struct i2c_client *client,
                                   pm_message_t state)
{
    struct bqGauge_device_info *di = i2c_get_clientdata(client);
    cancel_delayed_work_sync(&di->work);
    return 0;
}

static int bqGauge_battery_resume(struct i2c_client *client)
{
    struct bqGauge_device_info *di = i2c_get_clientdata(client);
    schedule_delayed_work(&di->work, 0);
    return 0;
}


static const struct i2c_device_id bqGauge_id[] = {
    { "bq27520", BQ27520 },
    {},
};
MODULE_DEVICE_TABLE(i2c, bqGauge_id);

static struct i2c_driver bqGauge_battery_driver = {
    .driver = {
        .name = "bqGauge-battery",
    },
    .probe      = bqGauge_battery_probe,
    .suspend    = bqGauge_battery_suspend,
    .resume     = bqGauge_battery_resume,
    .remove     = bqGauge_battery_remove,
    .id_table = bqGauge_id,
};

static inline int  bqGauge_battery_i2c_init(void)
{
    int ret = i2c_add_driver(&bqGauge_battery_driver);
    if (ret)
        printk(KERN_ERR "Unable to register bqGauge i2c driver\n");

    return ret;
}

static inline void __exit bqGauge_battery_i2c_exit(void)
{
    i2c_del_driver(&bqGauge_battery_driver);
}

#else

static inline int bqGauge_battery_i2c_init(void) { return 0; }
static inline void bqGauge_battery_i2c_exit(void) {};

#endif


/*
 * Module stuff
 */

static int  bqGauge_battery_init(void)
{
    int ret;

    ret = bqGauge_battery_i2c_init();

    return ret;
}
module_init(bqGauge_battery_init);

static void bqGauge_battery_exit(void)
{
    bqGauge_battery_i2c_exit();
}
module_exit(bqGauge_battery_exit);


MODULE_AUTHOR("Rodolfo Giometti <giometti@linux.it>");
MODULE_DESCRIPTION("bqGauge battery monitor driver");
MODULE_LICENSE("GPL");
