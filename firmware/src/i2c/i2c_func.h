//=============================================================================
// Copyright (C) 2018-2019, Qorvo, Inc.
//
// THIS SOFTWARE IS SUBJECT TO A SOURCE CODE LICENSE AGREEMENT WHICH PROVIDES,
// AMONG OTHER THINGS:  (i) THAT IT CAN BE USED ONLY TO ADAPT THE LICENSEE'S
// APPLICATION TO PAC PROCESSORS SUPPLIED BY QORVO, INC.;
// (ii) THAT  IT IS PROVIDED "AS IS" WITHOUT WARRANTY;  (iii) THAT
// QORVO, INC. IS NOT LIABLE FOR ANY INDIRECT DAMAGES OR FOR DIRECT
// DAMAGES EXCEEDING US$1,500;  AND (iv) THAT IT CAN BE DISCLOSED TO AND USED
// ONLY BY CERTAIN AUTHORIZED PERSONS.
//
//=============================================================================

#include <src/common.h>

#ifndef I2C_FUNC_H
#define I2C_FUNC_H

#define DF_I2C_BUSY_TICK    (25000)
#define DF_I2C_RESET_VALUE  0x07
#define DF_I2C_ADDR_WRITE   0x00
#define DF_I2C_ADDR_READ    0x01

//=====================
// I2C State Defines
//=====================
#define DF_I2C_STAT_MASTER__BUS_ERROR                                                   0x00
#define DF_I2C_STAT_MASTER__FIRST_START_TRANSMITTED                                     0x08
#define DF_I2C_STAT_MASTER__REPEATED_START_TRANSMITTED                                  0x10
#define DF_I2C_STAT_MASTER__ADDR_AND_WRITE_BIT_TRANSMITTED_AND_ACK_RECEIVED             0x18
#define DF_I2C_STAT_MASTER__ADDR_AND_WRITE_BIT_TRANSMITTED_AND_NACK_RECEIVED            0x20
#define DF_I2C_STAT_MASTER__DATA_TRANSMITTED_AND_ACK_RECEIVED                           0x28
#define DF_I2C_STAT_MASTER__DATA_TRANSMITTED_AND_NACK_RECEIVED                          0x30
#define DF_I2C_STAT_MASTER__ADDR_AND_READ_BIT_TRANSMITTED_AND_ACK_RECEIVED              0x40
#define DF_I2C_STAT_MASTER__ADDR_AND_READ_BIT_TRANSMITTED_AND_NACK_RECEIVED             0x48
#define DF_I2C_STAT_MASTER__DATA_RECEIVED_AND_ACK_TRANSMITTED                           0x50
#define DF_I2C_STAT_MASTER__DATA_RECEIVED_AND_NACK_TRANSMITTED                          0x58

#define DF_I2C_STAT__ARBITRATION_LOST_IN_ADDR_OR_DATA                                                       0x38
#define DF_I2C_STAT__ARBITRATION_LOST_IN_MASTER_SLAVEADDR_AND_WRITEBIT_RECEIVED_AND_ACK_TRANSMITTED         0x68
#define DF_I2C_STAT__ARBITRATION_LOST_IN_MASTER_GENERAL_CALL_ADDR_RECEIVED_AND_ACK_TRANSMITTED              0x78
#define DF_I2C_STAT__ARBITRATION_LOST_IN_MASTER_SLAVEADDR_AND_READBIT_RECEIVED_AND_ACK_TRANSMITTED          0xB0

#define DF_I2C_STAT_SLAVE__ADDR_AND_WRITE_BIT_RECEIVED_AND_ACK_TRANSMITTED              0x60
#define DF_I2C_STAT_SLAVE__GENERAL_CALL_ADDR_RECEIVED_AND_ACK_TRANSMITTED               0x70
#define DF_I2C_STAT_SLAVE__DATA_AFTER_GENERAL_CALL_ADDR_RECEIVED_AND_ACK_TRANSMITTED    0x90
#define DF_I2C_STAT_SLAVE__DATA_AFTER_ADDR_RECEIVED_AND_ACK_TRANSMITTED                 0x80
#define DF_I2C_STAT_SLAVE__DATA_AFTER_ADDR_RECEIVED_AND_NACK_TRANSMITTED                0x88
#define DF_I2C_STAT_SLAVE__STOP_OR_RESTART_RECEIVED                                     0xA0
#define DF_I2C_STAT_SLAVE__ADDR_AND_READ_BIT_RECEIVED_AND_ACK_TRANSMITTED               0xA8
#define DF_I2C_STAT_SLAVE__DATA_TRANSMITTED_AND_ACK_RECEIVED                            0xB8
#define DF_I2C_STAT_SLAVE__DATA_TRANSMITTED_AND_NACK_RECEIVED                           0xC0
#define DF_I2C_STAT_SLAVE__LAST_DATA_TRANSMITTED_AND_ACK_RECEIVED                       0xC8
#define DF_I2C_STAT_SLAVE__LAST_DATA_TRANSMITTED_AND_NACK_RECEIVED                      0xD0
#define DF_I2C_STAT_SLAVE__SECOND_ADDR_BYTE_TRANSMITTED_AND_ACK_RECEIVED                0xE0
#define DF_I2C_STAT_SLAVE__SECOND_ADDR_BYTE_TRANSMITTED_AND_NACK_RECEIVED               0xE8

#define DF_I2C_STAT_NO_INFOMATION                                                       0xF8

// I2C Master/Slave Enumeration Type
typedef enum 
{
    I2C_MS_MASTER = 0,                      // The I2C controller acts as a bus master
    I2C_MS_SLAVE  = 1                       // The I2C controller acts as a bus slave
} I2C_MS_TYPE;

extern void i2c_init(I2C_MS_TYPE ms_mode, uint8_t slave_addr);
extern void i2c_reset(void);
extern uint32_t i2c_master_read_one(uint8_t device_addr, uint8_t *read_data);
extern uint32_t i2c_master_read_multi(uint8_t device_addr, uint8_t *read_data, uint32_t byte_num);
extern uint32_t i2c_master_write_one(uint8_t device_addr, uint8_t write_data);
extern uint32_t i2c_master_write_multi(uint8_t device_addr, uint8_t *write_data, uint32_t byte_num);
extern uint32_t i2c_master_read_one_standard(uint8_t device_addr, uint8_t data_addr, uint8_t *read_data);
extern uint32_t i2c_master_read_multi_standard(uint8_t device_addr, uint8_t data_addr, uint8_t *read_data, uint32_t byte_num);
extern uint32_t i2c_master_write_one_standard(uint8_t device_addr, uint8_t data_addr, uint8_t write_data);
extern uint32_t i2c_master_write_multi_standard(uint8_t device_addr, uint8_t data_addr, uint8_t *write_data, uint32_t byte_num);

#endif
