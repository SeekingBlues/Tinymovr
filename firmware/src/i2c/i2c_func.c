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
 
#include "i2c_func.h"

//==============================================================================
/// @brief
///     I2CSCL <--> PC0
///     I2CSDA <--> PC1
///
//==============================================================================
void i2c_io_select_pc01(I2C_MS_TYPE ms_mode)
{
    if (ms_mode == I2C_MS_MASTER)
    {
        PAC55XX_GPIOC->MODE.P0 = IO_PUSH_PULL_OUTPUT;           // SCL
        PAC55XX_GPIOC->MODE.P1 = IO_PUSH_PULL_OUTPUT;           // SDA
    }
    else
    {
        PAC55XX_GPIOC->MODE.P0 = IO_HIGH_IMPEDENCE_INPUT;       // SCL
        PAC55XX_GPIOC->MODE.P1 = IO_HIGH_IMPEDENCE_INPUT;       // SDA
    }

    PAC55XX_SCC->PCMUXSEL.w &= 0xFFFFFF00;                      // Clear Port Pin selection
    PAC55XX_SCC->PCMUXSEL.w |= 0x00000077;                      // Set Port Pin as I2C
}

//==============================================================================
/// @brief
///     I2CSCL <--> PC4
///     I2CSDA <--> PC5
///
//==============================================================================
void i2c_io_select_pc45(I2C_MS_TYPE ms_mode)
{
    if (ms_mode == I2C_MS_MASTER)
    {
        PAC55XX_GPIOC->MODE.P4 = IO_PUSH_PULL_OUTPUT;           // SCL
        PAC55XX_GPIOC->MODE.P5 = IO_PUSH_PULL_OUTPUT;           // SDA
    }
    else
    {
        PAC55XX_GPIOC->MODE.P4 = IO_HIGH_IMPEDENCE_INPUT;       // SCL
        PAC55XX_GPIOC->MODE.P5 = IO_HIGH_IMPEDENCE_INPUT;       // SDA
    }

    PAC55XX_SCC->PCMUXSEL.w &= 0xFF00FFFF;                      // Clear Port Pin selection
    PAC55XX_SCC->PCMUXSEL.w |= 0x00770000;                      // Set Port Pin as I2C
}

//==============================================================================
/// @brief
///     I2CSDA <--> PD6
///     I2CSCL <--> PD7
///
//==============================================================================
void i2c_io_select_pd67(I2C_MS_TYPE ms_mode)
{
    if (ms_mode == I2C_MS_MASTER)
    {
        PAC55XX_GPIOD->MODE.P6 = IO_PUSH_PULL_OUTPUT;           // SDA
        PAC55XX_GPIOD->MODE.P7 = IO_PUSH_PULL_OUTPUT;           // SCL
    }
    else
    {
        PAC55XX_GPIOD->MODE.P6 = IO_HIGH_IMPEDENCE_INPUT;       // SDA
        PAC55XX_GPIOD->MODE.P7 = IO_HIGH_IMPEDENCE_INPUT;       // SCL
    }

    PAC55XX_SCC->PDMUXSEL.w &= 0x00FFFFFF;                      // Clear Port Pin selection
    PAC55XX_SCC->PDMUXSEL.w |= 0x77000000;                      // Set Port Pin as I2C
}

//==============================================================================
/// @brief
///     I2CSCL <--> PE0
///     I2CSDA <--> PE1
///
//==============================================================================
void i2c_io_select_pe01(I2C_MS_TYPE ms_mode)
{
    if (ms_mode == I2C_MS_MASTER)
    {
        PAC55XX_GPIOE->MODE.P0 = IO_PUSH_PULL_OUTPUT;           // SCL
        PAC55XX_GPIOE->MODE.P1 = IO_PUSH_PULL_OUTPUT;           // SDA
    }
    else
    {
        PAC55XX_GPIOE->MODE.P0 = IO_HIGH_IMPEDENCE_INPUT;       // SCL
        PAC55XX_GPIOE->MODE.P1 = IO_HIGH_IMPEDENCE_INPUT;       // SDA
    }

    PAC55XX_SCC->PEMUXSEL.w &= 0xFFFFFF00;                      // Clear Port Pin selection
    PAC55XX_SCC->PEMUXSEL.w |= 0x00000066;                      // Set Port Pin as I2C
}

//==============================================================================
/// @brief
///     I2CSCL <--> PE4
///     I2CSDA <--> PE5
///
//==============================================================================
void i2c_io_select_pe45(I2C_MS_TYPE ms_mode)
{
    if (ms_mode == I2C_MS_MASTER)
    {
        PAC55XX_GPIOE->MODE.P4 = IO_PUSH_PULL_OUTPUT;           // SCL
        PAC55XX_GPIOE->MODE.P5 = IO_PUSH_PULL_OUTPUT;           // SDA
    }
    else
    {
        PAC55XX_GPIOE->MODE.P4 = IO_HIGH_IMPEDENCE_INPUT;       // SCL
        PAC55XX_GPIOE->MODE.P5 = IO_HIGH_IMPEDENCE_INPUT;       // SDA
    }

    PAC55XX_SCC->PEMUXSEL.w &= 0xFF00FFFF;                      // Clear Port Pin selection
    PAC55XX_SCC->PEMUXSEL.w |= 0x00660000;                      // Set Port Pin as I2C
}

//==============================================================================
/// @brief
///     I2CSCL <--> PF6
///     I2CSDA <--> PF7
///
//==============================================================================
void i2c_io_select_pf67(I2C_MS_TYPE ms_mode)
{
    if (ms_mode == I2C_MS_MASTER)
    {
        PAC55XX_GPIOF->MODE.P6 = IO_PUSH_PULL_OUTPUT;           // SCL
        PAC55XX_GPIOF->MODE.P7 = IO_PUSH_PULL_OUTPUT;           // SDA
    }
    else
    {
        PAC55XX_GPIOF->MODE.P6 = IO_HIGH_IMPEDENCE_INPUT;       // SCL
        PAC55XX_GPIOF->MODE.P7 = IO_HIGH_IMPEDENCE_INPUT;       // SDA
    }

    PAC55XX_SCC->PFMUXSEL.w &= 0x00FFFFFF;                      // Clear Port Pin selection
    PAC55XX_SCC->PFMUXSEL.w |= 0x77000000;                      // Set Port Pin as I2C
}

//==============================================================================
/// @brief
///     I2CSCL <--> PG4
///     I2CSDA <--> PG6
///
//==============================================================================
void i2c_io_select_pg46(I2C_MS_TYPE ms_mode)
{
    if (ms_mode == I2C_MS_MASTER)
    {
        PAC55XX_GPIOG->MODE.P4 = IO_PUSH_PULL_OUTPUT;           // SCL
        PAC55XX_GPIOG->MODE.P6 = IO_PUSH_PULL_OUTPUT;           // SDA
    }
    else
    {
        PAC55XX_GPIOG->MODE.P4 = IO_HIGH_IMPEDENCE_INPUT;       // SCL
        PAC55XX_GPIOG->MODE.P6 = IO_HIGH_IMPEDENCE_INPUT;       // SDA
    }

    PAC55XX_SCC->PFMUXSEL.w &= 0xF0F0FFFF;                      // Clear Port Pin selection
    PAC55XX_SCC->PFMUXSEL.w |= 0x03040000;                      // Set Port Pin as I2C
}

//==============================================================================
/// @brief
///     Select the I2C IO setting (choose one)
///
//==============================================================================
void i2c_io_config(I2C_MS_TYPE ms_mode)
{
    // Pick one
//    i2c_io_select_pc01(ms_mode);
//    i2c_io_select_pc45(ms_mode);
//    i2c_io_select_pd67(ms_mode);
//    i2c_io_select_pe01(ms_mode);
//    i2c_io_select_pe45(ms_mode);
    i2c_io_select_pf67(ms_mode);
//    i2c_io_select_pg46(ms_mode);
}

//==============================================================================
/// @brief
///     enable the interrupt
///
//==============================================================================
void i2c_interrupt_enable(void)
{
    NVIC_ClearPendingIRQ(I2C_IRQn);
    NVIC_SetPriority(I2C_IRQn, 3);
    NVIC_EnableIRQ(I2C_IRQn);
}

//==============================================================================
/// @brief
///     init the I2C port.
///         including Master/Slave mode; Slave address; Frequency and IO port.
///
///@param[in]
///     I2C_MS_TYPE:
///         I2C_MS_MASTER
///         I2C_MS_SLAVE
///     slave_addr:
///         The 7-bit address when config as slave.
///
//==============================================================================
void i2c_init(I2C_MS_TYPE ms_mode, uint8_t slave_addr)
{
    // Config the IO
    i2c_io_config(ms_mode);

    // Clock setting
    PAC55XX_I2C->I2CCLK.M = 5;                                  // Fsamp = PCLK / (2 ^ M) = 150M / (2^5) = 4.6875MHz
    PAC55XX_I2C->I2CCLK.N = 4;                                  // Fscl = Fsamp / (N + 1) / 10 = Fsamp/(4+1)/10 = 93.75KHz

    if (ms_mode == I2C_MS_SLAVE)
    {
        // Config the address
        PAC55XX_I2C->I2CADR0.ADDRESS = slave_addr;
        PAC55XX_I2C->I2CADRM0.MASK = 0x7F;

        PAC55XX_I2C->I2CCONSET.I2CIE = 1;
        PAC55XX_I2C->I2CCONSET.AA = 1;                          // Send ACK

        i2c_interrupt_enable();
    }

    // Enable the I2C
    PAC55XX_I2C->I2CCONSET.I2CEN = 1;
}

//==============================================================================
/// @brief
///     just delay some count
///
///@param[in]
///     delay_count: the delay counts
///
///@param[out]
///     None
///
//==============================================================================
void I2C_delay(uint32_t delay_count)
{
    uint32_t delay_count_temp;

    for (delay_count_temp=0; delay_count_temp<delay_count; delay_count_temp++)
    {
        __NOP();
    }
}

//==============================================================================
/// @brief
///     reset the I2C
///
//==============================================================================
void i2c_reset(void)
{
    PAC55XX_I2C->I2CRST.RST = DF_I2C_RESET_VALUE;
}

//==============================================================================
/// @brief
///     read a byte of target address from I2C manually
///
///@param[in]
///     device_addr:
///         the device address
///     read_data:
///         The data read back.
///
///@param[out]
///     result: the result of the UART byte read
///
///@retval
///     0: All is OK.
///     others: Some error occurs.
///
//==============================================================================
uint32_t i2c_master_read_one(uint8_t device_addr, uint8_t *read_data)
{
    uint32_t result = PAC5XXX_OK;
    uint32_t wait_tick = 0;

    // Write a start
    PAC55XX_I2C->I2CCONSET.I2CIE = 1;
    PAC55XX_I2C->I2CCONSET.STA = 1;                             // Start
    while (PAC55XX_I2C->I2CSTAT.w != DF_I2C_STAT_MASTER__FIRST_START_TRANSMITTED)
    {
        wait_tick++;
        if (wait_tick > DF_I2C_BUSY_TICK)
        {
            result = PAC5XXX_ERROR;
            break;
        }
    }

    wait_tick = 0;
    PAC55XX_I2C->I2CCONCLR.SIC = 1;

    // Write a addr with read
    PAC55XX_I2C->I2CCONSET.I2CIE = 1;
    PAC55XX_I2C->I2CDAT.DATA = ((device_addr << 1) | DF_I2C_ADDR_READ);       // Send address, Read command
    while (PAC55XX_I2C->I2CSTAT.w != DF_I2C_STAT_MASTER__ADDR_AND_READ_BIT_TRANSMITTED_AND_ACK_RECEIVED)
    {
        wait_tick++;
        if (wait_tick > DF_I2C_BUSY_TICK)
        {
            result = PAC5XXX_ERROR;
            break;
        }

        if (PAC55XX_I2C->I2CSTAT.w == DF_I2C_STAT_MASTER__ADDR_AND_READ_BIT_TRANSMITTED_AND_NACK_RECEIVED)
        {
            result = PAC5XXX_ERROR;
            PAC55XX_I2C->I2CCONSET.STO = 1;                     // Stop
            PAC55XX_I2C->I2CCONCLR.SIC = 1;
            return result;
        }
    }

    wait_tick = 0;
    PAC55XX_I2C->I2CCONCLR.SIC = 1;

    PAC55XX_I2C->I2CCONCLR.AAC = 1;                             // Send ACK to end READ
    PAC55XX_I2C->I2CCONSET.I2CIE = 1;
    while (PAC55XX_I2C->I2CSTAT.w != DF_I2C_STAT_MASTER__DATA_RECEIVED_AND_NACK_TRANSMITTED)
    {
        wait_tick++;
        if (wait_tick > DF_I2C_BUSY_TICK)
        {
            result = PAC5XXX_ERROR;
            PAC55XX_I2C->I2CCONSET.STO = 1;                     // Stop
            PAC55XX_I2C->I2CCONCLR.SIC = 1;
            return result;
        }
    }

    *read_data = PAC55XX_I2C->I2CDAT.DATA;                      // Read data

    PAC55XX_I2C->I2CCONSET.STO = 1;                             // Stop
    PAC55XX_I2C->I2CCONCLR.SIC = 1;

    return result;
}

//==============================================================================
/// @brief
///     read serial bytes of target address from I2C manually
///
///@param[in]
///     device_addr:
///         the device address
///     read_data:
///         The data read back.
///     byte_num:
///         the number to read.
///
///@param[out]
///     result: the result of the UART byte read
///
///@retval
///     0: All is OK.
///     others: Some error occurs.
///
//==============================================================================
uint32_t i2c_master_read_multi(uint8_t device_addr, uint8_t *read_data, uint32_t byte_num)
{
    uint32_t result = PAC5XXX_OK;
    uint32_t wait_tick = 0;
    uint32_t byte_num_temp;

    // Write a start
    PAC55XX_I2C->I2CCONSET.I2CIE = 1;
    PAC55XX_I2C->I2CCONSET.STA = 1;                             // Start
    while (PAC55XX_I2C->I2CSTAT.w != DF_I2C_STAT_MASTER__FIRST_START_TRANSMITTED)
    {
        wait_tick++;
        if (wait_tick > DF_I2C_BUSY_TICK)
        {
            result = PAC5XXX_ERROR;
            break;
        }
    }

    wait_tick = 0;
    PAC55XX_I2C->I2CCONCLR.SIC = 1;

    // Write a addr with write
    PAC55XX_I2C->I2CCONSET.I2CIE = 1;
    PAC55XX_I2C->I2CDAT.DATA = ((device_addr << 1) | DF_I2C_ADDR_READ);       // Send address, Write command
    while (PAC55XX_I2C->I2CSTAT.w != DF_I2C_STAT_MASTER__ADDR_AND_READ_BIT_TRANSMITTED_AND_ACK_RECEIVED)
    {
        wait_tick++;
        if (wait_tick > DF_I2C_BUSY_TICK)
        {
            result = PAC5XXX_ERROR;
            break;
        }

        if (PAC55XX_I2C->I2CSTAT.w == DF_I2C_STAT_MASTER__ADDR_AND_READ_BIT_TRANSMITTED_AND_NACK_RECEIVED)
        {
            result = PAC5XXX_ERROR;
            PAC55XX_I2C->I2CCONSET.STO = 1;                     // Stop
            PAC55XX_I2C->I2CCONCLR.SIC = 1;
            return result;
        }
    }

    wait_tick = 0;
    PAC55XX_I2C->I2CCONCLR.SIC = 1;

    for (byte_num_temp=0; byte_num_temp<byte_num-1; byte_num_temp++)
    {
        PAC55XX_I2C->I2CCONSET.AA = 1;                          // Send ACK to end READ
        PAC55XX_I2C->I2CCONSET.I2CIE = 1;
        while (PAC55XX_I2C->I2CSTAT.w != DF_I2C_STAT_MASTER__DATA_RECEIVED_AND_ACK_TRANSMITTED)
        {
            wait_tick++;
            if (wait_tick > DF_I2C_BUSY_TICK)
            {
                result = PAC5XXX_ERROR;
                PAC55XX_I2C->I2CCONSET.STO = 1;                 // Stop
                PAC55XX_I2C->I2CCONCLR.SIC = 1;
                return result;
            }
        }

        wait_tick = 0;
        *read_data++ = PAC55XX_I2C->I2CDAT.DATA;                // Read data

        PAC55XX_I2C->I2CCONCLR.SIC = 1;
    }

    // Final byte to read
    PAC55XX_I2C->I2CCONCLR.AAC = 1;                             // Send NACK to end READ
    PAC55XX_I2C->I2CCONSET.I2CIE = 1;
    while (PAC55XX_I2C->I2CSTAT.w != DF_I2C_STAT_MASTER__DATA_RECEIVED_AND_NACK_TRANSMITTED)
    {
        wait_tick++;
        if (wait_tick > DF_I2C_BUSY_TICK)
        {
            result = PAC5XXX_ERROR;
            PAC55XX_I2C->I2CCONSET.STO = 1;
            PAC55XX_I2C->I2CCONCLR.SIC = 1;
            return result;
        }
    }

    *read_data++ = PAC55XX_I2C->I2CDAT.DATA;                    // Read data

    PAC55XX_I2C->I2CCONSET.STO = 1;                             // Stop
    PAC55XX_I2C->I2CCONCLR.SIC = 1;

    return result;
}

//==============================================================================
/// @brief
///     Write a 8-bit data to a setting target address by I2C as a master
///
///@param[in]
///     device_addr:
///         the device address
///     write_data:
///         The data to write.
///
///@param[out]
///     result: the result of the I2C write
///
///@retval
///     0: All is OK.
///     others: Some error occurs.
///
//==============================================================================
uint32_t i2c_master_write_one(uint8_t device_addr, uint8_t write_data)
{
    uint32_t result = PAC5XXX_OK;
    uint32_t wait_tick = 0;

    // Write a start
    PAC55XX_I2C->I2CCONSET.I2CIE = 1;
    PAC55XX_I2C->I2CCONSET.STA = 1;                             // Start
    while (PAC55XX_I2C->I2CSTAT.w != DF_I2C_STAT_MASTER__FIRST_START_TRANSMITTED)
    {
        wait_tick++;
        if (wait_tick > DF_I2C_BUSY_TICK)
        {
            result = PAC5XXX_ERROR;
            break;
        }
    }

    wait_tick = 0;
    PAC55XX_I2C->I2CCONCLR.SIC = 1;

    // Write a addr with write
    PAC55XX_I2C->I2CCONSET.I2CIE = 1;
    PAC55XX_I2C->I2CDAT.DATA = ((device_addr << 1) | DF_I2C_ADDR_WRITE);      // Send address, Write command
    while (PAC55XX_I2C->I2CSTAT.w != DF_I2C_STAT_MASTER__ADDR_AND_WRITE_BIT_TRANSMITTED_AND_ACK_RECEIVED)
    {
        wait_tick++;
        if (wait_tick > DF_I2C_BUSY_TICK)
        {
            result = PAC5XXX_ERROR;
            break;
        }

        if (PAC55XX_I2C->I2CSTAT.w == DF_I2C_STAT_MASTER__ADDR_AND_WRITE_BIT_TRANSMITTED_AND_NACK_RECEIVED)
        {
            result = PAC5XXX_ERROR;
            PAC55XX_I2C->I2CCONSET.STO = 1;                     // Stop
            PAC55XX_I2C->I2CCONCLR.SIC = 1;
            return result;
        }
    }

    wait_tick = 0;
    PAC55XX_I2C->I2CCONCLR.SIC = 1;

    // Write a data
    PAC55XX_I2C->I2CCONSET.I2CIE = 1;
    PAC55XX_I2C->I2CDAT.DATA = write_data;                      // Send address, Write command
    while (PAC55XX_I2C->I2CSTAT.w != DF_I2C_STAT_MASTER__DATA_TRANSMITTED_AND_ACK_RECEIVED)
    {
        wait_tick++;
        if (wait_tick > DF_I2C_BUSY_TICK)
        {
            result = PAC5XXX_ERROR;
            break;
        }

        if (PAC55XX_I2C->I2CSTAT.w == DF_I2C_STAT_MASTER__DATA_TRANSMITTED_AND_NACK_RECEIVED)
        {
            result = PAC5XXX_ERROR;
            PAC55XX_I2C->I2CCONSET.STO = 1;                     // Stop
            PAC55XX_I2C->I2CCONCLR.SIC = 1;
            return result;
        }
    }

    wait_tick = 0;
    PAC55XX_I2C->I2CCONSET.STO = 1;                             // Stop
    PAC55XX_I2C->I2CCONCLR.SIC = 1;

    return result;
}

//==============================================================================
/// @brief
///     Write a serial bytes to the device address by I2C as a master
///
///@param[in]
///     device_addr:
///         the device address
///     write_data: The data point to write.
///     byte_num: the number to write.
///
///@param[out]
///     result: the result of the I2C write
///
///@retval
///     0: All is OK.
///     others: Some error occurs.
///
//==============================================================================
uint32_t i2c_master_write_multi(uint8_t device_addr, uint8_t *write_data, uint32_t byte_num)
{
    uint32_t result = PAC5XXX_OK;
    uint32_t wait_tick = 0;
    uint32_t byte_num_temp;

    // Write a start
    PAC55XX_I2C->I2CCONSET.I2CIE = 1;
    PAC55XX_I2C->I2CCONSET.STA = 1;                             // Start
    while (PAC55XX_I2C->I2CSTAT.w != DF_I2C_STAT_MASTER__FIRST_START_TRANSMITTED)
    {
        wait_tick++;
        if (wait_tick > DF_I2C_BUSY_TICK)
        {
            result = PAC5XXX_ERROR;
            break;
        }
    }

    wait_tick = 0;
    PAC55XX_I2C->I2CCONCLR.SIC = 1;

    // Write a addr with write
    PAC55XX_I2C->I2CCONSET.I2CIE = 1;
    PAC55XX_I2C->I2CDAT.DATA = ((device_addr << 1) | DF_I2C_ADDR_WRITE);      // Send address, Write command
    while (PAC55XX_I2C->I2CSTAT.w != DF_I2C_STAT_MASTER__ADDR_AND_WRITE_BIT_TRANSMITTED_AND_ACK_RECEIVED)
    {
        wait_tick++;
        if (wait_tick > DF_I2C_BUSY_TICK)
        {
            result = PAC5XXX_ERROR;
            break;
        }

        if (PAC55XX_I2C->I2CSTAT.w == DF_I2C_STAT_MASTER__ADDR_AND_WRITE_BIT_TRANSMITTED_AND_NACK_RECEIVED)
        {
            result = PAC5XXX_ERROR;
            PAC55XX_I2C->I2CCONSET.STO = 1;                     // Stop
            PAC55XX_I2C->I2CCONCLR.SIC = 1;
            return result;
        }
    }

    wait_tick = 0;
    PAC55XX_I2C->I2CCONCLR.SIC = 1;

    for (byte_num_temp=0; byte_num_temp<byte_num; byte_num_temp++)
    {
        // Write a data
        PAC55XX_I2C->I2CCONSET.I2CIE = 1;
        PAC55XX_I2C->I2CDAT.DATA = *write_data++;               // Send address, Write command
        while (PAC55XX_I2C->I2CSTAT.w != DF_I2C_STAT_MASTER__DATA_TRANSMITTED_AND_ACK_RECEIVED)
        {
            wait_tick++;
            if (wait_tick > DF_I2C_BUSY_TICK)
            {
                result = PAC5XXX_ERROR;
                break;
            }

            if (PAC55XX_I2C->I2CSTAT.w == DF_I2C_STAT_MASTER__DATA_TRANSMITTED_AND_NACK_RECEIVED)
            {
                result = PAC5XXX_ERROR;
                PAC55XX_I2C->I2CCONSET.STO = 1;                 // Stop
                PAC55XX_I2C->I2CCONCLR.SIC = 1;
                return result;
            }
        }

        wait_tick = 0;

        if (byte_num_temp == byte_num - 1)
            PAC55XX_I2C->I2CCONSET.STO = 1;                     // Stop

        PAC55XX_I2C->I2CCONCLR.SIC = 1;
    }

    return result;
}

////////////////////////////////////////////
// ========== Standard Functions ==========
////////////////////////////////////////////
//==============================================================================
/// @brief
///     read a byte of target address from I2C manually
///
///@param[in]
///     device_addr:
///         the device address
///     data_addr:
///         the data address
///     read_data:
///         The data read back.
///
///@param[out]
///     result: the result of the UART byte read
///
///@retval
///     0: All is OK.
///     others: Some error occurs.
///
//==============================================================================
uint32_t i2c_master_read_one_standard(uint8_t device_addr, uint8_t data_addr, uint8_t *read_data)
{
    uint32_t result = PAC5XXX_OK;
    uint32_t wait_tick = 0;

    // Write a start
    PAC55XX_I2C->I2CCONSET.I2CIE = 1;
    PAC55XX_I2C->I2CCONSET.STA = 1;                             // Start
    while (PAC55XX_I2C->I2CSTAT.w != DF_I2C_STAT_MASTER__FIRST_START_TRANSMITTED)
    {
        wait_tick++;
        if (wait_tick > DF_I2C_BUSY_TICK)
        {
            result = PAC5XXX_ERROR;
            break;
        }
    }

    wait_tick = 0;
    PAC55XX_I2C->I2CCONCLR.SIC = 1;

    // Write a addr with write
    PAC55XX_I2C->I2CCONSET.I2CIE = 1;
    PAC55XX_I2C->I2CDAT.DATA = ((device_addr << 1) | DF_I2C_ADDR_WRITE);      // Send address, Write command
    while (PAC55XX_I2C->I2CSTAT.w != DF_I2C_STAT_MASTER__ADDR_AND_WRITE_BIT_TRANSMITTED_AND_ACK_RECEIVED)
    {
        wait_tick++;
        if (wait_tick > DF_I2C_BUSY_TICK)
        {
            result = PAC5XXX_ERROR;
            break;
        }

        if (PAC55XX_I2C->I2CSTAT.w == DF_I2C_STAT_MASTER__ADDR_AND_WRITE_BIT_TRANSMITTED_AND_NACK_RECEIVED)
        {
            result = PAC5XXX_ERROR;
            PAC55XX_I2C->I2CCONSET.STO = 1;                     // Stop
            PAC55XX_I2C->I2CCONCLR.SIC = 1;
            return result;
        }
    }

    wait_tick = 0;
    PAC55XX_I2C->I2CCONCLR.SIC = 1;

    // Write a data
    PAC55XX_I2C->I2CCONSET.I2CIE = 1;
    PAC55XX_I2C->I2CDAT.DATA = data_addr;                       // Send address, Write command
    while (PAC55XX_I2C->I2CSTAT.w != DF_I2C_STAT_MASTER__DATA_TRANSMITTED_AND_ACK_RECEIVED)
    {
        wait_tick++;
        if (wait_tick > DF_I2C_BUSY_TICK)
        {
            result = PAC5XXX_ERROR;
            break;
        }

        if (PAC55XX_I2C->I2CSTAT.w == DF_I2C_STAT_MASTER__DATA_TRANSMITTED_AND_NACK_RECEIVED)
        {
            result = PAC5XXX_ERROR;
            PAC55XX_I2C->I2CCONSET.STO = 1;                     // Stop
            PAC55XX_I2C->I2CCONCLR.SIC = 1;
            return result;
        }
    }

    wait_tick = 0;
    PAC55XX_I2C->I2CCONCLR.SIC = 1;

    // ReWrite a start
    PAC55XX_I2C->I2CCONSET.I2CIE = 1;
    PAC55XX_I2C->I2CCONSET.STA = 1;                             // Start
    while (PAC55XX_I2C->I2CSTAT.w != DF_I2C_STAT_MASTER__REPEATED_START_TRANSMITTED)
    {
        wait_tick++;
        if (wait_tick > DF_I2C_BUSY_TICK)
        {
            result = PAC5XXX_ERROR;
            break;
        }
    }

    wait_tick = 0;
    PAC55XX_I2C->I2CCONCLR.SIC = 1;

    // Write a addr with read
    PAC55XX_I2C->I2CCONSET.I2CIE = 1;
    PAC55XX_I2C->I2CDAT.DATA = ((device_addr << 1) | DF_I2C_ADDR_READ);       // Send address, Read command
    while (PAC55XX_I2C->I2CSTAT.w != DF_I2C_STAT_MASTER__ADDR_AND_READ_BIT_TRANSMITTED_AND_ACK_RECEIVED)
    {
        wait_tick++;
        if (wait_tick > DF_I2C_BUSY_TICK)
        {
            result = PAC5XXX_ERROR;
            break;
        }

        if (PAC55XX_I2C->I2CSTAT.w == DF_I2C_STAT_MASTER__ADDR_AND_READ_BIT_TRANSMITTED_AND_NACK_RECEIVED)
        {
            result = PAC5XXX_ERROR;
            PAC55XX_I2C->I2CCONSET.STO = 1;                     // Stop
            PAC55XX_I2C->I2CCONCLR.SIC = 1;
            return result;
        }
    }

    wait_tick = 0;
    PAC55XX_I2C->I2CCONCLR.SIC = 1;

    PAC55XX_I2C->I2CCONCLR.AAC = 1;                             // Send ACK to end READ
    PAC55XX_I2C->I2CCONSET.I2CIE = 1;
    while (PAC55XX_I2C->I2CSTAT.w != DF_I2C_STAT_MASTER__DATA_RECEIVED_AND_NACK_TRANSMITTED)
    {
        wait_tick++;
        if (wait_tick > DF_I2C_BUSY_TICK)
        {
            result = PAC5XXX_ERROR;
            PAC55XX_I2C->I2CCONSET.STO = 1;                     // Stop
            PAC55XX_I2C->I2CCONCLR.SIC = 1;
            return result;
        }
    }

    *read_data = PAC55XX_I2C->I2CDAT.DATA;                      // Read data

    PAC55XX_I2C->I2CCONSET.STO = 1;                             // Stop
    PAC55XX_I2C->I2CCONCLR.SIC = 1;

    return result;
}

//==============================================================================
/// @brief
///     read serial bytes of target address from I2C manually
///
///@param[in]
///     device_addr:
///         the device address
///     data_addr:
///         the data address
///     read_data:
///         The data read back.
///     byte_num:
///         the number to read.
///
///@param[out]
///     result: the result of the UART byte read
///
///@retval
///     0: All is OK.
///     others: Some error occurs.
///
//==============================================================================
uint32_t i2c_master_read_multi_standard(uint8_t device_addr, uint8_t data_addr, uint8_t *read_data, uint32_t byte_num)
{
    uint32_t result = PAC5XXX_OK;
    uint32_t wait_tick = 0;
    uint32_t byte_num_temp;

    // Write a start
    PAC55XX_I2C->I2CCONSET.I2CIE = 1;
    PAC55XX_I2C->I2CCONSET.STA = 1;                             // Start
    while (PAC55XX_I2C->I2CSTAT.w != DF_I2C_STAT_MASTER__FIRST_START_TRANSMITTED)
    {
        wait_tick++;
        if (wait_tick > DF_I2C_BUSY_TICK)
        {
            result = PAC5XXX_ERROR;
            break;
        }
    }

    wait_tick = 0;
    PAC55XX_I2C->I2CCONCLR.SIC = 1;

    // Write a addr with write
    PAC55XX_I2C->I2CCONSET.I2CIE = 1;
    PAC55XX_I2C->I2CDAT.DATA = ((device_addr << 1) | DF_I2C_ADDR_WRITE);      // Send address, Write command
    while (PAC55XX_I2C->I2CSTAT.w != DF_I2C_STAT_MASTER__ADDR_AND_WRITE_BIT_TRANSMITTED_AND_ACK_RECEIVED)
    {
        wait_tick++;
        if (wait_tick > DF_I2C_BUSY_TICK)
        {
            result = PAC5XXX_ERROR;
            break;
        }

        if (PAC55XX_I2C->I2CSTAT.w == DF_I2C_STAT_MASTER__ADDR_AND_WRITE_BIT_TRANSMITTED_AND_NACK_RECEIVED)
        {
            result = PAC5XXX_ERROR;
            PAC55XX_I2C->I2CCONSET.STO = 1;                     // Stop
            PAC55XX_I2C->I2CCONCLR.SIC = 1;
            return result;
        }
    }

    wait_tick = 0;
    PAC55XX_I2C->I2CCONCLR.SIC = 1;

    // Write a data
    PAC55XX_I2C->I2CCONSET.I2CIE = 1;
    PAC55XX_I2C->I2CDAT.DATA = data_addr;                       // Send address, Write command
    while (PAC55XX_I2C->I2CSTAT.w != DF_I2C_STAT_MASTER__DATA_TRANSMITTED_AND_ACK_RECEIVED)
    {
        wait_tick++;
        if (wait_tick > DF_I2C_BUSY_TICK)
        {
            result = PAC5XXX_ERROR;
            break;
        }

        if (PAC55XX_I2C->I2CSTAT.w == DF_I2C_STAT_MASTER__DATA_TRANSMITTED_AND_NACK_RECEIVED)
        {
            result = PAC5XXX_ERROR;
            PAC55XX_I2C->I2CCONSET.STO = 1;                     // Stop
            PAC55XX_I2C->I2CCONCLR.SIC = 1;
            return result;
        }
    }

    wait_tick = 0;
    PAC55XX_I2C->I2CCONCLR.SIC = 1;

    // ReWrite a start
    PAC55XX_I2C->I2CCONSET.I2CIE = 1;
    PAC55XX_I2C->I2CCONSET.STA = 1;                             // Start
    while (PAC55XX_I2C->I2CSTAT.w != DF_I2C_STAT_MASTER__REPEATED_START_TRANSMITTED)
    {
        wait_tick++;
        if (wait_tick > DF_I2C_BUSY_TICK)
        {
            result = PAC5XXX_ERROR;
            break;
        }
    }

    wait_tick = 0;
    PAC55XX_I2C->I2CCONCLR.SIC = 1;

    // Write a addr with read
    PAC55XX_I2C->I2CCONSET.I2CIE = 1;
    PAC55XX_I2C->I2CDAT.DATA = ((device_addr << 1) | DF_I2C_ADDR_READ);       // Send address, Read command
    while (PAC55XX_I2C->I2CSTAT.w != DF_I2C_STAT_MASTER__ADDR_AND_READ_BIT_TRANSMITTED_AND_ACK_RECEIVED)
    {
        wait_tick++;
        if (wait_tick > DF_I2C_BUSY_TICK)
        {
            result = PAC5XXX_ERROR;
            break;
        }

        if (PAC55XX_I2C->I2CSTAT.w == DF_I2C_STAT_MASTER__ADDR_AND_READ_BIT_TRANSMITTED_AND_NACK_RECEIVED)
        {
            result = PAC5XXX_ERROR;
            PAC55XX_I2C->I2CCONSET.STO = 1;                     // Stop
            PAC55XX_I2C->I2CCONCLR.SIC = 1;
            return result;
        }
    }

    wait_tick = 0;
    PAC55XX_I2C->I2CCONCLR.SIC = 1;

    for (byte_num_temp=0; byte_num_temp<byte_num-1; byte_num_temp++)
    {
        PAC55XX_I2C->I2CCONSET.AA = 1;                          // Send ACK to end READ
        PAC55XX_I2C->I2CCONSET.I2CIE = 1;
        while (PAC55XX_I2C->I2CSTAT.w != DF_I2C_STAT_MASTER__DATA_RECEIVED_AND_ACK_TRANSMITTED)
        {
            wait_tick++;
            if (wait_tick > DF_I2C_BUSY_TICK)
            {
                result = PAC5XXX_ERROR;
                PAC55XX_I2C->I2CCONSET.STO = 1;                 // Stop
                PAC55XX_I2C->I2CCONCLR.SIC = 1;
                return result;
            }
        }

        wait_tick = 0;
        *read_data++ = PAC55XX_I2C->I2CDAT.DATA;                // Read data

        PAC55XX_I2C->I2CCONCLR.SIC = 1;
    }

    // Final byte to read
    PAC55XX_I2C->I2CCONCLR.AAC = 1;                             // Send NACK to end READ
    PAC55XX_I2C->I2CCONSET.I2CIE = 1;
    while (PAC55XX_I2C->I2CSTAT.w != DF_I2C_STAT_MASTER__DATA_RECEIVED_AND_NACK_TRANSMITTED)
    {
        wait_tick++;
        if (wait_tick > DF_I2C_BUSY_TICK)
        {
            result = PAC5XXX_ERROR;
            PAC55XX_I2C->I2CCONSET.STO = 1;
            PAC55XX_I2C->I2CCONCLR.SIC = 1;
            return result;
        }
    }

    *read_data++ = PAC55XX_I2C->I2CDAT.DATA;                    // Read data

    PAC55XX_I2C->I2CCONSET.STO = 1;                             // Stop
    PAC55XX_I2C->I2CCONCLR.SIC = 1;

    return result;
}

//==============================================================================
/// @brief
///     Write a 8-bit data to a setting target address by I2C as a master
///
///@param[in]
///     device_addr:
///         the device address
///     data_addr:
///         the data address
///     write_data:
///         The data to write.
///
///@param[out]
///     result: the result of the I2C write
///
///@retval
///     0: All is OK.
///     others: Some error occurs.
///
//==============================================================================
uint32_t i2c_master_write_one_standard(uint8_t device_addr, uint8_t data_addr, uint8_t write_data)
{
    uint32_t result = PAC5XXX_OK;
    uint32_t wait_tick = 0;

    // Write a start
    PAC55XX_I2C->I2CCONSET.I2CIE = 1;
    PAC55XX_I2C->I2CCONSET.STA = 1;                             // Start
    while (PAC55XX_I2C->I2CSTAT.w != DF_I2C_STAT_MASTER__FIRST_START_TRANSMITTED)
    {
        wait_tick++;
        if (wait_tick > DF_I2C_BUSY_TICK)
        {
            result = PAC5XXX_ERROR;
            break;
        }
    }

    wait_tick = 0;
    PAC55XX_I2C->I2CCONCLR.SIC = 1;

    // Write a addr with write
    PAC55XX_I2C->I2CCONSET.I2CIE = 1;
    PAC55XX_I2C->I2CDAT.DATA = ((device_addr << 1) | DF_I2C_ADDR_WRITE);      // Send address, Write command
    while (PAC55XX_I2C->I2CSTAT.w != DF_I2C_STAT_MASTER__ADDR_AND_WRITE_BIT_TRANSMITTED_AND_ACK_RECEIVED)
    {
        wait_tick++;
        if (wait_tick > DF_I2C_BUSY_TICK)
        {
            result = PAC5XXX_ERROR;
            break;
        }

        if (PAC55XX_I2C->I2CSTAT.w == DF_I2C_STAT_MASTER__ADDR_AND_WRITE_BIT_TRANSMITTED_AND_NACK_RECEIVED)
        {
            result = PAC5XXX_ERROR;
            PAC55XX_I2C->I2CCONSET.STO = 1;                     // Stop
            PAC55XX_I2C->I2CCONCLR.SIC = 1;
            return result;
        }
    }

    wait_tick = 0;
    PAC55XX_I2C->I2CCONCLR.SIC = 1;

    // Write a data
    PAC55XX_I2C->I2CCONSET.I2CIE = 1;
    PAC55XX_I2C->I2CDAT.DATA = data_addr;                       // Send address, Write command
    while (PAC55XX_I2C->I2CSTAT.w != DF_I2C_STAT_MASTER__DATA_TRANSMITTED_AND_ACK_RECEIVED)
    {
        wait_tick++;
        if (wait_tick > DF_I2C_BUSY_TICK)
        {
            result = PAC5XXX_ERROR;
            break;
        }

        if (PAC55XX_I2C->I2CSTAT.w == DF_I2C_STAT_MASTER__DATA_TRANSMITTED_AND_NACK_RECEIVED)
        {
            result = PAC5XXX_ERROR;
            PAC55XX_I2C->I2CCONSET.STO = 1;                     // Stop
            PAC55XX_I2C->I2CCONCLR.SIC = 1;
            return result;
        }
    }

    wait_tick = 0;
    PAC55XX_I2C->I2CCONCLR.SIC = 1;

    // Write a data
    PAC55XX_I2C->I2CCONSET.I2CIE = 1;
    PAC55XX_I2C->I2CDAT.DATA = write_data;                      // Send address, Write command
    while (PAC55XX_I2C->I2CSTAT.w != DF_I2C_STAT_MASTER__DATA_TRANSMITTED_AND_ACK_RECEIVED)
    {
        wait_tick++;
        if (wait_tick > DF_I2C_BUSY_TICK)
        {
            result = PAC5XXX_ERROR;
            break;
        }

        if (PAC55XX_I2C->I2CSTAT.w == DF_I2C_STAT_MASTER__DATA_TRANSMITTED_AND_NACK_RECEIVED)
        {
            result = PAC5XXX_ERROR;
            PAC55XX_I2C->I2CCONSET.STO = 1;                     // Stop
            PAC55XX_I2C->I2CCONCLR.SIC = 1;
            return result;
        }
    }

    wait_tick = 0;
    PAC55XX_I2C->I2CCONSET.STO = 1;                             // Stop
    PAC55XX_I2C->I2CCONCLR.SIC = 1;

    return result;
}

//==============================================================================
/// @brief
///     Write a serial bytes to the device address by I2C as a master
///
///@param[in]
///     device_addr:
///         the device address
///     data_addr:
///         the data address
///     write_data:
///         The data point to write.
///     byte_num:
///         the number to write.
///
///@param[out]
///     result: the result of the I2C write
///
///@retval
///     0: All is OK.
///     others: Some error occurs.
///
//==============================================================================
uint32_t i2c_master_write_multi_standard(uint8_t device_addr, uint8_t data_addr, uint8_t *write_data, uint32_t byte_num)
{
    uint32_t result = PAC5XXX_OK;
    uint32_t wait_tick = 0;
    uint32_t byte_num_temp;

    // Write a start
    PAC55XX_I2C->I2CCONSET.I2CIE = 1;
    PAC55XX_I2C->I2CCONSET.STA = 1;                             // Start
    while (PAC55XX_I2C->I2CSTAT.w != DF_I2C_STAT_MASTER__FIRST_START_TRANSMITTED)
    {
        wait_tick++;
        if (wait_tick > DF_I2C_BUSY_TICK)
        {
            result = PAC5XXX_ERROR;
            break;
        }
    }

    wait_tick = 0;
    PAC55XX_I2C->I2CCONCLR.SIC = 1;

    // Write a addr with write
    PAC55XX_I2C->I2CCONSET.I2CIE = 1;
    PAC55XX_I2C->I2CDAT.DATA = ((device_addr << 1) | DF_I2C_ADDR_WRITE);      // Send address, Write command
    while (PAC55XX_I2C->I2CSTAT.w != DF_I2C_STAT_MASTER__ADDR_AND_WRITE_BIT_TRANSMITTED_AND_ACK_RECEIVED)
    {
        wait_tick++;
        if (wait_tick > DF_I2C_BUSY_TICK)
        {
            result = PAC5XXX_ERROR;
            break;
        }

        if (PAC55XX_I2C->I2CSTAT.w == DF_I2C_STAT_MASTER__ADDR_AND_WRITE_BIT_TRANSMITTED_AND_NACK_RECEIVED)
        {
            result = PAC5XXX_ERROR;
            PAC55XX_I2C->I2CCONSET.STO = 1;                     // Stop
            PAC55XX_I2C->I2CCONCLR.SIC = 1;
            return result;
        }
    }

    wait_tick = 0;
    PAC55XX_I2C->I2CCONCLR.SIC = 1;

    // Write a data
    PAC55XX_I2C->I2CCONSET.I2CIE = 1;
    PAC55XX_I2C->I2CDAT.DATA = data_addr;                       // Send address, Write command
    while (PAC55XX_I2C->I2CSTAT.w != DF_I2C_STAT_MASTER__DATA_TRANSMITTED_AND_ACK_RECEIVED)
    {
        wait_tick++;
        if (wait_tick > DF_I2C_BUSY_TICK)
        {
            result = PAC5XXX_ERROR;
            break;
        }

        if (PAC55XX_I2C->I2CSTAT.w == DF_I2C_STAT_MASTER__DATA_TRANSMITTED_AND_NACK_RECEIVED)
        {
            result = PAC5XXX_ERROR;
            PAC55XX_I2C->I2CCONSET.STO = 1;                     // Stop
            PAC55XX_I2C->I2CCONCLR.SIC = 1;
            return result;
        }
    }

    wait_tick = 0;
    PAC55XX_I2C->I2CCONCLR.SIC = 1;

    for (byte_num_temp=0; byte_num_temp<byte_num; byte_num_temp++)
    {
        // Write a data
        PAC55XX_I2C->I2CCONSET.I2CIE = 1;
        PAC55XX_I2C->I2CDAT.DATA = *write_data++;               // Send address, Write command
        while (PAC55XX_I2C->I2CSTAT.w != DF_I2C_STAT_MASTER__DATA_TRANSMITTED_AND_ACK_RECEIVED)
        {
            wait_tick++;
            if (wait_tick > DF_I2C_BUSY_TICK)
            {
                result = PAC5XXX_ERROR;
                break;
            }

            if (PAC55XX_I2C->I2CSTAT.w == DF_I2C_STAT_MASTER__DATA_TRANSMITTED_AND_NACK_RECEIVED)
            {
                result = PAC5XXX_ERROR;
                PAC55XX_I2C->I2CCONSET.STO = 1;                 // Stop
                PAC55XX_I2C->I2CCONCLR.SIC = 1;
                return result;
            }
        }

        wait_tick = 0;

        if (byte_num_temp == byte_num-1)
            PAC55XX_I2C->I2CCONSET.STO = 1;                     // Stop

        PAC55XX_I2C->I2CCONCLR.SIC = 1;
    }

    return result;
}
