#include <string.h>
#include <src/controller/controller.h>

#include "i2c.h"
#include "i2c_func.h"

#define SLAVE_ADDR 0x38
#define BUF_LEN    5

static bool s_is_writing = false;
static uint8_t s_rx_buf[BUF_LEN];
static size_t s_rx_idx = 0;

void I2C_Init(void)
{
    i2c_init(I2C_MS_SLAVE, SLAVE_ADDR);
}

void I2C_IRQHandler(void)
{
    uint8_t i2c_status = PAC55XX_I2C->I2CSTAT.w;
    switch (i2c_status) {
    case DF_I2C_STAT_SLAVE__ADDR_AND_WRITE_BIT_RECEIVED_AND_ACK_TRANSMITTED:
        // Write start
        s_is_writing = true;
        s_rx_idx = 0;
        break;
    case DF_I2C_STAT_SLAVE__ADDR_AND_READ_BIT_RECEIVED_AND_ACK_TRANSMITTED:
        // Read start
        break;
    case DF_I2C_STAT_SLAVE__DATA_AFTER_ADDR_RECEIVED_AND_ACK_TRANSMITTED:
        // Writing
        if (s_is_writing && s_rx_idx < BUF_LEN)
            s_rx_buf[s_rx_idx++] = PAC55XX_I2C->I2CDAT.DATA;
        break;
    case DF_I2C_STAT_SLAVE__DATA_TRANSMITTED_AND_ACK_RECEIVED:
        // Reading
        break;
    case DF_I2C_STAT_SLAVE__STOP_OR_RESTART_RECEIVED:
        // Read or write stop
        if (s_is_writing) {
            s_is_writing = false;
            I2C_ReceiveMessageHandler();
        }
        break;
    default:
        ;
    }

    PAC55XX_I2C->I2CCONCLR.SIC = 1;
}

void I2C_process_message(void)
{
    uint8_t cmd = s_rx_buf[0];
    int32_t value = 0;
    memcpy(&value, s_rx_buf + 1, sizeof(value));
    switch (cmd) {
    case 'P':
        controller_set_Iq_setpoint_user_frame(0);
        controller_set_vel_setpoint_user_frame(0);
        controller_set_pos_setpoint_user_frame(value);
        controller_set_mode(CTRL_POSITION);
        break;
    case 'V':
        controller_set_Iq_setpoint_user_frame(0);
        controller_set_vel_setpoint_user_frame(value);
        controller_set_mode(CTRL_VELOCITY);
        controller_set_vel_setpoint_user_frame(value);
        break;
    default:
        ;
    }
}
