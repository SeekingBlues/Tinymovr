
//  * This file is part of the Tinymovr-Firmware distribution
//  * (https://github.com/yconst/tinymovr-firmware).
//  * Copyright (c) 2020-2023 Ioannis Chatzikonstantinou.
//  * 
//  * This program is free software: you can redistribute it and/or modify  
//  * it under the terms of the GNU General Public License as published by  
//  * the Free Software Foundation, version 3.
//  *
//  * This program is distributed in the hope that it will be useful, but 
//  * WITHOUT ANY WARRANTY; without even the implied warranty of 
//  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
//  * General Public License for more details.
//  *
//  * You should have received a copy of the GNU General Public License 
//  * along with this program. If not, see <http://www.gnu.org/licenses/>.

#pragma once

typedef enum
{
    SCHEDULER_ERRORS_NONE = 0,
    SCHEDULER_ERRORS_CONTROL_BLOCK_REENTERED = (1 << 0)
} scheduler_errors_flags;

typedef struct 
{
	bool adc_interrupt;
	bool can_interrupt;
	bool uart_message_interrupt;
    bool i2c_message_interrupt;
    bool wwdt_interrupt;
	bool busy;

    uint8_t errors;
} SchedulerState;

void WaitForControlLoopInterrupt(void);

uint8_t scheduler_get_errors(void);
