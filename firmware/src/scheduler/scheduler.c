
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

#include <src/common.h>
#include <src/system/system.h>
#include <src/gatedriver/gatedriver.h>
#include <src/adc/adc.h>
#include <src/uart/uart_interface.h>
#include <src/uart/uart_lowlevel.h>
#include <src/i2c/i2c.h>
#include <src/encoder/encoder.h>
#include <src/encoder/ma7xx.h>
#include <src/observer/observer.h>
#include <src/scheduler/scheduler.h>
#include <src/watchdog/watchdog.h>

volatile uint32_t msTicks = 0;

SchedulerState state = {0};

void WaitForControlLoopInterrupt(void)
{
	
	while (!state.adc_interrupt)
	{
		
		if (state.uart_message_interrupt)
		{
			// Handle UART
			state.uart_message_interrupt = false;
			UART_process_message();
		}
		else if (state.i2c_message_interrupt)
		{
			state.i2c_message_interrupt = false;
			I2C_process_message();
		}
		else if (state.wwdt_interrupt)
		{
			state.wwdt_interrupt = false;
			WWDT_process_interrupt();
		}
		else
		{
			state.busy = false;
			// Go back to sleep
			__DSB();
			__ISB();
			__WFI();
		}
	}
	state.busy = true;
	state.adc_interrupt = false;
	// We have to service the control loop by updating
	// current measurements and encoder estimates.
	if (ENCODER_MA7XX == encoder_get_type())
	{
		ma7xx_send_angle_cmd();
	}
	ADC_update();
	
	encoder_update(true);
	observer_update();
	// At this point control is returned to main loop.
}

void ADC_IRQHandler(void)
{
	PAC55XX_ADC->ADCINT.ADCIRQ0IF = 1;
	// Only in case the gate driver is enabled, ensure
	// the control deadline is not missed,
	// i.e. the previous control loop is complete prior
	// to the ADC triggering the next
	if ((gate_driver_is_enabled() == true) && (state.busy == true))
	{
		state.errors |= SCHEDULER_ERRORS_CONTROL_BLOCK_REENTERED;
		// We do not change the control state here, to
		// avoid any concurrency issues
	}
	else
	{
		state.adc_interrupt = true;
	}
}

void SysTick_Handler(void)
{                               
    msTicks = msTicks + 1; 
    system_update();
}

void UART_ReceiveMessageHandler(void)
{
	state.uart_message_interrupt = true;
}

void I2C_ReceiveMessageHandler(void)
{
	state.i2c_message_interrupt = true;
}

void Wdt_IRQHandler(void)
{
	state.wwdt_interrupt = true;
	PAC55XX_WWDT->WWDTLOCK = WWDTLOCK_REGS_WRITE_AVALABLE;
    // Interrupt flag needs to be cleared here
    PAC55XX_WWDT->WWDTFLAG.IF = 1;
}

TM_RAMFUNC uint8_t scheduler_get_errors(void)
{
	return state.errors;
}
