//*****************************************************************************
//
// led_task.h - Prototypes for the LED task.
//
// Copyright (c) 2012-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.4.178 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#ifndef __GPIO_TASK_H__
#define __GPIO_TASK_H__

//*****************************************************************************
//
// Prototypes for the LED task.
//
//*****************************************************************************
uint32_t GPIOTaskInit(void);
void GPIO_CAP1_set_high(void);
void GPIO_CAP1_set_low(void);
void GPIO_CAP2_set_high(void);
void GPIO_CAP2_set_low(void);
void GPIO_CAP3_set_high(void);
void GPIO_CAP3_set_low(void);
void GPIO_Breaker_set_low(void);
void GPIO_Breaker_set_high(void);
int get_load_v_rms(); 
int get_load_i_rms();
int get_dist_v_rms();
int get_dist_i_rms();

#endif // __LED_TASK_H__
