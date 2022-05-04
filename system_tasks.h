//*****************************************************************************
//
// system_tasks.h - Prototypes for the system tasks.
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

#ifndef __SYSTEM_TASKS_H__
#define __SYSTEM_TASKS_H__

#include <stdint.h>

#define PWM_PERIOD_TICKS    (5000)
#define PWM_TICKS_0             (125) // 0 degrees
#define PWM_TICKS_180             (625) // 180 degrees
#define PWM_TICKS_60       (290) // 60 degrees

//*****************************************************************************
//
// Prototypes for the system Tasks.
//
//*****************************************************************************
uint32_t MPU6050TaskInit(void* pvParameters);
uint32_t BaseMotorTaskInit(void* pvParameters);
uint32_t ArmMotorTaskInit(void* pvParameters);

#endif // __SYSTEM_TASKS_H__
