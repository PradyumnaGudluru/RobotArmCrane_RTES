//*****************************************************************************
//
// system_tasks.c - System tasks.
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

#include "system_tasks.h"
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/timer.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "utils/uartstdio.h"
#include "robot_system_timers.h"
#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "mpu6050.h"

//*****************************************************************************
//
// The stack size for the system tasks.
//
//*****************************************************************************
#define TASK_STACK_SIZE         128         // Stack size in words

#define COUNTS_PER_G            (8192.0)

typedef struct motor_data_s {
    float pitch;
    float roll;
} motor_data_t;


//*****************************************************************************
//
// The item size and queue size for the Processing message queue.
//
//*****************************************************************************
#define PROCESSING_ITEM_SIZE           sizeof(motor_data_t)
#define PROCESSING_QUEUE_SIZE          128

xSemaphoreHandle g_pMpuSemaphore;
xSemaphoreHandle g_pBaseMotorSemaphore;
xSemaphoreHandle g_pArmMotorSemaphore;

xQueueHandle g_pBaseMotorDataQueue;
xQueueHandle g_pArmMotorDataQueue;

extern xSemaphoreHandle g_pUARTSemaphore;

//*****************************************************************************
//
// MPU6050 Task.
//
//*****************************************************************************
static void MPU6050Task(void *pvParameters)
{
    uint8_t ui8AccelBuffer[6];
    int16_t i16X, i16Y, i16Z;
    motor_data_t xMotorData;
    float fX, fY, fZ;
    float fPrevPitch, fPrevRoll;

    TimerEnable(TIMER5_BASE, TIMER_A);
    vStartMpuTimer();
    vStartBaseMotorTimer();
    vStartArmMotorTimer();
    uint16_t t1, t2;

    while (1)
    {
        xSemaphoreTake(g_pMpuSemaphore, portMAX_DELAY);
        xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
        UARTprintf("MPU6050 Task Start: %d ms.\n", ui32GetTickMs());
        xSemaphoreGive(g_pUARTSemaphore);

        t1 = (uint16_t)TimerValueGet(TIMER5_BASE, TIMER_A);

        vMPU6050_Read(0x3B, &ui8AccelBuffer[0], 6);
        i16X = (uint16_t)(ui8AccelBuffer[0] << 8 | ui8AccelBuffer[1]);
        i16Y = (uint16_t)(ui8AccelBuffer[2] << 8 | ui8AccelBuffer[3]);
        i16Z = (uint16_t)(ui8AccelBuffer[4] << 8 | ui8AccelBuffer[5]);
        fX = i16X / COUNTS_PER_G;
        fY = i16Y / COUNTS_PER_G;
        fZ = i16Z / COUNTS_PER_G;
        xMotorData.pitch = (atan2(fX, sqrt(fY * fY + fZ*fZ))*180.0)/M_PI;
        xMotorData.roll = (atan2(fY, sqrt(fX * fX + fZ*fZ))*180.0)/M_PI;

        if ((xMotorData.roll > fPrevRoll + 5) || (xMotorData.roll < fPrevRoll - 5))
        {
//            UARTprintf("Send to base motor service...\n");
            xQueueSend(g_pBaseMotorDataQueue, &xMotorData, portMAX_DELAY);
        }
        if ((xMotorData.pitch > fPrevPitch + 5) || (xMotorData.pitch < fPrevPitch - 5))
        {
            xQueueSend(g_pArmMotorDataQueue, &xMotorData, portMAX_DELAY);
        }

        fPrevPitch = xMotorData.pitch;
        fPrevRoll = xMotorData.roll;

        t2 = (uint16_t)TimerValueGet(TIMER5_BASE, TIMER_A);

        xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
        UARTprintf("MPU6050 Read: %d us.\n", (uint16_t)(t1 - t2));
        xSemaphoreGive(g_pUARTSemaphore);
    }
}

//*****************************************************************************
//
// Base Motor Task.
//
//*****************************************************************************
static void BaseMotorTask(void *pvParameters)
{
    uint16_t t1, t2;
    motor_data_t xMotorData;
    uint32_t ui32DutyCycleTicks;

    while (1)
    {
        xSemaphoreTake(g_pBaseMotorSemaphore, portMAX_DELAY);

        xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
        UARTprintf("Base Motor Task Start: %d ms.\n", ui32GetTickMs());
        xSemaphoreGive(g_pUARTSemaphore);

        t1 = (uint16_t)TimerValueGet(TIMER5_BASE, TIMER_A);
        if(xQueueReceive(g_pBaseMotorDataQueue, &xMotorData, 0) == pdPASS)
        {

            ui32DutyCycleTicks = ((xMotorData.roll * 500)/180) + ((PWM_TICKS_0 + PWM_TICKS_180)/2);
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, ui32DutyCycleTicks);

            t2 = (uint16_t)TimerValueGet(TIMER5_BASE, TIMER_A);

            xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
            UARTprintf("Base Motor Set Angle: %d us.\n", (uint16_t)(t1 - t2));
            UARTprintf("Roll: %d degrees.\n", (int)xMotorData.roll);
            xSemaphoreGive(g_pUARTSemaphore);
        }
    }
}

//*****************************************************************************
//
// Arm Motor Task.
//
//*****************************************************************************
static void ArmMotorTask(void *pvParameters)
{
    uint16_t t1, t2;
    motor_data_t xMotorData;
    uint32_t ui32DutyCycleTicks;

    while (1)
    {
        xSemaphoreTake(g_pArmMotorSemaphore, portMAX_DELAY);
        xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
        UARTprintf("Arm Motor Task Start: %d ms.\n", ui32GetTickMs());
        xSemaphoreGive(g_pUARTSemaphore);

        t1 = (uint16_t)TimerValueGet(TIMER5_BASE, TIMER_A);

        if(xQueueReceive(g_pArmMotorDataQueue, &xMotorData, 0) == pdPASS)
        {

            if (xMotorData.pitch < -30)
            {
                xMotorData.pitch = -30;
            }
            else if (xMotorData.pitch > 30)
            {
                xMotorData.pitch = 30;
            }

            ui32DutyCycleTicks = ((xMotorData.pitch * 500)/180) + ((PWM_TICKS_0 + PWM_TICKS_60)/2);
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, ui32DutyCycleTicks);

            t2 = (uint16_t)TimerValueGet(TIMER5_BASE, TIMER_A);

            xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
            UARTprintf("Arm Motor Set Angle: %d us.\n", (uint16_t)(t1 - t2));
            UARTprintf("Pitch: %d degrees.\n", (int)xMotorData.pitch);
            xSemaphoreGive(g_pUARTSemaphore);
        }
    }
}

//*****************************************************************************
//
// Initializes the MPU6050 task.
//
//*****************************************************************************
uint32_t MPU6050TaskInit(void* pvParameters)
{
    //
    // Create a counting semaphore for the MPU6050 task.
    //
    g_pMpuSemaphore = xSemaphoreCreateCounting(1, 1);
    uint8_t ui8InitBuffer[1] = { 0x00 };

    vMPU6050_Init();

    vMPU6050_Write(0x6B, ui8InitBuffer, 1);

    g_pBaseMotorDataQueue = xQueueCreate(PROCESSING_QUEUE_SIZE, PROCESSING_ITEM_SIZE);
    g_pArmMotorDataQueue = xQueueCreate(PROCESSING_QUEUE_SIZE, PROCESSING_ITEM_SIZE);

    //
    // Create the Processing task.
    //
    if(xTaskCreate(MPU6050Task, (const portCHAR *)"MPU6050 Task", TASK_STACK_SIZE, pvParameters,
                   tskIDLE_PRIORITY + PRIORITY_MPU6050_TASK, NULL) != pdTRUE)
    {
        return(1);
    }

    //
    // Success.
    //
    return(0);
}

//*****************************************************************************
//
// Initializes the Base Motor task.
//
//*****************************************************************************
uint32_t BaseMotorTaskInit(void* pvParameters)
{
    //
    // Create a counting semaphore for the Motor task.
    //
    g_pBaseMotorSemaphore = xSemaphoreCreateCounting(1, 1);

    //
    // Create the Processing task.
    //
    if(xTaskCreate(BaseMotorTask, (const portCHAR *)"Base Motor Task", TASK_STACK_SIZE, pvParameters,
                   tskIDLE_PRIORITY + PRIORITY_MOTOR_TASK, NULL) != pdTRUE)
    {
        return(1);
    }

    //
    // Success.
    //
    return(0);
}

//*****************************************************************************
//
// Initializes the Arm Motor task.
//
//*****************************************************************************
uint32_t ArmMotorTaskInit(void* pvParameters)
{
    //
    // Create a counting semaphore for the Motor task.
    //
    g_pArmMotorSemaphore = xSemaphoreCreateCounting(1, 1);

    //
    // Create the Processing task.
    //
    if(xTaskCreate(ArmMotorTask, (const portCHAR *)"Arm Motor Task", TASK_STACK_SIZE, pvParameters,
                   tskIDLE_PRIORITY + PRIORITY_MOTOR_TASK, NULL) != pdTRUE)
    {
        return(1);
    }

    //
    // Success.
    //
    return(0);
}
