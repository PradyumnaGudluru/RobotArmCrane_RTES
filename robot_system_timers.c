/*
 * robot_system_timer.c
 *
 *  Created on: Mar 25, 2022
 *      Author: vishn
 */

#include "robot_system_timers.h"
#include <stdbool.h>
#include <stdint.h>
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "semphr.h"

#define MS_MULTIPLIER       (1000)

xTimerHandle g_pTimestampTimerHandle;
xTimerHandle g_pMpuTimerHandle;
xTimerHandle g_pBaseMotorTimerHandle;
xTimerHandle g_pArmMotorTimerHandle;

const uint32_t ui32IdMpuTimer = 1;
const uint32_t ui32IdBaseMotorTimer = 2;
const uint32_t ui32IdArmMotorTimer = 3;

static volatile uint32_t g_ui32Counter = 0;

extern xSemaphoreHandle g_pMpuSemaphore;
extern xSemaphoreHandle g_pBaseMotorSemaphore;
extern xSemaphoreHandle g_pArmMotorSemaphore;

//*****************************************************************************
//
// Timer Callback Function.
//
//*****************************************************************************
void vTimerCallback(TimerHandle_t pxTimer)
{
    uint32_t ui32ID;
    g_ui32Counter++;

    ui32ID = (uint32_t)pvTimerGetTimerID(pxTimer);

    if (ui32ID == ui32IdMpuTimer)
    {
        xSemaphoreGive(g_pMpuSemaphore);
    }
    else if (ui32ID == ui32IdBaseMotorTimer)
    {
        xSemaphoreGive(g_pBaseMotorSemaphore);
    }
    else if (ui32ID == ui32IdArmMotorTimer)
    {
        xSemaphoreGive(g_pArmMotorSemaphore);
    }
}

//*****************************************************************************
//
// Initializes timer.
//
//*****************************************************************************
void vInitTimers(uint32_t ui32Ms)
{
    g_pTimestampTimerHandle = xTimerCreate("Timestamp Timer", (TickType_t)ui32Ms, pdTRUE, (void *)0, vTimerCallback);
    g_pMpuTimerHandle = xTimerCreate("MPU Timer", (TickType_t)25, pdTRUE, (void *)ui32IdMpuTimer, vTimerCallback);
    g_pBaseMotorTimerHandle = xTimerCreate("Base Motor Timer", (TickType_t)50, pdTRUE, (void *)ui32IdBaseMotorTimer, vTimerCallback);
    g_pArmMotorTimerHandle = xTimerCreate("Arm Motor Timer", (TickType_t)50, pdTRUE, (void *)ui32IdArmMotorTimer, vTimerCallback);
    xTimerStart(g_pTimestampTimerHandle, 0);
}

//*****************************************************************************
//
// Get ticks in ms.
//
//*****************************************************************************
uint32_t ui32GetTickMs(void)
{
    return g_ui32Counter;
}

//*****************************************************************************
//
// Starts MPU timer.
//
//*****************************************************************************
void vStartMpuTimer(void)
{
    xTimerStart(g_pMpuTimerHandle, 0);
}

//*****************************************************************************
//
// Stop MPU timer.
//
//*****************************************************************************
void vStopMpuTimer(void)
{
    xTimerStop(g_pMpuTimerHandle, 0);
}

//*****************************************************************************
//
// Starts Base Motor timer.
//
//*****************************************************************************
void vStartBaseMotorTimer(void)
{
    xTimerStart(g_pBaseMotorTimerHandle, 0);
}

//*****************************************************************************
//
// Starts Base Motor timer.
//
//*****************************************************************************
void vStartArmMotorTimer(void)
{
    xTimerStart(g_pArmMotorTimerHandle, 0);
}
