/*
 * robot_system_timers.h
 *
 *  Created on: Mar 25, 2022
 *      Author: vishn
 */

#ifndef ROBOT_SYSTEM_TIMERS_H_
#define ROBOT_SYSTEM_TIMERS_H_

#include <stdint.h>

//*****************************************************************************
//
// Initializes timer.
//
//*****************************************************************************
void vInitTimers(uint32_t ui32Ms);

//*****************************************************************************
//
// Get ticks in ms.
//
//*****************************************************************************
uint32_t ui32GetTickMs(void);

//*****************************************************************************
//
// Starts MPU timer.
//
//*****************************************************************************
void vStartMpuTimer(void);

//*****************************************************************************
//
// Stops MPU timer.
//
//*****************************************************************************
void vStopMpuTimer(void);

//*****************************************************************************
//
// Starts Base Motor timer.
//
//*****************************************************************************
void vStartBaseMotorTimer(void);

//*****************************************************************************
//
// Starts Arm Motor timer.
//
//*****************************************************************************
void vStartArmMotorTimer(void);



#endif /* ROBOT_SYSTEM_TIMERS_H_ */