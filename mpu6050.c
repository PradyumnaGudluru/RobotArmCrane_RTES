/*
 * mpu6050.c
 *
 *  Created on: Apr 14, 2022
 *      Author: vishn
 */

#include "mpu6050.h"

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_i2c.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "utils/uartstdio.h"

void vMPU6050_Init(void)
{
    ROM_SysCtlPeripheralDisable(SYSCTL_PERIPH_I2C0);
    ROM_SysCtlPeripheralDisable(SYSCTL_PERIPH_GPIOB);
    ROM_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);
    //ROM_SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOB);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while(!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0) || !ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));

    //configure PB2 to SCL and PB3 SDA
    ROM_GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    ROM_GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    ROM_GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    ROM_GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    //init I2C Master with 100kbps
    ROM_I2CMasterInitExpClk(I2C0_BASE, ROM_SysCtlClockGet(), false);
}


void vMPU6050_Write(uint8_t ui8Reg, uint8_t* ui8WrBuff, uint8_t ui8WrBuffLen)
{
    uint8_t ui8I;
    //Set device slave address
    I2CMasterSlaveAddrSet(I2C0_BASE, SLAVE_ADDRESS, false);

    //Send command
    I2CMasterDataPut(I2C0_BASE, ui8Reg);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(I2C0_BASE));

    //Send data
    I2CMasterDataPut(I2C0_BASE, ui8WrBuff[0]);
    for (ui8I = 1; ui8I < ui8WrBuffLen; ui8I++)
    {
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
        while(I2CMasterBusy(I2C0_BASE));

        I2CMasterDataPut(I2C0_BASE, ui8WrBuff[ui8I]);
    }

    //Send stop bit
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while(I2CMasterBusy(I2C0_BASE));
}


void vMPU6050_Read(uint8_t ui8Reg, uint8_t* ui8RdBuff, uint8_t ui8RdBuffLen)
{
    uint8_t ui8I;
    //Set device slave address
    I2CMasterSlaveAddrSet(I2C0_BASE, SLAVE_ADDRESS, false);

    //Send command
    I2CMasterDataPut(I2C0_BASE, ui8Reg);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(I2C0_BASE));

    //Send Sr and read bytes
    I2CMasterSlaveAddrSet(I2C0_BASE, SLAVE_ADDRESS, true);

    if (ui8RdBuffLen == 1)
    {
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
        while(I2CMasterBusy(I2C0_BASE));

        ui8RdBuff[0] = I2CMasterDataGet(I2C0_BASE);
    }
    else
    {
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
        while(I2CMasterBusy(I2C0_BASE));

        ui8RdBuff[0] = I2CMasterDataGet(I2C0_BASE);

        for (ui8I = 1; ui8I < ui8RdBuffLen - 1; ui8I++)
        {
            I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
            while(I2CMasterBusy(I2C0_BASE));

            ui8RdBuff[ui8I] = I2CMasterDataGet(I2C0_BASE);
        }

        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
        while(I2CMasterBusy(I2C0_BASE));

        ui8RdBuff[ui8I] = I2CMasterDataGet(I2C0_BASE);
    }
}


