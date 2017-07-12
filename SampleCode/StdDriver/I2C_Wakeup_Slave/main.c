/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 16 $
 * $Date: 16/06/21 7:44p $
 * @brief
 *           Demonstrate how to set I2C to wake-up MCU from power-down mode.
 *           Needs to work with I2C_Wakeup_Master sample code.
 * @note
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NUC123.h"

#define PLLCON_SETTING      CLK_PLLCON_72MHz_HXT
#define PLL_CLOCK           72000000

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t slave_buff_addr;
volatile uint8_t g_au8SlvData[256];
volatile uint8_t g_au8SlvRxData[3];
volatile uint8_t g_u8SlvPWRDNWK, g_u8SlvI2CWK;
volatile uint8_t g_u8DeviceAddr;
volatile uint8_t g_u8SlvDataLen;

typedef void (*I2C_FUNC)(uint32_t u32Status);

static I2C_FUNC s_I2C0HandlerFn = NULL;

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C0_IRQHandler(void)
{
    uint32_t u32Status;

    /* Check I2C Wake-up interrupt flag set or not */
    if(I2C_GET_WAKEUP_FLAG(I2C0))
    {
        /* Clear I2C Wake-up interrupt flag */
        I2C_CLEAR_WAKEUP_FLAG(I2C0);
			
        g_u8SlvI2CWK = 1;		
			
        return;
    }

    u32Status = I2C_GET_STATUS(I2C0);

    if(I2C_GET_TIMEOUT_FLAG(I2C0))
    {
        /* Clear I2C0 Timeout Flag */
        I2C_ClearTimeoutFlag(I2C0);
    }
    else
    {
        if(s_I2C0HandlerFn != NULL)
            s_I2C0HandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Power Wake-up IRQ Handler                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void PWRWU_IRQHandler(void)
{
    /* Check system power down mode wake-up interrupt flag */
    if(((CLK->PWRCON) & CLK_PWRCON_PD_WU_STS_Msk) != 0)
    {
        /* Clear system power down wake-up interrupt flag */
        CLK->PWRCON |= CLK_PWRCON_PD_WU_STS_Msk;

        g_u8SlvPWRDNWK = 1;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Slave Transmit/Receive Callback Function                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_SlaveTRx(uint32_t u32Status)
{
    if(u32Status == 0x60)                       /* Own SLA+W has been receive; ACK has been return */
    {
        g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI_AA);
    }
    else if(u32Status == 0x80)                 /* Previously address with own SLA address
                                                   Data has been received; ACK has been returned*/
    {
        g_au8SlvRxData[g_u8SlvDataLen] = (unsigned char)I2C_GET_DATA(I2C0);
        g_u8SlvDataLen++;

        if(g_u8SlvDataLen == 2)
        {
            slave_buff_addr = (g_au8SlvRxData[0] << 8) + g_au8SlvRxData[1];
        }
        if(g_u8SlvDataLen == 3)
        {
            g_au8SlvData[slave_buff_addr] = g_au8SlvRxData[2];
            g_u8SlvDataLen = 0;
        }
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI_AA);
    }
    else if(u32Status == 0xA8)                  /* Own SLA+R has been receive; ACK has been return */
    {
        I2C_SET_DATA(I2C0, g_au8SlvData[slave_buff_addr]);
        slave_buff_addr++;
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI_AA);
    }
    else if(u32Status == 0xC0)                 /* Data byte or last data in I2CDAT has been transmitted
                                                   Not ACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI_AA);
    }
    else if(u32Status == 0x88)                 /* Previously addressed with own SLA address; NOT ACK has
                                                   been returned */
    {
        g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI_AA);
    }
    else if(u32Status == 0xA0)                 /* A STOP or repeated START has been received while still
                                                   addressed as Slave/Receiver*/
    {
        g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI_AA);
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  System initial function                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable XT1_OUT (PF0) and XT1_IN (PF1) */
    SYS->GPF_MFP |= SYS_GPF_MFP_PF0_XT1_OUT | SYS_GPF_MFP_PF1_XT1_IN;

    /* Enable Internal RC 22.1184MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC, CLK_CLKDIV_HCLK(1));

    /* Enable external XTAL 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);

    /* Waiting for external XTAL clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable I2C0 module clock */
    CLK_EnableModuleClock(I2C0_MODULE);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFP = SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD;

    /* Set GPF multi-function pins for I2C0 SDA and SCL */
    SYS->GPF_MFP |= (SYS_GPF_MFP_PF2_I2C0_SDA | SYS_GPF_MFP_PF3_I2C0_SCL);
    SYS->ALT_MFP1 &= ~(SYS_ALT_MFP1_PF2_Msk | SYS_ALT_MFP1_PF3_Msk);
    SYS->ALT_MFP1 |= (SYS_ALT_MFP1_PF2_I2C0_SDA | SYS_ALT_MFP1_PF3_I2C0_SCL);
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

void I2C0_Init(void)
{
    /* Open I2C module and set bus clock */
    I2C_Open(I2C0, 100000);

    /* Get I2C0 Bus Clock */
    printf("I2C0 clock %d Hz\n", I2C_GetBusClockFreq(I2C0));

    /* Set I2C 4 Slave Addresses */
    I2C_SetSlaveAddr(I2C0, 0, 0x15, 0);   /* Slave Address : 0x15 */
    I2C_SetSlaveAddr(I2C0, 1, 0x35, 0);   /* Slave Address : 0x35 */
    I2C_SetSlaveAddr(I2C0, 2, 0x55, 0);   /* Slave Address : 0x55 */
    I2C_SetSlaveAddr(I2C0, 3, 0x75, 0);   /* Slave Address : 0x75 */

    /* Set I2C 4 Slave Addresses Mask */
    I2C_SetSlaveAddrMask(I2C0, 0, 0x01);
    I2C_SetSlaveAddrMask(I2C0, 1, 0x04);
    I2C_SetSlaveAddrMask(I2C0, 2, 0x01);
    I2C_SetSlaveAddrMask(I2C0, 3, 0x04);

    /* Enable I2C interrupt */
    I2C_EnableInt(I2C0);
    NVIC_EnableIRQ(I2C0_IRQn);
}

void I2C0_Close(void)
{
    /* Disable I2C0 interrupt and clear corresponding NVIC bit */
    I2C_DisableInt(I2C0);
    NVIC_DisableIRQ(I2C0_IRQn);

    /* Disable I2C0 and close I2C0 clock */
    I2C_Close(I2C0);
    CLK_DisableModuleClock(I2C0_MODULE);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t i;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    /* Lock protected registers */
    SYS_LockReg();;

    /*
        This sample code is I2C SLAVE mode and it simulates EEPROM function
    */
    printf("\n");
    printf("+-----------------------------------------------------------------------+\n");
    printf("| NUC123 I2C Driver Sample Code (Slave) for wake-up & access Slave test |\n");
    printf("| Needs to work with I2C_Wakeup_Master sample code.                     |\n");
    printf("|      I2C Master (I2C0) <---> I2C Slave (I2C0)                         |\n");
    printf("| !! This sample code requires two borads to test !!                    |\n");
    printf("+-----------------------------------------------------------------------+\n");

    printf("Configure I2C0 as a slave.\n");
    printf("The I/O connection for I2C0:\n");
    printf("I2C0_SDA(PF.2), I2C0_SCL(PF.3)\n");

    /* Init I2C0 */
    I2C0_Init();

    for(i = 0; i < 0x100; i++)
    {
        g_au8SlvData[i] = 0;
    }

    /* I2C function to Transmit/Receive data as slave */
    s_I2C0HandlerFn = I2C_SlaveTRx;
		
    /* Set I2C0 enter Not Address SLAVE mode */
    I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI_AA);

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable power wake-up interrupt */
    CLK->PWRCON |= CLK_PWRCON_PD_WU_INT_EN_Msk;
    NVIC_EnableIRQ(PWRWU_IRQn);

    /* Enable I2C wake-up */
    I2C_EnableWakeup(I2C0);

    printf("\n");
    printf("Enter PD 0x%x 0x%x\n", I2C0->I2CON , I2C0->I2CSTATUS);
    printf("\n");
    printf("CHIP enter power down status.\n");
    /* Waiting for UART printf finish*/
    while(((UART0->FSR) & UART_FSR_TE_FLAG_Msk) == 0);

    if(((I2C0->I2CON)&I2C_I2CON_SI_Msk) != 0)
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
    }
    
    /*  Use WFI instruction to idle the CPU. NOTE:
        If ICE is attached, system will wakeup immediately because ICE is a wakeup event. */
    CLK_PowerDown();

    while((g_u8SlvPWRDNWK & g_u8SlvI2CWK) == 0);    
    printf("Power-down Wake-up INT 0x%x\n", ((CLK->PWRCON) & CLK_PWRCON_PD_WU_STS_Msk));		
    printf("I2C0 WAKE INT 0x%x\n", I2C0->I2CWKUPSTS);
		
    /* Disable power wake-up interrupt */
    CLK->PWRCON &= ~CLK_PWRCON_PD_WU_INT_EN_Msk;
    NVIC_DisableIRQ(PWRWU_IRQn);

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n");
    printf("Slave wake-up from power down status.\n");

    printf("\n");
    printf("Slave Waiting for receiving data.\n");

    while(1);
}



