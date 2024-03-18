/******************************************************************************
 * @file     main.c
 * @brief
 *           Demonstrate how to transfer data between USB device and PC through USB HID interface.
 *           A windows tool is also included in this sample code to connect with USB device.
 *
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC123.h"
#include "hid_transfer.h"

/*--------------------------------------------------------------------------*/
void SYS_Init(void)
{

    /* Enable XT1_OUT (PF.0) and XT1_IN (PF.1) */
    SYS->GPF_MFP &= ~(SYS_GPF_MFP_PF0_Msk | SYS_GPF_MFP_PF1_Msk);
    SYS->GPF_MFP |= SYS_GPF_MFP_PF0_XT1_OUT | SYS_GPF_MFP_PF1_XT1_IN;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184 MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC, CLK_CLKDIV_HCLK(1));

    /* Enable external XTAL 12 MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);

    /* Waiting for external XTAL clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);

    /* Set core clock */
    CLK_SetCoreClock(72000000);

    /* Enable module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(USBD_MODULE);

    /* Select module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));
    CLK_SetModuleClock(USBD_MODULE, 0, CLK_CLKDIV_USB(3));


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk);
    SYS->GPB_MFP |= SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD;
    /* Set PC.13 as CLKO function pin */
    SYS->GPC_MFP &= ~(SYS_GPC_MFP_PC13_Msk);
    SYS->GPC_MFP |= SYS_GPC_MFP_PC13_CLKO;
    SYS->ALT_MFP &= ~(SYS_ALT_MFP_PC13_Msk);
    SYS->ALT_MFP |= SYS_ALT_MFP_PC13_CLKO;

    /* Enable CLKO (PC.13) for monitor HCLK. CLKO = HCLK/8 Hz*/
    CLK_EnableCKO(CLK_CLKSEL2_FRQDIV_S_HCLK, 2, (uint32_t)NULL);
}


void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS->IPRSTC2 |=  SYS_IPRSTC2_UART0_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_UART0_RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 115200);
    UART0->LCR = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

void PowerDown()
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Wakeup Enable */
    USBD_ENABLE_INT(USBD_INTEN_WAKEUP_EN_Msk);

    CLK_PowerDown();

    /* Clear PWR_DOWN_EN if it is not clear by itself */
    if(CLK->PWRCON & CLK_PWRCON_PWR_DOWN_EN_Msk)
        CLK->PWRCON ^= CLK_PWRCON_PWR_DOWN_EN_Msk;

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /*
        This sample code demonstrate how to use HID interface to transfer data
        between PC and USB device.
        A demo window tool are also included in "WindowsTool" directory with this
        sample code. User can use it to test data transfer with this sample code.

    */

    /* Unlock write-protected registers */
    SYS_UnlockReg();

    /* Init system and multi-funcition I/O */
    SYS_Init();

    /* Init UART for debug message */
    UART0_Init();

    printf("\n");
    printf("+--------------------------------------------------------+\n");
    printf("|          NuMicro USB HID Transfer Sample Code          |\n");
    printf("+--------------------------------------------------------+\n");

    /* Open USB controller */
    USBD_Open(&gsInfo, HID_ClassRequest, NULL);

    /*Init Endpoint configuration for HID */
    HID_Init();

    /* Start USB device */
    USBD_Start();

    /* Enable USB device interrupt */
    NVIC_EnableIRQ(USBD_IRQn);

    while(SYS->PDID)
    {
        /* Enter power down when USB suspend */
        if(g_u8Suspend)
            PowerDown();
    }
}



/*** (C) COPYRIGHT 2014~2015 Nuvoton Technology Corp. ***/

