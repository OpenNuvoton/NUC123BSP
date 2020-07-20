/******************************************************************************
 * @file     main.c
 * @brief
 *           Show how to implement a USB micro printer device.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC123.h"
#include "micro_printer.h"

int IsDebugFifoEmpty(void);

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

    /* Enable peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(USBD_MODULE);

    /* Select peripheral clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));
    CLK_SetModuleClock(USBD_MODULE, 0, CLK_CLKDIV_USB(3));


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk);
    SYS->GPB_MFP |= (SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD);
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

    printf("Enter power down ...\n");
    while(!IsDebugFifoEmpty());

    /* Wakeup Enable */
    USBD_ENABLE_INT(USBD_INTEN_WAKEUP_EN_Msk);

    CLK_PowerDown();

    /* Clear PWR_DOWN_EN if it is not clear by itself */
    if(CLK->PWRCON & CLK_PWRCON_PWR_DOWN_EN_Msk)
        CLK->PWRCON ^= CLK_PWRCON_PWR_DOWN_EN_Msk;

    printf("device wakeup!\n");

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint8_t Str[9];

    /* Unlock protected registers */
    SYS_UnlockReg();

    SYS_Init();

    UART0_Init();

    printf("\n");
    printf("+--------------------------------------------------------+\n");
    printf("|         NuMicro USB Micro Printer Sample Code          |\n");
    printf("+--------------------------------------------------------+\n");

    /* Open USB controller */
    USBD_Open(&gsInfo, PTR_ClassRequest, NULL);

    /* Endpoint configuration */
    PTR_Init();
    /* Start USB device */
    USBD_Start();
    NVIC_EnableIRQ(USBD_IRQn);

    PB->PMD = 0x5000;   // PB.6, PB.7 output mode

    while(1)
    {
        /* Enter power down when USB suspend */
        if(g_u8Suspend)
            PowerDown();

        CLK_SysTickDelay(2000);   // delay
        if(++Str[1] > 0x39)
            Str[1] = 0x30;      // increase 1 to 10 than reset to 0
        PB->DOUT ^= 0x40; // PB.6
    }
}



/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/

