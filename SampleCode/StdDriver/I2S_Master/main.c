/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 9 $
 * $Date: 15/07/02 11:18a $
 * @brief
 *           Demonstrate how I2S works in master mode.
 *           This sample code needs to work with I2S_Slave sample code.
 * @note
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NUC123.h"

uint32_t g_u32TxValue;

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable XT1_OUT (PF0) and XT1_IN (PF1) */
    SYS->GPF_MFP &= ~(SYS_GPF_MFP_PF0_Msk | SYS_GPF_MFP_PF1_Msk);
    SYS->GPF_MFP |= SYS_GPF_MFP_PF0_XT1_OUT | SYS_GPF_MFP_PF1_XT1_IN;

    /* Enable External XTAL (4~24 MHz) */
    CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);

    /* Switch HCLK clock source to HXT */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HXT, CLK_CLKDIV_HCLK(1));

    /* Set PLL to power down mode and PLL_STB bit in CLKSTATUS register will be cleared by hardware.*/
    CLK->PLLCON |= CLK_PLLCON_PD_Msk;

    /* Set core clock rate to 50 MHz */
    CLK_SetCoreClock(50000000);

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(I2S_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk);
    SYS->GPB_MFP |= (SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD);

    /* Set GPA multi-function pins for I2S MCLK. */
    SYS->GPA_MFP &= ~(SYS_GPA_MFP_PA15_Msk);
    SYS->GPA_MFP |= SYS_GPA_MFP_PA15_I2S_MCLK;
    /* Set multi function pin for I2S: GPC0, GPC1, GPC2, GPC3, GPA15 */
    SYS->GPC_MFP &= ~(SYS_GPC_MFP_PC0_Msk | SYS_GPC_MFP_PC1_Msk | SYS_GPC_MFP_PC2_Msk | SYS_GPC_MFP_PC3_Msk);
    SYS->GPC_MFP |= SYS_GPC_MFP_PC0_I2S_LRCLK | SYS_GPC_MFP_PC1_I2S_BCLK | SYS_GPC_MFP_PC2_I2S_DI | SYS_GPC_MFP_PC3_I2S_DO;
    SYS->ALT_MFP &= ~(SYS_ALT_MFP_PA15_Msk | SYS_ALT_MFP_PC0_Msk | SYS_ALT_MFP_PC1_Msk | SYS_ALT_MFP_PC2_Msk | SYS_ALT_MFP_PC3_Msk);
    SYS->ALT_MFP |= SYS_ALT_MFP_PA15_I2S_MCLK | SYS_ALT_MFP_PC0_I2S_LRCLK | SYS_ALT_MFP_PC1_I2S_BCLK | SYS_ALT_MFP_PC2_I2S_DI | SYS_ALT_MFP_PC3_I2S_DO;
}

void I2S_IRQHandler()
{
    /* Write 4 TX values to TX FIFO */
    I2S_WRITE_TX_FIFO(I2S, g_u32TxValue);
    I2S_WRITE_TX_FIFO(I2S, g_u32TxValue);
    I2S_WRITE_TX_FIFO(I2S, g_u32TxValue);
    I2S_WRITE_TX_FIFO(I2S, g_u32TxValue);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32DataCount, u32RxValue1, u32RxValue2;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("+-----------------------------------------------------------+\n");
    printf("|            I2S Driver Sample Code (master mode)           |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("  I2S configuration:\n");
    printf("      Sample rate 16 kHz\n");
    printf("      Word width 16 bits\n");
    printf("      Stereo mode\n");
    printf("      I2S format\n");
    printf("      TX value: 0x55005501, 0x55025503, ..., 0x55FE55FF, wraparound\n");
    printf("  The I/O connection for I2S:\n");
    printf("      I2S_LRCLK (PC0)\n      I2S_BCLK(PC1)\n");
    printf("      I2S_DI (PC2)\n      I2S_DO (PC3)\n\n");
    printf("  NOTE: Connect with a I2S slave device.\n");
    printf("        This sample code will transmit a TX value 50000 times, and then change to the next TX value.\n");
    printf("        When TX value or the received value changes, the new TX value or the current TX value and the new received value will be printed.\n");

    /* select source from HXT (12MHz) */
    CLK_SetModuleClock(I2S_MODULE, CLK_CLKSEL2_I2S_S_HXT, 0);

    /* Master mode, 16-bit word width, stereo mode, I2S format. Set TX and RX FIFO threshold to middle value. */
    I2S_Open(I2S, I2S_MODE_MASTER, 16000, I2S_DATABIT_16, I2S_STEREO, I2S_FORMAT_I2S);
    NVIC_EnableIRQ(I2S_IRQn);

    /* Initiate data counter */
    u32DataCount = 0;
    /* Initiate TX value and Rx value */
    g_u32TxValue = 0x55005501;
    u32RxValue1 = 0;
    u32RxValue2 = 0;
    /* Enable TX threshold level interrupt */
    I2S_EnableInt(I2S, I2S_IE_TXTHIE_Msk);

    printf("Start I2S ...\nTX value: 0x%X\n", g_u32TxValue);

    while(1)
    {
        if((I2S->STATUS & I2S_STATUS_RXEMPTY_Msk) == 0)
        {
            u32DataCount++;
            u32RxValue2 = I2S_READ_RX_FIFO(I2S);
            if(u32RxValue1 != u32RxValue2)
            {
                u32RxValue1 = u32RxValue2;
                printf("TX value: 0x%X;  RX value: 0x%X\n", g_u32TxValue, u32RxValue1);
            }
            if(u32DataCount >= 50000)
            {
                g_u32TxValue = 0x55005500 | ((g_u32TxValue + 0x00020002) & 0x00FF00FF); /* g_u32TxValue: 0x55005501, 0x55025503, ..., 0x55FE55FF */
                printf("TX value: 0x%X\n", g_u32TxValue);
                u32DataCount = 0;
            }
        }
    }
}

/*** (C) COPYRIGHT 2014~2015 Nuvoton Technology Corp. ***/
