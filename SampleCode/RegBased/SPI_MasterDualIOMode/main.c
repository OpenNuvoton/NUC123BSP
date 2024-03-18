/**************************************************************************//**
 * @file     main.c
 * @version  V2.0
 * $Revision: 2 $
 * $Date: 15/07/02 11:17a $
 * @brief
 *           Demonstrate how to communicate with an off-chip SPI slave device with Dual I/O mode and FIFO mode.
 *           This sample code needs to work with SPI_SlaveDualIOMode sample code.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 *
 * @copyright Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC123.h"


#define TEST_COUNT 16

uint32_t g_au32SourceData[TEST_COUNT];
uint32_t g_au32DestinationData[TEST_COUNT];
volatile uint32_t g_u32TxDataCount;
volatile uint32_t g_u32RxDataCount;

/* Function prototype declaration */
void SYS_Init(void);
void UART0_Init(void);
void SPI_Init(void);

/* ------------- */
/* Main function */
/* ------------- */
int main(void)
{
    uint32_t u32DataCount;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART0_Init();

    /* Init SPI */
    SPI_Init();

    printf("\n\n");
    printf("+--------------------------------------------------------+\n");
    printf("|             SPI Master Mode Sample Code                |\n");
    printf("+--------------------------------------------------------+\n");
    printf("\n");
    printf("Configure SPI0 as a master.\n");
    printf("Bit length of a transaction: 32\n");
    printf("The I/O connection for SPI0:\n");
    printf("    SPI0_SS0 (PC.0)\n    SPI0_CLK (PC.1)\n");
    printf("    SPI0_MISO0 (PC.2)\n    SPI0_MOSI0 (PC.3)\n\n");
    printf("SPI controller will enable Dual I/O mode and FIFO mode, then transfer %d data to a off-chip slave device.\n", TEST_COUNT);
    printf("Afterward the SPI controller will receive %d data from the off-chip slave device.\n", TEST_COUNT);
    printf("After the transfer is done, the %d received data will be printed out.\n", TEST_COUNT);
    printf("The SPI master configuration is ready.\n");

    for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
    {
        /* Write the initial value to source buffer */
        g_au32SourceData[u32DataCount] = 0x00550000 + u32DataCount;
        /* Clear destination buffer */
        g_au32DestinationData[u32DataCount] = 0;
    }

    printf("Before starting the data transfer, make sure the slave device is ready.\n");
    printf("Press any key to start the Dual I/O output transfer.");
    getchar();
    printf("\n");

    /* Enable Dual I/O output mode */
    SPI_ENABLE_DUAL_OUTPUT_MODE(SPI0);
    /* Set TX FIFO threshold, enable TX FIFO threshold interrupt and RX FIFO time-out interrupt */
    SPI0->FIFO_CTL = (2 << SPI_FIFO_CTL_TX_THRESHOLD_Pos) | SPI_FIFO_CTL_TIMEOUT_INTEN_Msk | SPI_FIFO_CTL_TX_INTEN_Msk;
    g_u32TxDataCount = 0;
    g_u32RxDataCount = 0;
    NVIC_EnableIRQ(SPI0_IRQn);

    /* Wait for transfer done */
    while(g_u32RxDataCount < TEST_COUNT);

    /* Enable Dual I/O input mode */
    SPI_ENABLE_DUAL_INPUT_MODE(SPI0);
    printf("Dual I/O output transfer is done. Before starting the Dual I/O input transfer, make sure the slave device is ready.\n");
    printf("Press any key to start the Dual I/O input transfer.");
    getchar();
    printf("\n");
    
    g_u32TxDataCount = 0;
    g_u32RxDataCount = 0;
    /* Set TX FIFO threshold, enable TX FIFO threshold interrupt and RX FIFO time-out interrupt */
    SPI0->FIFO_CTL = (2 << SPI_FIFO_CTL_TX_THRESHOLD_Pos) | SPI_FIFO_CTL_TIMEOUT_INTEN_Msk | SPI_FIFO_CTL_TX_INTEN_Msk;
    
    /* Wait for transfer done */
    while(g_u32RxDataCount < TEST_COUNT);
    
    /* Print the received data */
    printf("Received data:\n");
    for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
    {
        printf("%d:\t0x%X\n", u32DataCount, g_au32DestinationData[u32DataCount]);
    }
    /* Disable TX FIFO threshold interrupt and RX FIFO time-out interrupt */
    SPI0->FIFO_CTL = 0;
    NVIC_DisableIRQ(SPI0_IRQn);
    
    printf("\n\nExit SPI driver sample code.\n");

    /* Disable SPI0 peripheral clock */
    CLK->APBCLK &= (~CLK_APBCLK_SPI0_EN_Msk);
    while(1);
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable XT1_OUT (PF0) and XT1_IN (PF1) */
    SYS->GPF_MFP &= ~(SYS_GPF_MFP_PF0_Msk | SYS_GPF_MFP_PF1_Msk);
    SYS->GPF_MFP |= SYS_GPF_MFP_PF0_XT1_OUT | SYS_GPF_MFP_PF1_XT1_IN;

    /* Enable external 12 MHz XTAL */
    CLK->PWRCON |= CLK_PWRCON_XTL12M_EN_Msk;

    /* Waiting for clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_XTL12M_STB_Msk));

    /* Select HXT as the clock source of HCLK */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLK_S_Msk)) | CLK_CLKSEL0_HCLK_S_HXT;

    /* Select HXT as the clock source of UART; select HCLK as the clock source of SPI0. */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~(CLK_CLKSEL1_UART_S_Msk | CLK_CLKSEL1_SPI0_S_Msk))) | (CLK_CLKSEL1_UART_S_HXT | CLK_CLKSEL1_SPI0_S_HCLK);

    /* Enable UART and SPI0 clock */
    CLK->APBCLK = CLK_APBCLK_UART0_EN_Msk | CLK_APBCLK_SPI0_EN_Msk;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk);
    SYS->GPB_MFP |= (SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD);

    /* Setup SPI0 multi-function pins */
    SYS->GPC_MFP &= ~(SYS_GPC_MFP_PC0_Msk | SYS_GPC_MFP_PC1_Msk | SYS_GPC_MFP_PC2_Msk | SYS_GPC_MFP_PC3_Msk);
    SYS->GPC_MFP |= SYS_GPC_MFP_PC0_SPI0_SS0 | SYS_GPC_MFP_PC1_SPI0_CLK | SYS_GPC_MFP_PC2_SPI0_MISO0 | SYS_GPC_MFP_PC3_SPI0_MOSI0;
    SYS->ALT_MFP &= ~(SYS_ALT_MFP_PC0_Msk | SYS_ALT_MFP_PC1_Msk | SYS_ALT_MFP_PC2_Msk | SYS_ALT_MFP_PC3_Msk);
    SYS->ALT_MFP |= SYS_ALT_MFP_PC0_SPI0_SS0 | SYS_ALT_MFP_PC1_SPI0_CLK | SYS_ALT_MFP_PC2_SPI0_MISO0 | SYS_ALT_MFP_PC3_SPI0_MOSI0;
}

void UART0_Init(void)
{
    /* Word length is 8 bits; 1 stop bit; no parity bit. */
    UART0->LCR = UART_LCR_WLS_Msk;
    /* Using mode 2 calculation: UART bit rate = UART peripheral clock rate / (BRD setting + 2) */
    /* UART peripheral clock rate 12 MHz; UART bit rate 115200 bps. */
    /* 12000000 / 115200 bps ~= 104 */
    /* 104 - 2 = 0x66. */
    UART0->BAUD = UART_BAUD_DIV_X_EN_Msk | UART_BAUD_DIV_X_ONE_Msk | (0x66);
}

void SPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure SPI0 as a master, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Enable FIFO mode. */
    SPI0->CNTRL = SPI_CNTRL_FIFO_Msk | SPI_MASTER | SPI_CNTRL_TX_NEG_Msk;
    /* Enable the automatic hardware slave selection function. Select the SPI0_SS0 pin and configure as low-active. */
    SPI0->SSR = SPI_SSR_AUTOSS_Msk | SPI_SS0;
    /* Set IP clock divider. SPI clock rate = HCLK / ((2+1)*2) = 2 MHz */
    SPI0->DIVIDER = (SPI0->DIVIDER & (~SPI_DIVIDER_DIVIDER_Msk)) | 2;
}

void SPI0_IRQHandler(void)
{
    /* Check RX EMPTY flag */
    while(SPI_GET_RX_FIFO_EMPTY_FLAG(SPI0) == 0)
    {
        /* Read RX FIFO */
        g_au32DestinationData[g_u32RxDataCount++] = SPI_READ_RX0(SPI0);
    }
    /* Check TX FULL flag and TX data count */
    while((SPI_GET_TX_FIFO_FULL_FLAG(SPI0) == 0) && (g_u32TxDataCount < TEST_COUNT))
    {
        /* Write to TX FIFO */
        SPI_WRITE_TX0(SPI0, g_au32SourceData[g_u32TxDataCount++]);
    }
    if(g_u32TxDataCount >= TEST_COUNT)
        SPI0->FIFO_CTL &= ~SPI_FIFO_CTL_TX_INTEN_Msk; /* Disable TX FIFO threshold interrupt */

    /* Check the RX FIFO time-out interrupt flag */
    if(SPI0->STATUS & SPI_STATUS_TIMEOUT_Msk)
    {
        /* If RX FIFO is not empty, read RX FIFO. */
        while(SPI_GET_RX_FIFO_EMPTY_FLAG(SPI0) == 0)
            g_au32DestinationData[g_u32RxDataCount++] = SPI_READ_RX0(SPI0);
    }
}


/*** (C) COPYRIGHT 2014~2015 Nuvoton Technology Corp. ***/

