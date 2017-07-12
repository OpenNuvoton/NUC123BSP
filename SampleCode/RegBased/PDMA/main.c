/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 6 $
 * $Date: 15/07/02 3:06p $
 * @brief    Demonstrate how to use PDMA channel 0 to transfer data from memory to memory.
 * @note
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NUC123.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/

#define PLLCON_SETTING      CLK_PLLCON_72MHz_HXT
#define PLL_CLOCK           72000000

uint32_t PDMA_TEST_LENGTH = 64;
uint8_t SrcArray[256];
uint8_t DestArray[256];
uint32_t volatile u32IsTestOver = 0xFF;

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

/**
 * @brief       DMA IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The DMA default IRQ, declared in startup_nuc200Series.s.
 */
void PDMA_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS();

    if(status & 0x1)    /* CH0 */
    {
        if(PDMA_GET_CH_INT_STS(0) & 0x2)
            u32IsTestOver = 0;
        PDMA_CLR_CH_INT_FLAG(0, PDMA_ISR_BLKD_IF_Msk);
    }
    else if(status & 0x2)      /* CH1 */
    {
        if(PDMA_GET_CH_INT_STS(1) & 0x2)
            u32IsTestOver = 1;
        PDMA_CLR_CH_INT_FLAG(1, PDMA_ISR_BLKD_IF_Msk);
    }
    else if(status & 0x4)      /* CH2 */
    {
        if(PDMA_GET_CH_INT_STS(2) & 0x2)
            u32IsTestOver = 2;
        PDMA_CLR_CH_INT_FLAG(2, PDMA_ISR_BLKD_IF_Msk);
    }
    else if(status & 0x8)      /* CH3 */
    {
        if(PDMA_GET_CH_INT_STS(3) & 0x2)
            u32IsTestOver = 3;
        PDMA_CLR_CH_INT_FLAG(3, PDMA_ISR_BLKD_IF_Msk);
    }
    else if(status & 0x10)      /* CH4 */
    {
        if(PDMA_GET_CH_INT_STS(4) & 0x2)
            u32IsTestOver = 4;
        PDMA_CLR_CH_INT_FLAG(4, PDMA_ISR_BLKD_IF_Msk);
    }
    else if(status & 0x20)      /* CH5 */
    {
        if(PDMA_GET_CH_INT_STS(5) & 0x2)
            u32IsTestOver = 5;
        PDMA_CLR_CH_INT_FLAG(5, PDMA_ISR_BLKD_IF_Msk);
    }
    else
        printf("unknown interrupt !!\n");
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable IRC22M clock */
    CLK->PWRCON |= CLK_PWRCON_IRC22M_EN_Msk;

    /* Waiting for IRC22M clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_IRC22M_STB_Msk));

    /* Switch HCLK clock source to HIRC */
    CLK->CLKSEL0 = CLK_CLKSEL0_HCLK_S_HIRC;

    /* Set PLL to Power-down mode and PLL_STB bit in CLKSTATUS register will be cleared by hardware.*/
    CLK->PLLCON |= CLK_PLLCON_PD_Msk;

    /* Enable XT1_OUT(PF.0) and XT1_IN(PF.1) */
    SYS->GPF_MFP |= SYS_GPF_MFP_PF0_XT1_OUT | SYS_GPF_MFP_PF1_XT1_IN;

    /* Enable external 12 MHz XTAL */
    CLK->PWRCON |= CLK_PWRCON_XTL12M_EN_Msk;

    /* Enable PLL and Set PLL frequency */
    CLK->PLLCON = PLLCON_SETTING;

    /* Waiting for clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_PLL_STB_Msk));
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_XTL12M_STB_Msk));

    /* Switch HCLK clock source to PLL, STCLK to HCLK/2 */
    CLK->CLKSEL0 = CLK_CLKSEL0_STCLK_S_HCLK_DIV2 | CLK_CLKSEL0_HCLK_S_PLL;

    /* Enable peripheral clock */
    CLK->AHBCLK |= CLK_AHBCLK_PDMA_EN_Msk;
    CLK->APBCLK = CLK_APBCLK_UART0_EN_Msk;

    /* Peripheral clock source */
    CLK->CLKSEL1 = CLK_CLKSEL1_UART_S_PLL;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD, TXD */
    SYS->GPB_MFP = SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD;
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
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(PllClock, 115200);
    UART0->LCR = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %dHz <%d>\n", SystemCoreClock, 0);

    printf("+--------------------------------------+ \n");
    printf("|    NUC123 PDMA Driver Sample Code   | \n");
    printf("+--------------------------------------+ \n");

    /* Open Channel 0 */
    PDMA_GCR->GCRCSR |= (1 << 8);
    /* set transfer byte count(transfer width is 32) */
    PDMA0->BCR = (PDMA_TEST_LENGTH << 2);
    /* set source and destination address */
    PDMA0->SAR = (uint32_t)SrcArray;
    PDMA0->DAR = (uint32_t)DestArray;
    /* set transfer direction */
    PDMA0->CSR = (PDMA0->CSR & ~(PDMA_CSR_SAD_SEL_Msk | PDMA_CSR_DAD_SEL_Msk)) | (PDMA_SAR_INC | PDMA_DAR_INC);
    /* enable block transfer done interrupt */
    PDMA0->IER |= PDMA_IER_BLKD_IE_Msk;
    NVIC_EnableIRQ(PDMA_IRQn);
    u32IsTestOver = 0xFF;
    /* trigger transfer */
    PDMA0->CSR |= (PDMA_CSR_TRIG_EN_Msk | PDMA_CSR_PDMACEN_Msk);
    while(u32IsTestOver == 0xFF);

    if(u32IsTestOver == 0)
        printf("test done...\n");

    PDMA_GCR->GCRCSR = 0;
    while(1);
}




