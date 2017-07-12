/****************************************************************************
 * @file     main.c
 * @version  V2.0
 * $Revision: 8 $
 * $Date: 15/07/02 3:04p $
 * @brief
 *           Demonstrate how to use single cycle scan mode and finishes one cycle of conversion for the specified channels.
 *
 * @note
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NUC123.h"

#define PLLCON_SETTING  CLK_PLLCON_72MHz_HXT
#define PLL_CLOCK       72000000


/*---------------------------------------------------------------------------------------------------------*/
/* Define Function Prototypes                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void);
void UART0_Init(void);
void AdcSingleCycleScanModeTest(void);


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184MHz clock */
    CLK->PWRCON |= CLK_PWRCON_OSC22M_EN_Msk;

    /* Waiting for Internal RC clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_OSC22M_STB_Msk));

    /* Switch HCLK clock source to Internal RC */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLK_S_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_HIRC;

    /* Enable XT1_OUT(PF.0) and XT1_IN(PF.1) */
    SYS->GPF_MFP |= SYS_GPF_MFP_PF0_XT1_OUT | SYS_GPF_MFP_PF1_XT1_IN;

    /* Enable external XTAL 12MHz clock */
    CLK->PWRCON |= CLK_PWRCON_XTL12M_EN_Msk;

    /* Waiting for external XTAL clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_XTL12M_STB_Msk));

    /* Set core clock as PLL_CLOCK from PLL */
    CLK->PLLCON = PLLCON_SETTING;
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_PLL_STB_Msk));
    CLK->CLKSEL0 &= (~CLK_CLKSEL0_HCLK_S_Msk);
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_PLL;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    //SystemCoreClockUpdate();
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For SYS_SysTickDelay()

    /* Enable UART module clock */
    CLK->APBCLK |= CLK_APBCLK_UART0_EN_Msk;

    /* Enable ADC module clock */
    CLK->APBCLK |= CLK_APBCLK_ADC_EN_Msk ;

    /* Select UART module clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UART_S_Msk;
    CLK->CLKSEL1 |= CLK_CLKSEL1_UART_S_HXT;

    /* Select ADC module clock source */
    CLK->CLKSEL1 &= CLK_CLKSEL1_ADC_S_Msk ;
    CLK->CLKSEL1 |= CLK_CLKSEL1_ADC_S_HIRC ;

    /* ADC clock source is 22.1184MHz, set divider to 7, ADC clock is 22.1184/7 MHz */
    CLK->CLKDIV  = (CLK->CLKDIV & ~CLK_CLKDIV_ADC_N_Msk) | (((7) - 1) << CLK_CLKDIV_ADC_N_Pos);


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk);
    SYS->GPB_MFP |= SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD;

    /* Disable the GPD0 - GPD3 digital input path to avoid the leakage current. */
    PD->OFFD = 0xF << GPIO_OFFD_OFFD_Pos;

    /* Configure the GPD0 - GPD3 ADC analog input pins */
    SYS->GPD_MFP &= ~(SYS_GPD_MFP_PD0_Msk | SYS_GPD_MFP_PD1_Msk | SYS_GPD_MFP_PD2_Msk | SYS_GPD_MFP_PD3_Msk) ;
    SYS->GPD_MFP |= SYS_GPD_MFP_PD0_ADC0 | SYS_GPD_MFP_PD1_ADC1 | SYS_GPD_MFP_PD2_ADC2 | SYS_GPD_MFP_PD3_ADC3 ;
    SYS->ALT_MFP1 &= ~(SYS_ALT_MFP1_PD0_Msk | SYS_ALT_MFP1_PD1_Msk | SYS_ALT_MFP1_PD2_Msk | SYS_ALT_MFP1_PD3_Msk);
    SYS->ALT_MFP1 |= SYS_ALT_MFP1_PD0_ADC0 | SYS_ALT_MFP1_PD1_ADC1 | SYS_ALT_MFP1_PD2_ADC2 | SYS_ALT_MFP1_PD3_ADC3;

}

/*---------------------------------------------------------------------------------------------------------*/
/* Init UART                                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_Init()
{
    /* Reset UART IP */
    SYS->IPRSTC2 |=  SYS_IPRSTC2_UART0_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_UART0_RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 115200);
    UART0->LCR = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function: AdcSingleCycleScanModeTest                                                                    */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*   None.                                                                                                 */
/*                                                                                                         */
/* Returns:                                                                                                */
/*   None.                                                                                                 */
/*                                                                                                         */
/* Description:                                                                                            */
/*   ADC single cycle scan mode test.                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void AdcSingleCycleScanModeTest()
{
    uint8_t  u8Option;
    uint32_t u32ChannelCount;
    int32_t  i32ConversionData;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                 ADC single cycle scan mode sample code               |\n");
    printf("+----------------------------------------------------------------------+\n");

    while(1)
    {
        printf("\n\nSelect input mode:\n");
        printf("  [1] Single end input (channel 0, 1, 2 and 3)\n");
        printf("  Other keys: exit single cycle scan mode test\n");
        u8Option = getchar();
        if(u8Option == '1')
        {
            /* Set the ADC operation mode as single-cycle, input mode as single-end and enable the ADC converter  */
            ADC->ADCR = (ADC_ADCR_ADMD_SINGLE_CYCLE | ADC_ADCR_ADEN_CONVERTER_ENABLE);
            /* Enable analog input channel 0, 1, 2 and 3 */
            ADC->ADCHER |= ((ADC->ADCHER & ~ADC_ADCHER_CHEN_Msk) | (0xF));

            /* Clear the A/D interrupt flag for safe */
            ADC->ADSR = ADC_ADSR_ADF_Msk;

            /* Start A/D conversion */
            ADC->ADCR |= ADC_ADCR_ADST_Msk;

            /* Wait conversion done */
            while(!((ADC->ADSR & ADC_ADSR_ADF_Msk) >> ADC_ADSR_ADF_Pos));

            for(u32ChannelCount = 0; u32ChannelCount < 4; u32ChannelCount++)
            {
                i32ConversionData = (ADC->ADDR[(u32ChannelCount)] & ADC_ADDR_RSLT_Msk) >> ADC_ADDR_RSLT_Pos;
                printf("Conversion result of channel %d: 0x%X (%d)\n", u32ChannelCount, i32ConversionData, i32ConversionData);
            }
        }
        else
            return ;

    }
}




/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* Single cycle scan mode test */
    AdcSingleCycleScanModeTest();

    /* Reset ADC module */
    SYS->IPRSTC2 |= (1 << SYS_IPRSTC2_ADC_RST_Pos) ;
    SYS->IPRSTC2 &= ~(1 << (SYS_IPRSTC2_ADC_RST_Pos)) ;

    /* Disable ADC IP clock */
    CLK->APBCLK &= ~CLK_APBCLK_ADC_EN_Msk;

    /* Disable External Interrupt */
    NVIC_DisableIRQ(ADC_IRQn);

    printf("Exit ADC sample code\n");

    while(1);

}

