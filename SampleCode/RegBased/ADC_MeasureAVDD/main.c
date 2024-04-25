/****************************************************************************
 * @file     main.c
 * @version  V2.0
 * $Revision: 1 $
 * $Date: 16/06/22 10:08a $
 * @brief
 *           Measure AVDD voltage by ADC.
 *
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 *
 * @copyright Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC123.h"

#define PLLCON_SETTING  CLK_PLLCON_72MHz_HXT
#define PLL_CLOCK       72000000

#define VBG_VOLTAGE (1200) /* 1.20V = 1200 mV (Typical band-gap voltage) */
#define ADC_SAMPLE_COUNT 128 /* The last line of GetAVDDCodeByADC() need revise when ADC_SAMPLE_COUNT is changed. */
/* For example, if ADC_SAMPLE_COUNT is changed to 64, then the code need revised to "return (u32Sum >> 6);" */

/*---------------------------------------------------------------------------------------------------------*/
/* Define Function Prototypes                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void);
void UART0_Init(void);
void AdcMeasureAVDD(void);
uint32_t GetAVDDCodeByADC(void);
uint32_t GetAVDDVoltage(void);


/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t g_u8ADF;



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
    SYS->GPF_MFP &= ~(SYS_GPF_MFP_PF0_Msk | SYS_GPF_MFP_PF1_Msk);
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
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CyclesPerUs automatically. */
    //SystemCoreClockUpdate();
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For CLK_SysTickDelay()

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
/* Function: GetAVDDVoltage                                                                                */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*   None.                                                                                                 */
/*                                                                                                         */
/* Returns:                                                                                                */
/*   AVDD voltage(mV).                                                                                     */
/*                                                                                                         */
/* Description:                                                                                            */
/*   Use Band-gap voltage to calculate AVDD voltage                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t GetAVDDVoltage(void)
{
    uint32_t  u32ConversionResult;
    uint64_t u64MvAVDD;

    /* Calculate Vref by using conversion result of VBG */
    u32ConversionResult = GetAVDDCodeByADC();

    /* u32ConversionResult = VBG * 1024 / Vref, Vref = AVDD */
    /* => AVDD = VBG * 1024 / u32ConversionResult */
    u64MvAVDD = (VBG_VOLTAGE << 10) / (uint64_t)u32ConversionResult;

    printf("Conversion result: 0x%X\n", u32ConversionResult);

    return (uint32_t)u64MvAVDD;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function: GetAVDDCodeByADC                                                                              */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*   None.                                                                                                 */
/*                                                                                                         */
/* Returns:                                                                                                */
/*   ADC code of AVDD voltage.                                                                             */
/*                                                                                                         */
/* Description:                                                                                            */
/*   Get ADC conversion result of Band-gap voltage.                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t GetAVDDCodeByADC(void)
{
    uint32_t u32Count, u32Sum, u32Data, u32TimeOutCnt;

    /* Configure ADC: single-end input, single scan mode, enable ADC analog circuit. */
    ADC->ADCR = (ADC_ADCR_ADMD_SINGLE | ADC_ADCR_ADEN_CONVERTER_ENABLE);
    /* Configure the analog input source of channel 7 as internal band-gap voltage */
    ADC->ADCHER = ((ADC->ADCHER & ~(ADC_ADCHER_CHEN_Msk|ADC_ADCHER_PRESEL_Msk)) | ((1 << 7)|ADC_ADCHER_PRESEL_INT_BANDGAP));

    /* Power on ADC */
    ADC->ADCR |= ADC_ADCR_ADEN_Msk;

    /* Clear conversion finish flag */
    ADC->ADSR = ADC_ADSR_ADF_Msk;

    /* Enable ADC conversion finish interrupt */
    ADC->ADCR |= ADC_ADCR_ADIE_Msk;
    NVIC_EnableIRQ(ADC_IRQn);

    g_u8ADF = 0;
    u32Sum = 0;

    /* sample times are according to ADC_SAMPLE_COUNT definition */
    for(u32Count = 0; u32Count < ADC_SAMPLE_COUNT; u32Count++)
    {
        /* Delay for band-gap voltage stability */
        CLK_SysTickDelay(100);

        /* Start A/D conversion */
        ADC->ADCR |= ADC_ADCR_ADST_Msk;

        u32Data = 0;

        /* Wait conversion done */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while(g_u8ADF == 0)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for ADC conversion done time-out!\n");
                return 0;
            }
        }
        g_u8ADF = 0;
        /* Get the conversion result */
        u32Data = (ADC->ADDR[(7)] & ADC_ADDR_RSLT_Msk) >> ADC_ADDR_RSLT_Pos;
        /* Sum each conversion data */
        u32Sum += u32Data;
    }
    /* Disable ADC interrupt */
    ADC->ADCR &= ~ADC_ADCR_ADIE_Msk;
    /* Disable ADC */
    ADC->ADCR &= ~ADC_ADCR_ADEN_Msk;

    /* Return the average of ADC_SAMPLE_COUNT samples */
    return (u32Sum >> 7);
}



/*---------------------------------------------------------------------------------------------------------*/
/* ADC interrupt handler                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void ADC_IRQHandler(void)
{
    uint32_t u32Flag;

    /* Get ADC conversion finish interrupt flag */
    u32Flag = ADC->ADSR;

    /* Check ADC conversion finish */
    if(u32Flag & ADC_ADF_INT)
        g_u8ADF = 1;

    /* Clear conversion finish flag */
    ADC->ADSR = ADC_ADSR_ADF_Msk;
}


/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32AVDDVoltage;

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

    printf("\nSystem clock rate: %d Hz\n", SystemCoreClock);

    printf("+----------------------------------------------------------------------+\n");
    printf("|                 ADC for AVDD Measurement sample code                 |\n");
    printf("+----------------------------------------------------------------------+\n");

    printf("\nIn this sample code, software will get voltage value from AVDD.\n");

    /*------------------------------------------------------------------------------------------------------------------
       The method of measured AVDD voltage is using ADC to get conversion result of band-gap voltage.

       For example, the typical value of band-gap voltage is 1.20 V, and Vref of ADC is from AVDD.
       Through getting ADC conversion result of band-gap voltage, then AVDD voltage can be calculated by below formula:

           ConversionResult = VBG * 1024 / Vref, Vref = AVDD and VBG = 1.20V
           => AVDD = 1.20V * 1024 / ConversionResult


       Note 1 : The measured AVDD has deviation that causes by the band-gap voltage has deviation in different temperature, power voltage and ADC conversion deviation.(4 LSB)
                The deviation of measured AVDD is list as follows:

                The Spec. of band-gap voltage in NUC123 series is as follows:
                -----------------------------------------------------------------------------------------
                |                  | Min.   | Typ.   | Max.   |                                         |
                |                  |--------------------------- VDD = 2.5 V ~ 5.5 V                     |
                | band-gap voltage | 1.18 V | 1.20 V | 1.22 V | temperature = -40 ~ 110 degrees Celsius |
                |                  |        |        |        |                                         |
                -----------------------------------------------------------------------------------------

                Deviation range of measured AVDD
                ----------------------------------------------------
                |                | Min. Deviation | Max. Deviation |
                |                |                |                |
                |                | VBG = 1.18 V   | VBG = 1.22 V   |
                |--------------------------------------------------|
                |  AVDD = 2.5 V  |   -4.57%       |    4.86%       |
                |--------------------------------------------------|
                |  AVDD = 5.5 V  |   -7.85%       |    8.98%       |
                ----------------------------------------------------


       Note 2: The typical value of band-gap voltage can be modified by VBG_VOLTAGE definition.

    ------------------------------------------------------------------------------------------------------------------*/
    /* Measure AVDD */
    u32AVDDVoltage = GetAVDDVoltage();
    printf("AVDD Voltage: %dmV\n", u32AVDDVoltage);

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

