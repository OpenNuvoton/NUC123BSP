/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to minimize power consumption when entering power down mode.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC123.h"


#define HCLK_CLOCK          72000000
#define GPIO_P0_TO_P15      0xFFFF


/*
//-------- <<< Use Configuration Wizard in Context Menu >>> -----------------
*/

/*
// <o0> LVR
//      <0=> Disable
//      <1=> Enable
*/
#define SET_LVR       1

/*
// <o0> POR
//      <0=> Disable
//      <1=> Enable
*/
#define SET_POR       0

/*
// <o0> LIRC
//      <0=> Disable
//      <1=> Enable
*/
#define SET_LIRC      0

/*
// <o0> LXT
//      <0=> Disable
//      <1=> Enable
*/
#define SET_LXT       0


void PowerDownFunction(void);
void GPAB_IRQHandler(void);
void LvrSetting(void);
void PorSetting(void);
int32_t LircSetting(void);
void SYS_Init(void);
void UART0_Init(void);

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    uint32_t u32TimeOutCnt;

    /* Check if all the debug messages are finished */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    UART_WAIT_TX_EMPTY(DEBUG_PORT)
        if(--u32TimeOutCnt == 0) break;

    /* Enter to Power-down mode */
    CLK_PowerDown();
}

/**
 * @brief       GPIO PA/PB IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The PA/PB default IRQ, declared in startup_NUC123Series.s.
 */
void GPAB_IRQHandler(void)
{
    /* To check if PB.5 interrupt occurred */
    if(GPIO_GET_INT_FLAG(PB, BIT5))
    {
        GPIO_CLR_INT_FLAG(PB, BIT5);
        printf("PB.5 INT occurred.\n");
    }
    else
    {
        /* Un-expected interrupt. Just clear all PA and PB interrupts */
        PA->ISRC = PA->ISRC;
        PB->ISRC = PB->ISRC;
        printf("Un-expected interrupts.\n");
    }
}


void LvrSetting(void)
{
    if(SET_LVR == 0)
    {
        /* Disable LVR */
        SYS_DISABLE_LVR();
        CLK_SysTickDelay(200);
    }
    else
    {
        /* Enable LVR */
        SYS_ENABLE_LVR();
        CLK_SysTickDelay(200);
    }
}

void PorSetting(void)
{
    if(SET_POR == 0)
    {
        /* Disable POR */
        SYS_DISABLE_POR();
    }
    else
    {
        /* Enable POR */
        SYS_ENABLE_POR();
    }
}

int32_t LircSetting(void)
{
    uint32_t u32TimeOutCnt;

    if(SET_LIRC == 0)
    {
        /* Disable LIRC and wait for LIRC stable flag is cleared */
        CLK_DisableXtalRC(CLK_PWRCON_OSC10K_EN_Msk);
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while( CLK->CLKSTATUS & CLK_CLKSTATUS_IRC10K_STB_Msk )
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for LIRC disable time-out!\n");
                return -1;
            }
        }
    }
    else
    {
        /* Enable LIRC and wait for LIRC stable flag is set */
        CLK_EnableXtalRC(CLK_PWRCON_OSC10K_EN_Msk);
        if( CLK_WaitClockReady(CLK_CLKSTATUS_OSC10K_STB_Msk) == 0)
        {
            printf("Wait for LIRC enable time-out!\n");
            return -1;
        }
    }

    return 0;
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable XT1_OUT(PF0) and XT1_IN(PF1) */
    SYS->GPF_MFP &= ~(SYS_GPF_MFP_PF0_Msk | SYS_GPF_MFP_PF1_Msk);
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

    /* Set core clock as HCLK_CLOCK */
    CLK_SetCoreClock(HCLK_CLOCK);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set GPB multi-function pins for UART0 RXD(PB.0) and TXD(PB.1) */
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk);
    SYS->GPB_MFP |= (SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD);

}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-------------------------------------------------------------------+\n");
    printf("|  SYS_PowerDown_MinCurrent and Wake-up by PB.5 Sample Code         |\n");
    printf("+-------------------------------------------------------------------+\n\n");

    printf("+-------------------------------------------------------------------+\n");
    printf("| Operating sequence                                                |\n");
    printf("|  1. Remove all continuous load, e.g. LED.                         |\n");
    printf("|  2. Configure all GPIO as Quasi-bidirectional Mode                |\n");
    printf("|  3. Disable analog function, e.g. POR module                      |\n");
    printf("|  4. Disable unused clock, e.g. LIRC                               |\n");
    printf("|  5. Enter to Power-Down                                           |\n");
    printf("|  6. Wait for PB.5 falling-edge interrupt event to wake-up the MCU |\n");
    printf("+-------------------------------------------------------------------+\n\n");

    /* Set function pin to GPIO mode except UART pin to print message */
    SYS->GPA_MFP = 0;
    SYS->GPB_MFP = SYS_GPB_MFP_PB1_UART0_TXD;
    SYS->GPC_MFP = 0;
    SYS->GPD_MFP = 0;
    SYS->GPF_MFP = 0;
    SYS->ALT_MFP = 0;
    SYS->ALT_MFP1 = 0;

    /* Configure all GPIO as Quasi-bidirectional Mode. They ard default output high. */
    GPIO_SetMode(PA, GPIO_P0_TO_P15, GPIO_PMD_QUASI);
    GPIO_SetMode(PB, GPIO_P0_TO_P15, GPIO_PMD_QUASI);
    GPIO_SetMode(PC, GPIO_P0_TO_P15, GPIO_PMD_QUASI);
    GPIO_SetMode(PD, GPIO_P0_TO_P15, GPIO_PMD_QUASI);
    GPIO_SetMode(PF, GPIO_P0_TO_P15, GPIO_PMD_QUASI);

    /* Unlock protected registers for Power-down setting */
    SYS_UnlockReg();

    /* LVR setting */
    LvrSetting();

    /* POR setting */
    PorSetting();

    /* LIRC setting */
    if( LircSetting() < 0 ) goto lexit;

    /* Wake-up source configuration */
    /* Configure PB.5 as Quasi mode and enable interrupt by falling edge trigger */
    GPIO_SetMode(PB, BIT5, GPIO_PMD_QUASI);
    GPIO_EnableInt(PB, 5, GPIO_INT_FALLING);
    NVIC_EnableIRQ(GPAB_IRQn);

    /* Enter to Power-down mode */
    printf("Enter to Power-Down ......\n");
    PowerDownFunction();

    /* Waiting for PB.5 falling-edge interrupt event */
    printf("System waken-up done.\n\n");

lexit:

    while(1);
}
