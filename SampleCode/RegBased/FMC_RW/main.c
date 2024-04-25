/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 6 $
 * $Date: 15/07/02 11:17a $
 * @brief    Demonstrate how to read/program embedded flash by ISP function.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NUC123.h"

#define PLLCON_SETTING      CLK_PLLCON_72MHz_HXT
#define PLL_CLOCK           72000000

int32_t g_FMC_i32ErrCode;

void SYS_Init(void)
{
    int32_t i;
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable  XT1_OUT(PF0) and XT1_IN(PF1) */
    SYS->GPF_MFP &= ~(SYS_GPF_MFP_PF0_Msk | SYS_GPF_MFP_PF1_Msk);
    SYS->GPF_MFP |= SYS_GPF_MFP_PF0_XT1_OUT | SYS_GPF_MFP_PF1_XT1_IN;

    /* Enable External XTAL (4~24 MHz) */
    CLK->PWRCON |= CLK_PWRCON_XTL12M_EN_Msk;

    CLK->PLLCON = PLLCON_SETTING;

    /* Waiting for clock ready */
    i = 22000000; // For timeout
    while(i-- > 0)
    {
        if((CLK->CLKSTATUS & (CLK_CLKSTATUS_PLL_STB_Msk | CLK_CLKSTATUS_XTL12M_STB_Msk)) ==
                (CLK_CLKSTATUS_PLL_STB_Msk | CLK_CLKSTATUS_XTL12M_STB_Msk))
            break;
    }

    /* Switch HCLK clock source to PLL */
    CLK->CLKSEL0 = CLK_CLKSEL0_HCLK_S_PLL;

    /* Enable IP clock */
    CLK->APBCLK = CLK_APBCLK_UART0_EN_Msk;

    /* Select IP clock source */
    CLK->CLKSEL1 = CLK_CLKSEL1_UART_S_PLL;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CyclesPerUs automatically. */
    //SystemCoreClockUpdate();
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For CLK_SysTickDelay()

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPB multi-function pins for UART0 RXD and TXD  */
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk);
    SYS->GPB_MFP |= SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD;
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
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(PLL_CLOCK, 115200);
    UART0->LCR = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

int main()
{
    char *cBootMode[] = {"LDROM+IAP", "LDROM", "APROM+IAP", "APROM"};
    uint32_t u32CBS;
    uint32_t u32Data, u32RData;
    uint32_t u32Addr;
    uint32_t u32Cfg0, u32Cfg1;

    /* Disable register write-protection function */
    SYS_UnlockReg();

    /* Initial clocks and multi-functions */
    SYS_Init();

    /* Initial UART */
    UART0_Init();

    printf("\n\n");
    printf("+----------------------------------------+\n");
    printf("|          NUC123 FMC Sample Code        |\n");
    printf("+----------------------------------------+\n");

    /* Enable FMC ISP functions */
    FMC->ISPCON |=  FMC_ISPCON_ISPEN_Msk | FMC_ISPCON_APUEN_Msk | FMC_ISPCON_LDUEN_Msk | FMC_ISPCON_CFGUEN_Msk;

    /* Check current boot mode */
    u32CBS = (FMC->ISPSTA & FMC_ISPSTA_CBS_Msk) >> FMC_ISPSTA_CBS_Pos;
    printf("  Current Boot Mode is ....................... [%s]\n", cBootMode[u32CBS]);

    /* Show the user configuration */
    u32Cfg0 = FMC_Read(FMC_CONFIG_BASE);
    u32Cfg1 = FMC_Read(FMC_CONFIG_BASE + 4);
    printf("  CFG0 ....................................... [0x%08x]\n", u32Cfg0);
    printf("  CFG1 ....................................... [0x%08x]\n", u32Cfg1);

    /* Read UID */
    printf("  UID[31: 0] ................................. [0x%08x]\n", FMC_ReadUID(0));
    printf("  UID[63:32] ................................. [0x%08x]\n", FMC_ReadUID(1));
    printf("  UID[95:64] ................................. [0x%08x]\n", FMC_ReadUID(2));

    /* The ROM address for erase/write/read demo */
    u32Addr = 0x4000;

    /* Erase Demo */
    FMC_Erase(u32Addr);

    /* Write Demo */
    u32Data = 0x12345678;
    FMC_Write(u32Addr, u32Data);

    /* Read Demo */
    u32RData = FMC_Read(u32Addr);

    printf("  Write %08x to 0x%08x ............... ", u32Addr, u32Data);
    if(u32Data == u32RData)
        printf("[OK]\n");
    else
        printf("[FAIL]\n");

    /* Disable FMC ISP function */
    FMC->ISPCON &=  ~FMC_ISPCON_ISPEN_Msk;

    /* Lock protected registers */
    SYS_LockReg();

    printf("\nFMC Sample Code Completed.\n");

    while(SYS->PDID);
}

/*** (C) COPYRIGHT 2014~2015 Nuvoton Technology Corp. ***/
