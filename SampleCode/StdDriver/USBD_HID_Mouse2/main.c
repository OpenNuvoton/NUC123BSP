/******************************************************************************
 * @file     main.c
 * @brief
 *           Demonstrate how to implement a USB mouse device.
 *           It use PC0 ~ PC5 to control mouse direction and mouse key.
 *           It also supports USB suspend and remote wakeup.
 * @note
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC123.h"
#include "hid_mouse.h"


/*--------------------------------------------------------------------------*/
void SYS_Init(void)
{

    /* Enable XT1_OUT (PF.0) and XT1_IN (PF.1) */
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
    SYS->GPB_MFP |= (SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD);
    /* Set PC.13 as CLKO function pin */
    SYS->GPC_MFP = SYS_GPC_MFP_PC13_CLKO;
    SYS->ALT_MFP = SYS_ALT_MFP_PC13_CLKO;

    /* Enable CLKO (PC.13) for monitor HCLK. CLKO = HCLK/8 Hz*/
    CLK_EnableCKO(CLK_CLKSEL2_FRQDIV_S_HCLK, 2, NULL);
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

void GPIO_Init(void)
{
    /* Enable PC0~5 interrupt for wakeup */

    PC->ISRC |= 0x3f;
    PC->IEN |= 0x3f | (0x3f << 16);
    PC->DBEN |= 0x3f;      // Enable key debounce
    GPIO->DBNCECON = 0x16; // Debounce time is about 6ms
    NVIC_EnableIRQ(GPCDF_IRQn);
}


void GPCDF_IRQHandler(void)
{
    PC->ISRC = 0x3f;
    //PB4 ^= 1;

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    SYS_Init();

    UART0_Init();

    GPIO_Init();

    printf("\n");
    printf("+-----------------------------------------------------+\n");
    printf("|          NuMicro USB HID Mouse Sample Code          |\n");
    printf("+-----------------------------------------------------+\n");

    /* This sample code is used to simulate a mouse with suspend and remote wakeup supported.
       User can use PC0~PC5 key to control the moviement of mouse.
       PB4 is used as a LED to show working status.
    */

    USBD_Open(&gsInfo, HID_ClassRequest, NULL);


    /* Endpoint configuration */
    HID_Init();
    USBD_Start();
    NVIC_EnableIRQ(USBD_IRQn);


    PB4 = 0; // LED to show system is on line
    while(1)
    {
        HID_UpdateMouseData();
    }
}



/*** (C) COPYRIGHT 2014~2015 Nuvoton Technology Corp. ***/

