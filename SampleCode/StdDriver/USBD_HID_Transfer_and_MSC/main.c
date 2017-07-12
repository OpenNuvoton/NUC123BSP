/******************************************************************************
 * @file     main.c
 * @brief
 *           Demonstrate how to implement a composite device. (HID Transfer and Mass storage)
 *           Transfer data between USB device and PC through USB HID interface. 
 *           A windows tool is also included in this sample code to connect with a USB device.
 *           
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC123.h"
#include "HID_Transfer_and_MSC.h"
#include "massstorage.h"

#define CONFIG_BASE      0x00300000
#define DATA_FLASH_BASE  MASS_STORAGE_OFFSET


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

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t au32Config[2];

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init system and multi-function I/O */
    SYS_Init();

    /* Init UART for debug message */
    UART0_Init();

    printf("\n");
    printf("+--------------------------------------------------------------+\n");
    printf("|     NuMicro USB HID Transfer and MassStorage Sample Code     |\n");
    printf("+--------------------------------------------------------------+\n");

    /* Enable FMC ISP function */
    FMC_Open();

    /* Read User Configuration */
    FMC_ReadConfig(au32Config, 2);

    /* Check if Data Flash Size is 36K. If not, to re-define Data Flash size and to enable Data Flash function */
    if(((au32Config[0] & 0x01) == 1) || ((au32Config[0] & 0x04) == 0x4) || (au32Config[1] != DATA_FLASH_BASE))
    {
        FMC_EnableConfigUpdate();
        au32Config[0] &= ~0x5;
        au32Config[1] = DATA_FLASH_BASE;
        FMC_Erase(CONFIG_BASE);
        if(FMC_WriteConfig(au32Config, 2) < 0)
        {
            printf("Error! Fail to write User Configuration.\n");
            /* Disable FMC ISP function */
            FMC_Close();
            return -1;
        }

        FMC_ReadConfig(au32Config, 2);
        if(((au32Config[0] & 0x01) == 1) || ((au32Config[0] & 0x04) == 0x4) || (au32Config[1] != DATA_FLASH_BASE))
        {
            printf("Error! Program User Configuration Failed!\n");
            /* Disable FMC ISP function */
            FMC_Close();
            return -1;
        }

        /* Reset Chip to reload new CONFIG value */
        SYS->IPRSTC1 = SYS_IPRSTC1_CHIP_RST_Msk;
    }
    
    printf("NuMicro USB MassStorage Start!\n");

    /* Open USB controller */
    USBD_Open(&gsInfo, HID_MSC_ClassRequest, NULL);

    USBD_SetConfigCallback(MSC_SetConfig);

    /* Endpoint configuration */
    HID_MSC_Init();
    /* Start USB device */
    USBD_Start();
    NVIC_EnableIRQ(USBD_IRQn);

    while(1)
    {
        MSC_ProcessCmd();
    }
}



/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/

