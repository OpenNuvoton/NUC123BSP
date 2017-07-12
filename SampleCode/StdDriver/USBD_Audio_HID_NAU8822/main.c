/******************************************************************************
 * @file     main.c
 * @brief
 *           Demonstrate how to implement a USB audio class device with HID key.
 *           NAU8822 is used in this sample code to play the audio data from Host.
 *           It also supports to record data from NAU8822 to Host.
 * @note
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC123.h"
#include "usbd_audio.h"

extern uint8_t volatile g_u32EP4Ready;

void HID_UpdateKbData(void);

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
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(I2C0_MODULE);
    CLK_EnableModuleClock(I2C1_MODULE);
    CLK_EnableModuleClock(I2S_MODULE);

    /* Select module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));
    CLK_SetModuleClock(USBD_MODULE, 0, CLK_CLKDIV_USB(3));
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0_S_HXT, 0);
    CLK_SetModuleClock(I2C0_MODULE, 0, 0);
    CLK_SetModuleClock(I2C1_MODULE, 0, 0);
    CLK_SetModuleClock(I2S_MODULE, CLK_CLKSEL2_I2S_S_PLL, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFP = (SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD);

    /* I/O Configure for NuEdu-NUC123 board */
    /* Set PF.2 and PF.3 to be I2C1SDA and I2C1SCL */
    /* Set I2S interface: I2SLRCLK (PC.0), I2SBCLK (PC.1), I2SDI (PC.2), I2SDO (PC.3) */
    /* Set PA.15 as I2SMCLK function pin */
    SYS->GPA_MFP = SYS_GPA_MFP_PA15_I2S_MCLK;
    SYS->GPC_MFP = SYS_GPC_MFP_PC0_I2S_LRCLK | SYS_GPC_MFP_PC1_I2S_BCLK | SYS_GPC_MFP_PC2_I2S_DI | SYS_GPC_MFP_PC3_I2S_DO;
    SYS->GPF_MFP |= SYS_GPF_MFP_PF2_I2C0_SDA | SYS_GPF_MFP_PF3_I2C0_SCL;
    SYS->ALT_MFP = SYS_ALT_MFP_PC0_I2S_LRCLK | SYS_ALT_MFP_PC1_I2S_BCLK | SYS_ALT_MFP_PC2_I2S_DI | SYS_ALT_MFP_PC3_I2S_DO |
                   SYS_ALT_MFP_PA10_I2C1_SDA | SYS_ALT_MFP_PA11_I2C1_SCL | SYS_ALT_MFP_PA15_I2S_MCLK;
    SYS->ALT_MFP1 = SYS_ALT_MFP1_PF2_I2C0_SDA | SYS_ALT_MFP1_PF3_I2C0_SCL;

}

void UART0_Init(void)
{
    /* Reset IP */
    SYS_ResetModule(UART0_RST);
    UART0->LCR = 0x3;
    UART0->BAUD = 0x30000066;
}

void I2C_Init(void)
{
    /* Open I2C and set clock to 100k */
    I2C_Open(I2C_PORT, 100000);

    /* Get I2C Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C_PORT));
}

void KEY_Init(void)
{
    /* Init key I/O */
    PB->PMD |= (0x3ul << 3 * 2) | (0x3ul << 12 * 2) | (0x3ul << 13 * 2) | (0x3ul << 8 * 2) | (0x3ul << 10 * 2);
    PB3 = 1;    // Play/pause
    PB12 = 1;   // Next
    PB13 = 1;   // Previous
    PB8 = 1;    // Volumn Up
    PB10 = 1;   // Volumn Down

    /* Enable Debounce and set debounce time */
    PB->DBEN = (1 << 3) | (1 << 12) | (1 << 13) | (1 << 8) | (1 << 10);
    GPIO->DBNCECON = 0x17; // ~12.8 ms

}



/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    int32_t i;

    /*
        This sample code is used to demo USB Audio Class + NAU8822 (WAU8822) with HID key.
        User can define PLAY_RATE in usbd_audio.h to support 48000Hz, 32000Hz, 16000Hz and 8000Hz.

        The audio is input from NAU8822 AUXIN.
        The audio is output by NAU8822 Headphone output.

        NAU8822 is connect with I2S (PC.0~PC.3) and controlled by I2C0 (PF.2, PF.3).
        NAU8822 clock source is also come from I2S (MCLK, PA.15).
        Headphone MUTE control by PC13.

        HID key could be configured as HID keyboard or HID consumer (Media key). Default is HID consumer.
        (Defined in usbd_audio.h)

        keyboard:
        PB3 ==> 'a'
        PB12 ==> 'b'

        consumber:
        PB3  ==> Play/Pause
        PB12 ==> Next
        PB13 ==> Previous
        PB8  ==> Vol+
        PB10 ==> Vol-
    */


    /* Unlock Protected Regsiter */
    SYS_UnlockReg();

    /* Initial system & multi-function */
    SYS_Init();

    /* Initial UART0 for debug message */
    UART0_Init();

    /* Init HID key */
    KEY_Init();

    printf("\n");
    printf("+-------------------------------------------------------+\n");
    printf("|          NuMicro USB Audio CODEC Sample Code          |\n");
    printf("+-------------------------------------------------------+\n");

    /* Init I2C to access WAU8822 */
    I2C_Init();

    /* Initialize WAU8822 codec */
    WAU8822_Setup();

    /* Headphone MUTE off */
    PC->PMD |= (3 << 13 * 2);
    PC13 = 0;

    I2S_Open(I2S, I2S_MODE_SLAVE, PLAY_RATE, I2S_DATABIT_16, I2S_STEREO, I2S_FORMAT_I2S);

    /* Set MCLK and enable MCLK */
    I2S_EnableMCLK(I2S, 12000000);

    /* Fill dummy data to I2S TX for start I2S iteration */
    for(i = 0; i < 8; i++)
        I2S_WRITE_TX_FIFO(I2S, 0);

    /* Start I2S play iteration */
    I2S_EnableInt(I2S, I2S_IE_TXTHIE_Msk | I2S_IE_RXTHIE_Msk);

    USBD_Open(&gsInfo, UAC_ClassRequest, (SET_INTERFACE_REQ)UAC_SetInterface);
    /* Endpoint configuration */
    UAC_Init();
    USBD_Start();
    NVIC_EnableIRQ(USBD_IRQn);
    NVIC_EnableIRQ(I2S_IRQn);

    /* I2S interrupt has higher frequency then USBD interrupt.
       Therefore, we need to set I2S with higher priority to avoid
       I2S interrupt pending too long time when USBD interrupt happen. */
    NVIC_SetPriority(USBD_IRQn, 3);
    NVIC_SetPriority(I2S_IRQn, 2);


    /* start to IN data */
    g_u32EP4Ready = 1;


    while(SYS->PDID)
    {
        uint8_t ch;
        uint32_t u32Reg, u32Data;
        extern int32_t kbhit(void);

        /* Adjust codec sampling rate to synch with USB. The adjustment range is +-0.005% */
        AdjFreq();

        /* Set audio volume according USB volume control settings */
        VolumnControl();

        /* User can change audio codec settings by I2C at run-time if necessary */
        if(!kbhit())
        {
            printf("\nEnter codec setting:\n");
            // Get Register number
            ch = getchar();
            u32Reg = ch - '0';
            ch = getchar();
            u32Reg = u32Reg * 10 + (ch - '0');
            printf("%d\n", u32Reg);

            // Get data
            ch = getchar();
            u32Data = (ch >= '0' && ch <= '9') ? ch - '0' : ch - 'a' + 10;
            ch = getchar();
            u32Data = u32Data * 16 + ((ch >= '0' && ch <= '9') ? ch - '0' : ch - 'a' + 10);
            ch = getchar();
            u32Data = u32Data * 16 + ((ch >= '0' && ch <= '9') ? ch - '0' : ch - 'a' + 10);
            printf("%03x\n", u32Data);
            I2C_WriteWAU8822(u32Reg,  u32Data);
        }

        /* HID Keyboard */
        HID_UpdateKbData();

    }
}

void HID_UpdateKbData(void)
{
    int32_t i;
    uint8_t *buf;
    uint32_t key = 0xF;
    static uint32_t preKey;
    int32_t n;

    n = 8;
    if(g_u32EP4Ready)
    {
        buf = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP4));

        // PB3, play/pause
        // PB12, Next
        // PB13, Previous
        // PB8, Vol+
        // PB10, Vol-
        key = !PB3 | (!PB12 << 1) | (!PB13 << 1) | (!PB8 << 1) | (!PB10 << 1);

        if(key == 0)
        {
            for(i = 0; i < n; i++)
            {
                buf[i] = 0;
            }

            if(key != preKey)
            {
                preKey = key;
                g_u32EP4Ready = 0;
                /* Trigger to note key release */
                USBD_SET_PAYLOAD_LEN(EP4, n);
            }
        }
        else
        {

#if(HID_FUNCTION == HID_KEYBOARD)
            preKey = key;

            if(!PB3)
                buf[2] = 0x04; /* Key A */
            else if(!PB12)
                buf[2] = 0x05;

            g_u32EP4Ready = 0;
            USBD_SET_PAYLOAD_LEN(EP4, n);

#elif(HID_FUNCTION == HID_CONSUMER)
            // Don't repeat key when it is media key
            if(preKey != key)
            {
                preKey = key;
                buf[0] = 0;
                buf[1] = 0;
                if(!PB3)
                    buf[1] |= HID_CTRL_PAUSE;
                else if(!PB12)
                    buf[1] |= HID_CTRL_NEXT;
                else if(!PB13)
                    buf[1] |= HID_CTRL_PREVIOUS;
                else if(!PB8)
                    buf[0] |= HID_CTRL_VOLUME_INC;
                else if(!PB10)
                    buf[0] |= HID_CTRL_VOLUME_DEC;

                g_u32EP4Ready = 0;
                USBD_SET_PAYLOAD_LEN(EP4, n);
            }

#endif

        }
    }
}

/*** (C) COPYRIGHT 2014~2015 Nuvoton Technology Corp. ***/

