/******************************************************************************
 * @file     main.c
 * @brief
 *           Demonstrate how to implement a USB virtual com port device.
 *           It supports one virtual comport.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC123.h"
#include "cdc_serial.h"

/*--------------------------------------------------------------------------*/
STR_VCOM_LINE_CODING gLineCoding = {115200, 0, 0, 8};   /* Baud rate : 115200    */
/* Stop bit     */
/* parity       */
/* data bits    */
uint16_t gCtrlSignal = 0;     /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */

/*--------------------------------------------------------------------------*/
#define RXBUFSIZE           512 /* RX buffer size */
#define TXBUFSIZE           512 /* RX buffer size */


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t comRbuf[RXBUFSIZE];
volatile uint16_t comRbytes = 0;
volatile uint16_t comRhead = 0;
volatile uint16_t comRtail = 0;

volatile uint8_t comTbuf[TXBUFSIZE];
volatile uint16_t comTbytes = 0;
volatile uint16_t comThead = 0;
volatile uint16_t comTtail = 0;

uint8_t gRxBuf[64] = {0};
volatile uint8_t *gpu8RxBuf = 0;
volatile uint32_t gu32RxSize = 0;
volatile uint32_t gu32TxSize = 0;

volatile int8_t gi8BulkOutReady = 0;

void SYS_Init(void)
{

    /* Enable XT1_OUT (PF.0) and XT1_IN (PF.1) */
    SYS->GPF_MFP &= ~(SYS_GPF_MFP_PF0_Msk | SYS_GPF_MFP_PF1_Msk);
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
    CLK_EnableModuleClock(UART_MODULE);
    CLK_EnableModuleClock(USBD_MODULE);

    /* Select module clock source */
    CLK_SetModuleClock(UART_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));
    CLK_SetModuleClock(USBD_MODULE, 0, CLK_CLKDIV_USB(3));


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set GPB multi-function pins for UART RXD and TXD */
    UART_MFP_INIT();
    
    
    /* Set PC.13 as CLKO function pin */
    SYS->GPC_MFP &= ~(SYS_GPC_MFP_PC13_CLKO);
    SYS->GPC_MFP |= SYS_GPC_MFP_PC13_CLKO;
    SYS->ALT_MFP &= ~(SYS_ALT_MFP_PC13_CLKO);
    SYS->ALT_MFP |= SYS_ALT_MFP_PC13_CLKO;

    /* Enable CLKO (PC.13) for monitor HCLK. CLKO = HCLK/8 Hz*/
    CLK_EnableCKO(CLK_CLKSEL2_FRQDIV_S_HCLK, 2, (uint32_t)NULL);
}


void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS->IPRSTC2 |=  SYS_IPRSTC2_UART0_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_UART0_RST_Msk;
    SYS->IPRSTC2 |=  SYS_IPRSTC2_UART1_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_UART1_RST_Msk;

    __NOP();
    __NOP();
    /* Configure UART and set UART Baudrate */
    UART->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 115200);
    UART->LCR = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;

    /* Enable UART Interrupt */
    UART->IER = UART_IER_RTO_IEN_Msk | UART_IER_RDA_IEN_Msk;

}


/*---------------------------------------------------------------------------------------------------------*/
/* UART Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART_IRQHandler(void)
{
    uint32_t u32IntStatus;
    uint8_t bInChar;
    int32_t size;

    u32IntStatus = UART->ISR;

    if((u32IntStatus & 0x1 /* RDAIF */) || (u32IntStatus & 0x10 /* TOUT_IF */))
    {
        /* Receiver FIFO threshold level is reached or RX time out */

        /* Get all the input characters */
        while((UART->FSR & UART_FSR_RX_EMPTY_Msk) == 0)
        {
            /* Get the character from UART Buffer */
            bInChar = UART->DATA;

            /* Check if buffer full */
            if(comRbytes < RXBUFSIZE)
            {
                /* Enqueue the character */
                comRbuf[comRtail++] = bInChar;
                if(comRtail >= RXBUFSIZE)
                    comRtail = 0;
                comRbytes++;
            }
            else
            {
                /* FIFO over run */
            }
        }
    }

    if(u32IntStatus & 0x2 /* THRE_IF */)
    {

        if(comTbytes && (UART->IER & UART_IER_THRE_IEN_Msk))
        {
            /* Fill the TX FIFO */
            size = comTbytes;
            if(size >= UART0_FIFO_SIZE)
            {
                size = UART0_FIFO_SIZE;
            }

            while(size)
            {
                bInChar = comTbuf[comThead++];
                UART->DATA = bInChar;
                if(comThead >= TXBUFSIZE)
                    comThead = 0;
                comTbytes--;
                size--;
            }
        }
        else
        {
            /* No more data, just stop TX (Stop work) */
            UART->IER &= (~UART_IER_THRE_IEN_Msk);
        }
    }

}

void VCOM_TransferData(void)
{
    int32_t i, i32Len;

    /* Check whether USB is ready for next packet or not */
    if(gu32TxSize == 0)
    {
        /* Check whether we have new COM Rx data to send to USB or not */
        if(comRbytes)
        {
            i32Len = comRbytes;
            if(i32Len > EP2_MAX_PKT_SIZE)
                i32Len = EP2_MAX_PKT_SIZE;

            for(i = 0; i < i32Len; i++)
            {
                gRxBuf[i] = comRbuf[comRhead++];
                if(comRhead >= RXBUFSIZE)
                    comRhead = 0;
            }

            __set_PRIMASK(1);
            comRbytes -= i32Len;
            __set_PRIMASK(0);

            gu32TxSize = i32Len;
            USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2)), (uint8_t *)gRxBuf, i32Len);
            USBD_SET_PAYLOAD_LEN(EP2, i32Len);
        }
        else
        {
            /* Prepare a zero packet if previous packet size is EP2_MAX_PKT_SIZE and 
               no more data to send at this moment to note Host the transfer has been done */
            i32Len = USBD_GET_PAYLOAD_LEN(EP2);
            if(i32Len == EP2_MAX_PKT_SIZE)
                USBD_SET_PAYLOAD_LEN(EP2, 0);
        }
    }

    /* Process the Bulk out data when bulk out data is ready. */
    if(gi8BulkOutReady && (gu32RxSize <= TXBUFSIZE - comTbytes))
    {
        for(i = 0; i < gu32RxSize; i++)
        {
            comTbuf[comTtail++] = gpu8RxBuf[i];
            if(comTtail >= TXBUFSIZE)
                comTtail = 0;
        }

        __set_PRIMASK(1);
        comTbytes += gu32RxSize;
        __set_PRIMASK(0);

        gu32RxSize = 0;
        gi8BulkOutReady = 0; /* Clear bulk out ready flag */

        /* Ready to get next BULK out */
        USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
    }

    /* Process the software TX FIFO */
    if(comTbytes)
    {
        /* Check if TX is working */
        if((UART->IER & UART_IER_THRE_IEN_Msk) == 0)
        {
            /* Send one bytes out */
            UART->DATA = comTbuf[comThead++];
            if(comThead >= TXBUFSIZE)
                comThead = 0;

            comTbytes--;

            /* Enable TX Empty Interrupt. (Trigger first one) */
            UART->IER |= UART_IER_THRE_IEN_Msk;
        }
    }
}

void PowerDown()
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Wakeup Enable */
    USBD_ENABLE_INT(USBD_INTEN_WAKEUP_EN_Msk);

    CLK_PowerDown();

    /* Clear PWR_DOWN_EN if it is not clear by itself */
    if(CLK->PWRCON & CLK_PWRCON_PWR_DOWN_EN_Msk)
        CLK->PWRCON ^= CLK_PWRCON_PWR_DOWN_EN_Msk;

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    SYS_Init();
    UART_Init();

#if(UART_PORT == 0)    
    printf("\n\n");
    printf("+------------------------------------------------------------+\n");
    printf("|          NuMicro USB Virtual COM Port Sample Code          |\n");
    printf("+------------------------------------------------------------+\n");
    printf("Set PB.0 as UART RX pin and PB.1 as UART TX pin\n");
#endif    

    /* Open USB controller */
    USBD_Open(&gsInfo, VCOM_ClassRequest, NULL);

    /* Endpoint configuration */
    VCOM_Init();
    /* Start USB device */
    USBD_Start();
    NVIC_EnableIRQ(USBD_IRQn);
    NVIC_EnableIRQ(UART_IRQn);

    while(1)
    {
        /* Enter power down when USB suspend */
        if(g_u8Suspend)
            PowerDown();

        VCOM_TransferData();
    }
}



/*** (C) COPYRIGHT 2014~2015 Nuvoton Technology Corp. ***/

