/******************************************************************************
 * @file     main.c
 * @brief
 *           Demonstrate how to implement a USB dual virtual COM port device.
 * @note
 *
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC123.h"
#include "cdc_serial.h"

/*--------------------------------------------------------------------------*/
STR_VCOM_LINE_CODING gLineCoding0 = {115200, 0, 0, 8};   /* Baud rate : 115200    */
STR_VCOM_LINE_CODING gLineCoding1 = {115200, 0, 0, 8};   /* Baud rate : 115200    */
/* Stop bit     */
/* parity       */
/* data bits    */
uint16_t gCtrlSignal0 = 0;     /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */
uint16_t gCtrlSignal1 = 0;     /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */

/*--------------------------------------------------------------------------*/
#define RXBUFSIZE           512 /* RX buffer size */
#define TXBUFSIZE           512 /* RX buffer size */

#define TX_FIFO_SIZE_0      16  /* TX Hardware FIFO size */
#define TX_FIFO_SIZE_1      16  /* TX Hardware FIFO size */

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
/* UART0 */
volatile uint8_t comRbuf0[RXBUFSIZE];
volatile uint16_t comRbytes0 = 0;
volatile uint16_t comRhead0 = 0;
volatile uint16_t comRtail0 = 0;

volatile uint8_t comTbuf0[TXBUFSIZE];
volatile uint16_t comTbytes0 = 0;
volatile uint16_t comThead0 = 0;
volatile uint16_t comTtail0 = 0;

uint8_t gRxBuf0[64] = {0};
volatile uint8_t *gpu8RxBuf0 = 0;
volatile uint32_t gu32RxSize0 = 0;
volatile uint32_t gu32TxSize0 = 0;

volatile int8_t gi8BulkOutReady0 = 0;

/* UART1 */
volatile uint8_t comRbuf1[RXBUFSIZE];
volatile uint16_t comRbytes1 = 0;
volatile uint16_t comRhead1 = 0;
volatile uint16_t comRtail1 = 0;

volatile uint8_t comTbuf1[TXBUFSIZE];
volatile uint16_t comTbytes1 = 0;
volatile uint16_t comThead1 = 0;
volatile uint16_t comTtail1 = 0;

uint8_t gRxBuf1[64] = {0};
volatile uint8_t *gpu8RxBuf1 = 0;
volatile uint32_t gu32RxSize1 = 0;
volatile uint32_t gu32TxSize1 = 0;

volatile int8_t gi8BulkOutReady1 = 0;



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
    CLK_EnableModuleClock(UART1_MODULE);
    CLK_EnableModuleClock(USBD_MODULE);

    /* Select peripheral clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));
    CLK_SetModuleClock(USBD_MODULE, 0, CLK_CLKDIV_USB(3));


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFP |= (SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD | SYS_GPB_MFP_PB4_UART1_RXD | SYS_GPB_MFP_PB5_UART1_TXD);
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

    /* Enable UART Interrupt */
    UART0->IER = UART_IER_RTO_IEN_Msk | UART_IER_RDA_IEN_Msk;

}

void UART1_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS->IPRSTC2 |=  SYS_IPRSTC2_UART1_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_UART1_RST_Msk;

    /* Configure UART1 and set UART1 Baudrate */
    UART1->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 115200);
    UART1->LCR = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;

    /* Enable UART Interrupt */
    UART1->IER = UART_IER_RTO_IEN_Msk | UART_IER_RDA_IEN_Msk;

}


/*---------------------------------------------------------------------------------------------------------*/
/* UART Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_IRQHandler(void)
{
    uint32_t u32IntStatus;
    uint8_t bInChar;
    int32_t size;

    u32IntStatus = UART0->ISR;

    if((u32IntStatus & UART_ISR_RDA_IF_Msk) || (u32IntStatus & UART_ISR_TOUT_IF_Msk))
    {
        /* Receiver FIFO threshold level is reached or Rx time out */

        /* Get all the input characters */
        while((UART0->FSR & UART_FSR_RX_EMPTY_Msk) == 0)
        {
            /* Get the character from UART Buffer */
            bInChar = UART0->DATA;

            /* Check if buffer full */
            if(comRbytes0 < RXBUFSIZE)
            {
                /* Enqueue the character */
                comRbuf0[comRtail0++] = bInChar;
                if(comRtail0 >= RXBUFSIZE)
                    comRtail0 = 0;
                comRbytes0++;
            }
            else
            {
                /* FIFO over run */
            }
        }
    }

    if(u32IntStatus & UART_ISR_THRE_IF_Msk)
    {

        if(comTbytes0)
        {
            /* Fill the Tx FIFO */
            size = comTbytes0;
            if(size >= TX_FIFO_SIZE_0)
            {
                size = TX_FIFO_SIZE_0;
            }

            while(size)
            {
                bInChar = comTbuf0[comThead0++];
                UART0->DATA = bInChar;
                if(comThead0 >= TXBUFSIZE)
                    comThead0 = 0;
                comTbytes0--;
                size--;
            }
        }
        else
        {
            /* No more data, just stop Tx (Stop work) */
            UART0->IER &= (~UART_IER_THRE_IEN_Msk);
        }
    }

}

void UART1_IRQHandler(void)
{
    uint32_t u32IntStatus;
    uint8_t bInChar;
    int32_t size;

    u32IntStatus = UART1->ISR;

    if((u32IntStatus & UART_ISR_RDA_IF_Msk) || (u32IntStatus & UART_ISR_TOUT_IF_Msk))
    {
        /* Receiver FIFO threshold level is reached or Rx time out */

        /* Get all the input characters */
        while((UART1->FSR & UART_FSR_RX_EMPTY_Msk) == 0)
        {
            /* Get the character from UART Buffer */
            bInChar = UART1->DATA;

            /* Check if buffer full */
            if(comRbytes1 < RXBUFSIZE)
            {
                /* Enqueue the character */
                comRbuf1[comRtail1++] = bInChar;
                if(comRtail1 >= RXBUFSIZE)
                    comRtail1 = 0;
                comRbytes1++;
            }
            else
            {
                /* FIFO over run */
            }
        }
    }

    if(u32IntStatus & UART_ISR_THRE_IF_Msk)
    {

        if(comTbytes1)
        {
            /* Fill the Tx FIFO */
            size = comTbytes1;
            if(size >= TX_FIFO_SIZE_1)
            {
                size = TX_FIFO_SIZE_1;
            }

            while(size)
            {
                bInChar = comTbuf1[comThead1++];
                UART1->DATA = bInChar;
                if(comThead1 >= TXBUFSIZE)
                    comThead1 = 0;
                comTbytes1--;
                size--;
            }
        }
        else
        {
            /* No more data, just stop Tx (Stop work) */
            UART1->IER &= (~UART_IER_THRE_IEN_Msk);
        }
    }

}

void VCOM_TransferData(void)
{
    int32_t i, i32Len;

    /* Check whether USB is ready for next packet or not */
    if(gu32TxSize0 == 0)
    {
        /* Check whether we have new COM Rx data to send to USB or not */
        if(comRbytes0)
        {
            i32Len = comRbytes0;
            if(i32Len > EP2_MAX_PKT_SIZE)
                i32Len = EP2_MAX_PKT_SIZE;

            for(i = 0; i < i32Len; i++)
            {
                gRxBuf0[i] = comRbuf0[comRhead0++];
                if(comRhead0 >= RXBUFSIZE)
                    comRhead0 = 0;
            }

            __set_PRIMASK(1);
            comRbytes0 -= i32Len;
            __set_PRIMASK(0);

            gu32TxSize0 = i32Len;
            USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2)), (uint8_t *)gRxBuf0, i32Len);
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

    if(gu32TxSize1 == 0)
    {
        /* Check whether we have new COM Rx data to send to USB or not */
        if(comRbytes1)
        {
            i32Len = comRbytes1;
            if(i32Len > EP7_MAX_PKT_SIZE)
                i32Len = EP7_MAX_PKT_SIZE;

            for(i = 0; i < i32Len; i++)
            {
                gRxBuf1[i] = comRbuf1[comRhead1++];
                if(comRhead1 >= RXBUFSIZE)
                    comRhead1 = 0;
            }

            __set_PRIMASK(1);
            comRbytes1 -= i32Len;
            __set_PRIMASK(0);

            gu32TxSize1 = i32Len;
            USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP7)), (uint8_t *)gRxBuf1, i32Len);
            USBD_SET_PAYLOAD_LEN(EP7, i32Len);
        }
        else
        {
            /* Prepare a zero packet if previous packet size is EP7_MAX_PKT_SIZE and 
               no more data to send at this moment to note Host the transfer has been done */
            i32Len = USBD_GET_PAYLOAD_LEN(EP7);
            if(i32Len == EP7_MAX_PKT_SIZE)
                USBD_SET_PAYLOAD_LEN(EP7, 0);
        }
    }

    /* Process the Bulk out data when bulk out data is ready. */
    if(gi8BulkOutReady0 && (gu32RxSize0 <= TXBUFSIZE - comTbytes0))
    {
        for(i = 0; i < gu32RxSize0; i++)
        {
            comTbuf0[comTtail0++] = gpu8RxBuf0[i];
            if(comTtail0 >= TXBUFSIZE)
                comTtail0 = 0;
        }

        __set_PRIMASK(1);
        comTbytes0 += gu32RxSize0;
        __set_PRIMASK(0);

        gu32RxSize0 = 0;
        gi8BulkOutReady0 = 0; /* Clear bulk out ready flag */

        /* Ready to get next BULK out */
        USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
    }
    
    if(gi8BulkOutReady1 && (gu32RxSize1 <= TXBUFSIZE - comTbytes1))
    {
        for(i = 0; i < gu32RxSize1; i++)
        {
            comTbuf1[comTtail1++] = gpu8RxBuf1[i];
            if(comTtail1 >= TXBUFSIZE)
                comTtail1 = 0;
        }

        __set_PRIMASK(1);
        comTbytes1 += gu32RxSize1;
        __set_PRIMASK(0);

        gu32RxSize1 = 0;
        gi8BulkOutReady1 = 0; /* Clear bulk out ready flag */

        /* Ready to get next BULK out */
        USBD_SET_PAYLOAD_LEN(EP6, EP6_MAX_PKT_SIZE);
    }

    /* Process the software Tx FIFO */
    if(comTbytes0)
    {
        /* Check if Tx is working */
        if((UART0->IER & UART_IER_THRE_IEN_Msk) == 0)
        {
            /* Send one bytes out */
            UART0->DATA = comTbuf0[comThead0++];
            if(comThead0 >= TXBUFSIZE)
                comThead0 = 0;

            __set_PRIMASK(1);
            comTbytes0--;
            __set_PRIMASK(0);

            /* Enable Tx Empty Interrupt. (Trigger first one) */
            UART0->IER |= UART_IER_THRE_IEN_Msk;
        }
    }
    
    if(comTbytes1)
    {
        /* Check if Tx is working */
        if((UART1->IER & UART_IER_THRE_IEN_Msk) == 0)
        {
            /* Send one bytes out */
            UART1->DATA = comTbuf1[comThead1++];
            if(comThead1 >= TXBUFSIZE)
                comThead1 = 0;

            __set_PRIMASK(1);
            comTbytes1--;
            __set_PRIMASK(0);

            /* Enable Tx Empty Interrupt. (Trigger first one) */
            UART1->IER |= UART_IER_THRE_IEN_Msk;
        }
    }
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
    UART1_Init();

    printf("\n\n");
    printf("+------------------------------------------------------------+\n");
    printf("|       NuMicro USB Virtual COM Dual Port Sample Code        |\n");
    printf("+------------------------------------------------------------+\n");
    
    /* Open USB controller */
    USBD_Open(&gsInfo, VCOM_ClassRequest, NULL);

    /* Endpoint configuration */
    VCOM_Init();
    /* Start USB device */
    USBD_Start();
    NVIC_EnableIRQ(USBD_IRQn);
    NVIC_EnableIRQ(UART0_IRQn);
    NVIC_EnableIRQ(UART1_IRQn);

    while(1)
    {
        VCOM_TransferData();
    }
}



/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/

