/**************************************************************************//**
 * @file     uart_transfer.c
 * @version  V1.00
 * @brief    General UART ISP slave Sample file
 *
 * @note
 * Copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include <string.h>
#include "targetdev.h"


#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t  uart_rcvbuf[MAX_PKT_SIZE] = {0};
#else
__align(4) uint8_t  uart_rcvbuf[MAX_PKT_SIZE] = {0};
#endif

uint8_t volatile bUartDataReady = 0;
uint8_t volatile bufhead = 0;


/* please check "targetdev.h" for chip specifc define option */

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART interrupt event                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void UART_N_IRQHandler(void)
{
    /* Determine interrupt source */
    uint32_t u32IntSrc = UART_N->ISR;

    /* RDA FIFO interrupt and RDA timeout interrupt */
    if (u32IntSrc & (UART_ISR_RDA_IF_Msk|UART_ISR_TOUT_IF_Msk)) {
        /* Read data until RX FIFO is empty or data is over maximum packet size */        
        while (((UART_N->FSR & UART_FSR_RX_EMPTY_Msk) == 0) && (bufhead < MAX_PKT_SIZE)) {	
            uart_rcvbuf[bufhead++] = UART_N->RBR;
        }
    }

    /* Reset data buffer index */      
    if (bufhead == MAX_PKT_SIZE) {
        bUartDataReady = TRUE;
        bufhead = 0;
    } else if (u32IntSrc & UART_ISR_TOUT_IF_Msk) {
        bufhead = 0;
    }
}

extern __align(4) uint8_t response_buff[64];
void PutString(void)
{
    uint32_t i;

    /* UART send response to master */    
    for (i = 0; i < MAX_PKT_SIZE; i++) {
        
        /* Wait for TX not full */
        while ((UART_N->FSR & UART_FSR_TX_FULL_Msk));

        /* UART send data */ 
        UART_N->THR = response_buff[i];
    }
}

void UART_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    
    /* Select UART function */    
    UART_N->FUN_SEL = UART_FUNC_SEL_UART;

    /* Set UART line configuration */    
    UART_N->LCR = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;

    /* Set UART Rx and RTS trigger level */       
    UART_N->FCR = UART_FCR_RFITL_14BYTES | UART_FCR_RTS_TRI_LEV_14BYTES;
    
    /* Set UART baud rate */    
    UART_N->BAUD = (UART_BAUD_MODE0 | UART_BAUD_MODE0_DIVIDER(__HIRC, 115200));

    /* Set time-out interrupt comparator */
    UART_N->TOR = (UART_N->TOR & ~UART_TOR_TOIC_Msk) | (0x40);
    
    /* Set UART NVIC */
    NVIC_SetPriority(UART_N_IRQn, 2);
    NVIC_EnableIRQ(UART_N_IRQn);
    
    /* Enable tim-out counter, Rx tim-out interrupt and Rx ready interrupt */
    UART_N->IER = (UART_IER_TIME_OUT_EN_Msk | UART_IER_RTO_IEN_Msk | UART_IER_RDA_IEN_Msk);
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
