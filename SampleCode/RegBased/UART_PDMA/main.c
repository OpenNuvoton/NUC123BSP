/****************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 4 $
 * $Date: 15/07/15 10:05a $
 * @brief
 *           Transmit and receive UART data with PDMA.
 *
 * @note
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NUC123.h"


#define PLLCON_SETTING  CLK_PLLCON_72MHz_HXT
#define PLL_CLOCK       72000000

#define UART_RX_DMA_CH 0
#define UART_TX_DMA_CH 1


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
int32_t UART_TEST_LENGTH = 64;
uint8_t SrcArray[64];
uint8_t DestArray[64];
volatile int32_t IntCnt;
volatile int32_t IsTestOver;
volatile uint32_t g_u32TwoChannelPdmaTest=0;
extern char GetChar(void);


/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int main(void);


/*---------------------------------------------------------------------------------------------------------*/
/* Clear buffer funcion                                                                              	   */
/*---------------------------------------------------------------------------------------------------------*/
void ClearBuf(uint32_t u32Addr, uint32_t u32Length, uint8_t u8Pattern)
{
	uint8_t* pu8Ptr;
	uint32_t i;
	
	pu8Ptr = (uint8_t *)u32Addr;
	
	for (i=0; i<u32Length; i++)
	{
		*pu8Ptr++ = u8Pattern;
	}
}

/*---------------------------------------------------------------------------------------------------------*/
/* Bulid Src Pattern function                                                                         	   */
/*---------------------------------------------------------------------------------------------------------*/
void BuildSrcPattern(uint32_t u32Addr, uint32_t u32Length)
{
    uint32_t i=0,j,loop;
    uint8_t* pAddr;
    
    pAddr = (uint8_t *)u32Addr;
    
    do {
        if (u32Length > 256)
	    	loop = 256;
	    else
	    	loop = u32Length;
	    	
	   	u32Length = u32Length - loop;    	

        for(j=0;j<loop;j++)
            *pAddr++ = (uint8_t)(j+i);
            
	    i++;        
	} while ((loop !=0) || (u32Length !=0));         
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Tx PDMA Channel Configuration                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA_UART_TxTest(void)
{ 
    /* UART Tx PDMA channel configuration */ 
    PDMA1->CSR = 
        PDMA_CSR_PDMACEN_Msk |  /* PDMA channel enable */  
        PDMA_SAR_INC |          /* Increment source address */ 
        PDMA_DAR_FIX |          /* Fixed destination address */
        PDMA_WIDTH_8|           /* Transfer width 8 bits */ 
        (0x2 << PDMA_CSR_MODE_SEL_Pos); /* Memory-to-Peripheral mode */     
    PDMA1->SAR = (uint32_t)SrcArray;    /* Source address */
    PDMA1->DAR = (uint32_t)&UART1->THR; /* Destination address */
    PDMA1->BCR = UART_TEST_LENGTH;      /* Transfer count */	     
    
    /* Service selection */
    PDMA_GCR->PDSSR1 = (PDMA_GCR->PDSSR1 & (~PDMA_PDSSR1_UART1_TXSEL_Msk)) | (UART_TX_DMA_CH<<PDMA_PDSSR1_UART1_TXSEL_Pos);
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Rx PDMA Channel Configuration                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA_UART_RxTest(void)
{
    /* UART Rx PDMA channel configuration */        
    PDMA0->CSR = 
        PDMA_CSR_PDMACEN_Msk |      /* PDMA channel enable */   
        PDMA_SAR_FIX |              /* Fixed source address */ 
        PDMA_DAR_INC |              /* Increment destination address */
        PDMA_WIDTH_8 |              /* Transfer width 8 bits */  
        (0x1 << PDMA_CSR_MODE_SEL_Pos); /* Peripheral-to-Memory mode */ 
    PDMA0->SAR = (uint32_t)&UART1->RBR; /* Source address */
    PDMA0->DAR = (uint32_t)DestArray;   /* Destination address */
    PDMA0->BCR = UART_TEST_LENGTH;      /* Transfer count */		    
    
    /* Service selection */
    PDMA_GCR->PDSSR1 = (PDMA_GCR->PDSSR1 & (~PDMA_PDSSR1_UART1_RXSEL_Msk)) | (UART_RX_DMA_CH<<PDMA_PDSSR1_UART1_RXSEL_Pos);
}

/*---------------------------------------------------------------------------------------------------------*/
/* PDMA Callback function                                                                           	   */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA_Callback_0(void)
{
 	printf("\tTransfer Done %d!\r",++IntCnt);

    /* Use PDMA to do UART loopback test 10 times */
	if(IntCnt<10)
	{   
        /* Trigger PDMA */         
        PDMA1->CSR |= PDMA_CSR_TRIG_EN_Msk;
        PDMA0->CSR |= PDMA_CSR_TRIG_EN_Msk;      
	}
	else
	{
        /* Test is over */  
		IsTestOver = TRUE;
	}    
}

void PDMA_Callback_1(void)
{
 	int32_t i ;
 	
 	printf("\tTransfer Done %d!\t",++IntCnt);
    
    /* Show UART Rx data */     
	for(i=0;i<UART_TEST_LENGTH;i++)
        printf(" 0x%x(%c),",inpb(((uint32_t)DestArray+i)),inpb(((uint32_t)DestArray+i)));
	printf("\n");
    
    /* Use PDMA to do UART Rx test 10 times */
	if(IntCnt<10)
	{
        /* Trigger PDMA */        
        PDMA0->CSR |= PDMA_CSR_TRIG_EN_Msk; 
	}
	else
	{
        /* Test is over */
		IsTestOver = TRUE;
	}    
}

void PDMA_IRQHandler(void)
{    
    /* Get PDMA Block transfer down interrupt status */
    if(PDMA_GET_CH_INT_STS(UART_RX_DMA_CH) & PDMA_ISR_BLKD_IF_Msk)
    {
        /* Clear PDMA Block transfer down interrupt flag */   
        PDMA_CLR_CH_INT_FLAG(UART_RX_DMA_CH, PDMA_ISR_BLKD_IF_Msk);   
        
        /* Handle PDMA block transfer done interrupt event */
        if(g_u32TwoChannelPdmaTest == 1)
        {
            PDMA_Callback_0();
        }
        else if(g_u32TwoChannelPdmaTest == 0)
        {
            PDMA_Callback_1();
        }
    }      
}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_IRQHandler(void)
{	
    /* Get UART0 Rx data and send the data to UART1 Tx */
    if( UART_GET_INT_FLAG(UART0, UART_ISR_RDA_INT_Msk) )
        UART1->THR = UART0->RBR;
}

/*---------------------------------------------------------------------------------------------------------*/
/* PDMA Sample Code:                                                                                  	   */
/*         i32option : ['1'] UART1 TX/RX PDMA Loopback                                                     */
/*                     [Others] UART1 RX PDMA test                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA_UART(int32_t i32option)
{    
    /* Source data initiation */    
	BuildSrcPattern((uint32_t)SrcArray, UART_TEST_LENGTH);
    ClearBuf((uint32_t)DestArray, UART_TEST_LENGTH, 0xFF);
    
    /* Reset PDMA module */
    SYS->IPRSTC1 |=  SYS_IPRSTC1_PDMA_RST_Msk;
    SYS->IPRSTC1 &= ~SYS_IPRSTC1_PDMA_RST_Msk;  

	if(i32option =='1')
	{
		printf("  [Using TWO PDMA channel].\n");
	 	printf("  This sample code will use PDMA to do UART1 loopback test 10 times.\n");
        printf("  Please connect UART1_RXD(PB.4) <--> UART1_TXD(PB.5) before testing.\n");
	    printf("  After connecting PB.4 <--> PB.5, press any key to start transfer.\n");  
	    g_u32TwoChannelPdmaTest = 1;
        getchar();
	}
	else
	{
		UART_TEST_LENGTH = 2;	   /* Test Length */
 		printf("  [Using ONE PDMA channel].\n");
	 	printf("  This sample code will use PDMA to do UART1 Rx test 10 times.\n");
        printf("  Please connect UART1_RXD(PB.4) <--> UART1_TXD(PB.5) before testing.\n");
	    printf("  After connecting PB.4 <--> PB.5, press any key to start transfer.\n");
        g_u32TwoChannelPdmaTest = 0; 
	    getchar();
		printf("  Please input %d bytes to trigger PDMA one time.(Ex: Press 'a''b')\n", UART_TEST_LENGTH);
	}
 
	if(g_u32TwoChannelPdmaTest==1)
	{                   
        /* Enable PDMA channel 1 clock */  
        PDMA_GCR->GCRCSR |= PDMA_GCRCSR_CLK1_EN_Msk;           

        /* UART Tx PDMA configuration */          
        PDMA_UART_TxTest();             
	}
        
    /* Enable PDMA channel 0 clock */  
    PDMA_GCR->GCRCSR |= PDMA_GCRCSR_CLK0_EN_Msk; 

    /* UART Rx PDMA configuration */          
    PDMA_UART_RxTest();  

	/* Enable PDMA Channel 0 Block Transfer Done Interrupt */ 
    PDMA0->IER |= PDMA_IER_BLKD_IE_Msk;      
 	IntCnt = 0;       
    IsTestOver = FALSE;        
    NVIC_EnableIRQ(PDMA_IRQn);
    
    /* Enable UART0 RDA interrupt */
    if(g_u32TwoChannelPdmaTest==0)
    {
        UART_ENABLE_INT(UART0, UART_IER_RDA_IEN_Msk);
        NVIC_EnableIRQ(UART0_IRQn); 
    }
    
	/* Trigger PDMA */
    PDMA0->CSR |= (PDMA_CSR_TRIG_EN_Msk | PDMA_CSR_PDMACEN_Msk);    

	if(g_u32TwoChannelPdmaTest==1)
        PDMA1->CSR |= (PDMA_CSR_TRIG_EN_Msk | PDMA_CSR_PDMACEN_Msk);  
    
    /* Enable UART Tx and Rx PDMA function */
    if(g_u32TwoChannelPdmaTest==1)
        UART1->IER |= UART_IER_DMA_TX_EN_Msk;   
    else
        UART1->IER &= ~UART_IER_DMA_TX_EN_Msk;
        
    UART1->IER |= UART_IER_DMA_RX_EN_Msk; 

    /* Wait for PDMA operation finish */
    while(IsTestOver == FALSE);   

    /* Disable UART Tx and Rx PDMA function */    
	UART1->IER &= ~(UART_IER_DMA_TX_EN_Msk|UART_IER_DMA_RX_EN_Msk);   
    
    /* Disable PDMA channel */
    PDMA_GCR->GCRCSR = 0;   
    
	/* Disable PDMA Interrupt */
    PDMA0->IER &= ~PDMA_IER_BLKD_IE_Msk; 
    NVIC_DisableIRQ(PDMA_IRQn);    
    
    /* Disable UART0 RDA interrupt */
    UART_DISABLE_INT(UART0, UART_IER_RDA_IEN_Msk);
    NVIC_DisableIRQ(UART0_IRQn);   	
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
    CLK->PWRCON |= CLK_PWRCON_OSC22M_EN_Msk;

    /* Waiting for Internal RC clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_OSC22M_STB_Msk));

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLK_S_Msk)) | CLK_CLKSEL0_HCLK_S_HIRC;
    CLK->CLKDIV = (CLK->CLKDIV & (~CLK_CLKDIV_HCLK_N_Msk)) | CLK_CLKDIV_HCLK(1);

    /* Set PLL to power down mode and PLL_STB bit in CLKSTATUS register will be cleared by hardware */
    CLK->PLLCON |= CLK_PLLCON_PD_Msk;      
    
    /* Enable external XTAL 12MHz clock */
    CLK->PWRCON |= CLK_PWRCON_XTL12M_EN_Msk;

    /* Waiting for external XTAL clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_XTL12M_STB_Msk));

    /* Set core clock as PLL_CLOCK from PLL */
    CLK->PLLCON = PLLCON_SETTING;
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_PLL_STB_Msk));
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLK_S_Msk)) | CLK_CLKSEL0_HCLK_S_PLL;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    //SystemCoreClockUpdate();
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For CLK_SysTickDelay()

    /* Enable UART module clock */
    CLK->APBCLK |= (CLK_APBCLK_UART0_EN_Msk | CLK_APBCLK_UART1_EN_Msk);  
    
    /* Enable PDMA clock source */
    CLK->AHBCLK |= CLK_AHBCLK_PDMA_EN_Msk;  

    /* Select UART module clock source */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UART_S_Msk)) | CLK_CLKSEL1_UART_S_HXT;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set GPB multi-function pins for UART0 RXD(PB.0) and TXD(PB.1) */
    /* Set GPB multi-function pins for UART1 RXD(PB.4) and TXD(PB.5) */

    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk | SYS_GPB_MFP_PB4_Msk | SYS_GPB_MFP_PB5_Msk);
    SYS->GPB_MFP |= (SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD | SYS_GPB_MFP_PB4_UART1_RXD | SYS_GPB_MFP_PB5_UART1_TXD);

    SYS->ALT_MFP &= ~(SYS_ALT_MFP_PB4_Msk | SYS_ALT_MFP_PB5_Msk);
    SYS->ALT_MFP |= (SYS_ALT_MFP_PB4_UART1_RXD | SYS_ALT_MFP_PB5_UART1_TXD);

}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS->IPRSTC2 |=  SYS_IPRSTC2_UART0_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_UART0_RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 115200);
    UART0->LCR = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

void UART1_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART1 */
    SYS->IPRSTC2 |=  SYS_IPRSTC2_UART1_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_UART1_RST_Msk;

    /* Configure UART1 and set UART1 Baudrate */
    UART1->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 115200);
    UART1->LCR = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/

int main(void)
{
   	uint8_t unItem;       
    
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /* Init UART1 for testing */
    UART1_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);

    printf("\nUART PDMA Sample Program");

    /* UART PDMA sample function */
	do
	{
		printf("\n\n");
		printf("+------------------------------------------------------------------------+\n");
	    printf("|                      UART PDMA Driver Sample Code                      |\n");
	    printf("+------------------------------------------------------------------------+\n");                    
		printf("| [1] Using TWO PDMA channel to test. < TX1(CH1)-->RX1(CH0) >            |\n");
	    printf("| [2] Using ONE PDMA channel to test. < TX1-->RX1(CH0) >                 |\n");
		printf("+------------------------------------------------------------------------+\n");                     
		unItem = getchar();
	
		IsTestOver = FALSE;
        if((unItem == '1') || (unItem == '2'))
        {
		    PDMA_UART(unItem);
            printf("\n\n  UART PDMA sample code is complete.\n");
        }
        
    }while(unItem!=27);

    while(1);
}

