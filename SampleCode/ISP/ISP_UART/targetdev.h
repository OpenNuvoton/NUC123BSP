/***************************************************************************//**
 * @file     targetdev.h
 * @brief    ISP support function header file
 * @version  0x33
 *
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "NUC123.h"
#include "uart_transfer.h"
#include "isp_user.h"

/* rename for uart_transfer.c */
#define UART_N						UART0
#define UART_N_IRQHandler		    UART0_IRQHandler
#define UART_N_IRQn					UART0_IRQn

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
