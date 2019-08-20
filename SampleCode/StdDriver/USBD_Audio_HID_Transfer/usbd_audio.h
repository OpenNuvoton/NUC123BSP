/******************************************************************************
 * @file     usbd_audio.h
 * @brief    NuMicro series USB driver header file
 * @version  1.0.0
 * @date     22, December, 2013
 *
 * @note
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __USBD_UAC_H__
#define __USBD_UAC_H__

#include "NUC123.h"

/* Define the vendor id and product id */
#define USBD_VID        0x0416
#define USBD_PID        0xB00A

#define I2C_PORT        I2C0

#define UAC_MICROPHONE  0
#define UAC_SPEAKER     1

/*!<Define Audio information */
#define PLAY_RATE       48000       /* The audo play sampling rate. It could be 8000, 16000, 32000, and 48000 */
#define PLAY_CHANNELS   2           /* Number of channels. Don't Change */

#define REC_RATE        PLAY_RATE   /* The record sampling rate. Must be the same with PLAY_RATE */
#define REC_CHANNELS    2           /* Number of channels. Don't Change */

#define REC_FEATURE_UNITID      0x05
#define PLAY_FEATURE_UNITID     0x06

#define BUF_LEN     32*12
#define REC_LEN     REC_RATE / 1000

/* Define Descriptor information */
#if(PLAY_CHANNELS == 1)
#define PLAY_CH_CFG     1
#endif
#if(PLAY_CHANNELS == 2)
#define PLAY_CH_CFG     3
#endif

#if(REC_CHANNELS == 1)
#define REC_CH_CFG     1
#endif
#if(REC_CHANNELS == 2)
#define REC_CH_CFG     3
#endif

#define PLAY_RATE_LO    (PLAY_RATE & 0xFF)
#define PLAY_RATE_MD    ((PLAY_RATE >> 8) & 0xFF)
#define PLAY_RATE_HI    ((PLAY_RATE >> 16) & 0xFF)

#define REC_RATE_LO     (REC_RATE & 0xFF)
#define REC_RATE_MD     ((REC_RATE >> 8) & 0xFF)
#define REC_RATE_HI     ((REC_RATE >> 16) & 0xFF)

/********************************************/
/* Audio Class Current State                */
/********************************************/
/*!<Define Audio Class Current State */
#define UAC_STOP_AUDIO_RECORD           0
#define UAC_START_AUDIO_RECORD          1
#define UAC_PROCESSING_AUDIO_RECORD     2
#define UAC_BUSY_AUDIO_RECORD           3

/***************************************************/
/*      Audio Class-Specific Request Codes         */
/***************************************************/
/*!<Define Audio Class Specific Request */
#define UAC_REQUEST_CODE_UNDEFINED  0x00
#define UAC_SET_CUR                 0x01
#define UAC_GET_CUR                 0x81
#define UAC_SET_MIN                 0x02
#define UAC_GET_MIN                 0x82
#define UAC_SET_MAX                 0x03
#define UAC_GET_MAX                 0x83
#define UAC_SET_RES                 0x04
#define UAC_GET_RES                 0x84
#define UAC_SET_MEM                 0x05
#define UAC_GET_MEM                 0x85
#define UAC_GET_STAT                0xFF
/*!<Define HID Class Specific Request */
#define HID_SET_REPORT              0x09
#define HID_SET_IDLE                0x0A
#define HID_SET_PROTOCOL            0x0B


#define MUTE_CONTROL                0x01
#define VOLUME_CONTROL              0x02

/*-------------------------------------------------------------*/
#define USBD_HANDLER(ep)        ep##_Handler(void)

#define ISO_IN_MAX_PKT_SIZE         192
#define ISO_OUT_MAX_PKT_SIZE        200
#define HID_TRANS_IN_MAX_PKT_SIZE   32
#define HID_TRANS_OUT_MAX_PKT_SIZE  32
#define HID_KEY_IN_MAX_PKT_SIZE     16
#define HID_MEDIA_IN_MAX_PKT_SIZE   16

#define ISO_IN_EP               EP6
#define ISO_OUT_EP              EP5
#define HID_TRANS_IN_EP         EP2
#define HID_TRANS_OUT_EP        EP3
#define HID_KEY_IN_EP           EP4
#define HID_MEDIA_IN_EP         EP7

#define ISO_IN_HANDLER(x)           USBD_HANDLER(EP6)
#define ISO_OUT_HANDLER(x)          USBD_HANDLER(EP5)
#define HID_TRANS_IN_HANDLER(x)     USBD_HANDLER(EP2)
#define HID_TRANS_OUT_HANDLER(x)    USBD_HANDLER(EP3)
#define HID_KEY_IN_HANDLER(x)       USBD_HANDLER(EP4)
#define HID_MEDIA_IN_HANDLER(x)     USBD_HANDLER(EP7)

/* Define EP maximum packet size */
#define EP0_MAX_PKT_SIZE    8
#define EP1_MAX_PKT_SIZE    EP0_MAX_PKT_SIZE
#define EP6_MAX_PKT_SIZE    ISO_IN_MAX_PKT_SIZE
#define EP5_MAX_PKT_SIZE    ISO_OUT_MAX_PKT_SIZE
#define EP2_MAX_PKT_SIZE    HID_TRANS_IN_MAX_PKT_SIZE
#define EP3_MAX_PKT_SIZE    HID_TRANS_OUT_MAX_PKT_SIZE
#define EP4_MAX_PKT_SIZE    HID_KEY_IN_MAX_PKT_SIZE
#define EP7_MAX_PKT_SIZE    HID_MEDIA_IN_MAX_PKT_SIZE




/* Define the interrupt In EP number */
#define ISO_IN_EP_NUM           ISO_IN_EP
#define ISO_OUT_EP_NUM          ISO_OUT_EP
#define HID_TRANS_IN_EP_NUM     HID_TRANS_IN_EP
#define HID_TRANS_OUT_EP_NUM    HID_TRANS_OUT_EP
#define HID_KEY_IN_EP_NUM       HID_KEY_IN_EP
#define HID_MEDIA_IN_EP_NUM     HID_MEDIA_IN_EP

#define SETUP_BUF_BASE      0
#define SETUP_BUF_LEN       8
#define EP0_BUF_BASE        (SETUP_BUF_BASE + SETUP_BUF_LEN)
#define EP0_BUF_LEN         EP0_MAX_PKT_SIZE
#define EP1_BUF_BASE        (SETUP_BUF_BASE + SETUP_BUF_LEN)
#define EP1_BUF_LEN         EP1_MAX_PKT_SIZE
#define EP2_BUF_BASE        (EP1_BUF_BASE + EP1_BUF_LEN)
#define EP2_BUF_LEN         EP2_MAX_PKT_SIZE
#define EP3_BUF_BASE        (EP2_BUF_BASE + EP2_BUF_LEN)
#define EP3_BUF_LEN         EP3_MAX_PKT_SIZE
#define EP4_BUF_BASE        (EP3_BUF_BASE + EP3_BUF_LEN)
#define EP4_BUF_LEN         EP4_MAX_PKT_SIZE
#define EP5_BUF_BASE        (EP4_BUF_BASE + EP4_BUF_LEN)
#define EP5_BUF_LEN         EP5_MAX_PKT_SIZE
#define EP6_BUF_BASE        (EP5_BUF_BASE + EP5_BUF_LEN)
#define EP6_BUF_LEN         EP6_MAX_PKT_SIZE
#define EP7_BUF_BASE        (EP6_BUF_BASE + EP6_BUF_LEN)
#define EP7_BUF_LEN         EP7_MAX_PKT_SIZE



//---------------------------------------------------------------
//  Media Key
#define HID_CTRL_MUTE        0x01
#define HID_CTRL_VOLUME_INC  0x02
#define HID_CTRL_VOLUME_DEC  0x04

#define HID_CTRL_EJECT       0x08
#define HID_CTRL_PLAY        0x01
#define HID_CTRL_STOP        0x02
#define HID_CTRL_PAUSE       0x04
#define HID_CTRL_NEXT        0x08
#define HID_CTRL_PREVIOUS    0x10
#define HID_CTRL_RECORD      0x20
#define HID_CTRL_REWIND      0x40
#define HID_CTRL_FF          0x80


/*-------------------------------------------------------------*/
#define HID_CMD_SIGNATURE   0x43444948

/* HID Transfer Commands */
#define HID_CMD_NONE     0x00
#define HID_CMD_ERASE    0x71
#define HID_CMD_READ     0xD2
#define HID_CMD_WRITE    0xC3
#define HID_CMD_TEST     0xB4

#define PAGE_SIZE        256

#pragma pack(1)
typedef struct
{
    uint8_t u8Cmd;
    uint8_t u8Size;
    uint32_t u32Arg1;
    uint32_t u32Arg2;
    uint32_t u32Signature;
    uint32_t u32Checksum;
} CMD_T;


/*-------------------------------------------------------------*/
extern volatile uint32_t g_usbd_UsbAudioState;
extern volatile uint8_t g_u8KeyReady;
extern volatile uint8_t g_u8MediaKeyReady;

void UAC_DeviceEnable(uint8_t u8Object);
void UAC_DeviceDisable(uint8_t u8Object);
void UAC_SendRecData(void);
void UAC_GetPlayData(int16_t *pi16src, int16_t i16Samples);

/*-------------------------------------------------------------*/
void UAC_Init(void);
void UAC_ClassRequest(void);
void UAC_SetInterface(void);

void EP2_Handler(void);
void EP3_Handler(void);
void EP4_Handler(void);
void EP5_Handler(void);
void EP6_Handler(void);
void EP7_Handler(void);

void WAU8822_Setup(void);
void timer_init(void);
void AdjFreq(void);
void VolumnControl(void);
void I2C_WriteWAU8822(uint8_t u8addr, uint16_t u16data);

#endif  /* __USBD_UAC_H_ */

/*** (C) COPYRIGHT 2014~2015 Nuvoton Technology Corp. ***/
