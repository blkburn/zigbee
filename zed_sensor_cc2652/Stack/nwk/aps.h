/**************************************************************************************************
  Filename:       APS.h
  Revised:        $Date: 2015-06-02 15:55:43 -0700 (Tue, 02 Jun 2015) $
  Revision:       $Revision: 43961 $

  Description:    Primitives of the Application Support Sub Layer Task functions.


  Copyright 2004-2015 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

#ifndef APS_H
#define APS_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************************************************************
 * INCLUDES
 */
#include "aps_mede.h"
#include "binding_table.h"
#include "reflecttrack.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * TYPEDEFS
 */
typedef struct
{
  uint16_t             nwkSrcAddr;
  uint8_t              nwkSecure;
  aps_FrameFormat_t* aff;
} APS_CmdInd_t;

typedef struct
{
  OsalPort_EventHdr hdr;
  zAddrType_t SrcAddress;
  uint16_t SrcPanId;
  uint16_t macDestAddr;
  NLDE_Signal_t sig;
  byte SecurityUse;
  uint32_t timestamp;
  uint8_t nwkSeqNum;
  uint8_t radius;
  aps_FrameFormat_t aff;
} apsInMsg_t;

typedef void (*pfnBindingTimeoutCB)( void );

typedef struct
{
  uint16_t srcAddr;
  uint8_t  broadcastId;
  uint16_t expiryTime;
} apsEndDeviceBroadcast_t;

/*********************************************************************
 * CONSTANTS
 */

// APS Command IDs - sent messages
#define APS_INCOMING_MSG 0x01
#define APS_CMD_PKT      0x02

// APS Message Fields
#define APS_MSG_ID     0x00
#define APS_MSG_ID_LEN 0x01

// APS Command Messages
#define APS_CMD_PKT_HDR ((uint8_t)                    \
                         (sizeof(APSME_CmdPkt_t) +  \
                          APS_MSG_ID_LEN          ))

// APS Events
#define APS_EDBIND_TIMEOUT_TIMER_ID 0x0001
#define APS_DATA_REQUEST_HOLD       0x0002
#define APS_ENDDEVICE_BROADCAST_EVT 0x0004
#define APS_DUP_REJ_TIMEOUT_EVT     0x0008

// APS Relector Types
#define APS_REFLECTOR_PUBLIC  0
#define APS_REFLECTOR_PRIVATE 1

/*********************************************************************
 * GLOBAL VARIABLES
 */
extern uint8_t APS_Counter;
extern byte APS_TaskID;
extern uint16_t AIB_MaxBindingTime;
extern uint8_t  AIB_apsUseExtendedPANID[Z_EXTADDR_LEN];
extern uint8_t  AIB_apsUseInsecureJoin;
extern uint8_t *AIB_apsTrustCenterAddress;
extern uint8_t APS_AllowDataRequests;
extern uint32_t APS_lastDupTime;


#if ( ZG_BUILD_ENDDEVICE_TYPE ) && ( RFD_RX_ALWAYS_ON_CAPABLE == TRUE )
  extern apsEndDeviceBroadcast_t apsEndDeviceBroadcastTable[];
#endif

/*********************************************************************
 * APS System Functions
 */

/*
 * Initialization function for the APS layer.
 */
extern void APS_Init( byte task_id );

/*
 * Event Loop Processor for APS.
 */
extern uint32_t APS_event_loop( byte task_id, uint32_t events );

/*
 * Setup the End Device Bind Timeout
 */
extern void APS_SetEndDeviceBindTimeout( uint16_t timeout, pfnBindingTimeoutCB pfnCB );

/*
 * APS Command Indication
 */
extern void APS_CmdInd( APS_CmdInd_t* ind );

/*
 * APS Reflector Initiatialization
 *   APS_REFLECTOR_PUBLIC
 *   APS_REFLECTOR_PRIVATE
 */
extern void APS_ReflectorInit( void );

#if ( ZG_BUILD_ENDDEVICE_TYPE ) && ( RFD_RX_ALWAYS_ON_CAPABLE == TRUE )
  /*
   * APS_InitEndDeviceBroadcastTable - Initialize the End Device Broadcast table
   */
  extern void APS_InitEndDeviceBroadcastTable( void );

  /*
   * APS_EndDeviceBroadcastCheck
   */
  extern uint8_t APS_EndDeviceBroadcastCheck( NLDE_FrameFormat_t *ff );
#endif // ( ZG_BUILD_ENDDEVICE_TYPE ) && ( RFD_RX_ALWAYS_ON_CAPABLE == TRUE )


/*********************************************************************
 * REFLECTOR FUNCTION POINTERS
 */

extern ZStatus_t (*pAPS_UnBind)( byte SrcEndpInt,
                            uint16_t ClusterId, zAddrType_t *DstAddr, byte DstEndpInt );

/*
 * Fill in pItem w/ info from the Nth valid binding table entry.
 */
extern ZStatus_t (*pAPS_GetBind)( uint16_t Nth, apsBindingItem_t *pItem );

extern uint8_t (*pAPS_DataConfirmReflect)( nwkDB_t *rec, uint8_t status );

extern void (*pAPS_DataIndReflect)( zAddrType_t *SrcAddress, aps_FrameFormat_t *aff,
                 NLDE_Signal_t *sig, uint8_t AckRequest, uint8_t discoverRoute,
                 uint8_t SecurityUse, uint32_t timestamp,
                 uint16_t txOptions, uint8_t apsRetries );

/****************************************************************************
****************************************************************************/
#ifdef __cplusplus
}
#endif

#endif /* APS_H */


