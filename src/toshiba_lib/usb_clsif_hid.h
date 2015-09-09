/**
 * @file usb_clsif_hid.h
 * @brief USB HID Class Define
 * @version V0.0
 * @date $Date:: 2014-07-07 15:59:27 +0900 #$
 * @note
 */

/*
 * COPYRIGHT (C) 2014
 * TOSHIBA CORPORATION SEMICONDUCTOR & STORAGE PRODUCTS COMPANY
 * ALL RIGHTS RESERVED
 *
 * THE SOURCE CODE AND ITS RELATED DOCUMENTATION IS PROVIDED "AS IS". TOSHIBA
 * CORPORATION MAKES NO OTHER WARRANTY OF ANY KIND, WHETHER EXPRESS, IMPLIED OR,
 * STATUTORY AND DISCLAIMS ANY AND ALL IMPLIED WARRANTIES OF MERCHANTABILITY,
 * SATISFACTORY QUALITY, NON INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
 * 
 * THE SOURCE CODE AND DOCUMENTATION MAY INCLUDE ERRORS. TOSHIBA CORPORATION
 * RESERVES THE RIGHT TO INCORPORATE MODIFICATIONS TO THE SOURCE CODE IN LATER
 * REVISIONS OF IT, AND TO MAKE IMPROVEMENTS OR CHANGES IN THE DOCUMENTATION OR
 * THE PRODUCTS OR TECHNOLOGIES DESCRIBED THEREIN AT ANY TIME.
 * 
 * TOSHIBA CORPORATION SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGE OR LIABILITY ARISING FROM YOUR USE OF THE SOURCE CODE OR
 * ANY DOCUMENTATION, INCLUDING BUT NOT LIMITED TO, LOST REVENUES, DATA OR
 * PROFITS, DAMAGES OF ANY SPECIAL, INCIDENTAL OR CONSEQUENTIAL NATURE, PUNITIVE
 * DAMAGES, LOSS OF PROPERTY OR LOSS OF PROFITS ARISING OUT OF OR IN CONNECTION
 * WITH THIS AGREEMENT, OR BEING UNUSABLE, EVEN IF ADVISED OF THE POSSIBILITY OR
 * PROBABILITY OF SUCH DAMAGES AND WHETHER A CLAIM FOR SUCH DAMAGE IS BASED UPON
 * WARRANTY, CONTRACT, TORT, NEGLIGENCE OR OTHERWISE.
 */

#ifndef USB_CLSIF_HID_H
#define USB_CLSIF_HID_H

#include "usbstack.h"

/* ***************************************************************************** */
/*                         Configuration                                         */
/* ***************************************************************************** */

/* Number of report descriptor(s) */
#define UHC_NUM_RDESCS                  1

/* Report size (Depends on HID Report Descriptor) */
#define UHC_MAX_GETREPORT_SIZE          8
#define UHC_MAX_SETREPORT_SIZE          1

/* Interrupt endpoint descriptor bInterval value */
#define UHC_IN_INTERVAL                 0x0A
#define UHC_OUT_INTERVAL                0x0A

/* Interrupt endpoint number */
#define UHC_IN_EPNUM                    3
#define UHC_OUT_EPNUM                   2

/* for InterruptOUT pipe implementation. (not implement and no test) */
// #define UHC_INTR_OUT_EP

/* ***************************************************************************** */
/*                         HID class I/F                                         */
/* ***************************************************************************** */

#define USBS_HID_KIND_INT   0x01
#define USBS_HID_KIND_CTRL  0x02
#define USBS_HID_KIND_IDLE  0x03

/**
*	@fn			-	bool USBS_HID_GetReport(uint8_t id, uint8_t *buf, uint32_t *length);
*	@brief		-	This function is called back when receives the GetReport request.
*	@param [in]	-	kind
*						USBS_HID_KIND_INT   Completion of send the previous report on interrupt endpoint. If returns true, report will be send to interrupt endpoint.
*						USBS_HID_KIND_CTRL  GetReport request from control endpoint. If returns true, report will be send to control endpoint.
*						USBS_HID_KIND_IDLE  Expire idle period. If returns true, report will be send to interrupt endpoint.
*	@param [in]	-	id       report ID
*	@param [out]-	buf      Buffer to write the send data.
*	@param [out]-	length   Length to be sent
*	@return		-	Whether the report is available.
**/ 
bool USBS_HID_GetReport(uint8_t kind, uint8_t id, uint8_t *buf, uint32_t *length);

/**
*	@fn			-	bool USBS_HID_SetReport(uint8_t id, const uint8_t *buf, uint32_t length);
*	@brief		-	This function is called back when receives the SetReport request.
*	@param [in]	-	kind
*						USBS_HID_KIND_INT   Request from interrupt endpoint
*						USBS_HID_KIND_CTRL  SetReport request from control endpoint
*	@param [in]	-	id       report ID
*	@param [in]	-	buf      Received report data. 
*	@param [in]	-	length   Length of received report data. 
*	@return		-	success(true) or fail(false)
**/ 
bool USBS_HID_SetReport(uint8_t kind, uint8_t id, const uint8_t *buf, uint32_t length);

/**
*	@fn			-	USBS_STATUS USBS_HID_PutReport(uint8_t id, const uint8_t *buf, uint32_t length);
*	@brief		-	This function is called to prepare the asynchronous report data.
*	@param [in]	-	id       report ID
*	@param [in]	-	buf      Report data to be sent.
*	@param [in]	-	length   Length of report data. 
*	@return		-	\ref USBS_STATUS
**/ 
USBS_STATUS USBS_HID_PutReport(uint8_t id, const uint8_t *buf, uint32_t length);

#endif /* USB_CLSIF_HID_H */
