/**
 * @file usb_clsif.h
 * @brief a header file for TZ10xx USB Stack class if
 * @version V0.0
 * @date $Date:: 2014-10-14 14:07:40 +0900 #$
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
#ifndef USB_CLSIF_H
#define USB_CLSIF_H

#include "usbstack_config.h"

#include "usb_desc.h"
#include "usb_device.h"

/* USB クラスドライバ・通知インタフェース */
typedef struct USB_class_interface {
	struct {
		/* callbacks for standard device requests */
		USB_err_t   (*fnGetStatus)(uint8_t /* bmRequestType */, uint16_t /* wIndex */, uint16_t * /* outStatus */); 
		USB_err_t   (*fnClearFeature)(uint8_t /* bmRequestType */, uint16_t /* wValue */, uint16_t /* wIndex */); 
		USB_err_t   (*fnSetFeature)(uint16_t /* wValue */, uint16_t /* wIndex */); 
		USB_err_t   (*fnGetDescriptor)(uint16_t /* wValue */, uint16_t /* wIndex */, uint16_t /* wLength */, const void ** /* outData */, uint16_t * /* outLen */); 
		USB_err_t   (*fnSetDescriptor)(uint16_t /* wValue */, uint16_t /* wIndex */, uint16_t /* wLength */); 
		USB_err_t   (*fnGetConfig)(uint8_t * /* outCfgValue */); 
		USB_err_t   (*fnSetConfig)(uint8_t /* cfgValue */); 
		USB_err_t   (*fnGetInterface)(uint16_t /* wIndex */, uint8_t * /* outAltSetting */); 
		USB_err_t   (*fnSetInterface)(uint16_t /* wValue */, uint16_t /* wIndex */); 
		USB_err_t   (*fnSynchFrame)(uint16_t /* wIndex */, uint16_t * /* frameNumber */); 
		/* callbacks for class specific, vendor specific requests */
		USB_err_t   (*fnClassRequest)(const struct USB_deviceRequest *); 
		USB_err_t   (*fnVendorRequest)(const struct USB_deviceRequest *); 
	} stRequests; 

	struct {
		/* callbacks for device status change */
		USB_err_t   (*fnClassInitialize)(void); 
		USB_err_t   (*fnClassUninitialize)(void); 
		void        (*fnDeviceReset)(void); 
		void        (*fnDeviceAttached)(uint8_t /* speed */); 
		void        (*fnDeviceAddressed)(uint8_t /* addr */); 
		void        (*fnDeviceConfigured)(uint8_t /* cfgno */); 
		void        (*fnDeviceDetached)(void); 
		void        (*fnDeviceSuspend)(void); 
		void        (*fnDeviceResume)(void); 
		void        (*fnEPStallCleared)(uint16_t /* epnumber */); 
		void        (*fnAfterXferDone)(void); 
	} stChanges; 
} const USB_clsIF_t; 

extern USB_clsIF_t ClassIF;

/* transactions on EP0 */
USB_err_t    USB_devReqSendData(const void *pvBuf, uint16_t usLen, void (*const fnDone)(const struct USB_deviceRequest *));
USB_err_t    USB_devReqRecvData(void *pvBuf, uint16_t usLen,  void (*const fnDone)(const struct USB_deviceRequest *));
USB_err_t    USB_devReqSendStatus(void);
USB_err_t    USB_devReqStall(void);

/* operations to EPx */
USB_err_t    USB_openEP(const struct USB_endpointDesc *pstEdesc, void (*fnDone)(USB_xfer_t *)); 
USB_err_t    USB_stallEP(uint8_t ep_addr, bool stall);
USB_err_t    USB_abortEP(uint8_t ep_addr);
USB_err_t    USB_xferEP(uint8_t ep_addr, USB_xfer_t *pstXfer);
USB_err_t    USB_closeEP(uint8_t ep_addr); 
USB_err_t    USB_isBusyEP(uint8_t ep_addr, USB_xfer_t **ppstXfer);
USB_err_t    USB_isOpenEP(uint8_t ep_addr);
USB_err_t    USB_SetDPID(uint8_t ep_addr, uint8_t data_pid);
USB_err_t    USB_getEPState(uint8_t ep_addr, USB_ep_state_t *state);
USB_err_t    USB_setEPState(uint8_t ep_addr, USB_ep_state_t state);
USB_state_t  USB_getDeviceState(void);
USB_err_t    USB_RemoteWakeup(void);
uint16_t     USB_getFrameNumber(void);
USB_err_t    USB_listenSOFintr(void (*fnSof)(void));

#endif /* __USB_CLSIF_H__ */
