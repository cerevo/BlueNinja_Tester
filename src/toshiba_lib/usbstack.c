/**
 * @file usbstack.c
 * @brief USB Stack
 * @version V0.0
 * @date $Date:: 2014-10-28 11:53:48 +0900 #$
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
#if !defined(__STDC_VERSION__) || (__STDC_VERSION__ < 199901L)
#error ISO/IEC 9899:1999 or later version is required to compile this C source.
#endif

#include "usbstack_config.h" // usbstack config
#include "usbstack.h"        // usbstack core if
#include "usb_clsif.h"       // usbstack class if
#include "timer.h"

// gpio driver
#include "gpio_wrapper.h"
#include "TZ10xx.h" // for "pmulv", "USB_IRQn"

// usb driver, usb define
#include "USBD_TZ10xx.h"
#include "usb_desc.h"
#include "usb_device.h"
#include "ip_config.h"

// misc
#include "include.h"
#include <string.h>
#include "usbdebug.h"

/* gpio driver if */
static void USBS_gpio_vbus_callback(uint32_t pin);

/* usb driver if */
static void USBS_EndpointEvent_callback(uint8_t ep_addr, ARM_USBD_EP_EVENT ep_event);
static void USBS_DeviceEvent_callback(ARM_USBD_EVENT event);
static ARM_USBD_STATUS usbs_EndpointTransfer(uint8_t ep_addr, uint8_t * data, uint32_t  num);

typedef struct USB_ctrlXfer {
	USB_devReq_t	stReq; 
	USB_devReq_t	stReqPending; 
	USB_xfer_t		stDataXfer; 
	struct {
		void*			pvData;	 /* 転送データ */
		uint32_t		ulDataLen;  /* データ転送長 */
		uint32_t		ulTotalLen; /* 実際に転送した長さ */
		bool			isZeroLenPacket;  /* データステージの最後にゼロ長パケットが必要か */
		void (*fnDone)(const struct USB_deviceRequest *);  /* データステージのデータ転送完了時コールバック */
	} DataPhase;
} USB_ctrlXfer_t; 

/* USB device data structure which store all device
	and endpoint information. */
typedef struct USB_device {
	
	/* USB Device */
		/* class driver Interface*/
		const USB_clsIF_t		*pstClassIF;
		/*Connection speed of USB*/
		uint8_t					iSpeed;
		/*State of the USB device*/
		USB_state_t				iDevState;
		uint8_t					ucDevAddr;
		/*USB configuration number*/
		uint8_t					ucConfig;
		uint8_t					ucAlt;
		/*SOF callback*/
		void (*fnSof)(void);
	
	/* EP0, Default Control pipe */
		/* Information (device request) Control transfer */
		USB_ctrlXfer_t			stCtrlXfer;
		/* EP 0 state*/
		EP0_State_t				EP0State;
	
	/* Endpoints context */
	struct {
		/* pointer to hold endpoint descriptor buffer start address for bidirectional endpoint*/
		const USB_edesc_t		*pstEdesc[2];
		/* Pointer to hold transfer information for bidirectional endpoint */
		USB_xfer_t				*pstXfer[2];
		/* Enpoint status */
		volatile USB_ep_state_t	eState[2];
		/*Transfer completion callback for bidirectional endpoint */
		void (*fnXferDone[2])(USB_xfer_t *);  
	} stEPs[USC_NUM_EPS];
} USB_device_t; 

typedef enum {
	USBS_STATE_UNINITIALIZED,
	USBS_STATE_INITIALIZED,
	USBS_STATE_ACTIVE
} USBS_STATE;

typedef struct _USBS_CONTEXT {
	USBS_STATE state;
	bool is_usbd_active;
	ARM_USBD_CAPABILITIES usbd_cap;
	TZ10XX_DRIVER_GPIO_WRAPPER* gpio_wrapper;
	USB_device_t device;
	volatile usb_event_t event;
	#if USC_RECONNECT_ON_TIMEOUT
	enum {
		ROT_STATE_NONE,
		// timeout occured
		ROT_STATE_TIMEOUT,
		// disconnected
		ROT_STATE_DELAY_AFTER_DISCONNECT, // delaying
		ROT_STATE_DELAY_COMPLETED,
		// reconnect
	} reconnect_on_timeout_state;
	#if USC_RECONNECT_ON_TIMEOUT_TEST
	uint8_t reconnect_on_timeout_test;
	#endif
	#endif
} USBS_CONTEXT;

static USBS_CONTEXT usbs = {
	.state = USBS_STATE_UNINITIALIZED,
	.is_usbd_active = 0,
	.gpio_wrapper = NULL,
	.event.d32 = 0,
	#if USC_RECONNECT_ON_TIMEOUT
	.reconnect_on_timeout_state = ROT_STATE_NONE,
	#if USC_RECONNECT_ON_TIMEOUT_TEST
	.reconnect_on_timeout_test = 1,
	#endif
	#endif
};

// Pseudo USB descriptors for EP0 open.
static const USB_edesc_t USBS_desc_EP0out = {
	.bLength          = USB_EDESC_SIZE,
	.bDescriptorType  = USB_DESC_ENDPOINT,
	.bEndpointAddress = USB_EP_DIR_OUT | 0,
	.bmAttributes     = USB_EP_CTRL,
	.wMaxPacketSize   = {USC_MAXPACKETSIZE0, 0},
	.bInterval        = 0
};

static const USB_edesc_t USBS_desc_EP0in = {
	.bLength          = USB_EDESC_SIZE,
	.bDescriptorType  = USB_DESC_ENDPOINT,
	.bEndpointAddress = USB_EP_DIR_IN | 0,
	.bmAttributes     = USB_EP_CTRL,
	.wMaxPacketSize   = {USC_MAXPACKETSIZE0, 0},
	.bInterval        = 0
};

/* ***************************************************************************** */
/*                        File local Function declaration                        */
/* ***************************************************************************** */

static USBS_STATUS USBS_Connect_core(void);
static void USB_evtDetached(void);
static void USB_evtUSBSuspend(void);
static void USB_evtStatusDone(void);
static void USB_evtDeviceRequest(void);

static USB_err_t USB_xferEP0(uint8_t Dir, USB_ctrlXfer_t *pstCXfer);


/* ***************************************************************************** */
/*                        File local variable definitions                        */
/* ***************************************************************************** */
static int8_t      EP0outBuf[USC_MAXPACKETSIZE0] __attribute__ ((aligned (32)));
static int8_t      EP0inBuf[USC_MAXPACKETSIZE0]  __attribute__ ((aligned (32)));


/* ***************************************************************************** */
/*                        Endpoint0 stage handling functions                     */
/* ***************************************************************************** */

/**
*	@fn			-	static USB_err_t USB_EP0DataInXferZero(USB_ctrlXfer_t *pstCXfer)
*	@brief		-	Function to configure EP0 to transmit IN data 0 length packet.
*	@param[in]	-	pstCXfer - Pointer to control transfer data structure
*	@return		-	\ref USB_err_t 
**/ 
static USB_err_t
USB_EP0DataInXferZero(USB_ctrlXfer_t *pstCXfer)
{
	USB_xfer_t *pstXfer = &pstCXfer->stDataXfer;
	pstXfer->usLen = 0;
	memset(EP0inBuf, 0, sizeof(EP0inBuf));
	pstXfer->pvBuffer = EP0inBuf;
	return USB_xferEP0(1, pstCXfer);
}

/**
*	@fn			-	static USB_err_t USB_EP0DataOutXferZero(USB_ctrlXfer_t *pstCXfer)
*	@brief		-	Function to configure EP0 to transmit IN data 0 length packet.
*	@param[in]	-	pstCXfer - Pointer to control transfer data structure
*	@return		-	\ref USB_err_t  
**/ 
static USB_err_t
USB_EP0DataOutXferZero(USB_ctrlXfer_t *pstCXfer)
{
	USB_xfer_t *pstXfer = &pstCXfer->stDataXfer;
	pstXfer->usLen = 0;
	memset(EP0outBuf, 0, sizeof(EP0outBuf));
	pstXfer->pvBuffer = EP0outBuf;
	return USB_xferEP0(0, pstCXfer);
}

/**
*	@fn			-	static USB_err_t USB_EP0DataInPhaseXfer(USB_ctrlXfer_t *pstCXfer)
*	@brief		-	Function used to configure EP0 to transmit setup data in phase
*	@param[in]	-	pstCXfer - Pointer to control transfer data structure
*	@return		-	\ref USB_err_t  
**/ 
static USB_err_t
USB_EP0DataInPhaseXfer(USB_ctrlXfer_t *pstCXfer)
{
	USB_xfer_t *pstXfer = &pstCXfer->stDataXfer;
	pstXfer->usLen = min (pstCXfer->DataPhase.ulDataLen - pstCXfer->DataPhase.ulTotalLen, USB_GETW(usbs.device.stEPs[0].pstEdesc[1]->wMaxPacketSize));
	if (pstXfer->usLen) {
		memcpy(EP0inBuf, (void *)((uint32_t)pstCXfer->DataPhase.pvData + pstCXfer->DataPhase.ulTotalLen), pstXfer->usLen);
	}
	pstXfer->pvBuffer = EP0inBuf;
	
	return USB_xferEP0(1, pstCXfer);
}

/**
*	@fn			-	static USB_err_t USB_EP0DataOutPhaseXfer(USB_ctrlXfer_t *pstCXfer)
*	@brief		-	Function used to configure EP0 to transmit setup data in phase
*	@param[in]	-	pstCXfer - Pointer to control transfer data structure
*	@return		-	\ref USB_err_t  
**/ 
static USB_err_t
USB_EP0DataOutPhaseXfer(USB_ctrlXfer_t *pstCXfer)
{
	USB_xfer_t *pstXfer = &pstCXfer->stDataXfer;
	pstXfer->usLen = min (pstCXfer->DataPhase.ulDataLen - pstCXfer->DataPhase.ulTotalLen, USB_GETW(usbs.device.stEPs[0].pstEdesc[0]->wMaxPacketSize));
	pstXfer->pvBuffer = EP0outBuf;
	return USB_xferEP0(0, pstCXfer);
}

/**
*	@fn			-	static void USB_SetupPhase(void* pvBuffer)
*	@brief		-	Function used to configure EP0 to receive setup packet
*	@param[in]	-   pvBuffer - Pointer to SETUP packet receive buffer.
*	@return		-	\ref USB_err_t 
**/ 
static USB_err_t USB_SetupPhase(void)
{
	usbs.device.EP0State = EP0_IDLE;
	// @note USB driver always receive the SETUP packet on EP0OUT.
	
	return USB_OK;
}

/**
*	@fn			-	static void USB_StatusOutPhase(USB_ctrlXfer_t *pstCXfer)
*	@brief		-	Function used to configure EP0 to receive setup status phase
*	@param[in]	-	pstCXfer - Pointer to control transfer data structure.	 
*	@return		-	\ref USB_err_t 
**/ 
static USB_err_t USB_StatusOutPhase(USB_ctrlXfer_t *pstCXfer)
{
	if (EP0_STALL == usbs.device.EP0State) {
		return USB_EABORT;
	}
	// setup StatusOUT phase
	usbs.device.EP0State = EP0_OUT_STATUS_PHASE;
	
	// setup one xfer
	return USB_EP0DataOutXferZero(pstCXfer);
}

/**
*	@fn			-	static USB_err_t USB_DataOutPhase(USB_ctrlXfer_t *pstCXfer, void* pvData, uint32_t ulDataLen)
*	@brief		-	Function used to configure EP0 to transmit setup data in phase
*	@param[in]	-	pstCXfer - Pointer to control transfer data structure. 
*	@param[in]	-	pvData   - Pointer to packet receive buffer.
*	@param[in]	-	ulDataLen- Receive Length
*	@return		-	\ref USB_err_t
**/ 
static USB_err_t USB_DataOutPhase(USB_ctrlXfer_t *pstCXfer, void* pvData, uint32_t ulDataLen)
{
	if (EP0_STALL == usbs.device.EP0State) {
		return USB_EABORT;
	}
	// setup DataOUT phase
	usbs.device.EP0State = EP0_OUT_DATA_PHASE;  
	pstCXfer->DataPhase.pvData = pvData;
	pstCXfer->DataPhase.ulDataLen = ulDataLen;
	pstCXfer->DataPhase.isZeroLenPacket = false;
	
	// setup one xfer
	pstCXfer->DataPhase.ulTotalLen = 0;
	
	return USB_EP0DataOutPhaseXfer(pstCXfer);
}

/**
*	@fn			-	static void USB_StatusInPhase(USB_ctrlXfer_t *pstCXfer)
*	@brief		-	Function used to configure EP0 to transmit setup status phase
*	@param[in]	-	pstCXfer - Pointer to control transfer data structure.	 
*	@return		-	\ref USB_err_t  
**/ 
static USB_err_t USB_StatusInPhase(USB_ctrlXfer_t *pstCXfer)
{
	if (EP0_STALL == usbs.device.EP0State) {
		return USB_EABORT;
	}
	// setup StatusIN phase
	usbs.device.EP0State = EP0_IN_STATUS_PHASE;
	// setup one xfer
	return USB_EP0DataInXferZero(pstCXfer);
}

/**
*	@fn			-	static void USB_DataInPhase(USB_ctrlXfer_t *pstCXfer)
*	@brief		-	Function used to configure EP0 to transmit setup data in phase
*	@param[in]	-	pstCXfer - Pointer to control transfer data structure.	 
*	@param[in]	-	pvData   - Pointer to packet transmit data.
*	@param[in]	-	ulDataLen- Transmit Length
*	@return		-	\ref USB_err_t 
**/ 
static USB_err_t USB_DataInPhase(USB_ctrlXfer_t *pstCXfer, void* pvData, uint32_t ulDataLen)
{
	if (EP0_STALL == usbs.device.EP0State) {
		return USB_EABORT;
	}
	// setup DataIN phase
	usbs.device.EP0State = EP0_IN_DATA_PHASE;  
	pstCXfer->DataPhase.pvData = pvData;
	pstCXfer->DataPhase.ulDataLen = ulDataLen;
	// Packet送信の後に Zero Length Packetが必要か調べる
	const USB_devReq_t *pstReq = &pstCXfer->stReq; 
	bool_t isZLP;
	if (USB_GETW(pstReq->wLength) < ulDataLen) {
		return USB_EINVAL;
		
	} else if (0 == ulDataLen) {
		isZLP = false; // そもそも0 length転送要求なので、「後のZero Length Packet」については不要。
		
	} else if ( (USB_GETW(pstReq->wLength) != ulDataLen) &&
				(0 == (ulDataLen % USB_GETW(usbs.device.stEPs[0].pstEdesc[1]->wMaxPacketSize)) )) {
		isZLP = true;
					
	} else {
		isZLP = false;
	}
	pstCXfer->DataPhase.isZeroLenPacket = isZLP;
	
	// setup one xfer
	pstCXfer->DataPhase.ulTotalLen = 0;
	
	log_debug("%s ulDataLen %d isZLP %d", __func__, ulDataLen, isZLP);
	
	return USB_EP0DataInPhaseXfer(pstCXfer);
}

/* ******************************************************************************/
/*             Standard enumeration request handling functions                  */
/* ******************************************************************************/

/**
*	@fn			-	static USB_err_t checkStateAndRecip(uint8_t bmRequestType, uint16_t usIndex)
*	@brief		-	
*	@param[in]	-	bmRequestType
*	@param[in]	-	usIndex
*	@return		-	\ref USB_err_t 
**/ 
static USB_err_t
checkStateAndRecip(uint8_t bmRequestType, uint16_t usIndex)
{
	USB_err_t iStatus = USB_EINVAL; 
	
	USB_state_t current = USB_STATE_GETSTATE(usbs.device.iDevState);
	
	if(current <= USB_STATE_DEFAULT) {
		goto bad; 
	}
	
	switch(USB_DR_GETRECIP(bmRequestType))
	{
	case USB_DR_RECIP_DEVICE:
		if(usIndex != 0) {
			goto bad; // EP0以外はbad
		}
		break;
	case USB_DR_RECIP_INTERFACE:
		if(current == USB_STATE_ADDRESSED) {
			goto bad; // Addressed ステートはbad
		}
		break;
	case USB_DR_RECIP_ENDPOINT:
		if(current == USB_STATE_ADDRESSED && usIndex != 0) {
			goto bad; // Addressed ステートでEP0でないとbad
			
		} else if(current == USB_STATE_CONFIGURED) {
			uint8_t ucEP = USB_EP_GETNUMBER(usIndex);
			uint8_t ucDir  = USB_EP_GETDIR(usIndex) >> 7;
			if (ucEP >= USC_NUM_EPS) {
				goto bad; // パラメータエラー
			} else if(!usbs.device.stEPs[ucEP].pstEdesc[ucDir]) {
				goto bad; // EPnが開いていない
			}
		}
		break;
	default:
		goto bad;
	}
	iStatus = USB_OK; 
	
bad:
	return iStatus;
}

/**
*	@fn			-	static USB_err_t USB_stdDevReqGetStatus(USB_ctrlXfer_t *pstCXfer)
*	@brief		-	Function to process standard device request GetStatus.
*	@param[in]	-	pstCXfer -  Control transfer information	
*	@return		-	\ref USB_err_t 
**/ 
static USB_err_t
USB_stdDevReqGetStatus(USB_ctrlXfer_t *pstCXfer)
{
	static uint16_t ucGetStatus __attribute__ ((aligned (4))) ;
	
	uint16_t usIndex; 
	USB_err_t iStatus = USB_EINVAL; 
	const USB_devReq_t *pstReq = &pstCXfer->stReq; 

	usIndex = USB_GETW(pstReq->wIndex); 
	if(USB_DR_GETDIR(pstReq->bmRequestType) != USB_DR_DIR_D2H ||
	   USB_GETW(pstReq->wValue) != 0 ||
	   USB_GETW(pstReq->wLength) != 2) {
		goto bad; 
	}
	if (USB_OK != checkStateAndRecip(pstReq->bmRequestType, usIndex)) {
		goto bad; 
	}

	switch(USB_DR_GETRECIP(pstReq->bmRequestType)) {
	case USB_DR_RECIP_DEVICE:       /* GetStatus(Device) */
		if(usbs.device.pstClassIF->stRequests.fnGetStatus) {
			/* Call the registered callback */
			iStatus = usbs.device.pstClassIF->stRequests.fnGetStatus(pstReq->bmRequestType, usIndex, &ucGetStatus); 
		}
		break; 

	case USB_DR_RECIP_INTERFACE:    /* GetStatus(Interface) */
		if(usbs.device.pstClassIF->stRequests.fnGetStatus) {
			/* Call the registered callback */
			iStatus = usbs.device.pstClassIF->stRequests.fnGetStatus(pstReq->bmRequestType, usIndex, &ucGetStatus); 
		}
		break; 

	case USB_DR_RECIP_ENDPOINT:     /* GetStatus(Endpoint) */
		{
			uint8_t ucEP = USB_EP_GETNUMBER(usIndex);
			uint8_t Dir  = USB_EP_GETDIR(usIndex) >> 7;
			if (USB_EP_HALTED == usbs.device.stEPs[ucEP].eState[Dir]) {
				ucGetStatus = USB_SE_HALT;
			} else {
				ucGetStatus = 0;
			}
			iStatus = USB_OK; 
		}
		break; 

	default:
		break;
	}
	
	if(iStatus == USB_OK) {
		/* Transmit the status */
		iStatus = USB_DataInPhase(pstCXfer, &ucGetStatus, sizeof(ucGetStatus));
	}
bad:
	return iStatus; 
}

/**
*	@fn			-	static USB_err_t USB_stdDevReqClearFeature(USB_ctrlXfer_t *pstCXfer)
*	@brief		-	Function to process standard device request Clear feature.
*	@param[in]	-	pstCXfer -  Control transfer information	 
*	@return		-	\ref USB_err_t 
**/ 
static USB_err_t
USB_stdDevReqClearFeature(USB_ctrlXfer_t *pstCXfer)
{
	uint16_t usIndex;
	uint16_t usValue;
	USB_err_t iStatus = USB_EINVAL; 
	const USB_devReq_t *pstReq = &pstCXfer->stReq; 

	usIndex = USB_GETW(pstReq->wIndex); 
	usValue = USB_GETW(pstReq->wValue); 
	if(USB_DR_GETDIR(pstReq->bmRequestType) != USB_DR_DIR_H2D || USB_GETW(pstReq->wLength) != 0) {
		goto bad; 
	}
	if (USB_OK != checkStateAndRecip(pstReq->bmRequestType, usIndex)) {
		goto bad; 
	}

	if(usbs.device.pstClassIF->stRequests.fnClearFeature) {
		iStatus = usbs.device.pstClassIF->stRequests.fnClearFeature(pstReq->bmRequestType, usValue, usIndex); 
	} else {
		switch(USB_DR_GETRECIP(pstReq->bmRequestType)) {
		case USB_DR_RECIP_DEVICE:       /* ClearFeature(Device) */
			switch(usValue)
			{
			case USB_FD_TEST_MODE:      // ClearFeature(TEST_MODE) はありません
			case USB_FD_REMOTE_WAKEUP:  // ここでは対処不能なのでエラーを返す
			default:
				break;
			}
			break; 

		case USB_DR_RECIP_ENDPOINT:     /* ClearFeature(Endpoint) */
			if(usValue == USB_FE_ENDPOINT_HALT) {
				uint8_t ucEP = USB_EP_GETNUMBER(usIndex);
				uint8_t Dir  = USB_EP_GETDIR(usIndex) >> 7;
				if (USB_EP_HALTED == usbs.device.stEPs[ucEP].eState[Dir]) {
					if (0 < ucEP) {
						USB_stallEP(usIndex & 0xFF, false);
						usbs.device.stEPs[ucEP].eState[Dir] = USB_EP_RUNNING;
					}
				}
				iStatus = USB_OK;
			}
			break; 

		default:
			break;
		}
	}
	
	if(iStatus == USB_OK) {
		iStatus = USB_StatusInPhase(&usbs.device.stCtrlXfer);
	}

bad:
	return iStatus; 
}

/**
*	@fn			-	static USB_err_t USB_stdDevReqSetFeature(USB_ctrlXfer_t *pstCXfer)
*	@brief		-	Function to process standard device request Set feature.
*	@param[in]	-	pstCXfer -  Control transfer information	 
*	@return		-	\ref USB_err_t 
**/ 
static USB_err_t
USB_stdDevReqSetFeature(USB_ctrlXfer_t *pstCXfer)
{
	uint16_t usIndex;
	uint16_t usValue; 
	USB_err_t iStatus = USB_EINVAL; 
	const USB_devReq_t *pstReq = &pstCXfer->stReq; 

	usIndex = USB_GETW(pstReq->wIndex); 
	usValue = USB_GETW(pstReq->wValue); 
	if(USB_DR_GETDIR(pstReq->bmRequestType) != USB_DR_DIR_H2D || USB_GETW(pstReq->wLength) != 0) {
		goto bad; 
	}
	if (USB_OK != checkStateAndRecip(pstReq->bmRequestType, usIndex)) {
		goto bad; 
	}

	if(usbs.device.pstClassIF->stRequests.fnSetFeature) {
		iStatus = usbs.device.pstClassIF->stRequests.fnSetFeature(usValue, usIndex); 
	} else {
		switch(USB_DR_GETRECIP(pstReq->bmRequestType)) {
		case USB_DR_RECIP_DEVICE:       /* SetFeature(Device) */
			switch(usValue)
			{
			case USB_FD_TEST_MODE:
				// ステータスが送信完了した後の、USB_EP0StatusXferDone() で処理します。
				iStatus = USB_OK; 
				break;
			case USB_FD_REMOTE_WAKEUP: // ここでは対処不能なのでエラーを返す
			default:
				break;
			}
			break; 

		case USB_DR_RECIP_ENDPOINT:     /* SetFeature(Endpoint) */
			if(usValue == USB_FE_ENDPOINT_HALT) {
				USB_stallEP(usIndex & 0xFF, true);
				if (0 == USB_EP_GETNUMBER(usIndex)) {
					usbs.device.EP0State = EP0_STALL;
				}
				iStatus = USB_OK;
			}
			break; 

		default:
			break;
		}
	}

	if(iStatus == USB_OK) {
		iStatus = USB_StatusInPhase(&usbs.device.stCtrlXfer);
	}

bad:
	return iStatus; 
}

/**
*	@fn			-	static USB_err_t USB_stdDevReqSetAddress(USB_ctrlXfer_t *pstCXfer)
*	@brief		-	Function to process standard device request Set Address.
*	@param[in]	-	pstCXfer -  Control transfer information	 
*	@return		-	\ref USB_err_t 
**/ 
static USB_err_t
USB_stdDevReqSetAddress(USB_ctrlXfer_t *pstCXfer)
{
	uint16_t usValue; 
	USB_err_t iStatus = USB_EINVAL; 
	const USB_devReq_t *pstReq = &pstCXfer->stReq; 

	usValue = USB_GETW(pstReq->wValue); 
	if(USB_DR_GETDIR(pstReq->bmRequestType) != USB_DR_DIR_H2D ||
	   USB_DR_GETRECIP(pstReq->bmRequestType) != USB_DR_RECIP_DEVICE ||
	   USB_GETW(pstReq->wIndex) != 0 ||
	   USB_GETW(pstReq->wLength) != 0) {
		goto bad; 
	}
	usbs.device.ucDevAddr = (uint8_t)(usValue & 0xff); 
	Driver_USBD0.DeviceSetAddress(usbs.device.ucDevAddr, ARM_USBD_SET_ADDRESS_SETUP);
	
	iStatus = USB_StatusInPhase(&usbs.device.stCtrlXfer);

bad:
	return iStatus; 
}

/**
*	@fn			-	static USB_err_t USB_stdDevReqGetDesc(USB_ctrlXfer_t *pstCXfer)
*	@brief		-	Function to process standard device request Get Descriptor.
*	@param[in]	-	pstCXfer -  Control transfer information	 
*	@return		-	\ref USB_err_t 
**/ 
static USB_err_t
USB_stdDevReqGetDesc(USB_ctrlXfer_t *pstCXfer)
{
	uint16_t usDescLen; 
	const void *pvDesc; 
	USB_err_t iStatus = USB_EINVAL; 
	const USB_devReq_t *pstReq = &pstCXfer->stReq; 

	if(USB_DR_GETDIR(pstReq->bmRequestType) != USB_DR_DIR_D2H) {
		goto bad; 
	}
	
	if(usbs.device.pstClassIF->stRequests.fnGetDescriptor) {
		iStatus = usbs.device.pstClassIF->stRequests.fnGetDescriptor(USB_GETW(pstReq->wValue), USB_GETW(pstReq->wIndex), USB_GETW(pstReq->wLength), &pvDesc, &usDescLen); 
	} else {
		iStatus = USB_ENOSYS; 
	}

	if(iStatus == USB_OK) {
		iStatus = USB_DataInPhase(pstCXfer, (void *)pvDesc, usDescLen <= USB_GETW(pstReq->wLength) ? usDescLen : USB_GETW(pstReq->wLength));
		if (iStatus != USB_OK) {
			log_error("%s iStatus %d", __func__, iStatus);
		}
		
		#if USC_RECONNECT_ON_TIMEOUT_TEST
		USB_state_t current = USB_STATE_GETSTATE(usbs.device.iDevState);
		if (current == USB_STATE_CONFIGURED) {
			if (usbs.reconnect_on_timeout_state == ROT_STATE_NONE) {
				if (usbs.reconnect_on_timeout_test) {
					usbs.reconnect_on_timeout_test--;
					usbs.reconnect_on_timeout_state = ROT_STATE_TIMEOUT;
					log_warning("%s USC_RECONNECT_ON_TIMEOUT_TEST %d", __func__, usbs.reconnect_on_timeout_test);
				}
			}
		}
		#endif
	}

bad:
	return iStatus; 
}

/**
*	@fn			-	static USB_err_t USB_stdDevReqSetDesc(USB_ctrlXfer_t *pstCXfer)
*	@brief		-	Function to process standard device request Set Descriptor.
*	@param[in]	-	pstCXfer -  Control transfer information	 
*	@return		-	\ref USB_err_t 
**/ 
static USB_err_t
USB_stdDevReqSetDesc(USB_ctrlXfer_t *pstCXfer)
{
	USB_err_t iStatus = USB_EINVAL; 
	const USB_devReq_t *pstReq = &pstCXfer->stReq; 

	if(USB_DR_GETDIR(pstReq->bmRequestType) != USB_DR_DIR_H2D ||
	   USB_DR_GETRECIP(pstReq->bmRequestType) != USB_DR_RECIP_DEVICE) {
		goto bad; 
	}

	if(usbs.device.pstClassIF->stRequests.fnSetDescriptor) {
		iStatus = usbs.device.pstClassIF->stRequests.fnSetDescriptor(USB_GETW(pstReq->wValue), USB_GETW(pstReq->wIndex), USB_GETW(pstReq->wLength)); 
	} else {
		iStatus = USB_ENOSYS; 
	}

bad:
	return iStatus; 
}

/**
*	@fn			-	static USB_err_t USB_stdDevReqGetConfig(USB_ctrlXfer_t *pstCXfer)
*	@brief		-	Function to process standard device request Get Configuration.
*	@param[in]	-	pstCXfer -  Control transfer information	 
*	@return		-	\ref USB_err_t 
**/ 
static USB_err_t
USB_stdDevReqGetConfig(USB_ctrlXfer_t *pstCXfer)
{
	static uint8_t ucGetConfig __attribute__ ((aligned (4))); 
	USB_err_t iStatus = USB_EINVAL; 
	const USB_devReq_t *pstReq = &pstCXfer->stReq; 
	USB_state_t current = USB_STATE_GETSTATE(usbs.device.iDevState);

	if(USB_DR_GETDIR(pstReq->bmRequestType) != USB_DR_DIR_D2H ||
	   USB_DR_GETRECIP(pstReq->bmRequestType) != USB_DR_RECIP_DEVICE ||
	   USB_GETW(pstReq->wLength) != 1) {
		goto bad; 
	}
	if((current != USB_STATE_ADDRESSED) &&
	   (current != USB_STATE_CONFIGURED)) {
		goto bad; 
	}

	if(usbs.device.pstClassIF->stRequests.fnGetConfig) {
		iStatus = usbs.device.pstClassIF->stRequests.fnGetConfig(&ucGetConfig); 
	} else {
		ucGetConfig = usbs.device.ucConfig;
		iStatus = USB_OK; 
	}
	
	if (iStatus == USB_OK) {
		iStatus = USB_DataInPhase(pstCXfer, &ucGetConfig, sizeof(ucGetConfig));
	}

bad:
	return iStatus; 
}

/**
*	@fn			-	static USB_err_t USB_stdDevReqSetConfig(USB_ctrlXfer_t *pstCXfer)
*	@brief		-	Function to process standard device request Set Configuration.
*	@param[in]	-	pstCXfer -  Control transfer information	 
*	@return		-	\ref USB_err_t 
**/ 
static USB_err_t
USB_stdDevReqSetConfig(USB_ctrlXfer_t *pstCXfer)
{
	USB_err_t iStatus = USB_EINVAL; 
	const USB_devReq_t *pstReq = &pstCXfer->stReq; 
	USB_state_t current = USB_STATE_GETSTATE(usbs.device.iDevState);

	if(USB_DR_GETDIR(pstReq->bmRequestType) != USB_DR_DIR_H2D ||
	   USB_DR_GETRECIP(pstReq->bmRequestType) != USB_DR_RECIP_DEVICE ||
	   USB_GETW(pstReq->wIndex) != 0 ||
	   USB_GETW(pstReq->wLength) != 0) {
		goto bad; 
	}
	if((current != USB_STATE_ADDRESSED) &&
	   (current != USB_STATE_CONFIGURED)) {
		goto bad; 
	}

	if(usbs.device.pstClassIF->stRequests.fnSetConfig) {
		iStatus = usbs.device.pstClassIF->stRequests.fnSetConfig((uint8_t)(USB_GETW(pstReq->wValue) & 0xff)); 
	} else {
		iStatus = USB_ENOSYS; 
	}
	
	if(iStatus == USB_OK) {
		iStatus = USB_StatusInPhase(&usbs.device.stCtrlXfer);
	}
bad:
	return iStatus; 
}

/**
*	@fn			-	static USB_err_t USB_stdDevReqGetInterface(USB_ctrlXfer_t *pstCXfer)
*	@brief		-	Function to process standard device request Get Interface.
*	@param[in]	-	pstCXfer -  Control transfer information	 
*	@return		-	\ref USB_err_t 
**/ 
static USB_err_t
USB_stdDevReqGetInterface(USB_ctrlXfer_t *pstCXfer)
{
	static uint8_t ucAltSetting __attribute__ ((aligned (4))); 
	USB_err_t iStatus = USB_EINVAL; 
	const USB_devReq_t *pstReq = &pstCXfer->stReq; 
	USB_state_t current = USB_STATE_GETSTATE(usbs.device.iDevState);

	if(USB_DR_GETDIR(pstReq->bmRequestType) != USB_DR_DIR_D2H ||
	   USB_DR_GETRECIP(pstReq->bmRequestType) != USB_DR_RECIP_INTERFACE ||
	   USB_GETW(pstReq->wValue) != 0 ||
	   USB_GETW(pstReq->wLength) != 1) {
		goto bad; 
	}
	if(current != USB_STATE_CONFIGURED) {
		goto bad; 
	}
	if(usbs.device.pstClassIF->stRequests.fnGetInterface) {
		iStatus = usbs.device.pstClassIF->stRequests.fnGetInterface(USB_GETW(pstReq->wIndex), &ucAltSetting); 
	} else {
		ucAltSetting = usbs.device.ucAlt;
		iStatus = USB_OK; 
	}

	if(iStatus == USB_OK) {
		iStatus = USB_DataInPhase(pstCXfer, &ucAltSetting, sizeof(ucAltSetting));
	}

bad:
	return iStatus; 
}

/**
*	@fn			-	static USB_err_t USB_stdDevReqSetInterface(USB_ctrlXfer_t *pstCXfer)
*	@brief		-	Function to process standard device request Set Interface.
*	@param[in]	-	pstCXfer -  Control transfer information	 
*	@return		-	\ref USB_err_t 
**/ 
static USB_err_t
USB_stdDevReqSetInterface(USB_ctrlXfer_t *pstCXfer)
{
	USB_err_t iStatus = USB_EINVAL; 
	const USB_devReq_t *pstReq = &pstCXfer->stReq; 
	USB_state_t current = USB_STATE_GETSTATE(usbs.device.iDevState);

	if(USB_DR_GETDIR(pstReq->bmRequestType) != USB_DR_DIR_H2D ||
	   USB_DR_GETRECIP(pstReq->bmRequestType) != USB_DR_RECIP_INTERFACE ||
	   USB_GETW(pstReq->wLength) != 0) {
		goto bad; 
	}
	if(current != USB_STATE_CONFIGURED) {
		goto bad; 
	}

	if(usbs.device.pstClassIF->stRequests.fnSetInterface) {
		iStatus = usbs.device.pstClassIF->stRequests.fnSetInterface(USB_GETW(pstReq->wValue), USB_GETW(pstReq->wIndex)); 
	} else {
		iStatus = USB_ENOSYS; 
	}

	if(iStatus == USB_OK) {
		USB_StatusInPhase(&usbs.device.stCtrlXfer);
	}

bad:
	return iStatus; 
}

/* Standard device request function table */
static USB_err_t (*const stdDevReqFuncs[])(USB_ctrlXfer_t *pstCXfer) = {
	USB_stdDevReqGetStatus, 
	USB_stdDevReqClearFeature,
	NULL,	
	USB_stdDevReqSetFeature,
	NULL,	
	USB_stdDevReqSetAddress, 
	USB_stdDevReqGetDesc, 
	USB_stdDevReqSetDesc, 
	USB_stdDevReqGetConfig, 
	USB_stdDevReqSetConfig, 
	USB_stdDevReqGetInterface, 
	USB_stdDevReqSetInterface, 
	NULL, 
};

/* ******************************************************************************/
/*             USB event handling functions                                    */
/* ******************************************************************************/

/**
*	@fn			-	static void USB_evtDeviceRequest(void)
*	@brief		-	Function to process device request events
*	@return		-	void
**/ 
static void
USB_evtDeviceRequest(void)
{
	USB_err_t iStatus;
	uint8_t ucType; 
	USB_ctrlXfer_t *pstCXfer = &usbs.device.stCtrlXfer;
	const USB_devReq_t *pstReq = &pstCXfer->stReq;
	
	if (usbs.event.b.requestabort) {
		usbs.event.b.requestabort = 0;
		log_warning("%s requestabort", __func__);
	}
	
	// Accept new request
	memcpy(&pstCXfer->stReq, &pstCXfer->stReqPending, sizeof(pstCXfer->stReq));
	memset(&pstCXfer->stDataXfer, 0, sizeof(pstCXfer->stDataXfer));
	pstCXfer->stDataXfer.iStatus = USB_OK; 
	memset(&pstCXfer->DataPhase, 0, sizeof(pstCXfer->DataPhase));

	iStatus = USB_EINVAL; 
	ucType = USB_DR_GETTYPE(pstReq->bmRequestType); 
	switch(ucType)
	{
	case USB_DR_TYPE_STANDARD:
		if (pstReq->bRequest < (sizeof(stdDevReqFuncs) / sizeof(stdDevReqFuncs[0])) &&
			stdDevReqFuncs[pstReq->bRequest]) {
			iStatus = stdDevReqFuncs[pstReq->bRequest](pstCXfer); 
		}
		break;
	case USB_DR_TYPE_CLASS:
		if (usbs.device.pstClassIF->stRequests.fnClassRequest) {
			iStatus = usbs.device.pstClassIF->stRequests.fnClassRequest(pstReq); 
		}
		break;
	case USB_DR_TYPE_VENDOR:
		if (usbs.device.pstClassIF->stRequests.fnVendorRequest) {
			iStatus = usbs.device.pstClassIF->stRequests.fnVendorRequest(pstReq); 
		}
		break;
	default:
		break;
	}
	
	/* Check the status */
	if(iStatus != USB_OK) {
		USB_devReqStall();
	}
}

/**
*	@fn			-	static void USB_evtStatusDone(void)
*	@brief		-	Function to process USB Enumeration status phase done event
*	@return		-	None
**/
static void
USB_evtStatusDone(void)
{
	USB_devReq_t *pstReq = &usbs.device.stCtrlXfer.stReq;
	USB_state_t current = USB_STATE_GETSTATE(usbs.device.iDevState);
	
	switch(USB_DR_GETTYPE(pstReq->bmRequestType))
	{
	case USB_DR_TYPE_STANDARD:
		switch(pstReq->bRequest)
		{
		case USB_DR_SET_CONFIG:
			if(current == USB_STATE_ADDRESSED) {
				Driver_USBD0.DeviceConfigure(false);
				if (usbs.device.pstClassIF->stChanges.fnDeviceAddressed) {
					usbs.device.pstClassIF->stChanges.fnDeviceAddressed(usbs.device.ucDevAddr);
				}
				USBS_Configured(usbs.device.ucConfig, usbs.device.ucAlt); // Have been Deconfigured
				
			} else if(current == USB_STATE_CONFIGURED) {
				Driver_USBD0.DeviceConfigure(true);
				if (usbs.device.pstClassIF->stChanges.fnDeviceConfigured) {
					usbs.device.pstClassIF->stChanges.fnDeviceConfigured(usbs.device.ucConfig);
				}
				USBS_Configured(usbs.device.ucConfig, usbs.device.ucAlt); // Have been Configured
			}
			break;
			
		case USB_DR_SET_INTERFACE:
			if(current == USB_STATE_CONFIGURED) {
				USBS_Configured(usbs.device.ucConfig, usbs.device.ucAlt); // Have been Configured, may have been altered
			}
			break;
			
		case USB_DR_SET_FEATURE:
			if ((USB_DR_RECIP_DEVICE == USB_DR_GETRECIP(pstReq->bmRequestType)) && 
				(USB_FD_TEST_MODE == USB_GETW(pstReq->wValue)) ) {
				Driver_USBD0.DeviceSetTestMode(USB_GETW(pstReq->wIndex) >> 8);
				log_warning("%s SET_FEATURE TestMode 0x%02X Enter", __func__, (USB_GETW(pstReq->wIndex) >> 8));
			}
			break;
			
		case USB_DR_CLEAR_FEATURE:
			if(USB_DR_GETRECIP(pstReq->bmRequestType) == USB_DR_RECIP_ENDPOINT &&
				USB_GETW(pstReq->wValue) == 0) {
				/* ClearFeature(Endpoint Halt) */
				if (usbs.device.pstClassIF->stChanges.fnEPStallCleared) {
					usbs.device.pstClassIF->stChanges.fnEPStallCleared(USB_EP_GETNUMBER(USB_GETW(pstReq->wIndex)));
				}
			}
			break;
			
		default:
			break;
		}
		
	case USB_DR_TYPE_CLASS:
		break;
		
	case USB_DR_TYPE_VENDOR:
		break;
		
	default:
		break;
	}
}

/**
*	@fn			-	static void USB_evtEPWriteXferDone(void)
*	@brief		-	Function to process endpoint IN transfer complete event
*	@param[in]	-   ucEP
*	@return		-	None
**/
static inline bool
USB_evtEPXferDone(uint8_t ucEP, uint8_t Dir)
{
	uint32_t ep_event = USB_EVENT_EP(ucEP,Dir);
	
	if (usbs.event.d32 & ep_event)
	{
		usbs.event.d32 &= ~ep_event;
		if (0 == ucEP) {
			USB_ctrlXfer_t *pstCXfer = &usbs.device.stCtrlXfer;
			void (*fnDone)(const struct USB_deviceRequest *);
			fnDone = pstCXfer->DataPhase.fnDone;
			pstCXfer->DataPhase.fnDone = NULL;
			if (fnDone) {
				fnDone(&pstCXfer->stReq);
			} else {
				log_error("%s EP 0 fnDone NULL", __func__);
			}
			
		} else {
			USB_xfer_t *pstXfer; 
			pstXfer = usbs.device.stEPs[ucEP].pstXfer[Dir];
			if(pstXfer) {
				if (USB_EP_RUNNING != usbs.device.stEPs[ucEP].eState[Dir]) {
					pstXfer->iStatus = USB_EABORT;
					log_warning("%s eState %d", __func__, usbs.device.stEPs[ucEP].eState[Dir]);
				} else {
					pstXfer->iStatus = USB_OK;
				}
				/* 転送を削除 */
				usbs.device.stEPs[ucEP].pstXfer[Dir] = NULL;
				/* 転送完了コールバック */
				if(usbs.device.stEPs[ucEP].fnXferDone[Dir]) {
					log_debug("%s EP %d Dir %d callback", __func__, ucEP, Dir);
					usbs.device.stEPs[ucEP].fnXferDone[Dir](pstXfer);
				}
			} else {
				log_error("%s EP %d pstXfer NULL", __func__, ucEP);
			}
		}
		return true;
	}
	
	return false;
}

static void
USB_evtAfterXferDone(void)
{
	if (usbs.device.pstClassIF->stChanges.fnAfterXferDone) {
		usbs.device.pstClassIF->stChanges.fnAfterXferDone(); 
	}
}

static USB_err_t
USB_EP0SetupXferDone(void)
{
	const uint8_t ucEP = 0;
	const uint8_t Dir = 0;
	
	if ((USB_EP_RUNNING == usbs.device.stEPs[ucEP].eState[Dir]) || 
		(USB_EP_HALTED  == usbs.device.stEPs[ucEP].eState[Dir]) ) {
		// SETUPパケットを受信すると、EP0 OUT/IN のstallがクリアされます。
		usbs.device.stEPs[ucEP].eState[0] = USB_EP_RUNNING;
		usbs.device.stEPs[ucEP].eState[1] = USB_EP_RUNNING;
	} else {
		log_error("%s EP state %d", __func__, usbs.device.stEPs[ucEP].eState[Dir]);
		return USB_EIO;
	}
	
	switch(usbs.device.EP0State)
	{
	case EP0_IN_DATA_PHASE:
	case EP0_OUT_DATA_PHASE:
	case EP0_IN_STATUS_PHASE:
	case EP0_OUT_STATUS_PHASE:
		// HostによりTransferが中断され新しいTransferが開始された場合。
		// ep0inが転送中の場合はドライバによってTransferAbortされます。
		// Delete transfer
		usbs.device.stEPs[ucEP].pstXfer[0] = NULL;
		usbs.device.stEPs[ucEP].pstXfer[1] = NULL;
		usbs.device.stCtrlXfer.DataPhase.fnDone = NULL;
		// New Transfer state
		usbs.device.EP0State = EP0_IDLE;
		usbs.event.b.requestabort = 1;
		/* FALLTHROUGH */
	case EP0_IDLE:
		{
			uint16_t buf[4];
			
			// 受信したSETUPパケットをコピー
			Driver_USBD0.ReadSetupPacket((uint8_t*)buf);
			
			// SETUPパケットの内容をparseする
			//log_debug("SETUP received %04X %04X %04X %04X ", buf[0], buf[1], buf[2], buf[3]);
			log_warning("SETUP received %04X %04X %04X %04X ", buf[0], buf[1], buf[2], buf[3]);
			USB_devReq_t *pstReq = &usbs.device.stCtrlXfer.stReqPending;
			pstReq->bmRequestType = (USB_byte_t)(buf[0]      & 0xff);
			pstReq->bRequest      = (USB_byte_t)(buf[0] >> 8 & 0xff);
			USB_SETW(pstReq->wValue,  buf[1]);
			USB_SETW(pstReq->wIndex,  buf[2]);
			USB_SETW(pstReq->wLength, buf[3]);
		}
		usbs.event.b.request = 1;
		return USB_OK;
		
	default:
		return USB_EIO;
	}
}

/**
*	@fn			-	static USB_err_t USB_EP0DataInXferDone(void)
*	@brief		-	Function to EP0 IN data packet transmit done.
*	@return		-	\ref USB_err_t  
**/ 
static USB_err_t
USB_EP0DataInXferDone(void)
{
	const uint8_t ucEP = 0;
	const uint8_t Dir = 1;
	
	USB_xfer_t *pstXfer; 
	pstXfer = usbs.device.stEPs[ucEP].pstXfer[Dir];
	if(pstXfer) {
		if (USB_EP_RUNNING != usbs.device.stEPs[ucEP].eState[Dir]) {
			pstXfer->iStatus = USB_EABORT;
			log_warning("usbstack %s eState %d", __func__, usbs.device.stEPs[ucEP].eState[Dir]);
		} else {
			pstXfer->iStatus = USB_OK;
		}
		/* 転送を削除 */
		usbs.device.stEPs[ucEP].pstXfer[Dir] = NULL;
		/* 転送完了 */
		{
			USB_ctrlXfer_t *pstCXfer = &usbs.device.stCtrlXfer;
			
			pstCXfer->DataPhase.ulTotalLen += pstXfer->usActLen;
			if (pstCXfer->DataPhase.ulTotalLen < pstCXfer->DataPhase.ulDataLen) {
				// setup one xfer again
				return USB_EP0DataInPhaseXfer(pstCXfer);
			} else if (pstCXfer->DataPhase.isZeroLenPacket) {
				// Zero Length Packet
				pstCXfer->DataPhase.isZeroLenPacket = false;
				return USB_EP0DataInXferZero(pstCXfer);
			} else {
				// Callback at USBS_Thread(), or Status ACK
				if (pstCXfer->DataPhase.fnDone) {
					usbs.event.d32 |= USB_EVENT_EP(ucEP,Dir);
					return USB_OK;
				} else {
					// complete DataIN phase
					return USB_StatusOutPhase(&usbs.device.stCtrlXfer);
				}
			}
		}
	} else {
		log_error("%s pstXfer NULL", __func__);
	}
	
	return USB_EIO;
}

/**
*	@fn			-	static USB_err_t USB_EP0DataOutXferDone(void)
*	@brief		-	Function to EP0 IN data packet transmit done.
*	@return		-	\ref USB_err_t  
**/ 
static USB_err_t
USB_EP0DataOutXferDone(void)
{
	const uint8_t ucEP = 0;
	const uint8_t Dir = 0;
	
	USB_xfer_t *pstXfer; 
	pstXfer = usbs.device.stEPs[ucEP].pstXfer[Dir];
	if(pstXfer) {
		if (USB_EP_RUNNING != usbs.device.stEPs[ucEP].eState[Dir]) {
			pstXfer->iStatus = USB_EABORT;
			log_warning("usbstack %s eState %d", __func__, usbs.device.stEPs[ucEP].eState[Dir]);
		} else {
			pstXfer->iStatus = USB_OK;
		}
		/* 転送を削除 */
		usbs.device.stEPs[ucEP].pstXfer[Dir] = NULL;
		
		/* 転送完了 */
		{
			USB_ctrlXfer_t *pstCXfer = &usbs.device.stCtrlXfer;
			if (pstXfer->usActLen) {
				memcpy((void *)((uint32_t)pstCXfer->DataPhase.pvData + pstCXfer->DataPhase.ulTotalLen), pstXfer->pvBuffer, pstXfer->usActLen);
			}
			pstCXfer->DataPhase.ulTotalLen += pstXfer->usActLen;
			if (pstCXfer->DataPhase.ulTotalLen < pstCXfer->DataPhase.ulDataLen) {
				// setup one xfer again
				return USB_EP0DataOutPhaseXfer(pstCXfer);
			} else if (pstCXfer->DataPhase.isZeroLenPacket) {
				// Zero Length Packet
				pstCXfer->DataPhase.isZeroLenPacket = false;
				return USB_EP0DataOutXferZero(pstCXfer);
			} else {
				// Callback at USBS_Thread(), or Status ACK
				if (pstCXfer->DataPhase.fnDone) {
					usbs.event.d32 |= USB_EVENT_EP(ucEP,Dir);
					return USB_OK;
				} else {
					// complete DataOut phase as success
					return USB_StatusInPhase(pstCXfer);
				}
			}
		}
	} else {
		log_error("%s pstXfer NULL", __func__);
	}
	
	return USB_EIO;
}

/**
*	@fn			-	static void USB_EP0StatusXferDone()
*	@brief		-	Function to configure device state according to setup request.
*	@return		-	\ref USB_err_t 
**/ 
static USB_err_t USB_EP0StatusXferDone(void)
{
	const uint8_t ucEP = 0;
	uint8_t Dir;
	
	switch(usbs.device.EP0State)
	{
	case EP0_IN_STATUS_PHASE:
		Dir = 1;
		break;
	case EP0_OUT_STATUS_PHASE:
		Dir = 0;
		break;
	default:
		log_error("%s Invalid EP0State %d", __func__, usbs.device.EP0State);
		return USB_EIO;
	}
	
	USB_xfer_t *pstXfer; 
	pstXfer = usbs.device.stEPs[ucEP].pstXfer[Dir];
	if(pstXfer) {
		if (USB_EP_RUNNING != usbs.device.stEPs[ucEP].eState[Dir]) {
			pstXfer->iStatus = USB_EABORT;
			log_warning("usbstack %s eState %d", __func__, usbs.device.stEPs[ucEP].eState[Dir]);
		} else {
			pstXfer->iStatus = USB_OK;
		}
		/* 転送を削除 */
		usbs.device.stEPs[ucEP].pstXfer[Dir] = NULL;
		/* 転送完了 */
		{
			uint16_t usValue; 
			
			USB_devReq_t *pstReq = &usbs.device.stCtrlXfer.stReq; 
			usValue = USB_GETW(pstReq->wValue); 
			switch(pstReq->bRequest)
			{
			case USB_DR_SET_CONFIG:
				usbs.device.ucConfig     = (uint8_t)(usValue & 0xff); 
				usbs.device.ucAlt        = 0;
				usbs.device.iDevState    = usValue > 0 ? USB_STATE_CONFIGURED : USB_STATE_ADDRESSED; 
				usbs.event.b.requestdone = 1; 
				break;
				
			case USB_DR_SET_INTERFACE:
				usbs.device.ucAlt        = (uint8_t)(usValue & 0xff);
				usbs.event.b.requestdone = 1; 
				break;
				
			case USB_DR_SET_ADDRESS:
				usbs.device.ucDevAddr = (uint8_t)(usValue & 0xff); 
				usbs.device.iDevState = usValue > 0 ? USB_STATE_ADDRESSED : USB_STATE_DEFAULT; 
				usbs.event.b.requestdone = 1; 
				break;
			case USB_DR_SET_FEATURE:
			case USB_DR_CLEAR_FEATURE:
				usbs.event.b.requestdone = 1; 
				break;
			default:
				break;
			}
			USB_SetupPhase();
		}
	} else {
		log_error("%s pstXfer NULL", __func__);
	}
	
	return USB_OK;
}

/**
*	@fn			-	static void USB_evtUSBSuspend(void)
*	@brief		-	Function to process USB Suspend event
*	@return		-	None
**/
static void
USB_evtUSBSuspend(void)
{
	USBS_Suspend(true);  // USB stack enter suspend
	
	if (usbs.device.pstClassIF->stChanges.fnDeviceSuspend) {
		usbs.device.pstClassIF->stChanges.fnDeviceSuspend(); 
	}
	
	if (EP0_IDLE != usbs.device.EP0State) {
		// abort EP0
	}
}

/**
*	@fn			-	static void USB_evtUSBResume(void)
*	@brief		-	Function to process USB Resume event
*	@return		-	None
**/
static void
USB_evtUSBResume(void)
{
	USBS_Suspend(false);  // USB stack exit suspend
	
	if (usbs.device.pstClassIF->stChanges.fnDeviceResume) {
		usbs.device.pstClassIF->stChanges.fnDeviceResume(); 
	}
}

/**
*	@fn			-	static void USB_evtUSBReset(bool attach)
*	@brief		-	Function to process USB Reset event
*	@return		-	None
**/
static void
USB_evtUSBReset(bool attach)
{
	USBS_Suspend(false);  // USB stack exit suspend
	
	if (usbs.device.pstClassIF->stChanges.fnDeviceReset) {
		usbs.device.pstClassIF->stChanges.fnDeviceReset();
	}
	
	if (attach) {
		// Store link speed
		ARM_USBD_STATE usbd_state;
		usbd_state = Driver_USBD0.DeviceGetState();
		usbs.device.iSpeed = usbd_state.speed;
		if (usbs.device.pstClassIF->stChanges.fnDeviceAttached) {
			usbs.device.pstClassIF->stChanges.fnDeviceAttached(usbs.device.iSpeed);
		}
	}
	
	USBS_Configured(usbs.device.ucConfig, usbs.device.ucAlt); // May have been Deconfigured
}

/**
*	@fn			-	static void USB_evtDetached(void)
*	@brief		-	Function to process USB disconnect event
*	@return		-	None
**/
static void
USB_evtDetached(void)
{
	USBS_Suspend(false);  // USB stack exit suspend
	
	// Detach Class Layer
	if (usbs.device.pstClassIF->stChanges.fnDeviceDetached) {
		usbs.device.pstClassIF->stChanges.fnDeviceDetached();
	}
	
	if (usbs.is_usbd_active) {
		
		// Close EP0
		USB_abortEP(USB_ep_addr(0,0));
		USB_abortEP(USB_ep_addr(0,1));
		USB_closeEP(USB_ep_addr(0,0));
		USB_closeEP(USB_ep_addr(0,1));
		usbs.device.EP0State = EP0_DISCONNECT;
		
		// Force deactivate
		Driver_USBD0.DeviceDisconnect();
		
		// Wait for disconnect if capable it.
		if (usbs.usbd_cap.event_disconnect) {
			ARM_USBD_STATE usbd_state;
			do {
				usbd_state = Driver_USBD0.DeviceGetState();
			} while(usbd_state.connected);
		}
		usbs.is_usbd_active = 0;
		
		Driver_USBD0.PowerControl(ARM_POWER_OFF);
		Driver_USBD0.Uninitialize();
		NVIC_ClearPendingIRQ(USB_IRQn);
		USBS_Lock();
	}
	
	// Detach Device Layer
	usbs.device.ucDevAddr  = 0;
	usbs.device.ucConfig   = 0;
	usbs.device.ucAlt      = 0;
	usbs.device.iDevState  = USB_STATE_DETACH;
	
	USBS_Configured(usbs.device.ucConfig, usbs.device.ucAlt); // Have been Deconfigured
}

/**
*	@fn			-	static void USB_evtAttached(void)
*	@brief		-	Function to process USB connect event
*	@return		-	None
**/
static void 
USB_evtAttached(void)
{
	if (!usbs.is_usbd_active) {
		usbs.device.EP0State = EP0_DISCONNECT;
		do {
			if (ARM_USBD_OK != Driver_USBD0.Initialize(USBS_DeviceEvent_callback, USBS_EndpointEvent_callback)) break;
			if (ARM_USBD_OK != Driver_USBD0.PowerControl(ARM_POWER_FULL)) break;
			if (ARM_USBD_OK != Driver_USBD0.DeviceConnect()) break;
			ARM_USBD_STATE usbd_state = Driver_USBD0.DeviceGetState();
			usbs.is_usbd_active = usbd_state.active;
		} while(0);
		if (!usbs.is_usbd_active) {
			// USBD activation failure.
			Driver_USBD0.PowerControl(ARM_POWER_OFF);
			Driver_USBD0.Uninitialize();
			NVIC_ClearPendingIRQ(USB_IRQn);
			USBS_Lock();
			log_error("USBD activation failure.");
		} else {
			log_info("USBD activation success.");
		}
	}
}

/* ***************************************************************************** */
/*                             usbstack class if                                 */
/* ***************************************************************************** */

static inline USB_err_t isOpenEP(uint8_t ep_addr, uint8_t* pucEP, uint8_t* pucDir);

/**
*	@fn			-	USB_err_t USB_devReqSendData(const void *pvBuf, uint16_t usLen)
*	@brief		-	Function to do data transmission on EP0.
*	@param[in]	-	pvBuf - Input buffer
*	@param[in]	-	usLen - Transfer length
*	@param[in]	-	fnDone- Callback function
*	@return		-	\ref USB_err_t 
**/ 
USB_err_t
USB_devReqSendData(const void *pvBuf, uint16_t usLen, void (*const fnDone)(const struct USB_deviceRequest *))
{
	if(!pvBuf) {
		return USB_EINVAL; 
	}
	
	USB_ctrlXfer_t *pstCXfer = &usbs.device.stCtrlXfer;
	pstCXfer->DataPhase.fnDone = fnDone;
	/* Setting transfer information */
	return USB_DataInPhase(pstCXfer, (void *)pvBuf, usLen);
}

/**
*	@fn			-	USB_err_t USB_devReqRecvData(const void *pvBuf, uint16_t usLen)
*	@brief		-	Function to do data receive on EP0.
*	@param[in]	-	pvBuf - Input buffer
*	@param[in]	-	usLen - Transfer length
*	@param[in]	-	fnDone- Callback function
*	@return		-	\ref USB_err_t 
**/ 
USB_err_t
USB_devReqRecvData(void *pvBuf, uint16_t usLen,  void (*const fnDone)(const struct USB_deviceRequest *))
{
	if(!pvBuf) {
		return USB_EINVAL; 
	}

	USB_ctrlXfer_t *pstCXfer = &usbs.device.stCtrlXfer;
	pstCXfer->DataPhase.fnDone = fnDone;
	/* Setting transfer information */
	return USB_DataOutPhase(&usbs.device.stCtrlXfer, pvBuf, usLen);
}

/**
 * @fn			-	USB_err_t USB_devReqSendStatus(void)
 * @brief		-	Send Status
 * @return		-	\ref USB_err_t 
 */
USB_err_t
USB_devReqSendStatus(void)
{
	USB_ctrlXfer_t *pstCXfer = &usbs.device.stCtrlXfer;
	USB_err_t iErr;
	
	switch(usbs.device.EP0State)
	{
	case EP0_IDLE:           // ControlWrite(NoData)
	case EP0_OUT_DATA_PHASE: // ControlWrite(Data)
		iErr = USB_StatusInPhase(pstCXfer);
		break;
	case EP0_IN_DATA_PHASE:   // ControlRead(Data)
		iErr = USB_StatusOutPhase(pstCXfer);
		break;
	case EP0_STALL:
		iErr = USB_EABORT;
		break;
	default:
		iErr = USB_EINVAL;
		break;
	}
	
	return iErr;
}

/**
 * @fn			-	USB_err_t USB_devReqStall(void)
 * @brief		-	Send Status
 * @return		-	\ref USB_err_t 
 */
USB_err_t
USB_devReqStall(void)
{
	#ifdef DEBUG
	const USB_devReq_t *pstReq = &usbs.device.stCtrlXfer.stReq;
	#endif
	
	usbs.device.EP0State = EP0_STALL;
	log_error("%s error bmRequestType %d bRequest %d", __func__, pstReq->bmRequestType, pstReq->bRequest);
	log_error("%s error wValue %04X wIndex %04X wLength %04X", __func__, *(uint16_t*)pstReq->wValue, *(uint16_t*)pstReq->wIndex, *(uint16_t*)pstReq->wLength);
	#if 0
	if ((USB_DR_GETDIR(pstReq->bmRequestType) == USB_DR_DIR_H2D) && 
		(USB_GETW(pstReq->wLength) > 0)) {
		// ControlWrite, DataStage
		USB_stallEP(USB_ep_addr(0,1), true); // StatusStageでIN tokenを出してきたときのために、IN側もstallしておく
		USB_stallEP(USB_ep_addr(0,0), true);
		log_error("%s error1 ", __func__);
		
	} else if ((USB_DR_GETDIR(pstReq->bmRequestType) == USB_DR_DIR_D2H) && 
		(USB_GETW(pstReq->wLength) == 0)) {
		// ControlRead, NoData, StatusStage
		USB_stallEP(USB_ep_addr(0,0), true);
		log_error("%s error2 ", __func__);
			
	} else {
		// 以下の場合, EP1INをstall
		// ControlRead, DataStage
		// ControlWrite, NoData, StatusStage
		USB_stallEP(USB_ep_addr(0,1), true);
		log_error("%s error3 ", __func__);
	}
	#else
	USB_stallEP(USB_ep_addr(0,1), true);
	USB_stallEP(USB_ep_addr(0,0), true);
	#endif
	
	return USB_SetupPhase();
}

/**
*	@fn			-	USB_err_t USB_openEP(const struct USB_endpointDesc *pstEdesc, void (*const fnDone)(USB_xfer_t *))
*	@brief		-	Function used to open endpoint.
*	@param[in]	-	pstEdesc - Pointer to endpoint descriptor
*	@param[in]	-	fnDone   - Callback function to be registered. This callback function will be invoked when transfer is completed in the endpoint
*	@return		-	\ref USB_err_t 
**/ 
USB_err_t
USB_openEP(const struct USB_endpointDesc *pstEdesc, void (*const fnDone)(USB_xfer_t *))
{
	uint8_t ucEP; 
	uint8_t Dir;
	USB_err_t iErr; 
	
	iErr = isOpenEP(pstEdesc->bEndpointAddress, &ucEP, &Dir);
	if (USB_EIO != iErr) {
		goto end;
	}
	iErr = USB_OK;
	
	usbs.device.stEPs[ucEP].pstEdesc[Dir] = pstEdesc; 
	usbs.device.stEPs[ucEP].pstXfer[Dir] = NULL; 
	usbs.device.stEPs[ucEP].fnXferDone[Dir] = fnDone;
	usbs.device.stEPs[ucEP].eState[Dir] = USB_EP_RUNNING;
	ARM_USBD_STATUS status;
	status = Driver_USBD0.EndpointConfigure ((uint8_t)pstEdesc->bEndpointAddress, 
											 (ARM_USB_ENDPOINT_TYPE)USB_EP_GETTYPE(pstEdesc->bmAttributes),
											 *(uint16_t *)pstEdesc->wMaxPacketSize);
	if (ARM_USBD_OK != status) {
		iErr = USB_EIO;
	}
	
end:
	return iErr; 
}

/**
*	@fn			-	USB_err_t USB_xferEP(uint8_t ep_addr, USB_xfer_t *pstXfer)
*	@brief		-	This function configure endpoint for transfer.
*	@param[in]	-	ep_addr  - Endpoint address
*	@param[in]	-	pstXfer  - Pointer to transfer control structure
*	@return		-	\ref USB_err_t 
**/ 
USB_err_t
USB_xferEP(uint8_t ep_addr, USB_xfer_t *pstXfer)
{
	uint8_t ucEP; 
	uint8_t Dir;
	USB_err_t iErr; 
	USB_state_t current = USB_STATE_GETSTATE(usbs.device.iDevState);
	
	assert(NULL != pstXfer);
	
	if ((current != USB_STATE_CONFIGURED) &&
		(current != USB_STATE_ADDRESSED) ) {
		iErr = USB_EIO;
		log_error("%s ep_addr 0x%02X iDevState %d", __func__, ep_addr, current);
		goto end;
	}
	iErr = isOpenEP(ep_addr, &ucEP, &Dir);
	if (USB_OK != iErr) {
		log_error("%s ep_addr 0x%02X not open", __func__, ep_addr);
		goto end;
	}
	if (0 == ucEP) {
		iErr = USB_EINVAL; // Use USB_xferEP0()
		log_error("%s ep_addr 0x%02X", __func__, ep_addr);
		goto end;
	}
	if (USB_EP_RUNNING != usbs.device.stEPs[ucEP].eState[Dir]) {
		iErr = USB_EBUSY;
		log_error("%s ep_addr 0x%02X EP not Running", __func__, ep_addr);
		goto end; 
	}
	if(usbs.device.stEPs[ucEP].pstXfer[Dir]) {
		iErr = USB_EBUSY;
		log_error("%s ep_addr 0x%02X transfer busy", __func__, ep_addr);
		goto end; 
	}
	usbs.device.stEPs[ucEP].pstXfer[Dir] = pstXfer; 
	pstXfer->usActLen = 0; 
	pstXfer->iStatus = USB_EIO; 
	
	if (ARM_USBD_OK != usbs_EndpointTransfer(ep_addr, pstXfer->pvBuffer, pstXfer->usLen)) {
		// Transfer not started
		usbs.device.stEPs[ucEP].pstXfer[Dir] = NULL; 
		iErr = USB_EIO;
		log_error("%s transfer error", __func__);
	}
	
end:
	return iErr; 
}

/**
*	@fn			-	USB_err_t USB_xferEP0(uint8_t Dir, USB_ctrlXfer_t *pstCXfer)
*	@brief		-	This function configure endpoint for EP0 transfer.
*	@param[in]	-   Dir      - 0 out data/status, 1 in  data/status
*	@param[in]	-	pstCXfer - Pointer to transfer control structure
*	@return		-	\ref USB_err_t 
**/ 
static USB_err_t USB_xferEP0(uint8_t Dir, USB_ctrlXfer_t *pstCXfer)
{
	const uint8_t ucEP = 0; 
	USB_err_t iErr = USB_OK; 
	USB_state_t current = USB_STATE_GETSTATE(usbs.device.iDevState);
	
	if (current < USB_STATE_DEFAULT) {
		iErr = USB_EIO;
		log_error("%s dir %d iDevState %d", __func__, Dir, current);
		goto end;
	}
	
	/* Check EP state */
	if (USB_EP_RUNNING != usbs.device.stEPs[ucEP].eState[Dir]) {
		iErr = USB_EBUSY;  // EP not Running
		log_error("%s dir %d EP not Running", __func__, Dir);
		goto end; 
	}
	if (usbs.device.stEPs[ucEP].pstXfer[Dir]) {
		iErr = USB_EBUSY;  // transfer busy
		log_error("%s dir %d transfer busy", __func__, Dir);
		goto end; 
	}
	
	USB_xfer_t *pstXfer;
	pstXfer = &pstCXfer->stDataXfer;
	usbs.device.stEPs[ucEP].pstXfer[Dir] = pstXfer; 
	pstXfer->usActLen = 0; 
	pstXfer->iStatus = USB_EIO; 
	
	if (ARM_USBD_OK != usbs_EndpointTransfer(USB_ep_addr(0,Dir), pstXfer->pvBuffer, pstXfer->usLen)) {
		// Transfer not started
		usbs.device.stEPs[ucEP].pstXfer[Dir] = NULL; 
		iErr = USB_EIO;
		log_error("%s transfer error", __func__);
	}
	
end:
	return iErr; 
}

/**
*	@fn			-	USB_err_t USB_stallEP(uint8_t ep_addr)
*	@brief		-	Function will set stall or clear stall the endpoint.
*	@param[in]	-	ep_addr - Endpoint address
*	@param[in]	-	stall   - set or clear
*	@return		-	\ref USB_err_t 
**/ 
USB_err_t    USB_stallEP(uint8_t ep_addr, bool stall)
{
	uint8_t ucEP; 
	uint8_t Dir;
	USB_err_t iErr; 
	
	iErr = isOpenEP(ep_addr, &ucEP, &Dir);
	if (USB_OK != iErr) {
		goto end;
	}
	
	if (stall) {
		usbs.device.stEPs[ucEP].eState[Dir] = USB_EP_STALLING;
	}
	bool before;
	before = USBS_Unlock();
	Driver_USBD0.EndpointStall(USB_ep_addr(ucEP,Dir), stall);
	if (before) {
		USBS_Lock();
	}
	if (stall) {
		usbs.device.stEPs[ucEP].eState[Dir] = USB_EP_HALTED;
	} else {
		usbs.device.stEPs[ucEP].eState[Dir] = USB_EP_RUNNING;
	}
	
	if (usbs.device.stEPs[ucEP].pstXfer[Dir]) {
		// 転送リクエストが完了していない場合
		if (0 == ucEP) {
			// 転送を削除
			usbs.device.stEPs[ucEP].pstXfer[Dir] = NULL;
			usbs.device.stCtrlXfer.DataPhase.fnDone = NULL;
			
		} else {
			// 転送完了コールバックを呼ぶ
			usbs.event.d32 |= USB_EVENT_EP(ucEP,Dir);
			if (USB_evtEPXferDone(ucEP,Dir)) {
//				usbs.event.b.afterxferdone = 1;
			}
		}
	}
	
end:
	return iErr; 
}

/**
*	@fn			-	USB_err_t USB_abortEP(uint8_t ep_addr)
*	@brief		-	Function will abort the endpoint transfer.
*	@param[in]	-	ep_addr  - Endpoint address
*	@return		-	\ref USB_err_t 
**/ 
USB_err_t    USB_abortEP(uint8_t ep_addr)
{
	uint8_t ucEP; 
	uint8_t Dir;
	USB_err_t iErr; 
	
	iErr = isOpenEP(ep_addr, &ucEP, &Dir);
	if (USB_OK != iErr) {
		goto end;
	}
	
	usbs.device.stEPs[ucEP].eState[Dir] = USB_EP_ABORTING;
	bool before;
	before = USBS_Unlock();
	Driver_USBD0.EndpointAbort(USB_ep_addr(ucEP,Dir));
	if (before) {
		USBS_Lock();
	}
	usbs.device.stEPs[ucEP].eState[Dir] = USB_EP_ABORTED;
	
	if (usbs.device.stEPs[ucEP].pstXfer[Dir]) {
		// 転送リクエストが完了していない場合
		if (0 == ucEP) {
			// 転送を削除
			usbs.device.stEPs[ucEP].pstXfer[Dir] = NULL;
			usbs.device.stCtrlXfer.DataPhase.fnDone = NULL;
			
		} else {
			// 転送完了コールバックを呼ぶ
			usbs.event.d32 |= USB_EVENT_EP(ucEP,Dir);
			if (USB_evtEPXferDone(ucEP,Dir)) {
//				usbs.event.b.afterxferdone = 1;
			}
		}
	}
	
	usbs.device.stEPs[ucEP].eState[Dir] = USB_EP_RUNNING;
	
end:
	return iErr;
}

/**
*	@fn			-	USB_err_t USB_closeEP(const struct USB_endpointDesc *pstEdesc)
*	@brief		-	Function to close endpoint function.
*	@param[in]	-	ep_addr  - Endpoint address
*	@return		-	\ref USB_err_t 
**/ 
USB_err_t
USB_closeEP(uint8_t ep_addr)
{
	uint8_t ucEP; 
	uint8_t Dir;
	USB_err_t iErr; 
	
	iErr = isOpenEP(ep_addr, &ucEP, &Dir);
//	Force close
//	if (USB_OK != iErr) {
//		goto end;
//	}
	
	Driver_USBD0.EndpointUnconfigure (ep_addr);
	usbs.device.stEPs[ucEP].pstEdesc[Dir] = NULL; 
	usbs.device.stEPs[ucEP].pstXfer[Dir] = NULL; 
	usbs.device.stEPs[ucEP].fnXferDone[Dir] = NULL; 
	usbs.device.stEPs[ucEP].eState[Dir] = USB_EP_CLOSED;
	
//end:
	return iErr; 
}

/**
*	@fn			-	USB_err_t USB_isBusyEP(uint8_t ep_addr)
*	@brief		-	This function is returns whether the transfer is in progress
*	@param[in]	-	ep_addr  - Endpoint address
*	@param[out]	-	ppstXfer - Pointer to Xfer pointer
*	@return		-	\ref USB_err_t 
**/ 
USB_err_t
USB_isBusyEP(uint8_t ep_addr, USB_xfer_t **ppstXfer)
{
	uint8_t ucEP; 
	uint8_t Dir;
	USB_err_t iErr;
	
	iErr = isOpenEP(ep_addr, &ucEP, &Dir);
	if (USB_OK != iErr) {
		goto end;
	}
	
	USB_xfer_t *pstXfer; 
	pstXfer = usbs.device.stEPs[ucEP].pstXfer[Dir];
	if(pstXfer) {
		if (NULL != ppstXfer) {
			*ppstXfer = pstXfer;
		}
		iErr = USB_EBUSY;  // transfer busy
		goto end; 
	}
	
	iErr = USB_OK;  // transfer not busy
end:
	return iErr; 
}

/**
*	@fn			-	USB_err_t USB_isOpenEP(uint8_t ep_addr)
*	@brief		-	This function returns whether the EP is open.
*	@param[in]	-	ep_addr  - Endpoint address
*	@return		-	\ref USB_err_t 
**/ 
USB_err_t
USB_isOpenEP(uint8_t ep_addr)
{
	uint8_t ucEP; 
	uint8_t Dir;
	return isOpenEP(ep_addr, &ucEP, &Dir);
}

/**
*	@fn			-	USB_state_t  USB_getDeviceState(void)
*	@brief		-	This function returns USB Device state where usbstack is aware.
*	@return		-	\ref USB_state_t 
**/ 
USB_state_t  USB_getDeviceState(void)
{
	return usbs.device.iDevState;
}

/**
*	@fn			-	USB_err_t USB_getEPState(uint8_t ep_addr, USB_ep_state_t *state)
*	@brief		-	This function returns Endpoint state.
*	@param[in]	-	ep_addr  - Endpoint address
*	@param[out]	-	state    - Endpoint state
*	@return		-	\ref USB_err_t 
**/ 
USB_err_t
USB_getEPState(uint8_t ep_addr, USB_ep_state_t *state)
{
	uint8_t ucEP; 
	uint8_t Dir;
	USB_err_t iErr;
	
	iErr = isOpenEP(ep_addr, &ucEP, &Dir);
	if (USB_EINVAL == iErr) {
		return USB_EINVAL;
	}
	
	*state = usbs.device.stEPs[ucEP].eState[Dir];
	return iErr;
}

/**
*	@fn			-	USB_err_t USB_setEPState(uint8_t ep_addr, USB_ep_state_t state)
*	@brief		-	This function is set Endpoint state.
*	@param[in]	-	ep_addr  - Endpoint address
*	@param[in]	-	state    - Endpoint state
*	@return		-	\ref USB_err_t 
**/ 
USB_err_t
USB_setEPState(uint8_t ep_addr, USB_ep_state_t state)
{
	uint8_t ucEP; 
	uint8_t Dir;
	USB_err_t iErr;
	
	iErr = isOpenEP(ep_addr, &ucEP, &Dir);
	if (USB_EINVAL == iErr) {
		return USB_EINVAL;
	}
	
	usbs.device.stEPs[ucEP].eState[Dir] = state;
	return iErr;
}

/**
*	@fn			-	USB_err_t USB_SetDPID(uint8_t ep_addr)
*	@brief		-	This function is set Data PID value.
*	@param[in]	-	ep_addr  - Endpoint address
*	@param[in]	-	data_pid - DPID value to be set
*	@return		-	\ref USB_err_t 
**/ 
USB_err_t
USB_SetDPID(uint8_t ep_addr, uint8_t data_pid)
{
	uint8_t ucEP; 
	uint8_t Dir;
	USB_err_t iErr; 
	
	iErr = isOpenEP(ep_addr, &ucEP, &Dir);
	if (USB_OK != iErr) {
		goto end;
	}
	
	Driver_USBD0.EndpointSetDPID(ep_addr, data_pid);
end:
	return iErr; 
}

/**
*	@fn			-	USB_err_t    USB_RemoteWakeup(void)
*	@brief		-	Signal remote wakeup
*	@return		-	\ref USB_err_t 
**/ 
USB_err_t    USB_RemoteWakeup(void)
{
	USB_err_t err = USB_EINVAL;
	ARM_USBD_STATUS status;
	
	if (usbs.device.iDevState & USB_STATE_SUSPEND_BIT) {
		status = Driver_USBD0.DeviceRemoteWakeup();
		if (status == ARM_USBD_OK) {
			err = USB_OK;
		}
	} else {
		// Not in suspend.
	}
	
	return err;
}

/**
*	@fn			-	uint16_t     USB_getFrameNumber(void)
*	@brief		-	get current frame number.
*	@return		-	\ref FrameNumber
**/ 
uint16_t     USB_getFrameNumber(void)
{
	USB_state_t current = USB_STATE_GETSTATE(usbs.device.iDevState);
	if (current >= USB_STATE_DEFAULT) {
		return Driver_USBD0.GetFrameNumber();
	} else {
		return 0;
	}
}

/**
*	@fn			-	USB_err_t    USB_listenSOFintr(void (*fnSof)(void));
*	@brief		-	Register/Unregister SOF interrupt callback function.
*	@param[in]	-	fnSof   - Callback function of SOF
*	@return		-	\ref USB_err_t 
**/ 
USB_err_t    USB_listenSOFintr(void (*fnSof)(void))
{
	uint8_t control_code;
	ARM_USBD_STATUS status;
	
	usbs.device.fnSof = fnSof;
	if (fnSof) {
		control_code = USBD_TZ10xx_CONTROL_ENABLE_EVENT;
	} else {
		control_code = USBD_TZ10xx_CONTROL_DISABLE_EVENT;
	}
	
	status = Driver_USBD0.Control(control_code, USBD_TZ10xx_EVENT_SOF);
	if (ARM_USBD_OK == status) {
		return USB_OK;
	} else {
		return USB_EIO;
	}
}

static inline USB_err_t isOpenEP(uint8_t ep_addr, uint8_t* pucEP, uint8_t* pucDir)
{
	/* 引数チェック */
	assert(NULL != pucEP);
	assert(NULL != pucDir);
	
	uint8_t ucEP;
	uint8_t ucDir;
	
	*pucEP = ucEP  = USB_EP_GETNUMBER(ep_addr); 
	*pucDir= ucDir = USB_EP_GETDIR(ep_addr) >> 7;
	
	if (ucEP < USC_NUM_EPS) {
		if(usbs.device.stEPs[ucEP].pstEdesc[ucDir]) {
			return USB_OK; // EPが開いている
		}
		return USB_EIO; // EPが閉じている
	}
	return USB_EINVAL; // パラメータエラー
}

/* ***************************************************************************** */
/*                             usbstack core if                                  */
/* ***************************************************************************** */

USBS_STATUS        USBS_Initialize    (TZ10XX_DRIVER_GPIO_WRAPPER* gpio)
{
	#if USC_RECONNECT_ON_TIMEOUT
	if (usbs.reconnect_on_timeout_state != ROT_STATE_NONE) {
		clr_timer(USC_RECONNECT_ON_TIMEOUT_TIMER);
		usbs.reconnect_on_timeout_state = ROT_STATE_NONE;
	}
	#endif
	
	if (USBS_STATE_UNINITIALIZED != usbs.state) return USBS_ERROR;
	
	/*
	 check USB Device Driver
	 */
	ARM_DRV_VERSION usbd_version;
	usbd_version = Driver_USBD0.GetVersion();
	if (usbd_version.api != ARM_DRV_VERSION_MAJOR_MINOR(1,10)) return USBS_ERROR; // Unsupported USBD API Version
	//usbd_version.drv
	
	// This USB stack is USB driver assumes that vbus can not be detect.
	usbs.usbd_cap = Driver_USBD0.GetCapabilities();
	if (usbs.usbd_cap.event_power_on) return USBS_ERROR;
	if (usbs.usbd_cap.event_power_off) return USBS_ERROR;
	if (!usbs.usbd_cap.event_reset) return USBS_ERROR;
	if (!usbs.usbd_cap.event_suspend) return USBS_ERROR;
	if (!usbs.usbd_cap.event_resume) return USBS_ERROR;
	
	usbs.is_usbd_active = 0;
	
	/*
	 check GPIO Driver
	 */
	if (NULL == gpio) return USBS_ERROR;
	
	// Consider gpio passed as being initialized.
	usbs.gpio_wrapper = gpio;
	
//	usbs.gpio_wrapper->GetVersion();
	
	{
		GPIO_CAPABILITIES gpio_cap;
		gpio_cap = usbs.gpio_wrapper->GetCapabilities(USC_GPIO_VBUS_PIN);
		if (!gpio_cap.connected) return USBS_ERROR;
		if (!gpio_cap.event) return USBS_ERROR;
	}
	
	
	/*
	 USB Device initialize
	 */
	memset(&usbs.device, 0, sizeof(usbs.device)); 
	usbs.device.pstClassIF = &ClassIF;
	usbs.device.iDevState  = USB_STATE_DETACH;
	if (usbs.device.pstClassIF->stChanges.fnClassInitialize) {
		usbs.device.pstClassIF->stChanges.fnClassInitialize();
	}
	
	return USBS_Disconnect();
}

USBS_STATUS        USBS_Uninitialize       (void)
{
	#if USC_RECONNECT_ON_TIMEOUT
	if (usbs.reconnect_on_timeout_state != ROT_STATE_NONE) {
		clr_timer(USC_RECONNECT_ON_TIMEOUT_TIMER);
		usbs.reconnect_on_timeout_state = ROT_STATE_NONE;
	}
	#endif
	
	USBS_Disconnect();
	usbs.gpio_wrapper = NULL;
	if (usbs.device.pstClassIF->stChanges.fnClassUninitialize) {
		usbs.device.pstClassIF->stChanges.fnClassUninitialize();
	}
	usbs.state = USBS_STATE_UNINITIALIZED;
	return USBS_OK;
}

USBS_STATUS        USBS_Connect       (void)
{
	#if USC_RECONNECT_ON_TIMEOUT
	if (usbs.reconnect_on_timeout_state != ROT_STATE_NONE) {
		clr_timer(USC_RECONNECT_ON_TIMEOUT_TIMER);
		usbs.reconnect_on_timeout_state = ROT_STATE_NONE;
	}
	#endif
	
	return USBS_Connect_core();
}

static USBS_STATUS        USBS_Connect_core       (void)
{
	if (USBS_STATE_INITIALIZED != usbs.state) return USBS_ERROR;
	
	/* Setup gpio for VBus detection */
	#if (0 < USC_GPIO_VBUS_PIN) 
	{
		volatile uint32_t tmp;
		tmp = pmulv->CTRL_IO_AON_3;
		pmulv->CTRL_IO_AON_3 = tmp | (1u<<USC_GPIO_VBUS_PIN);
	}
	#endif
	
	GPIO_STATUS rtn;
	rtn = usbs.gpio_wrapper->Configure(USC_GPIO_VBUS_PIN, USC_GPIO_VBUS_DIR , USC_GPIO_VBUS_EVENT, USBS_gpio_vbus_callback);
	log_info("Configure  pin=%d  rtn=%d", USC_GPIO_VBUS_PIN, rtn);
	
	if (GPIO_OK == rtn) {
		usbs.state = USBS_STATE_ACTIVE;
		
		// エッジトリガの場合、まず現在の状態を調べる必要があるためコールバックを呼ぶ。
		switch(USC_GPIO_VBUS_EVENT)
		{
		case GPIO_EVENT_EDGE_POS:
		case GPIO_EVENT_EDGE_NEG:
		case GPIO_EVENT_EDGE_BOTH:
			USBS_gpio_vbus_callback(USC_GPIO_VBUS_PIN);
			break;
		default:
			break;
		}
		
		return USBS_OK;
	}
	
	return USBS_ERROR;
}

USBS_STATUS        USBS_Disconnect       (void)
{
	if (USBS_STATE_ACTIVE == usbs.state) {
		
		USBS_Lock();
		USB_evtDetached();
		
		/* Setup gpio for VBus detection */
		GPIO_STATUS rtn;
		rtn = usbs.gpio_wrapper->Unconfigure(USC_GPIO_VBUS_PIN, USC_GPIO_VBUS_DIR);
		log_info("gpio_wrapper Unconfigure  pin=%d  rtn=%d", USC_GPIO_VBUS_PIN, rtn);
		
		#if (0 < USC_GPIO_VBUS_PIN)
		{
			volatile uint32_t tmp;
			tmp = pmulv->CTRL_IO_AON_3;
			pmulv->CTRL_IO_AON_3 = tmp & ~(1u<<USC_GPIO_VBUS_PIN);
		}
		#endif
	}
	
	usbs.state = USBS_STATE_INITIALIZED;
	return USBS_OK;
}

USBS_STATUS        USBS_Thread    (void)
{
	static const usb_event_t ComEvent = {
		.d32 = 0,
		.b.wkupintr       = 1,
		.b.sessreqintr    = 1,
	};
	
	static const usb_event_t DevEvent = {
		.d32 = 0,
		.b.reset       = 1,
		.b.enumdone    = 1,
		.b.erlysuspend = 1,
		.b.usbsuspend  = 1,
	};
	
	static const usb_event_t XferEvent = {
		.d32 = 0,
		.b.requestdone = 1,
		.b.request     = 1,
		.b.afterxferdone  = 1, // xferdoneの後のタスクコンテキストから処理するイベント
		
		.b.ep0out      = 1,
		.b.ep0in       = 1,
		.b.ep1out      = 1,
		.b.ep1in       = 1,
		.b.ep2out      = 1,
		.b.ep2in       = 1,
		.b.ep3out      = 1,
		.b.ep3in       = 1,
	};
	
	while(usbs.event.d32)
	{
		/* Force Lock */
		USBS_Lock();
		
		/* OTG event*/
		{
			/* disconnect */
			if(usbs.event.b.sesenddet) {
				usbs.event.b.sesenddet = 0;
				USB_evtDetached(); 
				goto end;
			}
		}
		
		/* Host/Device common event */
		if (ComEvent.d32 & usbs.event.d32) {
			if (usbs.event.b.wkupintr) {
				usbs.event.b.wkupintr = 0;
				USB_evtUSBResume();
			}
			if (usbs.event.b.sessreqintr) {
				usbs.event.b.sessreqintr = 0;
				USB_evtAttached(); 
			}
			goto end;
		}
		
		/* Device global event */
		if (DevEvent.d32 & usbs.event.d32) {
			/* usb suspend interrupt*/
			if(usbs.event.b.usbsuspend) {
				usbs.event.b.usbsuspend = 0;
				USB_evtUSBSuspend();
			}
			/* USB Resest interrupt*/
			if(usbs.event.b.reset) {
				bool attach = usbs.event.b.enumdone;
				usbs.event.b.enumdone = 0;
				usbs.event.b.reset = 0;
				USB_evtUSBReset(attach);
			}
			goto end;
		}
		
		/* Xfer event */
		if (XferEvent.d32 & usbs.event.d32) {
			/* EP0 USB request done */
			if(usbs.event.b.requestdone) {
				usbs.event.b.requestdone =0;
				log_debug("%s USB_evtStatusDone()", __func__);
				USB_evtStatusDone();
			}
			/* EP0 USB request received */
			if(usbs.event.b.request) {
				usbs.event.b.request = 0;
//				log_debug("%s USB_evtDeviceRequest()", __func__);
				USB_evtDeviceRequest(); 
			}
			/* EP xfer done */
			for(uint8_t ucEP=0; ucEP<USC_NUM_EPS; ucEP++) {
				if (USB_evtEPXferDone(ucEP,0)) {
					usbs.event.b.afterxferdone = 1;
				}
				if (USB_evtEPXferDone(ucEP,1)) {
					usbs.event.b.afterxferdone = 1;
				}
			}
			/* afterxferdone */
			if(usbs.event.b.afterxferdone) {
				usbs.event.b.afterxferdone = 0;
				log_debug("%s USB_evtAfterXferDone()", __func__);
				USB_evtAfterXferDone();
			}
			goto end;
		}
		
		end:
		if (usbs.event.d32) {
			log_warning("%s unhandled usbs.event 0x%08X", __func__, usbs.event.d32);
//			usbs.event.d32 = 0;
		}
		
		if (usbs.is_usbd_active) {
			/* Force Unlock */
			USBS_Unlock();
		}
	}
	
	#if USC_RECONNECT_ON_TIMEOUT
	switch(usbs.reconnect_on_timeout_state)
	{
	case ROT_STATE_NONE:
		break;
	case ROT_STATE_TIMEOUT:
		if (USBS_STATE_ACTIVE == usbs.state) {
			USBS_Disconnect();
			usbs.reconnect_on_timeout_state = ROT_STATE_DELAY_AFTER_DISCONNECT;
			static void rot_delay_completed(uint32_t id, uintptr_t arg);
			set_timer(USC_RECONNECT_ON_TIMEOUT_TIMER, USC_RECONNECT_ON_TIMEOUT_DELAY/*milliseconds*/, rot_delay_completed, 0);
			log_warning("%s ROT disconnected. set_timer().", __func__);
		} else {
			usbs.reconnect_on_timeout_state = ROT_STATE_NONE;
		}
		break;
	case ROT_STATE_DELAY_AFTER_DISCONNECT:
		// Wait for delay time.
		break;
	case ROT_STATE_DELAY_COMPLETED:
		usbs.reconnect_on_timeout_state = ROT_STATE_NONE;
		USBS_Connect_core();
		log_warning("%s ROT connect.", __func__);
		break;
	default:
		break;
	}
	#endif
	
	return USBS_OK;
}

#if USC_RECONNECT_ON_TIMEOUT
static void rot_delay_completed(uint32_t id, uintptr_t arg)
{
	if (id == USC_RECONNECT_ON_TIMEOUT_TIMER) {
		if (usbs.reconnect_on_timeout_state == ROT_STATE_DELAY_AFTER_DISCONNECT) {
			usbs.reconnect_on_timeout_state = ROT_STATE_DELAY_COMPLETED;
		}
	}
}
#endif

static bool usbs_isLocked = true;

bool USBS_Lock(void)
{
	bool before = usbs_isLocked;
	NVIC_DisableIRQ(USB_IRQn);
	usbs_isLocked = true;
	return before;
}

bool USBS_Unlock(void)
{
	bool before = usbs_isLocked;
	usbs_isLocked = false;
	NVIC_EnableIRQ(USB_IRQn);
	return before;
}

/* ***************************************************************************** */
/*                             gpio driver if                                    */
/* ***************************************************************************** */

static void USBS_gpio_vbus_callback(uint32_t pin)
{
	if (USBS_STATE_ACTIVE != usbs.state) return;
	if (USC_GPIO_VBUS_PIN != pin) return;
	
	uint32_t val = 0;
	#ifdef DEBUG
	GPIO_STATUS rtn;
	rtn = usbs.gpio_wrapper->ReadPin(pin, &val);
	log_info("%s ReadPin  pin=%d  val=%d  rtn=%d", __func__, pin, val, rtn);
	#else
	usbs.gpio_wrapper->ReadPin(pin, &val);
	#endif
	
	if (val) {
		// VBus ON
		if (USB_STATE_GETSTATE(usbs.device.iDevState) == USB_STATE_DETACH) {
			usbs.device.iDevState = USB_STATE_POWERED;
			usbs.event.b.sessreqintr = 1;
		}
		
	} else {
		// VBus OFF
		usbs.event.b.sesenddet = 1;
	}
}

/* ***************************************************************************** */
/*                             usb driver if                                     */
/* ***************************************************************************** */

// Signal USB Device Event.
static void USBS_DeviceEvent_callback(ARM_USBD_EVENT event)
{
	if (USBS_STATE_ACTIVE != usbs.state) return;
	
	USB_state_t current = USB_STATE_GETSTATE(usbs.device.iDevState);
	
	switch (event) {
		
	case ARM_USBD_EVENT_RESET:
		log_info("ARM_USBD_EVENT_RESET");
		if (current >= USB_STATE_POWERED) { // Ignore the event at less than powered-state.
			usbs.device.iDevState = USB_STATE_DEFAULT;
			usbs.event.b.reset = 1;
			if (current == USB_STATE_POWERED) {
				usbs.event.b.enumdone = 1; // attached
			}
			USB_closeEP(USB_ep_addr(0,0));
			USB_closeEP(USB_ep_addr(0,1));
			usbs.device.ucDevAddr = 0;
			usbs.device.ucConfig  = 0;
			usbs.device.ucAlt     = 0;
			memset(&usbs.device.stCtrlXfer, 0, sizeof(usbs.device.stCtrlXfer));
			// To start receiving SETUP packet
			USB_openEP(&USBS_desc_EP0out, NULL);
			USB_openEP(&USBS_desc_EP0in, NULL);
			USB_SetupPhase();
		}
		break;
		
	case ARM_USBD_EVENT_SUSPEND:
		log_info("ARM_USBD_EVENT_SUSPEND");
		if (current >= USB_STATE_POWERED) { // Ignore the event at less than powered-state.
			usbs.device.iDevState |= USB_STATE_SUSPEND_BIT;
			usbs.event.b.usbsuspend =1;
		}
		break;
		
	case ARM_USBD_EVENT_RESUME:
		log_info("ARM_USBD_EVENT_RESUME");
		if (current >= USB_STATE_POWERED) { // Ignore the event at less than powered-state.
			usbs.device.iDevState &= ~USB_STATE_SUSPEND_BIT;
			usbs.event.b.wkupintr = 1;
		}
		break;
		
	case ARM_USBD_EVENT_REMOTE_WAKEUP:
		break;
		
	case (ARM_USBD_EVENT)(-1): // Error occured
		{
			USBD_TZ10xx_ERROR status;
			uint32_t param;
			status = Driver_USBD0.GetError(&param);
			log_info("ARM_USBD_EVENT -1 status %d param %d", status, param);
		}
		break;
		
	case (ARM_USBD_EVENT)(-2): // SOF Received
		if (usbs.device.fnSof) {
			usbs.device.fnSof();
		}
//		log_info("ARM_USBD_EVENT -2");
		break;
		
	default:
		break;
	}
}

// Signal USB Endpoint Event.
static void USBS_EndpointEvent_callback(uint8_t ep_addr, ARM_USBD_EP_EVENT ep_event)
{
	log_debug("%s ep_addr %02x ep_event %d", __func__, ep_addr, ep_event);
	
	if (USBS_STATE_ACTIVE != usbs.state) return;
	
	uint8_t ep_dir = (((ep_addr) & ARM_USB_ENDPOINT_DIRECTION_MASK) ? 1 : 0);
	uint8_t ep_num = ((ep_addr) & ARM_USB_ENDPOINT_NUMBER_MASK);
	
	switch(ep_event)
	{
	case ARM_USBD_EP_EVENT_SETUP:
	case ARM_USBD_EP_EVENT_OUT:
		if (0 != ep_dir) return; // Direction mismatch
		break;
	case ARM_USBD_EP_EVENT_IN:
		if (1 != ep_dir) return; // Direction mismatch
		break;
	case (ARM_USBD_EP_EVENT)(-1):// Error occured
		{
			USBD_TZ10xx_ERROR status;
			uint32_t param;
			status = Driver_USBD0.GetError(&param);
			log_info("ARM_USBD_EP_EVENT -1 status %d param %d", status, param);
			#if USC_RECONNECT_ON_TIMEOUT
			if (usbs.reconnect_on_timeout_state == ROT_STATE_NONE)
			{
				if ((status == USBD_TZ10xx_ERROR_EP_TIMEOUT) &&
					((uint8_t)param == USB_ep_addr(0,1)))
				{
					usbs.reconnect_on_timeout_state = ROT_STATE_TIMEOUT;
				}
			}
			#endif
		}
		return;
	default:
		return; // Unknown event
	}
	
	if (ARM_USBD_EP_EVENT_SETUP != ep_event)
	{
		USB_xfer_t *pstXfer; 
		pstXfer = usbs.device.stEPs[ep_num].pstXfer[ep_dir];
		if (!pstXfer) {
			log_error("%s pstXfer is null", __func__);
			return; // EP was not being transferred
		}
		
		// 転送完了した長さを格納
		pstXfer->usActLen = Driver_USBD0.EndpointTransferGetResult(USB_ep_addr(ep_num,ep_dir));
	}
	
	/*
	switch(usbs.device.stEPs[ep_num].eState[ep_dir])
	{
	case USB_EP_STALLING:
	case USB_EP_HALTED:
		// EP Stall(true)とすれちがいの転送完了
		break;
		
	case USB_EP_ABORTING:
		// EP Abortとすれちがいの転送完了
		break;
	default:
		break;
	}
	*/
	
	if (0 == ep_num) {
		switch(ep_event)
		{
		case ARM_USBD_EP_EVENT_SETUP:
			USB_EP0SetupXferDone();
			break;
			
		case ARM_USBD_EP_EVENT_OUT:
			switch(usbs.device.EP0State)
			{
			case EP0_OUT_DATA_PHASE:
				USB_EP0DataOutXferDone();
				break;
			case EP0_OUT_STATUS_PHASE:
				USB_EP0StatusXferDone();
				break;
			default:
				break;
			}
			break;
			
		case ARM_USBD_EP_EVENT_IN:
			switch(usbs.device.EP0State)
			{
			case EP0_IN_DATA_PHASE:
				USB_EP0DataInXferDone();
				break;
			case EP0_IN_STATUS_PHASE:
				USB_EP0StatusXferDone();
				break;
			default:
				break;
			}
			break;
		}
		
	} else {
		
		if (ARM_USBD_EP_EVENT_SETUP == ep_event) {
			return; // not supported
		}
		
		// USB IN transfer is "Endpoint Write" viewed from the device side.
		// USB OUT transfer is "Endpoint Read" viewed from the device side.
		usbs.event.d32 |= USB_EVENT_EP(ep_num,ep_dir);
		
		log_debug("%s 0x%08X", __func__, usbs.event.d32);
		
	}
}

static ARM_USBD_STATUS usbs_EndpointTransfer(uint8_t ep_addr, uint8_t * data, uint32_t  num)
{
	uint8_t ep_dir = (((ep_addr) & ARM_USB_ENDPOINT_DIRECTION_MASK) ? 1 : 0);
//	uint8_t ep_num = ((ep_addr) & ARM_USB_ENDPOINT_NUMBER_MASK);
	
	ARM_USBD_STATUS status;
	if (ep_dir) {
		status = Driver_USBD0.EndpointWriteStart(ep_addr, data, num);
	} else {
		status = Driver_USBD0.EndpointReadStart(ep_addr, data, num);
	}
	
	return status;
}

