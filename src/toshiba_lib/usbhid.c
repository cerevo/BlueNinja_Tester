/**
 * @file usbhid.c
 * @brief An example of TZ10xx USB HID Device
 * @version V0.0
 * @date $Date:: 2014-10-27 18:44:04 +0900 #$
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

#include "include.h"
#include "usb_clsif.h"
#include "usb_hid.h"
#include "usb_clsif_hid.h"
#include "usbdebug.h"
#include <string.h>

/* ***************************************************************************** */
/*                         Constants, Macros & Types                             */
/* ***************************************************************************** */

#define CDESC_FULL_SIZE  (USB_CDESC_SIZE + (USB_IDESC_SIZE + USB_HIDDESC_SIZE + (USB_EDESC_SIZE * HID_EP_NUMEPS)))
#define NUM_SDESCS                  (4)

/* HID context */
struct hid {
	const USB_ddesc_t *pstDdesc;                /* Device Descriptor*/
	struct {
		USB_cdesc_t *pstCdesc;                  /* Configuration descriptor*/
		USB_idesc_t *pstIdesc;                  /* Interface descriptor*/
		USB_hiddesc_t *pstHdesc;                /* HID class descriptor*/
		USB_edesc_t *ppstEdescs[HID_EP_NUMEPS]; /* Endpoint descriptor*/
	} stConfigs[1];
	const USB_sdesc_t *ppstSdescs[NUM_SDESCS];  /* Descriptor of each group*/
	const void *pvRdescs[UHC_NUM_RDESCS];       /* Report Descriptors */

	USB_speed_t         iSpeed;                 /* Connection speed of the device */
	USB_state_t         iState;                 /* State of the device */
	uint8_t             ucAddress;              /* 現在デバイスに設定されているアドレス */
	uint8_t             ucCurrentConfig;        /* 現在設定されているコンフィギュレーション番号 */
	uint8_t             ucCurrentAltSetting;    /* 現在設定されているコンフィギュレーションのオルタネート番号 */

	#ifdef UHC_INTR_OUT_EP
	USB_xfer_t  stOutXfer;                      /* Management of information transfer*/
	#endif
	USB_xfer_t  stInXfer;                       /* Management of information transfer*/
	uint8_t ucInXferReportID;
}; 

/* ***************************************************************************** */
/*                       Prototypes of Local functions                           */
/* ***************************************************************************** */
static USB_err_t HID_init(void);
static USB_err_t HID_fini(void);

inline static USB_err_t    HID_startIntrXfer(HID_ep_t iEp, USB_xfer_t *pstXfer); 
static void         HID_doneIntrInXfer(USB_xfer_t *pstXfer); 

static USB_err_t    HID_attach(void); 
static USB_err_t    HID_detach(void); 

/* ***************************************************************************** */
/*                               Local variables                                 */
/* ***************************************************************************** */
static struct hid stHid; 
static uint8_t hid_Protocol;
static uint8_t hid_Idle[UHC_NUM_RDESCS];
static uint8_t hid_GetReport[UHC_NUM_RDESCS][UHC_MAX_GETREPORT_SIZE] __attribute__ ((aligned (32)));
static uint8_t hid_SetReport[UHC_NUM_RDESCS][UHC_MAX_SETREPORT_SIZE] __attribute__ ((aligned (32)));

/* ***************************************************************************** */
/*                            Function definitions                               */
/* ***************************************************************************** */

static void
HID_deviceReset(void)
{
	if (USB_STATE_CONFIGURED == stHid.iState) {
		HID_detach(); 
	}
//	stHid.iSpeed = 0;
	stHid.iState = USB_STATE_DEFAULT; 
	stHid.ucAddress = 0;
	stHid.ucCurrentConfig = 0;
	stHid.ucCurrentAltSetting = 0;
}

static void
HID_deviceAttached(USB_speed_t iSpeed)
{
	if (USB_SPEED_FULL != iSpeed) {
		log_error("%s Unsupported device speed %d", __func__, iSpeed);
		return;
	}
	
	/* USB接続スピードの取得 */
	stHid.iSpeed = iSpeed; 
}

static void
HID_deviceAddressed(uint8_t ucAddr)
{
	stHid.ucAddress = ucAddr; 
	if(ucAddr > 0) {
		stHid.iState = USB_STATE_ADDRESSED; 
	} else {
		stHid.iState = USB_STATE_DEFAULT; 
	}
}

static void
HID_deviceDetached(void)
{
	if (USB_STATE_CONFIGURED == stHid.iState) {
		HID_detach(); 
	}
	stHid.iState = USB_STATE_DETACH; 
	stHid.ucAddress = 0;
	stHid.ucCurrentConfig = 0;
	stHid.ucCurrentAltSetting = 0;
}


/* ----------------------------------------------------------------------------- */
/*                           Standard Device Requests                            */
/* ----------------------------------------------------------------------------- */

static USB_err_t
HID_devreqGetStatus(uint8_t bmRequestType, uint16_t wIndex, uint16_t *outStatus)
{
	USB_err_t iErr = USB_EINVAL; 
	USB_idesc_t *pstIdesc; 

	switch(USB_DR_GETRECIP(bmRequestType)) {
	case USB_DR_RECIP_DEVICE:       /* GetStatus(Device) */
		if (wIndex == 0) {
			USB_cdesc_t *pstCdesc; 
			pstCdesc = stHid.stConfigs[0].pstCdesc; 
			if (pstCdesc->bmAttributes & USB_CA_SELF_POWERED) {
				*outStatus |= USB_SD_SELF_POWERED;
			}
			if (pstCdesc->bmAttributes & USB_CA_REMOTE_WAKEUP) {
				*outStatus |= USB_SD_REMOTE_WAKEUP;
			}
			iErr = USB_OK;
		}
		break;

	case USB_DR_RECIP_INTERFACE:    /* GetStatus(Interface) */
		pstIdesc = stHid.stConfigs[0].pstIdesc; 
		if(wIndex == pstIdesc->bInterfaceNumber) {
			*outStatus = 0;
			iErr = USB_OK;
		}
		break;
		
	case USB_DR_RECIP_ENDPOINT:     /* GetStatus(Endpoint) */
	default:
		break;
	}

	return iErr;
}

static USB_err_t
HID_devreqGetDescriptor(uint16_t wValue, uint16_t wIndex, uint16_t wLength, 
                        const void **ppvData, uint16_t *pusDataLen)
{
	USB_idesc_t *pstIdesc; 
	uint8_t ucType, ucIndex; 
	USB_err_t iErr = USB_EINVAL; 

	ucType = (uint8_t)((wValue >> 8) & 0xff); 
	ucIndex = (uint8_t)(wValue & 0xff); 

	log_info("%s ucType %d ucIndex %d", __func__, ucType, ucIndex);

	switch(ucType) {
	case USB_DESC_DEVICE:
		if((ucIndex != 0) || (wIndex != 0)) {
			break; 
		}
		*ppvData = stHid.pstDdesc; 
		*pusDataLen = min(wLength, USB_DDESC_SIZE); 
		iErr = USB_OK; 
		break; 

	case USB_DESC_CONFIG:
		if((ucIndex >= stHid.pstDdesc->bNumConfigurations) || (wIndex != 0)) {
			break; 
		}
		*ppvData = stHid.stConfigs[0].pstCdesc; 
		*pusDataLen = min(wLength, CDESC_FULL_SIZE); 
		iErr = USB_OK; 
		break; 

	case USB_DESC_STRING:
		if((ucIndex >= NUM_SDESCS) ||
		   ((ucIndex != 0) && (wIndex != USB_LANGID_EN_US))) {
			break; 
		}
		*ppvData = stHid.ppstSdescs[ucIndex]; 
		*pusDataLen = min(wLength, (uint16_t)stHid.ppstSdescs[ucIndex]->bLength);
		iErr = USB_OK; 
		break; 
	
	case USB_DESC_HID:
		if(ucIndex != 0) { // Descriptor Index
			break; 
		}
		// Check Interface number
		pstIdesc = stHid.stConfigs[0].pstIdesc; 
		if(wIndex != pstIdesc->bInterfaceNumber) {
			break;
		}
		*ppvData = stHid.stConfigs[0].pstHdesc;
		*pusDataLen = min(wLength, (uint16_t)stHid.stConfigs[0].pstHdesc->bLength);
		iErr = USB_OK; 
		break; 
	
	case USB_DESC_HID_REPORT:
		if(ucIndex >= UHC_NUM_RDESCS) { // Descriptor Index
			break; 
		}
		// Check Interface number
		pstIdesc = stHid.stConfigs[0].pstIdesc; 
		if(wIndex != pstIdesc->bInterfaceNumber) {
			break;
		}
		*ppvData = stHid.pvRdescs[ucIndex];
		*pusDataLen = min(wLength, USB_GETW(stHid.stConfigs[0].pstHdesc->list[ucIndex].wDescriptorLength));
		iErr = USB_OK; 
		break; 
		
	case USB_DESC_HID_PHYSICAL:
		// no physical descriptor
		break;

	default:
		break; 
	}

	return iErr; 
}


static USB_err_t
HID_devreqSetConfig(uint8_t ucCfgValue)
{
	USB_err_t iErr = USB_EINVAL; 
	
	switch(stHid.ucCurrentConfig) {
	case 0:
		switch(ucCfgValue)
		{
		case 0:
			// 0 -> 0 : Stay addressed state.
			iErr = USB_OK;
			break;
		case 1:
			// 0 -> 1 : Configured. Transitions to Configured state
			stHid.iState = USB_STATE_CONFIGURED; 
			stHid.ucCurrentConfig = ucCfgValue; 
			stHid.ucCurrentAltSetting = 0; 
			iErr = HID_attach(); 
			
			#ifdef UHC_INTR_OUT_EP
			// Prepare for next SetReport
			USB_err_t iErr = HID_IntrOutXfer();
			log_info("%s Next xfer started. %d", __func__, iErr);
			#endif
			break;
		default:
			// 0 -> X : Request Error
			break;
		}
		break;
		
	case 1:
		switch(ucCfgValue)
		{
		case 0:
			// 1 -> 0 : Unconfigured. Transitions to Addressed state.
			HID_detach(); 
			stHid.iState = USB_STATE_ADDRESSED; 
			stHid.ucCurrentConfig = 0; 
			stHid.ucCurrentAltSetting = 0; 
			iErr = USB_OK; 
			break;
		case 1:
			// 1 -> 1 : Stay Configured state. reset toggle
			HID_detach();
			iErr = HID_attach(); 
			break;
		default:
			// 1 -> X : Request Error
			break;
		}
		break;

	default:
		// X : Internal Error
		break;
	}
	
	return iErr; 
}

static USB_err_t
HID_devreqSetInterface(uint16_t wValue, uint16_t wIndex)
{
	USB_err_t iErr = USB_EINVAL; 
	USB_idesc_t *pstIdesc; 
	
	pstIdesc = stHid.stConfigs[0].pstIdesc; 
	
	// InterfaceNumber, AlternateSettingNumber Supported ?
	if(wIndex != pstIdesc->bInterfaceNumber) {
		goto end;
	}
	if(wValue != pstIdesc->bAlternateSetting) {
		goto end;
	}
	
	// Reset toggle to zero
	HID_detach();
	iErr = HID_attach();

end:
	return iErr;
}

/* ----------------------------------------------------------------------------- */
/*                        Class Specific Device Requests                         */
/* ----------------------------------------------------------------------------- */

static void
HID_clearContext(void)
{
	#ifdef UHC_INTR_OUT_EP
	memset(&stHid.stOutXfer, 0, sizeof(stHid.stOutXfer));
	#endif
	memset(&stHid.stInXfer, 0, sizeof(stHid.stInXfer));
	stHid.ucInXferReportID = 0;
}

static USB_err_t HID_IntrInXfer(uint8_t id, uint32_t length)
{
	stHid.stInXfer.pvBuffer = &hid_GetReport[id][0];
	stHid.stInXfer.usLen    = length;
	stHid.ucInXferReportID  = id;
	return HID_startIntrXfer(HID_EP_INTRIN, &stHid.stInXfer);
}
#ifdef UHC_INTR_OUT_EP
static USB_err_t HID_IntrOutXfer(uint8_t id, uint32_t length)
{
	stHid.stOutXfer.pvBuffer = &hid_SetReport[id][0];
	stHid.stOutXfer.usLen    = length;
	return HID_startIntrXfer(HID_EP_INTROUT, &stHid.stOutXfer);
}
#endif

/**
*	@fn			-	bool USBS_HID_PutReport(uint8_t id, const uint8_t *buf, uint32_t length);
*	@brief		-	This function is called to prepare the asynchronous report data.
*	@return		-	error
**/ 
USBS_STATUS USBS_HID_PutReport(uint8_t id, const uint8_t *buf, uint32_t length)
{
	if(stHid.iState != USB_STATE_CONFIGURED) {
		return USBS_ERROR_USB_STATE;
	}
	
	if (NULL != stHid.stInXfer.pvBuffer) {
		log_warning("%s transfer busy", __func__);
		return USBS_ERROR_BUSY;
	}
	
	#ifdef DEBUG
	if (!(UHC_MAX_GETREPORT_SIZE >= length)) {
		log_warning("%s assert(UHC_MAX_GETREPORT_SIZE >= length)", __func__);
		assert(0);
	}
	#endif
	memcpy(&hid_GetReport[id][0], buf, length);
	USB_err_t iErr = HID_IntrInXfer(id, length);
	return (iErr == USB_OK) ? USBS_OK : USBS_ERROR;
}

/**
 * @brief      ClassRequest, GetReport
 * @param      
 * @return     
 */
static USB_err_t
HID_clsreqGetReport(const USB_devReq_t *pstReq)
{
	USB_idesc_t *pstIdesc; 

	if((USB_DR_GETRECIP(pstReq->bmRequestType) != USB_DR_RECIP_INTERFACE) ||
	   (USB_DR_GETDIR(pstReq->bmRequestType) != USB_DR_DIR_D2H) ){
		return USB_EINVAL; 
	}
	pstIdesc = stHid.stConfigs[0].pstIdesc; 
	if(USB_GETW(pstReq->wIndex) != pstIdesc->bInterfaceNumber) {
		return USB_EINVAL; 
	}
	
	uint8_t ucReportType = (uint8_t)((USB_GETW(pstReq->wValue) >> 8) & 0xff); 
	uint8_t ucReportID   = (uint8_t) (USB_GETW(pstReq->wValue) & 0xff); 
	USB_err_t iErr = USB_EINVAL;

	switch(ucReportType)
	{
	case USB_HID_REPORT_INPUT:
		{
			uint32_t length = 0;
			bool ret = USBS_HID_GetReport(USBS_HID_KIND_CTRL, ucReportID, &hid_GetReport[ucReportID][0], &length);
			#ifdef DEBUG
			if (!(UHC_MAX_GETREPORT_SIZE >= length)) {
				log_warning("%s assert(UHC_MAX_GETREPORT_SIZE >= length)", __func__);
				assert(0);
			}
			#endif
			if (ret) {
				iErr = USB_devReqSendData(&hid_GetReport[ucReportID][0], min(length, USB_GETW(pstReq->wLength)), NULL);
			} else {
				iErr = USB_EIO;
			}
		}
		break;
	case USB_HID_REPORT_OUTPUT:
	case USB_HID_REPORT_FEATURE:
	default:
		break;
	}
	
	return iErr;
}

/**
 * @brief      ClassRequest, SetReport, Data receive done
 */
static void devreqSetReportDone(const struct USB_deviceRequest *pstReq)
{
	uint8_t ucReportID   = (uint8_t) (USB_GETW(pstReq->wValue) & 0xff); 
	if (USBS_HID_SetReport(USBS_HID_KIND_CTRL, ucReportID, &hid_SetReport[ucReportID][0], min(UHC_MAX_SETREPORT_SIZE, USB_GETW(pstReq->wLength)))) {
		USB_devReqSendStatus();
	} else {
		USB_devReqStall();
	}
}

/**
 * @brief      ClassRequest, SetReport
 * @param      
 * @return     
 */
static USB_err_t
HID_clsreqSetReport(const USB_devReq_t *pstReq)
{
	USB_idesc_t *pstIdesc; 

	if((USB_DR_GETRECIP(pstReq->bmRequestType) != USB_DR_RECIP_INTERFACE) ||
	   (USB_DR_GETDIR(pstReq->bmRequestType) != USB_DR_DIR_H2D) ){
		return USB_EINVAL; 
	}
	pstIdesc = stHid.stConfigs[0].pstIdesc; 
	if(USB_GETW(pstReq->wIndex) != pstIdesc->bInterfaceNumber) {
		return USB_EINVAL; 
	}

	uint8_t ucReportType = (uint8_t)((USB_GETW(pstReq->wValue) >> 8) & 0xff); 
	uint8_t ucReportID   = (uint8_t) (USB_GETW(pstReq->wValue) & 0xff); 
	USB_err_t iErr = USB_EINVAL;

	switch(ucReportType)
	{
	case USB_HID_REPORT_INPUT:
		break; 
	case USB_HID_REPORT_OUTPUT:
		iErr = USB_devReqRecvData(&hid_SetReport[ucReportID][0], min(UHC_MAX_SETREPORT_SIZE, USB_GETW(pstReq->wLength)), devreqSetReportDone);
		break;
	case USB_HID_REPORT_FEATURE:
	default:
		break;
	}
	
	return iErr; 
}

/**
 * @brief      ClassRequest
 * @param      USB_clsIF_tを参照
 * @return     
 */
static USB_err_t
HID_devreqClassRequest(const USB_devReq_t *pstReq)
{
	if(stHid.iState != USB_STATE_CONFIGURED) {
		return USB_EINVAL; 
	}
	if(USB_DR_GETTYPE(pstReq->bmRequestType) != USB_DR_TYPE_CLASS) {
		return USB_EINVAL; 
	}

	USB_err_t iErr = USB_OK;
	switch(pstReq->bRequest) {
	case USB_DR_HID_GET_REPORT  :
		iErr = HID_clsreqGetReport(pstReq); 
		break;
	case USB_DR_HID_GET_IDLE    :
		{
			uint8_t ucReportID   = (uint8_t) (USB_GETW(pstReq->wValue) & 0xff); 
			if (ucReportID < UHC_NUM_RDESCS) {
				iErr = USB_devReqSendData(&hid_Idle[ucReportID], sizeof(uint8_t), NULL);
			} else {
				iErr = USB_EINVAL;
			}
		}
		break;
	case USB_DR_HID_GET_PROTOCOL:
		{
			iErr = USB_devReqSendData(&hid_Protocol, sizeof(uint8_t), NULL);
		}
		break;
	case USB_DR_HID_SET_REPORT  :
		iErr = HID_clsreqSetReport(pstReq); 
		break;
	case USB_DR_HID_SET_IDLE    :
		{
			uint8_t ucIdle       = (uint8_t)((USB_GETW(pstReq->wValue) >> 8) & 0xff); 
			uint8_t ucReportID   = (uint8_t) (USB_GETW(pstReq->wValue) & 0xff); 
			if (ucReportID < UHC_NUM_RDESCS) {
				hid_Idle[ucReportID] = ucIdle;
				iErr = USB_devReqSendStatus();
				// not implement
				// set_timer(ucIdle);
				// when expired, call USBS_HID_GetReport(USBS_HID_KIND_IDLE);, set_timer(ucIdle); again.
			} else {
				iErr = USB_EINVAL;
			}
		}
		break;
	case USB_DR_HID_SET_PROTOCOL:
		{
			uint8_t ucProtocol   = (uint8_t) (USB_GETW(pstReq->wValue) & 0xff); 
			hid_Protocol = ucProtocol;
			iErr = USB_devReqSendStatus();
		}
		break;
	default:
		iErr = USB_EINVAL;
		break; 
	}
	
	return iErr; 
}

/* ============================================================================= */
/*                          HID       Class Transfer                             */
/* ============================================================================= */

/* EP Number */
static const uint8_t eptable[] = {
	UHC_IN_EPNUM,  // EPn, IN, Interrupt,
	#ifdef UHC_INTR_OUT_EP
	UHC_OUT_EPNUM, // EPm, OUT, Interrupt,
	#endif
};

inline static USB_err_t HID_abortEP(HID_ep_t iEp)
{
	return USB_abortEP(USB_ep_addr(eptable[iEp], iEp == HID_EP_INTRIN ? 1 : 0)); 
}

inline static USB_err_t HID_closeEP(HID_ep_t iEp)
{
	return USB_closeEP(USB_ep_addr(eptable[iEp], iEp == HID_EP_INTRIN ? 1 : 0)); 
}

inline static USB_err_t HID_startIntrXfer(HID_ep_t iEp, USB_xfer_t *pstXfer)
{
	return USB_xferEP(USB_ep_addr(eptable[iEp], iEp == HID_EP_INTRIN ? 1 : 0), pstXfer); 
}


/**
 * @brief      IntrIN エンドポイントの転送完了コールバック関数
 * @param      pstXfer  転送情報
 * @return     
 */
static void
HID_doneIntrInXfer(USB_xfer_t *pstXfer)
{
	pstXfer->pvBuffer = NULL;
	
	if (USB_OK != pstXfer->iStatus) {
		// Transfer abort or stall
		log_warning("%s iStatus %d", __func__, pstXfer->iStatus);
		return;
	}
	
	uint32_t length = 0;
	bool ret = USBS_HID_GetReport(USBS_HID_KIND_INT, stHid.ucInXferReportID, &hid_GetReport[stHid.ucInXferReportID][0], &length);
	#ifdef DEBUG
	if (!(UHC_MAX_GETREPORT_SIZE >= length)) {
		log_warning("%s assert(UHC_MAX_GETREPORT_SIZE >= length)", __func__);
		assert(0);
	}
	#endif
	
	if (ret) {
		USB_err_t iErr = HID_IntrInXfer(stHid.ucInXferReportID, length);
		log_info("%s Report data available, next xfer started. %d", __func__, iErr);
	}
}

#ifdef UHC_INTR_OUT_EP
static void
HID_doneIntrOutXfer(USB_xfer_t *pstXfer)
{
	pstXfer->pvBuffer = NULL;
	
	if (USB_OK != pstXfer->iStatus) {
		// Transfer abort or stall
		log_warning("%s iStatus %d", __func__, pstXfer->iStatus);
		return;
	}
	
	uint8_t ucReportID; //TODO: ReportID parse from received data.
	uint32_t length;
	bool ret;
	ret = USBS_HID_SetReport(USBS_HID_KIND_INT, ucReportID, &hid_SetReport[ucReportID][0], pstXfer->usActLen);
	
	// Prepare for next SetReport
	USB_err_t iErr = HID_IntrOutXfer();
	log_info("%s Next xfer started. %d", __func__, iErr);
}
#endif

/* ----------------------------------------------------------------------------- */
/*          HID Routine                                                          */
/* ----------------------------------------------------------------------------- */

USB_clsIF_t ClassIF = {
	.stRequests.fnGetStatus             = HID_devreqGetStatus,
	// .stRequests.fnClearFeature = 
	// .stRequests.fnSetFeature = 
	.stRequests.fnGetDescriptor         = HID_devreqGetDescriptor, 
	// .stRequests.fnGetConfig             = , 
	.stRequests.fnSetConfig             = HID_devreqSetConfig, 
	// .stRequests.fnGetInterface          = ,
	.stRequests.fnSetInterface          = HID_devreqSetInterface,
	.stRequests.fnClassRequest          = HID_devreqClassRequest, 
	
	.stChanges.fnClassInitialize        = HID_init,
	.stChanges.fnClassUninitialize      = HID_fini,
	.stChanges.fnDeviceReset            = HID_deviceReset, 
	.stChanges.fnDeviceAttached         = HID_deviceAttached, 
	.stChanges.fnDeviceAddressed        = HID_deviceAddressed, 
	// .stChanges.fnDeviceConfigured = 
	.stChanges.fnDeviceDetached         = HID_deviceDetached, 
	// .stChanges.fnDeviceSuspend          = , 
	// .stChanges.fnDeviceResume          = , 
	// .stChanges.fnEPStallCleared         = , 
	// .stChanges.fnAfterXferDone          = , 
}; 


static const USB_ddesc_t stDeviceDesc __attribute__ ((aligned (32)))= {
	USB_DDESC_SIZE,             /* bLength */
	USB_DESC_DEVICE,            /* bDescriptorType */
	{0x00, 0x02},               /* bcdUSB */
	USB_CLASS_INTERFACE,        /* bDeviceClass */
	USB_SUBCLASS_INTERFACE,     /* bDeviceSubClass */
	USB_PROTOCOL_INTERFACE,     /* bDeviceProtocol */
	USC_MAXPACKETSIZE0,         /* bMaxPacketSize0 */
	{0x30, 0x09},               /* idVendor  : TOSHIBA */
	{0x03, 0x17},               /* idProduct */
	{0x01, 0x00},               /* bcdDevice */
	0x01,                       /* iManufacturer */
	0x02,                       /* iProduct */
	0x03,                       /* iSerialNumber */
	0x01,                       /* bNumConfigurations */
}; 

/* HID Report #1 (keyboard) */
// dt2_4, Keybrd.hid
static const uint8_t pucReportDesc0[] __attribute__ ((aligned (32)))= {
	0x05, 0x01,
	0x09, 0x06, 
	0xA1, 0x01, 
	0x05, 0x07, 
	0x19, 0xE0, 
	0x29, 0xE7, 
	0x15, 0x00, 
	0x25, 0x01, 
	0x75, 0x01, 
	0x95, 0x08, 
	0x81, 0x02, 
	0x95, 0x01, 
	0x75, 0x08, 
	0x81, 0x03, 
	0x95, 0x05, 
	0x75, 0x01, 
	0x05, 0x08, 
	0x19, 0x01, 
	0x29, 0x05, 
	0x91, 0x02, 
	0x95, 0x01, 
	0x75, 0x03, 
	0x91, 0x03, 
	0x95, 0x06, 
	0x75, 0x08, 
	0x15, 0x00, 
	0x25, 0x65, 
	0x05, 0x07, 
	0x19, 0x00, 
	0x29, 0x65, 
	0x81, 0x00, 
	0xC0, 
};

/* String #0 (LANGID) */
static const uint8_t pucStringDesc0[] __attribute__ ((aligned (32)))= {
	0x04,                       /* bLength */
	USB_DESC_STRING,            /* bDescriptorType */
	(uint8_t)( USB_LANGID_EN_US & 0xff),
	(uint8_t)((USB_LANGID_EN_US >> 8) & 0xff), /* wLangID code 0 = en_US */
}; 

/* String #1 (iManufacturer in device desc) */
static const uint8_t pucStringDesc1[] __attribute__ ((aligned (32)))= {
	2 + 14,                     /* bLength */
	USB_DESC_STRING,            /* bDescriptorType */
	'T', 0x00,
	'O', 0x00,
	'S', 0x00,
	'H', 0x00,
	'I', 0x00,
	'B', 0x00,
	'A', 0x00,
}; 

/* String #2 (iProduct in device desc) */
static const uint8_t pucStringDesc2[] __attribute__ ((aligned (32)))= {
	2 + 12,                     /* bLength */
	USB_DESC_STRING,            /* bDescriptorType */
	'T', 0x00,
	'Z', 0x00,
	'1', 0x00,
	'0', 0x00,
	'0', 0x00,
	'0', 0x00,
}; 

/* String #3 (iSerialNumber in device desc) */
static const uint8_t pucStringDesc3[] __attribute__ ((aligned (32))) = {
	2 + 14,                     /* bLength */
	USB_DESC_STRING,            /* bDescriptorType */
	'0', 0x00,
	'0', 0x00,
	'0', 0x00,
	'0', 0x00,
	'0', 0x00,
	'0', 0x00,
	'2', 0x00,
}; 

/* Config descriptor buffer */
static uint8_t  ppucConfigDescs[1][CDESC_FULL_SIZE] __attribute__ ((aligned (32))); 

/**
 * @brief      USB HID class layerの初期化
 * @param      なし
 * @return     
 */
static USB_err_t
HID_init(void)
{
	USB_cdesc_t *pstCdesc; 
	USB_idesc_t *pstIdesc; 
	USB_hiddesc_t *pstHdesc; 
	USB_edesc_t *pstEdesc; 

	memset(&stHid, 0, sizeof(stHid)); 

	/* Device Descriptor */
	stHid.pstDdesc = &stDeviceDesc; 


	/* Configuration Descriptor */
	for (int i=0; i<1; i++){
		stHid.stConfigs[0].pstCdesc = (USB_cdesc_t *)&ppucConfigDescs[i][0]; 
		stHid.stConfigs[0].pstIdesc = (USB_idesc_t *)&ppucConfigDescs[i][USB_CDESC_SIZE]; 
		stHid.stConfigs[0].pstHdesc = (USB_hiddesc_t *)&ppucConfigDescs[i][USB_CDESC_SIZE + USB_IDESC_SIZE]; 
		for(int j=0; j<HID_EP_NUMEPS; j++) {
			stHid.stConfigs[0].ppstEdescs[j]  = (USB_edesc_t *)&ppucConfigDescs[i][USB_CDESC_SIZE + USB_IDESC_SIZE + USB_HIDDESC_SIZE + (j * USB_EDESC_SIZE)]; 
		}
	}
	
	pstCdesc = stHid.stConfigs[0].pstCdesc; 
	pstIdesc = stHid.stConfigs[0].pstIdesc; 
	pstHdesc = stHid.stConfigs[0].pstHdesc; 
	pstEdesc = stHid.stConfigs[0].ppstEdescs[0];  // array head

	/* Config descriptor */
	pstCdesc->bLength             = USB_CDESC_SIZE; 
	pstCdesc->bDescriptorType     = USB_DESC_CONFIG; 
	USB_SETW(pstCdesc->wTotalLength, CDESC_FULL_SIZE); 
	pstCdesc->bNumInterfaces      = 0x01; 
	pstCdesc->bConfigurationValue = 0x01; 
	pstCdesc->iConfiguration      = 0x00; 
	pstCdesc->bmAttributes        = 0x80; // SelfPower=0, RemoteWake=0
	pstCdesc->bMaxPower           = 0x0F;

	/* Interface descriptor */
	pstIdesc->bLength            = USB_IDESC_SIZE; 
	pstIdesc->bDescriptorType    = USB_DESC_INTERFACE; 
	pstIdesc->bInterfaceNumber   = 0x00; 
	pstIdesc->bAlternateSetting  = 0x00; 
	pstIdesc->bNumEndpoints      = 1;
	pstIdesc->bInterfaceClass    = USB_CLASS_HID;
	pstIdesc->bInterfaceSubClass = 0;
	pstIdesc->bInterfaceProtocol = 0;
	pstIdesc->iInterface         = 0x00; 

	/* HID Descriptor */
	pstHdesc->bLength            = USB_HIDDESC_SIZE;
	pstHdesc->bDescriptorType    = USB_DESC_HID;
	USB_SETW(pstHdesc->bcdHID, 0x0111);
	pstHdesc->bCountryCode       = 0;
	pstHdesc->bNumDescriptors    = UHC_NUM_RDESCS;
	// Set Report descriptor(s)
	for (int i=0; i<UHC_NUM_RDESCS; i++) {
		pstHdesc->list[i].bDescriptorType   = USB_DESC_HID_REPORT;
		switch(i) {
		case 0:
			USB_SETW(pstHdesc->list[i].wDescriptorLength, sizeof(pucReportDesc0));
			stHid.pvRdescs[i]  = (const void *)pucReportDesc0;
			break;
		default:
			break;
		}
	}
	
	// HID_ep_t の内容と一致させること。
	/* endpoint descriptor EP3IN */
	pstEdesc->bLength            = USB_EDESC_SIZE; 
	pstEdesc->bDescriptorType    = USB_DESC_ENDPOINT; 
	pstEdesc->bEndpointAddress   = USB_EP_DIR_IN | eptable[HID_EP_INTRIN];
	pstEdesc->bmAttributes       = USB_EP_INTR; 
	USB_SETW(pstEdesc->wMaxPacketSize, UHC_MAX_GETREPORT_SIZE);
	pstEdesc->bInterval          = UHC_IN_INTERVAL; 

	#ifdef UHC_INTR_OUT_EP
	/* endpoint descriptor EP2OUT */
	pstEdesc->bLength            = USB_EDESC_SIZE; 
	pstEdesc->bDescriptorType    = USB_DESC_ENDPOINT; 
	pstEdesc->bEndpointAddress   = USB_EP_DIR_OUT | eptable[HID_EP_INTROUT];
	pstEdesc->bmAttributes       = USB_EP_INTR; 
	USB_SETW(pstEdesc->wMaxPacketSize, UHC_MAX_SETREPORT_SIZE);
	pstEdesc->bInterval          = UHC_OUT_INTERVAL; 
	#endif

	/* String Descriptor */
	// NUM_SDESCS
	stHid.ppstSdescs[0] = (const USB_sdesc_t *)pucStringDesc0; 
	stHid.ppstSdescs[1] = (const USB_sdesc_t *)pucStringDesc1; 
	stHid.ppstSdescs[2] = (const USB_sdesc_t *)pucStringDesc2; 
	stHid.ppstSdescs[3] = (const USB_sdesc_t *)pucStringDesc3; 
	
	/* Init context */
	stHid.iState = USB_STATE_DETACH;

	return USB_OK; 
}

/**
 * @brief      USB HID class layerの終了
 * @param      なし
 * @return     
 */
static USB_err_t
HID_fini(void)
{
	return USB_OK;
}

/**
 * @brief      USB HID class layerの開始
 * @param      なし
 * @return     
 */
static USB_err_t
HID_attach(void)
{
	USB_err_t iErr; 
	const USB_edesc_t *pstEdesc; 

	if(stHid.iState != USB_STATE_CONFIGURED) {
		return USB_ENXIO; 
	}

	pstEdesc = stHid.stConfigs[0].ppstEdescs[HID_EP_INTRIN]; 
	iErr = USB_openEP(pstEdesc, HID_doneIntrInXfer);
	if(iErr != USB_OK) {
		log_warning("%s USB_openEP(InterruptIN) failed iErr %d", __func__, iErr);
		goto end; 
	}

	#ifdef UHC_INTR_OUT_EP
	pstEdesc = stHid.stConfigs[0].ppstEdescs[HID_EP_INTROUT]; 
	iErr = USB_openEP(pstEdesc, HID_doneIntrOutXfer);
	if(iErr != USB_OK) {
		log_warning("%s USB_openEP(InterruptOUT) failed iErr %d", __func__, iErr);
		goto end; 
	}
	#endif

end:
	return iErr; 
}

/**
 * @brief      USB HID class layerの終了
 * @param      なし
 * @return     
 */
static USB_err_t
HID_detach(void)
{
	USB_err_t iErr = USB_EINVAL; 

	iErr = HID_abortEP(HID_EP_INTRIN);
	if(iErr != USB_OK) {
		log_warning("%s HID_abortEP(HID_EP_INTRIN) failed iErr %d", __func__, iErr);
	}
	
	iErr = HID_closeEP(HID_EP_INTRIN);
	if(iErr != USB_OK) {
		log_warning("%s HID_closeEP(HID_EP_INTRIN) failed iErr %d", __func__, iErr);
	}

	#ifdef UHC_INTR_OUT_EP
	iErr = HID_abortEP(HID_EP_INTROUT);
	if(iErr != USB_OK) {
		log_warning("%s HID_abortEP(HID_EP_INTROUT) failed iErr %d", __func__, iErr);
	}
	
	iErr = HID_closeEP(HID_EP_INTROUT);
	if(iErr != USB_OK) {
		log_warning("%s HID_closeEP(HID_EP_INTROUT) failed iErr %d", __func__, iErr);
	}
	#endif

	HID_clearContext();

	return iErr; 
}
