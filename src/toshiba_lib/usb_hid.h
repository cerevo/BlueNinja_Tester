/**
 * @file usb_hid.h
 * @brief USB HID Class Define
 * @version V0.0
 * @date $Date:: 2014-07-09 11:12:54 +0900 #$
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

#ifndef __USB_HID_H__
#define __USB_HID_H__

#include "usb_desc.h"
#include "usb_clsif_hid.h"

/* ***************************************************************************** */
/*                         Constants, Macros & Types                             */
/* ***************************************************************************** */

/* --- Class, Subclass, Protocol codes --- */
#define USB_CLASS_HID               0x03

/* --- descriptor type (USB HID spec 7.2) --- */
#define USB_DESC_HID                0x21
#define USB_DESC_HID_REPORT         0x22
#define USB_DESC_HID_PHYSICAL       0x23

/* --- bRequest values in device request (USB HID spec 7.2) --- */
#define USB_DR_HID_GET_REPORT       0x01
#define USB_DR_HID_GET_IDLE         0x02
#define USB_DR_HID_GET_PROTOCOL     0x03
#define USB_DR_HID_SET_REPORT       0x09
#define USB_DR_HID_SET_IDLE         0x0A
#define USB_DR_HID_SET_PROTOCOL     0x0B

/* Report Type */
#define USB_HID_REPORT_INPUT        0x01
#define USB_HID_REPORT_OUTPUT       0x02
#define USB_HID_REPORT_FEATURE      0x03

typedef struct USB_hidDesc {
	USB_byte_t		bLength; 
	USB_byte_t		bDescriptorType; 
	USB_word_t		bcdHID; 
	USB_byte_t		bCountryCode; 
	USB_byte_t		bNumDescriptors; 
	struct _USB_hidDesc_list {
		USB_byte_t		bDescriptorType;
		USB_word_t		wDescriptorLength; 
	} list[UHC_NUM_RDESCS];
} USB_hiddesc_t; 
#define USB_HIDDESC_SIZE          (6 + (3 * UHC_NUM_RDESCS))

/* EP Index */
typedef enum {
	HID_EP_INTRIN   = 0, 
	#ifdef UHC_INTR_OUT_EP
	HID_EP_INTROUT  = 1,
	#endif
	HID_EP_NUMEPS, 
} HID_ep_t; 

#endif /* __USB_HID_H__ */
