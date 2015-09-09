/**
 * @file debug.c
 * @brief debug output
 * @version V0.0
 * @date $Date:: 2014-10-17 17:43:09 +0900 #$
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

#include "usbdebug.h"

#ifdef USB_USE_UART0

#include "Driver_UART.h"
extern ARM_DRIVER_UART Driver_UART0 ;

/* VT100 escape sequence */
#define ESC_ATTR_RESET  "\033[0m"
#define ESC_ATTR_BG_RED "\033[41m"
#define ESC_CUR_UP      "\033[1A"
#define ESC_RESET       "\033[2J"
#define ESC_CLR_END     "\033[V"

static char ctmp[160];
static unsigned char print_mask;

/**
 @brief Wrapper functions for UART0 output
 */
#include <string.h>
static void print(const char *str)
{
	int len = strlen(str);
	
	while (len > 0) {
		int r = Driver_UART0.WriteData((const uint8_t *)str, len);
		if (r < 0) {
			return; /* error */
		} else {
			str += r;
			len -= r;
		}
	}
}

/**
 @brief callback function for Driver_UART0
 */
static void uart0_handler(ARM_UART_EVENT e)
{
	print(ESC_ATTR_BG_RED);
	switch (e) {
	case ARM_UART_EVENT_RX_OVERRUN:
		print("[RX_OVERRUN]");
		break;
	case ARM_UART_EVENT_RX_BREAK:
		print("[RX_BREAK]");
		break;
	case ARM_UART_EVENT_RX_PARITY_ERROR:
		print("[RX_PARITY_ERROR]");
		break;
	case ARM_UART_EVENT_RX_FRAMING_ERROR:
		print("[RX_FRAMING_ERROR]");
		break;
	case ARM_UART_EVENT_RX_TIMEOUT:
		print("[RX_TIMEOUT]");
		/* Flush rx buffer to avoid further timeout events */
		Driver_UART0.FlushRxBuffer();
		break;
	default:
		break;
	}
	print(ESC_ATTR_RESET);
}

void sample_print_set_mask(unsigned char mask)
{
	print_mask = mask;
}

unsigned char sample_print_get_mask(void)
{
	return print_mask;
}

/**
 @brief initialize function for Driver_UART0
 */
void sample_print_init(void)
{
	print_mask = 0;
	
	/* Setup UART0 */
	Driver_UART0.Initialize(uart0_handler,
	    (1u << ARM_UART_EVENT_RX_OVERRUN)
	  | (1u << ARM_UART_EVENT_RX_BREAK)
	  | (1u << ARM_UART_EVENT_RX_PARITY_ERROR)
	  | (1u << ARM_UART_EVENT_RX_FRAMING_ERROR)
	  | (1u << ARM_UART_EVENT_RX_TIMEOUT));
	
	Driver_UART0.Configure(
	  115200,
	  8,
	  ARM_UART_PARITY_NONE,
	  ARM_UART_STOP_BITS_1,
	  ARM_UART_FLOW_CONTROL_NONE);
	
	Driver_UART0.PowerControl(ARM_POWER_FULL);
}

#include <stdarg.h>
#include <stdio.h>
void sample_print(const char *format, ...)
{
	if (print_mask) return;
	
	va_list arg;
	va_start(arg, format);
	vsnprintf(ctmp, sizeof(ctmp)-2, format, arg);
	va_end(arg);
	
	strcat(ctmp, "\r\n");
	print(ctmp);
}

#else  /* USB_USE_UART0 */

void sample_print_set_mask(unsigned char mask) {}
unsigned char sample_print_get_mask(void) {return 1;}
void sample_print_init(void) {}
void sample_print(const char *format, ...) {}

#endif /* USB_USE_UART0 */
