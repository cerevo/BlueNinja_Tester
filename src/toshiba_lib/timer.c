/**
 * @file timer.c
 * @brief 
 * @version V0.0
 * @date $Date:: 2014-09-19 15:34:33 +0900 #$
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

#include "timer.h"
#include "TZ10xx.h"
#include <string.h>

void usleep(uint32_t usec)
{
	int32_t t = ( SystemCoreClock / 1000000 ) * usec / 8; // 8cycle/loop
	while (--t > 0) {
		__NOP();
		__NOP();
		__NOP();
		__NOP();
	}
}

typedef struct _timer_ctx {
	volatile uint32_t count;
	void (* handler)(uint32_t id, uintptr_t arg);
	uintptr_t arg;
} timer_ctx_t;

static timer_ctx_t ctx[TIMER_NUM_MAX];

void init_timer(void)
{
	NVIC_ClearPendingIRQ(SysTick_IRQn);
	stop_timer();
	memset(ctx, 0, sizeof(ctx));
	start_timer();
}

void stop_timer(void)
{
	NVIC_DisableIRQ(SysTick_IRQn);
	SysTick->CTRL = 0;
}

void start_timer(void)
{
	SysTick_Config(SystemCoreClock / 1000);  /* Configure SysTick to generate an interrupt every millisecond */
	NVIC_EnableIRQ(SysTick_IRQn);
}

bool set_timer(uint32_t id, uint32_t count, void (* handler)(uint32_t id, uintptr_t arg), uintptr_t arg)
{
	bool result = false;
	
	NVIC_DisableIRQ(SysTick_IRQn);
	if (ctx[id].count == 0) {
		ctx[id].count   = count;
		ctx[id].handler = handler;
		ctx[id].arg     = arg;
		result = true;
	}
	NVIC_EnableIRQ(SysTick_IRQn);
	
	return result;
}

bool clr_timer(uint32_t id)
{
	bool result = false;
	
	NVIC_DisableIRQ(SysTick_IRQn);
	if (ctx[id].count) {
		ctx[id].handler = NULL;
		ctx[id].arg     = 0;
		ctx[id].count   = 0;
		result = true;
	}
	NVIC_EnableIRQ(SysTick_IRQn);
	
	return result;
}

void SysTick_Handler (void)
{
	for(int id=0; id<TIMER_NUM_MAX; id++) {
		if (ctx[id].count) {
			ctx[id].count--;
			if (ctx[id].count == 0) {
				if (ctx[id].handler) {
					ctx[id].handler(id, ctx[id].arg);
				}
			}
		}
	}
}

