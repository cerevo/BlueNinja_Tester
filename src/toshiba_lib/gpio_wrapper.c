/**
 * @file gpio_wrapper.c
 * @brief gpio wrapper
 * @version V0.0
 * @date $Date:: 2014-06-11 13:24:13 +0900 #$
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

#include "GPIO_TZ10xx.h"
extern TZ10XX_DRIVER_GPIO Driver_GPIO ;

#include "gpio_wrapper.h"

static uint32_t bmPinUse;
static uint8_t  NumOfPinUse;

/**
  \fn          static GPIO_GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRV_VERSION
*/
static ARM_DRV_VERSION GPIO_GetVersion(void)
{
	return Driver_GPIO.GetVersion();
}

/**
  \fn          static GPIO_CAPABILITIES GPIO_GetCapabilities (uint32_t pin)
  \brief       Get driver capabilities.
  \param[in]   pin      pin number
  \return      \ref GPIO_CAPABILITIES
*/
static GPIO_CAPABILITIES GPIO_GetCapabilities(uint32_t pin)
{
	return Driver_GPIO.GetCapabilities(pin);
}

/**
  \fn          static GPIO_STATUS GPIO_PowerControl (ARM_POWER_STATE  state)
  \brief       Controls GPIO Interface Power.
  \param[in]   state    Power state
  \return      \ref GPIO_STATUS
*/
static GPIO_STATUS GPIO_PowerControl (ARM_POWER_STATE  state)
{
	if (NumOfPinUse) return GPIO_ERROR; // Prevent power off during pin use.
	return Driver_GPIO.PowerControl(state);
}

/**
  \fn          static GPIO_STATUS GPIO_W_Initialize (void)
  \brief       Initialize GPIO Interface.
  \return      \ref GPIO_STATUS
*/
static GPIO_STATUS GPIO_Initialize (void)
{
	bmPinUse = 0;
	NumOfPinUse = 0;
	return Driver_GPIO.Initialize();
}

/**
  \fn          static GPIO_STATUS GPIO_Uninitialize (void)
  \brief       De-initialize GPIO Interface.
  \return      \ref GPIO_STATUS
*/
static GPIO_STATUS GPIO_Uninitialize (void)
{
	return Driver_GPIO.Uninitialize();
}


/**
  \fn          static GPIO_STATUS GPIO_Configure(uint32_t pin, GPIO_DIRECTION dir, GPIO_EVENT event, GPIO_SignalEvent_t cb_event)
  \brief       Configure GPIO pin.
  \param[in]   pin      pin number
  \param[in]   dir      GPIO direction
  \param[in]   event      GPIO event
  \param[in]   cb_event     Pointer to GPIO_SignalEvent
  \return      \ref GPIO_STATUS
*/
static GPIO_STATUS GPIO_Configure(uint32_t pin, GPIO_DIRECTION dir, GPIO_EVENT event, GPIO_SignalEvent_t cb_event)
{
	if (bmPinUse & (1 << pin)) return GPIO_ERROR; // Currently in use
	
	GPIO_STATUS status;
	status = Driver_GPIO.Configure(pin, dir, event, cb_event);
	if (GPIO_OK == status) {
		bmPinUse |= (1 << pin);
		NumOfPinUse++;
	}
	
	return status;
}

/**
  \fn          static GPIO_STATUS GPIO_ReadPin (uint32_t pin, uint32_t *val)
  \brief       Read value of a gpiopin.
  \param[in]   pin      pin number
  \param[out]  val      read value
  \return      \ref GPIO_STATUS
*/
static GPIO_STATUS GPIO_ReadPin(uint32_t pin, uint32_t *val)
{
	return Driver_GPIO.ReadPin(pin, val);
}

/**
  \fn          static GPIO_STATUS GPIO_Read ( uint32_t *val)
  \brief       Read  value of all gpiopins.
  \param[out]  val      read value
  \return      \ref GPIO_STATUS
*/
static GPIO_STATUS GPIO_Read(uint32_t *val)
{
	return Driver_GPIO.Read(val);
}

/**
  \fn          static GPIO_STATUS GPIO_WritePin ( uint32_t pin, uint32_t val)
  \brief       write value of a gpiopin.
  \param[in]  pin    pin number
  \param[in]  val      writing value
  \return      \ref GPIO_STATUS
*/
static GPIO_STATUS GPIO_WritePin( uint32_t pin, uint32_t val)
{
	return Driver_GPIO.WritePin(pin, val);
}

/**
  \fn          static GPIO_STATUS GPIO_Write ( uint32_t mask, uint32_t val)
  \brief       write value of all gpiopins.
  \param[in]  mask    pin mask
  \param[in]  val      writing value
  \return      \ref GPIO_STATUS
*/
static GPIO_STATUS GPIO_Write( uint32_t mask, uint32_t val)
{
	return Driver_GPIO.WritePin(mask, val);
}

/**
  \fn          static GPIO_STATUS GPIO_Unconfigure(uint32_t pin)
  \brief       Unconfigure GPIO pin.
  \param[in]   pin      pin number
  \return      \ref GPIO_STATUS
*/
static GPIO_STATUS GPIO_Unconfigure(uint32_t pin, GPIO_DIRECTION dir)
{
	if (0 == (bmPinUse & (1 << pin))) return GPIO_ERROR; // Currently not in use
	
	Driver_GPIO.Configure(pin, dir, GPIO_EVENT_DISABLE, NULL);
	bmPinUse &= ~(1 << pin);
	NumOfPinUse--;
	return GPIO_OK;
}


/* GPIO Driver Wrapper Control Block */
TZ10XX_DRIVER_GPIO_WRAPPER Driver_GPIO_Wrapper = {
	GPIO_GetVersion,
	GPIO_GetCapabilities,
	GPIO_Initialize,
	GPIO_Uninitialize,
	GPIO_PowerControl ,
	GPIO_Configure,
	GPIO_ReadPin,
	GPIO_Read,
	GPIO_WritePin,
	GPIO_Write,
	GPIO_Unconfigure
};

