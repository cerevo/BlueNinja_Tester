/**
 * @file   TZ01_system.c
 * @brief  USB detect test target for Cerevo CDP-TZ01B.
 *         (HIDに見える何もしないデバイス)
 * @author Cerevo Inc.
 */

/*
Copyright 2015 Cerevo Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

/** Includes **/
#include <stdint.h>
#include <stdbool.h>
/* MCU support. */
#include "TZ10xx.h"
#include "PMU_TZ10xx.h"
#include "GPIO_TZ10xx.h"

/* Board support. */
#include "TZ01_system.h"
#include "utils.h"

/* TOSHIBA USB Stack */
#include "gpio_wrapper.h"
#include "usbstack.h"
#include "usb_clsif_hid.h"
#include "usbdebug.h"


extern TZ10XX_DRIVER_PMU  Driver_PMU;
extern TZ10XX_DRIVER_GPIO Driver_GPIO;

uint8_t led_blink;

/**
 * USB初期化
 */
bool USB_init(void)
{
    /* Setting USB Clock */
    Driver_PMU.SelectClockSource(PMU_CSM_USB,  PMU_CLOCK_SOURCE_PLL);
    Driver_PMU.SetPrescaler(PMU_CD_USBB, 2);   // 24MHz
    Driver_PMU.SetPrescaler(PMU_CD_USBI, 1);   // 48MHz

    /* VBUS Detect */
    Driver_PMU.StandbyInputBuffer(PMU_IO_FUNC_GPIO_0, 0);
    Driver_GPIO_Wrapper.Initialize();
    Driver_GPIO_Wrapper.PowerControl(ARM_POWER_FULL);

    return true;
}

/**
 * USB接続
 */
void USB_connect(void)
{
    /* USB Stack Initialize & Connect */
    USBS_Initialize(&Driver_GPIO_Wrapper);
    USBS_Connect();
    
    TZ01_system_tick_start(USRTICK_NO_BLE_MAIN, 500);
    led_blink = 0;
}

/**
 * USBスタックの実行
 */
void USB_run(void)
{
    USBS_Thread();
    if (TZ01_system_tick_check_timeout(USRTICK_NO_BLE_MAIN)) {
        TZ01_system_tick_start(USRTICK_NO_BLE_MAIN, 500);
        led_blink = (led_blink == 0) ? 1 : 0;
        Driver_GPIO.WritePin(11, led_blink);
    }
}

/**
 * USB接続断
 */
void USB_disconnect(void)
{
    /* Disconnect & USB Stack Uninitialize */
    USBS_Disconnect();
    USBS_Uninitialize();
    
    TZ01_system_tick_stop(USRTICK_NO_BLE_MAIN);
}


/* TOSHIBAのUSBスタックのハンドラ */
void USBS_Configured (uint8_t config, uint8_t alt)
{
    /* 何もしない */
}

void USBS_Suspend (bool enter)
{
    /* 何もしない */
}

bool USBS_HID_GetReport(uint8_t kind, uint8_t id, uint8_t *buf, uint32_t *length)
{
    /* 何もしない */
    return true;
}

bool USBS_HID_SetReport(uint8_t kind, uint8_t id, const uint8_t *buf, uint32_t length)
{
    /* 何もしない */
    return true;
}
