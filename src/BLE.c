/**
 * @file ble_msg.c
 * @breaf Cerevo CDP-TZ01B sample program.
 * BLE PingPong
 *
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

#include <stdio.h>
#include "TZ10xx.h"
#include "PMU_TZ10xx.h"
#include "RNG_TZ10xx.h"

#include "twic_interface.h"
#include "blelib.h"

#include "TZ01_system.h"
#include "TZ01_console.h"

#define BNMSG_MTU    (23)

extern TZ10XX_DRIVER_RNG Driver_RNG;
extern TZ10XX_DRIVER_GPIO Driver_GPIO;

static uint8_t msg[80];

static uint64_t bnpp_bdaddr  = 0xc0ce00000000;   //

/*--- GATT profile definition ---*/
const uint8_t bnmsg_gap_device_name[] = "CDP-TZ01B_01";
const uint8_t bnmsg_gap_appearance[] = {0x00, 0x00};

const uint8_t bnmsg_di_manufname[] = "Cerevo";
const uint8_t bnmsg_di_fw_version[] = "0.1";
const uint8_t bnmsg_di_sw_version[] = "0.1";
const uint8_t bnmsg_di_model_string[] = "CDP-TZ01B_01";

/* BLElib unique id. */
enum {
    BLE_GATT_UNIQUE_ID_GAP_SERVICE = 0,
    BLE_GATT_UNIQUE_ID_GAP_DEVICE_NAME,
    BLE_GATT_UNIQUE_ID_GAP_APPEARANCE,
    BLE_GATT_UNIQUE_ID_DI_SERVICE,
    BLE_GATT_UNIQUE_ID_DI_MANUF_NAME,
    BLE_GATT_UNIQUE_ID_DI_FW_VERSION,
    BLE_GATT_UNIQUE_ID_DI_SW_VERSION,
    BLE_GATT_UNIQUE_ID_DI_MODEL_STRING,
};

/* GAP */
const BLELib_Characteristics gap_device_name = {
    BLE_GATT_UNIQUE_ID_GAP_DEVICE_NAME, 0x2a00, 0, BLELIB_UUID_16,
    BLELIB_PROPERTY_READ,
    BLELIB_PERMISSION_READ | BLELIB_PERMISSION_WRITE,
    bnmsg_gap_device_name, sizeof(bnmsg_gap_device_name),
    NULL, 0
};
const BLELib_Characteristics gap_appearance = {
    BLE_GATT_UNIQUE_ID_GAP_APPEARANCE, 0x2a01, 0, BLELIB_UUID_16,
    BLELIB_PROPERTY_READ,
    BLELIB_PERMISSION_READ,
    bnmsg_gap_appearance, sizeof(bnmsg_gap_appearance),
    NULL, 0
};
const BLELib_Characteristics *const gap_characteristics[] = { &gap_device_name, &gap_appearance };
    const BLELib_Service gap_service = {
    BLE_GATT_UNIQUE_ID_GAP_SERVICE, 0x1800, 0, BLELIB_UUID_16,
    true, NULL, 0,
    gap_characteristics, 2
};

/* DIS(Device Informatin Service) */
const BLELib_Characteristics di_manuf_name = {
    BLE_GATT_UNIQUE_ID_DI_MANUF_NAME, 0x2a29, 0, BLELIB_UUID_16,
    BLELIB_PROPERTY_READ,
    BLELIB_PERMISSION_READ,
    bnmsg_di_manufname, sizeof(bnmsg_di_manufname),
    NULL, 0
};
const BLELib_Characteristics di_fw_version = {
    BLE_GATT_UNIQUE_ID_DI_FW_VERSION, 0x2a26, 0, BLELIB_UUID_16,
    BLELIB_PROPERTY_READ,
    BLELIB_PERMISSION_READ,
    bnmsg_di_fw_version, sizeof(bnmsg_di_fw_version),
    NULL, 0
};
const BLELib_Characteristics di_sw_version = {
    BLE_GATT_UNIQUE_ID_DI_SW_VERSION, 0x2a28, 0, BLELIB_UUID_16,
    BLELIB_PROPERTY_READ,
    BLELIB_PERMISSION_READ,
    bnmsg_di_sw_version, sizeof(bnmsg_di_sw_version),
    NULL, 0
};
const BLELib_Characteristics di_model_string = {
    BLE_GATT_UNIQUE_ID_DI_MODEL_STRING, 0x2a24, 0, BLELIB_UUID_16,
    BLELIB_PROPERTY_READ,
    BLELIB_PERMISSION_READ,
    bnmsg_di_model_string, sizeof(bnmsg_di_model_string),
    NULL, 0
};
const BLELib_Characteristics *const di_characteristics[] = {
    &di_manuf_name, &di_fw_version, &di_sw_version, &di_model_string
};
const BLELib_Service di_service = {
    BLE_GATT_UNIQUE_ID_DI_SERVICE, 0x180a, 0, BLELIB_UUID_16,
    true, NULL, 0,
    di_characteristics, 4
};

/* Service list */
const BLELib_Service *const bnpp_service_list[] = {
    &gap_service, &di_service
};

/*- INDICATION data -*/
uint8_t bnmsg_advertising_data[] = {
    0x02, /* length of this data */
    0x01, /* AD type = Flags */
    0x06, /* LE General Discoverable Mode = 0x02 */
    /* BR/EDR Not Supported (i.e. bit 37
     * of LMP Extended Feature bits Page 0) = 0x04 */

    0x06, /* length of this data */
    0x08, /* AD type = Short local name */
    'T',  /* (T) */
    'Z',  /* (Z) */
    '1',
    '0',
    '3',

    0x05, /* length of this data */
    0x03, /* AD type = Complete list of 16-bit UUIDs available */
    0x00, /* Generic Access Profile Service 1800 */
    0x18,
    0x0A, /* Device Information Service 180A */
    0x18,
    };

    uint8_t bnmsg_scan_resp_data[] = {
    0x02, /* length of this data */
    0x01, /* AD type = Flags */
    0x06, /* LE General Discoverable Mode = 0x02 */
    /* BR/EDR Not Supported (i.e. bit 37
     * of LMP Extended Feature bits Page 0) = 0x04 */

    0x02, /* length of this data */
    0x0A, /* AD type = TX Power Level (1 byte) */
    0x00, /* 0dB (-127...127 = 0x81...0x7F) */

    0x0d, /* length of this data */
    0x09, /* AD type = Complete local name */
    'C', 'D', 'P', '-', 'T', 'Z', '0', '1', 'B', '_', '0', '1' /* CDP-TZ01B_01 */
};

/*=== BlueNinja messenger application ===*/
static uint64_t central_bdaddr;

/*= BLElib callback functions =*/
void connectionCompleteCb(const uint8_t status, const bool master, const uint64_t bdaddr, const uint16_t conn_interval)
{
    central_bdaddr = bdaddr;

    BLELib_requestMtuExchange(BNMSG_MTU);

    TZ01_system_tick_start(USRTICK_NO_BLE_MAIN, 100);
}

void connectionUpdateCb(const uint8_t status, const uint16_t conn_interval, const uint16_t conn_latency)
{
}

void disconnectCb(const uint8_t status, const uint8_t reason)
{
    TZ01_system_tick_stop(USRTICK_NO_BLE_MAIN);
}

BLELib_RespForDemand mtuExchangeDemandCb(const uint16_t client_rx_mtu_size, uint16_t *resp_mtu_size)
{
    *resp_mtu_size = BNMSG_MTU;
    return BLELIB_DEMAND_ACCEPT;
}

void mtuExchangeResultCb(const uint8_t status, const uint16_t negotiated_mtu_size)
{
}

void notificationSentCb(const uint8_t unique_id)
{
}

void indicationConfirmCb(const uint8_t unique_id)
{
}

void updateCompleteCb(const uint8_t unique_id)
{
}

void queuedWriteCompleteCb(const uint8_t status)
{
}

BLELib_RespForDemand readoutDemandCb(const uint8_t *const unique_id_array, const uint8_t unique_id_num)
{
    return BLELIB_DEMAND_ACCEPT;
}

BLELib_RespForDemand writeinDemandCb(const uint8_t unique_id, const uint8_t *const value, const uint8_t value_len)
{
    return BLELIB_DEMAND_ACCEPT;
}

void writeinPostCb(const uint8_t unique_id, const uint8_t *const value, const uint8_t value_len)
{
}

void isrNewEventCb(void)
{
    /* this sample always call BLELib_run() */
}

void isrWakeupCb(void)
{
    /* this callback is not used currently */
}

BLELib_CommonCallbacks tz01_common_callbacks = {
    connectionCompleteCb,
    connectionUpdateCb,
    mtuExchangeResultCb,
    disconnectCb,
    NULL,
    isrNewEventCb,
    isrWakeupCb
};

BLELib_ServerCallbacks tz01_server_callbacks = {
    mtuExchangeDemandCb,
    notificationSentCb,
    indicationConfirmCb,
    updateCompleteCb,
    queuedWriteCompleteCb,
    readoutDemandCb,
    writeinDemandCb,
    writeinPostCb,
};

/** Global **/

int BLE_init(void)
{
    if (TZ1EM_STATUS_OK != tz1emInitializeSystem())
        return 1; /* Must not use UART for LOG before twicIfLeIoInitialize. */
    
    /* create random bdaddr */
    uint32_t randval;
    Driver_PMU.SetPowerDomainState(PMU_PD_ENCRYPT, PMU_PD_MODE_ON);
    Driver_RNG.Initialize();
    Driver_RNG.PowerControl(ARM_POWER_FULL);
    Driver_RNG.Read(&randval);
    Driver_RNG.Uninitialize();
    bnpp_bdaddr |= (uint64_t)randval;

    /* initialize BLELib */
    int ret;
    BLELib_initialize(bnpp_bdaddr, BLELIB_BAUDRATE_2304, &tz01_common_callbacks, &tz01_server_callbacks, NULL, NULL);
    ret = BLELib_registerService(bnpp_service_list, 2);
    BLELib_setLowPowerMode(BLELIB_LOWPOWER_ON);

    return ret;
}

static bool is_adv = false;
static uint8_t led_blink = 0;

int BLE_run(void)
{
    int ret, res = 0;
    BLELib_State state;
    bool has_event;

    state = BLELib_getState();
    has_event = BLELib_hasEvent();

    switch (state) {
        case BLELIB_STATE_UNINITIALIZED:
        case BLELIB_STATE_INITIALIZED:
            break;

        case BLELIB_STATE_READY:
            if (is_adv == false) {
                ret = BLELib_startAdvertising(bnmsg_advertising_data, sizeof(bnmsg_advertising_data), bnmsg_scan_resp_data, sizeof(bnmsg_scan_resp_data));
                is_adv = true;
            }
            break;
        case BLELIB_STATE_ADVERTISING:
            is_adv = false;
            break;
        case BLELIB_STATE_ONLINE:
            if (TZ01_system_tick_check_timeout(USRTICK_NO_BLE_MAIN)) {
                TZ01_system_tick_start(USRTICK_NO_BLE_MAIN, 100);
                led_blink = (led_blink == 0) ? 1 : 0;
            }
            break;
        default:
            break;
    }

    if (has_event) {
        ret = BLELib_run();
    }

    return res;
}

void BLE_stop(void)
{
    switch (BLELib_getState()) {
        case BLELIB_STATE_ADVERTISING:
            BLELib_stopAdvertising();
            break;
        case BLELIB_STATE_ONLINE:
            BLELib_disconnect(central_bdaddr);
            break;
        default:
            break;
    }
}
