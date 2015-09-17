/**
 * @file   main.c
 * @brief  Application main.
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
#include <ctype.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "PMU_TZ10xx.h"
#include "GPIO_TZ10xx.h"
#include "RTC_TZ10xx.h"

#include "TZ01_system.h"
#include "TZ01_console.h"

#include "utils.h"

#include "IO.h"
#include "USB.h"
#include "BLE.h"

extern TZ10XX_DRIVER_GPIO Driver_GPIO;
extern TZ10XX_DRIVER_RTC Driver_RTC;

static uint8_t send_buf[80];

/*** ��� ***/
typedef enum {
    STAT_INIT,      //������
    STAT_POWINIT,   //�d��������
    STAT_MODSEL,    //���[�h�I��
    STAT_IO,        //I/O�e�X�g
    STAT_USB,       //USB�e�X�g
    STAT_BLE,       //BLE�e�X�g
    STAT_TERM,      //�e�X�g�I��
}   TESTER_STAT;
TESTER_STAT stat = STAT_INIT;

/*** �C�x���g(�R�}���h��M) ***/
typedef enum {
    EVT_NONE,
    EVT_CMD_M,  //���[�h�ύX�R�}���h��M
    EVT_CMD_G,  //DI�`�F�b�N�R�}���h��M
    EVT_CMD_A,  //ADC�`�F�b�N�R�}���h��M
    EVT_CMD_I,  //I2C PingPong���s�R�}���h��M
    EVT_CMD_9,  //9���Z���T�v���l�擾�R�}���h��M
    EVT_CMD_P,  //�C���Z���T�v���l�擾�R�}���h��M
    EVT_CMD_C,  //�[�dID�X�e�[�^�X�擾�R�}���h��M
    EVT_CMD_R,  //RTC�擾�R�}���h��M
}   TESTER_EVT;

#define CMD_BUF_LEN (5)
typedef struct {
    uint8_t data[CMD_BUF_LEN];
    uint8_t r_idx;
    uint8_t w_idx;
}   CMD_BUF;

static CMD_BUF cmd_buf;

static void init_cmd_buf(CMD_BUF *buf)
{
    memset(buf->data, 0, CMD_BUF_LEN);
    buf->r_idx = 0;
    buf->w_idx = 0;
}

static int put_cmd_buf(CMD_BUF *buf, uint8_t val)
{
    buf->data[buf->w_idx] = val;
    buf->w_idx = (buf->w_idx + 1) % CMD_BUF_LEN;
    if (buf->w_idx == buf->r_idx) {
        buf->r_idx = (buf->r_idx + 1) % CMD_BUF_LEN;
    }
    
    return val;
}

static int get_cmd_buf(CMD_BUF *buf)
{
    uint8_t val;
    
    if (buf->w_idx == buf->r_idx) {
        return -1;  //Buffer EMPTY
    }
    val = buf->data[buf->r_idx];
    buf->r_idx = (buf->r_idx + 1) % CMD_BUF_LEN;
    
    return val;
}

static bool recv_cmd(TESTER_EVT *evt, uint16_t *param)
{
    uint8_t     c;
    uint8_t     cmd;
    uint16_t    tmp;
    
    *evt = EVT_NONE;
    if (TZ01_console_getc(&c)) {
        if (c == '\r') {
            //�R�}���h�I�[
            cmd = get_cmd_buf(&cmd_buf);
            //�p�����[�^�𐔒l�ɕϊ�(3��10�i�����񁨐��l)
            *param = 0;
            for (int i = 2; i >= 0; i--) {
                c = get_cmd_buf(&cmd_buf);
                if (isdigit(c)) {
                    tmp = c - '0';
                    for (int j = 0; j < i; j++) {
                        tmp *= 10;
                    }
                    *param += tmp;
                } else {
                    //10�i����������Ȃ�
                    return false;  //Invalid parameter.
                }
            }
            //�R�}���h�̔���
            switch (cmd) {
            case 'm':
                if ((*param < 0) || ((*param > 3) && (*param != 999))) {
                    return false;  //Invalid parameter;
                }
                *evt = EVT_CMD_M;
                break;
            case 'g':
                if (*param != 0) {
                    return false;  //Invalid parameter;
                }
                *evt = EVT_CMD_G;
                break;
            case 'a':
                if ((*param < 0) || ((*param > 3) && (*param !=102))) {
                    return false;
                }
                *evt = EVT_CMD_A;
                break;
            case 'i':
                if (*param != 0) {
                    return false;
                }
                *evt = EVT_CMD_I;
                break;
            case '9':
                if (*param != 0) {
                    return false;
                }
                *evt = EVT_CMD_9;
                break;
            case 'p':
                if (*param != 0) {
                    return false;
                }
                *evt = EVT_CMD_P;
                break;
            case 'c':
                if (*param != 0) {
                    return false;
                }
                *evt = EVT_CMD_C;
                break;
            case 'r':
                if (*param != 0) {
                    return false;
                }
                *evt = EVT_CMD_R;
                break;
            default:
                return false;  //Invalid command.
            }
        } else {
            //�o�b�t�@�ɒǉ�
            put_cmd_buf(&cmd_buf, c);
        }
        return true;
    }
    return false;
}

/** Button **/
typedef struct {
    uint8_t     on      :1;
    uint8_t     hist    :5;
    uint16_t    on_cnt;
}   BtnStat;

static void sw_dev_init(void)
{
    //Sw1
    Driver_PMU.StandbyInputBuffer(PMU_IO_FUNC_GPIO_1, 0);
    Driver_GPIO.Configure(1, GPIO_DIRECTION_INPUT_HI_Z, GPIO_EVENT_DISABLE, NULL);
    //Sw2
    Driver_PMU.StandbyInputBuffer(PMU_IO_FUNC_GPIO_6, 0);
    Driver_GPIO.Configure(6, GPIO_DIRECTION_INPUT_HI_Z, GPIO_EVENT_DISABLE, NULL);
}

static void sw_init(BtnStat *btn, uint8_t on)
{
    btn->on = on;
    btn->hist = 0x1f;
    btn->on_cnt = 0;
}

static void sw_update(BtnStat *btn, int gpio)
{
    uint32_t val;
    
    btn->hist = (btn->hist << 1) & 0x1f;    //�������X�V
    if (Driver_GPIO.ReadPin(gpio, &val) == GPIO_ERROR) {
        return;
    }
    btn->hist |= (val & 0x01);
    
    if (btn->hist == 0x1f) {
        //OFF
        btn->on = 0;
        btn->on_cnt = 0;
    }
    if (btn->hist == 0x00) {
        //ON
        btn->on = 1;
        btn->on_cnt++;
    }
}

static uint8_t sw_state(BtnStat *btn)
{
    if (btn->on == 1) {
        if (btn->on_cnt < 200) {
            return 1;   //ON
        } else {
            return 2;   //HOLD
        }
    } else {
        return 0;       //OFF
    }
}

static BtnStat sw1_stat;
static BtnStat sw2_stat;

/* ��ԑJ�ڎ��̏��� */
//INIT��Ԃ֑J��
static void init_state_entry(void)
{
    RTC_TIME now = {
        0, 0, 0, 1, 1, 15, 4 /* 2015-01-01(��) 00:00:00 */
    };
    stat = STAT_INIT;
    /* Initialize */
    BLE_init();             //BLE��������
    TZ01_system_init();     //�V�X�e���̏�����
    TZ01_console_init();    //�R���\�[���̏�����(�Ύ���)
    /* GPIO */
    //PowerHold
    Driver_GPIO.Configure(3, GPIO_DIRECTION_OUTPUT_2MA, GPIO_EVENT_DISABLE, NULL);
    //Sw1, Sw2
    sw_dev_init();
    sw_init(&sw1_stat, 1);
    sw_init(&sw2_stat, 0);
    //LED1
    Driver_GPIO.Configure(10, GPIO_DIRECTION_OUTPUT_2MA, GPIO_EVENT_DISABLE, NULL);
    //LED2
    Driver_GPIO.Configure(11, GPIO_DIRECTION_OUTPUT_2MA, GPIO_EVENT_DISABLE, NULL);
    /* RTC */
    Driver_PMU.StartClockSource(PMU_CLOCK_SOURCE_OSC32K);
    Driver_PMU.SelectClockSource(PMU_CSM_RTC, PMU_CLOCK_SOURCE_32K);
    Driver_PMU.SetPrescaler(PMU_CD_RTC, 1);
    Driver_RTC.Initialize();
    Driver_RTC.SetTime(&now);
    /* I/O Tester */
    IO_init();
    
    /* USB */
    USB_init();
    
    TZ01_console_puts("TZ1 TEST PROGRAM\r\n");

    /* PowerHold��Hi�ɂ��� */
    Driver_GPIO.WritePin(3, 1);
    
    TZ01_system_tick_start(USRTICK_NO_GPIO_INTERVAL, 10);
}

//POWINIT�ւ̑J��
static void powinit_state_entry(void)
{
    stat = STAT_POWINIT;
    /* LED1��Hi�ɂ��� */
    Driver_GPIO.WritePin(10, 1);
    
    TZ01_console_puts("POWER INIT\r\n");
    
    TZ01_system_tick_start(USRTICK_NO_GPIO_INTERVAL, 10);
}

//MODSEL��Ԃ֑J��
static void modsel_state_entry(void)
{
    stat = STAT_MODSEL;
    TZ01_console_puts("RUNNING\r\n");
    
    Driver_GPIO.WritePin(11, 0);
    TZ01_system_tick_start(USRTICK_NO_GPIO_INTERVAL, 10);
}

static void io_state_entry(void)
{
    stat = STAT_IO;
    TZ01_console_puts("IO\r\n");
    
    Driver_GPIO.WritePin(11, 1);
}

//USB��Ԃ֑J��
static void usb_state_entry(void)
{
    stat = STAT_USB;
    TZ01_console_puts("USB\r\n");
    
    //USB�ڑ�
    USB_connect();
    
    TZ01_system_tick_start(USRTICK_NO_GPIO_INTERVAL, 10);
}

//BLE��Ԃ֑J��
static void ble_state_entry(void)
{
    stat = STAT_BLE;
    TZ01_console_puts("BLE\r\n");
    
    TZ01_system_tick_start(USRTICK_NO_GPIO_INTERVAL, 10);
}

//TERM��Ԃ֑J��
static void term_state_entry(void)
{
    stat = STAT_TERM;
    
    TZ01_console_puts("TERMINATED\r\n");
    TZ01_system_tick_start(USRTICK_NO_GPIO_INTERVAL, 200);
}

/* �e��Ԃ̐U�镑�� */
static void init_state_func(TESTER_EVT event, uint16_t param)
{
    static int prev_state = 1;
    int curr_state;
    
    if (TZ01_system_tick_check_timeout(USRTICK_NO_GPIO_INTERVAL)) {
        /* 10ms event*/
        TZ01_system_tick_start(USRTICK_NO_GPIO_INTERVAL, 10);
        sw_update(&sw1_stat, 1); //Sw1
        curr_state = sw_state(&sw1_stat);
        if ((prev_state == 0) && (curr_state == 1)) {
            powinit_state_entry();  //POWINIT��Ԃ֑J��
        }
        prev_state = curr_state;
    }
}

static void powinit_state_func(TESTER_EVT event, uint16_t param)
{
    if (TZ01_system_tick_check_timeout(USRTICK_NO_GPIO_INTERVAL)) {
        /* 10ms event*/
        TZ01_system_tick_start(USRTICK_NO_GPIO_INTERVAL, 10);
        sw_update(&sw2_stat, 6); //Sw2
        if (sw_state(&sw2_stat) == 1) {
            modsel_state_entry();  //MODESEL��Ԃ֑J��
        }
    }
}

static void modsel_state_func(TESTER_EVT event, uint16_t param)
{
    RTC_TIME now;
    static uint8_t led_v = 0;
    
    if (TZ01_system_tick_check_timeout(USRTICK_NO_GPIO_INTERVAL)) {
        /* 500ms event*/
        TZ01_system_tick_start(USRTICK_NO_GPIO_INTERVAL, 500);
        led_v = (led_v == 0) ? 1 : 0;
        Driver_GPIO.WritePin(10, led_v);
    }
    
    if (event == EVT_CMD_M) {
        switch (param) {
        case 1:     // I/O�e�X�g
            io_state_entry();   //IO��Ԃ�
            break;
        case 2:     // USB�e�X�g
            usb_state_entry();  //USB��Ԃ�
            break;
        case 3:     // BLE�e�X�g
            ble_state_entry();  //BLE��Ԃ�
            break;
        case 999:   // �I��
            term_state_entry(); //TERM��Ԃ�
            break;
        default:
            break;
        }
    }
    
    if (event == EVT_CMD_R) {
        Driver_RTC.GetTime(&now);
        sprintf(
            send_buf, "{\"cmd\":\"r000\",\"year\":%d,\"month\":%d,\"day\":%d,\"hour\":%d,\"minute\":%d,\"second\":%d}\r\n",
            now.year + 2000, now.mon, now.mday, now.hour, now.min, now.sec
        );
        TZ01_console_puts(send_buf);
    }
}

static void io_state_func(TESTER_EVT event, uint16_t param)
{
    static uint8_t led_v = 0;
    static uint8_t cnt_timeout = 0;

    /* UART Echo */
    IO_uart1_echo();
    
    /* ������s */
    if (TZ01_system_tick_check_timeout(USRTICK_NO_GPIO_INTERVAL)) {
        TZ01_system_tick_start(USRTICK_NO_GPIO_INTERVAL, 100);
        
        /* 500ms event*/
        if ((cnt_timeout % 5) == 0) {
            led_v = (led_v == 0) ? 1 : 0;
            Driver_GPIO.WritePin(10, led_v);
        }
        
        /* 100ms event */
        IO_periodic_run();
        
        cnt_timeout = (cnt_timeout + 1) % 10;
    }
    //�v���l��ԐM
    switch (event) {
    case EVT_CMD_G: //GPIO
        IO_respons_gpio();
        break;
    case EVT_CMD_A: //ADC
        IO_respons_adc(param);
        break;
    case EVT_CMD_I: //I2C PingPong
        IO_i2c_pingpong();
        break;
    case EVT_CMD_9: //9axis
        IO_respons_9axis();
        break;
    case EVT_CMD_P: //Airpressure
        IO_respons_airpressure();
        break;
    case EVT_CMD_C: //BatteryCharger
        IO_respons_charger_status();
        break;
    }
    //���[�h�؂�ւ�
    if (event == EVT_CMD_M) {
        switch (param) {
        case 0:
            modsel_state_entry();   //MODSEL��Ԃ֑J��
            break;
        default:
            break;
        }
    }
}

static void usb_state_func(TESTER_EVT event, uint16_t param)
{
    static uint8_t led_v = 0;

    USB_run();
    
    if (TZ01_system_tick_check_timeout(USRTICK_NO_GPIO_INTERVAL)) {
        /* 500ms event*/
        TZ01_system_tick_start(USRTICK_NO_GPIO_INTERVAL, 500);
        led_v = (led_v == 0) ? 1 : 0;
        Driver_GPIO.WritePin(10, led_v);
        Driver_GPIO.WritePin(11, led_v);
    }

    if (event == EVT_CMD_M) {
        switch (param) {
        case 0:
            USB_disconnect();       //USB�ؒf
            modsel_state_entry();   //MODSEL��Ԃ֑J��
            break;
        default:
            break;
        }
    }
    
}

static void ble_state_func(TESTER_EVT event, uint16_t param)
{
    static uint8_t led1_v = 0, led2_v = 0;
    static uint8_t cnt_timeout = 0;
    
    BLE_run();
    
    if (TZ01_system_tick_check_timeout(USRTICK_NO_GPIO_INTERVAL)) {
        /* 100ms event*/
        TZ01_system_tick_start(USRTICK_NO_GPIO_INTERVAL, 100);
        //LED1�_��(500ms)
        if ((cnt_timeout % 5) == 0) {
            led1_v = (led1_v == 0) ? 1 : 0;
            Driver_GPIO.WritePin(10, led1_v);
        }
        //LED2�_��(200ms)
        if ((cnt_timeout % 2) == 0) {
            led2_v = (led2_v == 0) ? 1 : 0;
            Driver_GPIO.WritePin(11, led2_v);
        }
        cnt_timeout = (cnt_timeout + 1) % 10;
    }
    
    if (event == EVT_CMD_M) {
        switch (param) {
        case 0:
            BLE_stop();             //BLE��~
            modsel_state_entry();   //MODSEL��Ԃ֑J��
            break;
        default:
            break;
        }
    }
}

static void term_state_func(TESTER_EVT event, uint16_t param)
{
    if (TZ01_system_tick_check_timeout(USRTICK_NO_GPIO_INTERVAL)) {
        //LED1, LED2 OFF
        Driver_GPIO.WritePin(10, 0);
        Driver_GPIO.WritePin(11, 0);
        //PowerHold����
        Driver_GPIO.WritePin(3, 0);
        
        for (;;) {}
    }
}

static void (*state_func[])(TESTER_EVT event, uint16_t param) = {
    init_state_func,
    powinit_state_func,
    modsel_state_func,
    io_state_func,
    usb_state_func,
    ble_state_func,
    term_state_func
};

int main(void)
{
    init_state_entry();

    //USB_connect();
    //TZ01_console_puts("USB connected\r\n");
    TESTER_EVT  event;
    uint16_t    param;
    for (;;) {
        TZ01_system_run();
        
        recv_cmd(&event, &param);
        (state_func[stat])(event, param);
    }
    
    TZ01_console_puts("Program terminated.\r\n");
    return 0;
}
