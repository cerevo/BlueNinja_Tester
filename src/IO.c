/**
 * @file   main.c
 * @brief  TZ1 I/O Tester.
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
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/* MCU support. */
#include "TZ10xx.h"
#include "PMU_TZ10xx.h"
#include "GPIO_TZ10xx.h"
#include "SPI_TZ10xx.h"
#include "Driver_I2C.h"
#include "Driver_UART.h"
#include "ADCC12_TZ10xx.h"
#include "ADCC24_TZ10xx.h"

#include "TZ01_system.h"
#include "TZ01_console.h"
#include "MPU-9250.h"
#include "BMP280.h"
#include "BQ24250.h"

#include "utils.h"

extern TZ10XX_DRIVER_PMU  Driver_PMU;
extern TZ10XX_DRIVER_GPIO Driver_GPIO;
extern TZ10XX_DRIVER_SPI Driver_SPI3;       //9���Z���T
extern ARM_DRIVER_I2C Driver_I2C0;          //PingPong
extern ARM_DRIVER_I2C Driver_I2C1;          //�C���Z���T
extern ARM_DRIVER_I2C Driver_I2C2;          //�[�dIC
extern ARM_DRIVER_UART Driver_UART1;        //�u���C�N�A�E�g�{�[�h(LPC11U35)�o�RUSB�V���A��
extern TZ10XX_DRIVER_ADCC12 Driver_ADCC12;
extern TZ10XX_DRIVER_ADCC24 Driver_ADCC24;

/*
 RingBuffer
 */
#define DAT_BUF_LEN (6)
typedef struct {
    uint32_t data[DAT_BUF_LEN];
    uint8_t r_idx;
    uint8_t w_idx;
}   DAT_BUF;

static void init_dat_buf(DAT_BUF *buf)
{
    memset(buf->data, 0, DAT_BUF_LEN * sizeof(uint32_t));
    buf->r_idx = 0;
    buf->w_idx = 0;
}

static int put_dat_buf(DAT_BUF *buf, uint32_t val)
{
    buf->data[buf->w_idx] = val;
    buf->w_idx = (buf->w_idx + 1) % DAT_BUF_LEN;
    if (buf->w_idx == buf->r_idx) {
        buf->r_idx = (buf->r_idx + 1) % DAT_BUF_LEN;
    }
    
    return val;
}

static bool get_dat_buf(DAT_BUF *buf, uint32_t *out)
{
    if (buf->w_idx == buf->r_idx) {
        return false;  //Buffer EMPTY
    }
    *out = buf->data[buf->r_idx];
    buf->r_idx = (buf->r_idx + 1) % DAT_BUF_LEN;
    
    return true;
}

//�����O�o�b�t�@��ǂ݂����Ĕz��Ɋi�[
static int get_series_buf(DAT_BUF *buf, uint32_t *out_buf, uint32_t out_sz)
{
    int i;
    uint32_t val;
    for (i = 0; i < out_sz; i++) {
        if (get_dat_buf(buf, &val)) {
            out_buf[i] = val;
        } else {
            break;
        }
    }
    return i;
}

/***/
static DAT_BUF gyro_x, gyro_y, gyro_z;
static DAT_BUF acel_x, acel_y, acel_z;
static DAT_BUF magm_x, magm_y, magm_z;
static DAT_BUF airp;
static DAT_BUF adc12_0, adc12_1, adc12_2, adc12_3;
static DAT_BUF adc24_2;

static uint8_t charger_reg[7];
static uint8_t send_buf[100];

/*** ���[�J���֐� ***/
static int uint32_sort(const void *a, const void *b)
{
    if (*(uint32_t*)a > *(uint32_t*)b) {
        return 1;
    } else if (*(uint32_t*)a < *(uint32_t*)b) {
        return -1;
    } else {
        return 0;
    }
}
/* �����l��Ԃ� */
static uint32_t get_median(uint32_t *buf, uint8_t sz)
{
    qsort((void*)buf, sz, sizeof(uint32_t), uint32_sort);
    
    return buf[sz / 2];
}

/*** �O���[�o���֐� ***/

void IO_init(void)
{
    /* PMU */
    Driver_PMU.SelectClockSource(PMU_CSM_UART1, PMU_CLOCK_SOURCE_OSC12M);
    Driver_PMU.SelectClockSource(PMU_CSM_ADCC12, PMU_CLOCK_SOURCE_SIOSC4M);
    Driver_PMU.SelectClockSource(PMU_CSM_ADCC24, PMU_CLOCK_SOURCE_SIOSC4M);
    Driver_PMU.SetPrescaler(PMU_CD_UART1, 1);
    Driver_PMU.SetPrescaler(PMU_CD_ADCC12, 1);
    Driver_PMU.SetPrescaler(PMU_CD_ADCC24, 1);
    
    Driver_PMU.StandbyInputBuffer(PMU_IO_FUNC_GPIO_7, 0);
    Driver_PMU.StandbyInputBuffer(PMU_IO_FUNC_GPIO_8, 0);
    Driver_PMU.StandbyInputBuffer(PMU_IO_FUNC_GPIO_9, 0);
    Driver_PMU.StandbyInputBuffer(PMU_IO_FUNC_GPIO_16, 0);
    Driver_PMU.StandbyInputBuffer(PMU_IO_FUNC_GPIO_17, 0);
    Driver_PMU.StandbyInputBuffer(PMU_IO_FUNC_GPIO_18, 0);
    Driver_PMU.StandbyInputBuffer(PMU_IO_FUNC_GPIO_19, 0);
    Driver_PMU.StandbyInputBuffer(PMU_IO_FUNC_GPIO_20, 0);
    Driver_PMU.StandbyInputBuffer(PMU_IO_FUNC_GPIO_21, 0);
    Driver_PMU.StandbyInputBuffer(PMU_IO_FUNC_GPIO_22, 0);
    Driver_PMU.StandbyInputBuffer(PMU_IO_FUNC_GPIO_23, 0);
    
    /* GPIO */
    Driver_GPIO.Configure(7, GPIO_DIRECTION_INPUT_HI_Z, GPIO_EVENT_DISABLE, NULL);
    Driver_GPIO.Configure(8, GPIO_DIRECTION_INPUT_HI_Z, GPIO_EVENT_DISABLE, NULL);
    Driver_GPIO.Configure(9, GPIO_DIRECTION_INPUT_HI_Z, GPIO_EVENT_DISABLE, NULL);
    Driver_GPIO.Configure(16, GPIO_DIRECTION_INPUT_HI_Z, GPIO_EVENT_DISABLE, NULL);
    Driver_GPIO.Configure(17, GPIO_DIRECTION_INPUT_HI_Z, GPIO_EVENT_DISABLE, NULL);
    Driver_GPIO.Configure(18, GPIO_DIRECTION_INPUT_HI_Z, GPIO_EVENT_DISABLE, NULL);
    Driver_GPIO.Configure(19, GPIO_DIRECTION_INPUT_HI_Z, GPIO_EVENT_DISABLE, NULL);
    Driver_GPIO.Configure(20, GPIO_DIRECTION_INPUT_HI_Z, GPIO_EVENT_DISABLE, NULL);
    Driver_GPIO.Configure(21, GPIO_DIRECTION_INPUT_HI_Z, GPIO_EVENT_DISABLE, NULL);
    Driver_GPIO.Configure(22, GPIO_DIRECTION_INPUT_HI_Z, GPIO_EVENT_DISABLE, NULL);
    Driver_GPIO.Configure(23, GPIO_DIRECTION_INPUT_HI_Z, GPIO_EVENT_DISABLE, NULL);
    
    /* ADCC12 */
    Driver_ADCC12.Initialize(NULL, 0);
    
    Driver_ADCC12.SetScanMode(ADCC12_SCAN_MODE_CYCLIC, ADCC12_CHANNEL_0);
    Driver_ADCC12.SetScanMode(ADCC12_SCAN_MODE_CYCLIC, ADCC12_CHANNEL_1);
    Driver_ADCC12.SetScanMode(ADCC12_SCAN_MODE_CYCLIC, ADCC12_CHANNEL_2);
    Driver_ADCC12.SetScanMode(ADCC12_SCAN_MODE_CYCLIC, ADCC12_CHANNEL_3);
    Driver_ADCC12.SetDataFormat(ADCC12_CHANNEL_0, ADCC12_UNSIGNED);
    Driver_ADCC12.SetDataFormat(ADCC12_CHANNEL_1, ADCC12_UNSIGNED);
    Driver_ADCC12.SetDataFormat(ADCC12_CHANNEL_2, ADCC12_UNSIGNED);
    Driver_ADCC12.SetDataFormat(ADCC12_CHANNEL_3, ADCC12_UNSIGNED);
    Driver_ADCC12.SetComparison(ADCC12_CMP_DATA_0, 0x00, ADCC12_CMP_NO_COMPARISON, ADCC12_CHANNEL_0);
    Driver_ADCC12.SetComparison(ADCC12_CMP_DATA_0, 0x00, ADCC12_CMP_NO_COMPARISON, ADCC12_CHANNEL_1);
    Driver_ADCC12.SetComparison(ADCC12_CMP_DATA_0, 0x00, ADCC12_CMP_NO_COMPARISON, ADCC12_CHANNEL_2);
    Driver_ADCC12.SetComparison(ADCC12_CMP_DATA_0, 0x00, ADCC12_CMP_NO_COMPARISON, ADCC12_CHANNEL_3);
    
    Driver_ADCC12.SetFIFOOverwrite(ADCC12_FIFO_MODE_STREAM);
    Driver_ADCC12.SetSamplingPeriod(ADCC12_SAMPLING_PERIOD_64MS);
    Driver_ADCC12.PowerControl(ARM_POWER_FULL);
    Driver_ADCC12.Start();
    
    /* ADCC24 */
    Driver_ADCC24.Initialize(NULL, 0);
    
    Driver_ADCC24.SetScanMode(ADCC24_SCAN_MODE_CYCLIC, ADCC24_CHANNEL_2);
    Driver_ADCC24.SetDataFormat(ADCC24_CHANNEL_2, ADCC24_SIGNED);
    Driver_ADCC24.SetWatermark(ADCC24_CHANNEL_2, ADCC24_WATERMARK_4);
    Driver_ADCC24.SetOffset(ADCC24_CHANNEL_2, 0x00);
    Driver_ADCC24.SetComparison(ADCC24_CMP_DATA_0, 0x00, ADCC24_CMP_NO_COMPARISON, ADCC24_CHANNEL_2);
    Driver_ADCC24.SetDecimation(ADCC24_CHANNEL_2, ADCC24_DOWN_SAMPLING_1_1);
    
    Driver_ADCC24.SetFIFOOverwrite(ADCC24_FIFO_MODE_STREAM);
    Driver_ADCC24.SetSamplingPeriod(ADCC24_SAMPLING_PERIOD_64MS);
    Driver_ADCC24.PowerControl(ARM_POWER_FULL);
    Driver_ADCC24.Start();
    
    /* UART1(ECHO) */
    Driver_UART1.Initialize(0, 0);
    Driver_UART1.Configure(9600, 8, ARM_UART_PARITY_NONE, ARM_UART_STOP_BITS_1, ARM_UART_FLOW_CONTROL_NONE);
    Driver_UART1.PowerControl(ARM_POWER_FULL);
    
    /* I2C0(PingPong) */
    Driver_I2C0.Initialize(NULL);
    Driver_I2C0.PowerControl(ARM_POWER_FULL);
    Driver_I2C0.BusSpeed(ARM_I2C_BUS_SPEED_STANDARD);
    
    /* TZ1�I���{�[�h�Z���T/�[�dIC */
    if (MPU9250_drv_init(&Driver_SPI3)) {
        MPU9250_drv_start_maesure(MPU9250_BIT_ACCEL_FS_SEL_16G, MPU9250_BIT_GYRO_FS_SEL_2000DPS, MPU9250_BIT_DLPF_CFG_20HZ, MPU9250_BIT_A_DLPFCFG_20HZ);
    } else {
        TZ01_console_puts("MPU9250_drv_init() failed\r\n");
    }
    BMP280_drv_init(&Driver_I2C1);
    BQ24250_drv_init(&Driver_I2C2, true);
}

/* UART1�̃G�R�[ */
void IO_uart1_echo(void)
{
    uint8_t c;
    
    while (Driver_UART1.DataAvailable()) {
        Driver_UART1.ReadData(&c, 1);
        Driver_UART1.WriteData(&c, 1);
    }
}

/* I2C0��PingPong */
void IO_i2c_pingpong(void)
{
    uint8_t pong[5];
    memset(pong, 0, sizeof(pong));
    /* PING���M */
    Driver_I2C0.SendData(0x48, "PING", 5, true);
    /* PONG��M */
    Driver_I2C0.ReceiveData(0x48, pong, 5, true);
    pong[4] = '\0';
    sprintf(send_buf, "{\"cmd\":\"i000\",\"recv\":\"%s\"}\r\n", pong);
    TZ01_console_puts(send_buf);
}

/* GPIO�ǂݏo�����ʏo�� */
void IO_respons_gpio(void)
{
    uint32_t g7, g8, g9, g16, g17, g18, g19, g20, g21, g22, g23;
    
    Driver_GPIO.ReadPin(7, &g7);
    Driver_GPIO.ReadPin(8, &g8);
    Driver_GPIO.ReadPin(9, &g9);
    Driver_GPIO.ReadPin(16, &g16);
    Driver_GPIO.ReadPin(17, &g17);
    Driver_GPIO.ReadPin(18, &g18);
    Driver_GPIO.ReadPin(19, &g19);
    Driver_GPIO.ReadPin(20, &g20);
    Driver_GPIO.ReadPin(21, &g21);
    Driver_GPIO.ReadPin(22, &g22);
    Driver_GPIO.ReadPin(23, &g23);
    
    sprintf(
        send_buf, "{\"cmd\":\"g000\",\"gpio\":{\"7\":%d,\"8\":%d,\"9\":%d,\"16\":%d,\"17\":%d,\"18\":%d,\"19\":%d,\"20\":%d,\"21\":%d,\"22\":%d,\"23\":%d}}\r\n",
        g7, g8, g9, g16, g17, g18, g19, g20, g21, g22, g23
    );
    TZ01_console_puts(send_buf);
}

/* ADC12/ADC24�̌v���l��Ԃ�(����5��̌v���̒����l) */
void IO_respons_adc(uint8_t param)
{
    uint32_t val[5];
    uint32_t median;
    
    switch (param) {
    case 0:
        //ADC12 Ch0
        if (get_series_buf(&adc12_0, val, 5) == 5) {
            median = get_median(val, 5);
        } else {
            median = 0;
        }
        break;
    case 1:
        //ADC12 Ch1
        if (get_series_buf(&adc12_1, val, 5) == 5) {
            median = get_median(val, 5);
        } else {
            median = 0;
        }
        break;
    case 2:
        //ADC12 Ch2
        if (get_series_buf(&adc12_2, val, 5) == 5) {
            median = get_median(val, 5);
        } else {
            median = 0;
        }
        break;
    case 3:
        //ADC12 Ch3
        if (get_series_buf(&adc12_3, val, 5) == 5) {
            median = get_median(val, 5);
        } else {
            median = 0;
        }
        break;
    case 102:
        //ADC24 Ch2
        if (get_series_buf(&adc24_2, val, 5) == 5) {
            median = get_median(val, 5);
        } else {
            median = 0;
        }
        break;
    default:
        return;
    }
    
    sprintf(send_buf, "{\"cmd\":\"a%03d\",\"adc\":%d}\r\n", param, median);
    TZ01_console_puts(send_buf);
}

/* 9���Z���T�̌v�����ʂ�Ԃ�(����5��̌v���̒����l) */
void IO_respons_9axis(void)
{
    uint32_t val[5];
    uint32_t gy[3], ac[3], mm[3];
    
    /* �W���C�� */
    // X
    if (get_series_buf(&gyro_x, val, 5) == 5) {
        //�ߋ�5��̌v���̒����l��Ԃ�
        gy[0] = get_median(val, 5);
    } else {
        //0��Ԃ�(�ߋ�5�񕪂̌v���f�[�^�������Ė���)
        gy[0] = 0;
    }
    // Y
    if (get_series_buf(&gyro_y, val, 5) == 5) {
        //�ߋ�5��̌v���̒����l��Ԃ�
        gy[1] = get_median(val, 5);
    } else {
        //0��Ԃ�(�ߋ�5�񕪂̌v���f�[�^�������Ė���)
        gy[1] = 0;
    }
    // Z
    if (get_series_buf(&gyro_z, val, 5) == 5) {
        //�ߋ�5��̌v���̒����l��Ԃ�
        gy[2] = get_median(val, 5);
    } else {
        //0��Ԃ�(�ߋ�5�񕪂̌v���f�[�^�������Ė���)
        gy[2] = 0;
    }
    
    /* �����x */
    // X
    if (get_series_buf(&acel_x, val, 5) == 5) {
        //�ߋ�5��̌v���̒����l��Ԃ�
        ac[0] = get_median(val, 5);
    } else {
        //0��Ԃ�(�ߋ�5�񕪂̌v���f�[�^�������Ė���)
        ac[0] = 0;
    }
    // Y
    if (get_series_buf(&acel_y, val, 5) == 5) {
        //�ߋ�5��̌v���̒����l��Ԃ�
        ac[1] = get_median(val, 5);
    } else {
        //0��Ԃ�(�ߋ�5�񕪂̌v���f�[�^�������Ė���)
        ac[1] = 0;
    }
    // Z
    if (get_series_buf(&acel_z, val, 5) == 5) {
        //�ߋ�5��̌v���̒����l��Ԃ�
        ac[2] = get_median(val, 5);
    } else {
        //0��Ԃ�(�ߋ�5�񕪂̌v���f�[�^�������Ė���)
        ac[2] = 0;
    }
    
    /* �n���C */
    // X
    if (get_series_buf(&magm_x, val, 5) == 5) {
        //�ߋ�5��̌v���̒����l��Ԃ�
        mm[0] = get_median(val, 5);
    } else {
        //0��Ԃ�(�ߋ�5�񕪂̌v���f�[�^�������Ė���)
        mm[0] = 0;
    }
    // Y
    if (get_series_buf(&magm_y, val, 5) == 5) {
        //�ߋ�5��̌v���̒����l��Ԃ�
        mm[1] = get_median(val, 5);
    } else {
        //0��Ԃ�(�ߋ�5�񕪂̌v���f�[�^�������Ė���)
        mm[1] = 0;
    }
    // Z
    if (get_series_buf(&magm_z, val, 5) == 5) {
        //�ߋ�5��̌v���̒����l��Ԃ�
        mm[2] = get_median(val, 5);
    } else {
        //0��Ԃ�(�ߋ�5�񕪂̌v���f�[�^�������Ė���)
        mm[2] = 0;
    }

    sprintf(send_buf,
        "{\"cmd\":\"9000\",\"gyro\":[%d,%d,%d],\"accel\":[%d,%d,%d],\"magnetometer\":[%d,%d,%d]}\r\n", 
        (int16_t)gy[0], (int16_t)gy[1], (int16_t)gy[2],
        (int16_t)ac[0], (int16_t)ac[1], (int16_t)ac[2],
        (int16_t)mm[0], (int16_t)mm[1], (int16_t)mm[2]
    );
    TZ01_console_puts(send_buf);
}

/* �C���Z���T */
void IO_respons_airpressure(void)
{
    uint32_t val[5];
    uint32_t median;
    
    if (get_series_buf(&airp, val, 5) == 5) {
        //�ߋ�5��̌v���̒����l��Ԃ�
        median = get_median(val, 5);
    } else {
        //0��Ԃ�(�ߋ�5�񕪂̌v���f�[�^�������Ė���)
        median = 0;
    }
    sprintf(send_buf, "{\"cmd\":\"p000\",\"airpressure\":%d}\r\n", median);
    TZ01_console_puts(send_buf);
}

/* �o�b�e���[�[�dIC���W�X�^�ǂݏo�����ʏo�� */
void IO_respons_charger_status(void)
{
    sprintf(
        send_buf, "{\"cmd\":\"c000\",\"reg\":[%d,%d,%d,%d,%d,%d,%d]}\r\n",
        charger_reg[0], charger_reg[1], charger_reg[2], 
        charger_reg[3], charger_reg[4], charger_reg[5], charger_reg[6]
    );
    TZ01_console_puts(send_buf);
}

/* I/O�e�X�g�̒������(�A�i���O�n�̒�����W) */
void IO_periodic_run(void)
{
    MPU9250_gyro_val gyro;
    MPU9250_accel_val acel;
    MPU9250_magnetometer_val magm;
    uint16_t val16;
    uint32_t val32;
    
    /* Gyro */
    if (MPU9250_drv_read_gyro(&gyro)) {
        put_dat_buf(&gyro_x, gyro.raw_x);
        put_dat_buf(&gyro_y, gyro.raw_y);
        put_dat_buf(&gyro_z, gyro.raw_z);
    }
    
    /* Accel */
    if (MPU9250_drv_read_accel(&acel)) {
        put_dat_buf(&acel_x, acel.raw_x);
        put_dat_buf(&acel_y, acel.raw_y);
        put_dat_buf(&acel_z, acel.raw_z);
    }
    
    /* Magnetometer */
    if (MPU9250_drv_read_magnetometer(&magm)) {
        put_dat_buf(&magm_x, magm.raw_x);
        put_dat_buf(&magm_y, magm.raw_y);
        put_dat_buf(&magm_z, magm.raw_z);
    }
    
    /* Airpressure */
    put_dat_buf(&airp, BMP280_drv_press_get());
    
    /* BatteryCharger */
    charger_reg[0] = BQ24250_drv_reg01_get();
    charger_reg[1] = BQ24250_drv_reg02_get();
    charger_reg[2] = BQ24250_drv_reg03_get();
    charger_reg[3] = BQ24250_drv_reg04_get();
    charger_reg[4] = BQ24250_drv_reg05_get();
    charger_reg[5] = BQ24250_drv_reg06_get();
    charger_reg[6] = BQ24250_drv_reg07_get();
    
    /* ADC12 */
    //Ch0
    Driver_ADCC12.ReadData(ADCC12_CHANNEL_0, &val16);
    put_dat_buf(&adc12_0, val16);
    //Ch1
    Driver_ADCC12.ReadData(ADCC12_CHANNEL_1, &val16);
    put_dat_buf(&adc12_1, val16);
    //Ch2
    Driver_ADCC12.ReadData(ADCC12_CHANNEL_2, &val16);
    put_dat_buf(&adc12_2, val16);
    //Ch3
    Driver_ADCC12.ReadData(ADCC12_CHANNEL_3, &val16);
    put_dat_buf(&adc12_3, val16);
    /* ADCC24 */
    Driver_ADCC24.ReadData(ADCC24_CHANNEL_2, &val32);
    put_dat_buf(&adc24_2, val32);
}

