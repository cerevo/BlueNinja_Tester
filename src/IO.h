#ifndef _IO_H_
#define _IO_H_
void IO_init(void);
void IO_uart1_echo(void);
void IO_i2c_pingpong(void);
void IO_respons_gpio(void);
void IO_respons_adc(uint8_t param);
void IO_respons_9axis(void);
void IO_respons_airpressure(void);
void IO_respons_charger_status(void);
void IO_periodic_run(void);
#endif
