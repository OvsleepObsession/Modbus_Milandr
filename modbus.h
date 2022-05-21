#include <MDR32F9Qx_port.h>
#include <MDR32F9Qx_rst_clk.h>
#include <MDR32F9Qx_timer.h>
#include <MDR32F9Qx_uart.h>
#include <MDR32Fx.h>


/* ----------------------- DEFINES --------------------------------*/

typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef uint8_t BOOL;
#ifndef TRUE
#define TRUE            1
#endif

#ifndef FALSE
#define FALSE           0
#endif

#ifndef SLAVE
#define SLAVE           0
#endif

#ifndef MASTER
#define MASTER          1
#endif

/* ----------------------- CRC16 --------------------------------*/

uint16_t get_crc(uint8_t *PtrBuffer, uint8_t SizeBuffer); // вычисление CRC
BOOL check_crc_in(int i); // проверка CRC
void fill_crc_out(void); // вставка CRC в конец ответного сообщения

/* ----------------------- DATA FILL AND REMOVAL --------------------------------*/

void storage_fill(void); // Тестовое заполнение input registers
void input_reg_fill(uint16_t *ptr, int size); // установка данных input registers
void hold_reg_fill(uint16_t *ptr, int size); // установка данных holding registers
void d_inputs_fill(BOOL *ptr, int size); // установка данных descrete inputs
void coils_fill(BOOL *ptr, int size); // установка данных coils
void clear_arr(void); // Чистка входного массива

/* ----------------------- DATA PROCESS --------------------------------*/

void data_transmit(void); // отправка данных через UART
BOOL scan_data(void); // обработка пришедшего запроса и формирование ответного сообщения
BOOL data_receive(void); // получение запроса через UART

/* ------------------- CONNNECTION SETTINGS --------------------------- */

BOOL establish_settings(uint32_t baud, uint16_t wordlength, uint16_t stopbits, uint16_t parity);

/* ---------------------- INTERRUPTION -------------------------------- */

void UART2_IRQHandler(void);

/* ---------------------- USER FUNCTIONS -------------------------------- */

void start_modbus(BOOL mode); // 0 for Slave and 1 for Master





