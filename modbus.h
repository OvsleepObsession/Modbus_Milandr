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

uint16_t get_crc(uint8_t *PtrBuffer, uint8_t SizeBuffer); // ���������� CRC
BOOL check_crc_in(int i); // �������� CRC
void fill_crc_out(void); // ������� CRC � ����� ��������� ���������

/* ----------------------- DATA FILL AND REMOVAL --------------------------------*/

void storage_fill(void); // �������� ���������� input registers
void input_reg_fill(uint16_t *ptr, int size); // ��������� ������ input registers
void hold_reg_fill(uint16_t *ptr, int size); // ��������� ������ holding registers
void d_inputs_fill(BOOL *ptr, int size); // ��������� ������ descrete inputs
void coils_fill(BOOL *ptr, int size); // ��������� ������ coils
void clear_arr(void); // ������ �������� �������

/* ----------------------- DATA PROCESS --------------------------------*/

void data_transmit(void); // �������� ������ ����� UART
BOOL scan_data(void); // ��������� ���������� ������� � ������������ ��������� ���������
BOOL data_receive(void); // ��������� ������� ����� UART

/* ------------------- CONNNECTION SETTINGS --------------------------- */

BOOL establish_settings(uint32_t baud, uint16_t wordlength, uint16_t stopbits, uint16_t parity);

/* ---------------------- INTERRUPTION -------------------------------- */

void UART2_IRQHandler(void);

/* ---------------------- USER FUNCTIONS -------------------------------- */

void start_modbus(BOOL mode); // 0 for Slave and 1 for Master





