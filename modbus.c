#include "modbus.h"

/* ----------------------- GLOBAL ---------------------------------*/

uint8_t arr_in[256]; // âõîäíîé ìàññèâ, ñþäà ïðèõîäÿò çàïðîñû
uint8_t addr = 0x01; // àäðåñ âåäîìîãî
uint16_t input_reg[1000]; // input registers
uint16_t hold_reg[1000]; // holding registers
BOOL coils[1000]; // coils
BOOL d_inputs[1000]; // descrete imputs
uint8_t arr_out[256]; // âûõîäíîé ìàññèâ, èç êîòðîãî îòïðàâëÿþòñÿ äàííûå
uint8_t t; // ñ÷åò÷èê-èíäåêñ äëÿ âûõîäíîãî ìàññèâà
BOOL flag = FALSE;
BOOL crc_check = FALSE;

BOOL coils[1000] = {1,0,0,0,1,0,1,1,1}; // coils test

/* ----------------------- CRC16 --------------------------------*/

const uint16_t table_crc[] = { // òàáëèöà CRC
0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

uint16_t get_crc(uint8_t *PtrBuffer, uint8_t SizeBuffer) { // âû÷èñëåíèå CRC
	uint16_t Crc;
	Crc = 0xffff;
	while (SizeBuffer--)
		Crc = (Crc >> 8) ^ table_crc[(Crc & 0x00ff) ^ *PtrBuffer++];
	return (Crc);
}

BOOL check_crc_in(int i) // ñðàâíåíèå âû÷èñëåííîãî CRC ñ òåì, ÷òî ñîäåðæèòñÿ â çàïðîñå
{
	uint16_t shift = (uint16_t)arr_in[i-1] | (((uint16_t)arr_in[i]) << 8); 
	uint16_t test = get_crc(arr_in, i-1); 
	if (test == shift) return TRUE;
	else return FALSE;
}

void fill_crc_out()
{
	const uint16_t crc_out = get_crc(arr_out, t);
	arr_out[t++] = crc_out & 0x00ff;
	arr_out[t++] = crc_out >> 8;
}
/* ----------------------- DATA FILL AND REMOVAL --------------------------------*/
void storage_fill() // holding registers test
{
	for (int i = 0; i<100; i++)
	hold_reg[i] = i;
}

void input_reg_fill(uint16_t *ptr, int size)
{
	for (int i = 0; i<size; i++)
		input_reg[i] = *(ptr + i); 
}

void hold_reg_fill(uint16_t *ptr, int size)
{
	for (int i = 0; i<size; i++)
		hold_reg[i] = *(ptr + i); 
}

void d_inputs_fill(BOOL *ptr, int size)
{
	for (int i = 0; i<size; i++)
		d_inputs[i] = *(ptr + i); 
}

void coils_fill(BOOL *ptr, int size)
{
	for (int i = 0; i<size; i++)
		coils[i] = *(ptr + i); 
}

void clear_arr()
{
	for (int i = 0; i<256; i++)
	{
	arr_in[i] = 0x00;
	arr_out[i] = 0x00;
	}
}

/* ----------------------- DATA PROCESS --------------------------------*/
void data_transmit(){
	for (int i = 0; i<t; i++){
	UART_SendData(MDR_UART2, arr_out[i]);
	while(UART_GetFlagStatus(MDR_UART2, UART_FLAG_TXFE)!=SET);
	}
}

BOOL scan_data()
{
	t = 0;
	arr_out[t++] = addr; // áàéò àäðåñà óñòðîéñòâà
	arr_out[t++] = arr_in[1]; // êîä ôóíêöèè
	switch(arr_in[1])
	{
		case 0x01: // ÷òåíèå coils
		{
			uint16_t startaddr = (uint16_t)arr_in[3] | (((uint16_t)arr_in[2]) << 8);
			uint16_t count = (uint16_t)arr_in[5] | (((uint16_t)arr_in[4]) << 8);
			if (count%8==0) // ñëó÷àé, êîãäà êîëè÷åñòâî coils êðàòíî 8
			{
				arr_out[t++] = (uint8_t)((int)count/8);
				int cnt = 0; // ñ÷åò÷èê çàïèñè áèòîâ â áàéò
				for(int i = startaddr; i<(startaddr + count); i++)
				{
					if (coils[i]!=0)
					{
						arr_out[t] = 128 | arr_out[t]; // ïîáèòîâîå ñëîæåíèå ñ 10000000 
						++cnt;
					}
					else
					{
						arr_out[t] = 127 & arr_out[t]; // ïîáèòîâîå óìíîæåíèå ñ 01111111
						++cnt;
					}
					if (cnt<8)
						arr_out[t] = arr_out[t] >> 1; // îäíîáèòîâûé ñäâèã âïðàâî
					if (cnt == 8) // åñëè çàïèñàëè â òåêóùèé áàéò âñå 8 áèò, òî íà÷èíàåì çàïèñü â ñëåäóþùèé áàéò
					{
						cnt = 0;
						++t;
					}
				}
			}
			else // ñëó÷àé, êîãäà êîëè÷åñòâî coils íå êðàòíî 8
			{
				arr_out[t++] = (uint8_t)((int)count/8 + 1); 
				int cnt = 0;
				int shift = 7 - count%8; // ñäâèã äëÿ ïîñëåäíåãî áàéòà 
				for(int i = startaddr; i<(startaddr + count); i++)
				{
					if (coils[i]!=0)
					{
						arr_out[t] = 128 | arr_out[t];
						++cnt;
					}
					else
					{
						arr_out[t] = 127 & arr_out[t];
						++cnt;
					}
					if (cnt<8)
						arr_out[t] = arr_out[t] >> 1; // îäíîáèòîâûé ñäâèã âïðàâî
					if (cnt == 8)
					{
						cnt = 0;
						++t;
					}
					if (i == startaddr + count - 1) // åñëè ñ÷èòûâàåì ïîñëåäíèé coil
					{
						arr_out[t] = arr_out[t] >> shift; // ñäâèãàåì âïðàâî íà íåäîñòàþùåå ÷èñëî áèòîâ â ïîñëåäíåì áàéòå
						++t;
					}
				}
			}
			fill_crc_out(); // âû÷èñëåíèå è âñòàâêà CRC â êîíåö îòâåòíîãî ñîîáùåíèÿ
			data_transmit(); // îòïðàâêà ñîîáùåíèÿ
			clear_arr(); // ÷èñòêà âõîäíîãî è âûõîäíîãî ìàññèâà
			flag = FALSE; // ñáðîñ ôëàãà
			return TRUE;
		}
		case 0x02: // ÷òåíèå discrete imputs
		{
			uint16_t startaddr = (uint16_t)arr_in[3] | (((uint16_t)arr_in[2]) << 8);
			uint16_t count = (uint16_t)arr_in[5] | (((uint16_t)arr_in[4]) << 8);
			if (count%8==0) // ñëó÷àé, êîãäà êîëè÷åñòâî discrete inputs êðàòíî 8
			{
				arr_out[t++] = (uint8_t)((int)count/8);
				int cnt = 0; // ñ÷åò÷èê çàïèñè áèòîâ â áàéò
				for(int i = startaddr; i<(startaddr + count); i++)
				{
					if (d_inputs[i]!=0)
					{
						arr_out[t] = 128 | arr_out[t]; // ïîáèòîâîå ñëîæåíèå ñ 10000000 
						++cnt;
					}
					else
					{
						arr_out[t] = 127 & arr_out[t]; // ïîáèòîâîå óìíîæåíèå ñ 01111111
						++cnt;
					}
					if (cnt<8)
						arr_out[t] = arr_out[t] >> 1; // îäíîáèòîâûé ñäâèã âïðàâî
					if (cnt == 8) // åñëè çàïèñàëè â òåêóùèé áàéò âñå 8 áèò, òî íà÷èíàåì çàïèñü â ñëåäóþùèé áàéò
					{
						cnt = 0;
						++t;
					}
				}
			}
			else // ñëó÷àé, êîãäà êîëè÷åñòâî discrete imputs íå êðàòíî 8
			{
				arr_out[t++] = (uint8_t)((int)count/8 + 1); 
				int cnt = 0;
				int shift = 7 - count%8; // ñäâèã äëÿ ïîñëåäíåãî áàéòà 
				for(int i = startaddr; i<(startaddr + count); i++)
				{
					if (d_inputs[i]!=0)
					{
						arr_out[t] = 128 | arr_out[t];
						++cnt;
					}
					else
					{
						arr_out[t] = 127 & arr_out[t];
						++cnt;
					}
					if (cnt<8)
						arr_out[t] = arr_out[t] >> 1; // îäíîáèòîâûé ñäâèã âïðàâî
					if (cnt == 8)
					{
						cnt = 0;
						++t;
					}
					if (i == startaddr + count - 1) // åñëè ñ÷èòûâàåì ïîñëåäíèé discrete input
					{
						arr_out[t] = arr_out[t] >> shift; // ñäâèãàåì âïðàâî íà íåäîñòàþùåå ÷èñëî áèòîâ â ïîñëåäíåì áàéòå
						++t;
					}
				}
			}
			fill_crc_out();
			data_transmit();
			clear_arr();
			flag = FALSE;
			return TRUE;
		}	
		case 0x03: // ÷òåíèå holding registers
		{
			uint16_t startaddr = (uint16_t)arr_in[3] | (((uint16_t)arr_in[2]) << 8); // àäðåñ ðåãèñòðà
			uint16_t count = (uint16_t)arr_in[5] | (((uint16_t)arr_in[4]) << 8); // êîëè÷åñòâî äàííûõ
			arr_out[t++] = 2*(uint8_t)count; // êîëè÷åñòâî áàéò äàííûõ â îòâåòíîì ñîîáùåíèè
			for(int i = startaddr; i<(startaddr + count); i++)
			{
				arr_out[t++] = hold_reg[i] >> 8; // ñòàðøèé áàéò çíà÷åíèÿ
				arr_out[t++] = hold_reg[i] & 0x00ff; // ìëàäøèé áàéò çíà÷åíèÿ
			}
			fill_crc_out();
			data_transmit();
			clear_arr();
			flag = FALSE;
			return TRUE;
		}
		case 0x04: // ÷òåíèå imput registers
		{
			uint16_t startaddr = (uint16_t)arr_in[3] | (((uint16_t)arr_in[2]) << 8);
			uint16_t count = (uint16_t)arr_in[5] | (((uint16_t)arr_in[4]) << 8);
			arr_out[t++] = 2*(uint8_t)count;
			for(int i = startaddr; i<(startaddr + count); i++)
			{
				arr_out[t++] = input_reg[i] >> 8;
				arr_out[t++] = input_reg[i] & 0x00ff;
			}
			fill_crc_out();
			data_transmit();
			clear_arr();
			flag = FALSE;
			return TRUE;
		}
		case 0x05: // çàïèñü îäíîãî çíà÷åíèÿ coil
		{
			uint16_t coil_addr = (uint16_t)arr_in[3] | (((uint16_t)arr_in[2]) << 8);
			uint16_t coil_val = (uint16_t)arr_in[5] | (((uint16_t)arr_in[4]) << 8);
			if (coil_val!=0xFF00 && coil_val!=0x0000)
				return FALSE;
			if (coil_val==0xFF00)
				coils[coil_addr]= TRUE;
			if (coil_val==0x0000)
				coils[coil_addr]= FALSE;
			
			for(int i = 0; i < 8; i++)
				arr_out[i] = arr_in[i]; // îòâåòíîå ñîîáùåíèå - êîïèÿ çàïðîñà
			t = 8;
			data_transmit();
			clear_arr();
			flag = FALSE;
			return TRUE;
		}
		case 0x06: // çàïèñü îäíîãî çíà÷åíèÿ holding register
		{
			uint16_t reg_addr = (uint16_t)arr_in[3] | (((uint16_t)arr_in[2]) << 8);
			uint16_t reg_val = (uint16_t)arr_in[5] | (((uint16_t)arr_in[4]) << 8);
			hold_reg[reg_addr] = reg_val;
			for(int i = 0; i < 8; i++)
				arr_out[i] = arr_in[i];
			t = 8;
			data_transmit();
			clear_arr();
			flag = FALSE;
			return TRUE;
		}
		case 0x0F: // çàïèñü íåñêîëüêèõ çíà÷åíèé coils
		{
			uint16_t coil_addr = (uint16_t)arr_in[3] | (((uint16_t)arr_in[2]) << 8);
			uint16_t count = (uint16_t)arr_in[5] | (((uint16_t)arr_in[4]) << 8);
			int bytes = arr_in[6];
			uint8_t temp[bytes];
			for (int i=0; i<bytes; i++)
				temp[i] = arr_in[7+i];
			int c = 0;
			int cnt = 0;
			for(int i = coil_addr; i<(coil_addr + count); i++)
			{
				if((temp[c] & 1) == 0)
				{
					coils[i] = FALSE;
					++cnt;
				}
				if((temp[c] & 1) == 1)
				{
					coils[i] = TRUE;
					++cnt;
				}
				if (cnt < 8)
					temp[c] = temp[c] >> 1;
				if (cnt == 8)
				{
					++c;
					cnt = 0;
				}
			}
			for(int i = 0; i < 6; i++)
				arr_out[i] = arr_in[i];
			t = 6; 
			fill_crc_out();
			data_transmit();
			clear_arr();
			flag = FALSE;
			return TRUE;
		}
		case 0x10: // çàïèñü íåñêîëüêèõ çíà÷åíèé holding registers
		{
			uint16_t reg_addr = (uint16_t)arr_in[3] | (((uint16_t)arr_in[2]) << 8);
			uint16_t count = (uint16_t)arr_in[5] | (((uint16_t)arr_in[4]) << 8);
			int c = 8;
			for(int i = reg_addr; i<(reg_addr + count); i++)
			{
				hold_reg[i] = (uint16_t)arr_in[c] | (((uint16_t)arr_in[c-1]) << 8);
				c=c+2;
			}
			for(int i = 0; i < 6; i++)
				arr_out[i] = arr_in[i];
			t = 6; 
			fill_crc_out();
			data_transmit();
			clear_arr();
			flag = FALSE;
			return TRUE;
		}
	}
	return FALSE;
}

BOOL data_receive(void)
{
	arr_in[1] = (uint8_t)UART_ReceiveData(MDR_UART2); // Êîä ôóíêöèè
	if ((arr_in[1]!= 0x0F) && (arr_in[1]!= 0x10)) 
	{
	arr_in[2] = (uint8_t)UART_ReceiveData(MDR_UART2); // ñò. áàéò àäðåñà
	arr_in[3] = (uint8_t)UART_ReceiveData(MDR_UART2); // ìë. áàéò àäðåñà
	arr_in[4] = (uint8_t)UART_ReceiveData(MDR_UART2); // ñò. áàéò êîë-âà äàííûõ
	arr_in[5] = (uint8_t)UART_ReceiveData(MDR_UART2); // ìë. áàéò êîë-âà äàííûõ
	arr_in[6] = (uint8_t)UART_ReceiveData(MDR_UART2); // CRC
	arr_in[7] = (uint8_t)UART_ReceiveData(MDR_UART2); // CRC
	return check_crc_in(7);
	}
	else // åñëè çàïðîñ äëèííåå 8 áàéò
	{
		arr_in[2] = (uint8_t)UART_ReceiveData(MDR_UART2);
		arr_in[3] = (uint8_t)UART_ReceiveData(MDR_UART2);
		arr_in[4] = (uint8_t)UART_ReceiveData(MDR_UART2);
		arr_in[5] = (uint8_t)UART_ReceiveData(MDR_UART2);
		arr_in[6] = (uint8_t)UART_ReceiveData(MDR_UART2);
		int i = 7;
		for (uint8_t j = 0; j < arr_in[6] + 2; j++)
		{
			while (UART_GetFlagStatus (MDR_UART2, UART_FLAG_RXFE) == SET);
			arr_in[i++] = (uint8_t)UART_ReceiveData(MDR_UART2);
		}
		//while(UART_GetFlagStatus(MDR_UART2, UART_FLAG_RXFE)!=SET);
		return check_crc_in(i-1);
	}
}

/* ------------------- CONNNECTION SETTINGS --------------------------- */
BOOL establish_settings(uint32_t baud, uint16_t wordlength, uint16_t stopbits, uint16_t parity)
{
        // Âêëþ÷åíèå òàêòèðîâàíèÿ ïîðòà F
        RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTF, ENABLE); 
        // Îáúÿâëåíèå ñòðóêòóðû äëÿ èíèöèàëèçàöèè ïîðòà
        PORT_InitTypeDef uart2_port_set;  
        // Èíèöèàëèçàöèÿ ïîðòà F äëÿ ôóíêöèè UART
        // Íàñòðîéêà ïîðòà ïî óìîë÷àíèþ
        PORT_StructInit(&uart2_port_set);  
        // Ïåðåîïðåäåëåíèå ôóíêöèè ïîðòà
        uart2_port_set.PORT_FUNC = PORT_FUNC_OVERRID;  
        // Óñòàíîâêà êîðîòêîãî ôðîíòà
        uart2_port_set.PORT_SPEED = PORT_SPEED_MAXFAST;  
        // Öèôðîâîé ðåæèì ðàáîòû âûâîäà
        uart2_port_set.PORT_MODE = PORT_MODE_DIGITAL;  
        // Èíèöèàëèçàöèÿ âûâîäà PF1 êàê UART_TX (ïåðåäà÷à)
        uart2_port_set.PORT_Pin = PORT_Pin_1;
        uart2_port_set.PORT_OE = PORT_OE_OUT;
        PORT_Init(MDR_PORTF, &uart2_port_set);
        // Èíèöèàëèçàöèÿ âûâîäà PF0 êàê UART_RX (ïðèåì)
        uart2_port_set.PORT_Pin = PORT_Pin_0;
        uart2_port_set.PORT_OE = PORT_OE_IN;
				PORT_Init(MDR_PORTF, &uart2_port_set);
				// Óêàçàíèå òèïà ñòðóêòóðû è èìåíè ñòðóêòóðû 
				PORT_InitTypeDef Nastroyka_D; 
				// Ðàáîòà â àëüòåðíàòèâíîì ðåæèìå ïîðòà EXT_INT2 
				// Öèôðîâîé ðåæèì 
				Nastroyka_D.PORT_MODE = PORT_MODE_ANALOG; 
				// Íèçêàÿ ñêîðîñòü ïåðåêëþ÷åíèÿ (ïîëîãèé ôðîíò) 
				Nastroyka_D.PORT_SPEED = PORT_SPEED_SLOW; 
				// Êîíôèãóðàöèÿ ëèíèè ïîðòà êàê âõîäà 
				Nastroyka_D.PORT_OE = PORT_OE_IN; 
				// Îáúÿâëåíèå íîìåðà ëèíèè ïîðòà, êîòîðàÿ 
				// íàñòðàèâàåòñÿ äàííîé ñòðóêòóðîé 
				Nastroyka_D.PORT_Pin = PORT_Pin_2; 
				//Èíèöèàëèçàöèÿ ïîðòà C îáúÿâëåííîé ñòðóêòóðîé 
				PORT_Init(MDR_PORTD, &Nastroyka_D);
        // Ïðîöåäóðà èíèöèàëèçàöèè êîíòðîëëåðà UART
        // Âêëþ÷åíèå òàêòèðîâàíèÿ UART2
        RST_CLK_PCLKcmd(RST_CLK_PCLK_UART2, ENABLE);
        // Îáúÿâëåíèå ñòðóêòóðû äëÿ èíèöèàëèçàöèè êîíòðîëëåðà UART
        UART_InitTypeDef UART_InitStructure;
        // Äåëèòåëü òàêòîâîé ÷àñòîòû UART = 1
        UART_BRGInit(MDR_UART2,UART_HCLKdiv1);
        // Êîíôèãóðàöèÿ UART
        // Ñêîðîñòü ïåðåäà÷è äàííûõ
        UART_InitStructure.UART_BaudRate = baud;
        // Êîëè÷åñòâî áèò â ïîñûëêå 
        UART_InitStructure.UART_WordLength = wordlength;
        // ñòîï-áèò
        UART_InitStructure.UART_StopBits = stopbits;
        // Ïðîâåðêà ÷åòíîñòè
        UART_InitStructure.UART_Parity = parity;
        // Âêëþ÷èòü ðàáîòó áóôåðà FIFO ïðèåìíèêà è ïåðåäàò÷èêà,
        UART_InitStructure.UART_FIFOMode = UART_FIFO_ON;
        // Ðàçðåøèòü ïðèåì è ïåðåäà÷ó äàííûõ
        UART_InitStructure.UART_HardwareFlowControl = UART_HardwareFlowControl_RXE | UART_HardwareFlowControl_TXE;
        // Èíèöèàëèçàöèÿ UART2 ñ çàäàííûìè ïàðàìåòðàìè
        UART_Init(MDR_UART2, &UART_InitStructure);
        // Âêëþ÷èòü ñêîíôèãóðèðîâàííûé UART
				__enable_irq();
				UART_ITConfig(MDR_UART2, UART_IT_RX, ENABLE);
			  NVIC_EnableIRQ(UART2_IRQn);
        UART_Cmd(MDR_UART2, ENABLE);
				return TRUE;
}

/* ---------------------- INTERRUPTION -------------------------------- */
void UART2_IRQHandler(void)
{
	arr_in[0] = (uint8_t)UART_ReceiveData(MDR_UART2); 
	if (arr_in[0] == addr){
		flag = data_receive();
	}
}

/* ---------------------- USER FUNCTIONS -------------------------------- */
void start_modbus(BOOL mode)
{
	if (mode==0)
		while(1)
		{
			if (flag) scan_data();
		}
	if (mode==1)
		return;
}

// ------------------------------ ACTIVATION ----------------------------------

/*
int main (void) 
{
	establish_settings(19200, UART_WordLength8b, UART_StopBits1, UART_Parity_No);
	storage_fill();
	start_modbus(SLAVE);
}
*/
