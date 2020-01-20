/*
 * CAN.c
 *
 * Created: 16.03.2019 6:22:01
 *  Author: Michael
 */ 

#include "CAN.h"

		/* ************  SPCR  ************ //
	 SPIE 1  // –азрешение прерываний по SPI
	 SPE  1  // ¬ключение SPI
	 DORD 1  // ѕор€док передачи данных 0 - сначала будет передаватьс€ младший разр€д
			 //						   1 - сначала будет передаватьс€ старший разр€д
	 MSTR 1  // 1 Ц режим ведущего
			 // 0 Ц режим ведомого
	 CPOL 1  //		CPOL = 0 Ч сигнал синхронизации начинаетс€ с низкого уровн€;
			 //		CPOL = 1 Ч сигнал синхронизации начинаетс€ с высокого уровн€;
	 CPHA 1	 //		CPHA = 0 Ч выборка данных производитс€ по переднему фронту сигнала синхронизации;
			 //		CPHA = 1 Ч выборка данных производитс€ по заднему фронту сигнала синхронизации.
	 SPR1 1  //  
	 SPR0 1  //		SPI2X, SPR1, SPR0 - —корость передачи данных

		// ************  SPSR  ************ //
	 SPI2X 1  //
	 SPIF  0  //  ‘лаг завершени€ передачи - устанавливаетс€ после завершени€ передачи (приема)  и вызывает прерывание
					(как у ведущего, так и у ведомого). ¬ вызванном прерывании можно считывать и записывать новые данные
					дл€ передачи. —брасываетс€ автоматически при входе в подпрограмму обработки прерывани€.                          
	 WCOL  1  //  ‘лаг наложени€ записи Ц устанавливаетс€ при чтении либо записи данных при незавершенной передаче. 
					јвтоматически сбрасываетс€ чтением регистра SPSR, а также при чтении (записи) в регистр данных (SPDR). 


			// CAN Bitrate 125 kbps
			#define R_CNF1  (1<<BRP2)|(1<<BRP1)|(1<<BRP0)
			#define R_CNF2  (1<<BTLMODE)|(1<<PHSEG11)
			#define R_CNF3  (1<<PHSEG21)

			// CAN Bitrate 250 kbps
			#define R_CNF1  (1<<BRP1)|(1<<BRP0)
			#define R_CNF2  (1<<BTLMODE)|(1<<PHSEG11)
			#define R_CNF3  (1<<PHSEG21)

			// CAN Bitrate 500 kbps
			#define R_CNF1  (1<<BRP0)
			#define R_CNF2  (1<<BTLMODE)|(1<<PHSEG11)
			#define R_CNF3  (1<<PHSEG21)

			// CAN Bitrate 1 Mbps
			#define R_CNF1  0
			#define R_CNF2  (1<<BTLMODE)|(1<<PHSEG11)
			#define R_CNF3  (1<<PHSEG21) */


uint8_t mcp_Rx0DATA[8];



bool mcp_init(void)
{
	MCP_CS_OFF;
	SET_OUTPUT(P_CS);
	
	SET_0(P_SCK);
	SET_0(P_MOSI);
	SET_0(P_MISO);
	
	SET_OUTPUT(P_SCK);
	SET_OUTPUT(P_MOSI);
	SET_INPUT(P_MISO);
	
	SET_INPUT(MCP_INT);
	SET_1(MCP_INT);
	
	// active SPI master interface
	SPCR = ((1<<SPE)|(1<<MSTR)|(1<<SPIE))&(~(1<<SPR0))&(~(1<<SPR1));
	SPSR = 0;
	
	// reset MCP2515 by software reset.
	// After this he is in configuration mode.
	MCP_CS_ON;
	spi_write_read(CAN_RESET);
	MCP_CS_OFF;
	
	// wait a little bit until the MCP2515 has restarted
	_delay_us(10);
	
	// load CNF1..3 Register
	MCP_CS_ON;
	spi_write_read(CAN_WRITE);
	spi_write_read(CNF3);
	
	spi_write_read(PHSEG2_3TQ);							//PS2 = (PHSEG2 + 1) = 3 TQ	
	spi_write_read(BTLMODE_CNF3|PHSEG1_3TQ);			//PS1 = (PHSEG1 + 1) = 3 TQ  /  PropSeg = (PRSEG + 1) = 1 TQ
	spi_write_read(BRP0);								//SWJ(0,0) = 1*TQ = 1 TQ     /  TQ = 2 x (BRP + 1)/FOS = 250ns
														//CAN speed = 1/(TQ*(SWJ+PS1+PS2+PropSeg)) = 500 kb/s
	// activate interrupts
	spi_write_read(RX1IE_ENABLED|RX0IE_ENABLED);
	MCP_CS_OFF;
	
	// test if we could read back the value => is the chip accessible?
	if (mcp_read(CNF1) != BRP0) {
		return 0;
	}
	
	// deactivate the RXnBF Pins (High Impedance State)
	mcp_write(BFPCTRL, 0);
	
	// set TXnRTS as inputs
	mcp_write(TXRTSCTRL, 0);
	
	// turn off filters => receive any message
	mcp_write(RXB0CTRL, RXM0|RXM1);
	mcp_write(RXB1CTRL, RXM0|RXM1);
	
	// reset device to normal mode
	mcp_write(CANCTRL, 0);
	
	return 1;
}

uint8_t spi_write_read(uint8_t SPIdata)
{
	SPDR = SPIdata;
	while(!(SPSR & (1<<SPIF))); // ќжидаем получени€ данных
	return SPDR;
}

void mcp_write(uint8_t RegName, uint8_t Data)
{

	MCP_CS_ON; // ¬ыбрали микросхему сигналом CS
	_delay_us(2);
	spi_write_read(CAN_WRITE); // WRITE = 0b00000010 = 0x02
	spi_write_read(RegName); // јдрес регистра
	spi_write_read(Data);
	MCP_CS_OFF;
	_delay_us(2);
}

uint8_t mcp_read(uint8_t RegName)
{
	MCP_CS_ON; // ¬ыбрали микросхему сигналом CS
	_delay_us(2);
	spi_write_read(CAN_READ); // READ = 0b00000011 = 0x03
	spi_write_read(RegName); // јдрес регистра
	uint8_t buff = spi_write_read(0x00);
	MCP_CS_OFF;
	_delay_us(2);
	return buff;
}

void mcp_bit_modify(uint8_t adress, uint8_t mask, uint8_t data)
{
	MCP_CS_ON;
	_delay_us(2);
	spi_write_read(CAN_BIT_MODIFY);
	spi_write_read(adress);
	spi_write_read(mask);
	spi_write_read(data);
	_delay_us(2);
	MCP_CS_OFF;
}

uint8_t mcp_send_message(CANMessage* message){
    uint8_t status, address;
   
    // Status MCP2515
    MCP_CS_ON;
    spi_write_read(CAN_RD_STATUS);
    status = spi_write_read(0xff);
    spi_write_read(0xff);
    MCP_CS_OFF;
   
    /* Status byte:
     *
     * Bit  Function
     *  2   TXB0CNTRL.TXREQ
     *  4   TXB1CNTRL.TXREQ
     *  6   TXB2CNTRL.TXREQ
     */
   
    if (bit_is_clear(status, 2)) {
        address = 0x00;
    }
    else if (bit_is_clear(status, 4)) {
        address = 0x02;
    }
    else if (bit_is_clear(status, 6)) {
        address = 0x04;
    }
    else {
        return 0;
    }
   
    MCP_CS_ON;    // CS Low
    spi_write_read(CAN_LD_TX_BUFF | address);
   
    // Standard ID
    spi_write_read( message->id>>3);
    spi_write_read( message->id<<5);
   
    // Extended ID
    spi_write_read(0x00);
    spi_write_read(0x00);
   
    uint8_t length = message->length;
   
    if (length > 8) {
        length = 8;
    }
   
    if (message->rtr)
    {
        spi_write_read((1<<6) | length);
    }
    else
    {
        spi_write_read(length);
        for (uint8_t i=0;i<length;i++) {
            spi_write_read(message->data[i]);
        }
    }
    MCP_CS_OFF; 
   
    asm volatile ("nop");
    MCP_CS_ON;   
    if (address == 0x00) {
        spi_write_read(CAN_RTS | 0x01);
    } else {
        spi_write_read(CAN_RTS | address);
    }
    MCP_CS_OFF; 
   
    return address;
} 

uint8_t mcp_get_message(CANMessage *message)
{
	// Status 
	uint8_t status = mcp_read_rx_status();

	if (bit_is_set(status,6))
	{
		// message in buffer 0
		MCP_CS_ON;    // CS Low
		spi_write_read(CAN_RD_RX_BUFF);
	}
	else if (bit_is_set(status,7))
	{
		// message in buffer 1
		MCP_CS_ON;    // CS Low
		spi_write_read(CAN_RD_RX_BUFF | 0x04);
	}
	else {
		// Error: no message available
		return 0;
	}
	
	// Standard ID 
	message->id =  (uint16_t) spi_write_read(0xff) << 3; 
	message->id |= (uint16_t) spi_write_read(0xff) >> 5;
	
	spi_write_read(0xff);
	spi_write_read(0xff);
	
	// 
	uint8_t length = spi_write_read(0xff) & 0x0f;
	message->length = length;
	
	// 
	for (uint8_t i=0;i<length;i++) {
		message->data[i] = spi_write_read(0xff);
	}
	
	MCP_CS_OFF;
	
	if (bit_is_set(status,3)) {
		message->rtr = 1;
		} else {
		message->rtr = 0;
	}
	
	// interrupt Flag 
	if (bit_is_set(status,6)) {
		mcp_bit_modify(CANINTF, (1<<RX0IF), 0);
		} else {
		mcp_bit_modify(CANINTF, (1<<RX1IF), 0);
	}
	
	return (status & 0x07); // 0b0000 0111
}

uint8_t mcp_read_rx_status()
{
	uint8_t data;
	MCP_CS_ON;
	
	spi_write_read(CAN_RX_STATUS);
	//getting output data
	data = spi_write_read(0xff);
	//sending byte for finishing command
	spi_write_read(0xff);
	MCP_CS_OFF;
	
	return data;
}