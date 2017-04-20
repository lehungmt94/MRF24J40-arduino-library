/*
 * Author: Le Hung
 * Email: lehungmt94@gmail.com
 * Website: mculearning.wordpress.com
 * 
 * Module: http://ww1.microchip.com/downloads/en/DeviceDoc/39776C.pdf
*/

#include "mrf24.h"




/**
 * Constructor MRF24 Object.
 * @param pin_reset, @param pin_chip_select, @param pin_interrupt
 */
MRF24::MRF24(int pin_reset, int pin_chip_select, int pin_interrupt) {
    _pin_reset = pin_reset;
    _pin_cs = pin_chip_select;
    _pin_int = pin_interrupt;

    pinMode(_pin_reset, OUTPUT);
    pinMode(_pin_cs, OUTPUT);
    pinMode(_pin_int, INPUT);

    SPI.setBitOrder(MSBFIRST) ;
    SPI.setDataMode(SPI_MODE0);
    SPI.begin();
}

void MRF24::begin(void) {
	byte i = 0;
	//variable initialization
	LQI = 0;
	RSSI2 = 0;
	SEQ_NUMBER = 0x23;
	lost_data = 0;
	address_RX_FIFO = 0x300;
	address_TX_normal_FIFO = 0;

	for (i = 0; i < 2; i++) {
		ADDRESS_short_1[i] = 1;
		ADDRESS_short_2[i] = 2;
		PAN_ID_1[i] = 3;
		PAN_ID_2[i] = 3;
	}

	for (i = 0; i < 8; i++) {
		ADDRESS_long_1[i] = 1;
		ADDRESS_long_2[i] = 2;
	}
	DATA_TX[0] = 0;        // Initialize first byte

	reset();	//reset module
	software_reset();    // Activate software reset
	RF_reset();	//Reset RF in  MRF24


	

	init_ZIGBEE_nonbeacon();                  // Initialize ZigBee module
	nonbeacon_PAN_coordinator_device();
	set_TX_power(31);                         // Default set max TX power 
	set_frame_format_filter(1);               // 1 all frames, 3 data frame only
	set_reception_mode(1);                    // 1 normal mode
}

void MRF24::reset(void) {
    digitalWrite(_pin_reset, LOW);
    delay(10);  // just my gut
    digitalWrite(_pin_reset, HIGH);
    delay(20);  // from manual
}



//ReadWrite_Routines.c

// read data from short address register
 byte MRF24::read_ZIGBEE_short(byte address) {
	byte data_r = 0, dummy_data_r = 0;

	digitalWrite(_pin_cs, LOW);

	address = (address << 1) & 0b01111110;      // calculating addressing mode
	SPI.transfer(address);                        // addressing register
	data_r = SPI.transfer(dummy_data_r);           // read data from register

	digitalWrite(_pin_cs, HIGH);
	return data_r;
}


// Read data from long address register
byte MRF24::read_ZIGBEE_long(word address) {
	byte data_r = 0, dummy_data_r = 0;
	byte address_high = 0, address_low = 0;

	digitalWrite(_pin_cs, LOW);

	address_high = ((short int)(address >> 3) & 0b01111111) | 0x80;  //calculating addressing mode
	address_low = ((short int)(address << 5) & 0b11100000);         //calculating addressing mode
	SPI.transfer(address_high);            // addressing register
	SPI.transfer(address_low);             // addressing register
	data_r = SPI.transfer(dummy_data_r);    // read data from register

	digitalWrite(_pin_cs, HIGH);
	return data_r;
}



void MRF24::write_ZIGBEE_short(byte address, byte data_r) {
	digitalWrite(_pin_cs, LOW);

	address = ((address << 1) & 0b01111111) | 0x01; // calculating addressing mode
	SPI.transfer(address);       // addressing register
	SPI.transfer(data_r);        // write data in register

	digitalWrite(_pin_cs, HIGH);
}

void MRF24::write_ZIGBEE_long(word address, byte data_r) {
	byte address_high = 0, address_low = 0;

	digitalWrite(_pin_cs, LOW);

	address_high = (((short int)(address >> 3)) & 0b01111111) | 0x80;  // calculating addressing mode
	address_low = (((short int)(address << 5)) & 0b11100000) | 0x10;  // calculating addressing mode
	SPI.transfer(address_high);           // addressing register
	SPI.transfer(address_low);            // addressing register
	SPI.transfer(data_r);                 // write data in registerr

	digitalWrite(_pin_cs, HIGH);
}

void MRF24::read_RX_FIFO() {
	byte temp = 0;
	int i = 0;

	temp = read_ZIGBEE_short(MRF_BBREG1);      // disable receiving packets off air.
	temp = temp | 0x04;                    // mask for disable receiving packets
	write_ZIGBEE_short(MRF_BBREG1, temp);

	for (i = 0; i<128; i++) {
		if (i <  (1 + DATA_LENGHT + HEADER_LENGHT + 2 + 1 + 1))
			data_RX_FIFO[i] = read_ZIGBEE_long(address_RX_FIFO + i);  // reading valid data from RX FIFO
		if (i >= (1 + DATA_LENGHT + HEADER_LENGHT + 2 + 1 + 1))
			lost_data = read_ZIGBEE_long(address_RX_FIFO + i);        // reading invalid data from RX FIFO
	}

	DATA_RX[0] = data_RX_FIFO[HEADER_LENGHT + 1];               // coping valid data
	DATA_RX[1] = data_RX_FIFO[HEADER_LENGHT + 2];               // coping valid data
	DATA_RX[2] = data_RX_FIFO[HEADER_LENGHT + 3];               // coping valid data
	LQI = data_RX_FIFO[1 + HEADER_LENGHT + DATA_LENGHT + 2];  // coping valid data
	RSSI2 = data_RX_FIFO[1 + HEADER_LENGHT + DATA_LENGHT + 3];  // coping valid data

	temp = read_ZIGBEE_short(MRF_BBREG1);      // enable receiving packets off air.
	temp = temp & (!0x04);                 // mask for enable receiving
	write_ZIGBEE_short(MRF_BBREG1, temp);
}

/*
* Transmit packet
*/
void MRF24::start_transmit() {
	byte temp = 0;

	temp = read_ZIGBEE_short(MRF_TXNCON);
	temp = temp | 0x01;                 // mask for start transmit
	write_ZIGBEE_short(MRF_TXNCON, temp);
}

void MRF24::write_TX_normal_FIFO() {
	int i = 0;

	data_TX_normal_FIFO[0] = HEADER_LENGHT;
	data_TX_normal_FIFO[1] = HEADER_LENGHT + DATA_LENGHT;
	data_TX_normal_FIFO[2] = 0x01;                        // control frame
	data_TX_normal_FIFO[3] = 0x88;
	data_TX_normal_FIFO[4] = SEQ_NUMBER;                  // sequence number
	data_TX_normal_FIFO[5] = PAN_ID_2[1];                 // destinatoin pan
	data_TX_normal_FIFO[6] = PAN_ID_2[0];
	data_TX_normal_FIFO[7] = ADDRESS_short_2[0];          // destination address
	data_TX_normal_FIFO[8] = ADDRESS_short_2[1];
	data_TX_normal_FIFO[9] = PAN_ID_1[0];                 // source pan
	data_TX_normal_FIFO[10] = PAN_ID_1[1];
	data_TX_normal_FIFO[11] = ADDRESS_short_1[0];          // source address
	data_TX_normal_FIFO[12] = ADDRESS_short_1[1];

	data_TX_normal_FIFO[13] = DATA_TX[0];                  // data

	for (i = 0; i < (HEADER_LENGHT + DATA_LENGHT + 2); i++) {
		write_ZIGBEE_long(address_TX_normal_FIFO + i, data_TX_normal_FIFO[i]); // write frame into normal FIFO
	}

	set_not_ACK();
	set_not_encrypt();
	start_transmit();
}

//Reset_Routines.c
void MRF24::PWR_reset() {
	write_ZIGBEE_short(MRF_SOFTRST, 0x04);   // 0x04  mask for RSTPWR bit
}

void MRF24::BB_reset() {
	write_ZIGBEE_short(MRF_SOFTRST, 0x02);   // 0x02 mask for RSTBB bit
}

void MRF24::MAC_reset() {
	write_ZIGBEE_short(MRF_SOFTRST, 0x01);   // 0x01 mask for RSTMAC bit
}

void MRF24::software_reset() {                // PWR_reset,BB_reset and MAC_reset at once
	write_ZIGBEE_short(MRF_SOFTRST, 0x07);
}

void MRF24::RF_reset() {
	short int temp = 0;
	temp = read_ZIGBEE_short(MRF_RFCTL);
	temp = temp | 0x04;                  // mask for RFRST bit
	write_ZIGBEE_short(MRF_RFCTL, temp);
	temp = temp & (!0x04);               // mask for RFRST bit
	write_ZIGBEE_short(MRF_RFCTL, temp);
	delay(1);
}

//Misc_Routines.c

void MRF24::init_ZIGBEE_nonbeacon() {
	init_ZIGBEE_basic();
	set_CCA_mode(1);     // Set CCA mode to ED and set threshold
	set_RSSI_mode(2);    // RSSI2 mode
	enable_interrupt();  // Enables all interrupts
	set_channel(11);     // Channel 11
	RF_reset();
}
void MRF24::init_ZIGBEE_basic() {
	write_ZIGBEE_short(MRF_PACON2, 0x98);   // Initialize FIFOEN = 1 and TXONTS = 0x6
	write_ZIGBEE_short(MRF_TXSTBL, 0x95);   // Initialize RFSTBL = 0x9
	write_ZIGBEE_long(MRF_RFCON1, 0x01);    // Initialize VCOOPT = 0x01
	enable_PLL();                       // Enable PLL (PLLEN = 1)
	write_ZIGBEE_long(MRF_RFCON6, 0x90);    // Initialize TXFIL = 1 and 20MRECVR = 1
	write_ZIGBEE_long(MRF_RFCON7, 0x80);    // Initialize SLPCLKSEL = 0x2 (100 kHz Internal oscillator)
	write_ZIGBEE_long(MRF_RFCON8, 0x10);    // Initialize RFVCO = 1
	write_ZIGBEE_long(MRF_SLPCON1, 0x21);   // Initialize CLKOUTEN = 1 and SLPCLKDIV = 0x01
}

/*
* Tx power
*/
void MRF24::set_TX_power(byte power) {             // 0-31 possible variants
	if ((power < 0) || (power > 31))
		power = 31;
	power = 31 - power;                                     // 0 max, 31 min -> 31 max, 0 min
	power = ((power & 0b00011111) << 3) & 0b11111000;       // calculating power
	write_ZIGBEE_long(MRF_RFCON3, power);
}


void MRF24::enable_PLL() {
	write_ZIGBEE_long(MRF_RFCON2, 0x80);       // mask for PLL enable
}

void MRF24::disable_PLL() {
	write_ZIGBEE_long(MRF_RFCON2, 0x00);       // mask for PLL disable
}

void MRF24::set_PAN_ID(byte * address) {
	write_ZIGBEE_short(MRF_PANIDL, address[0]);
	write_ZIGBEE_short(MRF_PANIDH, address[1]);
}

/*
* Address
*/
void MRF24::set_short_address(byte * address) {
	write_ZIGBEE_short(MRF_SADRL, address[0]);
	write_ZIGBEE_short(MRF_SADRH, address[1]);
}

void MRF24::set_long_address(byte* address) {
	short int i = 0;

	for (i = 0; i < 8; i++) {
		write_ZIGBEE_short(MRF_EADR0 + i, address[i]);   // 0x05 address of EADR0
	}
}

/*
*  Flush RX FIFO pointer
*/
void MRF24::flush_RX_FIFO_pointer() {
	byte temp;

	temp = read_ZIGBEE_short(MRF_RXFLUSH);
	temp = temp | 0x01;                        // mask for flush RX FIFO
	write_ZIGBEE_short(MRF_RXFLUSH, temp);
}

/*
*  Frame format filter
*/
void MRF24::set_frame_format_filter(byte fff_mode) {   // 1 all frames, 2 command only, 3 data only, 4 beacon only
	byte temp = 0;

	switch (fff_mode) {
	case 1: {
		temp = read_ZIGBEE_short(MRF_RXFLUSH);      // all frames
		temp = temp & (!0x0E);                  // mask for all frames
		write_ZIGBEE_short(MRF_RXFLUSH, temp);
	}
		break;

	case 2: {
		temp = read_ZIGBEE_short(MRF_RXFLUSH);      // command only
		temp = temp & (!0x06);                  // mask for command only
		temp = temp | 0x08;                     // mask for command only
		write_ZIGBEE_short(MRF_RXFLUSH, temp);
	}
		break;

	case 3: {
		temp = read_ZIGBEE_short(MRF_RXFLUSH);      // data only
		temp = temp & (!0x0A);                  // mask for data only
		temp = temp | 0x04;                     // mask for data only
		write_ZIGBEE_short(MRF_RXFLUSH, temp);
	}
		break;

	case 4: {
		temp = read_ZIGBEE_short(MRF_RXFLUSH);      // beacon only
		temp = temp & (!0x0C);                  // mask for beacon only
		temp = temp | 0x02;                     // mask for beacon only
		write_ZIGBEE_short(MRF_RXFLUSH, temp);
	}
			break;
	}
}

/*
* Reception mode
*/
void MRF24::set_reception_mode(byte r_mode) { // 1 normal, 2 error, 3 promiscuous mode
	byte temp = 0;

	switch (r_mode) {
	case 1: {
		temp = read_ZIGBEE_short(MRF_RXMCR);      // normal mode
		temp = temp & (!0x03);                // mask for normal mode
		write_ZIGBEE_short(MRF_RXMCR, temp);
	}
		break;

	case 2: {
		temp = read_ZIGBEE_short(MRF_RXMCR);      // error mode
		temp = temp & (!0x01);                // mask for error mode
		temp = temp | 0x02;                   // mask for error mode
		write_ZIGBEE_short(MRF_RXMCR, temp);
	}
		break;

	case 3: {
		temp = read_ZIGBEE_short(MRF_RXMCR);      // promiscuous mode
		temp = temp & (!0x02);                // mask for promiscuous mode
		temp = temp | 0x01;                   // mask for promiscuous mode
		write_ZIGBEE_short(MRF_RXMCR, temp);
	}
		break;
	}
}

void MRF24::set_IFS_default() {
	byte temp = 0;

	write_ZIGBEE_short(MRF_RXMCR, 0x75);    // Min SIFS Period

	temp = read_ZIGBEE_short(MRF_TXPEND);
	temp = temp | 0x84;                 // Min LIFS Period
	write_ZIGBEE_short(MRF_TXPEND, temp);

	temp = read_ZIGBEE_short(MRF_TXSTBL);
	temp = temp | 0x50;                 // Min LIFS Period
	write_ZIGBEE_short(MRF_TXSTBL, temp);

	temp = read_ZIGBEE_short(MRF_TXTIME);
	temp = temp | 0x41;                 // Turnaround Time
	write_ZIGBEE_short(MRF_TXTIME, temp);
}

/*
* Interframe spacing
*/
void MRF24::set_IFS_recomended() {
	byte temp = 0;

	write_ZIGBEE_short(MRF_RXMCR, 0x93);    // Min SIFS Period

	temp = read_ZIGBEE_short(MRF_TXPEND);
	temp = temp | 0x7C;                 // MinLIFSPeriod
	write_ZIGBEE_short(MRF_TXPEND, temp);

	temp = read_ZIGBEE_short(MRF_TXSTBL);
	temp = temp | 0x90;                 // MinLIFSPeriod
	write_ZIGBEE_short(MRF_TXSTBL, temp);

	temp = read_ZIGBEE_short(MRF_TXTIME);
	temp = temp | 0x31;                 // TurnaroundTime
	write_ZIGBEE_short(MRF_TXTIME, temp);
}

/*
*  Encrypt
*/
void MRF24::set_encrypt(void) {
	byte temp = 0;

	temp = read_ZIGBEE_short(MRF_TXNCON);
	temp = temp | 0x02;                   // mask for set encrypt
	write_ZIGBEE_short(MRF_TXNCON, temp);
}

void MRF24::set_not_encrypt(void) {
	byte temp = 0;

	temp = read_ZIGBEE_short(MRF_TXNCON);
	temp = temp & (!0x02);                // mask for set not encrypt
	write_ZIGBEE_short(MRF_TXNCON, temp);
}

/*
* ACK request
*/
void MRF24::set_ACK(void) {
	byte temp = 0;

	temp = read_ZIGBEE_short(MRF_TXNCON);
	temp = temp | 0x04;                   // 0x04 mask for set ACK
	write_ZIGBEE_short(MRF_TXNCON, temp);
}

void MRF24::set_not_ACK(void) {
	byte temp = 0;

	temp = read_ZIGBEE_short(MRF_TXNCON);
	temp = temp & (!0x04);                // 0x04 mask for set not ACK
	write_ZIGBEE_short(MRF_TXNCON, temp);
}

/*
* Set type of device
*/
void MRF24::nonbeacon_PAN_coordinator_device() {
	byte temp = 0;

	temp = read_ZIGBEE_short(MRF_RXMCR);
	temp = temp | 0x08;                 // 0x08 mask for PAN coordinator
	write_ZIGBEE_short(MRF_RXMCR, temp);

	temp = read_ZIGBEE_short(MRF_TXMCR);
	temp = temp & 0xDF;                 // 0xDF mask for CSMA-CA mode
	write_ZIGBEE_short(MRF_TXMCR, temp);

	write_ZIGBEE_short(MRF_ORDER, 0xFF);    // BO, SO are 15
}

void MRF24::nonbeacon_device() {
	byte temp = 0;

	temp = read_ZIGBEE_short(MRF_RXMCR);
	temp = temp & 0xF3;                 // 0xF3 mask for PAN coordinator and coordinator
	write_ZIGBEE_short(MRF_RXMCR, temp);

	temp = read_ZIGBEE_short(MRF_TXMCR);
	temp = temp & 0xDF;                 // 0xDF mask for CSMA-CA mode
	write_ZIGBEE_short(MRF_TXMCR, temp);
}

void MRF24::nonbeacon_coordinator_device() {
	byte temp = 0;

	temp = read_ZIGBEE_short(MRF_RXMCR);
	temp = temp | 0x04;                 // 0x04 mask for coordinator
	write_ZIGBEE_short(MRF_RXMCR, temp);

	temp = read_ZIGBEE_short(MRF_TXMCR);
	temp = temp & 0xDF;                 // 0xDF mask for CSMA-CA mode
	write_ZIGBEE_short(MRF_TXMCR, temp);

	write_ZIGBEE_short(MRF_ORDER, 0xFF);    // BO, SO  are 15
}


/*
*  Set RSSI mode
*/
void MRF24::set_RSSI_mode(byte RSSI_mode) {       // 1 for RSSI1, 2 for RSSI2 mode
	byte temp = 0;

	switch (RSSI_mode) {
	case 1: {
		temp = read_ZIGBEE_short(MRF_BBREG6);
		temp = temp | 0x80;                       // 0x80 mask for RSSI1 mode
		write_ZIGBEE_short(MRF_BBREG6, temp);
	}
		break;

	case 2:
		write_ZIGBEE_short(MRF_BBREG6, 0x40);         // 0x40 data for RSSI2 mode
		break;
	}
}

/*
*  Set CCA mode
*/
void MRF24::set_CCA_mode(byte CCA_mode) {
	byte temp = 0;
	switch (CCA_mode) {
	case 1: {                               // ENERGY ABOVE THRESHOLD
		temp = read_ZIGBEE_short(MRF_BBREG2);
		temp = temp | 0x80;                   // 0x80 mask
		temp = temp & 0xDF;                   // 0xDF mask
		write_ZIGBEE_short(MRF_BBREG2, temp);
		write_ZIGBEE_short(MRF_CCAEDTH, 0x60);    // Set CCA ED threshold to -69 dBm
	}
			break;

	case 2: {                               // CARRIER SENSE ONLY
		temp = read_ZIGBEE_short(MRF_BBREG2);
		temp = temp | 0x40;                   // 0x40 mask
		temp = temp & 0x7F;                   // 0x7F mask
		write_ZIGBEE_short(MRF_BBREG2, temp);

		temp = read_ZIGBEE_short(MRF_BBREG2);     // carrier sense threshold
		temp = temp | 0x38;
		temp = temp & 0xFB;
		write_ZIGBEE_short(MRF_BBREG2, temp);
	}
			break;

	case 3: {                               // CARRIER SENSE AND ENERGY ABOVE THRESHOLD
		temp = read_ZIGBEE_short(MRF_BBREG2);
		temp = temp | 0xC0;                   // 0xC0 mask
		write_ZIGBEE_short(MRF_BBREG2, temp);

		temp = read_ZIGBEE_short(MRF_BBREG2);     // carrier sense threshold
		temp = temp | 0x38;                   // 0x38 mask
		temp = temp & 0xFB;                   // 0xFB mask
		write_ZIGBEE_short(MRF_BBREG2, temp);

		write_ZIGBEE_short(MRF_CCAEDTH, 0x60);    // Set CCA ED threshold to -69 dBm
	}
			break;
	}
}

void  MRF24::set_channel(byte channel_number) {               // 11-26 possible channels
	if ((channel_number > 26) || (channel_number < 11)) channel_number = 11;
	switch (channel_number) {
	case 11:
		write_ZIGBEE_long(MRF_RFCON0, 0x02);  // 0x02 for 11. channel
		break;
	case 12:
		write_ZIGBEE_long(MRF_RFCON0, 0x12);  // 0x12 for 12. channel
		break;
	case 13:
		write_ZIGBEE_long(MRF_RFCON0, 0x22);  // 0x22 for 13. channel
		break;
	case 14:
		write_ZIGBEE_long(MRF_RFCON0, 0x32);  // 0x32 for 14. channel
		break;
	case 15:
		write_ZIGBEE_long(MRF_RFCON0, 0x42);  // 0x42 for 15. channel
		break;
	case 16:
		write_ZIGBEE_long(MRF_RFCON0, 0x52);  // 0x52 for 16. channel
		break;
	case 17:
		write_ZIGBEE_long(MRF_RFCON0, 0x62);  // 0x62 for 17. channel
		break;
	case 18:
		write_ZIGBEE_long(MRF_RFCON0, 0x72);  // 0x72 for 18. channel
		break;
	case 19:
		write_ZIGBEE_long(MRF_RFCON0, 0x82);  // 0x82 for 19. channel
		break;
	case 20:
		write_ZIGBEE_long(MRF_RFCON0, 0x92);  // 0x92 for 20. channel
		break;
	case 21:
		write_ZIGBEE_long(MRF_RFCON0, 0xA2);  // 0xA2 for 21. channel
		break;
	case 22:
		write_ZIGBEE_long(MRF_RFCON0, 0xB2);  // 0xB2 for 22. channel
		break;
	case 23:
		write_ZIGBEE_long(MRF_RFCON0, 0xC2);  // 0xC2 for 23. channel
		break;
	case 24:
		write_ZIGBEE_long(MRF_RFCON0, 0xD2);  // 0xD2 for 24. channel
		break;
	case 25:
		write_ZIGBEE_long(MRF_RFCON0, 0xE2);  // 0xE2 for 25. channel
		break;
	case 26:
		write_ZIGBEE_long(MRF_RFCON0, 0xF2);  // 0xF2 for 26. channel
		break;
	}
	RF_reset();
}

/*
*  Interrupt
*/
void MRF24::enable_interrupt() {
	write_ZIGBEE_short(MRF_INTCON, 0x00);   // 0x00  all interrupts are enable
}


void MRF24::interrupt_handler(void) {
	Serial.println("this intteroup");
}

