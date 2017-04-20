/*
 * Author: Le Hung
 * Email: lehungmt94@gmail.com
 * Website: mculearning.wordpress.com
 * 
 * Module: http://ww1.microchip.com/downloads/en/DeviceDoc/39776C.pdf
*/

#ifndef LIB_MRF24_H
#define LIB_MRF24_H

#if defined(ARDUINO) && ARDUINO >= 100 // Arduino IDE version >= 1.0
    #include "Arduino.h"
#else // older Arduino IDE versions
    #include "WProgram.h"
#endif
#include "SPI.h"

#include "MRF24_Reg.h"




/**
 * Based on the TXSTAT register, but "better"
 */

const byte DATA_LENGHT = 1;
const byte HEADER_LENGHT = 11;

class MRF24
{
    public:

		int address_RX_FIFO, address_TX_normal_FIFO;

		byte data_RX_FIFO[1 + HEADER_LENGHT + DATA_LENGHT + 2 + 1 + 1], lost_data;
		byte ADDRESS_short_1[2], ADDRESS_long_1[8];        // Source address
		byte ADDRESS_short_2[2], ADDRESS_long_2[8];        // Destination address
		byte PAN_ID_1[2];               // Source PAN ID
		byte PAN_ID_2[2];               // Destination PAN ID
		byte DATA_RX[DATA_LENGHT], DATA_TX[DATA_LENGHT], data_TX_normal_FIFO[DATA_LENGHT + HEADER_LENGHT + 2];
		byte LQI, RSSI2, SEQ_NUMBER;


        MRF24(int pin_reset, int pin_chip_select, int pin_interrupt);
		void begin(void);
        void reset(void);
        void init(void);

		//ReadWrite_Routines.c
        byte read_ZIGBEE_short(byte address);
        byte read_ZIGBEE_long(word address);

        void write_ZIGBEE_short(byte address, byte data);
        void write_ZIGBEE_long(word address, byte data);
		
		void read_RX_FIFO();
		void start_transmit();
		void write_TX_normal_FIFO();
		


		//Reset_Routines.c
		void PWR_reset();
		void BB_reset();
		void MAC_reset();
		void RF_reset();
		void software_reset();


		//Misc_Routines.c
		void init_ZIGBEE_nonbeacon();
		void init_ZIGBEE_basic();
		void set_TX_power(byte power);
		void disable_PLL();
		void enable_PLL();

		void set_PAN_ID(byte * address);
		void set_long_address(byte * address);
		void set_short_address(byte * address);
		void flush_RX_FIFO_pointer();
		void set_frame_format_filter(byte fff_mode);
		void set_reception_mode(byte r_mode);
		void set_IFS_default();
		void set_IFS_recomended();
		void set_not_encrypt();
		void set_encrypt();
		void set_not_ACK();
		void set_ACK();
		void nonbeacon_device();
		void nonbeacon_coordinator_device();
		void nonbeacon_PAN_coordinator_device();
		void set_RSSI_mode(byte RSSI_mode);
		void set_CCA_mode(byte CCA_mode);
		void set_channel(byte channel_number);
		void enable_interrupt();


        void interrupt_handler(void);


    private:
        int _pin_reset;
        int _pin_cs;
        int _pin_int;

};

#endif  /* LIB_MRF24_H */
