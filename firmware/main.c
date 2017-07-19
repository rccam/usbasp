/*
 * USBasp - USB in-circuit programmer for Atmel AVR controllers
 *
 * Thomas Fischl <tfischl@gmx.de>
 *
 * License........: GNU GPL v2 (see Readme.txt)
 * Target.........: ATMega8 at 12 MHz
 * Creation Date..: 2005-02-20
 * Last change....: 2009-02-28
 *
 * PC2 SCK speed option.
 * GND  -> slow (8khz SCK),
 * open -> software set speed (default is Auto determine SCK)
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <string.h>  //PDI
#include <avr/eeprom.h> //ATTINY

#include "usbasp.h"
#include "usbdrv.h"
#include "isp.h"
#include "clock.h"
#include "tpi.h"
#include "tpi_defs.h"
#include "pdi.h"
#include "uart.h"

static uchar replyBuffer[8];

static uchar prog_state = PROG_STATE_IDLE;
static uchar prog_sck = USBASP_ISP_SCK_AUTO;

static uchar prog_address_newmode = 0;
static unsigned long prog_address;
static unsigned int prog_nbytes = 0;
static unsigned int prog_pagesize;
static uchar prog_blockflags;
static uchar prog_pagecounter;
#ifndef USBASP_CFG_DISABLE_PDI
static uchar prog_buf[128]; //PDI
static uchar prog_buf_pos;  //PDI
#endif
#ifdef USE_USBASP_TTY
static uchar debug_byte='X';
static unsigned long baud = BAUD_RATE;
#endif

/* Array of clock speeds to try during auto probing */
static const uchar auto_clocks[] = {
#ifndef FORCE_ISP_SW  //too fast for sw
	USBASP_ISP_SCK_1500,
	USBASP_ISP_SCK_375,
#endif
	USBASP_ISP_SCK_93_75,
	USBASP_ISP_SCK_16,
	USBASP_ISP_SCK_4,
	USBASP_ISP_SCK_1,
    USBASP_ISP_SCK_0_5,
};

/* Auto probing of spi clock
 * The compiler generates smaller code when this is a seperate function vs inline
 * in the usbFunctionSetup() code. (it won't all fit in an atmega48 without doing this)
 */
uchar autosck(uchar sck)
{
uchar i;

	for (i = 0; i < (sizeof(auto_clocks) / sizeof(auto_clocks[0])); i++) {
		ispSetSCKOption(auto_clocks[i]);
		ispSafeResetPulse(); // ensure each attempt starts fresh
		if (!ispEnterProgrammingMode()) {
			/*
			 * it worked, but try again, as an extra precaution
			 */
			ispSafeResetPulse();
			if (!ispEnterProgrammingMode()) {
				/* This is the highest clock speed that worked.
				 * take target briefly out of reset... 
				 * needed for subsequent flashing to work
				 * since programming mode will be entered again
				 */
				sck = auto_clocks[i];
				ispSafeResetPulse();
				return sck;
			}
		}
	}

	/*
	 * If we get here, auto sck selection failed.
	 * we use the sck that was passed in as the fallback sck.
	 * (sck is still the orignal fallback value passed in)
	 */
	ispSetSCKOption(sck);
	return sck;
}

uchar usbFunctionSetup(uchar data[8]) {

	uchar len = 0;
#ifdef USE_USBASP_TTY
    uchar i;
#endif

	if (data[1] == USBASP_FUNC_CONNECT) {
#ifndef ATTINY
		/* Force SCK speed if jumper set */
		if ((PINC & (1 << PC2)) == 0)
			prog_sck = USBASP_ISP_SCK_4;
#endif
		ispSetSCKOption(prog_sck);

		/* set compatibility mode of address delivering */
		prog_address_newmode = 0;

		ledRedOn();
		ispConnect();

		/*
		 * Auto sck probing is done here vs in FUNC_ENABLEPROG
		 * So that host applications that use FUNC_TRANSMIT to do
		 * their own raw ISP port access instead of letting the USBasp device
		 * do it for them will also benefit from the auto sck probing.
		 */

		if (prog_sck == USBASP_ISP_SCK_AUTO) {
			/*
			 * we could fail the connect command if auto sck fails,
			 * instead, we will fallback to a very slow clock.
			 * as a failsafe. (This shouldn't ever happen)
			 */
			prog_sck = autosck(USBASP_ISP_SCK_4); // if auto fails, fallback to a slow clock
		}

		replyBuffer[0] = 0;		// connect worked
		replyBuffer[1] = prog_sck;	// return connected ISP clock
		len = 2;

	} else if (data[1] == USBASP_FUNC_DISCONNECT) {
		ispDisconnect();
		ledRedOff();
		/* ensure next connect can auto clock select */
		prog_sck = USBASP_ISP_SCK_AUTO;

	} else if (data[1] == USBASP_FUNC_TRANSMIT) {
		replyBuffer[0] = ispTransmit(data[2]);
		replyBuffer[1] = ispTransmit(data[3]);
		replyBuffer[2] = ispTransmit(data[4]);
		replyBuffer[3] = ispTransmit(data[5]);
		len = 4;

	} else if (data[1] == USBASP_FUNC_READFLASH) {

		if (!prog_address_newmode)
			prog_address = (data[3] << 8) | data[2];

		prog_nbytes = (data[7] << 8) | data[6];
		prog_state = PROG_STATE_READFLASH;
		len = 0xff; /* multiple in */

	} else if (data[1] == USBASP_FUNC_READEEPROM) {

		if (!prog_address_newmode)
			prog_address = (data[3] << 8) | data[2];

		prog_nbytes = (data[7] << 8) | data[6];
		prog_state = PROG_STATE_READEEPROM;
		len = 0xff; /* multiple in */

	} else if (data[1] == USBASP_FUNC_ENABLEPROG) {
		len = 1;
		replyBuffer[0] = ispEnterProgrammingMode();

	} else if (data[1] == USBASP_FUNC_WRITEFLASH) {

		if (!prog_address_newmode)
			prog_address = (data[3] << 8) | data[2];

		prog_pagesize = data[4];
		prog_blockflags = data[5] & 0x0F;
		prog_pagesize += (((unsigned int) data[5] & 0xF0) << 4);
		if (prog_blockflags & PROG_BLOCKFLAG_FIRST) {
			prog_pagecounter = prog_pagesize;
		}
		prog_nbytes = (data[7] << 8) | data[6];
		prog_state = PROG_STATE_WRITEFLASH;
		len = 0xff; /* multiple out */

	} else if (data[1] == USBASP_FUNC_WRITEEEPROM) {

		if (!prog_address_newmode)
			prog_address = (data[3] << 8) | data[2];

		prog_pagesize = 0;
		prog_blockflags = 0;
		prog_nbytes = (data[7] << 8) | data[6];
		prog_state = PROG_STATE_WRITEEEPROM;
		len = 0xff; /* multiple out */

	} else if (data[1] == USBASP_FUNC_SETLONGADDRESS) {

		/* set new mode of address delivering (ignore address delivered in commands) */
		prog_address_newmode = 1;
		/* set new address */
		prog_address = *((unsigned long*) &data[2]);

	} else if (data[1] == USBASP_FUNC_SETISPSCK) {

		/* set sck option */
		prog_sck = data[2];
		replyBuffer[0] = 0;
		len = 1;
	} else if (data[1] == USBASP_FUNC_GETISPSCK) {

		/* get sck option,
		 * usefull to read back the automatically probed clock speed */
		replyBuffer[0] = 0;
		replyBuffer[1] = prog_sck;
		len = 2;

#ifndef USBASP_CFG_DISABLE_TPI

	} else if (data[1] == USBASP_FUNC_TPI_CONNECT) {

		tpi_dly_cnt = data[2] | (data[3] << 8);

		/* RST high */
		ISP_OUT |= (1 << ISP_RST);
		ISP_DDR |= (1 << ISP_RST);

		clockWait(3);

		/* RST low */
		ISP_OUT &= ~(1 << ISP_RST);
		ledRedOn();

		clockWait(16);
		tpi_init();
	
	} else if (data[1] == USBASP_FUNC_TPI_DISCONNECT) {

		tpi_send_byte(TPI_OP_SSTCS(TPISR));
		tpi_send_byte(0);

		clockWait(10);

		/* pulse RST */
		ISP_OUT |= (1 << ISP_RST);
		clockWait(5);
		ISP_OUT &= ~(1 << ISP_RST);
		clockWait(5);

		/* set all ISP pins inputs */
		ISP_DDR &= ~((1 << ISP_RST) | (1 << ISP_SCK) | (1 << ISP_MOSI));
		/* switch pullups off */
		ISP_OUT &= ~((1 << ISP_RST) | (1 << ISP_SCK) | (1 << ISP_MOSI));

		ledRedOff();
	
	} else if (data[1] == USBASP_FUNC_TPI_RAWREAD) {
		replyBuffer[0] = tpi_recv_byte();
		len = 1;
	
	} else if (data[1] == USBASP_FUNC_TPI_RAWWRITE) {
		tpi_send_byte(data[2]);
	
	} else if (data[1] == USBASP_FUNC_TPI_READBLOCK) {
		prog_address = (data[3] << 8) | data[2];
		prog_nbytes = (data[7] << 8) | data[6];
		prog_state = PROG_STATE_TPI_READ;
		len = 0xff; /* multiple in */
	
	} else if (data[1] == USBASP_FUNC_TPI_WRITEBLOCK) {
		prog_address = (data[3] << 8) | data[2];
		prog_nbytes = (data[7] << 8) | data[6];
		prog_state = PROG_STATE_TPI_WRITE;
		len = 0xff; /* multiple out */
#endif
#ifndef USBASP_CFG_DISABLE_PDI
    } else if (data[1] == USBASP_FUNC_PDI_CONNECT) {    //PDI
        if ((replyBuffer[0]=pdiInit())==PDI_STATUS_OK)
        ledRedOn();
        len=1;
        
    } else if (data[1] == USBASP_FUNC_PDI_DISCONNECT) { //PDI
        ledRedOff();
        pdiCleanup(data[2]);
        
    } else if (data[1] == USBASP_FUNC_PDI_SEND) {       //PDI
        prog_nbytes = (data[7] << 8) | data[6];
        prog_blockflags = data[2];
        prog_state = PROG_STATE_PDI_SEND;
        prog_buf_pos = 0;
        len = 0xff;
        
    } else if (data[1] == USBASP_FUNC_PDI_READ) {       //PDI
        memmove(&prog_address,data+2,4);
        prog_nbytes = (data[7] << 8) | data[6];
        prog_state = PROG_STATE_PDI_READ;
        len = 0xff;
#endif
#ifdef USE_USBASP_TTY // ****** EMK Modifications for DEBUG serial interface ******
    } else if (data[1] == USBASP_FUNC_UART_GETBYTE ) {
        // Get a byte From UART outBuf (if any) send to HOST ***
        
        if(UART_outbufsize()>0)
            replyBuffer[0] = UART_outBuf_get();  //Get a byte from the outBuf buffer
        else
            replyBuffer[0]=0;     //Nothing to send, send NULL byte
        
        len = 1;
        
    } else if (data[1] == USBASP_FUNC_UART_PUTBYTE ) {
        // Get Data From HOST to be sent to UART ***
        debug_byte= data[2];         //Save byte to debug_byte
        PORTC &=0xFC;                //Clear Low 2 bits
        PORTC |= ~debug_byte & 0x03; //Show Low 2 bits in LEDs
        UART_inBuf_put(debug_byte);  //Save in inBuf for sending to UART
        
    } else if (data[1] == USBASP_FUNC_UART_GETBYTECOUNT ) {
        // Get #of bytes from UART waiting to be sent to HOST ***
        replyBuffer[0] = UART_outbufsize();  //Get #of bytes in outBuf buffer
        len = 1;
        
    } else if (data[1] == USBASP_FUNC_UART_SETBAUDRATE) {
        // Set UART Baud Rate (53)
        baud = *((unsigned long*) &data[2]);
        UART_init(baud);
        replyBuffer[0] = 0x53;  //Return Code = Command in Hex
        len = 1;
        
    } else if (data[1] == USBASP_FUNC_TEST_CMD1) {
        // Just For Demonstration...
        PORTC |= 0x03;        //Both LEDs OFF
        for(i=0; i<10; i++)   //Make the LEDs blink for  816ms
        {
            clockWait(255);   //320us * 255 = 81.6ms
            PORTC++;
        }
        PORTC=0xFC;                   //Restore the state of the LEDs
        replyBuffer[0] = debug_byte;  //Just return last byte sent from Host
        len = 1;
#endif // **** [END] EMK Modifications for DEBUG serial interface ******
    } else if (data[1] == USBASP_FUNC_GETCAPABILITIES) {
#ifndef USBASP_CFG_DISABLE_TPI
		replyBuffer[0] = USBASP_CAP_0_TPI;
#else
		replyBuffer[0] = 0;
#endif
		replyBuffer[1] = 0;
		replyBuffer[2] = 0;
		replyBuffer[3] = 0;
		len = 4;
	}

	usbMsgPtr = (usbMsgPtr_t) replyBuffer;

	return len;
}

uchar usbFunctionRead(uchar *data, uchar len) {

	uchar i;

    if (prog_state == PROG_STATE_READFLASH) {
        
        /* fill packet ISP mode */
        
        for (i = 0; i < len; i++) {
            data[i] = ispReadFlash(prog_address);
            prog_address++;
        }
        
        /* last packet? */
        
        if (len < 8)
        prog_state = PROG_STATE_IDLE;
        
    } else if (prog_state == PROG_STATE_READEEPROM) {
        /* fill packet ISP mode */
        
        for (i = 0; i < len; i++) {
            data[i] = ispReadEEPROM(prog_address);
            prog_address++;
        }
        
        /* last packet? */
        
        if (len < 8)
        prog_state = PROG_STATE_IDLE;

#ifndef USBASP_CFG_DISABLE_TPI
    } else if (prog_state == PROG_STATE_TPI_READ) {
        tpi_read_block(prog_address, data, len);
        prog_address += len;
#endif
#ifndef USBASP_CFG_DISABLE_PDI
    } else if (prog_state == PROG_STATE_PDI_READ) {     //PDI
        pdiDisableTimerClock();
        pdiSendIdle();
        
        if (pdi_nvmbusy)
        pdiWaitNVM();
        
        uchar ret=pdiReadBlock(prog_address, data, len);
        pdiEnableTimerClock();
        
        if (ret!=PDI_STATUS_OK)
        return 0;
        
        prog_address += len;
#endif
    } else {
        return 0xff;
    }
    
	return len;
}

uchar usbFunctionWrite(uchar *data, uchar len) {

	uchar retVal = 0;
	uchar i;

    if (prog_state == PROG_STATE_WRITEFLASH || prog_state == PROG_STATE_WRITEEEPROM ) {
        for (i = 0; i < len; i++) {
            if (prog_state == PROG_STATE_WRITEFLASH) {
                /* Flash */
                
                if (prog_pagesize == 0) {
                    /* not paged */
                    ispWriteFlash(prog_address, data[i], 1);
                    
                } else {
                    /* paged */
                    ispWriteFlash(prog_address, data[i], 0);
                    prog_pagecounter--;
                    
                    if (prog_pagecounter == 0) {
                        ispFlushPage(prog_address, data[i]);
                        prog_pagecounter = prog_pagesize;
                    }
                    
                }
                
            } else {
                /* EEPROM */
                ispWriteEEPROM(prog_address, data[i]);
            }
            
            prog_nbytes--;
            
            if (prog_nbytes == 0) {
                prog_state = PROG_STATE_IDLE;
                
                if ((prog_blockflags & PROG_BLOCKFLAG_LAST) && (prog_pagecounter != prog_pagesize)) {
                    /* last block and page flush pending, so flush it now */
                    ispFlushPage(prog_address, data[i]);
                }
                
                retVal = 1; // Need to return 1 when no more data is to be received
            }
            
            prog_address++;
        }
        
        return retVal;
        
#ifndef USBASP_CFG_DISABLE_TPI
    } else if (prog_state == PROG_STATE_TPI_WRITE) {
        tpi_write_block(prog_address, data, len);
        prog_address += len;
        prog_nbytes -= len;
        
        if(prog_nbytes <= 0) {
            prog_state = PROG_STATE_IDLE;
            return 1;
        }
        
        return 0;
#endif
#ifndef USBASP_CFG_DISABLE_PDI
    } else if (prog_state == PROG_STATE_PDI_SEND) {     //PDI
        memmove(&prog_buf[prog_buf_pos],data,len);
        prog_buf_pos += len;
        prog_nbytes -= len;
        
        if (prog_nbytes==0) {
            pdiDisableTimerClock();
            pdiSendIdle();
            
            if ((prog_blockflags & USBASP_PDI_WAIT_BUSY) && pdi_nvmbusy)
            pdiWaitNVM();
            
            pdiSendBytes(prog_buf,prog_buf_pos);
            
            if (prog_blockflags & USBASP_PDI_MARK_BUSY)
            pdi_nvmbusy=1;
            
            pdiEnableTimerClock();
            prog_state = PROG_STATE_IDLE;
            return 1;
        }
#endif
        return 0;
    }
    
    return 0xff;
}

#ifndef USBASP_CFG_DISABLE_USB_LEDSTATUS
/*
 * Hooks to control green led by USB status:
 * turn on: when a usb addresss is assigned (during enumeration)
 * turn off: when usb bus is reset
 */
void usbHadReset() {
	ledGreenOff();
}

void usbAddressAssigned() {
	ledGreenOn();
}
#endif

#ifdef CALIB_RC
/* ------------------------------------------------------------------------- */
/* ------------------------ Oscillator Calibration ------------------------- */
/* ------------------------------------------------------------------------- */

/* Calibrate the RC oscillator to 8.25 MHz. The core clock of 16.5 MHz is
  + * derived from the 66 MHz peripheral clock by dividing. Our timing reference
  + * is the Start Of Frame signal (a single SE0 bit) available immediately after
  + * a USB RESET. We first do a binary search for the OSCCAL value and then
  + * optimize this value with a neighboorhod search.
  + * This algorithm may also be used to calibrate the RC oscillator directly to
  + * 12 MHz (no PLL involved, can therefore be used on almost ALL AVRs), but this
  + * is wide outside the spec for the OSCCAL value and the required precision for
  + * the 12 MHz clock! Use the RC oscillator calibrated to 12 MHz for
  + * experimental purposes only!
  + */
static void calibrateOscillator(void) {
    
    uchar step = 128;
    uchar trialValue = 0, optimumValue;
    int x, optimumDev, targetValue = (unsigned)(1499 * (double)F_CPU / 10.5e6 + 0.5);
    
    /* do a binary search: */
    do {
        OSCCAL = trialValue + step;
        x = usbMeasureFrameLength();    /* proportional to current real frequency */
            
        if(x < targetValue)             /* frequency still too low */
            trialValue += step;
            
        step >>= 1;
    } while(step > 0);
    
    /* We have a precision of +/- 1 for optimum OSCCAL here */
    /* now do a neighborhood search for optimum value */
    optimumValue = trialValue;
    optimumDev = x; /* this is certainly far away from optimum */
    
    for(OSCCAL = trialValue - 1; OSCCAL <= trialValue + 1; OSCCAL++) {
        x = usbMeasureFrameLength() - targetValue;
            
        if(x < 0)
            x = -x;
        
        if(x < optimumDev) {
            optimumDev = x;
            optimumValue = OSCCAL;
        }
    }
    
        OSCCAL = optimumValue;
}

void usbEventResetReady(void)
{
    calibrateOscillator();
    eeprom_write_byte(0, OSCCAL);   /* store the calibrated value in EEPROM */
}
#endif /*CALIB_RC*/

/* ------------------------------------------------------------------------- */
/* --------------------------------- main ---------------------------------- */
/* ------------------------------------------------------------------------- */
int main(void) {

#ifdef ATTINY
    uchar   calibrationValue;
#endif
    
	/* all USB and ISP pins inputs */
	DDRB = 0;

	/* no pullups on USB and ISP pins */
	PORTB = 0;
#ifndef ATTINY
	/* set PD0 and PD1 low to drive the ISP pins low */
	PORTD = 0;

	/*
 	 * set PD0 and PD1 as outputs so when used on 10 pin ISP connector,
	 * ISP pins 4 and 6 will be driven low
	 * PD2 is input for using INT0 used by V-USB
	 */
	DDRD = ((1 << PD0) | (1 << PD1) );


	/*
	 * set PC0 and PC1 as outputs for LEDs
	 * set PC2 as input for slow SCK jumper
	 */
	DDRC = ( (1<< PC0) | (1 << PC1) );

	/*
	 * turn off red (PC0) & green (PC1) leds, and enable pullup for slow SCK jumper (PC2)
	 */

	PORTC = ( (1 << PC0) | (1 << PC1) | (1 << PC2));
#else
    calibrationValue = eeprom_read_byte(0); /* calibration value from last time */
    
    if(calibrationValue != 0xff){  //first connection after flash may be wrong (no data in eeprom)
        OSCCAL = calibrationValue;
    }
#endif
	/*
	 * if USB led status is disabled, turn green led on now
	 * Otherwise, green led will light when device has enumerated
	 */
#ifdef USBASP_CFG_DISABLE_USB_LEDSTATUS
	ledGreenOn();
#endif
#ifdef USE_USBASP_TTY
    /* init UART */
    UART_init(baud);
#endif
	/* init timer */
	clockInit();

	cli();
	
	/* Initialize V-USB */
	usbInit();
	
	/*
	 * perform a USB device disconnect
	 * Note: this is only necessary if the device wants to re-enumerate
	 * with the host without unplugging when restarting.
	 * This would be the case for a Watchdog Reset
	 * (which is currently not used by this f/w)
	 * Or just after a f/w update.
	 * Warning:
	 * While it works, given the USBasp h/w, this does violate the USB spec.
	 * See the notes in usbdrv.h
	 * with respect to usbDeviceDisconnect() / usbDeviceConnect()
	 * for further details.
	 *
	 * WARNING:
	 * The code below will flash the red LED unless USB_LEDSTATUS is disabled by
	 * defining USBASP_CFG_DISABLE_USB_LEDSTATUS
	 * h/w that uses the red led output signal for controlling an outut buffer may
	 * have issues with this since this flash will momentarily enable the output buffer.
	 */
	 
	usbDeviceDisconnect();
#ifndef USBASP_CFG_DISABLE_USB_LEDSTATUS
	ledRedOn();
#endif
	// delay min of 10ms to allow host time to see disconnect
	clockWait(64); // delay 20 ms
#ifndef USBASP_CFG_DISABLE_USB_LEDSTATUS
	ledRedOff();
#endif
	usbDeviceConnect();
	
	sei(); // enable AVR interrupts

	/* main event loop */
	for (;;) {
		usbPoll();
#ifdef USE_USBASP_TTY
        UART_poll();
#endif
	}
	return 0;
}
