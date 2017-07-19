/*
 * usbasp.h - part of USBasp
 *
 * Autor..........: Thomas Fischl <tfischl@gmx.de>
 * Description....: Definitions and macros for usbasp
 * Licence........: GNU GPL v2 (see Readme.txt)
 * Creation Date..: 2009-02-28
 * Last change....: 2009-02-28
 */

#ifndef USBASP_H_
#define USBASP_H_

/* USB function call identifiers */
#define USBASP_FUNC_CONNECT     1
#define USBASP_FUNC_DISCONNECT  2
#define USBASP_FUNC_TRANSMIT    3
#define USBASP_FUNC_READFLASH   4
#define USBASP_FUNC_ENABLEPROG  5
#define USBASP_FUNC_WRITEFLASH  6
#define USBASP_FUNC_READEEPROM  7
#define USBASP_FUNC_WRITEEEPROM 8
#define USBASP_FUNC_SETLONGADDRESS 9
#define USBASP_FUNC_SETISPSCK 10
#define USBASP_FUNC_TPI_CONNECT      11
#define USBASP_FUNC_TPI_DISCONNECT   12
#define USBASP_FUNC_TPI_RAWREAD      13
#define USBASP_FUNC_TPI_RAWWRITE     14
#define USBASP_FUNC_TPI_READBLOCK    15
#define USBASP_FUNC_TPI_WRITEBLOCK   16
#define USBASP_FUNC_GETISPSCK        20
#define USBASP_FUNC_PDI_CONNECT      21
#define USBASP_FUNC_PDI_DISCONNECT   22
#define USBASP_FUNC_PDI_READ         23
#define USBASP_FUNC_PDI_SEND         24
#define USBASP_FUNC_GETCAPABILITIES 127

/* EMK Modifications for DEBUG serial interface */
#define USBASP_FUNC_UART_PUTBYTE      50
#define USBASP_FUNC_UART_GETBYTE      51
#define USBASP_FUNC_UART_GETBYTECOUNT 52
#define USBASP_FUNC_UART_SETBAUDRATE  53

#define USBASP_FUNC_TEST_CMD1         61
#define USBASP_FUNC_TEST_CMD2         62
#define USBASP_FUNC_TEST_CMD3         63

/* UART Baud Rate */
#define BAUD_RATE 9600
/* [END]EMK Modifications for DEBUG serial interface */

/* USBASP capabilities */
#define USBASP_CAP_0_TPI    0x01
#define USBASP_CAP_0_PDI    0x02

/* programming state */
#define PROG_STATE_IDLE         0
#define PROG_STATE_WRITEFLASH   1
#define PROG_STATE_READFLASH    2
#define PROG_STATE_READEEPROM   3
#define PROG_STATE_WRITEEEPROM  4
#define PROG_STATE_TPI_READ     5
#define PROG_STATE_TPI_WRITE    6
#define PROG_STATE_PDI_READ     7
#define PROG_STATE_PDI_SEND     8

/* Block mode flags */
#define PROG_BLOCKFLAG_FIRST    1
#define PROG_BLOCKFLAG_LAST     2

/* PDI */
#define USBASP_PDI_WAIT_BUSY   1
#define USBASP_PDI_MARK_BUSY   2

/* PDI */
#define EXIT_RESET_UNSPEC 0
#define EXIT_RESET_ENABLED 1
#define EXIT_RESET_DISABLED 2

/* ISP SCK speed identifiers */
#define USBASP_ISP_SCK_AUTO   0
#define USBASP_ISP_SCK_0_5    1   /* 500 Hz */
#define USBASP_ISP_SCK_1      2   /*   1 kHz */
#define USBASP_ISP_SCK_2      3   /*   2 kHz */
#define USBASP_ISP_SCK_4      4   /*   4 kHz */
#define USBASP_ISP_SCK_8      5   /*   8 kHz */
#define USBASP_ISP_SCK_16     6   /*  16 kHz */
#define USBASP_ISP_SCK_32     7   /*  32 kHz */
#define USBASP_ISP_SCK_93_75  8   /*  93.75 kHz */
#define USBASP_ISP_SCK_187_5  9   /* 187.5  kHz */
#define USBASP_ISP_SCK_375    10  /* 375 kHz   */
#define USBASP_ISP_SCK_750    11  /* 750 kHz   */
#define USBASP_ISP_SCK_1500   12  /* 1.5 MHz   */

#ifdef WITHOUT_LED
#define USBASP_CFG_DISABLE_USB_LEDSTATUS
#define ledRedOn()
#define ledRedOff()
#define ledGreenOn()
#define ledGreenOff()
#else
/* macros for gpio functions */
#define ledRedOn()    PORTC &= ~(1 << PC1)
#define ledRedOff()   PORTC |= (1 << PC1)
#define ledGreenOn()  PORTC &= ~(1 << PC0)
#define ledGreenOff() PORTC |= (1 << PC0)
#endif

#endif /* USBASP_H_ */
