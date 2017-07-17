/*
 * pdi.h - part of PDI patch
 *
 * Autor..........: szulat@www.elektroda.pl
 * Last change....: 2012-08-16
 * Info(PL).......: www.elektroda.pl/rtvforum/topic2359783.html?sid=dc5bf363a1210ab871ceb3db697da6e7
 * Info(EN).......: www.szulat.blogspot.com/2012/08/atxmega-programmer-for-050.html
 */

#ifndef _PDI_H_
#define _PDI_H_

#include <stdint.h>

#ifndef uchar
#define	uchar	unsigned char
#endif

extern uchar pdi_active;
extern uchar pdi_interrupted;
extern uchar pdi_interrupt_count;
extern uchar pdi_se0_count;
extern uchar pdi_rx;
extern uchar pdi_nvmbusy;

#define PDI_STATUS_OK 0
#define PDI_STATUS_TIMEOUT 1
#define PDI_STATUS_PARITY 2
#define PDI_STATUS_BADSTOP 3
#define PDI_STATUS_NVM_TIMEOUT 4
#define PDI_STATUS_COLLISION 5

uchar pdiInit();
void pdiCleanup(uchar keep_reset);
void pdiEnableTimerClock();
void pdiDisableTimerClock();
uchar pdiTimerClockEnabled();
void pdiSetClk1();
void pdiSetClk0();
void pdiSetData1();
void pdiSetData0();
void pdiSetDataIn();
uchar pdiGetData();
uchar pdiWaitNVM();
uchar pdiReadBlock(uint32_t addr,uchar* data,uchar len);
void pdiSendIdle();
void pdiSendBreak();
uchar byteParity(uchar b);
void pdiSendByte(uchar b);
void pdiSendBytes(uchar* ptr,uchar count);
void pdiSendByteX(uchar b,uchar extra);
uchar pdiReadCtrl(uint32_t addr, uchar *value);
uchar pdiResetDev(uchar reset);

#endif
