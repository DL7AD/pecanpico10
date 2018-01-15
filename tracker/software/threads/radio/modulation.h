#ifndef __MODULATION_H__
#define __MODULATION_H__

#include "ch.h"
#include "hal.h"

void lockRadio(void);
void lockRadioByCamera(void);
void unlockRadio(void);

void initAFSK(radioMSG_t* msg);
void sendAFSK(uint32_t frequency);
void init2FSK(radioMSG_t* msg);
void send2FSK(uint32_t frequency);

#endif

