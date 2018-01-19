#ifndef __MODULATION_H__
#define __MODULATION_H__

#include "ch.h"
#include "hal.h"
#include "ax25_pad.h"

void lockRadio(void);
void lockRadioByCamera(void);
void unlockRadio(void);

void initAFSK(void);
void sendAFSK(packet_t packet, uint32_t freq, uint8_t pwr);
void init2FSK(void);
void send2FSK(packet_t packet, uint32_t freq, uint8_t pwr);

#endif

