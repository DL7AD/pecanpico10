#ifndef __SD_H__
#define __SD_H__

#include "ch.h"
#include "hal.h"

extern MMCDriver MMCD1;

bool initSD(void);
bool writeBufferToFile(const char *filename, const uint8_t *buffer, uint32_t len);

#endif

