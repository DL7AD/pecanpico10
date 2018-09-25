/**
  * I2C wrapper for ChibiOS due to a bug: I2C blocking when I2C transfer suffered timeout
  * @see https://github.com/psas/stm32/commit/32ec8c97a1e5bf605bd5d41a89fc60b60e136af2
  */

#ifndef __PI2C_H__
#define __PI2C_H__

#include "ch.h"
#include "hal.h"
#include "debug.h"
#include "config.h"

void pi2cInit(void);

bool I2C_write8(I2CDriver *bus, uint8_t address, uint8_t reg, uint8_t value);
bool I2C_writeN(I2CDriver *bus, uint8_t address, uint8_t *txbuf, uint32_t length);
bool I2C_read8(I2CDriver *bus, uint8_t address, uint8_t reg, uint8_t *val);
bool I2C_read16(I2CDriver *bus, uint8_t address, uint8_t reg, uint16_t *val);

bool I2C_write8_16bitreg(I2CDriver *bus, uint8_t address, uint16_t reg, uint8_t value); // 16bit register (for OV5640)
bool I2C_read8_16bitreg(I2CDriver *bus, uint8_t address, uint16_t reg, uint8_t *val); // 16bit register (for OV5640)

bool I2C_read16_LE(I2CDriver *bus, uint8_t address, uint8_t reg, uint16_t *val);

void I2C_Lock(I2CDriver *bus);
void I2C_Unlock(I2CDriver *bus);

uint8_t I2C_hasError(void);

#endif

