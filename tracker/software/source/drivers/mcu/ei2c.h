#ifndef __EI2C_H__
#define __EI2C_H__

bool eI2C_write8(I2CDriver *bus, uint8_t address, uint8_t reg, uint8_t value);
bool eI2C_read8(I2CDriver *bus, uint8_t address, uint8_t reg, uint8_t *val);
bool eI2C_read16(I2CDriver *bus, uint8_t address, uint8_t reg, uint16_t *val);
bool eI2C_read16_LE(I2CDriver *bus, uint8_t address, uint8_t reg, uint16_t *val);

#endif

