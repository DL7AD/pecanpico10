#ifndef __EI2C_H__
#define __EI2C_H__

bool eI2C_write8(uint8_t address, uint8_t reg, uint8_t value);
bool eI2C_read8(uint8_t address, uint8_t reg, uint8_t *val);
bool eI2C_read16(uint8_t address, uint8_t reg, uint16_t *val);
bool eI2C_read16_LE(uint8_t address, uint8_t reg, uint16_t *val);

#endif

