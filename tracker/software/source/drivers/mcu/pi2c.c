/**
  * I2C wrapper for ChibiOS due to a bug: I2C blocking when I2C transfer suffered timeout
  * @see https://github.com/psas/stm32/commit/32ec8c97a1e5bf605bd5d41a89fc60b60e136af2
  */
#include "ch.h"
#include "hal.h"
#include "pi2c.h"

//#define I2C_DRIVER	(&I2CD1)

static uint8_t error;

const I2CConfig _i2cfg = {
	OPMODE_I2C,
	50000,
	STD_DUTY_CYCLE,
};

void I2C_Lock(I2CDriver *bus) {
#if  CH_CFG_USE_MUTEXES_RECURSIVE == TRUE
  i2cAcquireBus(bus);
#else
  (void)bus;
#endif
}

void I2C_Unlock(I2CDriver *bus) {
#if  CH_CFG_USE_MUTEXES_RECURSIVE == TRUE
  i2cReleaseBus(bus);
#else
  (void)bus;
#endif
}
/**
 * @note Error codes from i2cGetErrors()
 *
 * 0x01 I2C_BUS_ERROR
 * 0x02 I2C_ARBITRATION_LOST
 * 0x04 I2C_ACK_FAILURE
 * 0x08 I2C_OVERRUN
 * 0x10 I2C_PEC_ERROR
 * 0x20 I2C_TIMEOUT
 * 0x40 I2C_SMB_ALERT
 */
static bool I2C_transmit(I2CDriver *bus, uint8_t addr, uint8_t *txbuf, uint32_t txbytes, uint8_t *rxbuf, uint32_t rxbytes, sysinterval_t timeout) {
	i2cAcquireBus(bus);
	i2cStart(bus, &_i2cfg);
	msg_t i2c_status = i2cMasterTransmitTimeout(bus, addr, txbuf, txbytes, rxbuf, rxbytes, timeout);
	i2cStop(bus);
	i2cReleaseBus(bus);

	if(i2c_status == MSG_TIMEOUT) { // Restart I2C at timeout
		TRACE_ERROR("I2C  > TIMEOUT (ADDR 0x%02x)", addr);
		error = 0x1;
	} else if(i2c_status == MSG_RESET) {
		TRACE_ERROR("I2C  > RESET (ADDR 0x%02x) with errors %x", addr,
		            i2cGetErrors(bus));
		error = 0x1;
	} else {
		error = 0x0;
	}
	return i2c_status == MSG_OK;
}

bool I2C_write8(I2CDriver *bus, uint8_t address, uint8_t reg, uint8_t value)
{
	uint8_t txbuf[] = {reg, value};
	return I2C_transmit(bus, address, txbuf, 2, NULL, 0, TIME_MS2I(100));
}

bool I2C_writeN(I2CDriver *bus, uint8_t address, uint8_t *txbuf, uint32_t length)
{
	return I2C_transmit(bus, address, txbuf, length, NULL, 0, TIME_MS2I(100));
}

bool I2C_read8(I2CDriver *bus, uint8_t address, uint8_t reg, uint8_t *val)
{
	uint8_t txbuf[] = {reg};
	uint8_t rxbuf[1];
	bool ret = I2C_transmit(bus, address, txbuf, 1, rxbuf, 1, TIME_MS2I(100));
	*val = rxbuf[0];
	return ret;
}

bool I2C_read16(I2CDriver *bus, uint8_t address, uint8_t reg, uint16_t *val)
{
	uint8_t txbuf[] = {reg};
	uint8_t rxbuf[2];
	bool ret = I2C_transmit(bus, address, txbuf, 1, rxbuf, 2, TIME_MS2I(100));
	*val =  (rxbuf[0] << 8) | rxbuf[1];
	return ret;
}

bool I2C_read16_LE(I2CDriver *bus, uint8_t address, uint8_t reg, uint16_t *val) {
	bool ret = I2C_read16(bus, address, reg, val);
	*val = (*val >> 8) | (*val << 8);
	return ret;
}


bool I2C_read8_16bitreg(I2CDriver *bus, uint8_t address, uint16_t reg, uint8_t *val) // 16bit register (for OV5640)
{
	uint8_t txbuf[] = {reg >> 8, reg & 0xFF};
	uint8_t rxbuf[1];
	bool ret = I2C_transmit(bus, address, txbuf, 2, rxbuf, 1, TIME_MS2I(100));
	*val = rxbuf[0];
	return ret;
}

bool I2C_write8_16bitreg(I2CDriver *bus, uint8_t address, uint16_t reg, uint8_t value) // 16bit register (for OV5640)
{
	uint8_t txbuf[] = {reg >> 8, reg & 0xFF, value};
	return I2C_transmit(bus, address, txbuf, 3, NULL, 0, TIME_MS2I(100));
}

uint8_t I2C_hasError(void)
{
	return error;
}

