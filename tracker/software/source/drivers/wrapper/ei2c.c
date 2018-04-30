/**
  * I2C driver for the external I2C bus which is addressed on the TXD and RXD
  * pins of the Pecan. The I2C bus is bitbanged and operates at 45 kHz if the
  * STM32 is operated at SYSCLK=48MHz.
  * 
  * TXD pin: SCL
  * RXD pin: SDA
  * 
  * @see https://en.wikipedia.org/wiki/I%C2%B2C
  */

#include "ch.h"
#include "hal.h"
#include "debug.h"

#define SCL		LINE_IO_TXD /* SCL is connected to the TXD labeled line */
#define SDA		LINE_IO_RXD /* SDA is connected to the RXD labeled line */

static bool started = false;

static inline bool read_SCL(void) { // Return current level of SCL line, 0 or 1
	palSetLineMode(SCL, PAL_MODE_INPUT_PULLUP | PAL_STM32_OSPEED_HIGHEST);
	return palReadLine(SCL);
}
static inline bool read_SDA(void) { // Return current level of SDA line, 0 or 1
	palSetLineMode(SDA, PAL_MODE_INPUT_PULLUP | PAL_STM32_OSPEED_HIGHEST);
	return palReadLine(SDA);
}
static inline void set_SCL(void) { // Do not drive SCL(set pin high-impedance)
	palSetLineMode(SCL, PAL_MODE_INPUT_PULLUP | PAL_STM32_OSPEED_HIGHEST);
}
static inline void clear_SCL(void) { // Actively drive SCL signal low
	palSetLineMode(SCL, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	palClearLine(SCL);
}
static inline void set_SDA(void) { // Do not drive SDA(set pin high-impedance)
	palSetLineMode(SDA, PAL_MODE_INPUT_PULLUP | PAL_STM32_OSPEED_HIGHEST);
}
static inline void clear_SDA(void) { // Actively drive SDA signal low
	palSetLineMode(SDA, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	palClearLine(SDA);
}
static inline void arbitration_lost(void) {
	TRACE_ERROR("arbitration_lost");
}

static void i2c_start_cond(void) {
	if(started) { 
		// if started, do a restart condition
		// set SDA to 1
		set_SDA();
		set_SCL();
		sysinterval_t t0 = chVTGetSystemTime();
		while(read_SCL() == 0 && TIME_I2MS(chVTGetSystemTime()-t0) < 10) { // Clock stretching
			// You should add timeout to this loop
		}
	}
	if(read_SDA() == 0)
		arbitration_lost();

	// SCL is high, set SDA from 1 to 0.
	clear_SDA();
	clear_SCL();
	started = true;
}
static void i2c_stop_cond(void) {
	// set SDA to 0
	clear_SDA();
	set_SCL();
	// Clock stretching
	sysinterval_t t0 = chVTGetSystemTime();
	while(read_SCL() == 0 && TIME_I2MS(chVTGetSystemTime()-t0) < 10) { // Clock stretching
		// add timeout to this loop.
	}

	// SCL is high, set SDA from 0 to 1
	set_SDA();
	if(read_SDA() == 0)
		arbitration_lost();

	started = false;
}
// Write a bit to I2C bus
static void i2c_write_bit(bool bit) {
	if(bit) {
		set_SDA();
	} else {
		clear_SDA();
	}

	// Set SCL high to indicate a new valid SDA value is available
	set_SCL();

	// Wait for SDA value to be read by slave, minimum of 4us for standard mode
	sysinterval_t t0 = chVTGetSystemTime();
	while(read_SCL() == 0 && TIME_I2MS(chVTGetSystemTime()-t0) < 10) { // Clock stretching
		// You should add timeout to this loop
	}

	// SCL is high, now data is valid
	// If SDA is high, check that nobody else is driving SDA
	if(bit &&(read_SDA() == 0))
		arbitration_lost();

	// Clear the SCL to low in preparation for next change
	clear_SCL();
}

// Read a bit from I2C bus
static bool i2c_read_bit(void) {
	bool bit;
	// Let the slave drive data
	set_SDA();

	// Set SCL high to indicate a new valid SDA value is available
	set_SCL();

	sysinterval_t t0 = chVTGetSystemTime();
	while(read_SCL() == 0 && TIME_I2MS(chVTGetSystemTime()-t0) < 10) { // Clock stretching
		// You should add timeout to this loop
	}

	// SCL is high, read out bit
	bit = read_SDA();
	// Set SCL low in preparation for next operation
	clear_SCL();

	return bit;
}
// Write a byte to I2C bus. Return 0 if ack by the slave.
static bool i2c_write_byte(bool send_start, bool send_stop, uint8_t byte) {
	uint8_t bit;
	bool nack;

	if(send_start)
		i2c_start_cond();

	for(bit = 0; bit < 8; ++bit) {
		i2c_write_bit((byte & 0x80) != 0);
		byte <<= 1;
	}

	nack = i2c_read_bit();
	if(send_stop)
		i2c_stop_cond();

	return nack;
}

// Read a byte from I2C bus
static uint8_t i2c_read_byte(bool nack, bool send_stop) {
	uint8_t byte = 0;
	uint8_t bit;

	for(bit = 0; bit < 8; ++bit)
		byte =(byte << 1) | i2c_read_bit();

	i2c_write_bit(nack);
	if(send_stop)
		i2c_stop_cond();

	return byte;
}

bool eI2C_write8(uint8_t address, uint8_t reg, uint8_t value) {
	i2c_write_byte(true, false, address << 1);
	i2c_write_byte(false, false, reg);
	i2c_write_byte(false, true, value);
	return true;
}

bool eI2C_read8(uint8_t address, uint8_t reg, uint8_t *val) {
	i2c_write_byte(true, false, address << 1);
	i2c_write_byte(false, false, reg);
	i2c_write_byte(true, false,(address << 1) | 0x1);
	*val = i2c_read_byte(true, true);
	return true;
}

bool eI2C_read16(uint8_t address, uint8_t reg, uint16_t *val) {
	i2c_write_byte(true, false, address << 1);
	i2c_write_byte(false, false, reg);

	i2c_write_byte(true, false,(address << 1) | 0x1);
	uint8_t b1 = i2c_read_byte(false, false);
	uint8_t b2 = i2c_read_byte(true, true);
	*val = b2 |(b1 << 8);
	return true;
}

bool eI2C_read16_LE(uint8_t address, uint8_t reg, uint16_t *val) {
	i2c_write_byte(true, false, address << 1);
	i2c_write_byte(false, false, reg);

	i2c_write_byte(true, false,(address << 1) | 0x1);
	uint8_t b1 = i2c_read_byte(false, false);
	uint8_t b2 = i2c_read_byte(true, true);
	*val = b1 |(b2 << 8);
	return true;
}

