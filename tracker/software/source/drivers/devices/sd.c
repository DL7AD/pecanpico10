#include "ch.h"
#include "hal.h"
#include "ff.h"
#include "debug.h"
#include "portab.h"
//#include "config.h"

MMCDriver MMCD1;
bool sdInitialized = false;

bool initSD(void)
{

  /*
   * Quick check of card inserted.
   * This is OK for in flight.
   */
  if(palReadLine(LINE_SD_DET)) {
      TRACE_INFO("SD   > No SD card inserted");
      sdInitialized = false;
      return false;
  }
  TRACE_INFO("SD   > Access SD card");

    /* NOTE: SD_CS line is set in board.h and initialized to HIGH. */

	// Maximum speed SPI configuration
	static SPIConfig hs_spicfg = {
		.ssport	= PAL_PORT(LINE_SD_CS),
		.sspad	= PAL_PAD(LINE_SD_CS),
		.cr1	= SPI_CR1_MSTR
	};

	// Low speed SPI configuration
	static SPIConfig ls_spicfg = {
		.ssport	= PAL_PORT(LINE_SD_CS),
		.sspad	= PAL_PAD(LINE_SD_CS),
		.cr1	= SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_MSTR
	};

	// MMC/SD over SPI driver configuration
	static MMCConfig mmccfg = {&SPI_BUS1_DRIVER, &ls_spicfg, &hs_spicfg};

	// Check SD card presence
	spiAcquireBus(&SPI_BUS1_DRIVER);

	// Init MMC
	mmcObjectInit(&MMCD1);
	mmcStart(&MMCD1, &mmccfg);

	TRACE_DEBUG("SD   > Connect");
	if(mmcConnect(&MMCD1)) {
		TRACE_ERROR("SD   > SD card connection error");
        sdInitialized = false;
	} else {
		TRACE_INFO("SD   > SD card connection OK");
		sdInitialized = true;
	}
	spiReleaseBus(&SPI_BUS1_DRIVER);

	return sdInitialized;
}

bool writeBufferToFile(const char *filename, const uint8_t *buffer, uint32_t len)
{
	if(!sdInitialized)
		return false;

	spiAcquireBus(&SPI_BUS1_DRIVER);

	static FATFS fs;
	static FIL fdst;
	FRESULT res;
	bool gres = true; // Optimist

	// Mount SD card
	TRACE_INFO("SD   > Mount");
	res = f_mount(&fs, "/", 0);
	if(res != FR_OK)
	{

		TRACE_ERROR("SD   > Mounting failed (err=%d)", res);
		gres = false;

	} else {

		// Open file
		TRACE_INFO("SD   > Open file %s", filename);
		res = f_open(&fdst, (TCHAR*)filename, FA_CREATE_ALWAYS | FA_WRITE);
		if(res != FR_OK)
		{

			TRACE_ERROR("SD   > Opening file failed (err=%d)", res);
			gres = false;

		} else {

			// Write buffer into file
			TRACE_INFO("SD   > Write buffer to file (len=%d)", len);
			uint32_t len_written;
			f_write(&fdst, buffer, len, (UINT*)&len_written);
			if(len_written != len)
			{
				TRACE_ERROR("SD   > Writing failed (err=%d)", res);
				gres = false;
			}

			// Close file
			TRACE_INFO("SD   > Close file");
			res = f_close(&fdst);
			if(res != FR_OK)
			{
				TRACE_ERROR("SD   > Closing file failed (err=%d)", res);
				gres = false;
			}

		}

		// Unmount
		TRACE_INFO("SD   > Unmount");
		res = f_mount(0, "", 0);
		if(res != FR_OK)
		{
			TRACE_ERROR("SD   > Unmounting failed (err=%d)", res);
			gres = false;
		}
	}

	spiReleaseBus(&SPI_BUS1_DRIVER);

	return gres;
}

#if HAL_USE_SDC || defined(__DOXYGEN__)
/**
 * @brief   SDC card detection.
 */
bool sdc_lld_is_card_inserted(SDCDriver *sdcp) {

  (void)sdcp;
  /* TODO: Fill the implementation.*/
  return true;
}

/**
 * @brief   SDC card write protection detection.
 */
bool sdc_lld_is_write_protected(SDCDriver *sdcp) {

  (void)sdcp;
  /* TODO: Fill the implementation.*/
  return false;
}
#endif /* HAL_USE_SDC */

#if HAL_USE_MMC_SPI || defined(__DOXYGEN__)
/**
 * @brief   MMC_SPI card detection.
 */
bool mmc_lld_is_card_inserted(MMCDriver *mmcp) {

  (void)mmcp;
  return   palReadLine(LINE_SD_DET);
}

/**
 * @brief   MMC_SPI card write protection detection.
 */
bool mmc_lld_is_write_protected(MMCDriver *mmcp) {

  (void)mmcp;
  /* TODO: Fill the implementation.*/
  return false;
}
#endif
