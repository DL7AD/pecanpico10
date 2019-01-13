/**
 * Si446x driver specialized for APRS transmissions. The driver supports APRS
 * transmission and reception.
 * There can be either used the SLabs Si4463 or Si4464.
 */

#include "pktconf.h"
#include "debug.h"
#include "radio.h"
#include "geofence.h"
#include "si4463_patch.h"
#include "tcxo.h"


/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

static const uint8_t Si4463_Patch_Data_Array[] = {
        SI4463_PATCH_CMDS,
        0x00
 };

/*
 * @brief   The SPI configuration.
 * @note    the CS line is set dynamically per radio.
 * @note    the SPI LLD sets the SPI_CR1_MSTR bit
 * @note    baud rate is PCLK/2 when SPI_CR1_BR = 0
 */
static SPIConfig ls_spicfg = {
    .cr1    = 0
};

/**
 * Get pointer to the radio specific configuration.
 */
static const si446x_mcucfg_t *Si446x_getConfig(const radio_unit_t radio) {
  const radio_config_t *data = pktGetRadioData(radio);
return (si446x_mcucfg_t *)data->cfg;
}

/**
 * Get pointer to the radio specific volatile data.
 */
static si446x_data_t *Si446x_getData(const radio_unit_t radio) {
  const radio_config_t *data = pktGetRadioData(radio);
return (si446x_data_t *)data->dat;
}

/**
 * Acquire bus and set the select line in SPI configuration.
 */
static SPIDriver *Si446x_spiSetupBus(const radio_unit_t radio,
                                     SPIConfig *cfg) {
  SPIDriver *spip = Si446x_getConfig(radio)->spi;
  spiAcquireBus(spip);
  cfg->ssport = PAL_PORT(Si446x_getConfig(radio)->cs);
  cfg->sspad = PAL_PAD(Si446x_getConfig(radio)->cs);
  return spip;
}

/**
 * SPI write which uses CTS presented on radio GPIO1.
 * Used when starting the radio up from shutdown state.
 * @pre The MCU GPIO pin connected to 446x GPIO1 must be configured as input.
 */
static bool Si446x_writeBoot(const radio_unit_t radio,
                             const si446x_args_t *txData, size_t len) {
  /* Write data via SPI with CTS checked via GPIO1. */

  /* Acquire bus and then start SPI. */
  SPIDriver *spip = Si446x_spiSetupBus(radio, &ls_spicfg);
  spiStart(spip, &ls_spicfg);

  /* Poll for CTS with timeout of 100mS. */
  ioline_t cts = Si446x_getConfig(radio)->gpio1;
  uint8_t timeout = 100;
  do {
      chThdSleep(TIME_MS2I(1));
  } while(palReadLine(cts) != PAL_HIGH && --timeout);

  if(timeout == 0) {
#if 0
    TRACE_ERROR("SI   > CTS not received");
#endif
    /* Stop SPI and relinquish bus. */
    spiStop(spip);
    spiReleaseBus(spip);
    return true;
  }

  /* Transfer data now there is CTS.*/
  spiSelect(spip);
  spiSend(spip, len, txData);
  spiUnselect(spip);

  /* Stop SPI and relinquish bus. */
  spiStop(spip);
  spiReleaseBus(spip);

  return false;
}

/**
 * SPI write.
 * Returns false on success, true on fail.
 */
static bool Si446x_write(const radio_unit_t radio,
                               const si446x_args_t *txData, size_t len) {
    /* Transmit data by SPI with CTS polling by command. */

    /* Acquire bus, get SPI Driver object and then start SPI. */
    SPIDriver *spip = Si446x_spiSetupBus(radio, &ls_spicfg);
    spiStart(spip, &ls_spicfg);

    /* Poll for CTS with timeout of 100mS. */
    uint8_t timeout = 100;
    si446x_args_t rx_ready[] = {Si446x_READ_CMD_BUFF, 0x00};
    do {
      spiSelect(spip);
      spiExchange(spip, 1, rx_ready, &rx_ready[1]);
      spiUnselect(spip);
      chThdSleep(TIME_MS2I(1));
    } while(rx_ready[1] != Si446x_COMMAND_CTS && --timeout);

    if (timeout == 0) {
      TRACE_ERROR("SI   > CTS not received from radio %d", radio);
      /* Stop SPI and relinquish bus. */
      spiStop(spip);
      spiReleaseBus(spip);
      return true;
    }
    
    /* Transfer data. */
    spiSelect(spip);
    spiSend(spip, len, txData);
    spiUnselect(spip);

    /* Stop SPI and relinquish bus. */
    spiStop(spip);
    spiReleaseBus(spip);
    
    return false;
}

/**
 * SPI read which uses CTS on GPIO1.
 * Use this when first taking radio out of shutdown.
 * The MCU GPIO pin connected to 446x GPIO1 must be already configured.
 */
static bool Si446x_readBoot(const radio_unit_t radio,
						const uint8_t* txData, uint32_t txlen,
                        uint8_t* rxData, uint32_t rxlen) {

    /* Acquire bus and get SPI Driver object. */
    SPIDriver *spip = Si446x_spiSetupBus(radio, &ls_spicfg);

    /* Poll for CTS with timeout of 100mS. */
    ioline_t cts = Si446x_getConfig(radio)->gpio1;
    uint8_t timeout = 100;
    while(palReadLine(cts) != PAL_HIGH && --timeout) {
        chThdSleep(TIME_MS2I(1));
    }

    if(timeout == 0) {
      /* Relinquish bus. */
      spiReleaseBus(spip);
      TRACE_ERROR("SI   > CTS not received from radio %d", radio);
      return false;
    }

    /*
     * Now write command and any data.
     */
    spiStart(spip, &ls_spicfg);
    spiSelect(spip);
    spiSend(spip, txlen, txData);
    spiUnselect(spip);

    /* Poll for CTS from command. */
    timeout = 100;
    while(palReadLine(cts) != PAL_HIGH && --timeout) {
        chThdSleep(TIME_MS2I(1));
    }

    if(timeout == 0) {
      /* Stop SPI and relinquish bus. */
      spiStop(spip);
      spiReleaseBus(spip);
      TRACE_ERROR("SI   > CTS not received from radio %d", radio);
      return false;
    }

    /* Read the response. */
    si446x_args_t rx_ready[] = {Si446x_READ_CMD_BUFF, 0x00};
    spiSelect(spip);
    spiExchange(spip, rxlen, rx_ready, rxData);
    spiUnselect(spip);

    /* Stop SPI and relinquish bus. */
    spiStop(spip);
    spiReleaseBus(spip);

    return true;
}

/**
 * Read data from Si446x.
 * Returns false on success true on failure.
 */
static bool Si446x_read(const radio_unit_t radio,
		                const uint8_t* txData, uint32_t txlen,
                        uint8_t* rxData, uint32_t rxlen) {

    /* Acquire bus and then start SPI. */
    SPIDriver *spip = Si446x_spiSetupBus(radio, &ls_spicfg);
    spiStart(spip, &ls_spicfg);

    /*
     * Poll command buffer waiting for CTS from the READ_CMD_BUFF command.
     * This command does not itself cause CTS to report busy.
     * Allocate a buffer to use for CTS check.
     * Timeout after 100mS waiting for CTS.
     */
    uint8_t timeout = 100;
    si446x_args_t rx_ready[] = {Si446x_READ_CMD_BUFF, 0x00};
    do {
      spiSelect(spip);
      spiExchange(spip, 1, rx_ready, &rx_ready[1]);
      spiUnselect(spip);
      chThdSleep(TIME_MS2I(1));
    } while(rx_ready[1] != Si446x_COMMAND_CTS && --timeout);

    if(timeout == 0) {
      //TRACE_ERROR("SI   > CTS not received from radio %d", radio);
      /* Stop SPI and relinquish bus. */
      spiStop(spip);
      spiReleaseBus(spip);
      return true;
    }

    /*
     * Now write command and data.
     */
    spiSelect(spip);
    spiSend(spip, txlen, txData);
    spiUnselect(spip);
    /*
     * Poll waiting for CTS again using the READ_CMD_BUFF command.
     * Once CTS is received the response data is ready in the rx data buffer.
     * The buffer contains the command, CTS and 0 - 16 bytes of response.
     * Timeout after 100mS waiting for CTS.
     */
    timeout = 100;
    do {
      spiSelect(spip);
      spiExchange(spip, rxlen, rx_ready, rxData);
      spiUnselect(spip);
      chThdSleep(TIME_MS2I(1));
    } while(rxData[1] != Si446x_COMMAND_CTS && --timeout);

    /* Stop SPI and relinquish bus. */
    spiStop(spip);
    spiReleaseBus(spip);
    
   if(timeout == 0) {
      //TRACE_ERROR("SI   > CTS not received from radio %d", radio);
      return true;
    }
    return false;
}

/**
 * @brief Configure transceiver GPIOs.
 */
static void Si446x_configureGPIO(const radio_unit_t radio,
                                 const si446x_gpio_t *gpio) {

  si446x_args_t gpio_pin_cfg_command[sizeof(si446x_gpio_t) + 1] = {
      Si446x_GPIO_PIN_CFG   // Command type = GPIO settings
  };

  memcpy(&gpio_pin_cfg_command[1], gpio, sizeof(si446x_gpio_t));
  (void)Si446x_write(radio, gpio_pin_cfg_command, sizeof(gpio_pin_cfg_command));
}

/* TODO: Make set property a single func with size parameter. */
static void Si446x_setProperty8(const radio_unit_t radio,
		uint16_t reg, uint8_t val) {
  si446x_args_t msg[] = {Si446x_SET_PROPERTY,
                     (reg >> 8) & 0xFF, 0x01, reg & 0xFF, val};
  (void)Si446x_write(radio, msg, sizeof(msg));
}

/**
 *
 */
static void Si446x_setProperty16(const radio_unit_t radio,
		uint16_t reg, uint8_t val1, uint8_t val2) {
  si446x_args_t msg[] = {Si446x_SET_PROPERTY,
                     (reg >> 8) & 0xFF, 0x02, reg & 0xFF, val1, val2};
  (void)Si446x_write(radio, msg, sizeof(msg));
}

/**
 *
 */
static void Si446x_setProperty24(const radio_unit_t radio,
		                         uint16_t reg, uint8_t val1,
                                 uint8_t val2, uint8_t val3) {
  si446x_args_t msg[] = {Si446x_SET_PROPERTY,
                     (reg >> 8) & 0xFF, 0x03, reg & 0xFF, val1, val2, val3};
  (void)Si446x_write(radio, msg, sizeof(msg));
}

/**
 *
 */
static void Si446x_setProperty32(const radio_unit_t radio,
		                         uint16_t reg, uint8_t val1,
                                 uint8_t val2, uint8_t val3, uint8_t val4) {
    uint8_t msg[] = {Si446x_SET_PROPERTY,
                     (reg >> 8) & 0xFF, 0x04, reg & 0xFF,
                     val1, val2, val3, val4};
    (void)Si446x_write(radio, msg, sizeof(msg));
}

/*
 *  Radio States
 */

size_t Si446x_getModemStatus(const radio_unit_t radio, si446x_reply_t *status,
                           size_t size) {
  /* Get status. Leave any pending interrupts intact. */
    const uint8_t status_info[] = {Si446x_GET_MODEM_STATUS, 0xFF};
    (void)Si446x_read(radio, status_info, sizeof(status_info), status, size);
    return (size < 11 ? size : 11);
}

/**
 *
 */
radio_signal_t Si446x_getCurrentRSSI(const radio_unit_t radio) {
  /* Get status and return RSSI value. */
  si446x_reply_t rxData[11];
  Si446x_getModemStatus(radio, rxData, sizeof(rxData));
  return rxData[4];
}
/**
 *
 */
static si446x_state_t Si446x_getState(const radio_unit_t radio) {
  const si446x_args_t state_info[] = {Si446x_REQUEST_DEVICE_STATE};
  si446x_reply_t rxData[4];
  (void)Si446x_read(radio, state_info, sizeof(state_info), rxData, sizeof(rxData));
  return (si446x_state_t)(rxData[2] & 0xF);
}

/**
 *
 */
static void Si446x_startPacketTX(const radio_unit_t radio,
                                 const radio_ch_t chan,
                                 const radio_payload_t size) {
  si446x_args_t change_state_command[] = {Si446x_START_TX, chan,
                                    (Si446x_READY << 4),
                                    (size >> 8) & 0x1F, size & 0xFF};
  (void)Si446x_write(radio, change_state_command, sizeof(change_state_command));
}

/**
 *
 */
static void Si446x_setReadyState(const radio_unit_t radio) {
  const si446x_args_t change_state_command[] = {Si446x_CHANGE_STATE,
                                          Si446x_READY};
  (void)Si446x_write(radio, change_state_command, sizeof(change_state_command));
}

/**
 *
 */
static void Si446x_startRXState(const radio_unit_t radio,
                                const radio_ch_t chan) {
  const si446x_args_t change_state_command[] = {Si446x_START_RX, chan, 0x00, 0x00,
                                          0x00,
                                          Si446x_REMAIN,
                                          Si446x_REMAIN,
                                          Si446x_REMAIN};
  (void)Si446x_write(radio, change_state_command, sizeof(change_state_command));
}

/**
 *
 */
static void Si446x_setStandbyState(const radio_unit_t radio) {
  const si446x_args_t change_state_command[] = {Si446x_CHANGE_STATE,
                                          Si446x_STANDBY};
  (void)Si446x_write(radio, change_state_command, sizeof(change_state_command));
}

/**
 * Get temperature of chip.
 */
static bool Si446x_getTemperature(const radio_unit_t radio) {
  const si446x_args_t txData[] = {Si446x_GET_ADC_READING, 0x10};
  si446x_reply_t rxData[8];
  if(!Si446x_read(radio, txData, sizeof(txData), rxData, sizeof(rxData))) {
    uint16_t adc = rxData[7] | ((rxData[6] & 0x7) << 8);
    int16_t temp = (89900 * adc) / 4096 - 29300;
    Si446x_getData(radio)->lastTemp = temp;
    return true;
  }
  return false;
}

/**
 * Clear interrupt status of chip.
 * Returns false on success, true on failure.
 */
static bool Si446x_clearInterruptStatus(const radio_unit_t radio) {
  const si446x_args_t txData[] = {Si446x_GET_INT_STATUS, 0x00, 0x00, 0x00};
  uint8_t rxData[9];
  return Si446x_read(radio, txData, sizeof(txData), rxData, sizeof(rxData));
}

/**
 * Initializes Si446x transceiver chip.
 * Returns false on success, true on fail.
 */
static bool Si446x_init(const radio_unit_t radio) {

  TRACE_DEBUG("SI   > Initialize radio %d", radio);

  packet_svc_t *handler = pktGetServiceObject(radio);

  /*
   * Set MCU GPIO for radio GPIO1 (CTS).
   * Execute radio wake up sequence.
   */
  if(Si446x_radioWakeUp(radio)) {
    TRACE_ERROR("SI   > Wake up of radio %d failed", radio);
    return true;
  }

  /*
   * An uncorrected XO frequency could also be used here.
   * This XO setting is only for boot up and not used in any PLL calcs.
   */
  radio_clock_t si_clock = Si446x_getData(radio)->radio_clock;

  /* Calculate clock source parameters. */
  const uint8_t x3 = (si_clock >> 24) & 0x0FF;
  const uint8_t x2 = (si_clock >> 16) & 0x0FF;
  const uint8_t x1 = (si_clock >>  8) & 0x0FF;
  const uint8_t x0 = (si_clock >>  0) & 0x0FF;

  /*
   * Start the chip API with the POWER_UP command.
   * A second POWER_UP will take place if a patch needs to be applied.
   * The PART_INFO command is used to determine if this is a 4464 or 4463.
   */

  const uint8_t init_command[] = {Si446x_POWER_UP, 0x01,
                                  (Si446x_CLK_TCXO_EN & 0x1),
                                  x3, x2, x1, x0};
  /*
   * Use of writeBoot() enables SPI write without using OS delays.
   * The Si446x GPIO1 is set to CTS at start up.
   *
   * The Si446x SDO pin is configured to SDO data by the POWER_UP command.
   */
  (void)Si446x_writeBoot(radio, init_command, sizeof(init_command));

  /*
   * Next get the PART_INFO.
   * Store details for reference.
   * If the part requires a patch then reset and delay (TBD).
   * Output the patch and re-execute the POWER_UP command.
   */
  si446x_part_t part_info;
  const uint8_t get_part[] = {Si446x_GET_PART_INFO};
  Si446x_readBoot(radio, get_part, sizeof(get_part), (uint8_t *)&part_info,
              sizeof(part_info));

  /* Save the part information and ROM revision. */
  Si446x_getData(radio)->radio_part
                           = (part_info.info[3] << 8) + part_info.info[4];
  Si446x_getData(radio)->radio_rom_rev = part_info.info[9];
  Si446x_getData(radio)->radio_patch = 0;

  handler->radio_part = (part_info.info[3] << 8) + part_info.info[4];
  handler->radio_rom_rev = part_info.info[9];

  /*
   * Check if this radio requires a patch installed.
   * TODO: Probably should be in a table...
   */
  if(is_Si4463_patch_required(handler->radio_part, handler->radio_rom_rev)) {
    /* Power cycle radio and apply patch. */
    TRACE_DEBUG("SI   > Restart radio %d to apply patch", radio);
    Si446x_radioShutdown(radio);
    chThdSleep(TIME_MS2I(10));
    if(Si446x_radioWakeUp(radio)) {
      TRACE_ERROR("SI   > Wake up of radio %d to load patch failed", radio);
      return true;
    }
    uint16_t i = 0;
    while(Si4463_Patch_Data_Array[i] != 0) {
      if (!Si446x_writeBoot(radio, &Si4463_Patch_Data_Array[i + 1],
                       Si4463_Patch_Data_Array[i]))
      i += Si4463_Patch_Data_Array[i] + 1;
      else {
        TRACE_ERROR("SI   > Error during loading of patch to radio %d", radio);
        return true;
      }
    }
    const uint8_t init_command[] = {Si446x_POWER_UP, 0x81,
                                    (Si446x_CLK_TCXO_EN & 0x1),
                                    x3, x2, x1, x0};
    if (Si446x_writeBoot(radio, init_command, sizeof(init_command))) {
      TRACE_ERROR("SI   > Restart of radio %d after patch failed", radio);
      return true;
    }
  }

  /* Get and save the patch ID from FUNC_INFO for reference. */
  si446x_func_t func_info;
  const uint8_t get_func[] = {Si446x_GET_FUNC_INFO};
  Si446x_readBoot(radio, get_func, sizeof(get_func), (uint8_t *)&func_info,
              sizeof(func_info));

  /* Save the radio patch information. */
  Si446x_getData(radio)->radio_patch
                       = (func_info.info[5] << 8) + func_info.info[6];
  handler->radio_patch = (func_info.info[5] << 8) + func_info.info[6];

  TRACE_DEBUG("SI   > Patch ID 0x%x applied to radio %d",
              Si446x_getData(radio)->radio_patch,
              radio);

  /* Set the radio GPIOs to the basic configuration. */
  Si446x_configureGPIO(radio, &(Si446x_getConfig(radio)->init).gpio);

  /* Clear interrupts any pending interrupts. */
  (void)Si446x_clearInterruptStatus(radio);

  /* If Si446x is using its own xtal set the trim capacitor value. */
  #if !Si446x_CLK_TCXO_EN
  Si446x_setProperty8(radio, Si446x_GLOBAL_XO_TUNE, Si446x_XO_TUNE);
  #endif

  /* Fast response registers - not used at this time. */
  Si446x_setProperty8(radio, Si446x_FRR_CTL_A_MODE, 0x00);
  Si446x_setProperty8(radio, Si446x_FRR_CTL_B_MODE, 0x00);
  Si446x_setProperty8(radio, Si446x_FRR_CTL_C_MODE, 0x00);
  Si446x_setProperty8(radio, Si446x_FRR_CTL_D_MODE, 0x00);

  /* Disable interrupts globally. NIRQ pin is used for CCA. */
  Si446x_setProperty8(radio, Si446x_INT_CTL_ENABLE, 0x00);

  /* Set combined FIFO mode = 0x70. */
  Si446x_setProperty8(radio, Si446x_GLOBAL_CONFIG, 0x70);

  /* Clear TX & RX FIFO. */
  const uint8_t reset_fifo[] = {Si446x_FIFO_INFO, 0x03};
  (void)Si446x_write(radio, reset_fifo, sizeof(reset_fifo));
  /* No need to unset bits... see si docs. */

  /*
   * TODO: Move the TX and RX settings out into the respective functions.
   * This would split up into AFSK and FSK for RX & TX.
   * Leave only common setup and init here.
   */
  Si446x_setProperty8(radio, Si446x_PREAMBLE_TX_LENGTH, 0x00);
  Si446x_setProperty8(radio, Si446x_SYNC_CONFIG, 0x80);

  /* 32K clock disabled. Divided clock disabled. */
  Si446x_setProperty8(radio, Si446x_GLOBAL_CLK_CFG, 0x00);

  /* TODO: This setting would move to 2FSK RX. */
  Si446x_setProperty8(radio, Si446x_PREAMBLE_CONFIG_STD_1, 0x14);

  /* Bit polarity and mapping. */
  Si446x_setProperty8(radio, Si446x_MODEM_MAP_CONTROL, 0x00);

  /* Delta Sigma modulation control for PLL synthesizer. */
  Si446x_setProperty8(radio, Si446x_MODEM_DSM_CTRL, 0x07);

  /* Ramp down after TX final symbol. */
  Si446x_setProperty8(radio, Si446x_MODEM_TX_RAMP_DELAY, 0x01);

  /* PA ramp timing and modulation delay. */
  Si446x_setProperty8(radio, Si446x_PA_TC, 0x3D);

  /* Synthesizer PLL settings. */

  /* Antenna settings. */
  Si446x_setProperty8(radio, Si446x_MODEM_ANT_DIV_MODE, 0x01);
  Si446x_setProperty8(radio, Si446x_MODEM_ANT_DIV_CONTROL, 0x80);

  /* RSSI value compensation. */
    Si446x_setProperty8(radio, Si446x_MODEM_RSSI_COMP,
                        Si446x_MODEM_RSSI_COMP_VALUE);

  /*
   * TODO: Preamble configuration should be set in each mode.
   * Will be needed for RX FSK mode.
   * For now it is not relevant since:
   * - we don't have RX FSK implemented yet.
   * - RX AFSK preamble is decoded in the MCU DSP chain.
   * - TX AFSK encodes its own preamble and then upsamples the entire packet.
   * - TX 2FSK also encodes its own preamble which is sent as data by the PH.
   */
  Si446x_setProperty8(radio, Si446x_PREAMBLE_CONFIG, 0x21);

  /* Measure the chip temperature and save initial measurement. */
  Si446x_getTemperature(radio);
  handler->radio_init = true;

  /* Leave the radio in standby state. */
  Si446x_radioStandby(radio);
  return false;
}

/**
 * Intialize radio if it has been shutdown.
 *
 * NOTE: RADIO_CS and RADIO_SDN pins are configured in board.h
 * RADIO_SDN is configured to open drain pullup.
 * It is also pulled up on PCB by 100K.
 * The radio powers up in SDN mode.
 * CS is set as push-pull and initialized to HIGH.
 *
 * returns false on success, true on fail.
 */
bool Si446x_conditional_init(const radio_unit_t radio) {
  packet_svc_t *handler = pktGetServiceObject(radio);
  bool r = false;
  if(!handler->radio_init) {
    r = Si446x_init(radio);
  }
  /* Put the radio into ready state. This will terminate any other
     state that was active on the radio. */
  Si446x_setReadyState(radio);
  return r;
}

/*
 * Set radio frequency control parameters.
 * Values are calculated using the current XO frequency.
 * The base frequency and step size are set using current XO.
 * TX frequency and deviation are set using current XO.
 * RX IF frequency is set using current.
 * This is supplied by the TCXO service via Si446x_updateClock().
 *
 * This function also collects the chip temperature data at the moment.
 * TODO: Move temperature reading to???
 */
static bool Si446x_setSynthParameters(const radio_unit_t radio,
                              radio_freq_hz_t freq,
                              radio_chan_hz_t step,
                              radio_dev_hz_t dev) {

  /* Check frequency is in range of chip. */
  if(freq < 144000000UL || freq > 900000000UL)
    return false;


  /* Set the output divider as recommended in Si446x data sheet. */
  uint8_t outdiv = 0;
  uint8_t band = 0;
  if(freq < 705000000UL) {outdiv = 6;  band = 1;}
  if(freq < 525000000UL) {outdiv = 8;  band = 2;}
  if(freq < 353000000UL) {outdiv = 12; band = 3;}
  if(freq < 239000000UL) {outdiv = 16; band = 4;}
  if(freq < 177000000UL) {outdiv = 24; band = 5;}

  /*
   * Set the PLL band parameters.
   * Select high performance PLL.
   */

  /* Use performance PLL. */
#define Si446x_USE_HI_PLL   TRUE
#define Si446x_PRESCALER    (Si446x_USE_HI_PLL ? 2 : 4)
#define Si446x_PLL_MODE     (Si446x_USE_HI_PLL ? 0x08 : 0x00)
#define Si446x_2_19_SCALE   524288.0
#define Si446x_IF_OFFSET    64
#define Si446x_IF_SCALE     1
#define Si446x_WSIZE        0x20

  uint8_t set_band_property_command[] = {Si446x_SET_PROPERTY,
                                         0x20, 0x01, 0x51,
                                         (Si446x_PLL_MODE | band)};
  (void)Si446x_write(radio, set_band_property_command,
		  sizeof(set_band_property_command));

  /*
   *  Calculate the frequency control parameters...
   *  1. Base frequency integer and fractional parts.
   *  2. Channel stepping converted to PLL shift factor
   *  3. Deviation converted to PLL shift factor.
   *  4. RX IF
   */

  radio_clock_t si_clock = Si446x_getData(radio)->radio_clock;
  uint32_t pll_freq = Si446x_PRESCALER * si_clock;
  uint32_t pll_out = pll_freq / outdiv;

  /* Calculate integer part. */
  uint32_t n = ((uint32_t)(freq / pll_out)) - 1;
  float32_t ratio = (float32_t)freq / (float32_t)pll_out;
  float32_t rem  = ratio - (float32_t)n;

  /* Calculate fractional part. */
  uint32_t m = (uint32_t)(rem * Si446x_2_19_SCALE);
  uint8_t m2 = (m >> 16) & 0xFF;
  uint8_t m1 = (m >>  8) & 0xFF;
  uint8_t m0 = m & 0xFF;

  /* Calculate scaling factor for PLL shifting. */
  float32_t scaled_outdiv = outdiv * Si446x_2_19_SCALE;

  /* Calculate channel step size. */
  uint16_t c = (scaled_outdiv * step) / pll_freq;
  uint8_t c1 = (c >> 8) & 0xFF;
  uint8_t c0 = c & 0xFF;

  /* Set channel base frequency int, frac and step size. */
  uint8_t set_frequency_property_command[] = {Si446x_SET_PROPERTY,
                                              0x40, 0x04, 0x00, n,
                                              m2, m1, m0, c1, c0};
  (void)Si446x_write(radio, set_frequency_property_command,
               sizeof(set_frequency_property_command));

  /* Calculate and set RX for fixed IF mode (N = 1). */
  Si446x_setProperty8(radio, Si446x_MODEM_IF_CONTROL, 0x08);
  uint32_t rif = si_clock / (Si446x_IF_OFFSET * Si446x_IF_SCALE);
  int32_t i = -((scaled_outdiv * rif) / pll_freq);
  uint8_t i2 = (i >> 16) & 0x03;
  uint8_t i1 = (i >> 8) & 0xFF;
  uint8_t i0 = i & 0xFF;

  uint8_t set_modem_if[] = {Si446x_SET_PROPERTY, 0x20, 0x03, 0x01b, i2, i1, i0};
  (void)Si446x_write(radio, set_modem_if, sizeof(set_modem_if));

  /*
   * VCO calibration setting for TX and RX.
   * The 446x performs a VCO calibration whenever the frequency is changed.
   * The 446x counts the VCO cycles in a window and adjusts.
   * The TX count is centered to the specified frequency.
   * The RX count must be compensated for fixed and scaled IF mode.
   * This is due to the VCO being down shifted for IF.
   *
   * v =-(FREQ_IF_HZ*NOUTDIV/NPRESC*WSIZE/FXTAL_HZ)
   */

  /* Number of cycles to count for VCO calibration at frequency change. */
  Si446x_setProperty8(radio, Si446x_FREQ_CONTROL_W_SIZE, Si446x_WSIZE);

  /* Calculate and set the RX VCO adjustment factor. */
  int8_t v = ((rif * outdiv) / (Si446x_PRESCALER * Si446x_WSIZE)) / si_clock;
  Si446x_setProperty8(radio, Si446x_FREQ_CONTROL_VCOCNT_RX_ADJ, -v/*0xFA*/);

  /* Calculate and set TX deviation. */
  if(dev != 0) {
    /* Set TX deviation. */
    uint32_t x = (scaled_outdiv * dev) / pll_freq;
    uint8_t x2 = (x >> 16) & 0xFF;
    uint8_t x1 = (x >>  8) & 0xFF;
    uint8_t x0 = x & 0xFF;

    uint8_t set_deviation[] = {Si446x_SET_PROPERTY, 0x20, 0x03, 0x0a, x2, x1, x0};
    (void)Si446x_write(radio, set_deviation, sizeof(set_deviation));
  }

  /* TODO: Move NCO modulo and data rate setting into here? */

  /* Measure the chip temperature and update saved value.
   * TODO: Make this accessible via a RTO LLD and have collector read it.
   */
  Si446x_getTemperature(radio);
  return true;
}

static void Si446x_setPowerLevel(const radio_unit_t radio,
								 const radio_pwr_t level) {
    // Set the Power
    uint8_t set_pa_pwr_lvl_property_command[] = {Si446x_SET_PROPERTY,
                                                 0x22, 0x01, 0x01, level};
    (void)Si446x_write(radio, set_pa_pwr_lvl_property_command,
                 sizeof(set_pa_pwr_lvl_property_command));
}

/**
 * @brief  Simple receive mode for CCA carrier detection only.
 */
static void Si446x_setModemCCA_Detection(const radio_unit_t radio) {

  packet_svc_t *handler = pktGetServiceObject(radio);

/*
# BatchName Si4464
# Crys_freq(Hz): 26000000    Crys_tol(ppm): 20    IF_mode: 2
# High_perf_Ch_Fil: 1    OSRtune: 0    Ch_Fil_Bw_AFC: 0
# ANT_DIV: 0    PM_pattern: 15
# MOD_type: 2    Rsymb(sps): 1200    Fdev(Hz): 500    RXBW(Hz): 150000
# Manchester: 0    AFC_en: 0    Rsymb_error: 0.0    Chip-Version: 3
# RF Freq.(MHz): 144    API_TC: 29    fhst: 12500    inputBW: 0
# BERT: 0    RAW_dout: 0    D_source: 1    Hi_pfm_div: 1
#
# RX IF frequency is  -406250 Hz
# WB filter 2 (BW =  14.89 kHz);  NB-filter 2 (BW = 14.89 kHz)
#
# Modulation index: 0.833
*/

  /* Configure radio GPIOs. */
  Si446x_configureGPIO(radio, &(Si446x_getConfig(radio)->rafsk).gpio);

  /* Set DIRECT_MODE (asynchronous mode as 2FSK). */
  Si446x_setProperty8(radio, Si446x_MODEM_MOD_TYPE, 0x0A);

  /* Packet handler disabled in RX. */
  Si446x_setProperty8(radio, Si446x_PKT_CONFIG1, 0x41);

  if(is_part_Si4463(handler->radio_part))
    /* Run 4463 in 4464 compatibility mode (set SEARCH2 to zero). */
    Si446x_setProperty8(radio, Si446x_MODEM_RAW_SEARCH2, 0x00);
  Si446x_setProperty8(radio, Si446x_MODEM_RAW_CONTROL, 0x8F);
  Si446x_setProperty8(radio, Si446x_MODEM_RAW_SEARCH, 0xD6);
  if(is_part_Si4463(handler->radio_part))
#if Si446x_4463_USE_446X_COMPATABILITY == TRUE
    Si446x_setProperty16(radio, Si446x_MODEM_RAW_EYE, 0x00, 0x3B);
#else
    Si446x_setProperty16(radio, Si446x_MODEM_RAW_EYE, 0x00, 0x76);
#endif
  else
    Si446x_setProperty16(radio, Si446x_MODEM_RAW_EYE, 0x00, 0x3B);

  /*
   * OOK_MISC settings include parameters related to asynchronous mode.
   * Asynchronous mode is used for AFSK reception passed to DSP decode.
   *
   * SQUELCH[1:0] = 1 (don't toggle RX data if no signal received).
   * OOK_LIMIT_DISCG[5] Configures if the peak detector discharge is limited.
   *  Set 1 the peak detector discharge is disabled when the detected peak is lower than the input signal for low input levels.
   *  Versus 0 which sets peak detector discharges always.
   */
  Si446x_setProperty8(radio, Si446x_MODEM_OOK_CNT1, 0x85);
  Si446x_setProperty8(radio, Si446x_MODEM_OOK_PDTC, 0x2A);
  if(is_part_Si4463(handler->radio_part))
#if Si446x_4463_USE_446X_COMPATABILITY == TRUE
    Si446x_setProperty8(radio, Si446x_MODEM_OOK_MISC, 0x03);
#else
    Si446x_setProperty8(radio, Si446x_MODEM_OOK_MISC, 0x23);
#endif
  else
    Si446x_setProperty8(radio, Si446x_MODEM_OOK_MISC, 0x03);


  /* RX AFC control. */
  Si446x_setProperty8(radio, Si446x_MODEM_AFC_GEAR, 0x54);
  Si446x_setProperty8(radio, Si446x_MODEM_AFC_WAIT, 0x36);
  Si446x_setProperty16(radio, Si446x_MODEM_AFC_GAIN, 0x80, 0xAB);
  Si446x_setProperty16(radio, Si446x_MODEM_AFC_LIMITER, 0x02, 0x50);
  Si446x_setProperty8(radio, Si446x_MODEM_AFC_MISC, 0xC0); // 0x80

  /* RX AGC control. */
  Si446x_setProperty8(radio, Si446x_MODEM_AGC_CONTROL, 0xE0); // 0xE2 (bit 1 not used in 4464. It is used in 4463.)
  Si446x_setProperty8(radio, Si446x_MODEM_AGC_WINDOW_SIZE, 0x11);
  Si446x_setProperty8(radio, Si446x_MODEM_AGC_RFPD_DECAY, 0x63);
  Si446x_setProperty8(radio, Si446x_MODEM_AGC_IFPD_DECAY, 0x63);

  /* RX Bit clock recovery control. */
  Si446x_setProperty8(radio, Si446x_MODEM_MDM_CTRL, 0x80);
  Si446x_setProperty16(radio, Si446x_MODEM_BCR_OSR, 0x01, 0xC3);
  Si446x_setProperty24(radio, Si446x_MODEM_BCR_NCO_OFFSET, 0x01, 0x22, 0x60);
  Si446x_setProperty16(radio, Si446x_MODEM_BCR_GAIN, 0x00, 0x91);
  Si446x_setProperty8(radio, Si446x_MODEM_BCR_GEAR, 0x00);
  Si446x_setProperty8(radio, Si446x_MODEM_BCR_MISC1, 0xC2);

  /* RX IF controls. */
  Si446x_setProperty8(radio, Si446x_MODEM_IF_CONTROL, 0x08);
  Si446x_setProperty24(radio, Si446x_MODEM_IF_FREQ, 0x02, 0x80, 0x00);

  /* RX IF filter decimation controls. */
  Si446x_setProperty8(radio, Si446x_MODEM_DECIMATION_CFG1, 0x70);
  Si446x_setProperty8(radio, Si446x_MODEM_DECIMATION_CFG0, 0x10);
  if(is_part_Si4463(handler->radio_part))
#if Si446x_4463_USE_446X_COMPATABILITY == TRUE
    Si446x_setProperty8(radio, Si446x_MODEM_DECIMATION_CFG2, 0x00);
#else
    Si446x_setProperty8(radio, Si446x_MODEM_DECIMATION_CFG2, 0x0C);
#endif

  /* RSSI latching disabled. */
  Si446x_setProperty8(radio, Si446x_MODEM_RSSI_CONTROL, 0x00);

  /*
   *  RX IF filter coefficients.
   *  TODO: Add an RX filter set function.
   */
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE13_7_0, 0xFF);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE12_7_0, 0xC4);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE11_7_0, 0x30);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE10_7_0, 0x7F);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE9_7_0, 0x5F);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE8_7_0, 0xB5);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE7_7_0, 0xB8);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE6_7_0, 0xDE);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE5_7_0, 0x05);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE4_7_0, 0x17);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE3_7_0, 0x16);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE2_7_0, 0x0C);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE1_7_0, 0x03);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE0_7_0, 0x00);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COEM0, 0x15);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COEM1, 0xFF);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COEM2, 0x00);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COEM3, 0x00);

  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE13_7_0, 0xFF);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE12_7_0, 0xC4);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE11_7_0, 0x30);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE10_7_0, 0x7F);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE9_7_0, 0x5F);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE8_7_0, 0xB5);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE7_7_0, 0xB8);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE6_7_0, 0xDE);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE5_7_0, 0x05);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE4_7_0, 0x17);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE3_7_0, 0x16);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE2_7_0, 0x0C);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE1_7_0, 0x03);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE0_7_0, 0x00);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COEM0, 0x15);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COEM1, 0xFF);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COEM2, 0x00);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COEM3, 0x00);

  Si446x_setProperty8(radio, Si446x_PREAMBLE_CONFIG, 0x21);

  /* Unused Si4463 features for AFSK RX. */
  if(is_part_Si4463(handler->radio_part)) {
   /* DSA is not enabled. */
   Si446x_setProperty8(radio, Si446x_MODEM_DSA_CTRL1, 0x00); // 0xA0
   Si446x_setProperty8(radio, Si446x_MODEM_DSA_CTRL2, 0x00); // 0x04
   Si446x_setProperty8(radio, Si446x_MODEM_SPIKE_DET, 0x00); // 0x03
   Si446x_setProperty8(radio, Si446x_MODEM_ONE_SHOT_AFC, 0x00); // 0x07
   Si446x_setProperty8(radio, Si446x_MODEM_DSA_QUAL, 0x00); // 0x06
   Si446x_setProperty8(radio, Si446x_MODEM_DSA_RSSI, 0x00); // 0x78
   Si446x_setProperty8(radio, Si446x_MODEM_RSSI_MUTE, 0x00);
   Si446x_setProperty8(radio, Si446x_MODEM_DSA_MISC, 0x00); // 0x20
  }
}

/*
 *  Radio modulation settings
 */

static void Si446x_setModemAFSK_TX(const radio_unit_t radio) {

  /* Configure radio GPIOs. */
  Si446x_configureGPIO(radio, &(Si446x_getConfig(radio)->tafsk).gpio);

  /*
  * Set up GPIO port where the NIRQ from the radio is connected.
  * The NIRQ line is configured in the radio to output the CCA condition.
  * TODO: Cater for situation where CCA is not defined in the radio config.
  */
  pktSetGPIOlineMode(*Si446x_getConfig(radio)->tafsk.cca.line,
                     Si446x_getConfig(radio)->tafsk.cca.mode);

  /* Set receiver for CCA detection. */
  //Si446x_setModemCCAdetection(radio);

  // Setup the NCO modulo and oversampling mode
  radio_clock_t si_clock = Si446x_getData(radio)->radio_clock;
  uint32_t s = si_clock / 10;
  uint8_t f3 = (s >> 24) & 0xFF;
  uint8_t f2 = (s >> 16) & 0xFF;
  uint8_t f1 = (s >>  8) & 0xFF;
  uint8_t f0 = (s >>  0) & 0xFF;
  Si446x_setProperty32(radio, Si446x_MODEM_TX_NCO_MODE, f3, f2, f1, f0);

  // Setup the NCO data rate for APRS
  Si446x_setProperty24(radio, Si446x_MODEM_DATA_RATE, 0x00, 0x33, 0x90);

  /* Use 2FSK mode in conjunction with up-sampled AFSK from FIFO (PH). */
  Si446x_setProperty8(radio, Si446x_MODEM_MOD_TYPE, 0x02);

  /* Set PH bit order for AFSK. */
  Si446x_setProperty8(radio, Si446x_PKT_CONFIG1, 0x01);

  /*
   * Set AFSK filter.
   * TODO: Add set TX filter function.
   */
  const uint8_t coeff[] = {0x81, 0x9f, 0xc4, 0xee, 0x18, 0x3e, 0x5c, 0x70, 0x76};
  uint8_t i;
  for(i = 0; i < sizeof(coeff); i++) {
    si446x_args_t data[] = {0x11, 0x20, 0x01, 0x17-i, coeff[i]};
    (void)Si446x_write(radio, data, 5);
  }
}

/**
 *
 */
static void Si446x_setModemAFSK_RX(const radio_unit_t radio) {

  packet_svc_t *handler = pktGetServiceObject(radio);

/* Si446x_USE_AFSK_LCM_DATA_RATE == TRUE
#BatchName Si4464
#Crys_freq(Hz): 26000000    Crys_tol(ppm): 20    IF_mode: 2    High_perf_Ch_Fil: 1    OSRtune: 0    Ch_Fil_Bw_AFC: 0    ANT_DIV: 0    PM_pattern: 15
#MOD_type: 2    Rsymb(sps): 26400    Fdev(Hz): 2000    RXBW(Hz): 8400    Manchester: 0    AFC_en: 0    Rsymb_error: 0.0    Chip-Version: 3
#RF Freq.(MHz): 144    API_TC: 29    fhst: 12500    inputBW: 1    BERT: 0    RAW_dout: 0    D_source: 1    Hi_pfm_div: 1
#
# RX IF frequency is  -406250 Hz
# WB filter 15 (BW =  14.99 kHz);  NB-filter 15 (BW = 14.99 kHz)
#
# Modulation index: 0.152
 */

/* Si446x_USE_AFSK_LCM_DATA_RATE != TRUE && Si446x_USE_NB_RECEIVE_FILTER == TRUE
#BatchName Si4464
#Crys_freq(Hz): 26000000    Crys_tol(ppm): 20    IF_mode: 2    High_perf_Ch_Fil: 1    OSRtune: 0    Ch_Fil_Bw_AFC: 0    ANT_DIV: 0    PM_pattern: 15
#MOD_type: 2    Rsymb(sps): 1200    Fdev(Hz): 2000    RXBW(Hz): 8400    Manchester: 0    AFC_en: 0    Rsymb_error: 0.0    Chip-Version: 3
#RF Freq.(MHz): 144    API_TC: 29    fhst: 12500    inputBW: 1    BERT: 0    RAW_dout: 0    D_source: 1    Hi_pfm_div: 1
#
# RX IF frequency is  -406250 Hz
# WB filter 1 (BW =   8.27 kHz);  NB-filter 1 (BW = 8.27 kHz)
#
# Modulation index: 3.333
 */

/* Si446x_USE_AFSK_LCM_DATA_RATE != TRUE && Si446x_USE_NB_RECEIVE_FILTER != TRUE
#BatchName Si4464
#Crys_freq(Hz): 26000000    Crys_tol(ppm): 20    IF_mode: 2    High_perf_Ch_Fil: 1    OSRtune: 0    Ch_Fil_Bw_AFC: 0    ANT_DIV: 0    PM_pattern: 15
#MOD_type: 2    Rsymb(sps): 1200    Fdev(Hz): 2000    RXBW(Hz): 8400    Manchester: 0    AFC_en: 0    Rsymb_error: 0.0    Chip-Version: 3
#RF Freq.(MHz): 144    API_TC: 29    fhst: 12500    inputBW: 0    BERT: 0    RAW_dout: 0    D_source: 1    Hi_pfm_div: 1
#
# RX IF frequency is  -406250 Hz
# WB filter 1 (BW =  16.53 kHz);  NB-filter 1 (BW = 16.53 kHz)
#
# Modulation index: 3.333
*/
  /* Configure radio GPIOs. */
  Si446x_configureGPIO(radio, &(Si446x_getConfig(radio)->rafsk).gpio);

  /* Set DIRECT_MODE (asynchronous mode as 2FSK). */
  Si446x_setProperty8(radio, Si446x_MODEM_MOD_TYPE, 0x0A);

  /* Packet handler disabled in RX. */
  Si446x_setProperty8(radio, Si446x_PKT_CONFIG1, 0x40);

  if(is_part_Si4463(handler->radio_part))
    /* To run 4463 in 4464 compatibility mode (set SEARCH2 to zero). */
    /* 0xBC (SCH_FROZEN = 1 {Freeze min-max on gear switch, SCHPRD_HI = 6 SCHPRD_LO = 4 {SEARCH_4TB, SEARCH_8TB}) */
#if Si446x_4463_USE_446X_COMPATABILITY == TRUE
    Si446x_setProperty8(radio, Si446x_MODEM_RAW_SEARCH2, 0x00);
#else
    Si446x_setProperty8(radio, Si446x_MODEM_RAW_SEARCH2, 0xBC);
#endif

  /*
   * MODEM_RAW_CONTROL
   *  UNSTDPK[7] = 1 (raw mode for non-standard packet reception)
   *  CONSCHK_BYP[6] = 1 (don't stop mean value being updated by consecutive 1s or 0s)
   *   This is a change from the WDS output.
   * PM_PATTERN[3:2] = 3 preamble pattern is random
   * RAWGAIN[0] = 3  gain is 1
   */
  Si446x_setProperty8(radio, Si446x_MODEM_RAW_CONTROL, 0xCF);

  /*
   * Note: When MODEM_RAW_SEARCH2 is non zero MODEM_RAW_SEARCH settings are ignored.
   * SCH_FRZEN [7]  = 1 Freeze MA or min-max (mean) slicing on gear switch
   * SCH_FRZTH [6:5] = 5 consecutive search periods within frequency error threshold enabling gear switch
   * SCHPRD_HI[3:2] = 1 MA/min-max search period window length in bits for slicing threshold prior to gear switch
   * SCHPRD_LO[1:0] = 2 MA/min-max search period window length in bits for slicing threshold after gear switch
  */
  Si446x_setProperty8(radio, Si446x_MODEM_RAW_SEARCH, 0xD6);
  /* MODEM_RAW_EYE[0,10:8][1,7:0] */
#if Si446x_USE_AFSK_LCM_DATA_RATE == TRUE
  Si446x_setProperty16(radio, Si446x_MODEM_RAW_EYE, 0x00, 0x3B);
#else
#if Si446x_USE_NB_RECEIVE_FILTER == TRUE
  Si446x_setProperty16(radio, Si446x_MODEM_RAW_EYE, 0x01, 0x8F);
#else
  Si446x_setProperty16(radio, Si446x_MODEM_RAW_EYE, 0x00, 0xC8);
#endif
#endif
  /*
   * OOK_MISC settings include parameters related to asynchronous mode.
   * Asynchronous mode is used for AFSK reception passed to DSP decode.
   * ATTACK[6:4]
   * DECAY[3:0]
   */
#if Si446x_USE_AFSK_LCM_DATA_RATE == TRUE
  Si446x_setProperty8(radio, Si446x_MODEM_OOK_PDTC, 0x28);
#else
#if Si446x_USE_NB_RECEIVE_FILTER == TRUE
  Si446x_setProperty8(radio, Si446x_MODEM_OOK_PDTC, 0x29);
#else
  Si446x_setProperty8(radio, Si446x_MODEM_OOK_PDTC, 0x2A);
#endif
#endif
  /*
   * SQUELCH[1:0] = 1 When no signal is received, there is no toggling of RX data output.
   * This is a change from the WDS output.
   */
  Si446x_setProperty8(radio, Si446x_MODEM_OOK_CNT1, 0x85);
  /*
   * OOK_MISC settings include parameters related to asynchronous mode.
   * Asynchronous mode is used for AFSK reception passed to DSP decode.
   *
   * SQUELCH[1:0] = 1 (don't toggle RX data if no signal received).
   * OOK_LIMIT_DISCG[5] Configures if the peak detector discharge is limited.
   *  Set 1 the peak detector discharge is disabled when the detected peak is lower than the input signal for low input levels.
   *  Versus 0 which sets peak detector discharges always.
   */
  if(is_part_Si4463(handler->radio_part))
  #if Si446x_4463_USE_446X_COMPATABILITY == TRUE
    Si446x_setProperty8(radio, Si446x_MODEM_OOK_MISC, 0x03);
  #else
    Si446x_setProperty8(radio, Si446x_MODEM_OOK_MISC, 0x23);
  #endif
  else
    Si446x_setProperty8(radio, Si446x_MODEM_OOK_MISC, 0x03);

  /* RX AFC controls. */

  /*
   * AFC_GEAR
   *  GEAR_SW[7:6]  = ENUM_1 (Sync word detection - switch gears after detection of Sync Word - not active.)
   *  AFC_FAST[5:3] = 2 (higher gain results in slower AFC tracking)
   *  AFC_SLOW[2:0] = 4 (higher gain results in slower AFC tracking)
   */
  Si446x_setProperty8(radio, Si446x_MODEM_AFC_GEAR, 0x54);

  /*
   * AFC_WAIT
   *  SHWAIT[7:4] This specifies the wait period per PLL AFC correction cycle before gear switching has occurred.
   *  LGWAIT [3:0] This specifies the wait period per PLL AFC correction cycle after gear switching has occurred.
   */
#if Si446x_USE_AFSK_LCM_DATA_RATE == TRUE
  Si446x_setProperty8(radio, Si446x_MODEM_AFC_WAIT, 0x36);
#else
#if Si446x_USE_NB_RECEIVE_FILTER == TRUE
  Si446x_setProperty8(radio, Si446x_MODEM_AFC_WAIT, 0x36);
#else
  Si446x_setProperty8(radio, Si446x_MODEM_AFC_WAIT, 0x36);
#endif
#endif
  /*
   * AFC_GAIN
   *  ENAFC[0,7] Set to enable frequency error estimation and correction.
   *  AFCBD[0,6] Set to enable adaptive RX bandwidth (RX1_COEFF & RX2_COEFF)
   *  MODEM_AFC_GAIN[0,12:8 1,7:0] base gain scaled by AFC_GEAR (AFC_FAST & AFC_SLOW)
   */
#if Si446x_USE_AFSK_LCM_DATA_RATE == TRUE
  Si446x_setProperty16(radio, Si446x_MODEM_AFC_GAIN, 0x82, 0xAA);
#else
#if Si446x_USE_NB_RECEIVE_FILTER == TRUE
  Si446x_setProperty16(radio, Si446x_MODEM_AFC_GAIN, 0x80, 0x55);
#else
  Si446x_setProperty16(radio, Si446x_MODEM_AFC_GAIN, 0x80, 0xAB);
#endif
#endif
  /*
   * MODEM_AFC_LIMITER
   *  TODO: Ask Si about the calculation of AFC limiter values.
   */
#if Si446x_USE_AFSK_LCM_DATA_RATE == TRUE
  Si446x_setProperty16(radio, Si446x_MODEM_AFC_LIMITER, 0x00, 0x95);
#else
#if Si446x_USE_NB_RECEIVE_FILTER == TRUE
  Si446x_setProperty16(radio, Si446x_MODEM_AFC_LIMITER, 0x01, 0xF2);
#else
  Si446x_setProperty16(radio, Si446x_MODEM_AFC_LIMITER, 0x01, 0xE6);
#endif
#endif

  /*
   * ENAFCFRZ[7] = 1 gear switched but we don't switch,
   * NON_FRZEN[1] = 0 -> 1 (always enabled regardless of 1,0 string)
   */
  Si446x_setProperty8(radio, Si446x_MODEM_AFC_MISC, 0x82);

  /* RX AGC control.
   * AGCOVPKT[7] Selects whether the AGC operates over the entire packet, or only during acquisition of the Preamble.
   *  Set to 1 so AGC operates over entire packet (but in RAW mode the radio has no concept of a packet).
   * IFPDSLOW[6] Controls AGC for gain decreases of the IF PGA and LNA derived from peak detectors.
   *  The IFPDSLOW bit controls the step size of gain reductions to the PGA, and thus affects the slope or speed of the AGC attack time.
   *  Set to 1 so the IF programmable gain loop will always perform gain decreases in -3 dB steps versus -3 db initial and then -6 dB steps.
   * RFPDSLOW[5] Controls AGC for gain decreases of the RF PGA and LNA derived from peak detectors.
   *  The RFPDSLOW bit controls the step size of gain reductions to the PGA, and thus affects the slope or speed of the AGC attack time.
   *  Set to 1 so the IF programmable gain loop will always perform gain decreases in -3 dB steps versus -3 db initial and then -6 dB steps.
   * SGI_N[4] Selects whether gain increases are allowed due to selection of a weak antenna during the Antenna Diversity algorithm.
   *  This bit is effective only in ANT-DIV mode, and only during acquisition of the Preamble (when the signal strength of the two antennas is being evaluated).
   * AGC_SLOW[3] In the event that the AGC cycle speed cannot be configured to a sufficiently low value by MODEM_AGC_WINDOW_SIZE, the AGC speed may be reduced by an additional factor of 8x by setting the AGC_SLOW bit.
   * AGC_GAIN_CORR_EN[1] Selects whether the input gain of the RX A/D Converter is reduced when the condition of minimum AGC gain is detected.
   *  Set to 0 so ADC input gain is not reduced by 6 dB, when the condition of minimum AGC gain is detected.
   * RST_PKDT_PERIOD[0] Selects the period at which the IF and RF peak detectors are reset.
   *  Set to 0 so he peak detectors are reset only when a change in gain is indicated by the peak detector output versus being reset on each and every cycle of the AGC algorithm.
   * 0xE2 -> xE0 (ADC_GAIN_CORR_EN[1] exists in 4463 but not 4464.)
   *  When set the ADC input gain is reduced by 6 dB, when the condition of minimum AGC gain is detected.
   */
  if(is_part_Si4463(handler->radio_part))
#if Si446x_4463_USE_446X_COMPATABILITY == TRUE
    Si446x_setProperty8(radio, Si446x_MODEM_AGC_CONTROL, 0xE0);
#else
    Si446x_setProperty8(radio, Si446x_MODEM_AGC_CONTROL, 0xE2);
#endif
  else
    Si446x_setProperty8(radio, Si446x_MODEM_AGC_CONTROL, 0xE0);

  Si446x_setProperty8(radio, Si446x_MODEM_AGC_WINDOW_SIZE, 0x11);
#if Si446x_USE_AFSK_LCM_DATA_RATE == TRUE
  Si446x_setProperty8(radio, Si446x_MODEM_AGC_RFPD_DECAY, 0x12);
  Si446x_setProperty8(radio, Si446x_MODEM_AGC_IFPD_DECAY, 0x12);
#else
#if Si446x_USE_NB_RECEIVE_FILTER == TRUE
  Si446x_setProperty8(radio, Si446x_MODEM_AGC_RFPD_DECAY, 0x31);
  Si446x_setProperty8(radio, Si446x_MODEM_AGC_IFPD_DECAY, 0x31);
#else
  Si446x_setProperty8(radio, Si446x_MODEM_AGC_RFPD_DECAY, 0x63);
  Si446x_setProperty8(radio, Si446x_MODEM_AGC_IFPD_DECAY, 0x63);
#endif
#endif

  /* RX Bit clock recovery control. */
  Si446x_setProperty8(radio, Si446x_MODEM_MDM_CTRL, 0x80);
#if Si446x_USE_AFSK_LCM_DATA_RATE == TRUE
  Si446x_setProperty16(radio, Si446x_MODEM_BCR_OSR, 0x00, 0x52);
  Si446x_setProperty24(radio, Si446x_MODEM_BCR_NCO_OFFSET, 0x06, 0x3D, 0x10);
  Si446x_setProperty16(radio, Si446x_MODEM_BCR_GAIN, 0x03, 0x1F);
#else
#if Si446x_USE_NB_RECEIVE_FILTER == TRUE
  Si446x_setProperty16(radio, Si446x_MODEM_BCR_OSR, 0x00, 0xE2);
  Si446x_setProperty24(radio, Si446x_MODEM_BCR_NCO_OFFSET, 0x02, 0x44, 0xC0);
  Si446x_setProperty16(radio, Si446x_MODEM_BCR_GAIN, 0x01, 0x22);
#else
  Si446x_setProperty16(radio, Si446x_MODEM_BCR_OSR, 0x01, 0xC3);
  Si446x_setProperty24(radio, Si446x_MODEM_BCR_NCO_OFFSET, 0x01, 0x22, 0x60);
  Si446x_setProperty16(radio, Si446x_MODEM_BCR_GAIN, 0x00, 0x91);
#endif
#endif
  Si446x_setProperty8(radio, Si446x_MODEM_BCR_GEAR, 0x00);

  /*
   * BCR_MISC1
   *  BCRFBBYP[7] = 1 Feedback of the compensation term to the BCR tracking loop is bypassed (disabled).
   *  SLICEFBBYP[6] = 1 Feedback of the compensation term to the slicer is bypassed (disabled).
   *  RXNCOCOMP[4] = 0  Compensation of the BCR NCO frequency is disabled (normal operation).
   *  RXCOMP_LAT [3] = 0 BCR NCO compensation is sampled upon detection of the end of the Preamble (i.e., boundary between Preamble and Sync Word).
   *  CRGAINX2 [2] = 0 BCR loop gain is not doubled (normal operation).
   *  DIS_MIDPT[1] = 1 Correction of a BCR mid-point phase sampling condition by resetting the NCO is disabled.
   *
   *  BCR_MISC0
   *   default 0x00
   */
  Si446x_setProperty8(radio, Si446x_MODEM_BCR_MISC0, 0x00);
  Si446x_setProperty8(radio, Si446x_MODEM_BCR_MISC1, 0xC2);

  /* RX IF frequency controls are set in setSynthParameters() */

  /* RX IF CIC filter decimation controls. */
#if Si446x_USE_AFSK_LCM_DATA_RATE == TRUE
  Si446x_setProperty8(radio, Si446x_MODEM_DECIMATION_CFG1, 0x20);
  Si446x_setProperty8(radio, Si446x_MODEM_DECIMATION_CFG0, 0x10);
#else
#if Si446x_USE_NB_RECEIVE_FILTER == TRUE
  Si446x_setProperty8(radio, Si446x_MODEM_DECIMATION_CFG1, 0xB0);
  Si446x_setProperty8(radio, Si446x_MODEM_DECIMATION_CFG0, 0x10);
#else
  Si446x_setProperty8(radio, Si446x_MODEM_DECIMATION_CFG1, 0x70);
  Si446x_setProperty8(radio, Si446x_MODEM_DECIMATION_CFG0, 0x10);
#endif
#endif
  if(is_part_Si4463(handler->radio_part))
#if Si446x_4463_USE_446X_COMPATABILITY == TRUE
    /*
     * Si4463 revC2A has an extra decimator stage to be used for very
     * narrowband (< 1 kHz) Rx applications. For AFSK case this should
     * be bypassed to emulate the Si4464 configuration.
     * NDEC2AGC[2] = 1 enable AGC control of 2nd stage CIC
     * NDEC2GAIN[4:3] = 1 2nd stage CIC gain is 12dB
     */
    Si446x_setProperty8(radio, Si446x_MODEM_DECIMATION_CFG2, 0x00);
#else
    /*
     * NDEC2AGC[2] = 1 enable AGC control of 2nd stage CIC
     * NDEC2GAIN[4:3] = 1 2nd stage CIC gain is 12dB
     */
    Si446x_setProperty8(radio, Si446x_MODEM_DECIMATION_CFG2, 0x0C);
#endif
  /* RSSI controls. */
  /*
   * LATCH[2:0] = 0 disabled.
   * AVERAGE[4:3] = 0 average over 4*Tb
   */
  Si446x_setProperty8(radio, Si446x_MODEM_RSSI_CONTROL, 0x00);
  if(is_part_Si4463(handler->radio_part)) {
    /* RSSI jump control (ENRSSIJMP[3] 1 -> 0 to disable. */
#if Si446x_4463_USE_446X_COMPATABILITY == TRUE
    Si446x_setProperty8(radio, Si446x_MODEM_RSSI_CONTROL2, 0x00);
#else
    Si446x_setProperty8(radio, Si446x_MODEM_RSSI_CONTROL2, 0x18);
#endif
    Si446x_setProperty8(radio, Si446x_MODEM_RSSI_JUMP_THRESH, 0x06);
  }

  /* RX IF filter coefficients. */
#if Si446x_USE_AFSK_LCM_DATA_RATE == TRUE
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE13_7_0, 0xA2);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE12_7_0, 0xA0);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE11_7_0, 0x97);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE10_7_0, 0x8A);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE9_7_0, 0x79);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE8_7_0, 0x66);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE7_7_0, 0x52);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE6_7_0, 0x3F);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE5_7_0, 0x2E);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE4_7_0, 0x1F);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE3_7_0, 0x14);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE2_7_0, 0x0B);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE1_7_0, 0x06);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE0_7_0, 0x02);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COEM0, 0x00);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COEM1, 0x00);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COEM2, 0x00);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COEM3, 0x00);

  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE13_7_0, 0xA2);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE12_7_0, 0xA0);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE11_7_0, 0x97);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE10_7_0, 0x8A);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE9_7_0, 0x79);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE8_7_0, 0x66);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE7_7_0, 0x52);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE6_7_0, 0x3F);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE5_7_0, 0x2E);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE4_7_0, 0x1F);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE3_7_0, 0x14);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE2_7_0, 0x0B);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE1_7_0, 0x06);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE0_7_0, 0x02);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COEM0, 0x00);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COEM1, 0x00);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COEM2, 0x00);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COEM3, 0x00);
#else
#if Si446x_USE_NB_RECEIVE_FILTER == TRUE
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE13_7_0, 0xFF);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE12_7_0, 0xBA);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE11_7_0, 0x0F);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE10_7_0, 0x51);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE9_7_0, 0xCF);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE8_7_0, 0xA9);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE7_7_0, 0xC9);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE6_7_0, 0xFC);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE5_7_0, 0x1B);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE4_7_0, 0x1E);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE3_7_0, 0x0F);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE2_7_0, 0x01);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE1_7_0, 0xFC);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE0_7_0, 0xFD);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COEM0, 0x15);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COEM1, 0xFF);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COEM2, 0x00);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COEM3, 0x0F);

  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE13_7_0, 0xFF);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE12_7_0, 0xBA);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE11_7_0, 0x0F);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE10_7_0, 0x51);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE9_7_0, 0xCF);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE8_7_0, 0xA9);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE7_7_0, 0xC9);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE6_7_0, 0xFC);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE5_7_0, 0x1B);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE4_7_0, 0x1E);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE3_7_0, 0x0F);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE2_7_0, 0x01);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE1_7_0, 0xFC);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE0_7_0, 0xFD);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COEM0, 0x15);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COEM1, 0xFF);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COEM2, 0x00);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COEM3, 0x0F);
#else
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE13_7_0, 0xFF);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE12_7_0, 0xBA);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE11_7_0, 0x0F);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE10_7_0, 0x51);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE9_7_0, 0xCF);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE8_7_0, 0xA9);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE7_7_0, 0xC9);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE6_7_0, 0xFC);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE5_7_0, 0x1B);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE4_7_0, 0x1E);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE3_7_0, 0x0F);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE2_7_0, 0x01);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE1_7_0, 0xFC);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE0_7_0, 0xFD);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COEM0, 0x15);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COEM1, 0xFF);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COEM2, 0x00);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COEM3, 0x0F);

  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE13_7_0, 0xFF);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE12_7_0, 0xBA);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE11_7_0, 0x0F);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE10_7_0, 0x51);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE9_7_0, 0xCF);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE8_7_0, 0xA9);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE7_7_0, 0xC9);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE6_7_0, 0xFC);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE5_7_0, 0x1B);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE4_7_0, 0x1E);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE3_7_0, 0x0F);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE2_7_0, 0x01);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE1_7_0, 0xFC);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE0_7_0, 0xFD);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COEM0, 0x15);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COEM1, 0xFF);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COEM2, 0x00);
  Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COEM3, 0x0F);
#endif
#endif

  /* TODO: Not used in UNSTD mode. */
  Si446x_setProperty8(radio, Si446x_PREAMBLE_CONFIG, 0x21);

  /* Unused Si4463 features for AFSK RX. */
  if(is_part_Si4463(handler->radio_part)) {
   /* DSA[5] is not enabled. */
   Si446x_setProperty8(radio, Si446x_MODEM_DSA_CTRL1, 0x40);
   Si446x_setProperty8(radio, Si446x_MODEM_DSA_CTRL2, 0x04);
   Si446x_setProperty8(radio, Si446x_MODEM_SPIKE_DET, 0x03);
   Si446x_setProperty8(radio, Si446x_MODEM_ONE_SHOT_AFC, 0x07);
   Si446x_setProperty8(radio, Si446x_MODEM_DSA_QUAL, 0x06);
   Si446x_setProperty8(radio, Si446x_MODEM_DSA_RSSI, 0x78);
   Si446x_setProperty8(radio, Si446x_MODEM_RSSI_MUTE, 0x00);
   Si446x_setProperty8(radio, Si446x_MODEM_DSA_MISC, 0x20);
  }
}



/**
 *
 */
static void Si446x_setModem2FSK_TX(const radio_unit_t radio,
                                   const uint32_t speed) {

  /* Configure radio GPIOs. */
  Si446x_configureGPIO(radio, &(Si446x_getConfig(radio)->t2fsk).gpio);

  /*
  * Set up GPIO port where the NIRQ from the radio is connected.
  * The NIRQ line is configured in the radio to output the CCA condition.
  * TODO: Cater for situation where CCA is not defined in the radio config.
  */
  pktSetGPIOlineMode(*Si446x_getConfig(radio)->t2fsk.cca.line,
                     Si446x_getConfig(radio)->t2fsk.cca.mode);

  /* Setup the NCO modulo and oversampling mode. */
  radio_clock_t si_clock = Si446x_getData(radio)->radio_clock;
  uint32_t s = si_clock / 10;
  uint8_t f3 = (s >> 24) & 0xFF;
  uint8_t f2 = (s >> 16) & 0xFF;
  uint8_t f1 = (s >>  8) & 0xFF;
  uint8_t f0 = (s >>  0) & 0xFF;
  Si446x_setProperty32(radio, Si446x_MODEM_TX_NCO_MODE, f3, f2, f1, f0);

  /* Setup the NCO data rate for 2FSK. */
  Si446x_setProperty24(radio, Si446x_MODEM_DATA_RATE,
                       (uint8_t)(speed >> 16),
                       (uint8_t)(speed >> 8),
                       (uint8_t)speed);

  /* Use 2FSK from FIFO (PH). */
  Si446x_setProperty8(radio, Si446x_MODEM_MOD_TYPE, 0x02);

  /* Set PH bit order for 2FSK. */
  Si446x_setProperty8(radio, Si446x_PKT_CONFIG1, 0x01);

  /* No TX filtering used for 2FSK mode. */
}

/**
 *
 */
static void Si446x_setModem2FSK_RX(const radio_unit_t radio,
                                   const radio_mod_t mod) {

  packet_svc_t *handler = pktGetServiceObject(radio);

  /* Configure radio GPIOs. */
  Si446x_configureGPIO(radio, &(Si446x_getConfig(radio)->r2fsk).gpio);

  /* TODO: Everything.... */
  (void)handler;
  (void)mod;
}

/**
 * Radio Settings
 */
static uint8_t __attribute__((unused)) Si446x_getChannel(const radio_unit_t radio) {
  const uint8_t state_info[] = {Si446x_REQUEST_DEVICE_STATE};
  uint8_t rxData[4];
  (void)Si446x_read(radio, state_info, sizeof(state_info), rxData, sizeof(rxData));
  return rxData[3];
}

/*
 * Radio FIFO
 */

#if 0
static void Si446x_resetFIFO(const radio_unit_t radio) {
  /* Reset TX FIFO in case some remnant unsent data is left there. */
  const uint8_t reset_fifo[] = {0x15, 0x01};
  Si446x_write(radio, reset_fifo, 2);
}
#endif
/**
 *
 */
static bool Si446x_writeFIFO(const radio_unit_t radio,
                                 uint8_t *msg, uint8_t size) {
  uint8_t write_fifo[size + 1];
  write_fifo[0] = Si446x_WRITE_TX_FIFO;
  memcpy(&write_fifo[1], msg, size);
  return Si446x_write(radio, write_fifo, sizeof(write_fifo));
}

/**
 * Set variable with free.
 * Returns false on success, true on fail
 */
static bool Si446x_getTXfreeFIFO(const radio_unit_t radio,
                                    uint8_t reset, uint8_t *data) {
  const uint8_t fifo_info[] = {Si446x_FIFO_INFO, reset & 0x01};
  uint8_t rxData[4];
  if(Si446x_read(radio, fifo_info, sizeof(fifo_info), rxData, sizeof(rxData)))
    return true;
  *data = rxData[3];
  return false;
}

/**
 * Put radio in standby (low power) mode.
 * All registers are retained.
 */
void Si446x_radioStandby(const radio_unit_t radio) {
  TRACE_DEBUG("SI   > Set radio %d to standby state", radio);
  Si446x_setStandbyState(radio);
}

/**
 * The GPIO connected to radio SDN.
 * The GPIO is open drain output with pull-up at board initialization.
 * Thus the radio is in shutdown following board initialization.
 * Si446x GPIO1 is configured to output CTS (option 8) during POR.
 * Hence the MCU GPIO connected to radio GPIO1 can be used to check CTS.
 *
 * Radio init is performed in the radio manager thread init stage.
 * The radio GPIOs can be reconfigured after radio init is complete.
 *
 * Returns false on success, true on error.
 */
bool Si446x_radioWakeUp(const radio_unit_t radio) {
  TRACE_DEBUG("SI   > Wake up radio %i", radio);

  /* Set MCU GPIO input for POR and CTS of radio from GPIO0 and GPIO1. */
  palSetLineMode(Si446x_getConfig(radio)->gpio0, PAL_MODE_INPUT_PULLDOWN);
  palSetLineMode(Si446x_getConfig(radio)->gpio1, PAL_MODE_INPUT_PULLDOWN);

  /* First assert SDN high to ensure radio is asleep. */
  palSetLine(Si446x_getConfig(radio)->sdn);
  chThdSleep(TIME_MS2I(100));

  /* Assert SDN low to perform radio POR wakeup. */
  palClearLine(Si446x_getConfig(radio)->sdn);

  /*
   * Wait for transceiver to wake up.
   * During start up the POR state is on radio GPIO0.
   * GPIO0 goes from zero to one when POR completes.
   * The specified maximum wake-up time is 6mS.
   * The 50ms is a timeout to cover a fault condition.
   */
  uint8_t timeout = 50;
  while(--timeout && !pktReadGPIOline(Si446x_getConfig(radio)->gpio0)) {
    chThdSleep(TIME_MS2I(1));
  }
  if(timeout == 0) {
    TRACE_ERROR("SI   > Timeout waiting for POR on radio %i", radio);
    return true;
  }
  /* Return state of CTS after delay. */
  chThdSleep(TIME_MS2I(10));
  return !(pktReadGPIOline(Si446x_getConfig(radio)->gpio1) == PAL_HIGH);
}

/**
 * The radio is shutdown by setting SDN high.
 *
 * When radio is put in shutdown all registers are lost.
 */
void Si446x_radioShutdown(const radio_unit_t radio) {
  TRACE_DEBUG("SI   > Disable radio %i", radio);
  packet_svc_t *handler = pktGetServiceObject(radio);

  palSetLine(Si446x_getConfig(radio)->sdn);
  handler->radio_init = false;
  chThdSleep(TIME_MS2I(50));
}

/*
 * Radio TX/RX
 */


/**
 * @brief Used by TX to check CCA over a measurement interval.
 * @notes A maximum of 10 samples are taken in the measurement interval.
 *
 * @returns CCA count of samples taken over the measurement interval
 * @retval  Special time values are allowed in measurement period
 *          - TIME_INFINITE returns 0
 *          - TIME_IMMEDIATE returns results of one CCA measurement
 * @retval  Other measurement intervals will provide count of 1 - 10 samples
 *          This depends on system time resolution setting
 *
 */
static cnt_t Si446x_checkCCAthresholdForTX(const radio_unit_t radio,
                                           const radio_mod_t mod,
                                           const sysinterval_t interval) {

  ioline_t cca_line;
  /* Read CAA as setup by mod type. */
  switch(mod) {
  case MOD_2FSK_300:
  case MOD_2FSK_9k6:
  case MOD_2FSK_19k2:
  case MOD_2FSK_38k4:
  case MOD_2FSK_57k6:
  case MOD_2FSK_76k8:
  case MOD_2FSK_96k:
  case MOD_2FSK_115k2:
    cca_line = *Si446x_getConfig(radio)->t2fsk.cca.line;
    break;

  case MOD_AFSK: {
    cca_line = *Si446x_getConfig(radio)->tafsk.cca.line;
    break;
  }

  case MOD_CW:
    return false;

  case MOD_NONE:
    return false;
  }

  if(cca_line == PAL_NOLINE || interval == TIME_INFINITE)
    return (cnt_t)0;

  cnt_t cca = 0;

  /* Take 10 measurements over the period. */
  sysinterval_t slice = interval / 10;

  /* Measure sliced CCA instances in period. */
  for(uint16_t i = 0; i <= slice; i++) {
    cca += palReadLine(cca_line);
    /* Sleep one time slice. */
    chThdSleep(slice);
  }
  /* Return result. */
  return cca;
}
#if 0
/*
 * If radio is in RX state change to ready state and return true
 * Else return false.
 */
static bool Si446x_terminateReceiveState(const radio_unit_t radio) {
#if 0
  if(Si446x_getState(radio) == Si446x_STATE_RX) {
    Si446x_setReadyState(radio);
    while(Si446x_getState(radio) == Si446x_STATE_RX);
  } else {
    /* Radio may be in standby so get it back to ready. */
    Si446x_setReadyState(radio);
  }
#else
  if (Si446x_getState(radio) == Si446x_RX) {
    Si446x_setReadyState(radio);
    return true;
  }
  return false;
#endif
}
#endif
/*
 * Wait for radio to enter state
 * Return MSG_OK if state reached within timeout.
 * Otherwise return MSG_TIMEOUT.
 */
static msg_t Si446x_waitForState(const radio_unit_t radio,
                                 si446x_state_t state,
                                 sysinterval_t timeout) {
  if (timeout == TIME_IMMEDIATE)
    return (Si446x_getState(radio) != state) ? MSG_TIMEOUT : MSG_OK;
  if (timeout == TIME_INFINITE) {
    do {
      chThdSleep(TIME_MS2I(10));
    } while (Si446x_getState(radio) != state);
    return MSG_OK;
  }
  time_msecs_t ms = chTimeI2MS(timeout);
  while ((Si446x_getState(radio) != state) && (ms != 0)) {
    chThdSleep(TIME_MS2I(1));
    --ms;
  }
  return (ms == 0) ? MSG_TIMEOUT : MSG_OK;
}

/*
 * Return status of transmit state
 * Return MSG_OK if not transmit state or state ended before timeout
 * Otherwise return MSG_TIMEOUT.
 */
static msg_t Si446x_waitTransmitEnd(const radio_unit_t radio, sysinterval_t timeout) {
  if (timeout == TIME_IMMEDIATE)
    return (Si446x_getState(radio) == Si446x_TX) ? MSG_TIMEOUT : MSG_OK;
  if (timeout == TIME_INFINITE) {
    do {
      chThdSleep(TIME_MS2I(10));
    } while (Si446x_getState(radio) == Si446x_TX);
    return MSG_OK;
  }
  time_msecs_t ms = chTimeI2MS(timeout);
  while ((Si446x_getState(radio) == Si446x_TX) && (ms != 0)) {
    chThdSleep(TIME_MS2I(1));
    --ms;
  }
  return (ms == 0) ? MSG_TIMEOUT : MSG_OK;
}

/**
 * Wait for a clear time slot and initiate packet transmission.
 */
static bool Si446x_transmit_with_CCA(const radio_unit_t radio,
                            const radio_freq_hz_t freq,
                            const radio_chan_hz_t step,
                            const radio_ch_t chan,
                            const radio_pwr_t power,
                            const radio_mod_t mod,
                            const uint16_t size,
                            const radio_squelch_t rssi,
                            sysinterval_t cca_timeout) {

  /* Get an absolute operating frequency in Hz. */
  radio_freq_hz_t op_freq = pktComputeOperatingFrequency(radio, freq,
                                                      step, chan, RADIO_TX);

  if(op_freq == FREQ_INVALID) {
    TRACE_ERROR("SI   > Frequency out of range");
    TRACE_ERROR("SI   > abort transmission on radio %d", radio);
    return false;
  }

  mod_params_t mp;
  mp.type = mod;
  if(!pktLookupModParameters(radio, &mp)) {
    TRACE_ERROR( "SI   > Invalid modulation code on radio %d", radio);
    return false;
  }

  /* Check for ready state. */
  if (Si446x_waitForState(radio, Si446x_READY, TIME_S2I(5)) == MSG_TIMEOUT) {
    /* Force 446x into ready state. */
    Si446x_setReadyState(radio);
    TRACE_ERROR("SI   > %s timeout waiting for ready state radio %d",
                getModulation(mod), radio);
    return false;
  }

  /* Check for blind send request. */
  if(rssi != PKT_SI446X_NO_CCA_RSSI) {
    /*
     *  Listen on the TX frequency for a clear channel.
     *  - The radio is setup for CCA receive mode.
     *  - Set the RSSI threshold in the radio.
     *  - Put the radio into RX state.
     *  - Allow time for AGC to engage.
     *  - Measure the CCA status.
     */

    /* Frequency is an absolute frequency in Hz. */
    Si446x_setSynthParameters(radio, op_freq, step, 0);

    /* Set receiver for CCA detection. */
    Si446x_setModemCCA_Detection(radio);
    Si446x_setProperty8(radio, Si446x_MODEM_RSSI_THRESH, rssi);

    /* Minimum timeout for CCA is 1 second. */
    if(cca_timeout < TIME_S2I(1) || cca_timeout == TIME_INFINITE) {
      TRACE_WARN("SI   > CCA wait time on radio %d forced to 1 second."
          "invalid was specified", radio);
      cca_timeout = TIME_S2I(1);
    }

    /* Try to get clear channel. */
    TRACE_DEBUG( "SI   > Wait up to %.1f seconds on radio %d for CCA on"
        " %d.%03d MHz",
        (float32_t)(TIME_I2MS(cca_timeout) / 1000), radio,
        op_freq/1000000, (op_freq%1000000)/1000);

    /*
     * Start the receiver.
     * The si446x will not set CTS until RX has started.
     * Thus any subsequent commands will wait on CTS.
     */
    Si446x_startRXState(radio, chan);


    /* TODO: Move this into startRXState. */
    if (Si446x_waitForState(radio, Si446x_RX, TIME_MS2I(100) == MSG_TIMEOUT)) {
      TRACE_ERROR( "SI   > Radio %d did not enter CCA receive state."
                        " TX aborted.", radio);
    }
    // TODO: adjust measurement based on... link_speed_t speed = mp.tx_speed;

    /* Allow some time for AGC to engage. */
    chThdSleep(TIME_MS2I(5));

#define CCA_MEASURE_INTERVAL   TIME_MS2I(50)

    systime_t t0 = chVTGetSystemTime();
    sysinterval_t cca_time = CCA_MEASURE_INTERVAL;
    do {
      if(Si446x_checkCCAthresholdForTX(radio, mod, CCA_MEASURE_INTERVAL) == 0)
        break;
    } while(Si446x_getState(radio) == Si446x_RX
        && cca_timeout > (cca_time = chVTTimeElapsedSinceX(t0)));

    /* Clear channel timing. */
    TRACE_DEBUG( "SI   > CCA attained in %d milliseconds on radio %d",
                                         chTimeI2MS(cca_time), radio);

    /* Set radio back to ready state. */
    Si446x_setReadyState(radio);

    /* Check for ready state. */
    if (Si446x_waitForState(radio, Si446x_READY, TIME_MS2I(50)) == MSG_TIMEOUT) {
      /* Force 446x into ready state. */
      Si446x_setReadyState(radio);
      TRACE_ERROR("SI   > %s timeout waiting for ready state radio %d",
                  getModulation(mod), radio);
      return false;
    }
    //(void)Si446x_terminateReceiveState(radio);
  } /* End if CCA. */

  /* Frequency is an absolute frequency in Hz. */
  Si446x_setSynthParameters(radio, op_freq, step, mp.tx_dev);

  /* Setup TX configuration. */
  switch(mod) {
  case MOD_2FSK_300:
  case MOD_2FSK_9k6:
  case MOD_2FSK_19k2:
  case MOD_2FSK_38k4:
  case MOD_2FSK_57k6:
  case MOD_2FSK_76k8:
  case MOD_2FSK_96k:
  case MOD_2FSK_115k2:
    Si446x_setModem2FSK_TX(radio, mp.tx_speed);
    break;

  case MOD_AFSK:
    Si446x_setModemAFSK_TX(radio);
    break;

  case MOD_CW:
    Si446x_setModem2FSK_TX(radio, mp.tx_speed);
    break;

  case MOD_NONE:
  default:
    return false;
  } /* End switch on modulation. */

  // Transmit
  TRACE_INFO("SI   > Tune radio %d to %d.%03d MHz (TX)",
             radio, op_freq/1000000, (op_freq%1000000)/1000);

  /*
   * Set power level and start transmit.
   * The command does not reply with CTS until TX starts.
   */
  Si446x_setPowerLevel(radio, power);
  Si446x_startPacketTX(radio, chan, size);

  /* Wait for transmission to start. */
  if (Si446x_waitForState(radio, Si446x_TX, TIME_MS2I(100)) == MSG_TIMEOUT) {
    /* Force 446x out of state. */
    Si446x_setReadyState(radio);
    TRACE_ERROR("SI   > %s transmit failed to start on radio %d",
                getModulation(mod), radio);
    return false;
  }
  return true;
}

/*
 * Start or restore reception.
 *
 * return true if RX was enabled and/or resumed OK.
 * return false if RX was not enabled.
 */
bool Si4464_enableReceive(const radio_unit_t radio,
                          const radio_freq_hz_t rx_frequency,
                          const radio_chan_hz_t rx_step,
                          const radio_ch_t rx_chan,
                          const radio_squelch_t rx_rssi,
                          const radio_mod_t rx_mod) {

  /* Get an absolute operating frequency in Hz. */
  radio_freq_hz_t op_freq = pktComputeOperatingFrequency(radio,
                                                      rx_frequency,
                                                      rx_step,
                                                      rx_chan,
                                                      RADIO_RX);

  if(op_freq == FREQ_INVALID) {
    TRACE_ERROR("SI   > Frequency out of range");
    TRACE_ERROR("SI   > Abort reception on radio %d", radio);
    return false;
  }

  TRACE_INFO( "SI   > Enable reception %d.%03d MHz (ch %d),"
              " RSSI 0x%x, %s on radio %d",
              op_freq/1000000, (op_freq % 1000000)/1000,
              rx_chan,
              rx_rssi, getModulation(rx_mod), radio);

  /*
   *  Initialize radio before any commands.
   *  It may have been powered down or in standby.
   *  The radio will be in standby mode after initialisation.
   */
  if(Si446x_conditional_init(radio)) {
    /* Radio did not initialise. */
    TRACE_ERROR("SI   > Radio %d failed to initialise when enabling RX", radio);
    return false;
  }

  /* Wait for any TX in progress to end. */
  if(Si446x_waitTransmitEnd(radio, TIME_S2I(10))) {
    /* Force ready state. */
    Si446x_setReadyState(radio);
    TRACE_ERROR("SI   > Timeout waiting for TX state end "
                 "before starting %s receive on radio %d",
                 getModulation(rx_mod), radio);
  }

  /* Configure radio for modulation type. */
  switch(rx_mod) {
  case MOD_2FSK_300:
  case MOD_2FSK_9k6:
  case MOD_2FSK_19k2:
  case MOD_2FSK_38k4:
  case MOD_2FSK_57k6:
  case MOD_2FSK_76k8:
  case MOD_2FSK_96k:
  case MOD_2FSK_115k2:
    Si446x_setModem2FSK_RX(radio, rx_mod);
    TRACE_ERROR("SI   > Modulation type %s not supported in receive",
                getModulation(rx_mod));
    TRACE_ERROR("SI   > Abort reception on radio %d", radio);
    return false;

  case MOD_AFSK: {
    Si446x_setModemAFSK_RX(radio);
    break;
  }

  case MOD_CW:
    Si446x_setModemCCA_Detection(radio);
    break;

  case MOD_NONE:
    TRACE_ERROR("SI   > Invalid modulation type %s in receive",
                getModulation(rx_mod));
    TRACE_ERROR("SI   > Abort reception on radio %d", radio);
    return false;
  } /* End switch on mod type. */

  TRACE_INFO("SI   > Tune radio %d to %d.%03d MHz (RX)",
             radio, op_freq/1000000, (op_freq%1000000)/1000);

  /* Frequency must be an absolute frequency in Hz. */
  if(!Si446x_setSynthParameters(radio, op_freq, rx_step, 0))
    return false;

  /* Set squelch level. */
  Si446x_setProperty8(radio, Si446x_MODEM_RSSI_THRESH, rx_rssi);

  /* Clear any pending interrupts. */
  (void)Si446x_clearInterruptStatus(radio);

  /*
   * Start the receiver.
   * The si446x will not set CTS until RX has started.
   * Thus any subsequent commands will wait on CTS.
   */
  Si446x_startRXState(radio, rx_chan);

#if 0
  /* Wait for the receiver to start. */
  time_msecs_t timeout = 1000;
  while(Si446x_getState(radio) != Si446x_RX && --timeout)
      chThdSleep(TIME_MS2I(1));
  if(timeout == 0) {
    TRACE_ERROR("SI   > Failed to start receive state on radio %d", radio);
  }
  return timeout != 0;
#else
  return true;
#endif
}

/*
 * Called when a packet RX channel is closed.
 * If the receiver is active put it into standby.
 */
void Si446x_disableReceive(const radio_unit_t radio) {
  if(Si446x_getState(radio) == Si446x_RX) {
    Si446x_radioStandby(radio);
  }
}


/*
 * AFSK Transmitter functions
 */

/*
 *
 */
static uint8_t Si446x_getUpsampledNRZIbits(up_sampler_t *upsampler,
                                           uint8_t *buf) {
  uint8_t b = 0;
  for(uint8_t i = 0; i < 8; i++) {
    if(upsampler->current_sample_in_baud == 0) {
      if((upsampler->packet_pos & 7) == 0) { // Load up next byte
        upsampler->current_byte = buf[upsampler->packet_pos >> 3];
      } else { // Load up next bit
        upsampler->current_byte >>= 1;
      }
    }

    // Toggle tone (1200 <> 2200)
    upsampler->phase_delta = (upsampler->current_byte & 1)
        ? PHASE_DELTA_1200 : PHASE_DELTA_2200;
    /* Add delta-phase (position within SAMPLES_PER_BAUD). */
    upsampler->phase += upsampler->phase_delta;
    b |= ((upsampler->phase >> 16) & 1) << i;  // Set modulation bit

    if(++upsampler->current_sample_in_baud == SAMPLES_PER_BAUD) {
      upsampler->current_sample_in_baud = 0;
      upsampler->packet_pos++;
    }
  }
  return b;
}

/**
 *
 */
static void Si446x_transmitTimeoutI(thread_t *tp) {
  /* Tell the thread to terminate. */
  chSysLockFromISR();
  chEvtSignalI(tp, SI446X_EVT_TX_TIMEOUT);
  chSysUnlockFromISR();
}

/**
 * @brief Release transmit resources.
 * @notes The task object is returned to the Radio manager.
 *        The radio manager frees the task object.
 *
 * @param[in] Pointer to @p radioTask object.
 * @param[in] Reason code associated with release.
 *
 */
static void Si446x_transmitRelease(radio_task_object_t *rto, packet_t pp,
                                     msg_t msg) {

  /* Free packet object memory. */
  if (pp != NULL)
    pktReleaseBufferChain(pp);
  rto->result = msg;
  pktRadioSendComplete(rto);
  pktThdTerminateSelf();
  /* We never arrive here. */
}

/*
 * Simple AFSK send thread with minimised buffering and burst send capability.
 * Uses an iterator to size NRZI output and allocate suitable size buffer.
 *
 */
THD_FUNCTION(bloc_si_fifo_feeder_afsk, arg) {
  radio_task_object_t *rto = arg;

  radio_unit_t radio = rto->handler->radio;

  packet_t pp = rto->radio_dat.pkt.packet_out;

  chDbgAssert(pp != NULL, "no packet in radio task");

  /* Create thread name for this instance. */
  char tx_thd_name[PKT_THREAD_NAME_MAX];
  chsnprintf(tx_thd_name, sizeof(tx_thd_name),
             "%s_%03i",
             getModulation(rto->radio_dat.type),
             rto->radio_dat.seq_num);

  chRegSetThreadName(tx_thd_name);

  /* Lock radio for the entirety of the transmit session. */
  msg_t msg = pktLockRadio(radio, RADIO_TX, rto->radio_dat.timer);

  if (msg != MSG_OK) {
    TRACE_ERROR("SI   > TX reset or timeout when %s attempting radio %d acquisition",
                getModulation(rto->radio_dat.type), radio);

    Si446x_transmitRelease(rto, pp, msg);
    /* We never arrive here. */
  }

  /* Wait for any active receive stream in progress. Force inactive on timeout.
     Any stream is stopped but the 446x would remain in receive state. */
  msg = pktSetReceiveStreamInactive(radio,
                                rto->radio_dat.rssi == PKT_SI446X_NO_CCA_RSSI
                                ? TIME_IMMEDIATE : TIME_MS2I(2000));
  if (msg == MSG_ERROR) {
    /* Parameter error. */
    chDbgAssert(false, "parameter error");
  }

  /* Initialise the radio. It may be uninitialised, in standby or in receive. */
  if(Si446x_conditional_init(radio)) {
    /* Radio did not initialise. */
    TRACE_ERROR("SI   > Radio %d failed to initialise for %s TX", radio,
                          getModulation(rto->radio_dat.type));

    /* Unlock radio. */
    pktUnlockRadio(radio);

    Si446x_transmitRelease(rto, pp, MSG_ERROR);
    /* We never arrive here. */
  }

  /* Initialize timer and variables for AFSK encoder. */
  virtual_timer_t send_timer;
  chVTObjectInit(&send_timer);
  msg_t exit_msg;

  /* get an iterator object. */
  tx_iterator_t iterator;

  /*
   * Use the specified CCA RSSI level.
   * CCA RSSI level will be set to blind send after first packet.
   */
  radio_squelch_t rssi = rto->radio_dat.rssi;

  /* Process each packet. */
  do {

    /*
     * Set NRZI encoding format.
     * Iterator object.
     * Packet reference.
     * Preamble length (HDLC flags).
     * Postamble length (HDLC flags).
     * Tail length (HDLC zeros).
     * Scramble off for AFSK.
     */
    pktStreamIteratorInit(&iterator, pp, 30, 10, 10, false);

    /* Get the stream size by specifying buffer = NULL. */
    uint16_t all = pktStreamEncodingIterator(&iterator, NULL, 0);

    if (all == 0) {
      /* Nothing would be encoded. Release packet send object. */

      TRACE_ERROR("SI   > TX of %s has no NRZI data encoded for radio %d",
                  getModulation(rto->radio_dat.type), radio);

      /* Unlock radio. */
      pktUnlockRadio(radio);

      Si446x_transmitRelease(rto, pp, MSG_ERROR);
      /* We never arrive here. */
    } /* End if (all == 0). */

    /* Allocate buffer and perform NRZI encoding. */
    uint8_t layer0[all];
    pktStreamEncodingIterator(&iterator, layer0, all);

    all *= SAMPLES_PER_BAUD;

    /*
     * The radio is locked and packet receive is now inactive.
     * The 446x has been re-initialised.
     *
     */
    up_sampler_t upsampler = {0};
    upsampler.phase_delta = PHASE_DELTA_1200;

    /* Wait for current radio activity to stop. */
    if (Si446x_waitForState(radio, Si446x_READY, TIME_S2I(5)) == MSG_TIMEOUT) {
      /* Force 446x out of TX state. */
      Si446x_setReadyState(radio);
      TRACE_ERROR("SI   > %s transmit failed to get radio ready on radio %d",
                  getModulation(rto->radio_dat.type), radio);

      /* Unlock radio. */
      pktUnlockRadio(radio);

      Si446x_transmitRelease(rto, pp, MSG_ERROR);
      /* We never arrive here. */
    }

    /*
     * Get the FIFO buffer amount currently available.
     * The TX FIFO is reset to get full capacity.
     */
    uint8_t free = -1;
    if (Si446x_getTXfreeFIFO(radio, 0x01, &free) || free == 0) {
      /* FIFO command failed. */
      TRACE_ERROR("SI   > Radio %d FIFO info command failed for %s TX",
                  getModulation(rto->radio_dat.type), radio);

      /* Unlock radio. */
      pktUnlockRadio(radio);

      Si446x_transmitRelease(rto, pp, MSG_ERROR);
      /* We never arrive here. */
    }

    /* Allocate FIFO feeder buffer. */
    uint8_t localBuffer[free];

    /* Calculate initial FIFO fill. */
    uint16_t c = (all > free) ? free : all;

    /* The exit message if all goes well. */
    exit_msg = MSG_OK;

    /* Initial FIFO load. */
    for (uint16_t i = 0;  i < c; i++)
      localBuffer[i] = Si446x_getUpsampledNRZIbits(&upsampler, layer0);
    if (Si446x_writeFIFO(radio, localBuffer, c)) {

      /* Something failed so set radio ready to clear it. */
      Si446x_setReadyState(radio);

      /* Radio did not initialise. */
      TRACE_ERROR("SI   > Radio %d write to FIFO failed for %s TX",
                  radio, getModulation(rto->radio_dat.type));

      /* Unlock radio. */
      pktUnlockRadio(radio);

      Si446x_transmitRelease(rto, pp, MSG_ERROR);
      /* We never arrive here. */
    }

    uint8_t tide = 0;
#define SI446X_AFSK_CCA_TIMEOUT 5

    if (Si446x_transmit_with_CCA(radio,
                                rto->radio_dat.base_frequency,
                                rto->radio_dat.step_hz,
                                rto->radio_dat.channel,
                                rto->radio_dat.tx_power,
                                rto->radio_dat.type,
                                all,
                                rssi,
                                TIME_S2I(SI446X_AFSK_CCA_TIMEOUT))) {

#define SI446X_AFSK_TX_TIMEOUT 10
        /* TODO: Timeout to be calculated from speed and data size. */
        /* Request start of transmission. */
        /*
         * Start/re-start transmission timeout timer for this packet.
         * If the 446x gets locked the timer runs out and sends an event.
         * We'll then exit TX and release packet object(s).
         */
        chVTSet(&send_timer, TIME_S2I(SI446X_AFSK_TX_TIMEOUT),
                (vtfunc_t)Si446x_transmitTimeoutI, chThdGetSelfX());

        /* Feed the FIFO while data remains to be sent. */
        while ((all - c) > 0) {
          /* Get current TX FIFO free count. */
          uint8_t more;
          if (Si446x_getTXfreeFIFO(radio, 0x00, &more)) {
            /* Something failed so set radio ready to clear it. */
            Si446x_setReadyState(radio);

            /* Radio did not initialise. */
            TRACE_ERROR("SI   > Radio %d read of FIFO free failed for %s TX",
                        radio, getModulation(rto->radio_dat.type));

            /* Unlock radio. */
            pktUnlockRadio(radio);

            Si446x_transmitRelease(rto, pp, MSG_ERROR);
            /* We never arrive here. */
          }

          /* Fill free part of TX FIFO. */
          if (more != 0) {
            /* Update the FIFO free high water mark. */
            tide = (more > tide) ? more : tide;

            /* If there is more free than we need use remainder only. */
            more = (more > (all - c)) ? (all - c) : more;

            /* Load the FIFO. */
            for (uint16_t i = 0; i < more; i++)
              localBuffer[i] = Si446x_getUpsampledNRZIbits(&upsampler, layer0);

            if (Si446x_writeFIFO(radio, localBuffer, more)) {

              /* Something failed so set radio READY. */
              Si446x_setReadyState(radio);

              /* Unlock radio. */
              pktUnlockRadio(radio);

              TRACE_ERROR("SI   > Radio %d write to FIFO failed for %s TX",
                          radio, getModulation(rto->radio_dat.type));

              Si446x_transmitRelease(rto, pp, MSG_ERROR);
              /* We never arrive here. */
            }
          }

          /* Update count of sent bytes. */
          c += more;

          /*
           * Wait for a timeout event during NRZI send.
           * Time delay allows ~SAMPLES_PER_BAUD bytes to be consumed from FIFO.
           * If no timeout event go back and load more data to FIFO.
           * TODO: Use 446x interrupt to trigger FIFO fill.
           */
          /*        eventmask_t evt = chEvtWaitAnyTimeout(SI446X_EVT_TX_TIMEOUT,
                                              chTimeUS2I(833 * 8));*/
          eventmask_t evt = chEvtGetAndClearEvents(SI446X_EVT_TX_TIMEOUT);
          if (evt) {
            exit_msg = MSG_TIMEOUT;
            break;
          }
          /* FIXME: Let lower, peer and higher priority threads run. */
          chThdYield();
        } /* End while(). */
      } else {
        /* Transmit start failed. */
        TRACE_ERROR("SI   > %s transmit start failed on radio %d",
                    getModulation(rto->radio_dat.type), radio);
        exit_msg = MSG_ERROR;
      } /* End transmit. */

      chVTReset(&send_timer);
#if 0 // transmit now handles waiting for any prior TX to finish
      /* If nothing went wrong wait for TX to finish. Else don't wait. */
      while (Si446x_getState(radio) == Si446x_STATE_TX && exit_msg == MSG_OK) {
        /* TODO: Add an absolute timeout on this. */
        /* Sleep for an AFSK byte time. */
        chThdSleep(chTimeUS2I(833 * 8));
        continue;
      }
#endif
      /* No CCA on subsequent packets of burst send. */
      rssi = PKT_SI446X_NO_CCA_RSSI;

      /* Get the next linked packet to send before releasing current. */
      packet_t np = pp->nextp;
      if (exit_msg == MSG_OK) {
        /* Send was OK. Release the just completed packet. */
        pktReleaseBufferObject(pp);
      } else {
        /* Send failed so release any queue and terminate. */
        pktReleaseBufferChain(pp);
        np = NULL;
        /* Force 446x out of TX state. */
        Si446x_setReadyState(radio);
      }


      if (tide > (free / 2)) {
        /* Warn when free level is > 50% of FIFO size. */
        TRACE_WARN("SI   > %s TX FIFO free of %i exceeded safe level of %i on radio %d",
                   getModulation(rto->radio_dat.type), tide, free / 2, radio);
      }
      pp = np;
    } while (pp != NULL);  /* Process next packet. */

    /* Wait for transmission to finish. */
    if (Si446x_waitForState(radio, Si446x_READY, TIME_S2I(5)) == MSG_TIMEOUT) {
      /* Force 446x out of TX state. */
      Si446x_setReadyState(radio);
      exit_msg = MSG_TIMEOUT;
      TRACE_ERROR("SI   > %s transmit failed to cease on radio %d",
                  getModulation(rto->radio_dat.type), radio);
    }

    /* Unlock radio. */
    pktUnlockRadio(radio);

    /* Signal RTM that the TX is complete. */
    Si446x_transmitRelease(rto, pp, exit_msg);
    /* We never arrive here. */
  }

/*
 *
 */
bool Si446x_blocSendAFSK(radio_task_object_t *rt) {

    thread_t *afsk_feeder_thd = chThdCreateFromHeap(NULL,
                THD_WORKING_AREA_SIZE(SI_AFSK_FIFO_MIN_FEEDER_WA_SIZE),
                "tx_afsk_queued",
                NORMALPRIO - 10,
                bloc_si_fifo_feeder_afsk,
                rt);

    if(afsk_feeder_thd == NULL) {
      TRACE_ERROR("SI   > Unable to create AFSK transmit thread for radio %d",
                  rt->handler->radio);
      return false;
    }
    return true;
}

/*
 * 2FSK
 */

/*
 * New 2FSK send thread using minimized buffer space and burst send.
 */
THD_FUNCTION(bloc_si_fifo_feeder_fsk, arg) {
  radio_task_object_t *rto = arg;

  radio_unit_t radio = rto->handler->radio;
  packet_t pp = rto->radio_dat.pkt.packet_out;

  chDbgAssert(pp != NULL, "no packet in radio task");

  /* Create thread name for this instance. */
  char tx_thd_name[PKT_THREAD_NAME_MAX];
  chsnprintf(tx_thd_name, sizeof(tx_thd_name),
             "%s_%03i",
             getModulation(rto->radio_dat.type),
             rto->radio_dat.seq_num);

  chRegSetThreadName(tx_thd_name);

  /* Lock radio for the entirety of the transmit session. */
  msg_t msg = pktLockRadio(radio, RADIO_TX, rto->radio_dat.timer);

  if (msg != MSG_OK) {
    TRACE_ERROR("SI   > TX reset or timeout when %s attempting radio %d acquisition",
                getModulation(rto->radio_dat.type), radio);

    Si446x_transmitRelease(rto, pp, msg);
    /* We never arrive here. */
  }

  /* Wait for any active receive stream in progress. Force inactive on timeout.
     Any stream is stopped but the 446x would remain in receive state. */
  msg = pktSetReceiveStreamInactive(radio,
                                rto->radio_dat.rssi == PKT_SI446X_NO_CCA_RSSI
                                ? TIME_IMMEDIATE : TIME_MS2I(2000));
  if (msg == MSG_ERROR) {
    /* Parameter error. */
    chDbgAssert(false, "parameter error");
  }

  /* Initialise the radio. It may be uninitialised, in standby or in receive. */
  if(Si446x_conditional_init(radio)) {
    /* Radio did not initialise. */
    TRACE_ERROR("SI   > Radio %d failed to initialise for %s TX", radio,
                          getModulation(rto->radio_dat.type));

    /* Unlock radio. */
    pktUnlockRadio(radio);

    Si446x_transmitRelease(rto, pp, MSG_ERROR);
    /* We never arrive here. */
  }

  /*
   *  Initialize transmit timeout timer and iterator. */
  virtual_timer_t send_timer;
  chVTObjectInit(&send_timer);

  tx_iterator_t iterator;

  /* The exit message. */
  msg_t exit_msg;

  /*
   * Use the specified CCA RSSI level.
   * This can be be blind send (PKT_SI446X_NO_CCA_RSSI).
   * In burst mode CCA will be set to blind send after the first packet.
   */
  radio_squelch_t rssi = rto->radio_dat.rssi;

  do {
    /*
     * Set NRZI encoding format.
     * Iterator object.
     * Packet reference.
     * Preamble length (HDLC flags)
     * Postamble length (HDLC flags)
     * Tail length (HDLC zeros)
     * Scramble on
     */
    pktStreamIteratorInit(&iterator, pp, 100, 10, 10, true);

    /* Compute size of NRZI stream. */
    uint16_t all = pktStreamEncodingIterator(&iterator, NULL, 0);

    if (all == 0) {
      /* Nothing encoded. Release packet send object. */
      TRACE_ERROR("SI   > TX of %s has no NRZI data encoded for radio %d",
                  getModulation(rto->radio_dat.type), radio);

      /* Unlock radio. */
      pktUnlockRadio(radio);

      Si446x_transmitRelease(rto, pp, MSG_ERROR);
      /* We never arrive here. */
    }

    /* Allocate buffer and perform NRZI encoding in a single pass. */
    uint8_t layer0[all];

    pktStreamEncodingIterator(&iterator, layer0, all);

    /* Wait for current radio activity to stop. */
    if (Si446x_waitForState(radio, Si446x_READY, TIME_S2I(5)) == MSG_TIMEOUT) {
      /* Force 446x out of TX state. */
      Si446x_setReadyState(radio);
      TRACE_ERROR("SI   > %s transmit failed to get radio ready on radio %d",
                  getModulation(rto->radio_dat.type), radio);

      /* Unlock radio. */
      pktUnlockRadio(radio);

      Si446x_transmitRelease(rto, pp, MSG_ERROR);
      /* We never arrive here. */
    }
    /*
     * Get the FIFO buffer amount currently available.
     * The TX FIFO is reset to get full capacity.
     */
    uint8_t free = -1;

    if (Si446x_getTXfreeFIFO(radio, 0x01, &free) || free == 0) {
      /* FIFO command failed. */
      TRACE_ERROR("SI   > Radio %d FIFO info command failed for %s TX",
                  getModulation(rto->radio_dat.type), radio);

      /* Unlock radio. */
      pktUnlockRadio(radio);

      Si446x_transmitRelease(rto, pp, MSG_ERROR);
      /* We never arrive here. */
    }

    /* Calculate initial FIFO fill. */
    uint16_t c = (all > free) ? free : all;

    /* The exit message if all goes well. */
    exit_msg = MSG_OK;

    uint8_t *bufp = layer0;

    /* Initial FIFO load. */
    if (Si446x_writeFIFO(radio, bufp, c)) {

      /* Something failed so set radio ready to clear it. */
      Si446x_setReadyState(radio);

      /* Radio did not initialise. */
      TRACE_ERROR("SI   > Radio %d write to FIFO failed for %s TX",
                  radio, getModulation(rto->radio_dat.type));

      /* Unlock radio. */
      pktUnlockRadio(radio);

      Si446x_transmitRelease(rto, pp, MSG_ERROR);
      /* We never arrive here. */
    }

    /* Update buffer index after initial load. */
    bufp += c;

    /* Set FIFO tide level. */
    uint8_t tide = 0;

#define SI446X_2FSK_CCA_TIMEOUT 5
    /* TODO: Timeout to be calculated from speed and data size. */
    /* Request start of transmission. */
    if (Si446x_transmit_with_CCA(radio,
                       rto->radio_dat.base_frequency,
                       rto->radio_dat.step_hz,
                       rto->radio_dat.channel,
                       rto->radio_dat.tx_power,
                       rto->radio_dat.type,
                       all,
                       rssi,
                       TIME_S2I(SI446X_2FSK_CCA_TIMEOUT))) {

#define SI446X_2FSK_TX_TIMEOUT 10
        /*
         * Start/re-start transmission timeout timer for this packet.
         * If the 446x gets locked up we'll exit TX and release packet object(s).
         */
        chVTSet(&send_timer, TIME_S2I(SI446X_2FSK_TX_TIMEOUT),
                (vtfunc_t)Si446x_transmitTimeoutI, chThdGetSelfX());

        /* Feed the FIFO while data remains to be sent. */
        while ((all - c) > 0) {
          /* Get TX FIFO free count. */
          uint8_t more;
          if (Si446x_getTXfreeFIFO(radio, 0x00, &more)) {
            /* Something failed so set radio ready to clear it. */
            Si446x_setReadyState(radio);

            /* Radio did not initialise. */
            TRACE_ERROR("SI   > Radio %d read of FIFO free failed for %s TX",
                        radio, getModulation(rto->radio_dat.type));

            /* Unlock radio. */
            pktUnlockRadio(radio);

            Si446x_transmitRelease(rto, pp, MSG_ERROR);
            /* We never arrive here. */
          }

          /* Have the amount of free FIFO space now. */
          if (more != 0) {
            /* Update the FIFO free high water mark. */
            tide = (more > tide) ? more : tide;

            /* If there is more free than we need for send use remainder only. */
            more = (more > (all - c)) ? (all - c) : more;

            /* Load the FIFO. */
            if (Si446x_writeFIFO(radio, bufp, more)) {

              /* Something failed so set radio READY. */
              Si446x_setReadyState(radio);

              /* Unlock radio. */
              pktUnlockRadio(radio);

              TRACE_ERROR("SI   > Radio %d write to FIFO failed for %s TX",
                          radio, getModulation(rto->radio_dat.type));

              Si446x_transmitRelease(rto, pp, MSG_ERROR);
              /* We never arrive here. */
            }
          }
          /* Update buffer index. */
          bufp += more;
          c += more;

          /*
           * Wait for a timeout event during NRZI send.
           * Time delay allows ~10 bytes to be consumed from FIFO.
           * If no timeout event go back and load more data to FIFO.
           * TODO: Use interrupt to trigger FIFO fill.
           */
          /*        eventmask_t evt = chEvtWaitAnyTimeout(SI446X_EVT_TX_TIMEOUT,
                                              chTimeUS2I(104 * 8 * 10));*/
          eventmask_t evt = chEvtGetAndClearEvents(SI446X_EVT_TX_TIMEOUT);
          if (evt) {
            exit_msg = MSG_TIMEOUT;
            break;
          }
          /* TODO: Let lower, peer and higher priority threads run. */
          chThdYield();
        } /* End while(). */
      } else {
        /* Transmit start failed. */
        exit_msg = MSG_ERROR;
      }
      chVTReset(&send_timer);

#if 0 // transmit now handles waiting for any prior TX to finish
      while (Si446x_getState(radio) == Si446x_STATE_TX && exit_msg == MSG_OK) {
        /* TODO: Add an absolute timeout on this. */
        /* Sleep for a 2FSK byte time. */
        chThdSleep(chTimeUS2I(104 * 8 * 10));
        continue;
      }
#endif
      /* No CCA on subsequent packet sends. */
      rssi = PKT_SI446X_NO_CCA_RSSI;

      /* Get the next linked packet to send. */
      packet_t np = pp->nextp;
      if (exit_msg == MSG_OK) {

        /* Send was OK. Release the just completed packet. */
        pktReleaseBufferObject(pp);
      } else {
        /* Send failed so release any queue and terminate. */
        pktReleaseBufferChain(pp);
        np = NULL;
        /* Force 446x out of TX state. */
        Si446x_setReadyState(radio);
      }

      if (tide > (free / 2)) {
        /* Warn when free level is > 50% of FIFO size. */
        TRACE_WARN("SI   > 2FSK TX FIFO free at %i exceeded safe level of %i on radio %d",
                   tide, free / 2, radio);
      }

      /* Process next packet. */
      pp = np;
    } while (pp != NULL);  /* Process next packet. */

    /* Wait for transmission to finish. */
    if (Si446x_waitForState(radio, Si446x_READY, TIME_S2I(5)) == MSG_TIMEOUT) {
      /* Force 446x out of TX state. */
      Si446x_setReadyState(radio);
      exit_msg = MSG_TIMEOUT;
      TRACE_ERROR("SI   > %s transmit failed to cease on radio %d",
                  getModulation(rto->radio_dat.type), radio);
    }

    /* Unlock radio. */
    pktUnlockRadio(radio);

    /* Signal RTM that the TX is complete. */
    Si446x_transmitRelease(rto, pp, exit_msg);
    /* We never arrive here. */
}

/*
 * Return true on send successfully enqueued.
 * Task object will be returned
 * Return false on failure
 */
bool Si446x_blocSend2FSK(radio_task_object_t *rt) {

  thread_t *fsk_feeder_thd = NULL;

  fsk_feeder_thd = chThdCreateFromHeap(NULL,
              THD_WORKING_AREA_SIZE(SI_FSK_FIFO_FEEDER_WA_SIZE),
              "tx_2fsk_queued",
              NORMALPRIO - 10,
              bloc_si_fifo_feeder_fsk,
              rt);

  if(fsk_feeder_thd == NULL) {
    TRACE_ERROR("SI   > Unable to create FSK transmit thread for radio %d",
        rt->handler->radio);
    return false;
  }
  return true;
}

/*
 * Return true on send successfully enqueued.
 * Task object will be returned
 * Return false on failure
 */
bool Si446x_blocSendCW(radio_task_object_t *rt) {
  (void)rt;
  return false;
}

/**
 * Used by collector to get radio temp data.
 */
radio_temp_t Si446x_getLastTemperature(const radio_unit_t radio) {
  return Si446x_getData(radio)->lastTemp;
}

/**
 * Attach PWM sets up the hardware resources and mapping for ICU and CCA.
 * The ICU is started. The GPIOs are set in the required mode as defined by
 * the radio configuration.
 */
ICUDriver *Si446x_attachPWM(const radio_unit_t radio) {

  /* The radio RX_RAW_DATA output is routed to an ICU timer channel.  */
  pktSetGPIOlineMode(*Si446x_getConfig(radio)->rafsk.pwm.line,
                     Si446x_getConfig(radio)->rafsk.pwm.mode);

  /**
   * Set up GPIO port where the radio line outputting CCA is connected.
   * The MCU GPIO when enabled will be set to interrupt on CCA changes.
   */
  pktSetGPIOlineMode(*Si446x_getConfig(radio)->rafsk.cca.line,
                     Si446x_getConfig(radio)->rafsk.cca.mode);

  /* Initialise and start the ICU. */
  ICUDriver *picu = Si446x_getConfig(radio)->rafsk.icu;
  icuObjectInit(picu);
  icuStart(picu, &Si446x_getConfig(radio)->rafsk.cfg);
  return picu;
}

/**
 * Detach the radio ICU. Stops the ICU and set the GPIOs to input state.
 * Any GPIO interrupt/event settings should be disabled already.
 */
void Si446x_detachPWM(const radio_unit_t radio) {
  icuStop(Si446x_getConfig(radio)->rafsk.icu);

  /* Timer channel GPIO. */
  pktSetGPIOlineMode(*Si446x_getConfig(radio)->rafsk.pwm.line,
                     PAL_MODE_INPUT_PULLUP);

  /* CCA GPIO. */
  pktSetGPIOlineMode(*Si446x_getConfig(radio)->rafsk.cca.line,
                     PAL_MODE_INPUT_PULLUP);
}

/**
 * Enable the GPIO for CCA used in PWM mode of operation.
 * Set a callback and enable event on both edges.
 * The RX_DATA GPIO is configured by Si446x_attachPWM().
 */
void Si446x_enablePWMevents(const radio_unit_t radio,
                                        const radio_mod_t mod,
                                        const palcallback_t cb) {
 (void)mod;
  /* Set callback for squelch events. */
  palSetLineCallback(*Si446x_getConfig(radio)->rafsk.cca.line, cb,
                     Si446x_getConfig(radio)->rafsk.icu);

  /* Enabling events on both edges of CCA.*/
  palEnableLineEvent(*Si446x_getConfig(radio)->rafsk.cca.line,
                     PAL_EVENT_MODE_BOTH_EDGES);

  //return &Si446x_getConfig(radio)->rafsk.cfg;
}

/**
 * TODO: Switch on mod type
 */
void Si446x_disablePWMeventsI(const radio_unit_t radio, radio_mod_t mod) {
  (void)mod;
  palDisableLineEventI(*Si446x_getConfig(radio)->rafsk.cca.line);
}

/**
 *
 */
uint8_t Si446x_readCCAlineForRX(const radio_unit_t radio,
                           const radio_mod_t mod) {

  /* Read CAA as setup by mod type. */
  switch(mod) {
  case MOD_2FSK_300:
  case MOD_2FSK_9k6:
  case MOD_2FSK_19k2:
  case MOD_2FSK_38k4:
  case MOD_2FSK_57k6:
  case MOD_2FSK_76k8:
  case MOD_2FSK_96k:
  case MOD_2FSK_115k2:
    return palReadLine(*Si446x_getConfig(radio)->r2fsk.cca.line);

  case MOD_AFSK: {
    return palReadLine(*Si446x_getConfig(radio)->rafsk.cca.line);
  }

  case MOD_CW:
    return palReadLine(*Si446x_getConfig(radio)->r2fsk.cca.line);

  case MOD_NONE:
    break;
  }
  return 0;
}

/**
 * return true if clock change was applied else false.
 */
bool Si446x_updateClock(const radio_unit_t radio, const xtal_osc_t freq) {
  if(freq == 0)
    return false;
  radio_clock_t xo = Si446x_getData(radio)->radio_clock;
  if((radio_clock_t)freq != xo) {
    Si446x_getData(radio)->radio_clock = (radio_clock_t)freq;
    return true;
  }
  return false;
}
