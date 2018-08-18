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
 */
static SPIConfig ls_spicfg = {
    .cr1    = SPI_CR1_MSTR
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
                             const uint8_t* txData, uint32_t len) {
  /* Write data via SPI with CTS checked via GPIO1. */

  /* Acquire bus and then start SPI. */
  SPIDriver *spip = Si446x_spiSetupBus(radio, &ls_spicfg);
  spiStart(spip, &ls_spicfg);

  /* Poll for CTS with timeout of 100mS. */
  ioline_t cts = Si446x_getConfig(radio)->gpio1;
  uint8_t timeout = 100;
  do {
    if(timeout != 100)
      chThdSleep(TIME_MS2I(1));
  } while(palReadLine(cts) != PAL_HIGH && timeout--);

  if(!timeout) {
    TRACE_ERROR("SI   > CTS not received");
    /* Stop SPI and relinquish bus. */
    spiStop(spip);
    spiReleaseBus(spip);
    return false;
  }

  /* Transfer data now there is CTS.*/
  spiSelect(spip);
  spiSend(spip, len, txData);
  spiUnselect(spip);

  /* Stop SPI and relinquish bus. */
  spiStop(spip);
  spiReleaseBus(spip);

  return true;
}

/**
 *
 */
static bool Si446x_write(const radio_unit_t radio,
		const uint8_t* txData, uint32_t len) {
    /* Transmit data by SPI with CTS polling by command. */
    uint8_t null_spi[len];

    /* Acquire bus, get SPI Driver object and then start SPI. */
    SPIDriver *spip = Si446x_spiSetupBus(radio, &ls_spicfg);
    spiStart(spip, &ls_spicfg);

    /* Poll for CTS with timeout of 100mS. */
    uint8_t timeout = 100;
    uint8_t rx_ready[] = {Si446x_READ_CMD_BUFF, 0x00};
    do {
      spiSelect(spip);
      spiExchange(spip, 1, rx_ready, &rx_ready[1]);
      spiUnselect(spip);
      if(timeout != 100)
        chThdSleep(TIME_MS2I(1));
    } while(rx_ready[1] != Si446x_COMMAND_CTS && timeout--);

    if(!timeout) {
      TRACE_ERROR("SI   > CTS not received");
      /* Stop SPI and relinquish bus. */
      spiStop(spip);
      spiReleaseBus(spip);
      return false;
    }
    
    /* Transfer data. */
    spiSelect(spip);
    spiExchange(spip, len, txData, null_spi);
    spiUnselect(spip);

    /* Stop SPI and relinquish bus. */
    spiStop(spip);
    spiReleaseBus(spip);
    
    return true;
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
    while(palReadLine(cts) != PAL_HIGH && timeout--) {
        chThdSleep(TIME_MS2I(1));
    }

    if(!timeout) {
      /* Relinquish bus. */
      spiReleaseBus(spip);
      TRACE_ERROR("SI   > CTS not received");
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
    while(palReadLine(cts) != PAL_HIGH && timeout--) {
        chThdSleep(TIME_MS2I(1));
    }

    if(!timeout) {
      /* Stop SPI and relinquish bus. */
      spiStop(spip);
      spiReleaseBus(spip);
      TRACE_ERROR("SI   > CTS not received");
      return false;
    }

    /* Read the response. */
    uint8_t rx_ready[] = {Si446x_READ_CMD_BUFF, 0x00};
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
    uint8_t rx_ready[] = {Si446x_READ_CMD_BUFF, 0x00};
    do {
      if(timeout != 100)
        chThdSleep(TIME_MS2I(1));
      spiSelect(spip);
      spiExchange(spip, 1, rx_ready, &rx_ready[1]);
      spiUnselect(spip);

    } while(rx_ready[1] != Si446x_COMMAND_CTS && timeout--);

    if(!timeout) {
      TRACE_ERROR("SI   > CTS not received");
      /* Stop SPI and relinquish bus. */
      spiStop(spip);
      spiReleaseBus(spip);
      return false;
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
      if(timeout != 100)
        chThdSleep(TIME_MS2I(1));
      spiSelect(spip);
      spiExchange(spip, rxlen, rx_ready, rxData);
      spiUnselect(spip);
    } while(rxData[1] != Si446x_COMMAND_CTS && timeout--);

    /* Stop SPI and relinquish bus. */
    spiStop(spip);
    spiReleaseBus(spip);
    
   if(!timeout) {
      TRACE_ERROR("SI   > CTS not received");
      return false;
    }
    return true;
}

/* TODO: Make set property a single func with size parameter. */
static void Si446x_setProperty8(const radio_unit_t radio,
		uint16_t reg, uint8_t val) {
    uint8_t msg[] = {Si446x_SET_PROPERTY,
                     (reg >> 8) & 0xFF, 0x01, reg & 0xFF, val};
    Si446x_write(radio, msg, sizeof(msg));
}

static void Si446x_setProperty16(const radio_unit_t radio,
		uint16_t reg, uint8_t val1, uint8_t val2) {
    uint8_t msg[] = {Si446x_SET_PROPERTY,
                     (reg >> 8) & 0xFF, 0x02, reg & 0xFF, val1, val2};
    Si446x_write(radio, msg, sizeof(msg));
}

static void Si446x_setProperty24(const radio_unit_t radio,
		                         uint16_t reg, uint8_t val1,
                                 uint8_t val2, uint8_t val3) {
    uint8_t msg[] = {Si446x_SET_PROPERTY,
                     (reg >> 8) & 0xFF, 0x03, reg & 0xFF, val1, val2, val3};
    Si446x_write(radio, msg, sizeof(msg));
}

static void Si446x_setProperty32(const radio_unit_t radio,
		                         uint16_t reg, uint8_t val1,
                                 uint8_t val2, uint8_t val3, uint8_t val4) {
    uint8_t msg[] = {Si446x_SET_PROPERTY,
                     (reg >> 8) & 0xFF, 0x04, reg & 0xFF,
                     val1, val2, val3, val4};
    Si446x_write(radio, msg, sizeof(msg));
}

/**
 * Get temperature of chip.
 */
void Si446x_getTemperature(const radio_unit_t radio) {
  const uint8_t txData[] = {Si446x_GET_ADC_READING, 0x10};
  uint8_t rxData[8];
  Si446x_read(radio, txData, sizeof(txData), rxData, sizeof(rxData));
  uint16_t adc = rxData[7] | ((rxData[6] & 0x7) << 8);
  int16_t temp = (89900 * adc) / 4096 - 29300;
  Si446x_getData(radio)->lastTemp = temp;
}

/**
 * Initializes Si446x transceiver chip.
 */
static bool Si446x_init(const radio_unit_t radio) {

  TRACE_INFO("SI   > Start up and initialize radio %d", radio);

  packet_svc_t *handler = pktGetServiceObject(radio);

  /*
   * Set MCU GPIO for radio GPIO1 (CTS).
   * Execute radio startup sequence.
   */
  if(!Si446x_radioStartup(radio)) {
    TRACE_ERROR("SI   > Start up of radio %d failed", radio);
    return false;
  }

  /* Calculate clock source parameters. */
  const uint8_t x3 = (Si446x_CCLK >> 24) & 0x0FF;
  const uint8_t x2 = (Si446x_CCLK >> 16) & 0x0FF;
  const uint8_t x1 = (Si446x_CCLK >>  8) & 0x0FF;
  const uint8_t x0 = (Si446x_CCLK >>  0) & 0x0FF;

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
  Si446x_writeBoot(radio, init_command, sizeof(init_command));

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

  /* Save the part number and ROM revision. */
  handler->radio_part = (part_info.info[3] << 8) + part_info.info[4];
  handler->radio_rom_rev = part_info.info[9];

  /*
   * Check this radio requires a patch installed.
   * TODO: Probably should be in a table...
   */
  if(is_Si4463_patch_required(handler->radio_part, handler->radio_rom_rev)) {
    /* Power cycle radio and apply patch. */
    Si446x_radioShutdown(radio);
    chThdSleep(TIME_MS2I(10));
    Si446x_radioStartup(radio);
    uint16_t i = 0;
    while(Si4463_Patch_Data_Array[i] != 0) {
      Si446x_writeBoot(radio, &Si4463_Patch_Data_Array[i + 1],
                       Si4463_Patch_Data_Array[i]);
      i += Si4463_Patch_Data_Array[i] + 1;
    }
    const uint8_t init_command[] = {Si446x_POWER_UP, 0x81,
                                    (Si446x_CLK_TCXO_EN & 0x1),
                                    x3, x2, x1, x0};
    Si446x_writeBoot(radio, init_command, sizeof(init_command));
  }

  /* Get and save the patch ID from FUNC_INFO for reference. */
  si446x_func_t func_info;
  const uint8_t get_func[] = {Si446x_GET_FUNC_INFO};
  Si446x_readBoot(radio, get_func, sizeof(get_func), (uint8_t *)&func_info,
              sizeof(func_info));

  handler->radio_patch = (func_info.info[5] << 8) + func_info.info[6];

  /*
   * Set transceiver GPIOs.
   * GPIO0, 1 and NIRQ can now be reconfigured as required by TX or RX modes.
   * In that case each needs to setup GPIOs as required.
   */
  uint8_t gpio_pin_cfg_command2[] = {
      Si446x_GPIO_PIN_CFG,   // Command type = GPIO settings
      0x00,   // GPIO0        GPIO_MODE = DONOTHING
      0x15,   // GPIO1        GPIO_MODE = RAW_RX_DATA
      0x21,   // GPIO2        GPIO_MODE = RX_STATE
      0x20,   // GPIO3        GPIO_MODE = TX_STATE
      0x1B,   // NIRQ         NIRQ_MODE = CCA
      0x0B,   // SDO          SDO_MODE = SDO
      0x00    // GEN_CONFIG
  };

  Si446x_write(radio, gpio_pin_cfg_command2, sizeof(gpio_pin_cfg_command2));

  /* TODO: We should clear interrupts here with a GET_INT_STATUS. */

  /* If Si446x is using its own xtal set the trim capacitor value. */
  #if !Si446x_CLK_TCXO_EN
  Si446x_setProperty8(radio, Si446x_GLOBAL_XO_TUNE, 0x40);
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
  Si446x_write(radio, reset_fifo, sizeof(reset_fifo));
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

  /* PLL synthesizer settings. */
  Si446x_setProperty8(radio, Si446x_MODEM_CLKGEN_BAND, 0x0D);

  /* Deviation set to +-0.5KHz. */
  Si446x_setProperty24(radio, Si446x_MODEM_FREQ_DEV, 0x00, 0x00, 0x79);

  /* Ramp down after TX final symbol. */
  Si446x_setProperty8(radio, Si446x_MODEM_TX_RAMP_DELAY, 0x01);

  /* PA ramp timing and modulation delay. */
  Si446x_setProperty8(radio, Si446x_PA_TC, 0x3D);

  /* Synthesizer PLL settings. */
  Si446x_setProperty8(radio, Si446x_FREQ_CONTROL_INTE, 0x41);
  Si446x_setProperty24(radio, Si446x_FREQ_CONTROL_FRAC, 0x0B, 0xB1, 0x3B);
  Si446x_setProperty16(radio, Si446x_FREQ_CONTROL_CHANNEL_STEP_SIZE, 0x0B, 0xD1);
  Si446x_setProperty8(radio, Si446x_FREQ_CONTROL_W_SIZE, 0x20);
  Si446x_setProperty8(radio, Si446x_FREQ_CONTROL_VCOCNT_RX_ADJ, 0xFA);

  /* Antenna settings. */
  Si446x_setProperty8(radio, Si446x_MODEM_ANT_DIV_MODE, 0x01);
  Si446x_setProperty8(radio, Si446x_MODEM_ANT_DIV_CONTROL, 0x80);

  /* RSSI value compensation. */
  if(is_part_Si4463(handler->radio_part))
    Si446x_setProperty8(radio, Si446x_MODEM_RSSI_COMP, 0x44);
  else
    Si446x_setProperty8(radio, Si446x_MODEM_RSSI_COMP, 0x40);

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
  return true;
}

/**
 * Intialize radio only if it has been shutdown.
 */
bool Si446x_conditional_init(const radio_unit_t radio) {
  packet_svc_t *handler = pktGetServiceObject(radio);

  if(!handler->radio_init)
    return Si446x_init(radio);
  return true;
}

/*
 * Set radio NCO registers for frequency.
 * This function also collects the chip temperature data at the moment.
 * TODO: Move temperature reading to???
 */
bool Si446x_setBandParameters(const radio_unit_t radio,
                              radio_freq_t freq,
                              channel_hz_t step) {

  /* Check frequency is in range of chip. */
  if(freq < 144000000UL || freq > 900000000UL)
    return false;


  /* Set the output divider as recommended in Si446x data sheet. */
  uint32_t outdiv = 0;
  uint32_t band = 0;
  if(freq < 705000000UL) {outdiv = 6;  band = 1;}
  if(freq < 525000000UL) {outdiv = 8;  band = 2;}
  if(freq < 353000000UL) {outdiv = 12; band = 3;}
  if(freq < 239000000UL) {outdiv = 16; band = 4;}
  if(freq < 177000000UL) {outdiv = 24; band = 5;}

  /*
   * Initialize radio.
   */
  //Si446x_conditional_init(radio);

  /* Set the band parameter. */
  uint32_t sy_sel = 8;
  uint8_t set_band_property_command[] = {Si446x_SET_PROPERTY,
                                         0x20, 0x01, 0x51, (band + sy_sel)};
  Si446x_write(radio, set_band_property_command,
		  sizeof(set_band_property_command));

  /* Set the PLL parameters. */
  uint32_t f_pfd = 2 * Si446x_CCLK / outdiv;
  uint32_t n = ((uint32_t)(freq / f_pfd)) - 1;
  float ratio = (float)freq / (float)f_pfd;
  float rest  = ratio - (float)n;

  uint32_t m = (uint32_t)(rest * 524288UL);
  uint32_t m2 = m >> 16;
  uint32_t m1 = (m - m2 * 0x10000) >> 8;
  uint32_t m0 = (m - m2 * 0x10000 - (m1 << 8));

  uint32_t channel_increment = 524288 * outdiv * step / (2 * Si446x_CCLK);
  uint8_t c1 = channel_increment / 0x100;
  uint8_t c0 = channel_increment - (0x100 * c1);

  uint8_t set_frequency_property_command[] = {Si446x_SET_PROPERTY,
                                              0x40, 0x04, 0x00, n,
                                              m2, m1, m0, c1, c0};
  Si446x_write(radio, set_frequency_property_command,
               sizeof(set_frequency_property_command));

  uint32_t x = ((((uint32_t)1 << 19) * outdiv * 1300.0)/(2*Si446x_CCLK))*2;
  uint8_t x2 = (x >> 16) & 0xFF;
  uint8_t x1 = (x >>  8) & 0xFF;
  uint8_t x0 = (x >>  0) & 0xFF;
  uint8_t set_deviation[] = {Si446x_SET_PROPERTY, 0x20, 0x03, 0x0a, x2, x1, x0};
  Si446x_write(radio, set_deviation, sizeof(set_deviation));

  /* Measure the chip temperature and update saved value. */
  Si446x_getTemperature(radio);
  return true;
}

/*static void Si446x_setShift(uint16_t shift)
{
    if(!shift)
        return;

    float units_per_hz = (( 0x40000 * outdiv ) / (float)Si446x_CCLK);

    // Set deviation for 2FSK
    uint32_t modem_freq_dev = (uint32_t)(units_per_hz * shift / 2.0 );
    uint8_t modem_freq_dev_0 = 0xFF & modem_freq_dev;
    uint8_t modem_freq_dev_1 = 0xFF & (modem_freq_dev >> 8);
    uint8_t modem_freq_dev_2 = 0xFF & (modem_freq_dev >> 16);

    uint8_t set_modem_freq_dev_command[] = {0x11, 0x20, 0x03, 0x0A, modem_freq_dev_2, modem_freq_dev_1, modem_freq_dev_0};
    Si446x_write(set_modem_freq_dev_command, 7);
}*/

static void Si446x_setPowerLevel(const radio_unit_t radio,
								 const radio_pwr_t level) {
    // Set the Power
    uint8_t set_pa_pwr_lvl_property_command[] = {Si446x_SET_PROPERTY,
                                                 0x22, 0x01, 0x01, level};
    Si446x_write(radio, set_pa_pwr_lvl_property_command,
                 sizeof(set_pa_pwr_lvl_property_command));
}



/*
 *  Radio modulation settings
 */

static void Si446x_setModemAFSK_TX(const radio_unit_t radio) {
    // Setup the NCO modulo and oversampling mode
    uint32_t s = Si446x_CCLK / 10;
    uint8_t f3 = (s >> 24) & 0xFF;
    uint8_t f2 = (s >> 16) & 0xFF;
    uint8_t f1 = (s >>  8) & 0xFF;
    uint8_t f0 = (s >>  0) & 0xFF;
    Si446x_setProperty32(radio, Si446x_MODEM_TX_NCO_MODE, f3, f2, f1, f0);

    // Setup the NCO data rate for APRS
    Si446x_setProperty24(radio, Si446x_MODEM_DATA_RATE, 0x00, 0x33, 0x90);

    // Use up-sampled AFSK from FIFO (PH)
    Si446x_setProperty8(radio, Si446x_MODEM_MOD_TYPE, 0x02);

    /* Set PH bit order for AFSK. */
    Si446x_setProperty8(radio, Si446x_PKT_CONFIG1, 0x01);

    // Set AFSK filter
    const uint8_t coeff[] = {0x81, 0x9f, 0xc4, 0xee, 0x18, 0x3e, 0x5c, 0x70, 0x76};
    uint8_t i;
    for(i = 0; i < sizeof(coeff); i++) {
        uint8_t msg[] = {0x11, 0x20, 0x01, 0x17-i, coeff[i]};
        Si446x_write(radio, msg, 5);
    }
}

static void Si446x_setModemAFSK_RX(const radio_unit_t radio) {

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

  /* Set DIRECT_MODE (asynchronous mode as 2FSK). */
  Si446x_setProperty8(radio, Si446x_MODEM_MOD_TYPE, 0x0A);

  /* Packet handler disabled in RX. */
  Si446x_setProperty8(radio, Si446x_PKT_CONFIG1, 0x41);

  if(is_part_Si4463(handler->radio_part)) {
    /* Run 4463 in 4464 compatibility mode (set SEARCH2 to zero). */
    Si446x_setProperty8(radio, Si446x_MODEM_RAW_SEARCH2, 0x00);
  }
  Si446x_setProperty8(radio, Si446x_MODEM_RAW_CONTROL, 0x8F);
  Si446x_setProperty8(radio, Si446x_MODEM_RAW_SEARCH, 0xD6);
  Si446x_setProperty16(radio, Si446x_MODEM_RAW_EYE, 0x00, 0x3B);

  /*
   * OOK_MISC settings include parameters related to asynchronous mode.
   * Asynchronous mode is used for AFSK reception passed to DSP decode.
   */
  Si446x_setProperty8(radio, Si446x_MODEM_OOK_PDTC, 0x2A);
  Si446x_setProperty8(radio, Si446x_MODEM_OOK_CNT1, 0x85);
  Si446x_setProperty8(radio, Si446x_MODEM_OOK_MISC, 0x23);

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
  if(is_part_Si4463(handler->radio_part)) {
    Si446x_setProperty8(radio, Si446x_MODEM_DECIMATION_CFG2, 0x0C);
  }

  /* RSSI latching disabled. */
  Si446x_setProperty8(radio, Si446x_MODEM_RSSI_CONTROL, 0x00);

  /* RX IF filter coefficients. */
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

/**
 *
 */
static void Si446x_setModem2FSK_TX(const radio_unit_t radio,
		const uint32_t speed) {
    // Setup the NCO modulo and oversampling mode
    uint32_t s = Si446x_CCLK / 10;
    uint8_t f3 = (s >> 24) & 0xFF;
    uint8_t f2 = (s >> 16) & 0xFF;
    uint8_t f1 = (s >>  8) & 0xFF;
    uint8_t f0 = (s >>  0) & 0xFF;
    Si446x_setProperty32(radio, Si446x_MODEM_TX_NCO_MODE, f3, f2, f1, f0);

    // Setup the NCO data rate for 2FSK
    Si446x_setProperty24(radio, Si446x_MODEM_DATA_RATE,
                         (uint8_t)(speed >> 16),
                         (uint8_t)(speed >> 8), (uint8_t)speed);

    // Use 2FSK from FIFO (PH)
    Si446x_setProperty8(radio, Si446x_MODEM_MOD_TYPE, 0x02);

    /* Set PH bit order for 2FSK. */
    Si446x_setProperty8(radio, Si446x_PKT_CONFIG1, 0x01);
}


/**
 * Radio Settings
 */
static uint8_t __attribute__((unused)) Si446x_getChannel(const radio_unit_t radio) {
  const uint8_t state_info[] = {Si446x_REQUEST_DEVICE_STATE};
  uint8_t rxData[4];
  Si446x_read(radio, state_info, sizeof(state_info), rxData, sizeof(rxData));
  return rxData[3];
}

/*
 * Radio FIFO
 */

static void Si446x_writeFIFO(const radio_unit_t radio,
		uint8_t *msg, uint8_t size) {
  uint8_t write_fifo[size+1];
  write_fifo[0] = Si446x_WRITE_TX_FIFO;
  memcpy(&write_fifo[1], msg, size);
  Si446x_write(radio, write_fifo, size+1);
}

static uint8_t Si446x_getTXfreeFIFO(const radio_unit_t radio) {
  const uint8_t fifo_info[] = {Si446x_FIFO_INFO, 0x00};
  uint8_t rxData[4];
  Si446x_read(radio, fifo_info, sizeof(fifo_info), rxData, sizeof(rxData));
  return rxData[3];
}

/*
 *  Radio States
 */

radio_signal_t Si446x_getCurrentRSSI(const radio_unit_t radio) {
  /* Get status. Leave any pending interrupts intact. */
    const uint8_t status_info[] = {Si446x_GET_MODEM_STATUS, 0xEF};
    uint8_t rxData[11];
    Si446x_read(radio, status_info, sizeof(status_info), rxData, sizeof(rxData));
    return rxData[4];
}

static uint8_t Si446x_getState(const radio_unit_t radio) {
  const uint8_t state_info[] = {Si446x_REQUEST_DEVICE_STATE};
  uint8_t rxData[4];
  Si446x_read(radio, state_info, sizeof(state_info), rxData, sizeof(rxData));
  return rxData[2] & 0xF;
}

static void Si446x_setTXState(const radio_unit_t radio, uint8_t chan, uint16_t size){
  uint8_t change_state_command[] = {Si446x_START_TX, chan,
                                    (Si446x_STATE_READY << 4),
                                    (size >> 8) & 0x1F, size & 0xFF};
  Si446x_write(radio, change_state_command, sizeof(change_state_command));
}

static void Si446x_setReadyState(const radio_unit_t radio) {
  const uint8_t change_state_command[] = {Si446x_CHANGE_STATE,
                                          Si446x_STATE_READY};
  Si446x_write(radio, change_state_command, sizeof(change_state_command));
}

static void Si446x_setRXState(const radio_unit_t radio, uint8_t chan){
  const uint8_t change_state_command[] = {Si446x_START_RX, chan, 0x00, 0x00,
                                          0x00, 0x00, 0x08, 0x08};
  Si446x_write(radio, change_state_command, sizeof(change_state_command));
}

static void Si446x_setStandbyState(const radio_unit_t radio) {
  const uint8_t change_state_command[] = {Si446x_CHANGE_STATE,
                                          Si446x_STATE_STANDBY};
  Si446x_write(radio, change_state_command, sizeof(change_state_command));
}

/**
 *
 */
void Si446x_radioStandby(const radio_unit_t radio) {
  Si446x_setStandbyState(radio);
}

/**
 * The GPIO connected to radio SDN is set high in board initialization.
 * Thus the radio is in shutdown following board initialization.
 * Si446x GPIO1 is configured to output CTS (option 8) during POR.
 * We use the MCU GPIO connected to radio GPIO1 to check CTS here.
 *
 * Radio init is performed in the radio manager thread init stage.
 * The radio GPIOs can be reconfigured after radio init is complete.
 */
bool Si446x_radioStartup(const radio_unit_t radio) {

  TRACE_INFO("SI   > Enable radio %i", radio);

  /* Assert SDN low to perform radio POR wakeup. */
  palClearLine(Si446x_getConfig(radio)->sdn);

  /*
   * Set MCU GPIO input for POR and CTS of radio from GPIO0 and GPIO1.
   */
  palSetLineMode(Si446x_getConfig(radio)->gpio0, PAL_MODE_INPUT_PULLDOWN);
  palSetLineMode(Si446x_getConfig(radio)->gpio1, PAL_MODE_INPUT_PULLDOWN);

  /* Wait for transceiver to wake up (maximum wakeup time is 6mS).
   * During start up the POR state is on GPIO0.
   * This goes from zero to one when POR completes.
   * We could test this but for now just use a delay.
   */
  chThdSleep(TIME_MS2I(10));
  /* Return state of CTS after delay. */
  return pktReadGPIOline(Si446x_getConfig(radio)->gpio1) == PAL_HIGH;
}

/**
 * The radio is shutdown by setting SDN high.
 */
void Si446x_radioShutdown(const radio_unit_t radio) {
  TRACE_INFO("SI   > Disable radio %i", radio);
  packet_svc_t *handler = pktGetServiceObject(radio);

  palSetLine(Si446x_getConfig(radio)->sdn);
  handler->radio_init = false;
  chThdSleep(TIME_MS2I(1));
}

/*
 * Radio TX/RX
 */


/**
 * Get CCA over measurement interval.
 * Algorithm counts CCA pulses per millisecond (in systick time slices).
 * If more than one pulse per millisecond is counted then CCA is not true.
 */
static bool Si446x_checkCCAthreshold(const radio_unit_t radio, uint8_t ms) {
  /* Get the CCA line. */
  ioline_t cca_line = Si446x_getConfig(radio)->nirq;
  uint16_t cca = 0;
  /* Measure sliced CCA instances in period. */
  for(uint16_t i = 0; i < (ms * TIME_MS2I(1)); i++) {
    cca += Si446x_getCCA(cca_line);
    /* Sleep one tick. */
    chThdSleep(1);
  }
  /* Return result. */
  return cca > ms;
}

/**
 * Wait for a clear time slot and initiate packet transmission.
 */
static bool Si446x_transmit(const radio_unit_t radio,
                            const radio_freq_t freq,
                            const channel_hz_t step,
                            const radio_ch_t chan,
                            const radio_pwr_t power,
                            const uint16_t size,
                            const radio_squelch_t rssi,
                            sysinterval_t cca_timeout) {

  /* Get an absolute operating frequency in Hz. */
  radio_freq_t op_freq = pktComputeOperatingFrequency(radio, freq,
                                                      step, chan, RADIO_TX);

  if(op_freq == FREQ_INVALID) {
    TRACE_ERROR("SI   > Frequency out of range");
    TRACE_ERROR("SI   > abort transmission");
    return false;
  }

  /* Switch to ready state if receive is active. */
  if(Si446x_getState(radio) == Si446x_STATE_RX) {
    TRACE_INFO("SI   > Switch Si446x to ready state");
    Si446x_setReadyState(radio);
    chThdSleep(TIME_MS2I(1));
  }

  /* Frequency is an absolute frequency in Hz. */
  Si446x_setBandParameters(radio, op_freq, step);

  /* Check for blind send request. */
  if(rssi != PKT_SI446X_NO_CCA_RSSI) {
    Si446x_setProperty8(radio, Si446x_MODEM_RSSI_THRESH, rssi);

    /* Listen on the TX frequency. */
    Si446x_setRXState(radio, chan);
    /* Wait for RX state. */
    while(Si446x_getState(radio) != Si446x_STATE_RX) {
      chThdSleep(TIME_MS2I(1));
    }
    /* Minimum timeout for CCA is 1 second. */
    if(cca_timeout < TIME_S2I(1)) {
      TRACE_WARN("SI   > Minimum CCA wait time forced to 1 second,"
          " %d ms was specified", chTimeI2MS(cca_timeout));
      cca_timeout = TIME_S2I(1);
    }

    /* Try to get clear channel. */
    TRACE_INFO( "SI   > Wait up to %.1f seconds for CCA on"
        " %d.%03d MHz",
        (float32_t)(TIME_I2MS(cca_timeout) / 1000),
        op_freq/1000000, (op_freq%1000000)/1000);
#define CCA_VALID_TIME_MS   50
    sysinterval_t t0 = chVTGetSystemTime();
    while((Si446x_getState(radio) != Si446x_STATE_RX
        || Si446x_checkCCAthreshold(radio, CCA_VALID_TIME_MS))
        && chVTIsSystemTimeWithinX(t0, t0 + cca_timeout)) {
      chThdSleep(TIME_MS2I(1));
    }
    /* Clear channel timing. */
    TRACE_INFO( "SI   > CCA attained in %d milliseconds",
                chTimeI2MS(chVTTimeElapsedSinceX(t0)));
  }

  // Transmit
  TRACE_INFO("SI   > Tune Si446x to %d.%03d MHz (TX)",
             op_freq/1000000, (op_freq%1000000)/1000);
  Si446x_setReadyState(radio);
  while(Si446x_getState(radio) != Si446x_STATE_READY) {
    chThdSleep(TIME_MS2I(1));
  }
  /* Set power level and start transmit. */
  Si446x_setPowerLevel(radio, power);
  Si446x_setTXState(radio, chan, size);

  // Wait until transceiver enters transmit state
  /* TODO: Make a function to handle timeout on fail to reach state. */
  while(Si446x_getState(radio) != Si446x_STATE_TX) {
    chThdSleep(TIME_MS2I(1));
  }
  return true;
}

/*
 *
 */
bool Si446x_receiveNoLock(const radio_unit_t radio,
                          radio_freq_t freq,
                          channel_hz_t step,
                          radio_ch_t channel,
                          radio_squelch_t rssi,
                          radio_mod_t mod) {

  radio_freq_t op_freq = pktComputeOperatingFrequency(radio, freq,
                                                      step, channel,
                                                      RADIO_RX);
  if(op_freq == FREQ_INVALID) {
    TRACE_ERROR("SI   > Frequency out of range");
    TRACE_ERROR("SI   > abort transmission");
    return false;
  }

  uint16_t tot = 0;
  // Wait until transceiver finishes transmission (if there is any)
  while(Si446x_getState(radio) == Si446x_STATE_TX) {
    chThdSleep(TIME_MS2I(10));
    if(tot++ < 500)
      continue;

    /* Remove TX state. */
    Si446x_setReadyState(radio);
    TRACE_ERROR("SI   > Timeout waiting for TX state end");
    TRACE_ERROR("SI   > Attempt start of receive");
    break;
  }

  /* Configure radio for modulation type. */
  if(mod == MOD_AFSK) {
      Si446x_setModemAFSK_RX(radio);
  } else {
      TRACE_ERROR("SI   > Modulation type not supported in receive");
      TRACE_ERROR("SI   > abort reception");
      return false;
  }

  TRACE_INFO("SI   > Tune Si446x to %d.%03d MHz (RX)",
             op_freq/1000000, (op_freq%1000000)/1000);

  /* Set squelch level. */
  Si446x_setProperty8(radio, Si446x_MODEM_RSSI_THRESH, rssi);

  /* Start the receiver. */
  Si446x_setRXState(radio, channel);

  /* Wait for the receiver to start. */
  while(Si446x_getState(radio) != Si446x_STATE_RX)
      chThdSleep(TIME_MS2I(1));
  return true;
}

/*
 * Start or restore reception.
 *
 * return true if RX was enabled and/or resumed OK.
 * return false if RX was not enabled.
 */
bool Si4464_enableReceive(const radio_unit_t radio,
                          const radio_freq_t rx_frequency,
                          const channel_hz_t rx_step,
                          const radio_ch_t rx_chan,
                          const radio_squelch_t rx_rssi,
                          const radio_mod_t rx_mod) {

  /* Get an absolute operating frequency in Hz. */
  radio_freq_t op_freq = pktComputeOperatingFrequency(radio,
                                                      rx_frequency,
                                                      rx_step,
                                                      rx_chan,
                                                      RADIO_RX);


  TRACE_INFO( "SI   > Enable reception %d.%03d MHz (ch %d),"
              " RSSI %d, %s",
              op_freq/1000000, (op_freq % 1000000)/1000,
              rx_chan,
              rx_rssi, getModulation(rx_mod));

  /* Initialize radio before any commands as it may have been powered down. */
  Si446x_conditional_init(radio);

  /* Frequency must be an absolute frequency in Hz. */
  if(!Si446x_setBandParameters(radio, op_freq, rx_step))
    return false;

  return Si446x_receiveNoLock(radio, op_freq, rx_step,
                             rx_chan, rx_rssi, rx_mod);
}

/*
 * Called when a packet RX channel is closed.
 * If the receiver is active put it into standby.
 */
void Si446x_disableReceive(const radio_unit_t radio) {
  /* FIXME: Should have timeout. */
  if(Si446x_getState(radio) == Si446x_STATE_RX) {
    Si446x_radioStandby(radio);
  }
}

/*
 *
 */
void Si446x_terminateReceive(const radio_unit_t radio) {
  /* FIXME: Should provide status. */
  if(Si446x_getState(radio) == Si446x_STATE_RX) {
    Si446x_setReadyState(radio);
    while(Si446x_getState(radio) == Si446x_STATE_RX);
  }
}

/*
 *
 */
void Si446x_waitTransmitEnd(const radio_unit_t radio) {
  /* FIXME: Should have timeout. */
  while(Si446x_getState(radio) == Si446x_STATE_TX);
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

/*
 * Simple AFSK send thread with minimized buffering and burst send capability.
 * Uses an iterator to size NRZI output and allocate suitable size buffer.
 *
 */
THD_FUNCTION(bloc_si_fifo_feeder_afsk, arg) {
  radio_task_object_t *rto = arg;

  radio_unit_t radio = rto->handler->radio;

  packet_t pp = rto->packet_out;

  chDbgAssert(pp != NULL, "no packet in radio task");

  if(pktLockRadioTransmit(radio, TIME_INFINITE) == MSG_RESET) {
    TRACE_ERROR("SI   > AFSK TX reset from radio acquisition");
    /* Free packet object memory. */
    pktReleaseBufferChain(pp);

    /* Schedule thread and task object memory release. */
    pktLLDradioSendComplete(rto, chThdGetSelfX());

    /* Exit thread. */
    chThdExit(MSG_RESET);
    /* We never arrive here. */
    chSysHalt("TX AFSK exit");
  }

  /* Initialize radio before any commands as it may have been powered down. */
  Si446x_conditional_init(radio);

  /* Base frequency is an absolute frequency in Hz. */
  Si446x_setBandParameters(radio, rto->base_frequency,
                           rto->step_hz);

  /* Set 446x back to READY. */
  Si446x_terminateReceive(radio);

  /* Set the radio for AFSK upsampled mode. */
  Si446x_setModemAFSK_TX(radio);

  /* Initialize variables for AFSK encoder. */
  virtual_timer_t send_timer;

  chVTObjectInit(&send_timer);
  msg_t exit_msg;
  tx_iterator_t iterator;

  /*
   * Use the specified CCA RSSI level.
   * CCA level will be set to blind send after first packet.
   */
  radio_squelch_t rssi = rto->squelch;

  do {

    /*
     * Set NRZI encoding format.
     * Iterator object.
     * Packet reference.
     * Preamble length (HDLC flags)
     * Postamble length (HDLC flags)
     * Tail length (HDLC zeros)
     * Scramble off
     */
    pktStreamIteratorInit(&iterator, pp, 30, 10, 10, false);

    uint16_t all = pktStreamEncodingIterator(&iterator, NULL, 0);

    if(all == 0) {
      /* Nothing encoded. Release packet send object. */

      TRACE_ERROR("SI   > AFSK TX no NRZI data encoded");

      /* Free packet object memory. */
      pktReleaseBufferChain(pp);

      /* Schedule thread and task object memory release. */
      pktLLDradioSendComplete(rto, chThdGetSelfX());

      /* Unlock radio. */
      pktUnlockRadioTransmit(radio);

      /* Exit thread. */
      chThdExit(MSG_ERROR);
      /* We never arrive here. */
    }
    /* Allocate buffer and perform NRZI encoding. */
    uint8_t layer0[all];
    pktStreamEncodingIterator(&iterator, layer0, all);

    all *= SAMPLES_PER_BAUD;
    /* Reset TX FIFO in case some remnant unsent data is left there. */
    const uint8_t reset_fifo[] = {0x15, 0x01};
    Si446x_write(radio, reset_fifo, 2);

    up_sampler_t upsampler = {0};
    upsampler.phase_delta = PHASE_DELTA_1200;

    /* Maximum amount of FIFO data when using combined TX+RX (safe size). */
    uint8_t localBuffer[Si446x_FIFO_COMBINED_SIZE];

    /* Get the FIFO buffer amount currently available. */
    uint8_t free = Si446x_getTXfreeFIFO(radio);

    /* Calculate initial FIFO fill. */
    uint16_t c = (all > free) ? free : all;

    /*
     * Start transmission timeout timer.
     * If the 446x gets locked up we'll exit TX and release packet object.
     */
    chVTSet(&send_timer, TIME_S2I(10),
            (vtfunc_t)Si446x_transmitTimeoutI, chThdGetSelfX());

    /* The exit message if all goes well. */
    exit_msg = MSG_OK;

    /* Initial FIFO load. */
    for(uint16_t i = 0;  i < c; i++)
      localBuffer[i] = Si446x_getUpsampledNRZIbits(&upsampler, layer0);
    Si446x_writeFIFO(radio, localBuffer, c);

    uint8_t lower = 0;

    /* Request start of transmission. */
    if(Si446x_transmit(radio,
                       rto->base_frequency,
                       rto->step_hz,
                       rto->channel,
                       rto->tx_power,
                       all,
                       rssi,
                       TIME_S2I(10))) {

      /* Feed the FIFO while data remains to be sent. */
      while((all - c) > 0) {
        /* Get TX FIFO free count. */
        uint8_t more = Si446x_getTXfreeFIFO(radio);
        /* Update the FIFO free low water mark. */
        lower = (more > lower) ? more : lower;

        /* If there is more free than we need use remainder only. */
        more = (more > (all - c)) ? (all - c) : more;

        /* Load the FIFO. */
        for(uint16_t i = 0; i < more; i++)
          localBuffer[i] = Si446x_getUpsampledNRZIbits(&upsampler, layer0);
        Si446x_writeFIFO(radio, localBuffer, more); // Write into FIFO
        c += more;

        /*
         * Wait for a timeout event during up-sampled NRZI send.
         * Time delay allows ~SAMPLES_PER_BAUD bytes to be consumed from FIFO.
         * If no timeout event go back and load more data to FIFO.
         */
        eventmask_t evt = chEvtWaitAnyTimeout(SI446X_EVT_TX_TIMEOUT,
                                              chTimeUS2I(833 * 8));
        if(evt) {
          /* Force 446x out of TX state. */
          Si446x_setReadyState(radio);
          exit_msg = MSG_TIMEOUT;
          break;
        }
      }
    } else {
      /* Transmit start failed. */
      TRACE_ERROR("SI   > Transmit start failed");
      exit_msg = MSG_ERROR;
    } /* End transmit. */
    chVTReset(&send_timer);

    /*
     * If nothing went wrong wait for TX to finish.
     * Else don't wait.
     */
    while(Si446x_getState(radio) == Si446x_STATE_TX && exit_msg == MSG_OK) {
      /* TODO: Add an absolute timeout on this. */
      /* Sleep for an AFSK byte time. */
      chThdSleep(chTimeUS2I(833 * 8));
      continue;
    }

    /* No CCA on subsequent packet sends. */
    rssi = PKT_SI446X_NO_CCA_RSSI;

    if(lower > (free / 2)) {
      /*
       *  Warn when free level is more than 50% of FIFO size.
       *  This means the FIFO is not being filled fast enough.
       */
      TRACE_WARN("SI   > AFSK TX FIFO dropped below safe threshold %i", lower);
    }
    /* Get the next linked packet to send. */
    packet_t np = pp->nextp;
    if(exit_msg == MSG_OK) {
      /* Send was OK. Release the just completed packet. */
      pktReleaseBufferObject(pp);
    } else {
      /* Send failed so release any queue and terminate. */
      pktReleaseBufferChain(pp);
      np = NULL;
    }

    /* Process next packet. */
    pp = np;
  } while(pp != NULL);

  /* Save status in case a callback requires it. */
  rto->result = exit_msg;

  /* Finished send so schedule thread memory and task object release. */
  pktLLDradioSendComplete(rto, chThdGetSelfX());

  /* Unlock radio. */
  pktUnlockRadioTransmit(radio);

  /* Exit thread. */
  chThdExit(exit_msg);
}

/*
 *
 */
bool Si446x_blocSendAFSK(radio_task_object_t *rt) {

    thread_t *afsk_feeder_thd = NULL;

    /* Create a send thread name which includes the sequence number. */
    char tx_thd_name[16];
    chsnprintf(tx_thd_name, sizeof(tx_thd_name),
               "tx_afsk_%03i", rt->tx_seq_num);

    afsk_feeder_thd = chThdCreateFromHeap(NULL,
                THD_WORKING_AREA_SIZE(SI_AFSK_FIFO_MIN_FEEDER_WA_SIZE),
                tx_thd_name,
                NORMALPRIO - 10,
                bloc_si_fifo_feeder_afsk,
                rt);

    if(afsk_feeder_thd == NULL) {
      TRACE_ERROR("SI   > Unable to create AFSK transmit thread");
      return false;
    }
    return true;
}

/*
 * AFSK Receiver
 */

void Si446x_stopDecoder(void) {
    // TODO: Nothing yet here
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

  packet_t pp = rto->packet_out;

  chDbgAssert(pp != NULL, "no packet in radio task");

  /* Check for MSG_RESET which means system has forced radio release. */
  if(pktLockRadioTransmit(radio, TIME_INFINITE) == MSG_RESET) {
    TRACE_ERROR("SI   > 2FSK TX reset from radio acquisition");
    /* Free packet object memory. */
    pktReleaseBufferChain(pp);

    /* Schedule thread and task object memory release. */
    pktLLDradioSendComplete(rto, chThdGetSelfX());

    /* Exit thread. */
    chThdExit(MSG_RESET);
    /* We never arrive here. */
  }

  /* Initialize radio before any commands as it may have been powered down. */
  Si446x_conditional_init(radio);

  /* Set 446x back to READY from RX (if active). */
  Si446x_terminateReceive(radio);

  /* Base frequency must be an absolute frequency in Hz. */
  Si446x_setBandParameters(radio, rto->base_frequency, rto->step_hz);

  /* Set parameters for 2FSK transmission. */
  Si446x_setModem2FSK_TX(radio, rto->tx_speed);

  /* Initialize variables for 2FSK encoder. */

  virtual_timer_t send_timer;

  chVTObjectInit(&send_timer);

  tx_iterator_t iterator;

  /* The exit message. */
  msg_t exit_msg;

  /*
   * Use the specified CCA RSSI level.
   * CCA will be set to blind send after first packet.
   */
  radio_squelch_t rssi = rto->squelch;

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

    if(all == 0) {
      /* Nothing encoded. Release packet send object. */
      TRACE_ERROR("SI   > 2FSK TX no NRZI data encoded");

      /* Free packet object memory. */
      pktReleaseBufferChain(pp);

      rto->result = MSG_ERROR;

      /* Schedule thread and task object memory release. */
      pktLLDradioSendComplete(rto, chThdGetSelfX());

      /* Unlock radio. */
      pktUnlockRadioTransmit(radio);

      /* Exit thread. */
      chThdExit(MSG_ERROR);
      /* We never arrive here. */
    }
    /* Allocate buffer and perform NRZI encoding. */
    uint8_t layer0[all];

    pktStreamEncodingIterator(&iterator, layer0, all);

    /* Reset TX FIFO in case some remnant unsent data is left there. */
    const uint8_t reset_fifo[] = {0x15, 0x01};
    Si446x_write(radio, reset_fifo, 2);

    /* Get the FIFO buffer amount currently available. */
    uint8_t free = Si446x_getTXfreeFIFO(radio);

    /* Calculate initial FIFO fill. */
    uint16_t c = (all > free) ? free : all;

    /*
     * Start/re-start transmission timeout timer for this packet.
     * If the 446x gets locked up we'll exit TX and release packet object(s).
     */
    chVTSet(&send_timer, TIME_S2I(10),
            (vtfunc_t)Si446x_transmitTimeoutI, chThdGetSelfX());

    /* The exit message if all goes well. */
    exit_msg = MSG_OK;

    uint8_t *bufp = layer0;

    /* Initial FIFO load. */
    Si446x_writeFIFO(radio, bufp, c);
    bufp += c;
    uint8_t lower = 0;

    /* Request start of transmission. */
    if(Si446x_transmit(radio,
                       rto->base_frequency,
                       rto->step_hz,
                       rto->channel,
                       rto->tx_power,
                       all,
                       rssi,
                       TIME_S2I(10))) {
      /* Feed the FIFO while data remains to be sent. */
      while((all - c) > 0) {
        /* Get TX FIFO free count. */
        uint8_t more = Si446x_getTXfreeFIFO(radio);
        /* Update the FIFO free low water mark. */
        lower = (more > lower) ? more : lower;

        /* If there is more free than we need for send use remainder only. */
        more = (more > (all - c)) ? (all - c) : more;

        /* Load the FIFO. */
        Si446x_writeFIFO(radio, bufp, more); // Write into FIFO
        bufp += more;
        c += more;

        /*
         * Wait for a timeout event during up-sampled NRZI send.
         * Time delay allows ~10 bytes to be consumed from FIFO.
         * If no timeout event go back and load more data to FIFO.
         */
        eventmask_t evt = chEvtWaitAnyTimeout(SI446X_EVT_TX_TIMEOUT,
                                              chTimeUS2I(104 * 8 * 10));
        if(evt) {
          /* Force 446x out of TX state. */
          Si446x_setReadyState(radio);
          exit_msg = MSG_TIMEOUT;
          break;
        }
      }
    } else {
      /* Transmit start failed. */
      TRACE_ERROR("SI   > 2FSK transmit start failed");
      exit_msg = MSG_ERROR;
    }
    chVTReset(&send_timer);

    /*
     * If nothing went wrong wait for TX to finish.
     * Else don't wait.
     */
    while(Si446x_getState(radio) == Si446x_STATE_TX && exit_msg == MSG_OK) {
      /* TODO: Add an absolute timeout on this. */
      /* Sleep for a 2FSK byte time. */
      chThdSleep(chTimeUS2I(104 * 8 * 10));
      continue;
    }

    /* No CCA on subsequent packet sends. */
    rssi = PKT_SI446X_NO_CCA_RSSI;

    if(lower > (free / 2)) {
      /* Warn when free level is > 50% of FIFO size. */
      TRACE_WARN("SI   > AFSK TX FIFO dropped below safe threshold %i", lower);
    }
    /* Get the next linked packet to send. */
    packet_t np = pp->nextp;
    if(exit_msg == MSG_OK) {

      /* Send was OK. Release the just completed packet. */
      pktReleaseBufferObject(pp);
    } else {
      /* Send failed so release any queue and terminate. */
      pktReleaseBufferChain(pp);
      np = NULL;
    }

    /* Process next packet. */
    pp = np;
  } while(pp != NULL);

  /* Save status in case a callback requires it. */
  rto->result = exit_msg;

  /* Finished send so schedule thread memory and task object release. */
  pktLLDradioSendComplete(rto, chThdGetSelfX());

  /* Unlock radio. */
  pktUnlockRadioTransmit(radio);

  /* Exit thread. */
  chThdExit(exit_msg);
}

/*
 * Return true on send successfully enqueued.
 * Task object will be returned
 * Return false on failure
 */
bool Si446x_blocSend2FSK(radio_task_object_t *rt) {

  thread_t *fsk_feeder_thd = NULL;

  /* Create a send thread name which includes the sequence number. */
  char tx_thd_name[16];
  chsnprintf(tx_thd_name, sizeof(tx_thd_name),
             "tx_2fsk_%03i", rt->tx_seq_num);

  fsk_feeder_thd = chThdCreateFromHeap(NULL,
              THD_WORKING_AREA_SIZE(SI_FSK_FIFO_FEEDER_WA_SIZE),
              tx_thd_name,
              NORMALPRIO - 10,
              bloc_si_fifo_feeder_fsk,
              rt);

  if(fsk_feeder_thd == NULL) {
    TRACE_ERROR("SI   > Unable to create FSK transmit thread");
    return false;
  }
  return true;
}

/**
 * Used by collector. At the moment it collects for PKT_RADIO_1 only.
 * There should be an LLD API selecting the radio type via VMT etc.
 */
si446x_temp_t Si446x_getLastTemperature(const radio_unit_t radio) {
  return Si446x_getData(radio)->lastTemp;
}

/**
 *
 */
ICUDriver *Si446x_attachPWM(const radio_unit_t radio) {
  /* The RX_RAW_DATA input is routed to ICU timer channel.
   * TODO: The STM32 Alternate mode should be in the radio config data.
   * Then the ICU GPIO setting can be generalized.
   */
  //(void)pktSetLineModeICU(Si446x_getConfig(radio)->gpio1);
  pktSetGPIOlineMode(Si446x_getConfig(radio)->gpio1,
                     Si446x_getConfig(radio)->alt);

  /*
  * Set up GPIO port where the NIRQ from the radio is connected.
  * The NIRQ line is configured in the radio to output the CCA condition.
  */
  pktSetGPIOlineMode(Si446x_getConfig(radio)->nirq, PAL_MODE_INPUT_PULLUP);

  /*
   * Return the ICU this radio is assigned to.
   * TODO: Check that the ICU is not already taken?
   * This would only be a config error.
   * Packet channel control enforces single use of decoder PWM for AFSK.
   */
  return Si446x_getConfig(radio)->icu;
}

/**
 *
 */
bool Si446x_detachPWM(const radio_unit_t radio) {
  (void)radio;
  return true;
}

/**
 *
 */
const ICUConfig *Si446x_enablePWMevents(const radio_unit_t radio,
                          palcallback_t cb) {
  /* Set callback for squelch events. */
  palSetLineCallback(Si446x_getConfig(radio)->nirq, cb,
                     Si446x_getConfig(radio)->icu);

  /* Enabling events on both edges of CCA.*/
  palEnableLineEvent(Si446x_getConfig(radio)->nirq,
                     PAL_EVENT_MODE_BOTH_EDGES);

  return &Si446x_getConfig(radio)->cfg;
}

/**
 *
 */
void Si446x_disablePWMevents(radio_unit_t radio) {
  palDisableLineEvent(Si446x_getConfig(radio)->nirq);
}
/**
 *
 */
uint8_t Si446x_readCCA(const radio_unit_t radio) {
  return palReadLine(Si446x_getConfig(radio)->nirq);
}
