/*
    Aerospace Decoder - Copyright (C) 2018-2019 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

/**
 * @file        si446x.c
 * @brief       SiLabs 446x radio driver.
 *
 * @addtogroup  drivers
 * @{
 */

#include "pktconf.h"
#include "debug.h"
#include "radio.h"
#include "si4463_patch.h"
#include "tcxo.h"

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

static const uint8_t Si4463_Patch_Data_Array[] = {
        SI4463_PATCH_CMDS,
        0x00
 };

/*
 * @brief   The SPI configuration.
 * @note    The CS line is set dynamically per radio.
 * @note    The SPI LLD sets the SPI_CR1_MSTR bit
 * @note    Baud rate is PCLK/2 when SPI_CR1_BR = 0
 */
static SPIConfig ls_spicfg = {
    .cr1    = 0
};

/**
 * @brief Get pointer to the radio specific configuration.
 *
 * @param[in] radio radio unit ID
 *
 * @return pointer to radio data
 * @retval NULL if radio ID is invalid
 *
 * @api
 */
static const si446x_mcucfg_t *Si446x_getConfig(const radio_unit_t radio) {
  const radio_config_t *data = pktGetRadioData(radio);
  return (si446x_mcucfg_t *)data->cfg;
}

/**
 * @brief Get pointer to the radio volatile data.
 *
 * @param[in] radio radio unit ID
 *
 * @return pointer to radio volatile data
 * @retval NULL if radio ID is invalid
 *
 * @api
 */
static si446x_data_t *Si446x_getData(const radio_unit_t radio) {
  const radio_config_t *data = pktGetRadioData(radio);
return (si446x_data_t *)data->dat;
}

/**
 * @brief Acquire bus and set the select line in SPI configuration.
 *
 * @notapi
 */
static SPIDriver *Si446x_spiSetupBus(const radio_unit_t radio,
                                     SPIConfig *const cfg) {
  SPIDriver *spip = Si446x_getConfig(radio)->spi;
  palSetLine(Si446x_getConfig(radio)->cs);
  palSetLineMode(Si446x_getConfig(radio)->cs, PAL_MODE_OUTPUT_PUSHPULL);
  spiAcquireBus(spip);
  cfg->ssport = PAL_PORT(Si446x_getConfig(radio)->cs);
  cfg->sspad = PAL_PAD(Si446x_getConfig(radio)->cs);
  return spip;
}

/**
 * @brief   SPI write which uses CTS presented on radio GPIO1.
 * @note    Used when starting the radio up from shutdown state.
 * @pre     The MCU GPIO pin connected to 446x GPIO1 must be configured as input.
 *
 * @param[in] radio  radio unit ID
 * @param[in] txData pointer to data to write to SPI
 * @param[in] len    length of the data
 *
 * @return  Status of the operation
 * @retval  false on success
 * @retval  true  on failure
 *
 * @notapi
 */
static bool Si446x_writeBoot(const radio_unit_t radio,
                             const si446x_arg_t *txData, size_t len) {
  /* Write data via SPI with CTS checked via GPIO1. */

  /* Acquire bus and then start SPI. */
  SPIDriver *spip = Si446x_spiSetupBus(radio, &ls_spicfg);
  spiStart(spip, &ls_spicfg);

  /* Poll for CTS with timeout of 100mS. */
  ioline_t cts = Si446x_getConfig(radio)->gpio1;
  uint8_t timeout = 100;
  do {
      chThdSleep(TIME_MS2I(1));
  } while (palReadLine(cts) != PAL_HIGH && --timeout);

  if (timeout == 0) {
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
 * SPI write without consideration of CTS.
 * Used to write FIFO.
 * Returns false on success, true on fail.
 *
 * @notapi
 */
static bool Si446x_writeNoCTS(const radio_unit_t radio,
                               const si446x_arg_t *txData, size_t len) {
    /* Transmit data by SPI without CTS polling by command. */

    /* Acquire bus, get SPI Driver object and then start SPI. */
    SPIDriver *spip = Si446x_spiSetupBus(radio, &ls_spicfg);
    spiStart(spip, &ls_spicfg);

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
 * SPI write.
 * Returns false on success, true on fail.
 *
 * @notapi
 */
static bool Si446x_write(const radio_unit_t radio,
                               const si446x_arg_t *txData, const size_t len) {
    /* Transmit data by SPI with CTS polling by command. */

    /* Acquire bus, get SPI Driver object and then start SPI. */
    SPIDriver *spip = Si446x_spiSetupBus(radio, &ls_spicfg);
    spiStart(spip, &ls_spicfg);

    /* Poll for CTS only with timeout of ~100mS. */
    uint8_t timeout = 100;
    si446x_arg_t rx_ready[] = {Si446x_READ_CMD_BUFF_CMD, 0x00};
    do {
      spiSelect(spip);
      spiExchange(spip, 1, rx_ready, &rx_ready[1]);
      spiUnselect(spip);
      chThdSleep(TIME_MS2I(1));
    } while (rx_ready[1] != Si446x_COMMAND_CTS && --timeout);

    if (rx_ready[1] != Si446x_COMMAND_CTS) {
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
 * SPI write and wait for CTS after
 * Returns false on success, true on fail.
 *
 * @notapi
 */
static bool Si446x_writeWaitTrailingCTS(const radio_unit_t radio,
                               const si446x_arg_t *txData, const size_t len) {
    /* Transmit data by SPI with CTS polling by command. */

    /* Acquire bus, get SPI Driver object and then start SPI. */
    SPIDriver *spip = Si446x_spiSetupBus(radio, &ls_spicfg);
    spiStart(spip, &ls_spicfg);

    /* Poll for CTS only with timeout of ~100mS. */
    uint8_t timeout = 100;
    si446x_arg_t rx_ready[] = {Si446x_READ_CMD_BUFF_CMD, 0x00};
    do {
      spiSelect(spip);
      spiExchange(spip, 1, rx_ready, &rx_ready[1]);
      spiUnselect(spip);
      chThdSleep(TIME_MS2I(1));
    } while (rx_ready[1] != Si446x_COMMAND_CTS && --timeout);

    if (rx_ready[1] != Si446x_COMMAND_CTS) {
      /* Stop SPI and relinquish bus. */
      spiStop(spip);
      spiReleaseBus(spip);
      return true;
    }

    /* Transfer data. */
    spiSelect(spip);
    spiSend(spip, len, txData);
    spiUnselect(spip);

    /* Poll for CTS only with timeout of ~100mS. */
    timeout = 100;
    do {
      spiSelect(spip);
      spiExchange(spip, 1, rx_ready, &rx_ready[1]);
      spiUnselect(spip);
      chThdSleep(TIME_MS2I(1));
    } while (rx_ready[1] != Si446x_COMMAND_CTS && --timeout);

    /* Stop SPI and relinquish bus. */
    spiStop(spip);
    spiReleaseBus(spip);

    return rx_ready[1] != Si446x_COMMAND_CTS;
}

/**
 * SPI read which uses CTS on GPIO1.
 * Use this when first taking radio out of shutdown.
 * The MCU GPIO pin connected to 446x GPIO1 must be already configured.
 * Returns false on success, true on failure.
 *
 * @notapi
 */
static bool Si446x_readBoot(const radio_unit_t radio,
						const uint8_t* txData, const uint32_t txlen,
                        uint8_t* rxData, const uint32_t rxlen) {
    chDbgCheck(rxlen > 0);
    chDbgCheck(rxData != NULL);
    chDbgCheck(txlen > 0);
    chDbgCheck(txData != NULL);

    /* Acquire bus and get SPI Driver object. */
    SPIDriver *spip = Si446x_spiSetupBus(radio, &ls_spicfg);

    /* Poll for CTS with timeout of ~100mS. */
    ioline_t cts = Si446x_getConfig(radio)->gpio1;
    uint8_t timeout = 100;
    while (palReadLine(cts) != PAL_HIGH && --timeout) {
        chThdSleep(TIME_MS2I(1));
    }

    if (timeout == 0) {
      /* Relinquish bus. */
      spiReleaseBus(spip);
      return true;
    }

    /*
     * Now write command and any data.
     */
    spiStart(spip, &ls_spicfg);
    spiSelect(spip);
    spiSend(spip, txlen, txData);
    spiUnselect(spip);

    /* Poll for CTS with timeout of ~100ms. */
    timeout = 100;
    while (palReadLine(cts) != PAL_HIGH && --timeout) {
        chThdSleep(TIME_MS2I(1));
    }

    if (timeout == 0) {
      /* Stop SPI and relinquish bus. */
      spiStop(spip);
      spiReleaseBus(spip);
      return true;
    }

    /* Read the response. */
    si446x_arg_t rx_ready[] = {Si446x_READ_CMD_BUFF_CMD, 0x00};
    si446x_reply_t reply[rxlen + 2];
    spiSelect(spip);
    spiExchange(spip, sizeof(reply), rx_ready, reply);
    spiUnselect(spip);

    /* Stop SPI and relinquish bus. */
    spiStop(spip);
    spiReleaseBus(spip);

    /* Transfer reply data w/o CTS to user buffer. */
    memcpy(rxData, &reply[2], rxlen);

    return false;
}

/**
 * Read data from Si446x.
 * Returns false on success, true on failure.
 *
 * @notapi
 */
static bool Si446x_read(const radio_unit_t radio,
		                const uint8_t* txData, const uint32_t txlen,
                        uint8_t* rxData, const uint32_t rxlen) {
    chDbgCheck(rxlen > 0);
    chDbgCheck(rxData != NULL);
    chDbgCheck(txlen > 0);
    chDbgCheck(txData != NULL);

    /* Acquire bus and then start SPI. */
    SPIDriver *spip = Si446x_spiSetupBus(radio, &ls_spicfg);
    spiStart(spip, &ls_spicfg);

    /*
     * Poll command buffer waiting for CTS from the READ_CMD_BUFF command.
     * This command does not itself cause CTS to report busy.
     * Allocate a buffer to use for CTS check.
     * Timeout after ~100mS waiting for CTS.
     */
    uint8_t timeout = 100;
    si446x_arg_t rx_ready[] = {Si446x_READ_CMD_BUFF_CMD, 0x00};
    do {
      spiSelect(spip);
      spiExchange(spip, 1, rx_ready, &rx_ready[1]);
      spiUnselect(spip);
      chThdSleep(TIME_MS2I(1));
    } while (rx_ready[1] != Si446x_COMMAND_CTS && --timeout);

    if (timeout == 0) {
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
     * Timeout after ~100mS waiting for CTS.
     */

    si446x_reply_t reply[rxlen + 2];
    timeout = 100;
    do {
      spiSelect(spip);
      spiExchange(spip, sizeof(reply), rx_ready, reply);
      spiUnselect(spip);
      chThdSleep(TIME_MS2I(1));
    } while (reply[1] != Si446x_COMMAND_CTS && --timeout);

    /* Stop SPI and relinquish bus. */
    spiStop(spip);
    spiReleaseBus(spip);
    
    /* Transfer reply data w/o CTS to user buffer. */
    memcpy(rxData, &reply[2], rxlen);

    return timeout == 0;
}

/**
 * @brief Read fast response register data from Si446x.
 * @note  Takes an index from 0 - 3 for FRR A-D.
 * @note  Will read FRR in circular fashion to buffer if index wraps around.
 *
 * @param[in] radio     radio unit ID.
 * @param[in] index     index of FRR to start from (0..3 = A..D)
 * @param[in  rxData    array where FRR read results are stored.
 * @param[in] rxlen     number of FRR to read
 *
 * @return  status of operation.
 * @retval  false on success.
 * @retval  true on parameter error.
 */
bool Si446x_readFRR(const radio_unit_t radio, const uint8_t index,
                           uint8_t *const rxData, const uint8_t rxlen) {
  chDbgCheck(index < 4 && rxlen > 1 && rxlen < 4);

  si446x_arg_t gpio_frr_command[] = {Si446x_READ_FRR_A_CMD + index};

  /* Acquire bus and then start SPI. */
  SPIDriver *spip = Si446x_spiSetupBus(radio, &ls_spicfg);
  spiStart(spip, &ls_spicfg);

  /* Send FRR read command and read back register(s). */
  spiSelect(spip);
  spiSend(spip, sizeof(gpio_frr_command), gpio_frr_command);

  spiReceive(spip, rxlen, rxData);
  spiUnselect(spip);

  /* Stop SPI and relinquish bus. */
  spiStop(spip);
  spiReleaseBus(spip);

  return false;
}

/**
 * @brief Configure transceiver GPIOs.
 */
static bool Si446x_configureGPIO(const radio_unit_t radio,
                                 const si446x_gpio_t *gpio) {

  si446x_arg_t gpio_pin_cfg_command[sizeof(si446x_gpio_t) + 1] = {
      Si446x_GPIO_PIN_CFG_CMD};

  memcpy(&gpio_pin_cfg_command[1], gpio, sizeof(si446x_gpio_t));
  return Si446x_write(radio, gpio_pin_cfg_command, sizeof(gpio_pin_cfg_command));
}

/**
 * Set property with size parameter.
 */
static bool __attribute__((unused)) Si446x_setPropertyN(const radio_unit_t radio,
        uint16_t reg, si446x_arg_t *val, size_t num) {
  chDbgCheck(num > 0 && num <= 12);
  si446x_arg_t msg[4 + num];
  msg[0] = Si446x_SET_PROPERTY_CMD;
  msg[1] = (reg >> 8) & 0xFF;
  msg[2] = num;
  msg[3] = reg & 0xFF;
  memcpy(&msg[4], val, num);
  return Si446x_write(radio, msg, sizeof(msg));
}

/**
 *
 */
static bool Si446x_setProperty8(const radio_unit_t radio,
		uint16_t reg, uint8_t val) {
  si446x_arg_t msg[] = {Si446x_SET_PROPERTY_CMD,
                     (reg >> 8) & 0xFF, 0x01, reg & 0xFF, val};
  return Si446x_write(radio, msg, sizeof(msg));
}

/**
 *
 */
static bool Si446x_setProperty16(const radio_unit_t radio,
		uint16_t reg, uint8_t val1, uint8_t val2) {
  si446x_arg_t msg[] = {Si446x_SET_PROPERTY_CMD,
                     (reg >> 8) & 0xFF, 0x02, reg & 0xFF, val1, val2};
  return Si446x_write(radio, msg, sizeof(msg));
}

/**
 *
 */
static bool Si446x_setProperty24(const radio_unit_t radio,
		                         uint16_t reg, uint8_t val1,
                                 uint8_t val2, uint8_t val3) {
  si446x_arg_t msg[] = {Si446x_SET_PROPERTY_CMD,
                     (reg >> 8) & 0xFF, 0x03, reg & 0xFF, val1, val2, val3};
  return Si446x_write(radio, msg, sizeof(msg));
}

/**
 *
 */
static bool Si446x_setProperty32(const radio_unit_t radio,
		                         uint16_t reg, uint8_t val1,
                                 uint8_t val2, uint8_t val3, uint8_t val4) {
    uint8_t msg[] = {Si446x_SET_PROPERTY_CMD,
                     (reg >> 8) & 0xFF, 0x04, reg & 0xFF,
                     val1, val2, val3, val4};
    return Si446x_write(radio, msg, sizeof(msg));
}

#if Si446x_USE_INTERRUPTS
/**
 * Get interrupt status into provided reply field.
 *
 * STATUS bits indicate the current state of an internal interrupt event.
 * STATUS is cleared automatically upon termination or cessation of the corresponding internal interrupt event.
 * In the case of RSSI for example STATUS will be cleared upon entry into RX state or if RX is restarted.
 *
 * PENDING bits latch the rising edge of the corresponding STATUS.
 * PENDING bits are not cleared until explicitly done so with a command.
 *
 * In this function pending status of interrupts may be cleared or left intact.
 * When PENDING interrupts are cleared the reply stream reflects status prior to clearing.
 */
static bool Si446x_getInterruptStatus(const radio_unit_t radio,
                               si446x_reply_t *status, bool clear) {
  uint8_t status_cmd[] = {Si446x_GET_INT_STATUS_CMD, 0xFF, 0xFF, 0x7F};
  if (clear)
    memset(&status_cmd[1], 0x00, sizeof(status_cmd) - 1);
  return Si446x_read(radio, status_cmd, sizeof(status_cmd), status,
                     Si446x_GET_INT_STATUS_REPLY_SIZE);
}
#endif

/**
 *
 */
static bool Si446x_getModemStatus(const radio_unit_t radio,
                                    si446x_reply_t *status,
                                    size_t size) {
  /* Get status. Leave any pending interrupts intact. */
  const uint8_t status_info[] = {Si446x_GET_MODEM_STATUS_CMD, 0xFF};
  return Si446x_read(radio, status_info, sizeof(status_info), status, size);
}

/**
 * @brief  Reads the current RSSI value from modem status.
 *
 * @api
 */
radio_signal_t Si446x_getCurrentRSSI(const radio_unit_t radio) {
  /* Get status and return RSSI value. */
  si446x_reply_t rxData[Si446x_GET_MODEM_STATUS_REPLY_SIZE];
  if (Si446x_getModemStatus(radio, rxData, sizeof(rxData)))
      return (radio_signal_t)0xFF;
  return (radio_signal_t)rxData[2];
}

/**
 * Get device state and channel optionally
 */
static bool Si446x_requestDeviceState(const radio_unit_t radio,
                                                si446x_state_t *s,
                                                radio_ch_t *c) {
  const si446x_arg_t state_info[] = {Si446x_REQUEST_DEVICE_STATE_CMD};
  si446x_reply_t rxData[Si446x_REQUEST_DEVICE_STATE_REPLY_SIZE];
  if (Si446x_read(radio, state_info, sizeof(state_info),
                  rxData, sizeof(rxData)))
      return true;
  *s = (si446x_state_t)(rxData[0] & 0xF);
  if (c != NULL)
    *c = (si446x_state_t)(rxData[1]);
  return false;
}

#if 0
/**
 * Start TX.
 * Returns false on success, true on IO fail.
 */
static bool Si446x_startPacketTX(const radio_unit_t radio,
                                 const radio_ch_t chan,
                                 const radio_payload_t size) {
  si446x_arg_t change_state_command[] = {Si446x_START_TX_CMD, chan,
                                    (Si446x_READY << 4),
                                    (size >> 8) & 0x1F, size & 0xFF};
  return Si446x_write(radio, change_state_command, sizeof(change_state_command));
}
#endif
/**
 * Request start of TX and wait for TX to start (wait on trailing CTS).
 * Returns false on success, true on IO error.
 */
static bool Si446x_startPacketTXWaitCTS(const radio_unit_t radio,
                                 const radio_ch_t chan,
                                 const radio_payload_t size) {
  si446x_arg_t change_state_command[] = {Si446x_START_TX_CMD, chan,
                                    (Si446x_READY << 4),
                                    (size >> 8) & 0x1F, size & 0xFF};
  return Si446x_writeWaitTrailingCTS(radio, change_state_command,
                             sizeof(change_state_command));
}

/**
 * Set ready state.
 * Returns false on success, true on IO error.
 */
static bool Si446x_setReadyState(const radio_unit_t radio) {
  const si446x_arg_t change_state_command[] = {Si446x_CHANGE_STATE_CMD,
                                          Si446x_READY};
  return Si446x_write(radio, change_state_command, sizeof(change_state_command));
}

/**
 * Return false on success, true on IO error.
 */
static bool Si446x_startRXState(const radio_unit_t radio,
                                const radio_ch_t chan) {
  const si446x_arg_t change_state_command[] = {Si446x_START_RX_CMD, chan, 0x00, 0x00,
                                          0x00,
                                          Si446x_REMAIN,
                                          Si446x_REMAIN,
                                          Si446x_REMAIN};
  return Si446x_write(radio, change_state_command, sizeof(change_state_command));
}


/**
 * @brief Set 446x into RX state and wait for CTS confirming RX
 * Return false on success, true on IO error.
 */
static bool Si446x_startRXStateWaitCTS(const radio_unit_t radio,
                                const radio_ch_t chan) {
  const si446x_arg_t change_state_command[] = {Si446x_START_RX_CMD, chan, 0x00, 0x00,
                                          0x00,
                                          Si446x_REMAIN,
                                          Si446x_REMAIN,
                                          Si446x_REMAIN};
  return Si446x_writeWaitTrailingCTS(radio, change_state_command, sizeof(change_state_command));
}

/**
 * @brief  Set radio into standby state.
 * Returns false on success, true on IO error.
 */
static bool Si446x_setStandbyState(const radio_unit_t radio) {
  const si446x_arg_t change_state_command[] = {Si446x_CHANGE_STATE_CMD,
                                          Si446x_STANDBY};
  return Si446x_write(radio, change_state_command, sizeof(change_state_command));
}

/**
 * @brief  Get temperature of chip.
 * Return false on success, true on IO error.
 */
static bool Si446x_getTemperature(const radio_unit_t radio) {
  const si446x_arg_t txData[] = {Si446x_GET_ADC_READING_CMD,
                                  Si446x_GET_ADC_READING_TEMP_MASK};
  si446x_reply_t rxData[Si446x_GET_ADC_READING_REPLY_SIZE];
  if (!Si446x_read(radio, txData, sizeof(txData), rxData, sizeof(rxData))) {
    /* Get the temperature data. */
    uint16_t adc = rxData[Si446x_GET_ADC_READING_TEMP_OFFSET + 1]
                | ((rxData[Si446x_GET_ADC_READING_TEMP_OFFSET] & 0x7) << 8);
    int16_t temp = (89900 * adc) / 4096 - 29300;
    Si446x_getData(radio)->lastTemp = temp;
    return false;
  }
  return true;
}

/**
 * @brief  Clear interrupt status of chip.
 * Returns false on success, true on IO error.
 */
static bool Si446x_clearInterruptStatus(const radio_unit_t radio) {
  const si446x_arg_t txData[] = {Si446x_GET_INT_STATUS_CMD, 0x00, 0x00, 0x00};
  uint8_t rxData[Si446x_GET_INT_STATUS_REPLY_SIZE];
  return Si446x_read(radio, txData, sizeof(txData), rxData, sizeof(rxData));
}

/**
 * @brief  Put radio hardware in SDN.
 */
static void Si446x_powerDownRadio(const radio_unit_t radio) {
  palSetLine(Si446x_getConfig(radio)->sdn);
}

/**
 * @brief  Initializes Si446x transceiver chip.
 * Returns false on success, true on IO error.
 */
static bool Si446x_initDevice(const radio_unit_t radio) {

  TRACE_DEBUG("SI   > Initialize radio %d", radio);

  packet_svc_t *handler = pktGetServiceObject(radio);

  /*
   * Set MCU GPIO for radio GPIO1 (CTS).
   * Execute radio wake up sequence.
   */
  if (Si446x_radioWakeUp(radio)) {
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

  const uint8_t init_command[] = {Si446x_POWER_UP_CMD, 0x01,
                                  (Si446x_CLK_TCXO_EN & 0x1),
                                  x3, x2, x1, x0};
  /*
   * Use of writeBoot() enables SPI write without using OS delays.
   * The Si446x GPIO1 is set to CTS at start up.
   *
   * The Si446x SDO pin is configured to SDO data by the POWER_UP command.
   */
  if (Si446x_writeBoot(radio, init_command, sizeof(init_command)))
      return true;

  /*
   * Next get the PART_INFO.
   * Store details for reference.
   * If the part requires a patch then reset the 446x and delay.
   * Then output the patch and re-execute the POWER_UP command.
   */
  si446x_part_t part_info;
  const uint8_t get_part[] = {Si446x_GET_PART_INFO_CMD};
  if (Si446x_readBoot(radio, get_part, sizeof(get_part),
                        (uint8_t *)&part_info,
                        sizeof(part_info)))
    return true;

  /* Check if the radio responded with a valid ID. 0x0000 or 0xFFFF
     are treated as invalid and likely represent a failure of SPI. */
  uint16_t radio_part = (part_info.info[1] << 8) + part_info.info[2];
  if (radio_part == 0 || radio_part == 0xFFFF) {
    TRACE_ERROR("SI   > Part info of radio %d is invalid. Possible SPI fault",
                radio);
    return true;
  }

  /* Save the part information and ROM revision. */
  Si446x_getData(radio)->radio_part = radio_part;
  Si446x_getData(radio)->radio_rom_rev = part_info.info[7];
  Si446x_getData(radio)->radio_patch = 0;

  handler->radio_part = radio_part;
  handler->radio_rom_rev = part_info.info[7];

  /*
   * Check if this radio requires a patch installed.
   * TODO: Probably should be in a table where ROM rev is specified...
   */
  if (is_Si4463_patch_required(Si446x_getData(radio)->radio_part,
                               Si446x_getData(radio)->radio_rom_rev)) {
    /* Power cycle radio and apply patch. */
    TRACE_DEBUG("SI   > Restart radio %d to apply patch", radio);
    Si446x_powerDownRadio(radio);
    chThdSleep(TIME_MS2I(10));
    if (Si446x_radioWakeUp(radio)) {
      TRACE_ERROR("SI   > Wake up of radio %d to load patch failed", radio);
      return true;
    }
    /* Load the patch to the radio and then POWER_UP again. */
    uint16_t i = 0;
    while (Si4463_Patch_Data_Array[i] != 0) {
      if (!Si446x_writeBoot(radio, &Si4463_Patch_Data_Array[i + 1],
                       Si4463_Patch_Data_Array[i]))
      i += Si4463_Patch_Data_Array[i] + 1;
      else {
        TRACE_ERROR("SI   > Error during loading of patch to radio %d", radio);
        return true;
      }
    }
    const uint8_t init_command[] = {Si446x_POWER_UP_CMD, 0x81,
                                    (Si446x_CLK_TCXO_EN & 0x1),
                                    x3, x2, x1, x0};
    if (Si446x_writeBoot(radio, init_command, sizeof(init_command))) {
      TRACE_ERROR("SI   > Restart of radio %d after patch failed", radio);
      return true;
    }
  }

  /* Get and save the patch ID from FUNC_INFO for reference. */
  si446x_func_t func_info;
  const uint8_t get_func[] = {Si446x_GET_FUNC_INFO_CMD};
  if (Si446x_readBoot(radio, get_func, sizeof(get_func),
                        (uint8_t *)&func_info,
                        sizeof(func_info)))
    return true;

  /* Save the radio patch information. */
  Si446x_getData(radio)->radio_patch
                       = (func_info.info[3] << 8) + func_info.info[4];
  handler->radio_patch = (func_info.info[3] << 8) + func_info.info[4];

  if (Si446x_getData(radio)->radio_patch != 0) {
    TRACE_DEBUG("SI   > Patch ID 0x%x applied to radio %d",
                Si446x_getData(radio)->radio_patch, radio);
  }

  /* From here on any IO error is flagged and reported at end of init function. */
  bool error = false;

  /* Set the radio GPIOs to the basic configuration. */
  error |= Si446x_configureGPIO(radio, &(Si446x_getConfig(radio)->init).gpio);

  /* Clear interrupts any pending interrupts. */
  error |= Si446x_clearInterruptStatus(radio);

  /* If Si446x is using its own xtal set the trim capacitor value. */
#if !Si446x_CLK_TCXO_EN
  error |= Si446x_setProperty8(radio, Si446x_GLOBAL_XO_TUNE, Si446x_XO_TUNE);
#endif

  /* Fast response registers - not used at this time. */
  error |= Si446x_setProperty8(radio, Si446x_FRR_CTL_A_MODE, 0x00);
  error |= Si446x_setProperty8(radio, Si446x_FRR_CTL_B_MODE, 0x00);
  error |= Si446x_setProperty8(radio, Si446x_FRR_CTL_C_MODE, 0x00);
  error |= Si446x_setProperty8(radio, Si446x_FRR_CTL_D_MODE, 0x00);

  /* Disable interrupts globally initially. */
  error |= Si446x_setProperty8(radio, Si446x_INT_CTL_ENABLE, 0x00);

  /* Set high performance, fast TX start, half duplex FIFO & generic packet format. */
  error |= Si446x_setProperty8(radio, Si446x_GLOBAL_CONFIG, 0x30);

  /* Reset FIFO. */
  const uint8_t reset_fifo[] = {Si446x_FIFO_INFO_CMD, 0x03};
  error |= Si446x_write(radio, reset_fifo, sizeof(reset_fifo));

  /*
   * TODO: Move the TX and RX settings out into the respective functions.
   * This would split up into AFSK and FSK for RX & TX.
   * Leave only common setup and init here.
   */
  error |= Si446x_setProperty8(radio, Si446x_PREAMBLE_TX_LENGTH, 0x00);
  error |= Si446x_setProperty8(radio, Si446x_SYNC_CONFIG, 0x80);

  /* 32K clock disabled. Divided clock disabled. */
  error |= Si446x_setProperty8(radio, Si446x_GLOBAL_CLK_CFG, 0x00);

  /* TODO: This setting would move to 2FSK RX. */
  error |= Si446x_setProperty8(radio, Si446x_PREAMBLE_CONFIG_STD_1, 0x14);

  /* Bit polarity and mapping. */
  error |= Si446x_setProperty8(radio, Si446x_MODEM_MAP_CONTROL, 0x00);

  /* Delta Sigma modulation control for PLL synthesiser. */
  error |= Si446x_setProperty8(radio, Si446x_MODEM_DSM_CTRL, 0x07);

  /* Ramp down after TX final symbol. */
  error |= Si446x_setProperty8(radio, Si446x_MODEM_TX_RAMP_DELAY, 0x01);

  /* PA ramp timing and modulation delay. */
  error |= Si446x_setProperty8(radio, Si446x_PA_TC, 0x3D);

  /* Antenna settings. */
  error |= Si446x_setProperty8(radio, Si446x_MODEM_ANT_DIV_MODE, 0x01);
  error |= Si446x_setProperty8(radio, Si446x_MODEM_ANT_DIV_CONTROL, 0x80);

  /* RSSI value compensation. */
  error |= Si446x_setProperty8(radio, Si446x_MODEM_RSSI_COMP,
                        Si446x_MODEM_RSSI_COMP_VALUE);

  /*
   * TODO: Preamble configuration should be set in each mode.
   * Will be needed for RX FSK mode.
   * For now it is not relevant since:
   * - we don't have RX FSK implemented yet.
   * - RX AFSK preamble is decoded in the MCU DSP chain.
   * - TX AFSK encodes its own preamble and then up-samples the entire packet.
   * - TX 2FSK also encodes its own preamble which is sent as data from the PH.
   */
  error |= Si446x_setProperty8(radio, Si446x_PREAMBLE_CONFIG, 0x21);

  /* Measure the chip temperature and save initial measurement. */
  error |= Si446x_getTemperature(radio);
  /* Leave the radio in standby state. */
  error |= Si446x_radioStandby(radio);
  if (error)
    return true;
  handler->radio_init = true;
  return false;
}

#if Si446x_USE_INTERRUPTS
/**
 * @brief  Call back target of NIRQ GPIO event.
 */
static void Si446x_NIRQHandler(const radio_unit_t radio) {
  /*
   * @brief Resumes the thread waiting on the interrupt event.
   *
   * @notapi
   */
  chSysLockFromISR();
#if Si446x_USE_SEMAPHORE_INTERRUPT_SYNC == TRUE
  if (chBSemGetStateI(&Si446x_getData(radio)->wait_sem)) {
    /* Only signal if there is a semaphore owner. */
    chBSemSignalI(&Si446x_getData(radio)->wait_sem);
  }
#else
  chThdResumeI(&Si446x_getData(radio)->irq_dispatch, MSG_OK);
#endif
  chSysUnlockFromISR();
}

/**
 * @brief  Disable the GPIO assigned to the radio NIRQ.
 */
static void Si446x_disableNIRQEvent(const radio_unit_t radio) {
  /* Disable event for NIRQ.*/
  palDisableLineEvent(*Si446x_getConfig(radio)->xirq.nirq.pline);
  Si446x_getData(radio)->nirq_active = false;
}

/**
 * @brief  Enable the GPIO assigned to the radio NIRQ and set event CB.
 */
static void Si446x_enableNIRQEvent(const radio_unit_t radio) {
  /* If already enabled just exit. */
  if (Si446x_getData(radio)->nirq_active == true) {
    return;
  }
  /* Configure MCU GPIO. */
  pktSetGPIOlineMode(*Si446x_getConfig(radio)->xirq.nirq.pline,
                     Si446x_getConfig(radio)->xirq.nirq.mode);

  /* Set callback for NIRQ from radio. */
  palSetLineCallback(*Si446x_getConfig(radio)->xirq.nirq.pline,
                     (palcallback_t)Si446x_NIRQHandler, (void *)radio);

  /* Enabling events on falling edge of NIRQ.*/
  palEnableLineEvent(*Si446x_getConfig(radio)->xirq.nirq.pline,
                     PAL_EVENT_MODE_FALLING_EDGE);
  /* Flag enable is done. */
  Si446x_getData(radio)->nirq_active = true;
}

#if Si446x_USE_SEMAPHORE_INTERRUPT_SYNC == TRUE
/**
 * @brief   Set interrupt events to drive NIRQ in radio according to mask.
 * @pre     The MCU GPIO assigned to NIRQ has to be set for event and enabled.
 * @detail  For radio interrupts to drive NIRQ they need to be enabled with
 *          INT_CTL_ENABLE and INT_CTL_XX_ENABLE commands in the 446x.
 *          TODO: May need to use an array of interrupt bits and
 *          just enable all three groups if interrupts out different registers
 *          need to be drive NIRQ in the one call.
 *
 * @param[in] radio     radio unit ID
 * @param[in] reg       the interrupt register number
 *                      value is 0 (PH), 1 (MODEM) or 2 (CHIP)
 * @param[in] mask      mask of system radio events
 * @param[in] timeout   number of system ticks to wait
 *
 * @return  status message
 * @retval  MSG_OK      interrupt enabled
 * @retval  MSG_TIMEOUT timeout waiting to acquire interrupt system
 * @retval  MSG_RESET   interrupt system semaphore has been reset
 * @retval  MSG_ERROR   error in radio I/O
 *
 * @api
 */
msg_t Si446x_setRadioInterruptMask(const radio_unit_t radio,
                                   const si446x_arg_t reg,
                                   const si446x_arg_t mask,
                                   const sysinterval_t timeout) {
  chDbgCheck(reg <= Si446x_INT_CTL_CHIP_REG_INDEX);
  chDbgCheck(mask != 0);

  /* Take the interrupt semaphore. */
  msg_t msg = chBSemWaitTimeout(&Si446x_getData(radio)->wait_sem, timeout);
  if (msg != MSG_OK) {
    return msg;
  }

  /* Interrupt system now exclusive. Clear interrupt status. */
  if (Si446x_clearInterruptStatus(radio)) {
    chBSemSignal(&Si446x_getData(radio)->wait_sem);
    return MSG_ERROR;
  }

  /* Enable the interrupt(s). */
  if (Si446x_setProperty8(radio, Si446x_INT_CTL_PH_ENABLE + reg, mask)) {
    chBSemSignal(&Si446x_getData(radio)->wait_sem);
    return MSG_ERROR;
  }
  if (Si446x_setProperty8(radio, Si446x_INT_CTL_ENABLE, 1 << reg)) {
    chBSemSignal(&Si446x_getData(radio)->wait_sem);
    return MSG_ERROR;
  }
  return msg;
}

/**
 * @brief   Wait interrupt event in radio.
 * @pre     The interrupt(s) of interest have been set.
 *
 * @param[in] radio     radio unit ID
 * @param[in] timeout   number of system ticks to wait
 *
 * @return  status message
 * @retval  MSG_OK      interrupt processed
 * @retval  MSG_TIMEOUT timeout waiting for interrupt
 * @retval  MSG_RESET   interrupt semaphore has been reset
 * @retval  MSG_ERROR   radio IO error
 *
 * @api
 */
msg_t Si446x_waitRadioInterrupt(const radio_unit_t radio,
                                const sysinterval_t timeout) {

 /* Wait on the semaphore. NIRQ handler will signal. If not timeout. */
  msg_t msg = chBSemWaitTimeout(&Si446x_getData(radio)->wait_sem, timeout);
  if (msg == MSG_OK) {
    /* Get status. Will wait if SPI is locked but otherwise should be fast.
       The interrupt pending status is cleared. */
    if (Si446x_getInterruptStatus(radio,
                      (si446x_reply_t *)&Si446x_getData(radio)->int_status,
                      true)) {
      /* Radio SPI read error. */
      msg = MSG_ERROR;
    }
    /* Release the waiting thread. */
    chBSemSignal(&Si446x_getData(radio)->wait_sem);
  }
  return msg;
}
#else /* Si446x_USE_SEMAPHORE_INTERRUPT_SYNC != TRUE */
/**
 * @brief   Wait for interrupt events in radio according to mask.
 * @pre     The MCU GPIO assigned to NIRQ has to be set for event and enabled.
 * @detail  For radio interrupts to drive NIRQ they need to be enabled with
 *          INT_CTL_ENABLE and INT_CTL_XX_ENABLE commands in the 446x.
 *          TODO: Could simplify by sending an array of interrupt bits and
 *          just enable all three groups.
 *
 * @param[in] radio     radio unit ID
 * @param[in] reg       the interrupt register number
 *                      value is 0 (PH), 1 (MODEM) or 2 (CHIP)
 * @param[in] mask      mask of system radio events
 * @param[in] timeout   number of system ticks to wait
 *
 * @return  status message
 * @retval  MSG_OK      interrupt processed
 * @retval  MSG_TIMEOUT timeout waiting for interrupt
 * @retval  MSG_RESET   radio semaphore has been reset
 * @retval  MSG_ERROR   error in radio I/O
 *
 * @api
 */
msg_t Si446x_waitRadioInterrupt(const radio_unit_t radio,
                                const si446x_arg_t reg,
                                const si446x_arg_t mask,
                                const sysinterval_t timeout) {
  chDbgCheck(reg <= Si446x_INT_CTL_CHIP_REG_INDEX);
  chDbgCheck(mask != 0);
  (void) Si446x_setProperty8(radio, Si446x_INT_CTL_PH_ENABLE + reg, mask);
  (void) Si446x_setProperty8(radio, Si446x_INT_CTL_ENABLE, 1 << reg);

  /* Wait for the IRQ handler or OS timeout to wake us up. */
  chSysLock();
  msg_t imsg = chThdSuspendTimeoutS(&Si446x_getData(radio)->wait_thread, timeout);
  chSysUnlock();

  if (imsg == MSG_TIMEOUT) {
    /* If the interrupt wait period timed out disable interrupts. */
    (void) Si446x_setProperty8(radio, Si446x_INT_CTL_ENABLE, 0);
  }
  return imsg;
}

/*
 * @brief The radio interrupt dispatcher thread.
 * @param[in] arg  The radio unit id.
 *
 * @isr
 */
static THD_FUNCTION(Si446x_interrupt_dispatcher, arg) {

  radio_unit_t radio = (radio_unit_t)arg;

  /* Create thread name for this instance. */
  char isr_thd_name[PKT_THREAD_NAME_MAX];
  chsnprintf(isr_thd_name, sizeof(isr_thd_name),
             "isr_446x_%02i", radio);

  chRegSetThreadName(isr_thd_name);

  /* Synchronise then initialise thread references. */
  thread_t *thd = chMsgWait();
  Si446x_getData(radio)->cb = NULL;
  Si446x_getData(radio)->wait_thread = NULL;
  Si446x_getData(radio)->irq_dispatch = NULL;

  /* let the radio init proceed. */
  (void) chMsgGet(thd);

  /* Configure radio GPIOs for NIRQ dispatcher. */
  if (Si446x_configureGPIO(radio, &(Si446x_getConfig(radio)->xirq).gpio)) {
    chMsgRelease(thd, MSG_ERROR);
    chThdExit(MSG_OK);
  }

  /* Release the init process. */
  chMsgRelease(thd, MSG_OK);

  /* Ready to service requests now. */
  while (true) {
    /* Wait for the IRQ handler to wake us up. */
    chSysLock();
    msg_t wmsg = chThdSuspendS(&Si446x_getData(radio)->irq_dispatch);
    if (wmsg == MSG_RESET) {
      /* The radio is being shut down. */
      Si446x_disableNIRQEvent(radio);
      /* Resume any suspended thread if set. */
      chThdResumeS(&Si446x_getData(radio)->wait_thread, MSG_RESET);
      chThdExitS(MSG_OK);
    }
    msg_t imsg = MSG_OK;
    if (wmsg == MSG_OK) {
      chSysUnlock();
      /* Get status. Will wait if SPI is locked but otherwise should be fast.
         The interrupt pending status is cleared. */
      if (Si446x_getInterruptStatus(radio,
                        (si446x_reply_t *)&Si446x_getData(radio)->int_status,
                        true)) {
        /* Radio SPI read error. */
        imsg = MSG_ERROR;
      }
     chSysLock();
    }

    /* Execute call back if set. */
    if (Si446x_getData(radio)->cb != NULL) {
      /* Execute the callback then remove the callback reference. */
      Si446x_getData(radio)->cb(imsg);
      Si446x_getData(radio)->cb = NULL;
    }
    /* Resume suspended thread if set. */
    chThdResumeS(&Si446x_getData(radio)->wait_thread, imsg);
    chSysUnlock();
  }
}
#endif /* Si446x_USE_SEMAPHORE_INTERRUPT_SYNC != TRUE */
#endif /* Si446x_USE_INTERRUPTS */

/**
 * @brief Wait for radio to enter state
 *
 * @param[in] radio     radio unit ID
 * @param[in] state     radio state to be set
 * @param[in] timeout   the number of ticks before the operation times out
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *
 * @return status of operation
 * @retval MSG_OK       if state reached within timeout.
 * @retval MSG_TIMEOUT  if state not reached within timeout.
 * @retval MSG_ERROR    if IO error communicating with radio.
 *
 * @api
 *
 */
static msg_t Si446x_waitForState(const radio_unit_t radio,
                                 const si446x_state_t state,
                                 const sysinterval_t timeout) {
  si446x_state_t s;
  if (timeout == TIME_IMMEDIATE) {
    if (Si446x_requestDeviceState(radio, &s, NULL)) {
        return MSG_ERROR;
    }
    return s == state ? MSG_OK : MSG_TIMEOUT;
  }
  if (timeout == TIME_INFINITE) {
    do {
      chThdSleep(TIME_MS2I(10));
      if (Si446x_requestDeviceState(radio, &s, NULL)) {
        return MSG_ERROR;
      }
    } while (s != state);
    return MSG_OK;
  }

  time_msecs_t ms = chTimeI2MS(timeout);
  do {
    chThdSleep(TIME_MS2I(1));
    if (Si446x_requestDeviceState(radio, &s, NULL)) {
      return MSG_ERROR;
    }
  } while (--ms != 0 && s != state);
  return (ms == 0) ? MSG_TIMEOUT : MSG_OK;
}

/**
 * @brief Set a new radio state and wait for radio to enter the state
 *
 * @param[in] radio     radio unit ID
 * @param[in] state     radio state to be set
 * @param[in] timeout   the number of ticks before the operation times out
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *
 * @return status of operation
 * @retval MSG_OK       if state reached within timeout.
 * @retval MSG_TIMEOUT  if state not reached within timeout.
 * @retval MSG_ERROR    if IO error communicating with radio.
 *
 * @api
 *
 */
static msg_t Si446x_setStateWaitChange(const radio_unit_t radio,
                                       const si446x_state_t state,
                                       const sysinterval_t timeout) {
#if Si446x_USE_STATE_CHANGE_INTERRUPT == TRUE
  /* Clear any interrupts. */
  if (Si446x_clearInterruptStatus(radio)) {
      return MSG_ERROR;
  }
#endif
  const si446x_arg_t change_state_command[] = {Si446x_CHANGE_STATE_CMD,
                                          state};
#if Si446x_USE_STATE_CHANGE_INTERRUPT == TRUE
#if Si446x_USE_SEMAPHORE_INTERRUPT_SYNC == TRUE
  msg_t msg = Si446x_setRadioInterruptMask(radio,
                            Si446x_INT_CTL_CHIP_REG_INDEX,
                            Si446x_INT_CTL_CHIP_STATE_CHANGE_MASK,
                            TIME_MS2I(100));
  if (msg != MSG_OK) {
    return msg;
  }
  if (Si446x_write(radio, change_state_command, sizeof(change_state_command))) {
    return MSG_ERROR;
  }
  return Si446x_waitRadioInterrupt(radio, timeout);
#else
  if (Si446x_write(radio, change_state_command, sizeof(change_state_command))) {
    return MSG_ERROR;
  }
  return Si446x_waitRadioInterrupt(radio,
                                    Si446x_INT_CTL_CHIP_REG_INDEX,
                                    Si446x_INT_CTL_CHIP_STATE_CHANGE_MASK,
                                    timeout);
#endif /* Si446x_USE_SEMAPHORE_INTERRUPT_SYNC */
#else
  if (Si446x_write(radio, change_state_command, sizeof(change_state_command))) {
    return MSG_ERROR;
  }
  return Si446x_waitForState(radio, state, timeout);
#endif
}

/**
 * Initialise radio if it has been shutdown.
 *
 * NOTE: RADIO_CS and RADIO_SDN pins are configured in board.h
 * RADIO_SDN is configured to open drain pullup.
 * It is also pulled up on PCB by 100K.
 * The radio powers up in SDN mode.
 * CS is set as push-pull and initialized to HIGH.
 *
 * A interrupt handler is started to manage NIRQ interrupts.
 *
 * returns false on success, true on fail.
 *
 */
bool Si446x_driverInit(const radio_unit_t radio) {
  packet_svc_t *handler = pktGetServiceObject(radio);
  /* If the radio was powered off then full initialise required. */
  if (!handler->radio_init) {
    /* Initialise 446x registers. */
    if (Si446x_initDevice(radio)) {
      return true;
    }
#if Si446x_USE_INTERRUPTS
    Si446x_getData(radio)->nirq_active = false;
    if (Si446x_clearInterruptStatus(radio)) {
      return true;
    }
#if Si446x_USE_SEMAPHORE_INTERRUPT_SYNC == TRUE
    /* Initialise the interrupt semaphore. */
    chBSemObjectInit(&Si446x_getData(radio)->wait_sem, false);

    /* Configure radio GPIOs for NIRQ dispatcher. */
    if (Si446x_configureGPIO(radio, &(Si446x_getConfig(radio)->xirq).gpio)) {
      return true;
    }

    /* Configure PAL event handler and the interrupt semaphore.  */
    Si446x_enableNIRQEvent(radio);

#else
    /* The radio init is done. Now prepare the interrupt dispatcher. */
    thread_t *irq = chThdCreateFromHeap(NULL,
                             THD_WORKING_AREA_SIZE(SI_NIRQ_HANDLER_WA_SIZE),
                             NULL,
                             HIGHPRIO,
                             Si446x_interrupt_dispatcher,
                             (void *)radio);

    Si446x_getData(radio)->irq_dispatch = irq;
    if (irq == NULL) {
      return true;
    }

    /* Synchronize startup of interrupt dispatcher. */
    if (chMsgSend(irq, MSG_OK) == MSG_ERROR) {
      /* Initialisation failed. */
      chThdWait(irq);
      return true;
    }
#endif /* Si446x_USE_SEMAPHORE_INTERRUPT_SYNC */
#endif /* Si446x_USE_INTERRUPTS */
  }
#if Si446x_USE_INTERRUPTS
  /* Conditionally enable the NIRQ event system. */
  Si446x_enableNIRQEvent(radio);
#endif
  /* Force the radio into ready state. This will terminate any other
     state that was active on the radio. */
  bool status = Si446x_setReadyState(radio);
  status |= (Si446x_waitForState(radio, Si446x_READY, TIME_MS2I(100)) != MSG_OK);
  return status;
}

/*
 * Set radio frequency control parameters.
 * Values are calculated using the current XO frequency.
 * The base frequency and step size are set.
 * TX frequency and deviation are set.
 * RX IF frequency is set.
 * The current XO is supplied by the TCXO service via Si446x_updateClock().
 *
 * This function also collects the chip temperature data at the moment.
 * TODO: Move temperature reading to LLD (HAL) function.
 *       Collector will send UpdateOperatingConditions() RM and get results.
 */
static bool Si446x_setSynthParameters(const radio_unit_t radio,
                              radio_freq_hz_t freq,
                              radio_chan_hz_t step,
                              radio_dev_hz_t dev) {

  /* Check frequency is in range of chip. */
  if (freq < 144000000UL || freq > 900000000UL) {
    return false;
  }

  /* Set the output divider as recommended in Si446x data sheet. */
  uint8_t outdiv = 0;
  uint8_t band = 0;
  if (freq < 705000000UL) {outdiv = 6;  band = 1;}
  if (freq < 525000000UL) {outdiv = 8;  band = 2;}
  if (freq < 353000000UL) {outdiv = 12; band = 3;}
  if (freq < 239000000UL) {outdiv = 16; band = 4;}
  if (freq < 177000000UL) {outdiv = 24; band = 5;}

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

  /* Set MODEM_CLKGEN_BAND */
  if (Si446x_setProperty8(radio, Si446x_MODEM_CLKGEN_BAND,
                            (Si446x_PLL_MODE | band))) {
    return true;
  }

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
  uint8_t set_frequency_property_command[] = {Si446x_SET_PROPERTY_CMD,
                                              0x40, 0x06, 0x00, n,
                                              m2, m1, m0, c1, c0};
  if (Si446x_write(radio, set_frequency_property_command,
               sizeof(set_frequency_property_command))) {
    return false;
  }

  /* Calculate and set RX for fixed IF mode (N = 1). */
  if (Si446x_setProperty8(radio, Si446x_MODEM_IF_CONTROL, 0x08)) {
    return false;
  }
  uint32_t rif = si_clock / (Si446x_IF_OFFSET * Si446x_IF_SCALE);
  int32_t i = -((scaled_outdiv * rif) / pll_freq);
  uint8_t i2 = (i >> 16) & 0x03;
  uint8_t i1 = (i >> 8) & 0xFF;
  uint8_t i0 = i & 0xFF;

  uint8_t set_modem_if[] = {Si446x_SET_PROPERTY_CMD, 0x20, 0x03, 0x01b, i2, i1, i0};
  if (Si446x_write(radio, set_modem_if, sizeof(set_modem_if))) {
    return false;
  }
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
  if (Si446x_setProperty8(radio, Si446x_FREQ_CONTROL_W_SIZE, Si446x_WSIZE)) {
    return false;
  }
  /* Calculate and set the RX VCO adjustment factor. */
  int8_t v = ((rif * outdiv) / (Si446x_PRESCALER * Si446x_WSIZE)) / si_clock;
  if (Si446x_setProperty8(radio, Si446x_FREQ_CONTROL_VCOCNT_RX_ADJ, -v)) {
    return false;
  }
  /* Calculate and set TX deviation. */
  if (dev != 0) {
    /* Set TX deviation. */
    uint32_t x = (scaled_outdiv * dev) / pll_freq;
    uint8_t x2 = (x >> 16) & 0xFF;
    uint8_t x1 = (x >>  8) & 0xFF;
    uint8_t x0 = x & 0xFF;

    uint8_t set_deviation[] = {Si446x_SET_PROPERTY_CMD,
                               0x20, 0x03, 0x0a, x2, x1, x0};
    (void) Si446x_write(radio, set_deviation, sizeof(set_deviation));
  }

  /* TODO: Move NCO modulo and data rate setting into here? */

  /* Measure the chip temperature and update saved value.
   * TODO: Make this accessible via a RTO LLD and have collector read it.
   */
  (void) Si446x_getTemperature(radio);
  return true;
}

/**
 * Set TX power.
 * Return false on success, true on IO fail.
 */
static bool Si446x_setPowerLevel(const radio_unit_t radio,
								 const radio_pwr_t level) {
    // Set the Power
    uint8_t set_pa_pwr_lvl_property_command[] = {Si446x_SET_PROPERTY_CMD,
                                                 0x22, 0x01, 0x01, level};
    return Si446x_write(radio, set_pa_pwr_lvl_property_command,
                 sizeof(set_pa_pwr_lvl_property_command));
}

/**
 * @brief  Simple receive mode for CCA carrier sense only.
 * @return false if setup OK, true if CCA not supported or setup failed
 *
 */
static bool Si446x_setModemCCA_Detection(const radio_unit_t radio) {

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
  (void) Si446x_configureGPIO(radio, &(Si446x_getConfig(radio)->rcca).gpio);

  /*
   * Set up GPIO port where the NIRQ from the radio is connected.
   * The NIRQ line is configured in the radio to output the CCA condition.
   * TODO: Cater for situation where CCA is not defined in the radio config.
   */
  if (*Si446x_getConfig(radio)->rcca.cca.pline == PAL_NOLINE) {
    return true;
  }

  pktSetGPIOlineMode(*Si446x_getConfig(radio)->rcca.cca.pline,
                     Si446x_getConfig(radio)->rcca.cca.mode);

  /* Set DIRECT_MODE (asynchronous mode as 2FSK). */
  (void) Si446x_setProperty8(radio, Si446x_MODEM_MOD_TYPE, 0x0A);

  /* Packet handler disabled in RX. */
  (void) Si446x_setProperty8(radio, Si446x_PKT_CONFIG1, 0x41);

  if (is_part_Si4463(handler->radio_part)) {
    /* Run 4463 in 4464 compatibility mode (set SEARCH2 to zero). */
    (void) Si446x_setProperty8(radio, Si446x_MODEM_RAW_SEARCH2, 0x00);
  }
  (void) Si446x_setProperty8(radio, Si446x_MODEM_RAW_CONTROL, 0x8F);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_RAW_SEARCH, 0xD6);
  if (is_part_Si4463(handler->radio_part))
#if Si446x_4463_USE_446X_COMPATABILITY == TRUE
    (void) Si446x_setProperty16(radio, Si446x_MODEM_RAW_EYE, 0x00, 0x3B);
#else
    (void) Si446x_setProperty16(radio, Si446x_MODEM_RAW_EYE, 0x00, 0x76);
#endif
  else
    (void) Si446x_setProperty16(radio, Si446x_MODEM_RAW_EYE, 0x00, 0x3B);

  /*
   * OOK_MISC settings include parameters related to asynchronous mode.
   * Asynchronous mode is used for AFSK reception passed to DSP decode.
   *
   * SQUELCH[1:0] = 1 (don't toggle RX data if no signal received).
   * OOK_LIMIT_DISCG[5] Configures if the peak detector discharge is limited.
   *  Set 1 the peak detector discharge is disabled when the detected peak is lower than the input signal for low input levels.
   *  Versus 0 which sets peak detector discharges always.
   */
  (void) Si446x_setProperty8(radio, Si446x_MODEM_OOK_CNT1, 0x85);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_OOK_PDTC, 0x2A);
  if (is_part_Si4463(handler->radio_part)) {
#if Si446x_4463_USE_446X_COMPATABILITY == TRUE
    (void) Si446x_setProperty8(radio, Si446x_MODEM_OOK_MISC, 0x03);
#else
    (void) Si446x_setProperty8(radio, Si446x_MODEM_OOK_MISC, 0x23);
#endif
  }
  else {
    (void) Si446x_setProperty8(radio, Si446x_MODEM_OOK_MISC, 0x03);
  }

  /* RX AFC control. */
  (void) Si446x_setProperty8(radio, Si446x_MODEM_AFC_GEAR, 0x54);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_AFC_WAIT, 0x36);
  (void) Si446x_setProperty16(radio, Si446x_MODEM_AFC_GAIN, 0x80, 0xAB);
  (void) Si446x_setProperty16(radio, Si446x_MODEM_AFC_LIMITER, 0x02, 0x50);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_AFC_MISC, 0xC0); // 0x80

  /* RX AGC control. */
  (void) Si446x_setProperty8(radio, Si446x_MODEM_AGC_CONTROL, 0xE0); // 0xE2 (bit 1 not used in 4464. It is used in 4463.)
  (void) Si446x_setProperty8(radio, Si446x_MODEM_AGC_WINDOW_SIZE, 0x11);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_AGC_RFPD_DECAY, 0x63);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_AGC_IFPD_DECAY, 0x63);

  /* RX Bit clock recovery control. */
  (void) Si446x_setProperty8(radio, Si446x_MODEM_MDM_CTRL, 0x80);
  (void) Si446x_setProperty16(radio, Si446x_MODEM_BCR_OSR, 0x01, 0xC3);
  (void) Si446x_setProperty24(radio, Si446x_MODEM_BCR_NCO_OFFSET, 0x01, 0x22, 0x60);
  (void) Si446x_setProperty16(radio, Si446x_MODEM_BCR_GAIN, 0x00, 0x91);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_BCR_GEAR, 0x00);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_BCR_MISC1, 0xC2);

  /* RX IF controls. */
  (void) Si446x_setProperty8(radio, Si446x_MODEM_IF_CONTROL, 0x08);
  (void) Si446x_setProperty24(radio, Si446x_MODEM_IF_FREQ, 0x02, 0x80, 0x00);

  /* RX IF filter decimation controls. */
  (void) Si446x_setProperty8(radio, Si446x_MODEM_DECIMATION_CFG1, 0x70);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_DECIMATION_CFG0, 0x10);
  if (is_part_Si4463(handler->radio_part)) {
#if Si446x_4463_USE_446X_COMPATABILITY == TRUE
    (void) Si446x_setProperty8(radio, Si446x_MODEM_DECIMATION_CFG2, 0x00);
#else
    (void) Si446x_setProperty8(radio, Si446x_MODEM_DECIMATION_CFG2, 0x0C);
#endif
  }

  /* RSSI latching disabled. */
  (void) Si446x_setProperty8(radio, Si446x_MODEM_RSSI_CONTROL, 0x00);

  /*
   *  RX IF filter coefficients.
   *  TODO: Add an RX filter set function.
   */
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE13_7_0, 0xFF);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE12_7_0, 0xC4);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE11_7_0, 0x30);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE10_7_0, 0x7F);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE9_7_0, 0x5F);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE8_7_0, 0xB5);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE7_7_0, 0xB8);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE6_7_0, 0xDE);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE5_7_0, 0x05);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE4_7_0, 0x17);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE3_7_0, 0x16);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE2_7_0, 0x0C);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE1_7_0, 0x03);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE0_7_0, 0x00);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COEM0, 0x15);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COEM1, 0xFF);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COEM2, 0x00);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COEM3, 0x00);

  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE13_7_0, 0xFF);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE12_7_0, 0xC4);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE11_7_0, 0x30);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE10_7_0, 0x7F);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE9_7_0, 0x5F);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE8_7_0, 0xB5);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE7_7_0, 0xB8);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE6_7_0, 0xDE);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE5_7_0, 0x05);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE4_7_0, 0x17);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE3_7_0, 0x16);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE2_7_0, 0x0C);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE1_7_0, 0x03);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE0_7_0, 0x00);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COEM0, 0x15);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COEM1, 0xFF);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COEM2, 0x00);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COEM3, 0x00);

  (void) Si446x_setProperty8(radio, Si446x_PREAMBLE_CONFIG, 0x21);

  /* Unused Si4463 features for AFSK RX. */
  if (is_part_Si4463(handler->radio_part)) {
   /* DSA is not enabled. */
   (void) Si446x_setProperty8(radio, Si446x_MODEM_DSA_CTRL1, 0x00); // 0xA0
   (void) Si446x_setProperty8(radio, Si446x_MODEM_DSA_CTRL2, 0x00); // 0x04
   (void) Si446x_setProperty8(radio, Si446x_MODEM_SPIKE_DET, 0x00); // 0x03
   (void) Si446x_setProperty8(radio, Si446x_MODEM_ONE_SHOT_AFC, 0x00); // 0x07
   (void) Si446x_setProperty8(radio, Si446x_MODEM_DSA_QUAL, 0x00); // 0x06
   (void) Si446x_setProperty8(radio, Si446x_MODEM_DSA_RSSI, 0x00); // 0x78
   (void) Si446x_setProperty8(radio, Si446x_MODEM_RSSI_MUTE, 0x00);
   (void) Si446x_setProperty8(radio, Si446x_MODEM_DSA_MISC, 0x00); // 0x20
  }
  return false;
}

/*
 *  Radio modulation settings
 */

static void Si446x_setModemAFSK_TX(const radio_unit_t radio) {

  /* Configure radio GPIOs. */
  (void) Si446x_configureGPIO(radio, &(Si446x_getConfig(radio)->tafsk).gpio);

  /* Set up GPIO port where the CCA from the radio is connected. */
  pktSetGPIOlineMode(*Si446x_getConfig(radio)->tafsk.cca.pline,
                     Si446x_getConfig(radio)->tafsk.cca.mode);

  /* Setup the NCO modulo and over sampling mode. */
  radio_clock_t si_clock = Si446x_getData(radio)->radio_clock;
  uint32_t s = si_clock / 10;
  uint8_t f3 = (s >> 24) & 0xFF;
  uint8_t f2 = (s >> 16) & 0xFF;
  uint8_t f1 = (s >>  8) & 0xFF;
  uint8_t f0 = (s >>  0) & 0xFF;
  (void) Si446x_setProperty32(radio, Si446x_MODEM_TX_NCO_MODE, f3, f2, f1, f0);

  // Setup the NCO data rate for APRS
  (void) Si446x_setProperty24(radio, Si446x_MODEM_DATA_RATE, 0x00, 0x33, 0x90);

  /* Use 2FSK mode in conjunction with up-sampled AFSK from FIFO (PH). */
  (void) Si446x_setProperty8(radio, Si446x_MODEM_MOD_TYPE, 0x02);

  /* Set PH bit order for AFSK. */
  (void) Si446x_setProperty8(radio, Si446x_PKT_CONFIG1, 0x01);

  /* Set AFSK shaping filter. */
  const uint8_t coeff[] = {0x81, 0x9f, 0xc4, 0xee, 0x18, 0x3e, 0x5c, 0x70, 0x76};
  uint8_t i;
  for (i = 0; i < sizeof(coeff); i++) {
    si446x_arg_t data[] = {0x11, 0x20, 0x01, 0x17-i, coeff[i]};
    (void) Si446x_write(radio, data, 5);
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
  (void) Si446x_configureGPIO(radio, &(Si446x_getConfig(radio)->rafsk).gpio);

  /* Set DIRECT_MODE (asynchronous mode as 2FSK). */
  (void) Si446x_setProperty8(radio, Si446x_MODEM_MOD_TYPE, 0x0A);

  /* Packet handler disabled in RX. */
  (void) Si446x_setProperty8(radio, Si446x_PKT_CONFIG1, 0x40);

  if (is_part_Si4463(handler->radio_part)) {
    /* To run 4463 in 4464 compatibility mode (set SEARCH2 to zero). */
    /* 0xBC (SCH_FROZEN = 1 {Freeze min-max on gear switch, SCHPRD_HI = 6 SCHPRD_LO = 4 {SEARCH_4TB, SEARCH_8TB}) */
#if Si446x_4463_USE_446X_COMPATABILITY == TRUE
    (void) Si446x_setProperty8(radio, Si446x_MODEM_RAW_SEARCH2, 0x00);
#else
    (void) Si446x_setProperty8(radio, Si446x_MODEM_RAW_SEARCH2, 0xBC);
#endif
  }

  /*
   * MODEM_RAW_CONTROL
   *  UNSTDPK[7] = 1 (raw mode for non-standard packet reception)
   *  CONSCHK_BYP[6] = 1 (don't stop mean value being updated by consecutive 1s or 0s)
   *   This is a change from the WDS output.
   * PM_PATTERN[3:2] = 3 preamble pattern is random
   * RAWGAIN[0] = 3  gain is 1
   */
  (void) Si446x_setProperty8(radio, Si446x_MODEM_RAW_CONTROL, 0xCF);

  /*
   * Note: When MODEM_RAW_SEARCH2 is non zero MODEM_RAW_SEARCH settings are ignored.
   * SCH_FRZEN [7]  = 1 Freeze MA or min-max (mean) slicing on gear switch
   * SCH_FRZTH [6:5] = 5 consecutive search periods within frequency error threshold enabling gear switch
   * SCHPRD_HI[3:2] = 1 MA/min-max search period window length in bits for slicing threshold prior to gear switch
   * SCHPRD_LO[1:0] = 2 MA/min-max search period window length in bits for slicing threshold after gear switch
  */
  (void) Si446x_setProperty8(radio, Si446x_MODEM_RAW_SEARCH, 0xD6);
  /* MODEM_RAW_EYE[0,10:8][1,7:0] */
#if Si446x_USE_AFSK_LCM_DATA_RATE == TRUE
  (void) Si446x_setProperty16(radio, Si446x_MODEM_RAW_EYE, 0x00, 0x3B);
#else
#if Si446x_USE_NB_RECEIVE_FILTER == TRUE
  (void) Si446x_setProperty16(radio, Si446x_MODEM_RAW_EYE, 0x01, 0x8F);
#else
  (void) Si446x_setProperty16(radio, Si446x_MODEM_RAW_EYE, 0x00, 0xC8);
#endif
#endif
  /*
   * OOK_MISC settings include parameters related to asynchronous mode.
   * Asynchronous mode is used for AFSK reception passed to DSP decode.
   * ATTACK[6:4]
   * DECAY[3:0]
   */
#if Si446x_USE_AFSK_LCM_DATA_RATE == TRUE
  (void) Si446x_setProperty8(radio, Si446x_MODEM_OOK_PDTC, 0x28);
#else
#if Si446x_USE_NB_RECEIVE_FILTER == TRUE
  (void) Si446x_setProperty8(radio, Si446x_MODEM_OOK_PDTC, 0x29);
#else
  (void) Si446x_setProperty8(radio, Si446x_MODEM_OOK_PDTC, 0x2A);
#endif
#endif
  /*
   * SQUELCH[1:0] = 1 When no signal is received, there is no toggling of RX data output.
   * This is a change from the WDS output.
   */
  (void) Si446x_setProperty8(radio, Si446x_MODEM_OOK_CNT1, 0x85);
  /*
   * OOK_MISC settings include parameters related to asynchronous mode.
   * Asynchronous mode is used for AFSK reception passed to DSP decode.
   *
   * SQUELCH[1:0] = 1 (don't toggle RX data if no signal received).
   * OOK_LIMIT_DISCG[5] Configures if the peak detector discharge is limited.
   *  Set 1 the peak detector discharge is disabled when the detected peak is lower than the input signal for low input levels.
   *  Versus 0 which sets peak detector discharges always.
   */
  if (is_part_Si4463(handler->radio_part)) {
  #if Si446x_4463_USE_446X_COMPATABILITY == TRUE
    (void) Si446x_setProperty8(radio, Si446x_MODEM_OOK_MISC, 0x03);
  #else
    (void) Si446x_setProperty8(radio, Si446x_MODEM_OOK_MISC, 0x23);
  #endif
  }
  else {
    (void) Si446x_setProperty8(radio, Si446x_MODEM_OOK_MISC, 0x03);
  }
  /* RX AFC controls. */

  /*
   * AFC_GEAR
   *  GEAR_SW[7:6]  = ENUM_1 (Sync word detection - switch gears after detection of Sync Word - not active.)
   *  AFC_FAST[5:3] = 2 (higher gain results in slower AFC tracking)
   *  AFC_SLOW[2:0] = 4 (higher gain results in slower AFC tracking)
   */
  (void) Si446x_setProperty8(radio, Si446x_MODEM_AFC_GEAR, 0x54);

  /*
   * AFC_WAIT
   *  SHWAIT[7:4] This specifies the wait period per PLL AFC correction cycle before gear switching has occurred.
   *  LGWAIT [3:0] This specifies the wait period per PLL AFC correction cycle after gear switching has occurred.
   */
#if Si446x_USE_AFSK_LCM_DATA_RATE == TRUE
  (void) Si446x_setProperty8(radio, Si446x_MODEM_AFC_WAIT, 0x36);
#else
#if Si446x_USE_NB_RECEIVE_FILTER == TRUE
  (void) Si446x_setProperty8(radio, Si446x_MODEM_AFC_WAIT, 0x36);
#else
  (void) Si446x_setProperty8(radio, Si446x_MODEM_AFC_WAIT, 0x36);
#endif
#endif
  /*
   * AFC_GAIN
   *  ENAFC[0,7] Set to enable frequency error estimation and correction.
   *  AFCBD[0,6] Set to enable adaptive RX bandwidth (RX1_COEFF & RX2_COEFF)
   *  MODEM_AFC_GAIN[0,12:8 1,7:0] base gain scaled by AFC_GEAR (AFC_FAST & AFC_SLOW)
   */
#if Si446x_USE_AFSK_LCM_DATA_RATE == TRUE
  (void) Si446x_setProperty16(radio, Si446x_MODEM_AFC_GAIN, 0x82, 0xAA);
#else
#if Si446x_USE_NB_RECEIVE_FILTER == TRUE
  (void) Si446x_setProperty16(radio, Si446x_MODEM_AFC_GAIN, 0x80, 0x55);
#else
  (void) Si446x_setProperty16(radio, Si446x_MODEM_AFC_GAIN, 0x80, 0xAB);
#endif
#endif
  /*
   * MODEM_AFC_LIMITER
   *  TODO: Ask Si about the calculation of AFC limiter values.
   */
#if Si446x_USE_AFSK_LCM_DATA_RATE == TRUE
  (void) Si446x_setProperty16(radio, Si446x_MODEM_AFC_LIMITER, 0x00, 0x95);
#else
#if Si446x_USE_NB_RECEIVE_FILTER == TRUE
  (void) Si446x_setProperty16(radio, Si446x_MODEM_AFC_LIMITER, 0x01, 0xF2);
#else
  (void) Si446x_setProperty16(radio, Si446x_MODEM_AFC_LIMITER, 0x01, 0xE6);
#endif
#endif

  /*
   * ENAFCFRZ[7] = 1 gear switched but we don't switch,
   * NON_FRZEN[1] = 0 -> 1 (always enabled regardless of 1,0 string)
   */
  (void) Si446x_setProperty8(radio, Si446x_MODEM_AFC_MISC, 0x82);

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
  if (is_part_Si4463(handler->radio_part)) {
#if Si446x_4463_USE_446X_COMPATABILITY == TRUE
    (void) Si446x_setProperty8(radio, Si446x_MODEM_AGC_CONTROL, 0xE0);
#else
    (void) Si446x_setProperty8(radio, Si446x_MODEM_AGC_CONTROL, 0xE2);
#endif
  }
  else {
    (void) Si446x_setProperty8(radio, Si446x_MODEM_AGC_CONTROL, 0xE0);
  }
  (void) Si446x_setProperty8(radio, Si446x_MODEM_AGC_WINDOW_SIZE, 0x11);
#if Si446x_USE_AFSK_LCM_DATA_RATE == TRUE
  (void) Si446x_setProperty8(radio, Si446x_MODEM_AGC_RFPD_DECAY, 0x12);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_AGC_IFPD_DECAY, 0x12);
#else
#if Si446x_USE_NB_RECEIVE_FILTER == TRUE
  (void) Si446x_setProperty8(radio, Si446x_MODEM_AGC_RFPD_DECAY, 0x31);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_AGC_IFPD_DECAY, 0x31);
#else
  (void) Si446x_setProperty8(radio, Si446x_MODEM_AGC_RFPD_DECAY, 0x63);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_AGC_IFPD_DECAY, 0x63);
#endif
#endif

  /* RX Bit clock recovery control. */
  (void) Si446x_setProperty8(radio, Si446x_MODEM_MDM_CTRL, 0x80);
#if Si446x_USE_AFSK_LCM_DATA_RATE == TRUE
  (void) Si446x_setProperty16(radio, Si446x_MODEM_BCR_OSR, 0x00, 0x52);
  (void) Si446x_setProperty24(radio, Si446x_MODEM_BCR_NCO_OFFSET, 0x06, 0x3D, 0x10);
  (void) Si446x_setProperty16(radio, Si446x_MODEM_BCR_GAIN, 0x03, 0x1F);
#else
#if Si446x_USE_NB_RECEIVE_FILTER == TRUE
  (void) Si446x_setProperty16(radio, Si446x_MODEM_BCR_OSR, 0x00, 0xE2);
  (void) Si446x_setProperty24(radio, Si446x_MODEM_BCR_NCO_OFFSET, 0x02, 0x44, 0xC0);
  (void) Si446x_setProperty16(radio, Si446x_MODEM_BCR_GAIN, 0x01, 0x22);
#else
  (void) Si446x_setProperty16(radio, Si446x_MODEM_BCR_OSR, 0x01, 0xC3);
  (void) Si446x_setProperty24(radio, Si446x_MODEM_BCR_NCO_OFFSET, 0x01, 0x22, 0x60);
  (void) Si446x_setProperty16(radio, Si446x_MODEM_BCR_GAIN, 0x00, 0x91);
#endif
#endif
  (void) Si446x_setProperty8(radio, Si446x_MODEM_BCR_GEAR, 0x00);

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
  (void) Si446x_setProperty8(radio, Si446x_MODEM_BCR_MISC0, 0x00);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_BCR_MISC1, 0xC2);

  /* RX IF frequency controls are set in setSynthParameters() */

  /* RX IF CIC filter decimation controls. */
#if Si446x_USE_AFSK_LCM_DATA_RATE == TRUE
  (void) Si446x_setProperty8(radio, Si446x_MODEM_DECIMATION_CFG1, 0x20);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_DECIMATION_CFG0, 0x10);
#else
#if Si446x_USE_NB_RECEIVE_FILTER == TRUE
  (void) Si446x_setProperty8(radio, Si446x_MODEM_DECIMATION_CFG1, 0xB0);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_DECIMATION_CFG0, 0x10);
#else
  (void) Si446x_setProperty8(radio, Si446x_MODEM_DECIMATION_CFG1, 0x70);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_DECIMATION_CFG0, 0x10);
#endif
#endif
  if (is_part_Si4463(handler->radio_part)) {
#if Si446x_4463_USE_446X_COMPATABILITY == TRUE
    /*
     * Si4463 revC2A has an extra decimator stage to be used for very
     * narrowband (< 1 kHz) Rx applications. For AFSK case this should
     * be bypassed to emulate the Si4464 configuration.
     * NDEC2AGC[2] = 1 enable AGC control of 2nd stage CIC
     * NDEC2GAIN[4:3] = 1 2nd stage CIC gain is 12dB
     */
    (void) Si446x_setProperty8(radio, Si446x_MODEM_DECIMATION_CFG2, 0x00);
#else
    /*
     * NDEC2AGC[2] = 1 enable AGC control of 2nd stage CIC
     * NDEC2GAIN[4:3] = 1 2nd stage CIC gain is 12dB
     */
    (void) Si446x_setProperty8(radio, Si446x_MODEM_DECIMATION_CFG2, 0x0C);
#endif
  }

  /* RSSI controls. */
  /*
   * LATCH[2:0] = 0 disabled.
   * AVERAGE[4:3] = 0 average over 4*Tb
   */
  (void) Si446x_setProperty8(radio, Si446x_MODEM_RSSI_CONTROL, 0x00);
  if (is_part_Si4463(handler->radio_part)) {
    /* RSSI jump control (ENRSSIJMP[3] 1 -> 0 to disable. */
#if Si446x_4463_USE_446X_COMPATABILITY == TRUE
    (void) Si446x_setProperty8(radio, Si446x_MODEM_RSSI_CONTROL2, 0x00);
#else
    (void) Si446x_setProperty8(radio, Si446x_MODEM_RSSI_CONTROL2, 0x18);
#endif
    (void) Si446x_setProperty8(radio, Si446x_MODEM_RSSI_JUMP_THRESH, 0x06);
  }

  /* RX IF filter coefficients. */
#if Si446x_USE_AFSK_LCM_DATA_RATE == TRUE
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE13_7_0, 0xA2);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE12_7_0, 0xA0);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE11_7_0, 0x97);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE10_7_0, 0x8A);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE9_7_0, 0x79);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE8_7_0, 0x66);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE7_7_0, 0x52);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE6_7_0, 0x3F);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE5_7_0, 0x2E);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE4_7_0, 0x1F);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE3_7_0, 0x14);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE2_7_0, 0x0B);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE1_7_0, 0x06);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE0_7_0, 0x02);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COEM0, 0x00);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COEM1, 0x00);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COEM2, 0x00);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COEM3, 0x00);

  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE13_7_0, 0xA2);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE12_7_0, 0xA0);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE11_7_0, 0x97);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE10_7_0, 0x8A);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE9_7_0, 0x79);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE8_7_0, 0x66);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE7_7_0, 0x52);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE6_7_0, 0x3F);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE5_7_0, 0x2E);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE4_7_0, 0x1F);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE3_7_0, 0x14);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE2_7_0, 0x0B);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE1_7_0, 0x06);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE0_7_0, 0x02);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COEM0, 0x00);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COEM1, 0x00);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COEM2, 0x00);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COEM3, 0x00);
#else
#if Si446x_USE_NB_RECEIVE_FILTER == TRUE
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE13_7_0, 0xFF);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE12_7_0, 0xBA);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE11_7_0, 0x0F);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE10_7_0, 0x51);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE9_7_0, 0xCF);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE8_7_0, 0xA9);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE7_7_0, 0xC9);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE6_7_0, 0xFC);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE5_7_0, 0x1B);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE4_7_0, 0x1E);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE3_7_0, 0x0F);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE2_7_0, 0x01);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE1_7_0, 0xFC);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE0_7_0, 0xFD);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COEM0, 0x15);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COEM1, 0xFF);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COEM2, 0x00);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COEM3, 0x0F);

  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE13_7_0, 0xFF);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE12_7_0, 0xBA);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE11_7_0, 0x0F);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE10_7_0, 0x51);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE9_7_0, 0xCF);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE8_7_0, 0xA9);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE7_7_0, 0xC9);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE6_7_0, 0xFC);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE5_7_0, 0x1B);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE4_7_0, 0x1E);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE3_7_0, 0x0F);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE2_7_0, 0x01);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE1_7_0, 0xFC);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE0_7_0, 0xFD);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COEM0, 0x15);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COEM1, 0xFF);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COEM2, 0x00);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COEM3, 0x0F);
#else
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE13_7_0, 0xFF);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE12_7_0, 0xBA);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE11_7_0, 0x0F);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE10_7_0, 0x51);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE9_7_0, 0xCF);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE8_7_0, 0xA9);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE7_7_0, 0xC9);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE6_7_0, 0xFC);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE5_7_0, 0x1B);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE4_7_0, 0x1E);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE3_7_0, 0x0F);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE2_7_0, 0x01);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE1_7_0, 0xFC);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COE0_7_0, 0xFD);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COEM0, 0x15);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COEM1, 0xFF);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COEM2, 0x00);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX1_CHFLT_COEM3, 0x0F);

  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE13_7_0, 0xFF);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE12_7_0, 0xBA);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE11_7_0, 0x0F);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE10_7_0, 0x51);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE9_7_0, 0xCF);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE8_7_0, 0xA9);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE7_7_0, 0xC9);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE6_7_0, 0xFC);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE5_7_0, 0x1B);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE4_7_0, 0x1E);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE3_7_0, 0x0F);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE2_7_0, 0x01);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE1_7_0, 0xFC);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COE0_7_0, 0xFD);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COEM0, 0x15);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COEM1, 0xFF);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COEM2, 0x00);
  (void) Si446x_setProperty8(radio, Si446x_MODEM_CHFLT_RX2_CHFLT_COEM3, 0x0F);
#endif
#endif

  /* TODO: Not used in UNSTD mode. */
  (void) Si446x_setProperty8(radio, Si446x_PREAMBLE_CONFIG, 0x21);

  /* Unused Si4463 features for AFSK RX. */
  if (is_part_Si4463(handler->radio_part)) {
   /* DSA[5] is not enabled. */
   (void) Si446x_setProperty8(radio, Si446x_MODEM_DSA_CTRL1, 0x40);
   (void) Si446x_setProperty8(radio, Si446x_MODEM_DSA_CTRL2, 0x04);
   (void) Si446x_setProperty8(radio, Si446x_MODEM_SPIKE_DET, 0x03);
   (void) Si446x_setProperty8(radio, Si446x_MODEM_ONE_SHOT_AFC, 0x07);
   (void) Si446x_setProperty8(radio, Si446x_MODEM_DSA_QUAL, 0x06);
   (void) Si446x_setProperty8(radio, Si446x_MODEM_DSA_RSSI, 0x78);
   (void) Si446x_setProperty8(radio, Si446x_MODEM_RSSI_MUTE, 0x00);
   (void) Si446x_setProperty8(radio, Si446x_MODEM_DSA_MISC, 0x20);
  }
}


/**
 *
 */
static void Si446x_setModem2FSK_TX(const radio_unit_t radio,
                                   const uint32_t speed) {

  /* Configure radio GPIOs. */
  (void) Si446x_configureGPIO(radio, &(Si446x_getConfig(radio)->t2fsk).gpio);

  /*
  * Set up GPIO port where the NIRQ from the radio is connected.
  * The NIRQ line is configured in the radio to output the CCA condition.
  * TODO: Cater for situation where CCA is not defined in the radio config.
  */
  pktSetGPIOlineMode(*Si446x_getConfig(radio)->t2fsk.cca.pline,
                     Si446x_getConfig(radio)->t2fsk.cca.mode);

  /* Setup the NCO modulo and oversampling mode. */
  radio_clock_t si_clock = Si446x_getData(radio)->radio_clock;
  uint32_t s = si_clock / 10;
  uint8_t f3 = (s >> 24) & 0xFF;
  uint8_t f2 = (s >> 16) & 0xFF;
  uint8_t f1 = (s >>  8) & 0xFF;
  uint8_t f0 = (s >>  0) & 0xFF;
  (void) Si446x_setProperty32(radio, Si446x_MODEM_TX_NCO_MODE, f3, f2, f1, f0);

  /* Setup the NCO data rate for 2FSK. */
  (void) Si446x_setProperty24(radio, Si446x_MODEM_DATA_RATE,
                       (uint8_t)(speed >> 16),
                       (uint8_t)(speed >> 8),
                       (uint8_t)speed);

  /* Use 2FSK from FIFO (PH). */
  (void) Si446x_setProperty8(radio, Si446x_MODEM_MOD_TYPE, 0x02);

  /* Set PH bit order for 2FSK. */
  (void) Si446x_setProperty8(radio, Si446x_PKT_CONFIG1, 0x01);

  /* No TX filtering used for 2FSK mode. */
}

/**
 *
 */
static void Si446x_setModem2FSK_RX(const radio_unit_t radio,
                                   const radio_mod_t mod) {

  packet_svc_t *handler = pktGetServiceObject(radio);

  /* Configure radio GPIOs. */
  (void) Si446x_configureGPIO(radio, &(Si446x_getConfig(radio)->r2fsk).gpio);

  /* TODO: Everything.... */
  (void) handler;
  (void) mod;
}

/**
 * Radio Settings
 */
#if 0
static uint8_t __attribute__((unused)) Si446x_getChannel(const radio_unit_t radio) {
  const uint8_t state_info[] = {Si446x_REQUEST_DEVICE_STATE_CMD};
  uint8_t rxData[4];
  (void) Si446x_read(radio, state_info, sizeof(state_info), rxData, sizeof(rxData));
  return rxData[1];
}
#endif
/**
 *
 */
static bool Si446x_writeFIFOdataTX(const radio_unit_t radio,
                                 uint8_t *msg, uint8_t size) {
  uint8_t write_fifo[size + 1];
  write_fifo[0] = Si446x_WRITE_TX_FIFO_CMD;
  memcpy(&write_fifo[1], msg, size);
  return Si446x_writeNoCTS(radio, write_fifo, sizeof(write_fifo));
}

/**
 * Set variable with free.
 * Returns false on success, true on fail
 */
static bool Si446x_getFIFOfreeTX(const radio_unit_t radio,
                                    uint8_t reset, uint8_t *data) {
  const uint8_t fifo_info[] = {Si446x_FIFO_INFO_CMD, reset & 0x01};
  uint8_t rxData[Si446x_FIFO_INFO_REPLY_SIZE];
  if (Si446x_read(radio, fifo_info, sizeof(fifo_info), rxData, sizeof(rxData))) {
    return true;
  }
  *data = rxData[1];
  return false;
}

/**
 * Set TX FIFO threshold.
 */
static bool Si446x_setFIFOthresholdTX(const radio_unit_t radio,
                                      si446x_arg_t size) {
  return Si446x_setProperty8(radio, Si446x_PKT_TX_THRESHOLD, size);
}

/**
 * Put radio in standby (low power) mode.
 * All registers are retained.
 * Returns false on success, true on IO failure
 */
bool Si446x_radioStandby(const radio_unit_t radio) {
  TRACE_DEBUG("SI   > Set radio %d to standby state", radio);
#if Si446x_USE_INTERRUPTS
  /* Disable the interrupt event system first. */
  Si446x_disableNIRQEvent(radio);
#endif
  return Si446x_setStandbyState(radio);
}

#if 0
/**
 * @brief  Wake up radio from shut down (powered off)
 * @detail The GPIO connected to radio SDN is open drain output with
 *         pull-up at board initialization. Thus the radio is in shutdown
 *         following board initialization. Si446x GPIO1 by default
 *         outputs CTS during POR. Hence the MCU GPIO connected
 *         to radio GPIO1 can be used to check CTS. The radio GPIOs can
 *         be reconfigured after radio init is complete.
 *
 * @param[in] radio  radio unit ID
 *
 * @return Status of operation
 * @retval false on success
 * @retval true  on failure
 *
 * @api
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

  /* Wait for transceiver to wake up. During start up the POR state is
     on radio GPIO0. GPIO0 goes from zero to one when POR completes.
     The specified maximum wake-up time is 6mS.
     The 50ms is a timeout to cover a fault condition. */
  if (Si446x_getConfig(radio)->gpio0 != PAL_NOLINE) {
    /* There is an MCU port connected to device GPIO0. */
    uint8_t timeout = 50;
    while (--timeout && !pktReadGPIOline(Si446x_getConfig(radio)->gpio0)) {
      chThdSleep(TIME_MS2I(1));
    }
    if (timeout == 0) {
      TRACE_ERROR("SI   > Timeout waiting for POR on radio %i", radio);
      return true;
    }
  }
  /* Return state of CTS after delay. */
  chThdSleep(TIME_MS2I(10));
  return !(pktReadGPIOline(Si446x_getConfig(radio)->gpio1) == PAL_HIGH);
}
#else
/**
 * @brief  Wake up radio from shut down (powered off) state.
 * @detail The GPIO connected to radio SDN is configured as input with pullup
 *         at board initialization. Thus the radio is in shutdown by virtue
 *         of the pull-up following board initialization. The MCU GPIO is
 *         reconfigured to push-pull output here. The Si446x GPIO0 and 1 by
 *         default output POR and CTS state during POR. Hence the MCU GPIOs
 *         connected to radio GPIO0 (if any) and GPIO1 can be used in the
 *         POR process. If an MCU GPIO is not connected to radio GPIO0 then
 *         a blind POR is performed else POR is monitored during wake up.
 *         In both cases the state of CTS is returned indicating success/fail.
 *
 *         The radio GPIOs can be reconfigured after radio init is complete.
 *
 * @param[in] radio  radio unit ID
 *
 * @return Status of operation
 * @retval false on success
 * @retval true  on failure
 *
 * @api
 */
bool Si446x_radioWakeUp(const radio_unit_t radio) {
  TRACE_DEBUG("SI   > Wake up radio %i", radio);

  /* Get the SDN, POR and CTS line MCU GPIO connections. */
  ioline_t sdn = Si446x_getConfig(radio)->sdn;
  ioline_t por = Si446x_getConfig(radio)->gpio0;
  ioline_t cts = Si446x_getConfig(radio)->gpio1;

  /* Can't boot without SDN and CTS line control. */
  if (sdn == PAL_NOLINE || cts == PAL_NOLINE) {
    return true;
  }

  /* First (re)configure port and assert SDN high to ensure radio is asleep. */
  palSetLineMode(sdn, PAL_MODE_OUTPUT_PUSHPULL);
  palSetLine(sdn);
  chThdSleep(TIME_MS2I(100));

  if (por != PAL_NOLINE) {
    /* There is an MCU port connected to device GPIO0. Configure it. */
    palSetLineMode(por, PAL_MODE_INPUT_PULLDOWN);

    /* Assert SDN low to perform radio POR wakeup. */
    palClearLine(sdn);

    /* Wait for device to complete POR. The spec says 6ms maximum. */
    uint8_t timeout = 10;
    while (--timeout && !palReadLine(por)) {
      chThdSleep(TIME_MS2I(1));
    }
    if (timeout == 0) {
      TRACE_ERROR("SI   > Timeout waiting for POR on radio %i", radio);
      return true;
    }
  } else {

    /* Assert SDN low to perform radio POR wakeup. */
    palClearLine(sdn);

    /* Just use a blind timeout for POR time. */
    chThdSleep(TIME_MS2I(10));
  }
  /* Configure MCU port connected to gpio1. */
  palSetLineMode(cts, PAL_MODE_INPUT_PULLDOWN);
  chThdSleep(TIME_MS2I(10));

  /* Return state of CTS. */
  return !(palReadLine(cts) == PAL_HIGH);
}
#endif

/**
 * @brief  The radio is shutdown by setting SDN high.
 * @note   The radio loses all register settings.
 *
 * @param[in] radio  radio unit ID
 *
 * @api
 */
void Si446x_radioShutdown(const radio_unit_t radio) {
  TRACE_DEBUG("SI   > Shut down radio %i", radio);
  packet_svc_t *handler = pktGetServiceObject(radio);
  Si446x_powerDownRadio(radio);
#if Si446x_USE_INTERRUPTS
#if Si446x_USE_SEMAPHORE_INTERRUPT_SYNC == TRUE
  Si446x_disableNIRQEvent(radio);
  /* Reset any waiting thread. */
  chBSemReset(&Si446x_getData(radio)->wait_sem, false);
#else
  /* Terminate the dispatcher thread. */
  thread_reference_t trp = Si446x_getData(radio)->irq_dispatch;
  if (trp != NULL) {
    chThdResume(&trp, MSG_RESET);
    chThdWait(Si446x_getData(radio)->irq_dispatch);
  }
#endif /* Si446x_USE_SEMAPHORE_INTERRUPT_SYNC */
  Si446x_getData(radio)->nirq_active = false;
#endif /* Si446x_USE_INTERRUPTS */
  handler->radio_init = false;
  chThdSleep(TIME_MS2I(50));
}

/**
 * @brief Used by TX to check CCA over a measurement interval.
 * @note  A maximum of 10 samples are taken in the measurement interval.
 *
 * @param[in] radio     radio unit ID
 * @param[in] mod       modulation type
 * @param[in] interval  interval over which CCA will be measure
 *                      Special time values are allowed in measurement interval
 *                      - TIME_INFINITE returns 0
 *                      - TIME_IMMEDIATE returns results of one CCA measurement
 *
 * @return  CCA count of samples taken over the measurement interval
 * @retval  count of 1 - 10 samples depending on system time resolution
 *
 * @api
 */
static cnt_t Si446x_checkCCAthresholdForTX(const radio_unit_t radio,
                                           const radio_mod_t mod,
                                           const sysinterval_t interval) {

  ioline_t cca_line;
  /* Read CAA as setup by mod type. */
  switch (mod) {
  case MOD_2FSK_300:
  case MOD_2FSK_9k6:
  case MOD_2FSK_19k2:
  case MOD_2FSK_38k4:
  case MOD_2FSK_57k6:
  case MOD_2FSK_76k8:
  case MOD_2FSK_96k:
  case MOD_2FSK_115k2:
    cca_line = *Si446x_getConfig(radio)->t2fsk.cca.pline;
    break;

  case MOD_AFSK: {
    cca_line = *Si446x_getConfig(radio)->tafsk.cca.pline;
    break;
  }

  case MOD_CW:
    return false;

  case MOD_NONE:
    return false;
  }

  if (cca_line == PAL_NOLINE || interval == TIME_INFINITE) {
    return (cnt_t)0;
  }
  cnt_t cca = 0;

  /* Take 10 measurements over the period. */
  sysinterval_t slice = interval / 10;

  /* Measure sliced CCA instances in period. */
  for (uint16_t i = 0; i <= slice; i++) {
    cca += palReadLine(cca_line);
    /* Sleep one time slice. */
    chThdSleep(slice);
  }
  /* Return result. */
  return cca;
}

#if Si446x_USE_PACKET_END_INTERRUPT == FALSE
/*
 * Wait for radio to exit state
 * Return
 *  MSG_OK if state exited within timeout.
 *  MSG_TIMEOUT if state not exited within timeout.
 *  MSG_ERROR if read error from radio.
 */
static msg_t Si446x_waitForStateExit(const radio_unit_t radio,
                                 si446x_state_t state,
                                 sysinterval_t timeout) {
  si446x_state_t s;
  if (timeout == TIME_IMMEDIATE) {
    if (Si446x_requestDeviceState(radio, &s, NULL))
        return MSG_ERROR;
    return s != state ? MSG_OK : MSG_TIMEOUT;
  }
  if (timeout == TIME_INFINITE) {
    do {
      chThdSleep(TIME_MS2I(10));
      if (Si446x_requestDeviceState(radio, &s, NULL))
        return MSG_ERROR;
    } while (s == state);
    return MSG_OK;
  }

  time_msecs_t ms = chTimeI2MS(timeout);
  do {
    chThdSleep(TIME_MS2I(1));
    if (Si446x_requestDeviceState(radio, &s, NULL))
      return MSG_ERROR;
  } while (--ms != 0 && s == state);
  return (ms == 0) ? MSG_TIMEOUT : MSG_OK;
}

/*
 * Return status of transmit state
 * Return MSG_OK if not transmit state or state ended before timeout
 * Otherwise return MSG_TIMEOUT.
 */
static msg_t Si446x_waitTransmitEnd(const radio_unit_t radio, sysinterval_t timeout) {
  return Si446x_waitForStateExit(radio, Si446x_TX, timeout);
}
#endif

/**
 * @brief  Wait for a clear time slot and initiate packet transmission.
 *
 * @param[in]   freq
 * @param[in]   step
 * @param[in]   chan
 * @param[in]   power
 * @param[in]   mod
 * @param[in]   size
 * @param[in]   rssi
 * @param[in]   cca_timeout
 *
 * @return  Status of operation
 * @retval  true  operation completed successfully
 * @retval  false an error occurred
 *
 * @api
 */
static bool Si446x_transmitWithCCA(const radio_unit_t radio,
                            const radio_freq_hz_t freq,
                            const radio_chan_hz_t step,
                            const radio_ch_t chan,
                            const radio_pwr_t power,
                            const radio_mod_t mod,
                            const radio_payload_t size,
                            const radio_squelch_t rssi,
                            sysinterval_t cca_timeout) {

  /* Get an absolute operating frequency in Hz. */
  radio_freq_hz_t op_freq = pktComputeOperatingFrequency(radio, freq,
                                                      step, chan, RADIO_TX);

  if (op_freq == FREQ_INVALID) {
    TRACE_ERROR("SI   > Frequency out of range");
    TRACE_ERROR("SI   > abort transmission on radio %d", radio);
    return false;
  }

  mod_params_t mp;
  mp.type = mod;
  if (!pktLookupModParameters(radio, &mp)) {
    TRACE_ERROR( "SI   > Invalid modulation code on radio %d", radio);
    return false;
  }

  /* Check for ready state. */
  if (Si446x_waitForState(radio, Si446x_READY, TIME_S2I(5)) == MSG_TIMEOUT) {
    /* Force 446x into ready state. */
    (void) Si446x_setReadyState(radio);
    TRACE_ERROR("SI   > %s timeout waiting for ready state radio %d",
                getModulation(mod), radio);
    return false;
  }

  /* Check for blind send request. */
  if (rssi != PKT_SI446X_NO_CCA_RSSI) {
    /*
     *  Listen on the TX frequency for a clear channel.
     *  - The radio is setup for CCA receive mode.
     *  - Set the RSSI threshold in the radio.
     *  - Put the radio into RX state.
     *  - Measure the CCA status.
     */

    /* Frequency is an absolute frequency in Hz. */
    (void) Si446x_setSynthParameters(radio, op_freq, step, 0);

    /* Set receiver for CCA detection. CCA may not be supported. */
    if (!Si446x_setModemCCA_Detection(radio)) {
      /* Setup for CCA detection was OK. Set the RSSI threshold. */
      (void) Si446x_setProperty8(radio, Si446x_MODEM_RSSI_THRESH, rssi);

      /* Minimum timeout for CCA is 1 second. */
      if (cca_timeout < TIME_S2I(1) || cca_timeout == TIME_INFINITE) {
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
       * So wait on the CTS before proceeding.
       */
      if (Si446x_startRXStateWaitCTS(radio, chan)) {
        TRACE_ERROR("SI   > %s timeout waiting for RX start on radio %d",
                    getModulation(mod), radio);
        return false;
      }
  #define CCA_MEASURE_INTERVAL   TIME_MS2I(50)

      systime_t t0 = chVTGetSystemTime();
      sysinterval_t cca_time = CCA_MEASURE_INTERVAL;
      si446x_state_t s;
      do {
        if (Si446x_checkCCAthresholdForTX(radio, mod, CCA_MEASURE_INTERVAL) == 0)
          break;
        if (Si446x_requestDeviceState(radio, &s, NULL))
          return false;
      } while (s == Si446x_RX
          && cca_timeout > (cca_time = chVTTimeElapsedSinceX(t0)));

      /* Clear channel timing. */
      TRACE_DEBUG( "SI   > CCA validated at %d milliseconds on radio %d",
                                           chTimeI2MS(cca_time), radio);

      /* Take radio out of receive state now. */
      msg_t msg = Si446x_setStateWaitChange(radio, Si446x_READY, TIME_MS2I(100));
      if (msg != MSG_OK) {
        TRACE_ERROR("SI   > %s error waiting for ready state radio %d",
                    getModulation(mod), radio);
        return false;
      }
    } /* End if CCA detection setup OK. */
  } /* End if CCA. */

  /* Frequency is an absolute frequency in Hz. */
  (void) Si446x_setSynthParameters(radio, op_freq, step, mp.tx_dev);

  /* Setup TX configuration. */
  switch (mod) {
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

  TRACE_INFO("SI   > Transmit with radio %d on %d.%03d MHz",
             radio, op_freq/1000000, (op_freq%1000000)/1000);

  /*
   * Set power level and start transmit.
   * The 446x will not return CTS until TX starts.
   */
  if (Si446x_setPowerLevel(radio, power)) {
    return false;
  }
  if (Si446x_startPacketTXWaitCTS(radio, chan, size)) {
    /* Force 446x out of state. */
    (void) Si446x_setReadyState(radio);
    TRACE_ERROR("SI   > %s transmit failed to start on radio %d",
                getModulation(mod), radio);
    return false;
  }
#if 0
  /* Wait for transmission to start. */
  if (Si446x_waitForState(radio, Si446x_TX, TIME_MS2I(100)) == MSG_TIMEOUT) {
    /* Force 446x out of state. */
    (void) Si446x_setReadyState(radio);
    TRACE_ERROR("SI   > %s transmit failed to start on radio %d",
                getModulation(mod), radio);
    return false;
  }
#endif
  return true;
}

/*
 * @brief Start or restore reception.
 *
 * @param[in] radio radio unit ID
 * @param[in] rx_frequency
 * @param[in] rx_step
 * @param[in] rx_chan
 * @param[in] rx_rssi
 * @param[in] rx_mod
 *
 * @return  Status of operation
 * @retval  true if RX was enabled and/or resumed OK.
 * @retval  false if RX was not enabled.
 *
 * @api
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

  if (op_freq == FREQ_INVALID) {
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
   *  Initialise radio before any commands.
   *  The device may have been powered down or placed in standby state.
   *  Returns true on success, false on fail.
   */
  if (Si446x_driverInit(radio)) {
    /* Radio did not initialise. */
    TRACE_ERROR("SI   > Radio %d failed to initialise when enabling RX", radio);
    return false;
  }

#if Si446x_USE_PACKET_END_INTERRUPT == FALSE
  /*
   * Wait for any TX in progress to end.
   * TODO: Is this now redundant since the TX thread waits on packet end
   *       before releasing the radio.
   */
  if (Si446x_waitTransmitEnd(radio, TIME_S2I(10))) {
    /* Force ready state. */
    (void) Si446x_setReadyState(radio);
    TRACE_ERROR("SI   > Timeout waiting for TX state end "
                 "before starting %s receive on radio %d",
                 getModulation(rx_mod), radio);
  }
#endif
  /* Configure radio for modulation type. */
  switch (rx_mod) {
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
    (void) Si446x_setModemCCA_Detection(radio);
    break;

  case MOD_NONE:
    TRACE_ERROR("SI   > Invalid modulation type %s in receive",
                getModulation(rx_mod));
    TRACE_ERROR("SI   > Abort reception on radio %d", radio);
    return false;
  } /* End switch on mod type. */

  TRACE_INFO("SI   > Receive with radio %d on %d.%03d MHz",
             radio, op_freq/1000000, (op_freq%1000000)/1000);

  /* Frequency must be an absolute frequency in Hz. */
  if (!Si446x_setSynthParameters(radio, op_freq, rx_step, 0)) {
    return false;
  }
  /* Set squelch level. */
  if (Si446x_setProperty8(radio, Si446x_MODEM_RSSI_THRESH, rx_rssi)) {
      return false;
  }

  /*
   * Start the receiver.
   * The si446x will not set CTS until RX has started.
   * Thus any subsequent commands will wait on CTS in SPI handler.
   */
  return !Si446x_startRXState(radio, rx_chan);
}

#if Si446x_USE_COMMON_TX_THREAD == TRUE
/**
 * @brief  Initialise a link level packet handler
 *
 * @param[in] rto       pointer to a @p radioTask object
 * @param[in] iterator  pointer to a @p txIterator object
 * @param[in] sampler   pointer to a @p reSampler object
 *
 * @retval size of encoded data stream in bytes
 *
 * @api
 *
 */
static uint16_t Si446x_prepareLinkLevelHandler(radio_task_object_t *rto,
                                       tx_iterator_t *iterator,
                                       re_sampler_t *sampler) {

  radio_mod_t mod = rto->radio_dat.type;
  packet_t pp = rto->radio_dat.pkt.packet_out;
  switch (mod) {
  case MOD_2FSK_300:
  case MOD_2FSK_9k6:
  case MOD_2FSK_19k2:
  case MOD_2FSK_38k4:
  case MOD_2FSK_57k6:
  case MOD_2FSK_76k8:
  case MOD_2FSK_96k:
  case MOD_2FSK_115k2:
    /*
     * Set NRZI encoding format.
     * Iterator object.
     * Packet reference.
     * Preamble length (HDLC flags).
     * Postamble length (HDLC flags).
     * Tail length (HDLC zeros).
     * Scramble on for 2FSK.
     */
    pktStreamIteratorInit(iterator, pp->frame_data, pp->frame_len,
                                          100, 10, 10, true);

    /* For 2FSK the PH is 1:1 with the NRZI bit stream. */
    sampler->sample_rate = 1;
    break;

  case MOD_AFSK: {
    /*
     * Set NRZI encoding format.
     * Iterator object.
     * Packet reference.
     * Preamble length (HDLC flags).
     * Postamble length (HDLC flags).
     * Tail length (HDLC zeros).
     * Scramble off for AFSK.
     */
    pktStreamIteratorInit(iterator, pp->frame_data, pp->frame_len,
                                          30, 10, 10, false);
    sampler->phase_delta = PHASE_DELTA_1200;
    sampler->mark_delta = PHASE_DELTA_1200;
    sampler->space_delta = PHASE_DELTA_2200;
    sampler->sample_rate = SAMPLES_PER_AFSK_SYMBOL;
    break;
  }

  case MOD_CW:
    return false;

  case MOD_NONE:
    return false;
  }
  /* Get the stream size by specifying buffer = NULL. */
  return pktStreamEncodingIterator(iterator, NULL, 0);
}
#endif /* Si446x_USE_COMMON_TX_THREAD */

/**
 * @brief  Takes NRZI data and produces the radio PH bit stream.
 * @detail In the case of AFSK the PH is fed up-sampled NRZI data
 *         as a bit stream.
 *         The transmitter bit rate is run at the declared sample rate.
 *         In the 446x the TX filter is set to shape the resultant
 *         up-sampled bit stream.
 *         For 2FSK the sample rate is 1 so the data fed to the
 *         PH is 1:1 with NRZI.
 *
 * @param[in] sampler pointer to a re-sampler object
 * @param[in] buf     pointer to the buffer to receive the re-sampled data
 *
 * @return byte value to be fed to the radio FIFO (packet handler)
 *
 * @api
 */
static uint8_t Si446x_getPacketHandlerBits(re_sampler_t *sampler,
                                           uint8_t *buf) {
  uint8_t b = 0;
  if (sampler->sample_rate > 1) {
    for (uint8_t i = 0; i < 8; i++) {
      if (sampler->current_sample_in_baud == 0) {
        if ((sampler->packet_pos & 7) == 0) {
          /* Get next byte. */
          sampler->current_byte = buf[sampler->packet_pos >> 3];
        }
        else {
          /* Get next bit. */
          sampler->current_byte >>= 1;
        }
      }

      /* Toggle tone between 1200 <> 2200. */
      sampler->phase_delta = (sampler->current_byte & 1)
          ? sampler->mark_delta : sampler->space_delta;
      /* Add delta-phase (position within SAMPLES_PER_1200_BAUD). */
      sampler->phase += sampler->phase_delta;
      /* Set modulation bit. */
      b |= ((sampler->phase >> 16) & 1) << i;

      if (++sampler->current_sample_in_baud == sampler->sample_rate) {
        /* All samples done for this symbol. */
        sampler->current_sample_in_baud = 0;
        sampler->packet_pos++;
      }
    }
  } else {
    b = buf[sampler->packet_pos++];
  }
  return b;
}
#if Si446x_USE_TRANSMIT_TIMEOUT == TRUE
/**
 * This is a failsafe timeout for the transmit thread.
 * If this timeout happens something has failed in the TX thread IO.
 * i.e. the TX thread has not seen an exception or normal termination and has not stopped the virtual timer.
 * Currently the init function lacks timeouts so adding those should remove the possibility of an overall TX timeout.
 *
 * If there is a timeout the handling is complex.
 * Since we are in ISR context SPI is not allowed to reset the radio or SPI.
 * It is not possible to know if this is a transient or permanent h/w fail.
 *
 * So is it best to...
 * A) Use a sledge hammer solution by halting and let the watchdog reset the system.
 * OR
 * B) Queue a new RTO with a TX_HARDWARE_ERROR type so the RM is aware of an issue.
 *    The radio manager could reset the radio sempahore then shutdown and re-init the radio.
 *    Set the thread terminate in the TX thread (chThdTerminate(...)).
 *    If the TX thread does recover it should free packet and RTO resource and quit.
 *    Then the idle sweeper will release the thread memory.
 *    Otherwise the TX thread remains as a zombie and has the radio and resources locked.
 * OR
 * C) Get radio semaphore and if radio is still locked to the referenced thread, then reset the semaphore.
 *    All functions that wait on radio lock will need to responded to MSG_RESET semaphore result.
 *    Then proceed as in B to set thread terminate, etc.
 *
 */
static void Si446x_transmitTimeoutI(thread_t *thd) {
  (void) thd;
  chDbgAssert(false, "transmit timeout panic");
  chSysHalt("Transmit timeout");
}
#endif
/**
 * @brief Release transmit resources.
 * @notes The task object is returned to the Radio manager.
 *        The radio manager frees the task object and decreases
 *        the transmit reference count.
 *        The calling thread is scheduled for release.
 *
 * @param[in] Pointer to @p radioTask object.
 * @param[in] Packet buffer pointer.
 * @param[in] Reason code for release returned to RM.
 * @param[in] Pointer to an optional string message for TRACE output.
 *
 *
 */
static void Si446x_transmitRelease(radio_task_object_t *rto, packet_t pp,
                                     msg_t msg, char *str) {

  /* Free packet object memory. */
  if (pp != NULL) {
    pktReleaseBufferChain(pp);
  }
  /* Set the status in the RTO and dispatch it back to the RM. */
  radio_unit_t radio = rto->handler->radio;
  radio_mod_t mod = rto->radio_dat.type;
  rto->result = msg;
  pktRadioSendComplete(rto);
  switch (msg) {
  case MSG_OK:
    TRACE_INFO("SI   > TX OK on radio %d, mod %s, %s", radio,
              getModulation(mod), str);
    break;

  case MSG_RESET:
    TRACE_WARN("SI   > TX RESET on radio %d, mod %s, %s", radio,
               getModulation(mod), str);
    break;

  case MSG_TIMEOUT:
    TRACE_ERROR("SI   > TX TIMEOUT on radio %d, mod %s, %s", radio,
                getModulation(mod), str);
    break;

  case MSG_ERROR:
    TRACE_MON("SI   > TX ERROR on radio %d, mod %s, %s", radio,
              getModulation(mod), str);
    break;

  default:
    TRACE_MON("SI   > TX message on radio %d, mod %s, %s", radio,
              getModulation(mod), str);
    break;
  }
  pktThdTerminateSelf();
  /* We never arrive here. */
}

/**
 * @brief  Common send thread.
 * @detail Controls the entire transmission of a packet or linked sequence
 *         of packets. Handles NRZI encoding and data whitening as required.
 *         Also handles up-sampling for AFSK transmission to enable use of
 *         the radio packet handler for sending. Handles various PHY
 *         modulation settings within the radio.
 *         Will perform Clear Channel Assessment to avoid transmitting
 *         another station. Will also wait for an in progress receive before
 *         testing CCA.
 *
 * @param[in] pointer to a radio task object
 *
 * @special
 */
#if Si446x_USE_COMMON_TX_THREAD == TRUE
THD_FUNCTION(Si446x_packetTransmitHandler, arg) {
#else
THD_FUNCTION(bloc_si_fifo_feeder_afsk, arg) {
#endif
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

  /* Lock radio for packet transmit session. */
  msg_t msg = pktLockRadio(radio, RADIO_TX, rto->radio_dat.timer);

  if (msg != MSG_OK) {
    /* Lock returned MSG_TIMEOUT or MSG_RESET. Abort this TX request.
       Chained packets are freed and not sent. */
    Si446x_transmitRelease(rto, pp, msg, "attempting radio acquisition");
    /* We never arrive here. */
  }

  /* Wait for any active receive stream in progress. Force inactive on timeout.
     Any stream is stopped but the 446x would remain in receive state. */
  msg = pktSetReceiveStreamStandby(radio,
                                    rto->radio_dat.rssi == PKT_SI446X_NO_CCA_RSSI
                                    ? TIME_IMMEDIATE : TIME_MS2I(2000));
  if (msg == MSG_ERROR) {
    /* Parameter error. */
    chDbgAssert(false, "parameter error");
  }

  /* Initialise the radio. It may be uninitialised (powered down) or in standby. */
  if (Si446x_driverInit(radio)) {
    /* Radio did not initialise. */

    /* Unlock radio. */
    pktUnlockRadio(radio);

    /* Abort this TX request. Chained packets are freed and not sent. */
    Si446x_transmitRelease(rto, pp, MSG_ERROR, "failed to initialise");
    /* We never arrive here. */
  }
#if Si446x_USE_TRANSMIT_TIMEOUT == TRUE
  /* Initialize timer and variables for AFSK encoder. */
  virtual_timer_t send_timer;
  chVTObjectInit(&send_timer);
#endif
  /*
   * Use the specified CCA RSSI level.
   * CCA RSSI level will be set to blind send after first packet.
   */
  radio_squelch_t rssi = rto->radio_dat.rssi;

  /* Loop and send packet(s). */
  msg_t exit_msg;
  do {

    /* get an iterator and re-sampler object. */
    tx_iterator_t iterator;
    re_sampler_t sampler = {0};

#if Si446x_USE_COMMON_TX_THREAD
    uint16_t all = Si446x_prepareLinkLevelHandler(rto, &iterator, &sampler);
    if (all == 0) {
      /* Nothing would be encoded. Release packet send object. Unlock radio. */
      pktUnlockRadio(radio);

      /* Abort this TX request. Chained packets are freed and not sent. */
      Si446x_transmitRelease(rto, pp, MSG_ERROR, "no NRZI data encoded");
      /* We never arrive here. */
    } /* End if (all == 0). */
#else
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

      /* Unlock radio. */
      pktUnlockRadio(radio);

      /* Abort this TX request. Chained packets are freed and not sent. */
      Si446x_transmitRelease(rto, pp, MSG_ERROR, "no NRZI data encoded");
      /* We never arrive here. */
    } /* End if (all == 0). */
#endif /* Si446x_USE_COMMON_TX_THREAD */
    /* Allocate buffer and perform NRZI encoding. */
    uint8_t layer0[all];
    pktStreamEncodingIterator(&iterator, layer0, all);
#if Si446x_USE_COMMON_TX_THREAD
    all *= sampler.sample_rate;
#else

    /* Scale up to number of data bits per baud to send for this encoder. */
    all *= SAMPLES_PER_1200_BAUD;

    /*
     * The radio is locked and packet receive is now inactive.
     * The 446x has been re-initialised.
     */
    //up_sampler_t sampler = {0};
    sampler.phase_delta = PHASE_DELTA_1200;
    sampler.mark_delta = PHASE_DELTA_1200;
    sampler.space_delta = PHASE_DELTA_2200;
    sampler.sample_rate = SAMPLES_PER_AFSK_BAUD;
#endif /* Si446x_USE_COMMON_TX_THREAD */

#if Si446x_USE_PACKET_END_INTERRUPT == FALSE
    /*
     *  Wait for current radio activity to stop.
     *  TODO: Used for linked send. Put TX end check inside loop to remove?
     */
    if (Si446x_waitForState(radio, Si446x_READY, TIME_S2I(5)) == MSG_TIMEOUT) {
      /* Force 446x out of TX state. */
      (void) Si446x_setReadyState(radio);

      /* Unlock radio. */
      pktUnlockRadio(radio);

      /* Abort this TX request. Chained packets are freed and not sent. */
      Si446x_transmitRelease(rto, pp, MSG_ERROR, "failed to get radio ready");
      /* We never arrive here. */
    }
#endif
    /*
     * Get the FIFO buffer amount currently available.
     * The TX FIFO is reset to get full capacity.
     */
    uint8_t free = -1;
    if (Si446x_getFIFOfreeTX(radio, 0x01, &free) || free == 0) {
      /* FIFO command failed. */

      /* Unlock radio. */
      pktUnlockRadio(radio);

      /* Abort this TX request. Chained packets are freed and not sent. */
      Si446x_transmitRelease(rto, pp, MSG_ERROR, "FIFO info command failed");
      /* We never arrive here. */
    }

    /* Allocate FIFO feeder buffer. */
    uint8_t localBuffer[free];

    /* Calculate initial FIFO fill. */
    uint16_t c = (all > free) ? free : all;
#if Si446x_USE_TRANSMIT_TIMEOUT == TRUE
    /* The exit message if all goes well. */
    exit_msg = MSG_OK;
#endif
    /* Initial FIFO load. */
    for (uint16_t i = 0;  i < c; i++)
      localBuffer[i] = Si446x_getPacketHandlerBits(&sampler, layer0);
    if (Si446x_writeFIFOdataTX(radio, localBuffer, c)) {

      /* Something failed in FIFO load so set radio ready to clear it. */
      (void) Si446x_setReadyState(radio);

      /* Unlock radio. */
      pktUnlockRadio(radio);

      /* Abort this TX request. Chained packets are freed and not sent. */
      Si446x_transmitRelease(rto, pp, MSG_ERROR, "write to FIFO failed");
      /* We never arrive here. */
    }

    /* Set the initial FIFO tide monitoring level for this packet. */
    uint8_t tide = 0;

    if (!Si446x_transmitWithCCA(radio,
                                  rto->radio_dat.base_frequency,
                                  rto->radio_dat.step_hz,
                                  rto->radio_dat.channel,
                                  rto->radio_dat.tx_power,
                                  rto->radio_dat.type,
                                  all,
                                  rssi,
                                  TIME_S2I(SI446X_TX_CCA_TIMEOUT))) {

      /* Transmit start failed. */
      (void) Si446x_setReadyState(radio);

      /* Unlock radio. */
      pktUnlockRadio(radio);

      /* Abort this TX request. Chained packets are freed and not sent. */
      Si446x_transmitRelease(rto, pp, MSG_ERROR, "transmit start failed");
      /* We never arrive here. */
    } /* End if (!transmit). */

    /* Set the FIFO threshold to half empty. */
    if (Si446x_setFIFOthresholdTX(radio, free / 2)) {

      /* Something failed so set radio ready to clear it. */
      (void) Si446x_setReadyState(radio);

      /* Unlock radio. */
      pktUnlockRadio(radio);

      /* Abort this TX request. Chained packets are freed and not sent. */
      Si446x_transmitRelease(rto, pp, MSG_ERROR, "set FIFO threshold failed");
      /* We never arrive here. */
    }
#if Si446x_USE_TRANSMIT_TIMEOUT == TRUE
    /* Start/re-start transmission timeout timer for this packet. */
    chVTSet(&send_timer, TIME_S2I(SI446X_TRANSMIT_TIMEOUT),
            (vtfunc_t)Si446x_transmitTimeoutI, chThdGetSelfX());
#endif
    /* Feed the FIFO while data remains to be sent. */
    while ((all - c) > 0) {
#if Si446x_USE_FIFO_THRESHOLD_INTERRUPT == TRUE
      /* Enable TX FIFO interrupt and suspend thread waiting for
             interrupt. */
#if Si446x_USE_SEMAPHORE_INTERRUPT_SYNC == TRUE
      msg_t msg = Si446x_setRadioInterruptMask(radio,
                                       Si446x_INT_CTL_PH_REG_INDEX,
                                       Si446x_INT_CTL_PH_TX_FIFO_THRESH_MASK,
                                       TIME_S2I(SI446X_TX_FIFO_TIMEOUT));
      if (msg != MSG_OK) {
        /* Radio did not interrupt. Something failed or the radio
           has been closed. */

#if Si446x_USE_TRANSMIT_TIMEOUT == TRUE
        chVTReset(&send_timer);
#endif
        /* Set radio to ready state and unlock. */
        (void) Si446x_setReadyState(radio);
        pktUnlockRadio(radio);

        /* Abort this TX request. Chained packets are freed and not sent. */
        Si446x_transmitRelease(rto, pp, msg, "FIFO interrupt set failed");
        /* We never arrive here. */
      }
      msg_t result = Si446x_waitRadioInterrupt(radio, TIME_MS2I(100));
#else
      msg_t result = Si446x_waitRadioInterrupt(radio,
                                     Si446x_INT_CTL_PH_REG_INDEX,
                                     Si446x_INT_CTL_PH_TX_FIFO_THRESH_MASK,
                                     TIME_S2I(SI446X_TX_FIFO_TIMEOUT));
#endif /* Si446x_USE_SEMAPHORE_INTERRUPT_SYNC */
      if (result != MSG_OK) {

        /* Radio did not interrupt. Something failed or the radio
           has been closed. */

#if Si446x_USE_TRANSMIT_TIMEOUT == TRUE
        chVTReset(&send_timer);
#endif
        /* Set radio to ready state and unlock. */
        (void) Si446x_setReadyState(radio);
        pktUnlockRadio(radio);

        /* Abort this TX request. Chained packets are freed and not sent. */
        Si446x_transmitRelease(rto, pp, result, "FIFO interrupt timeout");
        /* We never arrive here. */
      }
#else /* Si446x_USE_FIFO_THRESHOLD_INTERRUPT != TRUE */
#if Si446x_USE_TRANSMIT_TIMEOUT == TRUE
#else
        chThdSleep(TIME_MS2I(1));
#endif
#endif /* Si446x_USE_FIFO_THRESHOLD_INTERRUPT == TRUE */
      uint8_t more;
      if (Si446x_getFIFOfreeTX(radio, 0x00, &more)) {

        /* Radio SPI command failed. */
#if Si446x_USE_TRANSMIT_TIMEOUT == TRUE
        chVTReset(&send_timer);
#endif
        /* Set radio to ready state and unlock. */
        (void) Si446x_setReadyState(radio);
        pktUnlockRadio(radio);

        /* Abort this TX request. Chained packets are freed and not sent. */
        Si446x_transmitRelease(rto, pp, MSG_ERROR, "read of FIFO free failed");
        /* We never arrive here. */
      }

      /* Fill free part of TX FIFO. */
      if (more != 0) {
        /* Update the FIFO free high water mark. */
        tide = (more > tide) ? more : tide;

        /* If there is more free than we need use remainder only.
           The loop will terminate in this pass and the FIFO interrupt
           won't be set again. */
        more = (more > (all - c)) ? (all - c) : more;

        /* Load the FIFO. */
        for (uint16_t i = 0; i < more; i++)
          localBuffer[i] = Si446x_getPacketHandlerBits(&sampler, layer0);

        if (Si446x_writeFIFOdataTX(radio, localBuffer, more)) {

          /* Radio IO failed. */
#if Si446x_USE_TRANSMIT_TIMEOUT == TRUE
          chVTReset(&send_timer);
#endif
          /* Set radio to ready state and unlock. */
          (void) Si446x_setReadyState(radio);
          pktUnlockRadio(radio);

          /* Abort this TX request. Any chained packets are not sent. */
          Si446x_transmitRelease(rto, pp, MSG_ERROR, "write to FIFO failed");
          /* We never arrive here. */
        }
      }

      /* Update count of sent bytes. */
      c += more;
    } /* End while (). */
#if Si446x_USE_TRANSMIT_TIMEOUT == TRUE
    chVTReset(&send_timer);

    if (exit_msg == MSG_TIMEOUT) {
      /* Set radio to ready state and unlock. */
    }
      (void) Si446x_setReadyState(radio);
      pktUnlockRadio(radio);

      /* Abort this TX request. Any chained packets are not sent. */
      Si446x_transmitRelease(rto, pp, MSG_ERROR, "transmit timed out");
      /* We never arrive here. */
    }
#endif

#if Si446x_USE_PACKET_END_INTERRUPT == TRUE
    /* Enable TX FIFO interrupt and suspend thread waiting for interrupt. */
#if Si446x_USE_SEMAPHORE_INTERRUPT_SYNC == TRUE
    msg_t msg = Si446x_setRadioInterruptMask(radio,
                                         Si446x_INT_CTL_PH_REG_INDEX,
                                         Si446x_INT_CTL_PH_PACKET_SENT_MASK,
                                         TIME_S2I(SI446X_TX_FIFO_TIMEOUT));
    if (msg != MSG_OK) {

      /* Setting radio interrupt failed. Timeout or the radio
         has been closed. */

#if Si446x_USE_TRANSMIT_TIMEOUT == TRUE
      chVTReset(&send_timer);
#endif
      /* Set radio to ready state and unlock. */
      (void) Si446x_setReadyState(radio);
      pktUnlockRadio(radio);

      /* Abort this TX request. Chained packets are freed and not sent. */
      Si446x_transmitRelease(rto, pp, msg, "FIFO interrupt set failed");
      /* We never arrive here. */
    }
    exit_msg = Si446x_waitRadioInterrupt(radio, TIME_MS2I(100));
#else
    exit_msg = Si446x_waitRadioInterrupt(radio,
                                             Si446x_INT_CTL_PH_REG_INDEX,
                                             Si446x_INT_CTL_PH_PACKET_SENT_MASK,
                                             TIME_S2I(SI446X_TX_FIFO_TIMEOUT));
#endif
    if (exit_msg != MSG_OK) {
      /* Radio did not interrupt. */

#if Si446x_USE_TRANSMIT_TIMEOUT == TRUE
      chVTReset(&send_timer);
#endif
      /* Set radio ready to clear it. */
      (void) Si446x_setReadyState(radio);

      /* Unlock radio. */
      pktUnlockRadio(radio);

      /* Abort this TX request. Any chained packets are not sent. */
      Si446x_transmitRelease(rto, pp, exit_msg, "packet sent interrupt fail");
      /* We never arrive here. */
    }
#endif

    /* No CCA on subsequent packets of burst send. */
    rssi = PKT_SI446X_NO_CCA_RSSI;

    /* Tide warning at 75%. */
    if (tide > (free * 2 / 3)) {
      /* Warn when free level reaches > 75% of FIFO size. */
      TRACE_WARN("SI   > %s TX FIFO free of %i exceeded safe level of %i on radio %d",
                 getModulation(rto->radio_dat.type), tide, (free * 3 / 2), radio);
    }

    /* Release current packet and get the next linked packet.
       Will be NULL if none. */
  } while ((pp = pktReleaseBufferObject(pp)) != NULL);  /* End do. */
#if Si446x_USE_PACKET_END_INTERRUPT == FALSE
  /* Wait for transmission to finish. */
  if ((exit_msg = Si446x_waitForState(radio, Si446x_READY,
                                      TIME_S2I(5))) != MSG_OK) {
    /* Force 446x out of TX state. */
    (void) Si446x_setReadyState(radio);
    TRACE_ERROR("SI   > %s transmit failed to cease on radio %d",
                getModulation(rto->radio_dat.type), radio);
  }
#endif
  /* Unlock radio. */
  pktUnlockRadio(radio);
#if Si446x_USE_TRANSMIT_TIMEOUT == TRUE
  /* Signal RTM that the TX is complete. */
  Si446x_transmitRelease(rto, NULL, exit_msg, NULL);
#else
  Si446x_transmitRelease(rto, NULL, exit_msg, "transmit end");
#endif
  /* We never arrive here. */
}

/*
 *
 */
bool Si446x_blocSendAFSK(radio_task_object_t *const rt) {

    thread_t *afsk_feeder_thd = chThdCreateFromHeap(NULL,
                THD_WORKING_AREA_SIZE(SI_AFSK_FIFO_MIN_FEEDER_WA_SIZE),
                "tx_afsk_queued",
                NORMALPRIO - 10,
#if Si446x_USE_COMMON_TX_THREAD == TRUE
                Si446x_packetTransmitHandler,
#else
                bloc_si_fifo_feeder_afsk,
#endif
                rt);

    if (afsk_feeder_thd == NULL) {
      TRACE_ERROR("SI   > Unable to create AFSK transmit thread for radio %d",
                  rt->handler->radio);
      return false;
    }
    return true;
}

#if Si446x_USE_COMMON_TX_THREAD == FALSE
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

    Si446x_transmitRelease(rto, pp, msg, NULL);
    /* We never arrive here. */
  }

  /* Wait for any active receive stream in progress. Force inactive on timeout.
     Any stream is stopped but the 446x would remain in receive state. */
  msg = pktSetReceiveStreamStandby(radio,
                                    rto->radio_dat.rssi == PKT_SI446X_NO_CCA_RSSI
                                    ? TIME_IMMEDIATE : TIME_MS2I(2000));
  if (msg == MSG_ERROR) {
    /* Parameter error. */
    chDbgAssert(false, "parameter error");
  }

  /* Initialise the radio. It may be uninitialised, in standby or in receive. */
  if (Si446x_driverInit(radio)) {
    /* Radio did not initialise. */
    TRACE_ERROR("SI   > Radio %d failed to initialise for %s TX", radio,
                getModulation(rto->radio_dat.type));

    /* Unlock radio. */
    pktUnlockRadio(radio);

    Si446x_transmitRelease(rto, pp, MSG_ERROR, NULL);
    /* We never arrive here. */
  }

  /*
   *  Initialize transmit timeout timer and iterator. */
#if Si446x_USE_TRANSMIT_TIMEOUT == TRUE
  virtual_timer_t send_timer;
  chVTObjectInit(&send_timer);
#endif
  tx_iterator_t iterator;

  /* The exit message. */
  msg_t exit_msg;

  /*
   * Use the specified CCA RSSI level.
   * This can be be blind send (PKT_SI446X_NO_CCA_RSSI).
   * In burst mode CCA will be set to blind send after the first packet.
   */
  radio_squelch_t rssi = rto->radio_dat.rssi;
  packet_t np;
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

      Si446x_transmitRelease(rto, pp, MSG_ERROR, NULL);
      /* We never arrive here. */
    }

    /* Allocate buffer and perform NRZI encoding in a single pass. */
    uint8_t layer0[all];

    pktStreamEncodingIterator(&iterator, layer0, all);
#if Si446x_USE_PACKET_END_INTERRUPT == FALSE
    /* Wait for current radio activity to stop. */
    if (Si446x_waitForState(radio, Si446x_READY, TIME_S2I(5)) == MSG_TIMEOUT) {
      /* Force 446x out of TX state. */
      (void) Si446x_setReadyState(radio);
      TRACE_ERROR("SI   > %s transmit failed to get radio ready on radio %d",
                  getModulation(rto->radio_dat.type), radio);

      /* Unlock radio. */
      pktUnlockRadio(radio);

      Si446x_transmitRelease(rto, pp, MSG_ERROR, NULL);
      /* We never arrive here. */
    }
#endif
    /*
     * Get the FIFO buffer amount currently available.
     * The TX FIFO is reset to get full capacity.
     */
    uint8_t free = -1;
    if (Si446x_getFIFOfreeTX(radio, 0x01, &free) || free == 0) {
      /* FIFO command failed. */
      TRACE_ERROR("SI   > Radio %d FIFO info command failed for %s TX",
                  radio, getModulation(rto->radio_dat.type));

      /* Unlock radio. */
      pktUnlockRadio(radio);

      Si446x_transmitRelease(rto, pp, MSG_ERROR, NULL);
      /* We never arrive here. */
    }

    /* Calculate initial FIFO fill. */
    uint16_t c = (all > free) ? free : all;

    /* The exit message if all goes well. */
    exit_msg = MSG_OK;

    uint8_t *bufp = layer0;

    /* Initial FIFO load. */
    if (Si446x_writeFIFOdataTX(radio, bufp, c)) {

      /* Something failed so set radio ready to clear it. */
      (void) Si446x_setReadyState(radio);

      /* Radio did not initialise. */
      TRACE_ERROR("SI   > Radio %d write to FIFO failed for %s TX",
                  radio, getModulation(rto->radio_dat.type));

      /* Unlock radio. */
      pktUnlockRadio(radio);

      Si446x_transmitRelease(rto, pp, MSG_ERROR, NULL);
      /* We never arrive here. */
    }

    /* Update buffer index after initial load. */
    bufp += c;

    /* Set FIFO tide level. */
    uint8_t tide = 0;

#define SI446X_2FSK_CCA_TIMEOUT 5
    /* TODO: Timeout to be calculated from speed and data size. */
    /* Request start of transmission. */
    if (Si446x_transmitWithCCA(radio,
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

      /* Set the FIFO threshold to half empty. */
      if (Si446x_setFIFOthresholdTX(radio, free / 2)) {

        /* Something failed so set radio ready to clear it. */
        (void) Si446x_setReadyState(radio);

        /* Unlock radio. */
        pktUnlockRadio(radio);

        TRACE_ERROR("SI   > Radio %d set FIFO threshold failed for %s TX",
                    radio, getModulation(rto->radio_dat.type));

        Si446x_transmitRelease(rto, pp, MSG_ERROR, NULL);
        /* We never arrive here. */
      }
#if Si446x_USE_TRANSMIT_TIMEOUT == TRUE
      /* Start/re-start transmission timeout timer for this packet. */
      chVTSet(&send_timer, TIME_S2I(SI446X_2FSK_TX_TIMEOUT),
              (vtfunc_t)Si446x_transmitTimeoutI, radio);
#endif
      /* Feed the FIFO while data remains to be sent. */
      while ((all - c) > 0) {

        /* Enable TX FIFO interrupt and suspend thread waiting for
             interrupt. */
        msg_t result = Si446x_waitRadioInterrupt(radio,
                                       Si446x_INT_CTL_PH_REG_INDEX,
                                       Si446x_INT_CTL_PH_TX_FIFO_THRESH_MASK,
                                       TIME_S2I(SI446X_2FSK_TX_TIMEOUT / 2));

        if (result != MSG_OK) {

          /* Something failed so cancel interrupt, stop transmit timer and
               set radio ready to clear it. */
#if 0
          if (result == MSG_TIMEOUT)
            (void) Si446x_cancelRadioInterrupt(radio);
#endif
#if Si446x_USE_TRANSMIT_TIMEOUT == TRUE
          chVTReset(&send_timer);
#endif
          (void) Si446x_setReadyState(radio);

          /* Radio did not interrupt. */
          TRACE_ERROR("SI   > Radio %d FIFO interrupt timeout for %s TX",
                      radio, getModulation(rto->radio_dat.type));

          /* Unlock radio. */
          pktUnlockRadio(radio);

          Si446x_transmitRelease(rto, pp, result, NULL);
          /* We never arrive here. */
        }
        /* Get TX FIFO free count. */
        uint8_t more;
        if (Si446x_getFIFOfreeTX(radio, 0x00, &more)) {

          /* Something failed so stop timer and set radio ready to clear it. */
#if Si446x_USE_TRANSMIT_TIMEOUT == TRUE
          chVTReset(&send_timer);
#endif
          (void) Si446x_setReadyState(radio);

          /* Radio did not initialise. */
          TRACE_ERROR("SI   > Radio %d read of FIFO free failed for %s TX",
                      radio, getModulation(rto->radio_dat.type));

          /* Unlock radio. */
          pktUnlockRadio(radio);

          Si446x_transmitRelease(rto, pp, MSG_ERROR, NULL);
          /* We never arrive here. */
        }

        /* Have the amount of free FIFO space now. */
        if (more != 0) {
          /* Update the FIFO free high water mark. */
          tide = (more > tide) ? more : tide;

          /* If there is more free than we need for send use remainder only. */
          more = (more > (all - c)) ? (all - c) : more;

          /* Load the FIFO. */
          if (Si446x_writeFIFOdataTX(radio, bufp, more)) {

            /* Something failed so set radio READY. */
            (void) Si446x_setReadyState(radio);

            /* Unlock radio. */
            pktUnlockRadio(radio);

            TRACE_ERROR("SI   > Radio %d write to FIFO failed for %s TX",
                        radio, getModulation(rto->radio_dat.type));

            Si446x_transmitRelease(rto, pp, MSG_ERROR, NULL);
            /* We never arrive here. */
          }
        }
        /* Update buffer index. */
        bufp += more;
        c += more;
#if Si446x_USE_TRANSMIT_TIMEOUT == TRUE
        /* Check for TX timeout. */
        eventmask_t evt = chEvtGetAndClearEvents(SI446X_EVT_TX_TIMEOUT);
        if (evt) {
          exit_msg = MSG_TIMEOUT;
          break;
        }
#endif
      } /* End while (). */
    }
    else {
      /* Transmit start failed. */
      exit_msg = MSG_ERROR;
    }
#if Si446x_USE_TRANSMIT_TIMEOUT == TRUE
    chVTReset(&send_timer);
#endif
    /* No CCA on subsequent packet sends. */
    rssi = PKT_SI446X_NO_CCA_RSSI;

    /* Get the next linked packet to send. */
    np = pp->nextp;
    if (exit_msg == MSG_OK) {

      /* Send was OK. Release the just completed packet. */
      (void) pktReleaseBufferObject(pp);
    }
    else {
      /* Send failed so release any queue and terminate. */
      pktReleaseBufferChain(pp);
      np = NULL;
      /* Force 446x out of TX state. */
      (void) Si446x_setReadyState(radio);
    }

    /* Tide warning at 75%. */
    if (tide > (free * 2 / 3)) {
      /* Warn when free level is > 75% of FIFO size. */
      TRACE_WARN("SI   > %s TX FIFO free of %i exceeded safe level of %i on radio %d",
                 getModulation(rto->radio_dat.type), tide, (free * 3 / 2), radio);
    }
#if Si446x_USE_PACKET_END_INTERRUPT == TRUE
    /* Enable TX FIFO interrupt and suspend thread waiting for interrupt. */
    msg_t result = Si446x_waitRadioInterrupt(radio,
                                   Si446x_INT_CTL_PH_REG_INDEX,
                                   Si446x_INT_CTL_PH_PACKET_SENT_MASK,
                                   TIME_S2I(SI446X_TX_FIFO_TIMEOUT));

    if (result != MSG_OK) {

      /* Something failed. Cancel interrupt if timeout, stop transmit
         timer and set radio ready to clear it. */
#if 0
      if (result == MSG_TIMEOUT)
        (void) Si446x_cancelRadioInterrupt(radio);
#endif
      (void) Si446x_setReadyState(radio);

      /* Radio did not interrupt. */
      TRACE_ERROR("SI   > Radio %d packet sent interrupt timeout for %s TX",
                  radio, getModulation(rto->radio_dat.type));

      /* Unlock radio. */
      pktUnlockRadio(radio);

      Si446x_transmitRelease(rto, pp, result, NULL);
      /* We never arrive here. */
    }
#endif
  } while ((pp = np) != NULL);  /* Process next packet. */
#if Si446x_USE_PACKET_END_INTERRUPT == FALSE
  /* Wait for transmission to finish. */
  if ((exit_msg = Si446x_waitForState(radio, Si446x_READY,
                                      TIME_S2I(5))) != MSG_OK) {
    /* Force 446x out of TX state. */
    (void) Si446x_setReadyState(radio);
    TRACE_ERROR("SI   > %s transmit failed to cease on radio %d",
                getModulation(rto->radio_dat.type), radio);
  }
#endif
  /* Unlock radio. */
  pktUnlockRadio(radio);

  /* Signal RTM that the TX is complete. */
  Si446x_transmitRelease(rto, pp, exit_msg, "transmit end");
  /* We never arrive here. */
}
#endif /* Si446x_USE_COMMON_TX_THREAD == FALSE */
/*
 * Return true on send successfully enqueued.
 * Task object will be returned
 * Return false on failure
 */
bool Si446x_blocSend2FSK(radio_task_object_t *const rt) {

  thread_t *fsk_feeder_thd = NULL;

  fsk_feeder_thd = chThdCreateFromHeap(NULL,
              THD_WORKING_AREA_SIZE(SI_FSK_FIFO_FEEDER_WA_SIZE),
              "tx_2fsk_queued",
              NORMALPRIO - 10,
#if Si446x_USE_COMMON_TX_THREAD == TRUE
              Si446x_packetTransmitHandler,
#else
              bloc_si_fifo_feeder_fsk,
#endif
              rt);

  if (fsk_feeder_thd == NULL) {
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
  (void) rt;
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
  pktSetGPIOlineMode(*Si446x_getConfig(radio)->rafsk.stream.pwm.pline,
                     Si446x_getConfig(radio)->rafsk.stream.pwm.mode);

  /**
   * Set up GPIO port where the radio line outputting CCA is connected.
   * This input can be read independently to live check CCA (RSSI > threshold).
   * The use case would be for checking CCA prior to transmit.
   *
   * For AFSK receive the MCU GPIO is enabled to interrupt on CCA.
   *
   */
  pktSetGPIOlineMode(*Si446x_getConfig(radio)->rafsk.stream.cca.pline,
                     Si446x_getConfig(radio)->rafsk.stream.cca.mode);

  /* Initialise and start the ICU. */
  ICUDriver *picu = Si446x_getConfig(radio)->rafsk.stream.icu;
  icuObjectInit(picu);
  icuStart(picu, &Si446x_getConfig(radio)->rafsk.stream.cfg);
  return picu;
}

/**
 * Detach the radio ICU. Stops the ICU and set the GPIOs to input state.
 * Any GPIO interrupt/event settings should have been disabled already.
 */
void Si446x_detachPWM(const radio_unit_t radio) {
  icuStop(Si446x_getConfig(radio)->rafsk.stream.icu);

  /* Timer channel GPIO. */
  pktSetGPIOlineMode(*Si446x_getConfig(radio)->rafsk.stream.pwm.pline,
  PAL_MODE_INPUT_PULLUP);

  /* CCA GPIO. */
  pktSetGPIOlineMode(*Si446x_getConfig(radio)->rafsk.stream.cca.pline,
  PAL_MODE_INPUT_PULLUP);
}

/**
 * Enable the GPIO for CCA used in PWM mode of operation.
 * Set a callback and enable event on both edges of CCA.
 * The RX_DATA GPIO is configured by Si446x_attachPWM().
 * Note that AFSK does not use the NIRQ RSSI (CCA) interrupt.
 * Instead the CCA event is routed to a GPIO with its own PAL leading and
 * trailing edge event handler.
 * This is partially due to the fact that the 446x does not have a "lost RSSI"
 * interrupt. So in AFSK mode the loss of RSSI has to be detected by a
 * normal PAL trailing edge event. Using NIRQ would actually add complexity.
 *
 * The mod parameter is not used at the moment since the only modulation
 * requiring the PWM/DSP chain is AFSK. FSK (if implemented) should
 * be processed in the 446x PH (if preamble and sync can be made to work).
 */
void Si446x_enablePWMevents(const radio_unit_t radio, const radio_mod_t mod,
                            const palcallback_t cb) {
  (void) mod;
  /* Set callback for squelch events. */
  palSetLineCallback(*Si446x_getConfig(radio)->rafsk.stream.cca.pline, cb,
                     Si446x_getConfig(radio)->rafsk.stream.icu);

  /* Enabling events on both edges of CCA.*/
  palEnableLineEvent(*Si446x_getConfig(radio)->rafsk.stream.cca.pline,
  PAL_EVENT_MODE_BOTH_EDGES);
}

/**
 * TODO: Switch on mod type (only if there is another mode that uses PWM)
 */
void Si446x_disablePWMeventsI(const radio_unit_t radio, radio_mod_t mod) {
  (void) mod;
  palDisableLineEventI(*Si446x_getConfig(radio)->rafsk.stream.cca.pline);
}

/**
 *
 */
uint8_t Si446x_readCCAlineForRX(const radio_unit_t radio, const radio_mod_t mod) {

  /* Read CCA as setup by mod type. */
  switch (mod) {
  case MOD_2FSK_300:
  case MOD_2FSK_9k6:
  case MOD_2FSK_19k2:
  case MOD_2FSK_38k4:
  case MOD_2FSK_57k6:
  case MOD_2FSK_76k8:
  case MOD_2FSK_96k:
  case MOD_2FSK_115k2:
    return palReadLine(*Si446x_getConfig(radio)->r2fsk.cca.pline);

  case MOD_AFSK: {
    return palReadLine(*Si446x_getConfig(radio)->rafsk.stream.cca.pline);
  }

  case MOD_CW:
    return palReadLine(*Si446x_getConfig(radio)->r2fsk.cca.pline);

  case MOD_NONE:
    break;
  }
  return PAL_LOW;
}

/**
 * @brief Conditionally update XO frequency of radio
 *
 * @param[in] radio Radio unit ID
 * @param[in] freq  XO frequency
 *
 * @return Status of update
 * @retval true if clock change was applied
 * @retval false if the clock update was not applied
 */
bool Si446x_updateClock(const radio_unit_t radio, const xtal_osc_t freq) {
  if (freq == 0)
    return false;
  radio_clock_t xo = Si446x_getData(radio)->radio_clock;
  if ((radio_clock_t)freq != xo) {
    Si446x_getData(radio)->radio_clock = (radio_clock_t)freq;
    /* TODO: The radio is locked when this gets called. If the radio is in
       AFSK receive then it could be forced to use the new TCXO immediately
       by stopping RX and then re-issuing the RX_START command. Otherwise
       the new XO will be used at the next synthesiser set. */
    return true;
  }
  return false;
}

/** @} */
