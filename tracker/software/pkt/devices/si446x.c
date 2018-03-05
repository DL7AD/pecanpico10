/**
 * Si446x driver specialized for APRS transmissions. The driver supports APRS
 * transmission and reception.
 * There can be either used the SLabs Si4463 or Si4464.
 */

//#include "ch.h"
//#include "hal.h"

//#include "si446x.h"
#ifndef PKT_IS_TEST_PROJECT
#include "debug.h"
#endif
#include "pktconf.h"

#ifdef PKT_IS_TEST_PROJECT
void ax25_delete(packet_t pp);
#endif

// Mutex
static mutex_t radio_mtx;               // Radio mutex
static bool nextTransmissionWaiting;    // Flag that informs the feeder thread to keep the radio switched on
static bool radio_mtx_init = false;

// Feeder thread variables
static thread_t* feeder_thd = NULL;
static THD_WORKING_AREA(si_fifo_feeder_wa, 4096);
//static uint8_t *radio_frame;
//static uint8_t *radio_frame_len;
static uint32_t radio_freq;
static uint8_t radio_pwr;

// Si446x variables
static uint32_t outdiv;
static int16_t lastTemp = 0x7FFF;
static bool radioInitialized;

// Receiver thread variables
static uint32_t rx_frequency;
static uint8_t rx_rssi;
static uint8_t rx_chan;
static mod_t rx_mod;
static void (*rx_cb)(uint8_t*, uint32_t);

static packet_svc_t *packetHandler;

static int16_t Si446x_getTemperature(void);

/* =================================================================== SPI communication ==================================================================== */

static const SPIConfig ls_spicfg = {
    .ssport = PAL_PORT(LINE_RADIO_CS),
    .sspad  = PAL_PAD(LINE_RADIO_CS),
    .cr1    = SPI_CR1_MSTR
};

static void Si446x_write(const uint8_t* txData, uint32_t len) {
    // Transmit data by SPI
  /* TODO: Add radio unit ID and get specific radio SPI driver. */
    uint8_t null_spi[len];

    /* Acquire bus and then start SPI. */
    spiAcquireBus(PKT_RADIO_SPI);
    spiStart(PKT_RADIO_SPI, &ls_spicfg);

    /* Poll for CTS. */
    uint8_t rx_ready[] = {Si446x_READ_CMD_BUFF, 0x00};
    do {
      spiSelect(PKT_RADIO_SPI);
      spiExchange(PKT_RADIO_SPI, 1, rx_ready, &rx_ready[1]);
      spiUnselect(PKT_RADIO_SPI);
    } while(rx_ready[1] != Si446x_COMMAND_CTS);

    /* Transfer data. Discard read back. */
    spiSelect(PKT_RADIO_SPI);
    spiExchange(PKT_RADIO_SPI, len, txData, null_spi);
    spiUnselect(PKT_RADIO_SPI);

    /* Stop SPI and relinquish bus. */
    spiStop(PKT_RADIO_SPI);
    spiReleaseBus(PKT_RADIO_SPI);
}

/**
 * Read data from Si446x. First CTS is polled.
 */
static void Si446x_read(const uint8_t* txData, uint32_t txlen, uint8_t* rxData, uint32_t rxlen) {
    // Transmit data by SPI
  /* TODO: Add radio unit ID and get SPI accordingly. */
    uint8_t null_spi[txlen];

    /* Acquire bus and then start SPI. */
    spiAcquireBus(PKT_RADIO_SPI);
    spiStart(PKT_RADIO_SPI, &ls_spicfg);

    /* Poll for CTS. */
    uint8_t rx_ready[] = {Si446x_READ_CMD_BUFF, 0x00};
    do {
      spiSelect(PKT_RADIO_SPI);
      spiExchange(PKT_RADIO_SPI, 1, rx_ready, &rx_ready[1]);
      spiUnselect(PKT_RADIO_SPI);
    } while(rx_ready[1] != Si446x_COMMAND_CTS);

    /* Write data. Discard read back. */
    spiSelect(PKT_RADIO_SPI);
    spiExchange(PKT_RADIO_SPI, txlen, txData, null_spi);

    /* Poll for read data. */
    do {
      spiUnselect(PKT_RADIO_SPI);
      spiSelect(PKT_RADIO_SPI);
      spiExchange(PKT_RADIO_SPI, rxlen, rx_ready, rxData);
    } while(rxData[1] != Si446x_COMMAND_CTS);

    /* Stop SPI and relinquish bus. */
    spiStop(PKT_RADIO_SPI);
    spiReleaseBus(PKT_RADIO_SPI);
}

static void Si446x_setProperty8(uint16_t reg, uint8_t val) {
    uint8_t msg[] = {0x11, (reg >> 8) & 0xFF, 0x01, reg & 0xFF, val};
    Si446x_write(msg, 5);
}

static void Si446x_setProperty16(uint16_t reg, uint8_t val1, uint8_t val2) {
    uint8_t msg[] = {0x11, (reg >> 8) & 0xFF, 0x02, reg & 0xFF, val1, val2};
    Si446x_write(msg, 6);
}

static void Si446x_setProperty24(uint16_t reg, uint8_t val1, uint8_t val2, uint8_t val3) {
    uint8_t msg[] = {0x11, (reg >> 8) & 0xFF, 0x03, reg & 0xFF, val1, val2, val3};
    Si446x_write(msg, 7);
}

static void Si446x_setProperty32(uint16_t reg, uint8_t val1, uint8_t val2, uint8_t val3, uint8_t val4) {
    uint8_t msg[] = {0x11, (reg >> 8) & 0xFF, 0x04, reg & 0xFF, val1, val2, val3, val4};
    Si446x_write(msg, 8);
}

/**
 * Initializes Si446x transceiver chip. Adjusts the frequency which is shifted by variable
 * oscillator voltage.
 * @param mv Oscillator voltage in mv
 */
static void Si446x_init(void) {
#ifdef PKT_IS_TEST_PROJECT
  dbgPrintf(DBG_INFO, "SI   > Init radio\r\n");
#else
  TRACE_INFO("SI   > Init radio");
#endif
  pktConfigureRadioGPIO();

    // Power up (send oscillator type)
    const uint8_t x3 = (Si446x_CCLK >> 24) & 0x0FF;
    const uint8_t x2 = (Si446x_CCLK >> 16) & 0x0FF;
    const uint8_t x1 = (Si446x_CCLK >>  8) & 0x0FF;
    const uint8_t x0 = (Si446x_CCLK >>  0) & 0x0FF;
    const uint8_t init_command[] = {0x02, 0x01, (Si446x_CLK_TCXO_EN & 0x1), x3, x2, x1, x0};
    Si446x_write(init_command, 7);
    chThdSleep(TIME_MS2I(25));

    // Set transceiver GPIOs
    uint8_t gpio_pin_cfg_command[] = {
        0x13,   // Command type = GPIO settings
        0x00,   // GPIO0        GPIO_MODE = DONOTHING
        0x15,   // GPIO1        GPIO_MODE = RAW_RX_DATA
        0x21,   // GPIO2        GPIO_MODE = RX_STATE
        0x20,   // GPIO3        GPIO_MODE = TX_STATE
        0x1B,   // NIRQ         NIRQ_MODE = CCA
        0x0B,   // SDO          SDO_MODE = SDO
        0x00    // GEN_CONFIG
    };
    Si446x_write(gpio_pin_cfg_command, 8);
    chThdSleep(TIME_MS2I(25));

    #if !Si446x_CLK_TCXO_EN
    Si446x_setProperty8(Si446x_GLOBAL_XO_TUNE, 0x00);
    #endif

    Si446x_setProperty8(Si446x_FRR_CTL_A_MODE, 0x00);
    Si446x_setProperty8(Si446x_FRR_CTL_B_MODE, 0x00);
    Si446x_setProperty8(Si446x_FRR_CTL_C_MODE, 0x00);
    Si446x_setProperty8(Si446x_FRR_CTL_D_MODE, 0x00);
    Si446x_setProperty8(Si446x_INT_CTL_ENABLE, 0x00);
    /* Set combined FIFO mode = 0x70. */
    //Si446x_setProperty8(Si446x_GLOBAL_CONFIG, 0x60);
    Si446x_setProperty8(Si446x_GLOBAL_CONFIG, 0x70);

    // Reset FIFO
    const uint8_t reset_fifo[] = {0x15, 0x01};
    Si446x_write(reset_fifo, 2);
    /* No need to do this unreset... see si docs. */
    //const uint8_t unreset_fifo[] = {0x15, 0x00};
    //Si446x_write(unreset_fifo, 2);

    /*
     * TODO: Move the TX and RX settings out into the respective functions.
     * This would split up into AFSK and FSK for RX & TX.
     * Leave only common setup and init in here for selected base band frequency.
     */
    Si446x_setProperty8(Si446x_PREAMBLE_TX_LENGTH, 0x00);
    /* TODO: Use PREAMBLE_CONFIG_NSTD, etc. to send flags?
     * Unfortunately this requires a preamble pattern of 88 bits.
     * The 446x only has up to 32 pattern bits.
     * Why 88 bits? Due to the oversampling used to create AFSK at 13.2ksps.
     * Each HDLC bit requires 11 TX bit times.
     *
     * The alternative is to use TX_FIELDS.
     * Send preamble (HDLC flags) using FIELD_1 in a loop with fixed data 0x7E.
     * Field length can be 4096 bytes so up to 372 flags could be sent.
     * The flag bit stream uses 11 bytes per flag.
     * Using 200 flags would be 11 * 200 = 2200 bytes (17,600 stream bits).
     * Set FIELD_1 as 2,200 bytes and feed 200 x the bit pattern to the FIFO.
     * The transition to FIELD_2 is handled in the 446x packet handler.
     * Then FIELD_2 FIFO data is fed from the layer0 (bit stream) data buffer.
     */
    Si446x_setProperty8(Si446x_SYNC_CONFIG, 0x80);

    Si446x_setProperty8(Si446x_GLOBAL_CLK_CFG, 0x00);
    Si446x_setProperty8(Si446x_MODEM_RSSI_CONTROL, 0x00);
    /* TODO: Don't need this setting? */
    Si446x_setProperty8(Si446x_PREAMBLE_CONFIG_STD_1, 0x14);
    Si446x_setProperty8(Si446x_PKT_CONFIG1, 0x41);
    Si446x_setProperty8(Si446x_MODEM_MAP_CONTROL, 0x00);
    Si446x_setProperty8(Si446x_MODEM_DSM_CTRL, 0x07);
    Si446x_setProperty8(Si446x_MODEM_CLKGEN_BAND, 0x0D);

    Si446x_setProperty24(Si446x_MODEM_FREQ_DEV, 0x00, 0x00, 0x79);
    Si446x_setProperty8(Si446x_MODEM_TX_RAMP_DELAY, 0x01);
    Si446x_setProperty8(Si446x_PA_TC, 0x3D);
    Si446x_setProperty8(Si446x_FREQ_CONTROL_INTE, 0x41);
    Si446x_setProperty24(Si446x_FREQ_CONTROL_FRAC, 0x0B, 0xB1, 0x3B);
    Si446x_setProperty16(Si446x_FREQ_CONTROL_CHANNEL_STEP_SIZE, 0x0B, 0xD1);
    Si446x_setProperty8(Si446x_FREQ_CONTROL_W_SIZE, 0x20);
    Si446x_setProperty8(Si446x_FREQ_CONTROL_VCOCNT_RX_ADJ, 0xFA);
    Si446x_setProperty8(Si446x_MODEM_MDM_CTRL, 0x80);
    Si446x_setProperty8(Si446x_MODEM_IF_CONTROL, 0x08);
    Si446x_setProperty24(Si446x_MODEM_IF_FREQ, 0x02, 0x80, 0x00);
    Si446x_setProperty8(Si446x_MODEM_DECIMATION_CFG1, 0x70);
    Si446x_setProperty8(Si446x_MODEM_DECIMATION_CFG0, 0x10);
    Si446x_setProperty16(Si446x_MODEM_BCR_OSR, 0x01, 0xC3);
    Si446x_setProperty24(Si446x_MODEM_BCR_NCO_OFFSET, 0x01, 0x22, 0x60);
    Si446x_setProperty16(Si446x_MODEM_BCR_GAIN, 0x00, 0x91);
    Si446x_setProperty8(Si446x_MODEM_BCR_GEAR, 0x00);
    Si446x_setProperty8(Si446x_MODEM_BCR_MISC1, 0xC2);
    Si446x_setProperty8(Si446x_MODEM_AFC_GEAR, 0x54);
    Si446x_setProperty8(Si446x_MODEM_AFC_WAIT, 0x36);
    Si446x_setProperty16(Si446x_MODEM_AFC_GAIN, 0x80, 0xAB);
    Si446x_setProperty16(Si446x_MODEM_AFC_LIMITER, 0x02, 0x50);
    Si446x_setProperty8(Si446x_MODEM_AFC_MISC, 0x80);
    Si446x_setProperty8(Si446x_MODEM_AGC_CONTROL, 0xE2);
    Si446x_setProperty8(Si446x_MODEM_AGC_WINDOW_SIZE, 0x11);
    Si446x_setProperty8(Si446x_MODEM_AGC_RFPD_DECAY, 0x63);
    Si446x_setProperty8(Si446x_MODEM_AGC_IFPD_DECAY, 0x63);
    Si446x_setProperty8(Si446x_MODEM_FSK4_GAIN1, 0x00);
    Si446x_setProperty8(Si446x_MODEM_FSK4_GAIN0, 0x02);
    Si446x_setProperty16(Si446x_MODEM_FSK4_TH, 0x35, 0x55);
    Si446x_setProperty8(Si446x_MODEM_FSK4_MAP, 0x00);
    Si446x_setProperty8(Si446x_MODEM_OOK_PDTC, 0x2A);
    Si446x_setProperty8(Si446x_MODEM_OOK_CNT1, 0x85);
    Si446x_setProperty8(Si446x_MODEM_OOK_MISC, 0x23);
    Si446x_setProperty8(Si446x_MODEM_RAW_SEARCH, 0xD6);
    Si446x_setProperty8(Si446x_MODEM_RAW_CONTROL, 0x8F);
    Si446x_setProperty16(Si446x_MODEM_RAW_EYE, 0x00, 0x3B);
    Si446x_setProperty8(Si446x_MODEM_ANT_DIV_MODE, 0x01);
    Si446x_setProperty8(Si446x_MODEM_ANT_DIV_CONTROL, 0x80);
    Si446x_setProperty8(Si446x_MODEM_RSSI_COMP, 0x40);

    // Temperature readout
    lastTemp = Si446x_getTemperature();
#ifdef PKT_IS_TEST_PROJECT
    dbgPrintf(DBG_INFO, "SI   > Transmitter temperature %d degC\r\n", lastTemp/100);
#else
    TRACE_INFO("SI   > Transmitter temperature %d degC\r\n", lastTemp/100);
#endif

    radioInitialized = true;
}

void Si446x_conditional_init() {
// Initialize radio
if(!radioInitialized)
    Si446x_init();
}

/*void init144_800(void) {
    // Configure Radio pins
    palSetLineMode(LINE_SPI_SCK, PAL_MODE_ALTERNATE(6) | PAL_STM32_OSPEED_HIGHEST);     // SCK
    palSetLineMode(LINE_SPI_MISO, PAL_MODE_ALTERNATE(6) | PAL_STM32_OSPEED_HIGHEST);    // MISO
    palSetLineMode(LINE_SPI_MOSI, PAL_MODE_ALTERNATE(6) | PAL_STM32_OSPEED_HIGHEST);    // MOSI
    palSetLineMode(LINE_RADIO_CS, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST); // RADIO CS
    palSetLineMode(LINE_RADIO_SDN, PAL_MODE_OUTPUT_PUSHPULL);                           // RADIO SDN

    // Pull CS of all SPI slaves high
    palSetLine(LINE_RADIO_CS);

    // Reset radio
    palSetLine(LINE_RADIO_SDN);
    chThdSleep(TIME_MS2I(10));

    // Power up transmitter
    palClearLine(LINE_RADIO_SDN);   // Radio SDN low (power up transmitter)
    chThdSleep(TIME_MS2I(10));      // Wait for transmitter to power up

    // Power up (transmits oscillator type)
    const uint8_t x3 = (RADIO_CLK >> 24) & 0x0FF;
    const uint8_t x2 = (RADIO_CLK >> 16) & 0x0FF;
    const uint8_t x1 = (RADIO_CLK >>  8) & 0x0FF;
    const uint8_t x0 = (RADIO_CLK >>  0) & 0x0FF;
    const uint8_t init_command[] = {0x02, 0x01, (RADIO_TCXO_EN & 0x1), x3, x2, x1, x0};
    Si446x_write(init_command, 7);
    chThdSleep(TIME_MS2I(25));

    // Set transmitter GPIOs
    uint8_t gpio_pin_cfg_command[] = {
        0x13,   // Command type = GPIO settings
        0x00,   // GPIO0        GPIO_MODE = DONOTHING
        0x14,   // GPIO1        GPIO_MODE = RX_DATA
        0x21,   // GPIO2        GPIO_MODE = RX_STATE
        0x20,   // GPIO3        GPIO_MODE = TX_STATE
        0x1B,   // NIRQ         NIRQ_MODE = CCA
        0x0B,   // SDO          SDO_MODE = SDO
        0x00    // GEN_CONFIG
    };
    Si446x_write(gpio_pin_cfg_command, 8);
    chThdSleep(TIME_MS2I(25));







    #if !RADIO_TCXO_EN
    Si446x_setProperty8(Si446x_GLOBAL_XO_TUNE, 0x00);
    #endif

    Si446x_setProperty8(Si446x_FRR_CTL_A_MODE, 0x00);
    Si446x_setProperty8(Si446x_FRR_CTL_B_MODE, 0x00);
    Si446x_setProperty8(Si446x_FRR_CTL_C_MODE, 0x00);
    Si446x_setProperty8(Si446x_FRR_CTL_D_MODE, 0x00);
    Si446x_setProperty8(Si446x_INT_CTL_ENABLE, 0x00);
    Si446x_setProperty8(Si446x_GLOBAL_CONFIG, 0x60);
    Si446x_setProperty8(Si446x_GLOBAL_CLK_CFG, 0x00);
    Si446x_setProperty8(Si446x_MODEM_RSSI_CONTROL, 0x00);
    Si446x_setProperty8(Si446x_MODEM_RSSI_THRESH, 0x5F);
    Si446x_setProperty8(Si446x_PREAMBLE_CONFIG_STD_1, 0x14);
    Si446x_setProperty8(Si446x_PKT_CONFIG1, 0x40);
    Si446x_setProperty8(Si446x_MODEM_MOD_TYPE, 0x0A);
    Si446x_setProperty8(Si446x_MODEM_MAP_CONTROL, 0x00);
    Si446x_setProperty8(Si446x_MODEM_DSM_CTRL, 0x07);
    Si446x_setProperty8(Si446x_MODEM_CLKGEN_BAND, 0x0D);

    //Si446x_setProperty8(Si446x_SYNTH_PFDCP_CPFF, 0x2C);
    //Si446x_setProperty8(Si446x_SYNTH_PFDCP_CPINT, 0x0E);
    //Si446x_setProperty8(Si446x_SYNTH_VCO_KV, 0x0B);
    //Si446x_setProperty8(Si446x_SYNTH_LPFILT3, 0x04);
    //Si446x_setProperty8(Si446x_SYNTH_LPFILT2, 0x0C);
    //Si446x_setProperty8(Si446x_SYNTH_LPFILT1, 0x73);
    //Si446x_setProperty8(Si446x_SYNTH_LPFILT0, 0x03);

    Si446x_setProperty24(Si446x_MODEM_DATA_RATE, 0x04, 0x07, 0x40);
    Si446x_setProperty32(Si446x_MODEM_TX_NCO_MODE, 0x01, 0x8C, 0xBA, 0x80);
    Si446x_setProperty24(Si446x_MODEM_FREQ_DEV, 0x00, 0x00, 0x79);
    Si446x_setProperty8(Si446x_MODEM_TX_RAMP_DELAY, 0x01);
    Si446x_setProperty8(Si446x_PA_TC, 0x3D);
    Si446x_setProperty8(Si446x_FREQ_CONTROL_INTE, 0x41);
    Si446x_setProperty24(Si446x_FREQ_CONTROL_FRAC, 0x0B, 0xB1, 0x3B);
    Si446x_setProperty16(Si446x_FREQ_CONTROL_CHANNEL_STEP_SIZE, 0x0B, 0xD1);
    Si446x_setProperty8(Si446x_FREQ_CONTROL_W_SIZE, 0x20);
    Si446x_setProperty8(Si446x_FREQ_CONTROL_VCOCNT_RX_ADJ, 0xFA);
    Si446x_setProperty8(Si446x_MODEM_MDM_CTRL, 0x80);
    Si446x_setProperty8(Si446x_MODEM_IF_CONTROL, 0x08);
    Si446x_setProperty24(Si446x_MODEM_IF_FREQ, 0x02, 0x80, 0x00);
    Si446x_setProperty8(Si446x_MODEM_DECIMATION_CFG1, 0x20);
    Si446x_setProperty8(Si446x_MODEM_DECIMATION_CFG0, 0x10);
    Si446x_setProperty16(Si446x_MODEM_BCR_OSR, 0x00, 0x52);
    Si446x_setProperty24(Si446x_MODEM_BCR_NCO_OFFSET, 0x06, 0x3D, 0x10);
    Si446x_setProperty16(Si446x_MODEM_BCR_GAIN, 0x03, 0x1F);
    Si446x_setProperty8(Si446x_MODEM_BCR_GEAR, 0x00);
    Si446x_setProperty8(Si446x_MODEM_BCR_MISC1, 0xC2);
    Si446x_setProperty8(Si446x_MODEM_AFC_GEAR, 0x54);
    Si446x_setProperty8(Si446x_MODEM_AFC_WAIT, 0x36);
    Si446x_setProperty16(Si446x_MODEM_AFC_GAIN, 0x82, 0xAA);
    Si446x_setProperty16(Si446x_MODEM_AFC_LIMITER, 0x00, 0x95);
    Si446x_setProperty8(Si446x_MODEM_AFC_MISC, 0x80);
    Si446x_setProperty8(Si446x_MODEM_AGC_CONTROL, 0xE2);
    Si446x_setProperty8(Si446x_MODEM_AGC_WINDOW_SIZE, 0x11);
    Si446x_setProperty8(Si446x_MODEM_AGC_RFPD_DECAY, 0x12);
    Si446x_setProperty8(Si446x_MODEM_AGC_IFPD_DECAY, 0x12);
    Si446x_setProperty8(Si446x_MODEM_FSK4_GAIN1, 0x00);
    Si446x_setProperty8(Si446x_MODEM_FSK4_GAIN0, 0x02);
    Si446x_setProperty16(Si446x_MODEM_FSK4_TH, 0x02, 0x6D);
    Si446x_setProperty8(Si446x_MODEM_FSK4_MAP, 0x00);
    Si446x_setProperty8(Si446x_MODEM_OOK_PDTC, 0x28);
    Si446x_setProperty8(Si446x_MODEM_OOK_CNT1, 0x85);
    Si446x_setProperty8(Si446x_MODEM_OOK_MISC, 0x23);
    Si446x_setProperty8(Si446x_MODEM_RAW_SEARCH, 0xDE);
    Si446x_setProperty8(Si446x_MODEM_RAW_CONTROL, 0x8F);
    Si446x_setProperty16(Si446x_MODEM_RAW_EYE, 0x00, 0x0F);
    Si446x_setProperty8(Si446x_MODEM_ANT_DIV_MODE, 0x01);
    Si446x_setProperty8(Si446x_MODEM_ANT_DIV_CONTROL, 0x80);
    Si446x_setProperty8(Si446x_MODEM_RSSI_COMP, 0x40);

    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX1_CHFLT_COE13_7_0, 0xA2);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX1_CHFLT_COE12_7_0, 0xA0);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX1_CHFLT_COE11_7_0, 0x97);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX1_CHFLT_COE10_7_0, 0x8A);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX1_CHFLT_COE9_7_0, 0x79);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX1_CHFLT_COE8_7_0, 0x66);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX1_CHFLT_COE7_7_0, 0x52);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX1_CHFLT_COE6_7_0, 0x3F);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX1_CHFLT_COE5_7_0, 0x2E);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX1_CHFLT_COE4_7_0, 0x1F);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX1_CHFLT_COE3_7_0, 0x14);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX1_CHFLT_COE2_7_0, 0x0B);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX1_CHFLT_COE1_7_0, 0x06);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX1_CHFLT_COE0_7_0, 0x02);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX1_CHFLT_COEM0, 0x00);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX1_CHFLT_COEM1, 0x00);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX1_CHFLT_COEM2, 0x00);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX1_CHFLT_COEM3, 0x00);

    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX2_CHFLT_COE13_7_0, 0xA2);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX2_CHFLT_COE12_7_0, 0xA0);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX2_CHFLT_COE11_7_0, 0x97);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX2_CHFLT_COE10_7_0, 0x8A);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX2_CHFLT_COE9_7_0, 0x79);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX2_CHFLT_COE8_7_0, 0x66);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX2_CHFLT_COE7_7_0, 0x52);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX2_CHFLT_COE6_7_0, 0x3F);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX2_CHFLT_COE5_7_0, 0x2E);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX2_CHFLT_COE4_7_0, 0x1F);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX2_CHFLT_COE3_7_0, 0x14);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX2_CHFLT_COE2_7_0, 0x0B);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX2_CHFLT_COE1_7_0, 0x06);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX2_CHFLT_COE0_7_0, 0x02);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX2_CHFLT_COEM0, 0x00);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX2_CHFLT_COEM1, 0x00);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX2_CHFLT_COEM2, 0x00);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX2_CHFLT_COEM3, 0x00);
}*/

static void Si446x_setFrequency(uint32_t freq)
{
    uint16_t shift = 0;

    // Set the output divider according to recommended ranges given in Si446x datasheet
    uint32_t band = 0;
    if(freq < 705000000UL) {outdiv = 6;  band = 1;};
    if(freq < 525000000UL) {outdiv = 8;  band = 2;};
    if(freq < 353000000UL) {outdiv = 12; band = 3;};
    if(freq < 239000000UL) {outdiv = 16; band = 4;};
    if(freq < 177000000UL) {outdiv = 24; band = 5;};

    // Set the band parameter
    uint32_t sy_sel = 8;
    uint8_t set_band_property_command[] = {0x11, 0x20, 0x01, 0x51, (band + sy_sel)};
    Si446x_write(set_band_property_command, 5);

    // Set the PLL parameters
    uint32_t f_pfd = 2 * Si446x_CCLK / outdiv;
    uint32_t n = ((uint32_t)(freq / f_pfd)) - 1;
    float ratio = (float)freq / (float)f_pfd;
    float rest  = ratio - (float)n;

    uint32_t m = (uint32_t)(rest * 524288UL);
    uint32_t m2 = m >> 16;
    uint32_t m1 = (m - m2 * 0x10000) >> 8;
    uint32_t m0 = (m - m2 * 0x10000 - (m1 << 8));

    uint32_t channel_increment = 524288 * outdiv * shift / (2 * Si446x_CCLK);
    uint8_t c1 = channel_increment / 0x100;
    uint8_t c0 = channel_increment - (0x100 * c1);

    uint8_t set_frequency_property_command[] = {0x11, 0x40, 0x04, 0x00, n, m2, m1, m0, c1, c0};
    Si446x_write(set_frequency_property_command, 10);

    uint32_t x = ((((uint32_t)1 << 19) * outdiv * 1300.0)/(2*Si446x_CCLK))*2;
    uint8_t x2 = (x >> 16) & 0xFF;
    uint8_t x1 = (x >>  8) & 0xFF;
    uint8_t x0 = (x >>  0) & 0xFF;
    uint8_t set_deviation[] = {0x11, 0x20, 0x03, 0x0a, x2, x1, x0};
    Si446x_write(set_deviation, 7);
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

static void Si446x_setPowerLevel(int8_t level)
{
    // Set the Power
    uint8_t set_pa_pwr_lvl_property_command[] = {0x11, 0x22, 0x01, 0x01, level};
    Si446x_write(set_pa_pwr_lvl_property_command, 5);
}



/* =========================================================== Radio specific modulation settings =========================================================== */

static void Si446x_setModemAFSK_TX(void)
{
    // Setup the NCO modulo and oversampling mode
    uint32_t s = Si446x_CCLK / 10;
    uint8_t f3 = (s >> 24) & 0xFF;
    uint8_t f2 = (s >> 16) & 0xFF;
    uint8_t f1 = (s >>  8) & 0xFF;
    uint8_t f0 = (s >>  0) & 0xFF;
    Si446x_setProperty32(Si446x_MODEM_TX_NCO_MODE, f3, f2, f1, f0);

    // Setup the NCO data rate for APRS
    Si446x_setProperty24(Si446x_MODEM_DATA_RATE, 0x00, 0x33, 0x90);

    // Use 2FSK from FIFO (PH)
    Si446x_setProperty8(Si446x_MODEM_MOD_TYPE, 0x02);

    // Set AFSK filter
    const uint8_t coeff[] = {0x81, 0x9f, 0xc4, 0xee, 0x18, 0x3e, 0x5c, 0x70, 0x76};
    uint8_t i;
    for(i=0; i<sizeof(coeff); i++) {
        uint8_t msg[] = {0x11, 0x20, 0x01, 0x17-i, coeff[i]};
        Si446x_write(msg, 5);
    }
}

static void Si446x_setModemAFSK_RX(void)
{
    // Setup the NCO modulo and oversampling mode
/*    uint32_t s = Si446x_CCLK;
    uint8_t f3 = (s >> 24) & 0xFF;
    uint8_t f2 = (s >> 16) & 0xFF;
    uint8_t f1 = (s >>  8) & 0xFF;
    uint8_t f0 = (s >>  0) & 0xFF;
    Si446x_setProperty32(Si446x_MODEM_TX_NCO_MODE, f3, f2, f1, f0);*/

    // Setup the NCO data rate for APRS
    //Si446x_setProperty24(Si446x_MODEM_DATA_RATE, 0x04, 0x07, 0x40);

    // Use 2FSK in DIRECT_MODE
    Si446x_setProperty8(Si446x_MODEM_MOD_TYPE, 0x0A);

    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX1_CHFLT_COE13_7_0, 0xFF);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX1_CHFLT_COE12_7_0, 0xC4);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX1_CHFLT_COE11_7_0, 0x30);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX1_CHFLT_COE10_7_0, 0x7F);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX1_CHFLT_COE9_7_0, 0x5F);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX1_CHFLT_COE8_7_0, 0xB5);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX1_CHFLT_COE7_7_0, 0xB8);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX1_CHFLT_COE6_7_0, 0xDE);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX1_CHFLT_COE5_7_0, 0x05);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX1_CHFLT_COE4_7_0, 0x17);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX1_CHFLT_COE3_7_0, 0x16);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX1_CHFLT_COE2_7_0, 0x0C);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX1_CHFLT_COE1_7_0, 0x03);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX1_CHFLT_COE0_7_0, 0x00);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX1_CHFLT_COEM0, 0x15);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX1_CHFLT_COEM1, 0xFF);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX1_CHFLT_COEM2, 0x00);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX1_CHFLT_COEM3, 0x00);

/*    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX2_CHFLT_COE13_7_0, 0xFF);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX2_CHFLT_COE12_7_0, 0xC4);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX2_CHFLT_COE11_7_0, 0x30);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX2_CHFLT_COE10_7_0, 0x7F);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX2_CHFLT_COE9_7_0, 0xF5);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX2_CHFLT_COE8_7_0, 0xB5);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX2_CHFLT_COE7_7_0, 0xB8);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX2_CHFLT_COE6_7_0, 0xDE);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX2_CHFLT_COE5_7_0, 0x05);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX2_CHFLT_COE4_7_0, 0x17);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX2_CHFLT_COE3_7_0, 0x16);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX2_CHFLT_COE2_7_0, 0x0C);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX2_CHFLT_COE1_7_0, 0x03);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX2_CHFLT_COE0_7_0, 0x00);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX2_CHFLT_COEM0, 0x15);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX2_CHFLT_COEM1, 0xFF);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX2_CHFLT_COEM2, 0x00);
    Si446x_setProperty8(Si446x_MODEM_CHFLT_RX2_CHFLT_COEM3, 0x00);*/
}

static void Si446x_setModem2FSK(uint32_t speed)
{
    // Setup the NCO modulo and oversampling mode
    uint32_t s = Si446x_CCLK / 10;
    uint8_t f3 = (s >> 24) & 0xFF;
    uint8_t f2 = (s >> 16) & 0xFF;
    uint8_t f1 = (s >>  8) & 0xFF;
    uint8_t f0 = (s >>  0) & 0xFF;
    Si446x_setProperty32(Si446x_MODEM_TX_NCO_MODE, f3, f2, f1, f0);

    // Setup the NCO data rate for 2FSK
    Si446x_setProperty24(Si446x_MODEM_DATA_RATE, (uint8_t)(speed >> 16), (uint8_t)(speed >> 8), (uint8_t)speed);

    // Use 2FSK from FIFO (PH)
    Si446x_setProperty8(Si446x_MODEM_MOD_TYPE, 0x03);
}

/* ======================================================================= Radio FIFO ======================================================================= */

static void Si446x_writeFIFO(uint8_t *msg, uint8_t size) {
    uint8_t write_fifo[size+1];
    write_fifo[0] = 0x66;
    memcpy(&write_fifo[1], msg, size);
    Si446x_write(write_fifo, size+1);
}

static uint8_t Si446x_freeFIFO(void) {
    const uint8_t fifo_info[2] = {0x15, 0x00};
    uint8_t rxData[4];
    Si446x_read(fifo_info, 2, rxData, 4);
    return rxData[3];
}

/* ====================================================================== Radio States ====================================================================== */

static uint8_t Si446x_getState(void)
{
    const uint8_t state_info[] = {0x33};
    uint8_t rxData[3];
    Si446x_read(state_info, sizeof(state_info), rxData, 3);
    return rxData[2] & 0xF;
}

static void Si446x_setTXState(uint16_t size)
{
    uint8_t change_state_command[] = {0x31, 0x00, (Si446x_STATE_READY << 4), (size >> 8) & 0x1F, size & 0xFF};
    Si446x_write(change_state_command, 5);
}

static void Si446x_setReadyState(void)
{
    const uint8_t change_state_command[] = {0x34, 0x03};
    Si446x_write(change_state_command, 2);
}

static void Si446x_setRXState(void)
{
    const uint8_t change_state_command[] = {0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x08};
    Si446x_write(change_state_command, 8);
}

static void Si446x_shutdown(void)
{
    // Wait for PH to finish transmission
    while(Si446x_getState() == Si446x_STATE_TX)
        chThdSleep(TIME_MS2I(1));

    if(!nextTransmissionWaiting) { // No thread is waiting for radio, so shutdown radio
#ifdef PKT_IS_TEST_PROJECT
      dbgPrintf(DBG_INFO, "SI   > Shutdown radio");
#else
        TRACE_INFO("SI   > Shutdown radio");
#endif
      pktDeconfigureRadioGPIO();

        radioInitialized = false;

    } else {
#ifdef PKT_IS_TEST_PROJECT
        dbgPrintf(DBG_INFO, "RAD  > Transmission finished");
        dbgPrintf(DBG_INFO, "RAD  > Keep radio switched on");
#else
        TRACE_INFO("RAD  > Transmission finished");
        TRACE_INFO("RAD  > Keep radio switched on");
#endif
    }
}

/* ======================================================================== Locking ========================================================================= */

static void lockRadio(void)
{
    // Initialize mutex
    if(!radio_mtx_init)
        chMtxObjectInit(&radio_mtx);
    radio_mtx_init = true;

    chMtxLock(&radio_mtx);
    nextTransmissionWaiting = true;

    // Wait for old feeder thread to terminate
    if(feeder_thd != NULL) // No waiting on first use
        chThdWait(feeder_thd);
}

void unlockRadio(void)
{
    nextTransmissionWaiting = false;
    chMtxUnlock(&radio_mtx);
}

void lockRadioByCamera(void)
{
    // Initialize mutex
    if(!radio_mtx_init)
        chMtxObjectInit(&radio_mtx);
    radio_mtx_init = true;

    chMtxLock(&radio_mtx);

    // Wait for old feeder thread to terminate
    if(feeder_thd != NULL) // No waiting on first use
        chThdWait(feeder_thd);
}

/* ====================================================================== Radio TX/RX ======================================================================= */

static bool Si446x_getLatchedCCA(uint8_t ms)
{
    uint16_t cca = 0;
    for(uint16_t i=0; i<ms*10; i++) {
        cca += Si446x_getCCA();
        chThdSleep(TIME_US2I(100));
    }
#ifdef PKT_IS_TEST_PROJECT
    dbgPrintf(DBG_INFO, "SI   > CCA=%03d RX=%d\r\n", cca, cca > ms/10);
#else
    TRACE_INFO("SI   > CCA=%03d RX=%d", cca, cca > ms/10);
#endif
    return cca > ms; // Max. 1 spike per ms
}

static bool Si446x_transmit(uint32_t frequency, int8_t power, uint16_t size, uint8_t rssi, sysinterval_t sql_timeout)
{
    if(!Si446x_inRadioBand(frequency)) {
#ifdef PKT_IS_TEST_PROJECT
      dbgPrintf(DBG_ERROR, "SI   > Frequency out of range\r\n");
      dbgPrintf(DBG_ERROR, "SI   > abort transmission\r\n");
#else
      TRACE_ERROR("SI   > Frequency out of range");
      TRACE_ERROR("SI   > abort transmission");
#endif
        return false;
    }

    // Switch to ready state
    if(Si446x_getState() == Si446x_STATE_RX) {
#ifdef PKT_IS_TEST_PROJECT
      dbgPrintf(DBG_INFO, "SI   > Switch Si446x to ready state\r\n");
#else
      TRACE_INFO("SI   > Switch Si446x to ready state");
#endif
        Si446x_setReadyState();
    }

    Si446x_setProperty8(Si446x_MODEM_RSSI_THRESH, rssi);
    Si446x_setFrequency(frequency);     // Set frequency
    Si446x_setRXState();

    // Wait until nobody is transmitting (until timeout)

    if(Si446x_getState() != Si446x_STATE_RX || Si446x_getLatchedCCA(50)) {
#ifdef PKT_IS_TEST_PROJECT
        dbgPrintf(DBG_INFO, "SI   > Wait for CCA to drop\r\n");
#else
        TRACE_INFO("SI   > Wait for CCA to drop");
#endif
        sysinterval_t t0 = chVTGetSystemTime();
        while((Si446x_getState() != Si446x_STATE_RX || Si446x_getLatchedCCA(50)) && chVTGetSystemTime()-t0 < sql_timeout)
            chThdSleep(TIME_US2I(100));
    }

    // Transmit
#ifdef PKT_IS_TEST_PROJECT
    dbgPrintf(DBG_INFO, "SI   > Tune Si446x (TX)\r\n");
#else
    TRACE_INFO("SI   > Tune Si446x (TX)");
#endif
    Si446x_setPowerLevel(power);        // Set power level
    Si446x_setReadyState();
    Si446x_setTXState(size);

    return true;
}

/*static*/ bool Si446x_receive_noLock(uint32_t frequency,
                                      uint8_t channel,
                                      uint8_t rssi,
                                      mod_t mod) {
    if(!Si446x_inRadioBand(frequency)) {
#ifdef PKT_IS_TEST_PROJECT
      dbgPrintf(DBG_ERROR, "SI   > Frequency out of range\r\n");
      dbgPrintf(DBG_ERROR, "SI   > abort reception\r\n");
#else
      TRACE_ERROR("SI   > Frequency out of range");
      TRACE_ERROR("SI   > abort reception");
#endif


      return false;
    }

    // Initialize radio
    if(!radioInitialized)
        Si446x_init();

    // Wait until transceiver finishes transmission (if there is any)
    while(Si446x_getState() == Si446x_STATE_TX) {
        chThdSleep(TIME_MS2I(5));
    }

    // Initialize radio
    if(mod == MOD_AFSK) {
        Si446x_setModemAFSK_RX();
    } else {
        Si446x_shutdown();
#ifdef PKT_IS_TEST_PROJECT
        dbgPrintf(DBG_ERROR, "SI   > Unknown modulation\r\n");
        dbgPrintf(DBG_ERROR, "SI   > abort reception\r\n");
#else
        TRACE_ERROR("SI   > Modulation not supported");
        TRACE_ERROR("SI   > abort reception");
#endif
        return false;
    }

    // Preserve settings in case transceiver changes to TX state
    rx_frequency = frequency;
    rx_rssi = rssi;
    rx_chan = channel;
    rx_mod = mod;

#ifdef PKT_IS_TEST_PROJECT
    dbgPrintf(DBG_INFO, "SI   > Tune Si446x (RX)\r\n");
#else
    TRACE_INFO("SI   > Tune Si446x (RX)");
#endif
    Si446x_setProperty8(Si446x_MODEM_RSSI_THRESH, rssi);
    Si446x_setFrequency(frequency);     // Set frequency
    Si446x_setRXState();

    // Wait for the receiver to start (because it is used as mutex)
    while(Si446x_getState() != Si446x_STATE_RX)
        chThdSleep(TIME_MS2I(1));
    return true;
}

bool Si446x_receive(uint32_t frequency, uint8_t rssi, uint8_t chan, mod_t mod)
{
    lockRadio();
    bool ret = Si446x_receive_noLock(frequency, chan, rssi, mod);
    unlockRadio();

    return ret;
}

static bool Si4464_restoreRX(void)
{
    bool ret = Si446x_receive_noLock(rx_frequency, rx_chan, rx_rssi, rx_mod);

    if(packetHandler) {
#ifdef PKT_IS_TEST_PROJECT
      dbgPrintf(DBG_INFO, "SI   > Resume packet reception\r\n");
#else
        TRACE_INFO("SI   > Resume packet reception");
#endif

        /* Resume decoding. */
        pktResumeDecoder(packetHandler);
    }

    return ret;
}

void Si446x_receive_stop(void)
{
    rx_frequency = 0;

    if(Si446x_getState() == Si446x_STATE_RX)
        Si446x_shutdown();
}

/* ==================================================================== AFSK Transmitter ==================================================================== */

#define PLAYBACK_RATE       13200
#define BAUD_RATE           1200                                    /* APRS AFSK baudrate */
#define SAMPLES_PER_BAUD    (PLAYBACK_RATE / BAUD_RATE)             /* Samples per baud (192kHz / 1200baud = 160samp/baud) */
#define PHASE_DELTA_1200    (((2 * 1200) << 16) / PLAYBACK_RATE)    /* Delta-phase per sample for 1200Hz tone */
#define PHASE_DELTA_2200    (((2 * 2200) << 16) / PLAYBACK_RATE)    /* Delta-phase per sample for 2200Hz tone */

static uint32_t phase_delta;            // 1200/2200 for standard AX.25
static uint32_t phase;                  // Fixed point 9.7 (2PI = TABLE_SIZE)
static uint32_t packet_pos;             // Next bit to be sent out
static uint32_t current_sample_in_baud; // 1 bit = SAMPLES_PER_BAUD samples
static uint8_t current_byte;
static uint8_t ctone = 0;

static bool encode_nrzi(bool bit)
{
    if((bit & 0x1) == 0)
        ctone = !ctone;
    return ctone;
}

static uint32_t pack(uint8_t *inbuf, uint32_t inlen, uint8_t* buf, uint32_t buf_len)
{
    memset(buf, 0, buf_len); // Clear buffer
    uint32_t blen = 0;

    // Preamble
    for(uint8_t i=0; i<30; i++) {
        for(uint8_t j=0; j<8; j++) {

            if(blen >> 3 >= buf_len) { // Buffer overflow
#ifdef PKT_IS_TEST_PROJECT
              dbgPrintf(DBG_ERROR, "SI   > Packet too long\r\n");
#else
                TRACE_ERROR("SI   > Packet too long");
#endif

                return blen;
            }

            buf[blen >> 3] |= encode_nrzi((0x7E >> j) & 0x1) << (blen % 8);
            blen++;
        }
    }

    // Insert CRC to buffer
    uint16_t crc = calc_crc16(inbuf, 0, inlen);
    inbuf[inlen++] = crc & 0xFF;
    inbuf[inlen++] = crc >> 8;

    uint32_t pos = 0;
    uint8_t bitstuff_cntr = 0;

    while(pos < inlen*8)
    {
        if(blen >> 3 >= buf_len) { // Buffer overflow
#ifdef PKT_IS_TEST_PROJECT
          dbgPrintf(DBG_ERROR, "SI   > Packet too long\r\n");
#else
          TRACE_ERROR("SI   > Packet too long");
#endif


            return blen;
        }

        bool bit;
        if(bitstuff_cntr < 5) { // Normal bit

            bit = (inbuf[pos >> 3] >> (pos%8)) & 0x1;
            if(bit == 1) {
                bitstuff_cntr++;
            } else {
                bitstuff_cntr = 0;
            }
            pos++;

        } else { // Fill stuffing bit

            bit = 0;
            bitstuff_cntr = 0;

        }

        // NRZ-I encode bit
        bool nrzi = encode_nrzi(bit);

        buf[blen >> 3] |= nrzi << (blen % 8);
        blen++;
    }

    // Final flag
    for(uint8_t i=0; i<10; i++)
        for(uint8_t j=0; j<8; j++) {

            if(blen >> 3 >= buf_len) { // Buffer overflow

#ifdef PKT_IS_TEST_PROJECT
                dbgPrintf(DBG_ERROR, "SI   > Packet too long\r\n");
#else
                TRACE_ERROR("SI   > Packet too long");
#endif
                return blen;
            }

            buf[blen >> 3] |= encode_nrzi((0x7E >> j) & 0x1) << (blen % 8);
            blen++;
        }

    return blen;
}

static uint8_t getUpsampledAFSKbyte(uint8_t* buf, uint32_t blen)
{
    if(packet_pos == blen)
      /* Packet transmission finished already so just return a zero. */
      return 0;

    uint8_t b = 0;
    for(uint8_t i=0; i<8; i++)
    {
        if(current_sample_in_baud == 0) {
            if((packet_pos & 7) == 0) { // Load up next byte
                current_byte = buf[packet_pos >> 3];
            } else { // Load up next bit
                current_byte = current_byte / 2;
            }
        }

        // Toggle tone (1200 <> 2200)
        phase_delta = (current_byte & 1) ? PHASE_DELTA_1200 : PHASE_DELTA_2200;

        phase += phase_delta;           // Add delta-phase (delta-phase tone dependent)
        b |= ((phase >> 16) & 1) << i;  // Set modulation bit

        current_sample_in_baud++;

        if(current_sample_in_baud == SAMPLES_PER_BAUD) {    // Old bit consumed, load next bit
            current_sample_in_baud = 0;
            packet_pos++;
        }
    }

    return b;
}

THD_FUNCTION(si_fifo_feeder_afsk, arg)
{
    packet_t pp = arg;
    chRegSetThreadName("radio_afsk_feeder");

    uint8_t layer0[3072];
    uint32_t layer0_blen = pack(pp->frame_data,
                                pp->frame_len, layer0, sizeof(layer0));

    // Initialize variables for timer
    phase_delta = PHASE_DELTA_1200;
    phase = 0;
    packet_pos = 0;
    current_sample_in_baud = 0;
    current_byte = 0;

    /* Create bit pattern for an HDLC flag. */
    //uint8_t flag_buffer[SAMPLES_PER_BAUD];
    //uint8_t a_flag[] = {0x7E};
    // WIP

    /* The maximum amount of FIFO data for either separate or combined. */
    uint8_t localBuffer[Si446x_FIFO_COMBINED_SIZE];

    /* Get the FIFO buffer amount currently available. */
    uint8_t free = Si446x_freeFIFO();

    /*
     * Round up modulation bits into next byte.
     * Calculate initial FIFO fill.
     */
    uint16_t all = ((layer0_blen * SAMPLES_PER_BAUD) + 7) / 8;
    uint16_t c = (all > free) ? free : all;

    // Initial FIFO fill
    for(uint16_t i = 0;  i < c; i++)
        localBuffer[i] = getUpsampledAFSKbyte(layer0, layer0_blen);
    Si446x_writeFIFO(localBuffer, c);

    // Start transmission
    if(Si446x_transmit(radio_freq, radio_pwr, all, 0x4F, TIME_S2I(10))) {
      /* Transmit started OK. */
      while(c < all) { // Do while bytes not written into FIFO completely
          // Determine free memory in Si446x-FIFO
          uint8_t more = Si446x_freeFIFO();
          if(more > all - c) {
              if((more = all - c) == 0) // Calculate remainder to send
                break; // End if nothing left
          }

          for(uint16_t i = 0; i < more; i++)
              localBuffer[i] = getUpsampledAFSKbyte(layer0, layer0_blen);

          Si446x_writeFIFO(localBuffer, more); // Write into FIFO
          c += more;
          chThdSleep(TIME_MS2I(15));
      }
    } else {
      /* Transmit start failed. */
#ifdef PKT_IS_TEST_PROJECT
          dbgPrintf(DBG_ERROR, "SI   > Transmit failed\r\n");
#else
          TRACE_ERROR("SI   > Transmit failed");
#endif
    }
    /*
     * Shutdown radio if reception has been interrupted. If reception was interrupted rx_frequency is set.
     * If reception has not been interrupted rx_frequency is set 0.
     */
    if(!rx_frequency) {
        Si446x_shutdown();
    } else {
        Si4464_restoreRX();
    }
    // Free packet object memory
    ax25_delete(pp);
    chThdExit(MSG_OK);
}

void Si446x_sendAFSK(packet_t pp, uint32_t freq, uint8_t pwr) {
    lockRadio();

    // Stop packet handler (if started)
    if(packetHandler) {

#ifdef PKT_IS_TEST_PROJECT
        dbgPrintf(DBG_INFO, "SI   > Pause packet reception\r\n");
#else
        TRACE_INFO("SI   > Pause packet reception");
#endif
        pktPauseDecoder(packetHandler);
    }

    // Initialize radio
    if(!radioInitialized)
        Si446x_init();
    Si446x_setModemAFSK_TX();

    // Set pointers for feeder
    /*
     * TODO: The frame object should contain freq and power as well.
     */

    radio_freq = freq;
    radio_pwr = pwr;

    // Start/re-start FIFO feeder
    feeder_thd = chThdCreateStatic(si_fifo_feeder_wa,
                                   sizeof(si_fifo_feeder_wa),
                                   HIGHPRIO,
                                   si_fifo_feeder_afsk,
                                   /*NULL*/pp);

    // Wait for the transmitter to start (because it is used as mutex)
    while(Si446x_getState() != Si446x_STATE_TX)
        chThdSleep(TIME_MS2I(1));

    unlockRadio();
}


/* ===================================================================== AFSK Receiver ====================================================================== */

void Si446x_mapCallback(pkt_data_object_t *pkt_buff) {
  /* Packet buffer. */
  ax25char_t *frame_buffer = pkt_buff->buffer;
  uint16_t frame_size = pkt_buff->packet_size;

  /* FIXME: This is a quick diagnostic implementation only. */
#if DUMP_PACKET_TO_SERIAL == TRUE
  pktDiagnosticOutput(pkt_buff->handler, pkt_buff);
#endif
if(pktIsBufferValidAX25Frame(pkt_buff)) {
  /* Perform the callback. */
  rx_cb(frame_buffer, frame_size);
  } else {
#ifdef PKT_IS_TEST_PROJECT
    dbgPrintf(DBG_INFO, "RX   > Invalid frame - dropped\r\n");
#else
    TRACE_INFO("RX   > Invalid frame - dropped");
#endif
  }
}

void Si446x_startDecoder(radio_freq_t freq, radio_squelch_t sq, void* cb) {

    rx_cb = cb;

    /* Open packet radio service. */
    pktOpenRadioService(PKT_RADIO_1,
                         DECODE_AFSK,
                         freq,
                         PKT_RADIO_CHANNEL_STEPPING_NONE,
                         &packetHandler);

    /* Start the decoder. */
    pktStartDataReception(packetHandler,
                           94,
                           sq,
                           Si446x_mapCallback);
}

void Si446x_stopDecoder(void) {
    // TODO: Nothing yet here
}

/* ========================================================================== 2FSK ========================================================================== */

THD_FUNCTION(si_fifo_feeder_fsk, arg)
{
    packet_t pp = arg;
    chRegSetThreadName("radio_2fsk_feeder");

/*
    uint8_t *frame = pp->frame_data;
    uint32_t len = pp->frame_len;

    uint16_t c = Si446x_freeFIFO();
    uint16_t all = (radio_msg.bin_len+7)/8;

    // Initial FIFO fill
    Si446x_writeFIFO(radio_msg.buffer, c);

    // Start transmission
    radioTune((uint32_t)frequency, 0, radio_msg.power, all);

    while(c < all) { // Do while bytes not written into FIFO completely
        // Determine free memory in Si446x-FIFO
        uint8_t more = Si446x_freeFIFO();
        if(more > all-c) {
            if((more = all-c) == 0) // Calculate remainder to send
              break; // End if nothing left
        }
        Si446x_writeFIFO(&radio_msg.buffer[c], more); // Write into FIFO
        c += more;
        chThdSleep(TIME_MS2I(15)); // That value is ok up to 96k
    }
*/

    /*
     * Shutdown radio if reception has been interrupted. If reception was interrupted rx_frequency is set.
     * If reception has not been interrupted rx_frequency is set 0.
     */
    if(!rx_frequency) {
        Si446x_shutdown();
    } else {
        Si4464_restoreRX();
    }

    // Delete packet
    ax25_delete(pp);
    chThdExit(MSG_OK);
}

void Si446x_send2FSK(packet_t pp, uint32_t freq, uint8_t pwr, uint32_t speed) {
    lockRadio();

    // Stop packet handler (if started)
    if(packetHandler)
      pktPauseDecoder(packetHandler);

    // Initialize radio
    if(!radioInitialized)
        Si446x_init();
    Si446x_setModem2FSK(speed);

    // Set pointers for feeder
    radio_freq = freq;
    radio_pwr = pwr;

    // Start/re-start FIFO feeder
    feeder_thd = chThdCreateStatic(si_fifo_feeder_wa,
                                   sizeof(si_fifo_feeder_wa),
                                   HIGHPRIO,
                                   si_fifo_feeder_fsk,
                                   pp);

    // Wait for the transmitter to start (because it is used as mutex)
    while(Si446x_getState() != Si446x_STATE_TX)
        chThdSleep(TIME_MS2I(1));

    unlockRadio();
}

/* ========================================================================== Misc ========================================================================== */

static int16_t Si446x_getTemperature(void) {
    const uint8_t txData[2] = {0x14, 0x10};
    uint8_t rxData[8];
    Si446x_read(txData, 2, rxData, 8);
    uint16_t adc = rxData[7] | ((rxData[6] & 0x7) << 8);
    return (89900*adc)/4096 - 29300;
}

int16_t Si446x_getLastTemperature(void) {
    if(lastTemp == 0x7FFF) { // Temperature was never measured => measure it now
        Si446x_init();
        Si446x_shutdown();
    }

    return lastTemp;
}

//#endif
