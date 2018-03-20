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
#ifndef PKT_IS_TEST_PROJECT
#include "radio.h"
#endif

// Access locking

#if Si446x_LOCK_BY_SEMAPHORE != TRUE
// Mutex
static mutex_t radio_mtx;               // Radio mutex
static bool radio_mtx_init = false;
#else
// Binary semaphore
static binary_semaphore_t radio_sem;
static bool radio_sem_init = false;
#endif

//static bool nextTransmissionWaiting;    // Flag that informs the feeder thread to keep the radio switched on

// Feeder thread variables
static thread_t* fsk_feeder_thd = NULL;
//static thread_t* afsk_feeder_thd = NULL;
#if USE_DYNAMIC_AFSK_TX != TRUE
static THD_WORKING_AREA(si_afsk_fifo_feeder_wa, SI_AFSK_FIFO_FEEDER_WA_SIZE);
#endif
#if USE_DYNAMIC_FSK_TX != TRUE
static THD_WORKING_AREA(si_fsk_fifo_feeder_wa, SI_FSK_FIFO_FEEDER_WA_SIZE);
#endif

/*
 * Transmitter global variables.
 * Saved when setting band. */
static uint32_t tx_frequency;
static uint16_t tx_step;

// Si446x variables
static int16_t lastTemp = 0x7FFF;
static bool radioInitialized;

// Receiver thread variables
static uint32_t rx_frequency;
static uint16_t rx_step;
static uint8_t rx_chan;
static uint8_t rx_rssi;
static mod_t rx_mod;
//static void (*rx_cb)(uint8_t*, uint32_t);

packet_svc_t *packetHandler;

//static int16_t Si446x_getTemperature(void);

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
  TRACE_INFO("SI   > Init radio");
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

    /* Clear FIFO. */
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
     * To do this with AFSK up-sampling requires a preamble pattern of 88 bits.
     * The 446x only has up to 32 pattern bits.
     * Why 88 bits? Due to the oversampling used to create AFSK at 13.2ksps.
     * Each HDLC bit takes 11 TX bit times.
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

bool Si446x_setBandParameters(uint32_t freq,
                              uint16_t step,
                              radio_mode_t mode) {

  /* Check band is in range. */
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

  /* Initialize radio GPIO and primary register set. */
  if(!radioInitialized)
    Si446x_init();

  switch(mode) {
  case RADIO_RX:
    rx_step = step;
    rx_frequency = freq;
    break;

  case RADIO_TX:
    tx_frequency = freq;
    tx_step = step;
    break;

  case RADIO_CCA:
    break;
  } /* End switch. */



    /* Set the band parameter. */
    uint32_t sy_sel = 8;
    uint8_t set_band_property_command[] = {0x11, 0x20, 0x01, 0x51, (band + sy_sel)};
    Si446x_write(set_band_property_command, 5);

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

    uint8_t set_frequency_property_command[] = {0x11, 0x40, 0x04, 0x00, n, m2, m1, m0, c1, c0};
    Si446x_write(set_frequency_property_command, 10);

    uint32_t x = ((((uint32_t)1 << 19) * outdiv * 1300.0)/(2*Si446x_CCLK))*2;
    uint8_t x2 = (x >> 16) & 0xFF;
    uint8_t x1 = (x >>  8) & 0xFF;
    uint8_t x0 = (x >>  0) & 0xFF;
    uint8_t set_deviation[] = {0x11, 0x20, 0x03, 0x0a, x2, x1, x0};
    Si446x_write(set_deviation, 7);
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

    // Use upsampled AFSK from FIFO (PH)
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


/* ====================================================================== Radio Settings ====================================================================== */

static uint8_t __attribute__((unused)) Si446x_getChannel(void) {
    const uint8_t state_info[] = {Si446x_REQUEST_DEVICE_STATE};
    uint8_t rxData[4];
    Si446x_read(state_info, sizeof(state_info), rxData, sizeof(rxData));
    return rxData[3];
}

/* ======================================================================= Radio FIFO ======================================================================= */

static void Si446x_writeFIFO(uint8_t *msg, uint8_t size) {
    uint8_t write_fifo[size+1];
    write_fifo[0] = 0x66;
    memcpy(&write_fifo[1], msg, size);
    Si446x_write(write_fifo, size+1);
}

static uint8_t Si446x_getTXfreeFIFO(void) {
    const uint8_t fifo_info[] = {Si446x_FIFO_INFO, 0x00};
    uint8_t rxData[4];
    Si446x_read(fifo_info, sizeof(fifo_info), rxData, sizeof(rxData));
    return rxData[3];
}

/* ====================================================================== Radio States ====================================================================== */

static uint8_t Si446x_getState(void)
{
    const uint8_t state_info[] = {Si446x_REQUEST_DEVICE_STATE};
    uint8_t rxData[4];
    Si446x_read(state_info, sizeof(state_info), rxData, sizeof(rxData));
    return rxData[2] & 0xF;
}

static void Si446x_setTXState(uint8_t chan, uint16_t size)
{
    uint8_t change_state_command[] = {0x31, chan,
                                      (Si446x_STATE_READY << 4),
                                      (size >> 8) & 0x1F, size & 0xFF};
    Si446x_write(change_state_command, sizeof(change_state_command));
}

static void Si446x_setReadyState(void)
{
    const uint8_t change_state_command[] = {0x34, 0x03};
    Si446x_write(change_state_command, sizeof(change_state_command));
}

static void Si446x_setRXState(uint8_t chan)
{
    const uint8_t change_state_command[] = {0x32, chan, 0x00, 0x00,
                                            0x00, 0x00, 0x08, 0x08};
    Si446x_write(change_state_command, sizeof(change_state_command));
}

static void Si446x_shutdown(void)
{
  TRACE_INFO("SI   > Shutdown radio");
  pktDeconfigureRadioGPIO();
  radioInitialized = false;
}

/* ======================================================================== Locking ========================================================================= */

void Si446x_lockRadio(radio_mode_t mode) {
#if Si446x_LOCK_BY_SEMAPHORE == TRUE
  /* Initialize semaphore. */
  if(!radio_sem_init)
    chBSemObjectInit(&radio_sem, false);
  radio_sem_init = true;

  chBSemWait(&radio_sem);
#else
    // Initialize mutex
    if(!radio_mtx_init)
        chMtxObjectInit(&radio_mtx);
    radio_mtx_init = true;

    chSysLock();
    chMtxLockS(&radio_mtx);
    nextTransmissionWaiting = true;
    ChSysUnlock();
#endif


    if(rx_frequency && mode == RADIO_TX) {
      TRACE_INFO("SI   > Pause packet reception for packet transmit");
      pktPauseDecoder(PKT_RADIO_1);
    }
}

static bool Si4464_restoreRX(void);

void Si446x_unlockRadio(radio_mode_t mode) {

#if Si446x_LOCK_BY_SEMAPHORE == TRUE
  chBSemSignal(&radio_sem);
#else
    chMtxUnlock(&radio_mtx);
#endif
    if(rx_frequency != 0 && mode == RADIO_TX) {
      Si4464_restoreRX();
    } else if(rx_frequency == 0) {
      Si446x_shutdown();
    }
}

void Si446x_unlockRadioByCamera(void) {
#if Si446x_LOCK_BY_SEMAPHORE == TRUE
  chBSemSignal(&radio_sem);
#else
    chMtxUnlock(&radio_mtx);
#endif
}

void Si446x_lockRadioByCamera(void)
{
#if Si446x_LOCK_BY_SEMAPHORE == TRUE
  /* Initialize semaphore. */
  if(!radio_sem_init)
    chBSemObjectInit(&radio_sem, false);
  radio_sem_init = true;
  chBSemWait(&radio_sem);
#else
    // Initialize mutex
    if(!radio_mtx_init)
        chMtxObjectInit(&radio_mtx);
    radio_mtx_init = true;

    chMtxLock(&radio_mtx);
#endif
}

/* ====================================================================== Radio TX/RX ======================================================================= */

static bool Si446x_isRadioInBand(uint8_t chan, radio_mode_t mode) {
  uint32_t base_freq;
  uint16_t step;
  switch(mode) {
  case RADIO_RX:
    base_freq = rx_frequency;
    step = rx_step;
    break;

  case RADIO_CCA:
  case RADIO_TX:
    base_freq = tx_frequency;
    step = tx_step;
    break;
  }
  uint32_t freq = Si446x_computeOperatingFrequency(base_freq, step, chan);
  return (Si446x_MIN_FREQ <= freq && freq < Si446x_MAX_FREQ);
}

static bool Si446x_getLatchedCCA(uint8_t ms) {
    uint16_t cca = 0;
    for(uint16_t i=0; i<ms*10; i++) {
        cca += Si446x_getCCA();
        /* FIXME: Using 5KHz systick lowest resolution is 200uS. */
        chThdSleep(TIME_US2I(100));
    }
    TRACE_INFO("SI   > CCA=%03d RX=%d", cca, cca > ms/10);
    return cca > ms; // Max. 1 spike per ms
}

/*
 * Wait for a clear time slot and initiate packet transmission.
 */
static bool Si446x_transmit(uint8_t chan,
                            int8_t power, uint16_t size,
                            uint8_t rssi, sysinterval_t sql_timeout)
{
    if(!Si446x_isRadioInBand(chan, RADIO_TX)) {
      TRACE_ERROR("SI   > Frequency out of range");
      TRACE_ERROR("SI   > abort transmission");
      return false;
    }

    // Switch to ready state
    if(Si446x_getState() == Si446x_STATE_RX) {
      TRACE_INFO("SI   > Switch Si446x to ready state");
      Si446x_setReadyState();
      chThdSleep(TIME_MS2I(1));
    }

    Si446x_setProperty8(Si446x_MODEM_RSSI_THRESH, rssi);
    /* Change band parameters for CCA RX temporarily. */
    Si446x_setBandParameters(tx_frequency, tx_step, RADIO_CCA);     // Set frequency
    Si446x_setRXState(chan);

    // Wait until nobody is transmitting (until timeout)

    if(Si446x_getState() != Si446x_STATE_RX || Si446x_getLatchedCCA(50)) {

        TRACE_INFO("SI   > Wait for clear channel");

        /* FIXME: Fix timeout. Using 5KHz systick lowest resolution is 200uS. */
        sysinterval_t t0 = chVTGetSystemTime();
        while((Si446x_getState() != Si446x_STATE_RX
            || Si446x_getLatchedCCA(50))
            && chVTGetSystemTime() - t0 < sql_timeout)
            chThdSleep(TIME_US2I(100));
    }

    // Transmit
    TRACE_INFO("SI   > Tune Si446x (TX)");
    Si446x_setReadyState();
    /* Set band parameters back to normal TX. */
    Si446x_setBandParameters(tx_frequency, tx_step, RADIO_CCA);     // Set frequency
    Si446x_setPowerLevel(power);        // Set power level
    Si446x_setTXState(chan, size);

    // Wait until transceiver enters transmission state
    /* TODO: Make a function to handle timeout on fail to reach state. */
    while(Si446x_getState() != Si446x_STATE_TX) {
        chThdSleep(TIME_US2I(500));
    }

    return true;
}

/*static*/ bool Si446x_receiveNoLock(uint8_t channel,
                                     uint8_t rssi,
                                     mod_t mod) {
  /* TODO: compute f + s*c. */
    if(!Si446x_isRadioInBand(channel, RADIO_RX)) {
      TRACE_ERROR("SI   > Frequency out of range");
      TRACE_ERROR("SI   > abort reception");
      return false;
    }

    // Initialize radio
/*    if(!radioInitialized)
        Si446x_init();*/

    uint16_t tot = 0;
    // Wait until transceiver finishes transmission (if there is any)
    while(Si446x_getState() == Si446x_STATE_TX) {
        chThdSleep(TIME_MS2I(10));
        if(tot++ < 500)
          continue;
        /* Remove TX state. */
        Si446x_setReadyState();

      TRACE_ERROR("SI   > Timeout waiting for TX state end");
      TRACE_ERROR("SI   > Attempt start of receive");

      break;
    }

    // Initialize radio
    if(mod == MOD_AFSK) {
        Si446x_setModemAFSK_RX();
    } else {
        //Si446x_shutdown();

        TRACE_ERROR("SI   > Modulation type not supported in receive");
        TRACE_ERROR("SI   > abort reception");

        return false;
    }

    // Preserve settings in case transceiver changes to TX state
    rx_rssi = rssi;
    rx_chan = channel;
    rx_mod = mod;


    TRACE_INFO("SI   > Tune Si446x (RX)");

    Si446x_setProperty8(Si446x_MODEM_RSSI_THRESH, rssi);
    //Si446x_setBandParameters(rx_frequency, rx_step, RADIO_RX);     // Set frequency
    Si446x_setRXState(channel);

    // Wait for the receiver to start (because it is used as mutex)
    while(Si446x_getState() != Si446x_STATE_RX)
        chThdSleep(TIME_MS2I(1));
    return true;
}

static bool Si4464_restoreRX(void) {
    bool ret = Si446x_receiveNoLock(rx_chan, rx_rssi, rx_mod);

    uint32_t op_freq = Si446x_computeOperatingFrequency(rx_frequency,
                                                        rx_step,
                                                        rx_chan);

    if(rx_frequency) {


        TRACE_INFO( "SI   > Resume packet reception %d.%03d MHz (ch %d),"
                    " RSSI %d, %s",
                    op_freq/1000000, (op_freq % 1000000)/1000,
                    rx_chan,
                    rx_rssi, getModulation(rx_mod));

        /* Resume decoding. */
        pktResumeDecoder(PKT_RADIO_1);
    }
    return ret;
}

void Si446x_receiveStop(void)
{
  /* FIXME: */
  if(Si446x_getState() == Si446x_STATE_RX) {
    rx_frequency = 0;
    Si446x_shutdown();
  }
}

/* ==================================================================== AFSK Transmitter ==================================================================== */

#define PLAYBACK_RATE       13200
#define BAUD_RATE           1200                                    /* APRS AFSK baudrate */
#define SAMPLES_PER_BAUD    (PLAYBACK_RATE / BAUD_RATE)             /* Samples per baud (13200Hz / 1200baud = 11samp/baud) */
#define PHASE_DELTA_1200    (((2 * 1200) << 16) / PLAYBACK_RATE)    /* Delta-phase per sample for 1200Hz tone */
#define PHASE_DELTA_2200    (((2 * 2200) << 16) / PLAYBACK_RATE)    /* Delta-phase per sample for 2200Hz tone */

static uint32_t phase_delta;            // 1200/2200 for standard AX.25
static uint32_t phase;                  // Fixed point 9.7 (2PI = TABLE_SIZE)
static uint32_t packet_pos;             // Next bit to be sent out
static uint32_t current_sample_in_baud; // 1 bit = SAMPLES_PER_BAUD samples
static uint8_t current_byte;
static uint8_t ctone = 0;

static bool Si446x_getBitAsNRZI(bool bit) {
    if((bit & 0x1) == 0)
        ctone = !ctone;
    return ctone;
}

/*
 * Create a bit stream of AFSK (NRZI & HDLC) encoded packet data.
 */
static uint32_t Si446x_encodeDataToAFSK(uint8_t *inbuf, uint32_t inlen,
                     uint8_t* buf, uint32_t buf_len, uint8_t pre_len) {
    memset(buf, 0, buf_len); // Clear buffer
    uint32_t blen = 0;

    // Preamble (HDLC flags)
    for(uint8_t i = 0; i <= pre_len; i++) {
        for(uint8_t j = 0; j < 8; j++) {

            if(blen >> 3 >= buf_len) { // Buffer overflow

                TRACE_ERROR("SI   > Preamble too long");


                return 0;
            }

            buf[blen >> 3] |= Si446x_getBitAsNRZI((0x7E >> j) & 0x1) << (blen % 8);
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

          TRACE_ERROR("SI   > Packet too long");

            return 0;
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
        bool nrzi = Si446x_getBitAsNRZI(bit);

        buf[blen >> 3] |= nrzi << (blen % 8);
        blen++;
    }

    // Final flag
    for(uint8_t i=0; i<10; i++)
        for(uint8_t j=0; j<8; j++) {

            if(blen >> 3 >= buf_len) { // Buffer overflow


                TRACE_ERROR("SI   > Packet too long");

                return 0;
            }

            buf[blen >> 3] |= Si446x_getBitAsNRZI((0x7E >> j) & 0x1) << (blen % 8);
            blen++;
        }

    return blen;
}

static uint8_t Si446x_getUpsampledAFSKbits(uint8_t* buf/*, uint32_t blen*/)
{
  /* This function may be called with different bit stream sources.
   * These will have their own blen so checking is not valid.
   */
    //if(packet_pos == blen)
      /* Packet transmission finished already so just return a zero. */
      //return 0;

    uint8_t b = 0;
    for(uint8_t i = 0; i < 8; i++)
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
        /* Add delta-phase (bit count within SAMPLES_PER_BAUD). */
        phase += phase_delta;
        b |= ((phase >> 16) & 1) << i;  // Set modulation bit

        current_sample_in_baud++;

        if(current_sample_in_baud == SAMPLES_PER_BAUD) {    // Old bit consumed, load next bit
            current_sample_in_baud = 0;
            packet_pos++;
        }
    }
    return b;
}

#define SI446X_EVT_AFSK_TX_TIMEOUT      EVENT_MASK(0)

static void Si446x_AFSKtransmitTimeoutI(thread_t *tp) {
  /* The tell the thread to terminate. */
  chEvtSignal(tp, SI446X_EVT_AFSK_TX_TIMEOUT);
}

THD_FUNCTION(si_fifo_feeder_afsk, arg) {
    packet_t pp = arg;

#if USE_DYNAMIC_AFSK_TX != TRUE
    chRegSetThreadName("446x_afsk_tx");
#endif

    /* Initialize variables for AFSK encoder. */
    ctone = 0;
    virtual_timer_t send_timer;

    chVTObjectInit(&send_timer);

#define PREAMBLE_FLAGS_A    30
#define PREAMBLE_FLAGS_B     0

    uint8_t layer0[AFSK_FEEDER_BUFFER_SIZE];
    /* Encode packet to AFSK (NRZI & HDLC) with optional preamble. */
    uint32_t layer0_blen = Si446x_encodeDataToAFSK(pp->frame_data,
                                                   pp->frame_len,
                                                   layer0, sizeof(layer0),
                                                   PREAMBLE_FLAGS_A);
    if(layer0_blen == 0) {
      /* Nothing encoded. Release packet send object. */
      pktReleaseSendObject(pp);

      /* Exit thread. */
      chThdExit(MSG_RESET);
    }
#if PREAMBLE_FLAGS_B > 0
    /* Create NRZI pattern for an HDLC flag. */
      uint8_t a_flag[] = {0x7e};
      uint8_t flag_nrz[sizeof(a_flag)];
      uint32_t flag_blen = Si446x_encodeDataToAFSK(a_flag, sizeof(a_flag),
                                                   flag_nrz, sizeof(flag_nrz),
                                                   0);

      /* WIP. */
#endif

    /* Reset TX FIFO in case some remnant unsent data is left there. */
    const uint8_t reset_fifo[] = {0x15, 0x01};
    Si446x_write(reset_fifo, 2);

    /* Initialize variables for up sampler. */
    phase_delta = PHASE_DELTA_1200;
    phase = 0;
    packet_pos = 0;
    current_sample_in_baud = 0;
    current_byte = 0;

    /* Maximum amount of FIFO data when using combined TX+RX (safe size). */
    uint8_t localBuffer[Si446x_FIFO_COMBINED_SIZE];

    /* Get the FIFO buffer amount currently available. */
    uint8_t free = Si446x_getTXfreeFIFO();

    /*
     * Account for all modulation bits (round up to a byte boundary).
     * Calculate initial FIFO fill.
     */
    uint16_t all = ((uint64_t)(layer0_blen * SAMPLES_PER_BAUD) + 7) / 8;
    uint16_t c = (all > free) ? free : all;


    TRACE_INFO("SI   > AFSK upsampled bytes to send %i", all);

    /*
     * Start transmission timeout timer.
     * If the 446x gets locked up we'll exit TX and release packet object.
     */
    chVTSet(&send_timer, TIME_S2I(10),
             (vtfunc_t)Si446x_AFSKtransmitTimeoutI, chThdGetSelfX());

    /* The exit message if all goes well. */
    msg_t exit_msg = MSG_OK;

    /* Initial FIFO load. */
    for(uint16_t i = 0;  i < c; i++)
        localBuffer[i] = Si446x_getUpsampledAFSKbits(layer0);
    Si446x_writeFIFO(localBuffer, c);

    /* Request start of transmission. */
    if(Si446x_transmit(pp->radio_chan, pp->radio_pwr, all, pp->cca_rssi, TIME_S2I(10))) {
      /* Feed the FIFO while data remains to be sent. */
      while((all - c) > 0) {
        /* Get TX FIFO free count. */
        uint8_t more = Si446x_getTXfreeFIFO();
        /* If there is more free than we need for send use remainder only. */
        more = (more > (all - c)) ? (all - c) : more;

        /* Load the FIFO. */
        for(uint16_t i = 0; i < more; i++)
            localBuffer[i] = Si446x_getUpsampledAFSKbits(layer0);
        Si446x_writeFIFO(localBuffer, more); // Write into FIFO
        c += more;

        /*
         * Wait for a timeout event during up-sampled AFSK byte time period.
         * Time delay allows for ~11 bytes of transmit data from the FIFO.
         * If no timeout event go back and load more data to FIFO.
         */
        eventmask_t evt = chEvtWaitAnyTimeout(SI446X_EVT_AFSK_TX_TIMEOUT,
                                   chTimeUS2I(833 * 8 * SAMPLES_PER_BAUD));
        if(evt) {
          /* Force 446x out of TX state. */
          Si446x_setReadyState();
          exit_msg = MSG_TIMEOUT;
          break;
        }
      }
    } else {
      /* Transmit start failed. */
        TRACE_ERROR("SI   > Transmit start failed");
    }
    chVTReset(&send_timer);

    /*
     * If nothing went wrong wait for TX to finish.
     * Else don't wait.
     */
    while(Si446x_getState() == Si446x_STATE_TX && exit_msg == MSG_OK) {
      /* Sleep for an AFSK up-sampled byte time. */
      chThdSleep(chTimeUS2I(833 * 8 * SAMPLES_PER_BAUD));
      continue;
    }

    // Free packet object memory
    pktReleaseSendObject(pp);

    /* Exit thread. */
    chThdExit(exit_msg);
}

THD_FUNCTION(new_si_fifo_feeder_afsk, arg) {
    packet_t pp = arg;

#if USE_DYNAMIC_AFSK_TX != TRUE
    chRegSetThreadName("446x_afsk_tx");
#endif

    /* Initialize variables for AFSK encoder. */

    virtual_timer_t send_timer;

    chVTObjectInit(&send_timer);

    uint8_t layer0[AFSK_FEEDER_BUFFER_SIZE];

    TRACE_INFO("SI   > Packet frame bytes %i", pp->frame_len);

    /* TODO: Make this a prepare function.
     * Create iterator object and complete TX frame.
     * Insert CRC in radio.c transmit function?
     */
/*    uint16_t crc = calc_crc16(pp->frame_data, 0, pp->frame_len);
    pp->frame_data[pp->frame_len++] = crc & 0xFF;
    pp->frame_data[pp->frame_len++] = crc >> 8;
    pp->frame_data[pp->frame_len++] = HDLC_FLAG;*/

    static tx_iterator_t iterator;

    pktStreamIteratorInit(&iterator, pp, false);

    static int32_t data = 0;
    data = pktStreamEncodingIterator(&iterator, layer0, sizeof(layer0));

    TRACE_INFO("SI   > Iterator data count %i, RLL count %i,"
        " out index %i, out bytes %i, out bits %i",
               data, iterator.rll_count,
               iterator.out_index, iterator.out_index >> 3,
               iterator.out_index % 8);

    /* Reset TX FIFO in case some remnant unsent data is left there. */
    const uint8_t reset_fifo[] = {0x15, 0x01};
    Si446x_write(reset_fifo, 2);

    /* Maximum amount of FIFO data when using combined TX+RX (safe size). */
    uint8_t localBuffer[Si446x_FIFO_COMBINED_SIZE];

    /* Get the FIFO buffer amount currently available. */
    uint8_t free = Si446x_getTXfreeFIFO();

    /* Calculate chunk size for stream iterator. */

    uint8_t stream_size = free / SAMPLES_PER_BAUD;

    /* Allocate a stream output buffer. */
    uint8_t stream_out[stream_size];

    TRACE_INFO("SI   > AFSK stream buffer %i @ 0x%x", stream_size, stream_out);
    /*
     * Account for all modulation bits (round up to a byte boundary).
     * Calculate initial FIFO fill.
     */
    uint16_t all = data * SAMPLES_PER_BAUD;
    uint16_t c = 0; /*(all > free) ? free : all;*/

    TRACE_INFO("SI   > AFSK frame bytes to send %i, upsampled %i", data, all);

    /*
     * Start transmission timeout timer.
     * If the 446x gets locked up we'll exit TX and release packet object.
     */
    chVTSet(&send_timer, TIME_S2I(10),
             (vtfunc_t)Si446x_AFSKtransmitTimeoutI, chThdGetSelfX());

    /* The exit message if all goes well. */
    msg_t exit_msg = MSG_OK;
    bool tx_started = false;

    /* Initialize variables for up sampler. */
    phase_delta = PHASE_DELTA_1200;
    phase = 0;
    packet_pos = 0;
    current_sample_in_baud = 0;
    current_byte = 0;

    uint8_t lower = Si446x_FIFO_COMBINED_SIZE;

    /* Feed the FIFO while data remains to be sent. */
    while((all - c) > 0) {
      /* Get TX FIFO free count. */
      uint8_t more = Si446x_getTXfreeFIFO();

      /* Update the FIFO low water mark. */
      lower = (more < lower) ? more : lower;

      /* If there is more free than we need for send use remainder only. */
      more = (more > (all - c)) ? (all - c) : more;

      /* Load the FIFO. */
      for(uint16_t i = 0; i < more; i++)
          localBuffer[i] = Si446x_getUpsampledAFSKbits(layer0);
      Si446x_writeFIFO(localBuffer, more); // Write into FIFO
      c += more;

      if(!tx_started) {
        /* Request start of transmission. */
        if(!Si446x_transmit(pp->radio_chan, pp->radio_pwr, all,
                           pp->cca_rssi, TIME_S2I(10))) {
          /* Transmit start failed. */
          TRACE_ERROR("SI   > Transmit start failed");
          exit_msg = MSG_RESET;
          break;
        } /* Else. */
            tx_started = true;
      }
      /*
       * Wait for a timeout event during up-sampled AFSK byte time period.
       * Time delay allows for ~11 bytes of transmit data from the FIFO.
       * If no timeout event go back and load more data to FIFO.
       */
      eventmask_t evt = chEvtWaitAnyTimeout(SI446X_EVT_AFSK_TX_TIMEOUT,
                                 chTimeUS2I(833 * 8 * SAMPLES_PER_BAUD));
      if(evt) {
        /* Force 446x out of TX state. */
        Si446x_setReadyState();
        exit_msg = MSG_TIMEOUT;
        break;
      }
    } /* End while. */
    chVTReset(&send_timer);

    /*
     * If nothing went wrong wait for TX to finish.
     * Else don't wait.
     */
    while(Si446x_getState() == Si446x_STATE_TX && exit_msg == MSG_OK) {
      /* Sleep for an AFSK up-sampled byte time. */
      chThdSleep(chTimeUS2I(833 * 8 * SAMPLES_PER_BAUD));
      continue;
    }

    // Free packet object memory
    pktReleaseSendObject(pp);

    TRACE_INFO("SI   > TX FIFO low level %i", lower);

    /* Exit thread. */
    chThdExit(exit_msg);
}

void Si446x_sendAFSK(packet_t pp,
                     uint8_t chan,
                     uint8_t pwr) {

    // Initialize radio
    if(!radioInitialized)
        Si446x_init();

    /* Set 446x back to READY. */
    Si446x_setReadyState();

    /* Set parameters for AFSK transmission. */
    Si446x_setModemAFSK_TX();

    /* Set transmit parameters. */
    pp->radio_chan = chan;
    pp->radio_pwr = pwr;

    /* TODO: Don't use fixed RSSI. */
    pp->cca_rssi = 0x4F;

    thread_t *afsk_feeder_thd = NULL;

#if USE_DYNAMIC_AFSK_TX == TRUE
    afsk_feeder_thd = chThdCreateFromHeap(NULL,
                THD_WORKING_AREA_SIZE(SI_AFSK_FIFO_FEEDER_WA_SIZE),
                "446x_afsk_tx",
                NORMALPRIO - 10,
                si_fifo_feeder_afsk,
                pp);
#else
    // Start/re-start FIFO feeder
    afsk_feeder_thd = chThdCreateStatic(si_afsk_fifo_feeder_wa,
                                   sizeof(si_afsk_fifo_feeder_wa),
                                   NORMALPRIO - 10,
                                   si_fifo_feeder_afsk,
                                   pp);
#endif

    if(afsk_feeder_thd == NULL) {
      /* Release packet object. */
      pktReleaseSendObject(pp);
      /* Unlock radio. */
      Si446x_unlockRadio(RADIO_TX);
      TRACE_ERROR("SI   > Unable to create AFSK transmit thread");
      return;
    }
    /* Wait for transmit thread to terminate. */
    msg_t send_msg = chThdWait(afsk_feeder_thd);
    if(send_msg == MSG_TIMEOUT) {
      TRACE_ERROR("SI   > Transmit AFSK timeout");
    }
    if(send_msg == MSG_RESET) {
      TRACE_ERROR("SI   > Transmit AFSK buffer overrun");
    }
    /* Unlock radio. */
    Si446x_unlockRadio(RADIO_TX);
}


/* ===================================================================== AFSK Receiver ====================================================================== */



void Si446x_stopDecoder(void) {
    // TODO: Nothing yet here
}

/* ========================================================================== 2FSK ========================================================================== */

THD_FUNCTION(si_fifo_feeder_fsk, arg) {
  packet_t pp = arg;

#if USE_DYNAMIC_FSK_TX != TRUE
    chRegSetThreadName("446x_2fsk_tx");
#endif

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
     * If reception was interrupted rx_frequency is set.
     * Otherwise rx_frequwncy is zero in which case the radio can be shutdown.
     */
    if(!rx_frequency) {
        Si446x_shutdown();
    } else {
        Si4464_restoreRX();
    }

    // Delete packet
    pktReleaseSendObject(pp);
    chThdExit(MSG_OK);
}

void Si446x_send2FSK(packet_t pp,
                     uint8_t chan,
                     uint8_t pwr,
                     uint32_t speed) {
    Si446x_lockRadio(RADIO_TX);

    // Initialize radio
    if(!radioInitialized)
        //Si446x_init();
    Si446x_setModem2FSK(speed);

    // Set pointers for feeder
    pp->radio_chan = chan;
    pp->radio_pwr = pwr;

    /* TODO: Don't use fixed RSSI. */
    pp->cca_rssi = 0x4F;

    // Start/re-start FIFO feeder
    fsk_feeder_thd = chThdCreateStatic(si_fsk_fifo_feeder_wa,
                                   sizeof(si_fsk_fifo_feeder_wa),
                                   HIGHPRIO,
                                   si_fifo_feeder_fsk,
                                   pp);

    msg_t send_msg = chThdWait(fsk_feeder_thd);
    if(send_msg == MSG_TIMEOUT) {
      TRACE_ERROR("SI   > Transmit 2FSK timeout");
    }
    /* Unlock radio. */
    Si446x_unlockRadio(RADIO_TX);
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
    if(radioInitialized) {
      Si446x_lockRadio(RADIO_RX);
      // Temperature readout
      lastTemp = Si446x_getTemperature();
      TRACE_INFO("SI   > Transmitter temperature %d degC\r\n", lastTemp/100);
      Si446x_unlockRadio(RADIO_RX);
    } else {
      TRACE_INFO("SI   > Transmitter temperature not available");
      return 0;
    }
  }
  return lastTemp;
}

//#endif
