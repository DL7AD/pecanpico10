#ifndef __si446x__H__
#define __si446x__H__

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

#define Si446x_LOCK_BY_SEMAPHORE    TRUE

#ifndef Si446x_CLK
#error "Si446x_CLK is not defined which is needed for Si446x."
#endif

#ifndef Si446x_CLK_OFFSET
#define Si446x_CLK_OFFSET 0
#endif

// Various macros

#define Si446x_getGPIO0()           palReadLine(LINE_RADIO_GPIO0)
#define Si446x_getGPIO1()           palReadLine(LINE_RADIO_GPIO1)
#define Si446x_getCCA()             palReadLine(LINE_RADIO_IRQ)

 /* Frequency offset corrected oscillator frequency */
#define Si446x_CCLK                 ((Si446x_CLK) + (Si446x_CLK_OFFSET)      \
                                      * (Si446x_CLK) / 1000000)

// Si4464 States

#define Si446x_STATE_NOCHANGE       0
#define Si446x_STATE_SLEEP          1
#define Si446x_STATE_SPI_ACTIVE     2
#define Si446x_STATE_READY          3
#define Si446x_STATE_READY2         4
#define Si446x_STATE_TX_TUNE        5
#define Si446x_STATE_RX_TUNE        6
#define Si446x_STATE_TX             7
#define Si446x_STATE_RX             8

/*
 * Commands.
 */

#define Si446x_READ_CMD_BUFF                      0x44
#define Si446x_START_TX                           0x31
#define Si446x_START_RX                           0x32
#define Si446x_REQUEST_DEVICE_STATE               0x33
#define Si446x_RX_HOP                             0x36
#define Si446x_FIFO_INFO                          0x15

/* Defined response values. */

#define Si446x_COMMAND_CTS                        0xFF

/*
 * Property group commands.
 * Format is 0xGGNN (GG = group, NN = number).
 */
#define Si446x_GLOBAL_XO_TUNE                   0x0000
#define Si446x_GLOBAL_CLK_CFG                   0x0001
#define Si446x_GLOBAL_CONFIG                    0x0003

#define Si446x_INT_CTL_ENABLE                   0x0100
#define Si446x_INT_CTL_MODEM_ENABLE             0x0102

#define Si446x_FRR_CTL_A_MODE                   0x0200
#define Si446x_FRR_CTL_B_MODE                   0x0201
#define Si446x_FRR_CTL_C_MODE                   0x0202
#define Si446x_FRR_CTL_D_MODE                   0x0203

#define Si446x_PREAMBLE_TX_LENGTH               0x1000

#define Si446x_PREAMBLE_CONFIG_STD_1            0x1001
#define Si446x_PREAMBLE_CONFIG_NSTD             0x1002
#define Si446x_PREAMBLE_CONFIG_STD_2            0x1003
#define Si446x_PREAMBLE_CONFIG                  0x1004
#define Si446x_PREAMBLE_PATTERN                 0x1005

#define Si446x_SYNC_CONFIG                      0x1100

#define Si446x_PKT_CONFIG1                      0x1206

#define Si446x_MODEM_MOD_TYPE                   0x2000
#define Si446x_MODEM_MAP_CONTROL                0x2001
#define Si446x_MODEM_DSM_CTRL                   0x2002
#define Si446x_MODEM_DATA_RATE                  0x2003
#define Si446x_MODEM_TX_NCO_MODE                0x2006
#define Si446x_MODEM_FREQ_DEV                   0x200A
#define Si446x_MODEM_TX_RAMP_DELAY              0x2018
#define Si446x_MODEM_MDM_CTRL                   0x2019
#define Si446x_MODEM_IF_CONTROL                 0x201A
#define Si446x_MODEM_IF_FREQ                    0x201B
#define Si446x_MODEM_DECIMATION_CFG1            0x201E
#define Si446x_MODEM_DECIMATION_CFG0            0x201F
#define Si446x_MODEM_BCR_OSR                    0x2022
#define Si446x_MODEM_BCR_NCO_OFFSET             0x2024
#define Si446x_MODEM_BCR_GAIN                   0x2027
#define Si446x_MODEM_BCR_GEAR                   0x2029
#define Si446x_MODEM_BCR_MISC1                  0x202A
#define Si446x_MODEM_AFC_GEAR                   0x202C
#define Si446x_MODEM_AFC_WAIT                   0x202D
#define Si446x_MODEM_AFC_GAIN                   0x202E
#define Si446x_MODEM_AFC_LIMITER                0x2030
#define Si446x_MODEM_AFC_MISC                   0x2032
#define Si446x_MODEM_AGC_CONTROL                0x2035
#define Si446x_MODEM_AGC_WINDOW_SIZE            0x2038
#define Si446x_MODEM_AGC_RFPD_DECAY             0x2039
#define Si446x_MODEM_AGC_IFPD_DECAY             0x203A
#define Si446x_MODEM_FSK4_GAIN1                 0x203B
#define Si446x_MODEM_FSK4_GAIN0                 0x203C
#define Si446x_MODEM_FSK4_TH                    0x203D
#define Si446x_MODEM_FSK4_MAP                   0x203F
#define Si446x_MODEM_OOK_PDTC                   0x2040
#define Si446x_MODEM_OOK_CNT1                   0x2042
#define Si446x_MODEM_OOK_MISC                   0x2043
#define Si446x_MODEM_RAW_SEARCH                 0x2044

#define Si446x_MODEM_RAW_CONTROL                0x2045
#define Si446x_MODEM_RAW_EYE                    0x2046
#define Si446x_MODEM_ANT_DIV_MODE               0x2048
#define Si446x_MODEM_ANT_DIV_CONTROL            0x2049
#define Si446x_MODEM_RSSI_THRESH                0x204A
#define Si446x_MODEM_RSSI_JUMP_THRESH           0x204B
#define Si446x_MODEM_RSSI_CONTROL               0x204C
#define Si446x_MODEM_RSSI_CONTROL2              0x204D
#define Si446x_MODEM_RSSI_COMP                  0x204E
#define Si446x_MODEM_CLKGEN_BAND                0x2051

#define Si446x_MODEM_CHFLT_RX1_CHFLT_COE13_7_0  0x2100
#define Si446x_MODEM_CHFLT_RX1_CHFLT_COE12_7_0  0x2101
#define Si446x_MODEM_CHFLT_RX1_CHFLT_COE11_7_0  0x2102
#define Si446x_MODEM_CHFLT_RX1_CHFLT_COE10_7_0  0x2103
#define Si446x_MODEM_CHFLT_RX1_CHFLT_COE9_7_0   0x2104
#define Si446x_MODEM_CHFLT_RX1_CHFLT_COE8_7_0   0x2105
#define Si446x_MODEM_CHFLT_RX1_CHFLT_COE7_7_0   0x2106
#define Si446x_MODEM_CHFLT_RX1_CHFLT_COE6_7_0   0x2107
#define Si446x_MODEM_CHFLT_RX1_CHFLT_COE5_7_0   0x2108
#define Si446x_MODEM_CHFLT_RX1_CHFLT_COE4_7_0   0x2109
#define Si446x_MODEM_CHFLT_RX1_CHFLT_COE3_7_0   0x210A
#define Si446x_MODEM_CHFLT_RX1_CHFLT_COE2_7_0   0x210B
#define Si446x_MODEM_CHFLT_RX1_CHFLT_COE1_7_0   0x210C
#define Si446x_MODEM_CHFLT_RX1_CHFLT_COE0_7_0   0x210D
#define Si446x_MODEM_CHFLT_RX1_CHFLT_COEM0      0x210E
#define Si446x_MODEM_CHFLT_RX1_CHFLT_COEM1      0x210F
#define Si446x_MODEM_CHFLT_RX1_CHFLT_COEM2      0x2110
#define Si446x_MODEM_CHFLT_RX1_CHFLT_COEM3      0x2111

#define Si446x_MODEM_CHFLT_RX2_CHFLT_COE13_7_0  0x2112
#define Si446x_MODEM_CHFLT_RX2_CHFLT_COE12_7_0  0x2113
#define Si446x_MODEM_CHFLT_RX2_CHFLT_COE11_7_0  0x2114
#define Si446x_MODEM_CHFLT_RX2_CHFLT_COE10_7_0  0x2115
#define Si446x_MODEM_CHFLT_RX2_CHFLT_COE9_7_0   0x2116
#define Si446x_MODEM_CHFLT_RX2_CHFLT_COE8_7_0   0x2117
#define Si446x_MODEM_CHFLT_RX2_CHFLT_COE7_7_0   0x2118
#define Si446x_MODEM_CHFLT_RX2_CHFLT_COE6_7_0   0x2119
#define Si446x_MODEM_CHFLT_RX2_CHFLT_COE5_7_0   0x211A
#define Si446x_MODEM_CHFLT_RX2_CHFLT_COE4_7_0   0x211B
#define Si446x_MODEM_CHFLT_RX2_CHFLT_COE3_7_0   0x211C
#define Si446x_MODEM_CHFLT_RX2_CHFLT_COE2_7_0   0x211D
#define Si446x_MODEM_CHFLT_RX2_CHFLT_COE1_7_0   0x211E
#define Si446x_MODEM_CHFLT_RX2_CHFLT_COE0_7_0   0x211F
#define Si446x_MODEM_CHFLT_RX2_CHFLT_COEM0      0x2120
#define Si446x_MODEM_CHFLT_RX2_CHFLT_COEM1      0x2121
#define Si446x_MODEM_CHFLT_RX2_CHFLT_COEM2      0x2122
#define Si446x_MODEM_CHFLT_RX2_CHFLT_COEM3      0x2123

#define Si446x_PA_PWR_LVL                       0x2201
#define Si446x_PA_TC                            0x2203

#define Si446x_SYNTH_PFDCP_CPFF                 0x2300
#define Si446x_SYNTH_PFDCP_CPINT                0x2301
#define Si446x_SYNTH_VCO_KV                     0x2302
#define Si446x_SYNTH_LPFILT3                    0x2303
#define Si446x_SYNTH_LPFILT2                    0x2304
#define Si446x_SYNTH_LPFILT1                    0x2305
#define Si446x_SYNTH_LPFILT0                    0x2306

#define Si446x_FREQ_CONTROL_INTE                0x4000
#define Si446x_FREQ_CONTROL_FRAC                0x4001
#define Si446x_FREQ_CONTROL_CHANNEL_STEP_SIZE   0x4004
#define Si446x_FREQ_CONTROL_W_SIZE              0x4006
#define Si446x_FREQ_CONTROL_VCOCNT_RX_ADJ       0x4007

#define PKT_SI446X_APRS_CHANNEL                 94
#define PKT_SI446X_SQUELCH_LEVEL                0x4F

#define Si446x_FIFO_SEPARATE_SIZE                64
#define Si446x_FIFO_COMBINED_SIZE               129

#define SI_AFSK_FIFO_MIN_FEEDER_WA_SIZE         1024
#define SI_FSK_FIFO_FEEDER_WA_SIZE              1024

/* AFSK NRZI up-sampler definitions. */
#define PLAYBACK_RATE       13200
#define BAUD_RATE           1200                                    /* APRS AFSK baudrate */
#define SAMPLES_PER_BAUD    (PLAYBACK_RATE / BAUD_RATE)             /* Samples per baud (13200Hz / 1200baud = 11samp/baud) */
#define PHASE_DELTA_1200    (((2 * 1200) << 16) / PLAYBACK_RATE)    /* Delta-phase per sample for 1200Hz tone */
#define PHASE_DELTA_2200    (((2 * 2200) << 16) / PLAYBACK_RATE)    /* Delta-phase per sample for 2200Hz tone */

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

typedef enum radioMode {
  RADIO_RX,
  RADIO_TX,
  RADIO_CCA
} radio_mode_t;

typedef struct {
  uint32_t phase_delta;            // 1200/2200 for standard AX.25
  uint32_t phase;                  // Fixed point 9.7 (2PI = TABLE_SIZE)
  uint32_t packet_pos;             // Next bit to be sent out
  uint32_t current_sample_in_baud; // 1 bit = SAMPLES_PER_BAUD samples
  uint8_t current_byte;
} up_iterator_t;

// Public methods

int16_t Si446x_getLastTemperature(radio_unit_t radio);
void Si446x_shutdown(radio_unit_t radio);
void Si446x_sendAFSK(packet_t pp);
void Si446x_send2FSK(packet_t pp);

void Si446x_disableReceive(radio_unit_t radio);
void Si446x_stopDecoder(void);
bool Si4464_resumeReceive(radio_unit_t radio,
                          radio_freq_t rx_frequency,
                          channel_hz_t rx_step,
                          radio_ch_t rx_chan,
                          radio_squelch_t rx_rssi,
                          mod_t rx_mod);
bool Si446x_receiveNoLock(radio_unit_t radio,
                          radio_freq_t rx_frequency,
                          channel_hz_t rx_step,
                          radio_ch_t chan,
                          radio_squelch_t rssi,
                          mod_t mod);
void Si446x_lockRadio(radio_mode_t mode);
void Si446x_unlockRadio(radio_mode_t mode);
void Si446x_lockRadioByCamera(void);
void Si446x_unlockRadioByCamera(void);
void Si446x_conditional_init(radio_unit_t radio);
bool Si446x_setBandParameters(radio_unit_t radio,
                              radio_freq_t freq,
                              channel_hz_t step);

/* TODO: Forward need for declaration. */
radio_freq_t pktComputeOperatingFrequency(radio_freq_t base_freq,
                                          channel_hz_t step,
                                          radio_ch_t chan);

static inline bool Si446x_isFrequencyInBand(radio_unit_t radio,
                                            radio_freq_t base_freq,
                                            channel_hz_t step,
                                            radio_ch_t chan) {
  (void)radio;

  radio_freq_t freq = pktComputeOperatingFrequency(base_freq,
                                            step,
                                            chan);

  /* TODO: Lookup frequency band table for radio. */
  return (Si446x_MIN_FREQ <= freq && freq < Si446x_MAX_FREQ);
}

extern void pktReleaseOutgoingBuffer(packet_t pp);

static inline void Si446x_releaseSendObject(packet_t pp) {
#if USE_NEW_PKT_TX_ALLOC == TRUE
  pktReleaseOutgoingBuffer(pp);
#else
  ax25_delete(pp);
#endif
}

#endif /* __si446x__H__ */

