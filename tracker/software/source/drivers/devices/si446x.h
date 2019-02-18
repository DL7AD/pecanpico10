
/**
 * @file    si446x.h
 * @brief   Silicon Labs radio driver.
 *
 * @addtogroup radio
 * @details Radio driver definitions.
 * @{
 */

#ifndef SI446x_H
#define SI446x_H

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

#define Si446x_4463_USE_446X_COMPATABILITY      TRUE
#define Si446x_USE_AFSK_LCM_DATA_RATE           FALSE
#define Si446x_USE_NB_RECEIVE_FILTER            TRUE
#define Si446x_USE_TRANSMIT_TIMEOUT             FALSE
#define Si446x_USE_SEMAPHORE_INTERRUPT_SYNC     TRUE
#define Si446x_USE_PACKET_END_INTERRUPT         TRUE
#define Si446x_USE_FIFO_THRESHOLD_INTERRUPT     TRUE
#define Si446x_USE_STATE_CHANGE_INTERRUPT       TRUE
#define Si446x_USE_COMMON_TX_THREAD             TRUE
#if Si446x_USE_PACKET_END_INTERRUPT == TRUE                                 \
  || Si446x_USE_FIFO_THRESHOLD_INTERRUPT == TRUE                            \
  || Si446x_USE_STATE_CHANGE_INTERRUPT == TRUE
#define Si446x_USE_INTERRUPTS                   TRUE
#else
#define Si446x_USE_INTERRUPTS                   FALSE
#endif

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

#define SI446X_EVT_TX_TIMEOUT                   EVENT_MASK(0)

#define Si446x_MODEM_RSSI_COMP_VALUE            0x40
#define SI446X_TX_CCA_TIMEOUT                   5
#define SI446X_TRANSMIT_TIMEOUT                 10
#define SI446X_TX_FIFO_TIMEOUT                  5
/*
 * Si446x commands.
 * The reply size is reply data only (w/o CTS).
 */
#define Si446x_NOP_CMD                          0x00
#define Si446x_NOP_REPLY_SIZE                   0
#define Si446x_GET_PART_INFO_CMD                0x01
#define Si446x_GET_PART_INFO_REPLY_SIZE         8
#define Si446x_POWER_UP_CMD                     0x02
#define Si446x_POWER_UP_CMD_REPLY_SIZE          0
#define Si446x_GET_FUNC_INFO_CMD                0x10
#define Si446x_GET_FUNC_INFO_REPLY_SIZE         6
#define Si446x_SET_PROPERTY_CMD                 0x11
#define Si446x_GET_PROPERTY_CMD                 0x12
#define Si446x_GPIO_PIN_CFG_CMD                 0x13
#define Si446x_GPIO_PIN_CFG_REPLY_SIZE          7
#define Si446x_GET_ADC_READING_CMD              0x14
#define Si446x_GET_ADC_READING_REPLY_SIZE       6
#define Si446x_GET_ADC_READING_TEMP_MASK        0x10
#define Si446x_GET_ADC_READING_BATT_MASK        0x08
#define Si446x_GET_ADC_READING_TEMP_OFFSET      0x04
#define Si446x_GET_ADC_READING_BATT_OFFSET      0x02
#define Si446x_FIFO_INFO_CMD                    0x15
#define Si446x_FIFO_INFO_REPLY_SIZE             2
#define Si446x_PACKET_INFO_CMD                  0x16
#define Si446x_PACKET_INFO_REPLY_SIZE           2
#define Si446x_GET_INT_STATUS_CMD               0x20
#define Si446x_GET_INT_STATUS_REPLY_SIZE        8
#define Si446x_GET_PH_STATUS_CMD                0x21
#define Si446x_GET_PH_STATUS_REPLY_SIZE         2
#define Si446x_GET_MODEM_STATUS_CMD             0x22
#define Si446x_GET_MODEM_STATUS_REPLY_SIZE      8
#define Si446x_GET_CHIP_STATUS_CMD              0x23
#define Si446x_GET_CHIP_STATUS_REPLY_SIZE       3
#define Si446x_START_TX_CMD                     0x31
#define Si446x_START_TX_REPLY_SIZE              0
#define Si446x_START_RX_CMD                     0x32
#define Si446x_START_RX_REPLY_SIZE              0
#define Si446x_REQUEST_DEVICE_STATE_CMD         0x33
#define Si446x_REQUEST_DEVICE_STATE_REPLY_SIZE  2
#define Si446x_CHANGE_STATE_CMD                 0x34
#define Si446x_CHANGE_STATE_REPLY_SIZE          0
#define Si446x_RX_HOP_CMD                       0x36
#define Si446x_RX_HOP_REPLY_SIZE                0
#define Si446x_TX_HOP_CMD                       0x37
#define Si446x_TX_HOP_REPLY_SIZE                0
#define Si446x_READ_CMD_BUFF_CMD                0x44
#define Si446x_READ_FRR_A_CMD                   0x50
#define Si446x_WRITE_TX_FIFO_CMD                0x66
#define Si446x_READ_RX_FIFO_CMD                 0x77

/* Defined response values. */
#define Si446x_COMMAND_CTS                      0xFF

/*
 * Property group commands, indexes and masks.
 * Format is 0xGGNN (GG = group, NN = number).
 */
#define Si446x_GLOBAL_XO_TUNE                   0x0000
#define Si446x_GLOBAL_CLK_CFG                   0x0001
#define Si446x_GLOBAL_CONFIG                    0x0003

#define Si446x_INT_CTL_ENABLE                   0x0100
#define Si446x_INT_CTL_PH_REG_INDEX             0x00
#define Si446x_INT_CTL_MODEM_REG_INDEX          0x01
#define Si446x_INT_CTL_CHIP_REG_INDEX           0x02
#define Si446x_INT_CTL_PH_ENABLE                0x0101
#define Si446x_INT_CTL_PH_RX_FIFO_MASK          (1 << 0)
#define Si446x_INT_CTL_PH_TX_FIFO_THRESH_MASK   (1 << 1)
#define Si446x_INT_CTL_PH_PACKET_SENT_MASK      (1 << 5)
#define Si446x_INT_CTL_MODEM_ENABLE             0x0102
#define Si446x_INT_CTL_CHIP_ENABLE              0x0103
#define Si446x_INT_CTL_CHIP_STATE_CHANGE_MASK   (1 << 4)

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
#define Si446x_PKT_LEN                          0x1208
#define Si446x_PKT_LEN_FIELD_SOURCE             0x1209
#define Si446x_PKT_LEN_ADJUST                   0x120A
#define Si446x_PKT_TX_THRESHOLD                 0x120B

#define Si446x_MODEM_MOD_TYPE                   0x2000
#define Si446x_MODEM_MAP_CONTROL                0x2001
#define Si446x_MODEM_DSM_CTRL                   0x2002
#define Si446x_MODEM_DATA_RATE                  0x2003
#define Si446x_MODEM_TX_NCO_MODE                0x2006
#define Si446x_MODEM_FREQ_DEV                   0x200A
#define Si446x_MODEM_TX_FILTER_COEFF_8          0x200F
#define Si446x_MODEM_TX_RAMP_DELAY              0x2018
#define Si446x_MODEM_MDM_CTRL                   0x2019
#define Si446x_MODEM_IF_CONTROL                 0x201A
#define Si446x_MODEM_IF_FREQ                    0x201B
#define Si446x_MODEM_DECIMATION_CFG1            0x201E
#define Si446x_MODEM_DECIMATION_CFG0            0x201F
#define Si446x_MODEM_DECIMATION_CFG2            0x2020
#define Si446x_MODEM_IFPKD_THRESHOLDS           0x2021
#define Si446x_MODEM_BCR_OSR                    0x2022
#define Si446x_MODEM_BCR_NCO_OFFSET             0x2024
#define Si446x_MODEM_BCR_GAIN                   0x2027
#define Si446x_MODEM_BCR_GEAR                   0x2029
#define Si446x_MODEM_BCR_MISC                   0x202A
#define Si446x_MODEM_BCR_MISC1                  0x202A
#define Si446x_MODEM_BCR_MISC0                  0x202B
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
#define Si446x_MODEM_OOK_BLOPK                  0x2041
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
#define Si446x_MODEM_RAW_SEARCH2                0x2050
#define Si446x_MODEM_CLKGEN_BAND                0x2051
#define Si446x_MODEM_SPIKE_DET                  0x2054
#define Si446x_MODEM_ONE_SHOT_AFC               0x2055
#define Si446x_MODEM_RSSI_MUTE                  0x2057
#define Si446x_MODEM_DSA_CTRL1                  0x205B
#define Si446x_MODEM_DSA_CTRL2                  0x205C
#define Si446x_MODEM_DSA_QUAL                   0x205D
#define Si446x_MODEM_DSA_RSSI                   0x205E
#define Si446x_MODEM_DSA_MISC                   0x205F

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

/* RSSI setting which bypasses CCA checks. */
#define PKT_SI446X_NO_CCA_RSSI                  0xFF

#define Si446x_FIFO_SEPARATE_SIZE                64
#define Si446x_FIFO_COMBINED_SIZE               129

/* Transmit threads working areas. */
#define SI_AFSK_FIFO_MIN_FEEDER_WA_SIZE         (1 * 1024)
#define SI_FSK_FIFO_FEEDER_WA_SIZE              (1 * 1024)

#define SI_NIRQ_HANDLER_WA_SIZE                 (512)

/*
 *  AFSK NRZI re-sampler definitions.
 */
#define RESAMPLE_RATE       13200
/* APRS AFSK baudrate */
#define SYMBOL_RATE         1200
/* Samples per baud (13200Hz / 1200baud = 11 samples/baud) */
#define SAMPLES_PER_AFSK_SYMBOL    (RESAMPLE_RATE / SYMBOL_RATE)
/* Delta-phase per sample for 1200Hz tone */
#define PHASE_DELTA_1200    (((2 * 1200) << 16) / RESAMPLE_RATE)
/* Delta-phase per sample for 2200Hz tone */
#define PHASE_DELTA_2200    (((2 * 2200) << 16) / RESAMPLE_RATE)

 /* Frequency offset corrected oscillator frequency */
#define Si446x_CCLK (Si446x_CLK + Si446x_CLK_ERROR)

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

#define is_part_Si4463(part) (part == 0x4463)

#define is_Si4463_patch_required(part, rom)                                  \
	(is_part_Si4463(part) && rom == 0x6)

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

typedef enum {
  /* Si4464 States. */
  Si446x_REMAIN       =          0,
  Si446x_SLEEP        =          1,
  Si446x_STANDBY      =          1,
  Si446x_SPI_ACTIVE   =          2,
  Si446x_READY        =          3,
  Si446x_READY2       =          4,
  Si446x_TX_TUNE      =          5,
  Si446x_RX_TUNE      =          6,
  Si446x_TX           =          7,
  Si446x_RX           =          8
} si446x_state_t;

/* SPI byte stream arguments and replies. */
typedef uint8_t si446x_reply_t;
typedef uint8_t si446x_arg_t;

/* AFSK encoder/up-sampler control object. */
typedef struct reSampler {
  uint32_t  phase_delta;            // 1200/2200 for standard AX.25
  uint32_t  phase;                  // Fixed point 9.7 (2PI = TABLE_SIZE)
  uint32_t  packet_pos;             // Index of next bit to be sent out
  uint8_t   current_sample_in_baud; // 1 AFSK bit = SAMPLES_PER_BAUD tx bits
  uint8_t   current_byte;
  uint8_t   sample_rate;            /**<< Up sample rate. */
  uint16_t  mark_delta;
  uint16_t  space_delta;
} re_sampler_t;

/* Indexes of GPIO parameters in 446x GPIO command. */
typedef enum {
  Si446x_GPIO0  = 0,
  Si446x_GPIO1  = 1,
  Si446x_GPIO2  = 2,
  Si446x_GPIO3  = 3,
  Si446x_NIRQ   = 4,
  Si446x_SDO    = 5,
  Si446x_CFG    = 6
} si446x_gpix_t;

typedef struct iolineRef {
  const ioline_t  *line;
  const iomode_t  mode;
} ioline_ref_t;

/* Configuration of GPIO for a radio. */
typedef struct Si446x_GPIO {
  si446x_arg_t        gpio0;
  si446x_arg_t        gpio1;
  si446x_arg_t        gpio2;
  si446x_arg_t        gpio3;
  si446x_arg_t        nirq;
  si446x_arg_t        sdo;
  si446x_arg_t        cfg;
} __attribute__((packed)) si446x_gpio_t;

/* MCU IO configuration for a specific radio. */
typedef struct Si446x_MCUCFG {
  /* IO line connections from radio GPIO to MCU GPIO. */
  const ioline_t	  gpio0;
  const ioline_t      gpio1;
  const ioline_t      gpio2;
  const ioline_t      gpio3;
  const ioline_t      nirq;
  const ioline_t	  sdn;
  const ioline_t	  cs;
  /* SPI bus this radio is connected to. */
  SPIDriver*	      spi;
  /* Radio GPIO setting after init. */
  struct {
    si446x_gpio_t     gpio;
  } init;
  /* Configuration of global IRQ dispatcher. */
  struct {
    si446x_gpio_t     gpio;   /**< Radio GPIO config for IRQ mode. */
    ioline_ref_t      nirq;
  } xirq;
  /* CCA detection only. */
  struct {
    si446x_gpio_t     gpio;
    ioline_ref_t      cca;
  } rcca;
  /* AFSK receive settings. */
  struct {
    si446x_gpio_t     gpio;   /**< Radio GPIO config for this mode. */
    ioline_ref_t      pwm;    /**< PWM (RAW_RX) from radio to MCU GPIO. */
    ioline_ref_t      cca;    /**< CCA (CCA) from radio to MCU GPIO. */
    ICUDriver*        icu;    /**< ICU (TIM) for capturing radio PWM. */
    ICUConfig         cfg;
  } rafsk;
  /* AFSK transmit settings. */
  struct {
    si446x_gpio_t     gpio;
    ioline_ref_t      cca;
  } tafsk;
  /* 2FSK receive settings. */
  struct {
    si446x_gpio_t     gpio;
    ioline_ref_t      cca;    /**< CCA (CCA) from radio to MCU GPIO. */
  } r2fsk;
  /* 2FSK transmit settings. */
  struct {
    si446x_gpio_t     gpio;
    ioline_ref_t      cca;
  } t2fsk;
} si446x_mcucfg_t;

/* Si446x part info. */
typedef struct {
  si446x_reply_t   info[Si446x_GET_PART_INFO_REPLY_SIZE];
} __attribute__((packed)) si446x_part_t;

/* Si446x func info. */
typedef struct {
  si446x_reply_t   info[Si446x_GET_FUNC_INFO_REPLY_SIZE];
} __attribute__((packed)) si446x_func_t;

/* Si446x full interrupt status. */
typedef struct {
  si446x_reply_t        int_pend;
  si446x_reply_t        int_status;
  si446x_reply_t        ph_pend;
  si446x_reply_t        ph_status;
  si446x_reply_t        modem_pend;
  si446x_reply_t        modem_status;
  si446x_reply_t        chip_pend;
  si446x_reply_t        chip_status;
} __attribute__((packed)) si446x_int_status_t;

/* Data associated with a specific radio. */
typedef struct Si446x_DAT {
  radio_part_t          radio_part;
  radio_rev_t           radio_rom_rev;
  radio_patch_t         radio_patch;
  radio_temp_t          lastTemp;
  radio_clock_t         radio_clock;
#if Si446x_USE_INTERRUPTS
  bool                  nirq_active;
#if Si446x_USE_SEMAPHORE_INTERRUPT_SYNC == TRUE
  binary_semaphore_t    wait_sem;
#else
  thread_t              *irq_dispatch;
  thread_reference_t    wait_thread;
#endif
#endif /* Si446x_USE_INTERRUPTS */
  radio_isr_cb_t        cb;
  si446x_int_status_t   int_status;
} si446x_data_t;

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

typedef struct radioTask radio_task_object_t;

extern void pktReleaseCommonPacketBuffer(packet_t pp);

#ifdef __cplusplus
extern "C" {
#endif
  radio_temp_t Si446x_getLastTemperature(const radio_unit_t radio);
  bool Si446x_radioWakeUp(const radio_unit_t radio);
  void Si446x_radioShutdown(const radio_unit_t radio);
  bool Si446x_radioStandby(const radio_unit_t radio);
  bool Si446x_blocSendAFSK(radio_task_object_t *rto);
  bool Si446x_blocSend2FSK(radio_task_object_t *rto);
  bool Si446x_blocSendCW(radio_task_object_t *rt);
  //void Si446x_disableReceive(radio_unit_t radio);
  bool Si4464_enableReceive(const radio_unit_t radio,
                            radio_freq_hz_t rx_frequency,
                            radio_chan_hz_t rx_step,
                            radio_ch_t rx_chan,
                            radio_squelch_t rx_rssi,
                            radio_mod_t rx_mod);
  void Si446x_lockRadio(const radio_mode_t mode);
  void Si446x_unlockRadio(const radio_mode_t mode);
  bool Si446x_driverInit(radio_unit_t radio);
  radio_signal_t Si446x_getCurrentRSSI(const radio_unit_t radio);
  ICUDriver *Si446x_attachPWM(const radio_unit_t radio);
  void       Si446x_detachPWM(const radio_unit_t radio);
  void Si446x_enablePWMevents(const radio_unit_t radio,
                                          const radio_mod_t mod,
                                          const palcallback_t cb);
  void Si446x_disablePWMeventsI(const radio_unit_t radio,
                                const radio_mod_t mod);
  uint8_t Si446x_readCCAlineForRX(const radio_unit_t radio, radio_mod_t mod);
  bool Si446x_updateClock(const radio_unit_t, const xtal_osc_t freq);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Driver inline functions.                                                  */
/*===========================================================================*/

static inline void Si446x_releaseSendObject(packet_t pp) {
  pktReleaseCommonPacketBuffer(pp);
}

#endif /* SI446x_H */

/** @} */
