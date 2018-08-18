#ifndef __TYPES_H__
#define __TYPES_H__

#include "ch.h"
#include "ax25_pad.h"
#include "ublox.h"

#define IS_AFSK(m)	((m) == MOD_AFSK)
#define IS_2FSK(m)	((m) == MOD_2FSK_9k6 || (m) == MOD_2FSK_19k2 || (m) == MOD_2FSK_38k4 || (m) == MOD_2FSK_57k6 \
					 || (m) == MOD_2FSK_76k8 || (m) == MOD_2FSK_96k || (m) == MOD_2FSK_115k2)

typedef enum {
	FREQ_INVALID = 0,
	FREQ_APRS_GEOFENCE,  /* Geofencing frequency (144.8 default). */
	FREQ_SCAN,           /* Frequency last found in RX scan. - TBI */
	FREQ_RX_APRS,        /* Active RX frequency - fall back to DYNAMIC. */
	FREQ_RX_CMDC,        /* Frequency used for command and control. TBI */
	FREQ_DEFAULT,        /* Default frequency specified in configuration */
	FREQ_CODES_END
} freq_codes_t;

#define FREQ_INVALID   0
#define FREQ_GEOFENCE  1 /* Geofencing frequency (144.8 default). */
#define FREQ_SCAN      2 /* Frequency based on band base + channel scan. */
#define FREQ_RX_APRS   3 /* Active RX frequency - fall back to DYNAMIC. */
#define FREQ_RX_CMDC   4 /* Frequency used for command and control. TBI */
#define FREQ_DEFAULT   5 /* Default frequency specified in configuration */
#define FREQ_CODES_END 6

#define CYCLE_CONTINUOUSLY	0

#define TYPE_NULL			0
#define TYPE_INT			1
#define TYPE_TIME			2
#define TYPE_STR			3

typedef enum {
	SLEEP_DISABLED = 0,
	SLEEP_WHEN_VBAT_BELOW_THRES,
	SLEEP_WHEN_VSOL_BELOW_THRES,
	SLEEP_WHEN_VBAT_ABOVE_THRES,
	SLEEP_WHEN_VSOL_ABOVE_THRES,
	SLEEP_WHEN_DISCHARGING,
	SLEEP_WHEN_CHARGING
} sleep_type_t;

typedef struct {
	sleep_type_t    type;
	volt_level_t    vbat_thres;
	volt_level_t    vsol_thres;
} sleep_conf_t;

typedef enum { // Modulation type
    MOD_NONE,
	MOD_AFSK,
	MOD_2FSK_9k6,
	MOD_2FSK_19k2,
	MOD_2FSK_38k4,
	MOD_2FSK_57k6,
	MOD_2FSK_76k8,
	MOD_2FSK_96k,
	MOD_2FSK_115k2
} radio_mod_t;

typedef enum {
	RES_NONE = 0,
	RES_QQVGA,
	RES_QVGA,
	RES_VGA,
	RES_VGA_ZOOMED,
	RES_XGA,
	RES_UXGA,
	RES_MAX
} resolution_t;

typedef struct {
  radio_pwr_t       pwr;
  radio_freq_t      freq;
  radio_mod_t             mod;
  union {
    radio_squelch_t   cca;
    radio_squelch_t   rssi;
  };
} radio_conf_t;

typedef struct {
  radio_pwr_t       pwr;
  radio_freq_t      freq;
  radio_mod_t             mod;
  link_speed_t      speed;
  radio_squelch_t   cca;
} radio_tx_conf_t; // Radio / Modulation

typedef struct {
  radio_freq_t      freq;
  radio_mod_t             mod;
  link_speed_t      speed;
  radio_squelch_t   rssi;
} radio_rx_conf_t; // Radio / Modulation

typedef struct {
  bool              active;
  sysinterval_t     init_delay;
  sysinterval_t     send_spacing;
  sleep_conf_t      sleep_conf;
  sysinterval_t     cycle;				// Cycle time (0: continuously)
  sysinterval_t     duration;
} thread_conf_t; // Thread

typedef struct {
  bool              active;
  sysinterval_t     init_delay;
  //sysinterval_t     send_spacing;
  sleep_conf_t      sleep_conf;
  sysinterval_t     cycle;              // Cycle time (0: continuously)
  sysinterval_t     duration;
  sysinterval_t		gps_wait;			// Maximum time to wait for a fix
  // Default lat, lon and alt when fixed is enabled
  bool              fixed;
  gps_coord_t       lat;
  gps_coord_t       lon;
  gps_alt_t         alt;
} telem_svc_conf_t; // Thread

typedef struct {
  telem_svc_conf_t  beacon;
  radio_tx_conf_t   radio_conf;
  // Protocol
  char              call[AX25_MAX_ADDR_LEN];
  char              path[16];
  aprs_sym_t        symbol;
  bool              aprs_msg;
  bool              run_once;
  sysinterval_t     gps_wait;
} bcn_app_conf_t;

typedef struct {
  thread_conf_t     svc_conf;
  radio_tx_conf_t   radio_conf;
  bool              redundantTx;
  // Protocol
  char              call[AX25_MAX_ADDR_LEN];
  char              path[16];
  resolution_t      res;					// Picture resolution
  uint8_t           quality;				// SSDV Quality ranging from 0-7
  bool              flip;                   // 180 image rotation
  uint32_t          buf_size;		    	// SRAM buffer size for the picture
} img_app_conf_t;

typedef struct {
  thread_conf_t     svc_conf;
  radio_tx_conf_t   radio_conf;
  // Protocol
  char              call[AX25_MAX_ADDR_LEN];
  char              path[16];
  uint8_t           density;				// Density of log points being sent out in 1/x (value 10 => 10%)
} log_app_conf_t;

typedef struct {
  thread_conf_t     svc_conf;
  radio_rx_conf_t   radio_conf;
  aprs_sym_t        symbol;
  // Protocol
  char              call[AX25_MAX_ADDR_LEN];
} thd_rx_conf_t;

typedef struct {
  radio_tx_conf_t   radio_conf;
  // Protocol
  char              call[AX25_MAX_ADDR_LEN];
  char              path[16];
  aprs_sym_t        symbol;
} thd_tx_conf_t;

typedef struct {
  radio_tx_conf_t   radio_conf;
  // Protocol
  char              call[AX25_MAX_ADDR_LEN];
  char              path[16];
  aprs_sym_t        symbol;
  bool              enabled;
} thd_base_conf_t;

/* APRS configuration. */
typedef struct {
  thd_rx_conf_t     rx;
  bool              aprs_msg;
  bool              digi;
  bcn_app_conf_t    tx;
} thd_aprs_conf_t;

typedef struct {
  bcn_app_conf_t	pos_pri;				// Primary position thread configuration
  bcn_app_conf_t	pos_sec;				// Secondary position thread configuration

  img_app_conf_t	img_pri;				// Primary image thread configuration
  img_app_conf_t	img_sec;				// Secondary image thread configuration

  log_app_conf_t	log;					// Log transmission configuration
  thd_aprs_conf_t   aprs;

  bool			    keep_cam_switched_on;	// Keep camera switched on and initialized, this makes image capturing faster but takes a lot of power over long time

  volt_level_t      gps_on_vbat;			// Battery voltage threshold at which GPS is switched on
  volt_level_t      gps_off_vbat;			// Battery voltage threshold at which GPS is switched off
  volt_level_t      gps_onper_vbat;			// Battery voltage threshold at which GPS is kept switched on all time. This value must be larger
                                          // When gps_on_vbat and gps_off_vbat otherwise this value has no effect. Value 0 disables this feature
  uint32_t          gps_pressure;           // Air pressure below which GPS is switched to airborne mode
  gps_hp_model_t    gps_low_alt;             // Model to use when air pressure is above gps_pa_threshold
  gps_lp_model_t    gps_high_alt;           // Model to use when air pressure is below gps_pa_threshold

  //APRS global
  sysinterval_t     tel_enc_cycle;          // Cycle for sending of telemetry config headers
  radio_freq_t      freq;                   // Default APRS frequency if geolocation not available
  // Base station call sign for receipt of tracker initiated sends
  // These are sends by the tracker which are not in response to a query.
  thd_base_conf_t   base;

  uint32_t          magic;                  // Key that indicates if the flash is loaded or has been updated
  uint16_t          crc;                    // CRC to verify content
} conf_t;

typedef struct {
  uint8_t           type;
  char              name[64];
  size_t            size;
  void              *ptr;
} conf_command_t;

#endif /* __TYPES_H__ */

