#ifndef __TYPES_H__
#define __TYPES_H__

#include "ch.h"
#include "ax25_pad.h"

#define FREQ_APRS_DYNAMIC	0
#define CYCLE_CONTINUOUSLY	0

#define TYPE_NULL			0
#define TYPE_INT			1
#define TYPE_TIME			2
#define TYPE_STR			3

typedef enum {
	SLEEP_DISABLED,
	SLEEP_WHEN_VBAT_BELOW_THRES,
	SLEEP_WHEN_VSOL_BELOW_THRES,
	SLEEP_WHEN_VBAT_ABOVE_THRES,
	SLEEP_WHEN_VSOL_ABOVE_THRES,
	SLEEP_WHEN_DISCHARGING,
	SLEEP_WHEN_CHARGING
} sleep_type_t;

typedef struct {
	sleep_type_t type;
	uint16_t vbat_thres;
	uint16_t vsol_thres;
} sleep_conf_t;

typedef enum { // Modulation type
    MOD_NONE,
	MOD_AFSK,
	MOD_2FSK
} mod_t;

typedef enum {
	RES_QQVGA,
	RES_QVGA,
	RES_VGA,
	RES_VGA_ZOOMED,
	RES_XGA,
	RES_UXGA,
	RES_MAX
} resolution_t;

typedef struct {
	int8_t				pwr;
	uint32_t			freq;	// 0: APRS region frequency (determined by geofencing), f>0 static frequency
	uint16_t            step;
	uint8_t             chan;
	mod_t				mod;
	uint16_t			preamble;
	uint32_t			speed;
	bool                redundantTx;
} radio_conf_t; // Radio / Modulation

typedef struct {
	bool				active;
	sysinterval_t		init_delay;
	sysinterval_t		packet_spacing;
	sleep_conf_t		sleep_conf;
	sysinterval_t		cycle;				// Cycle time (0: continously)
} thread_conf_t; // Thread

typedef struct {
	thread_conf_t thread_conf;
	radio_conf_t radio_conf;

	// Protocol
	char call[AX25_MAX_ADDR_LEN];
	char path[16];
	uint16_t symbol;

	sysinterval_t tel_enc_cycle;
} thd_pos_conf_t;

typedef struct {
	thread_conf_t thread_conf;
	radio_conf_t radio_conf;

	// Protocol
	char call[AX25_MAX_ADDR_LEN];
	char path[16];

	resolution_t res;						// Picture resolution
	uint8_t quality;						// SSDV Quality ranging from 0-7
	uint32_t buf_size;						// SRAM buffer size for the picture
} thd_img_conf_t;

typedef struct {
	thread_conf_t thread_conf;
	radio_conf_t radio_conf;

	// Protocol
	char call[AX25_MAX_ADDR_LEN];
	char path[16];

	uint8_t density;						// Density of log points being sent out in 1/x (value 10 => 10%)
} thd_log_conf_t;

typedef struct {
	radio_conf_t radio_conf;
	thread_conf_t thread_conf;

	// Protocol
	char call[AX25_MAX_ADDR_LEN];
	char path[16];
	uint16_t symbol;
} thd_rx_conf_t;

typedef struct {
	thd_pos_conf_t	pos_pri;				// Primary position thread configuration
	thd_pos_conf_t	pos_sec;				// Secondary position thread configuration

	thd_img_conf_t	img_pri;				// Primary image thread configuration
	thd_img_conf_t	img_sec;				// Secondary image thread configuration

	thd_log_conf_t	log;					// Log transmission configuration
	thd_rx_conf_t	rx;						// Receiver configuration

	uint8_t			rssi;					// Squelch for reception

	bool			dig_active;				// Digipeater active flag

	bool			keep_cam_switched_on;	// Keep camera switched on and initialized, this makes image capturing faster but takes a lot of power over long time

	uint16_t		gps_on_vbat;			// Battery voltage threshold at which GPS is switched on
	uint16_t		gps_off_vbat;			// Battery voltage threshold at which GPS is switched off
	uint16_t		gps_onper_vbat;			// Battery voltage threshold at which GPS is kept switched on all time. This value must be larger
											// than gps_on_vbat and gps_off_vbat otherwise this value has no effect. Value 0 disables this feature

	uint32_t magic;
} conf_t;

typedef struct {
	uint8_t type;
	char name[64];
	size_t size;
	void *ptr;
} conf_command_t;

#endif

