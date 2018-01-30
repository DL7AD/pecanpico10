#ifndef __TYPES_H__
#define __TYPES_H__

typedef struct {
	char callsign[10];			// APRS callsign
	uint16_t symbol;			// APRS symbol
	char path[16];				// APRS path
	uint16_t preamble;			// Preamble in milliseconds
	uint16_t tel_enc_cycle;		// Telemetry encoding cycle in seconds
	char tel_comment[64];		// Telemetry comment
} aprs_conf_t;

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

typedef struct {
	uint8_t dummy; // Not used yet
} afsk_conf_t;

typedef struct {
	uint32_t speed;
} fsk_conf_t;

typedef enum {
	FREQ_STATIC,			// Fixed frequency
	FREQ_APRS_REGION		// APRS region dependent (it changes it frequency automatically depending on which APRS frequency is used in this region)
} freq_type_t;

typedef struct {
	freq_type_t type;
	uint32_t hz;
} freq_conf_t;

typedef enum { // Modulation type
	MOD_NOT_SET,
	MOD_2FSK,
	MOD_AFSK
} mod_t;

typedef struct { // Radio message type
	uint8_t* 		buffer;			// Message (data)
	uint32_t		bin_len;		// Binary length (it bits)
	uint8_t			power;			// Power in dBm
	mod_t			mod;			// Modulation

	freq_conf_t*	freq;			// Frequency

	afsk_conf_t*	afsk_conf;		// AFSK config
	fsk_conf_t*		fsk_conf;		// 2FSK config
} radioMSG_t;

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
	char callsign[7];		// Callsign (or stream identifier)
	resolution_t res;		// Camera resolution
	uint8_t quality;		// JPEG quality
	uint8_t *ram_buffer;	// Camera Buffer
	uint32_t ram_size;		// Size of buffer
	uint32_t size_sampled;	// Actual image data size (do not set in config)
} ssdv_conf_t;

typedef enum {
	TRIG_ONCE,				// Trigger once and never again (e.g. transmit specific position packet only at startup)
	TRIG_NEW_POINT,			// Triggered when new track point available
	TRIG_TIMEOUT,			// Triggered by timeout (e.g. trasmit position every 120sec)
	TRIG_CONTINUOUSLY		// Continue continuously (e.g. send new image once old image sent completely)
} trigger_type_t;

typedef struct {
	trigger_type_t type;	// Trigger type
	uint32_t timeout;		// Timeout in seconds
} trigger_conf_t;

typedef struct {
	char				name[10];

	// Radio
	int8_t				power;
	freq_conf_t			frequency;
	mod_t				modulation;

	// Timing
	uint32_t			init_delay;
	uint32_t			packet_spacing;
	sleep_conf_t		sleep_conf;
	trigger_conf_t		trigger;

	// Modulation
	union {
		afsk_conf_t		afsk_conf;
		fsk_conf_t		fsk_conf;
	};

	// Protocol
	aprs_conf_t			aprs_conf;
	ssdv_conf_t			ssdv_conf;

	// Transmission
	bool redundantTx;					// Redundand packet transmission (APRS only)

	// Watchdog
	sysinterval_t		wdg_timeout;	// Time at which watchdog will reset the STM32, 0 inactive
} module_conf_t;

#endif

