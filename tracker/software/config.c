// Put your configuration settings here

#include "config.h"
#include "aprs.h"

conf_t conf_sram;

const conf_t conf_flash __attribute__((section(".flash1.__at_0x0"))) = {
	// Primary position transmission thread
	.pos_pri = {
		.thread_conf = {
			.active			= true,
			.cycle			= TIME_S2I(120),
			.init_delay		= TIME_S2I(5)
		},
		.radio_conf = {
			.pwr			= 0x7F,
			.freq			= FREQ_APRS_DYNAMIC,
			.mod			= MOD_AFSK,
			.preamble		= 200
		},

		.call				= "DL7AD-13",
		.path				= "WIDE1-1",
		.symbol				= SYM_DIGIPEATER,

		.tel_enc_cycle		= TIME_S2I(10800),
	},

	// Secondary position transmission thread
	.pos_sec = {
		.thread_conf = {
			.active			= false,
			.cycle			= TIME_S2I(120)
		},
		.radio_conf = {
			.pwr			= 0x7F,
			.freq			= FREQ_APRS_DYNAMIC,
			.mod			= MOD_AFSK,
			.preamble		= 200
		},

		.call				= "DL7AD-14",
		.path				= "WIDE1-1",
		.symbol				= SYM_BALLOON,

		.tel_enc_cycle		= TIME_S2I(10800),
	},

	// Primary image transmission thread
	.img_pri = {
		.thread_conf = {
			.active			= false,
			.cycle			= CYCLE_CONTINUOUSLY,
			.init_delay		= TIME_S2I(5),
			.packet_spacing	= TIME_S2I(30)
		},
		.radio_conf = {
			.pwr			= 0x7F,
			.freq			= FREQ_APRS_DYNAMIC,
			.mod			= MOD_AFSK,
			.preamble		= 200,
			.redundantTx	= true
		},

		.call				= "DL7AD-13",
		.path				= "DB0BLO",

		.res				= RES_VGA,
		.quality			= 4,
		.buf_size			= 64*1024
	},

	// Secondary image transmission thread
	.img_sec = {
		.thread_conf = {
			.active			= false,
			.cycle			= CYCLE_CONTINUOUSLY
		},
		.radio_conf = {
			.pwr			= 0x7F,
			.freq			= FREQ_APRS_DYNAMIC,
			.mod			= MOD_AFSK,
			.preamble		= 200
		},

		.call				= "DL7AD-14",
		.path				= "",

		.res				= RES_VGA,
		.quality			= 4,
		.buf_size			= 64*1024
	},

	// Log transmission thread
	.log = {
		.thread_conf = {
			.active			= false,
			.cycle			= TIME_S2I(5),
			.init_delay		= TIME_S2I(5)
		},
		.radio_conf = {
			.pwr			= 0x7F,
			.freq			= FREQ_APRS_DYNAMIC,
			.mod			= MOD_AFSK,
			.preamble		= 200
		},

		.call				= "DL7AD-13",
		.path				= "WIDE1-1",
	},
	.rx = {
		.thread_conf = {
			.active			= false
		},
		.radio_conf = {
			.pwr			= 0x7F,
			.freq			= FREQ_APRS_DYNAMIC,
			.mod			= MOD_AFSK,
			.preamble		= 200
		},

		.call				= "DL7AD-13",
		.path				= "WIDE1-1",
		.symbol				= SYM_DIGIPEATER
	},

	.rssi					= 0x3F,

	.dig_active				= false,

	.keep_cam_switched_on	= false,

	.gps_on_vbat			= 1000,
	.gps_off_vbat			= 1000,
	.gps_onper_vbat			= 1000
};
