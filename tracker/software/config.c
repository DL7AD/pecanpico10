/**
  * Put your configuration settings here. See description of all fields in types.h
  */

#include "config.h"
#include "aprs.h"

conf_t conf_sram;

const conf_t conf_flash_default = {
	// Primary position transmission thread
	.pos_pri = {
		.thread_conf = {
			.active			= false,
			.cycle			= TIME_S2I(300),
			.init_delay		= TIME_S2I(5)
		},
		.radio_conf = {
			.pwr			= 0x7F,
			.freq			= 144000000,
			.step           = 12500,
			.chan           = 94,
			.mod			= MOD_AFSK,
			.preamble		= 200
		},

		.call				= "VK2GJ-12",
		.path				= "WIDE2-1",
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
			.step           = 0,
			.chan           = 0,
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
			.active			= true,
			.cycle			= CYCLE_CONTINUOUSLY,
			.init_delay		= TIME_S2I(20),
			.packet_spacing	= TIME_S2I(0)
		},
		.radio_conf = {
			.pwr			= 0x7F,
            .freq           = 144000000,
            .step           = 12500,
            .chan           = 64,
			.mod			= MOD_2FSK,
			.preamble		= 200,
			.redundantTx	= false
		},

		.call				= "VK2GJ-15",
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
			.step           = 0,
			.chan           = 0,
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
			.cycle			= TIME_S2I(30),
			.init_delay		= TIME_S2I(5)
		},
		.radio_conf = {
			.pwr			= 0x7F,
			.freq			= FREQ_APRS_DYNAMIC,
			.step           = 0,
			.chan           = 0,
			.mod			= MOD_AFSK,
			.preamble		= 200
		},

		.call				= "DL7AD-13",
		.path				= "WIDE1-1",
		.density			= 10
	},
	.rx = {
		.thread_conf = {
			.active			= true
		},
		.radio_conf = {
			.pwr			= 0x7F,
			.freq			= 144000000,
			.step           = 12500,
			.chan           = 94,
			.mod			= MOD_AFSK,
			.preamble		= 200
		},

		.call				= "VK2GJ-4",
		.path				= "WIDE2-1",
		.symbol				= SYM_DIGIPEATER
	},

	.rssi					= 0x3F,

	.dig_active				= true,

	.keep_cam_switched_on	= false,

	.gps_on_vbat			= 1000,
	.gps_off_vbat			= 1000,
	.gps_onper_vbat			= 1000,

	.magic					= CONFIG_MAGIC_DEFAULT // Do not remove. This is the activation bit.
};
