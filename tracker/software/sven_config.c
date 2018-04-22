/**
  * Put your configuration settings here. See description of all fields in types.h
  */

#include "config.h"
#include "aprs.h"

conf_t conf_sram;

const conf_t conf_flash_default = {
	// Primary position node
	.pos_pri = {
		.thread_conf = {
			.active			= true,
			.cycle			= TIME_S2I(60*1),
			.init_delay		= TIME_S2I(5)
		},
		.radio_conf = {
			.pwr			= 0x7F,
			.freq			= FREQ_APRS_RECEIVE,
			.mod			= MOD_AFSK,
            .rssi           = 0x4F,
		},
		// Node identity
		.call				= "DL7AD-11",
		.path				= "WIDE1-1",
		.symbol				= SYM_SMALLAIRCRAFT,
		.aprs_msg           = true,

		.tel_enc_cycle		= TIME_S2I(10800)
	},

	// Secondary position node
	.pos_sec = {
		.thread_conf = {
			.active			= false,
			.cycle			= TIME_S2I(120),
            .init_delay     = TIME_S2I(60)
		},
		.radio_conf = {
			.pwr			= 0x7F,
			.freq			= FREQ_APRS_DYNAMIC,
			.mod			= MOD_AFSK,
            .rssi           = 0x4F
		},
        // Node identity
		.call				= "DL7AD-14",
		.path				= "WIDE1-1",
		.symbol				= SYM_BALLOON,
		.aprs_msg           = true,

		.tel_enc_cycle		= TIME_S2I(10800)
	},

	// Primary image node
	.img_pri = {
		.thread_conf = {
			.active			= true,
			.cycle			= TIME_S2I(0),
			.init_delay		= TIME_S2I(60*3),
			.send_spacing	= TIME_S2I(15)
		},
		.radio_conf = {
			.pwr			= 0x7F,
            .freq           = 144800000,
			.mod			= MOD_AFSK,
			.rssi           = 0x4F,
			.redundantTx	= false
		},
        // Node identity
		.call				= "DL7AD-11",
		.path				= "",

		.res				= RES_QVGA,
		.quality			= 4,
		.buf_size			= 40*1024
	},

	// Secondary image node
	.img_sec = {
		.thread_conf = {
			.active			= false,
            .cycle          = TIME_S2I(60*5),
            .init_delay     = TIME_S2I(60*1),
            .send_spacing   = TIME_S2I(30)
		},
		.radio_conf = {
			.pwr			= 0x7F,
			.freq			= 145175000,
			.mod			= MOD_AFSK,
            .rssi           = 0x4F
		},
        // Node identity
		.call				= "VK2GJ-14",
        .path               = "",

		.res				= RES_QVGA,
		.quality			= 4,
		.buf_size			= 15*1024
	},

	// Log node
	.log = {
		.thread_conf = {
			.active			= false,
			.cycle			= TIME_S2I(10),
			.init_delay		= TIME_S2I(5)
		},
		.radio_conf = {
			.pwr			= 0x7F,
			.freq			= FREQ_APRS_DYNAMIC,
			.mod			= MOD_AFSK,
            .rssi           = 0x4F
		},
        // Node identity
		.call				= "VK2GJ-13",
		.path				= "WIDE1-1",
		.density			= 10
	},

	// APRS node
	.aprs = {
      .thread_conf = {
          .active       = true,
          .init_delay   = TIME_S2I(20),
      },
      .rx = { // The receive identity for APRS
          .radio_conf = {
              .freq			= 144800000,
              .mod			= MOD_AFSK,
              .rssi         = 0x3F
          },
          // Node rx identity
           .call            = "DL7AD-11"
      },
      .tx = { // The transmit identity for digipeat transmit and messages responses
          .radio_conf = {
               .freq        = FREQ_APRS_RECEIVE,
               .pwr         = 0x7F,
               .mod         = MOD_AFSK,
               .rssi        = 0x4F
          },
          // Node tx identity
            .call           = "DL7AD-11",
            .path           = "WIDE2-1",
            .symbol         = SYM_DIGIPEATER,
            .beacon         = false,
            .lat            = -337331175,
            .lon            = 1511143478,
            .alt            = 144,
            .interval       = TIME_S2I(60*5),

            .tel_enc_cycle  = TIME_S2I(60*180)
      },
      .base = {
               // The base station identity
               // Tracker originated messages can be sent to this call sign sent
             .enabled       = false,
             .call          = "VK2GJ-7",
             .path          = "WIDE2-1",
      },
      .dig_active           = false,
      .freq                 = 144800000 // Default APRS frequency
	},

	// Power control
	.keep_cam_switched_on	= false,
	.gps_on_vbat			= 1000,
	.gps_off_vbat			= 1000,
	.gps_onper_vbat			= 1000,

	.magic					= CONFIG_MAGIC_DEFAULT // Do not remove. This is the activation bit.
};
