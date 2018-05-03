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
			.cycle			= TIME_S2I(60*30),
			.init_delay		= TIME_S2I(30)
		},
		.radio_conf = {
			.pwr			= 0x7F,
			.freq			= FREQ_APRS_RECEIVE,
			.mod			= MOD_AFSK,
            .cca            = 0x4F,
		},
		// Node identity
		.call				= "VK2GJ-12",
		.path				= "WIDE2-1",
		.symbol				= SYM_ANTENNA,
		.aprs_msg           = true,

        .tel_enc_cycle  = TIME_S2I(60*180) // How often to send telemetry config
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
            .cca            = 0x4F
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
			.active			= false,
			.cycle			= TIME_S2I(60*10),
			.init_delay		= TIME_S2I(60*5),
			.send_spacing	= TIME_S2I(5)
		},
		.radio_conf = {
			.pwr			= 0x7F,
            .freq           = 144800000,
			.mod			= MOD_2FSK,
			.cca            = 0x4F,
			.redundantTx	= false
		},
        // Node identity
		.call				= "VK2GJ-15",
		.path				= "",

		.res				= RES_VGA,
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
            .cca            = 0x4F
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
            .cca            = 0x4F
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
          .init_delay   = TIME_S2I(20)
      },
      // Default APRS frequency when geofence not resolved
      .freq                 = 145175000,
      // The receive identity for APRS
      .rx = {
          .radio_conf = {
              .freq			= FREQ_APRS_DYNAMIC,
              .mod			= MOD_AFSK,
              .rssi         = 0x3F
          },
          // Node rx identity
           .call            = "VK2GJ-4",
           .symbol          = SYM_ANTENNA   // Use this symbol in message responses
      },
      // The transmit identity for digipeat transmit and messages responses
      .digi = {
          .radio_conf = {
               .freq        = FREQ_APRS_RECEIVE,
               .pwr         = 0x7F,
               .mod         = MOD_AFSK,
               .cca         = 0x4F
          },
            .active     = true,
          // Node tx identity
            .call           = "VK2GJ-5",
            .path           = "WIDE2-1",
            .symbol         = SYM_DIGIPEATER,
            .beacon         = true,             // Set to have digi beacon position and telem
            .cycle          = TIME_S2I(60*30),  // Position and telem beacon interval
            .gps            = false,            // Set to have digi use GPS for position
            // A set location if GPS not enabled or unable to acquire lock.
            .lat            = -337331175,       // Degrees (1e-7)
            .lon            = 1511143478,       // Degrees (1e-7)
            .alt            = 144,              // Altitude in metres

            .tel_enc_cycle  = TIME_S2I(60*180) // How often to send telemetry config
      },
      // The base station identity
      .base = {
             // Tracker originated messages will be sent to this call sign if enabled
             .enabled       = true,
             .call          = "VK2GJ-7",
             .path          = "WIDE2-1",
      },
	},

	// Power control
	.keep_cam_switched_on	= false,
	.gps_on_vbat			= 3300,         // mV
	.gps_off_vbat			= 2500,         // mV
	.gps_onper_vbat			= 5000,         // mV

	// GPS altitude model control (air pressure determined by on-board BME280)
	.gps_pressure           = 90000,        // Air pressure (Pa) threshold for alt model switch
	.gps_low_alt            = GPS_STATIONARY,
	.gps_high_alt           = GPS_AIRBORNE_1G,

	.magic					= CONFIG_MAGIC_DEFAULT // Do not remove. This is the activation bit.
};
