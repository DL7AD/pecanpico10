#include "config.h"
#include "aprs.h"

conf_t conf_sram;

const conf_t conf_flash_default = {
    // Primary position app
    .pos_pri = {
        .thread_conf = {
            .active = true,
            .cycle = TIME_S2I(60),
            .init_delay = TIME_S2I(0)
        },
        .radio_conf = {
            .pwr = 0x7F,
            .freq = 144800000,
            .mod = MOD_AFSK,
            .cca = 0xFF,
        },
        // App identity
        .call = "DL7AD-12",
        .path = "WIDE1-1",
        .symbol = SYM_BALLOON,
        .aprs_msg = true,
        // How often to send telemetry config
        .tel_enc_cycle = TIME_S2I(60 * 60)
    },

    // Secondary position app
    .pos_sec = {
        .thread_conf = {
            .active = true,
            .cycle = TIME_S2I(180),
            .init_delay = TIME_S2I(60)
        },
        .radio_conf = {
            .pwr = 0x7F,
            .freq = 145825000,
            .mod = MOD_AFSK,
            .cca = 0xFF
        },
        // App identity
        .call = "DL7AD-15",
        .path = "",
        .symbol = SYM_BALLOON,
        .aprs_msg = true,

        .tel_enc_cycle = TIME_S2I(0)
    },

    // Primary image app
    .img_pri = {
        .thread_conf = {
            .active = true,
            .cycle = CYCLE_CONTINUOUSLY,
            .init_delay = TIME_S2I(300),
            .send_spacing = TIME_S2I(30)
        },
        .radio_conf = {
            .pwr = 0x7F,
            .freq = FREQ_APRS_DYNAMIC,
            .mod = MOD_AFSK,
            .cca = 0xFF

        },
        // App identity
        .call = "DL7AD-15",
        .path = "",

        // Image settings
        .res = RES_QVGA,
        .quality = 4,
        .buf_size = 40 * 1024,
        .redundantTx = false
    },

    // Secondary image app
    .img_sec = {
        .thread_conf = {
            .active = false,
            .cycle = TIME_S2I(60 * 30),
            .init_delay = TIME_S2I(60 * 1),
            .send_spacing = TIME_S2I(30)
        },
        .radio_conf = {
            .pwr = 0x7F,
            .freq = 145175000,
            .mod = MOD_AFSK,
            .cca = 0x4F
        },
        // App identity
        .call = "VK2GJ-14",
        .path = "",

        // Image settings
        .res = RES_QVGA,
        .quality = 4,
        .buf_size = 15 * 1024,
        .redundantTx = false
    },

    // Log app
    .log = {
        .thread_conf = {
            .active = false,
            .cycle = TIME_S2I(10),
            .init_delay = TIME_S2I(5)
        },
        .radio_conf = {
            .pwr = 0x7F,
            .freq = FREQ_APRS_DYNAMIC,
            .mod = MOD_AFSK,
            .cca = 0x4F
        },
        // Node identity
        .call = "VK2GJ-13",
        .path = "WIDE1-1",
        .density = 10
    },

    // APRS app
    .aprs = {
        .thread_conf = {
            .active = true,
            .init_delay = TIME_S2I(10)
        },
        // Default APRS frequency when geofence not resolved
        .freq = 145825000,
        // The receive identity for APRS
        .rx = {
            .radio_conf = {
                .freq = 145825000,
                .mod = MOD_AFSK,
                .rssi = 0x5F
            },
            // App rx identity
            .call = "DL7AD-15",
            .symbol = SYM_BALLOON // Use this symbol in message responses
        },
        // The digipeat transmit identity and messages responses
        .digi = {
            .radio_conf = {
                .freq = 145825000,
                .pwr = 0x7F,
                .mod = MOD_AFSK,
                .cca = 0xFF
            },
            .active = true,
            // Digipeat identity
            .call = "DL7AD-15",
            .path = "WIDE2-1",
            .symbol = SYM_DIGIPEATER,
            .beacon = false, // Set to have digi beacon position and telem
            .cycle = TIME_S2I(60 * 30), // Position and telem beacon interval
            .gps = false, // Set to have digi use GPS for position
            // A set location if GPS not enabled or unable to acquire lock.
            .lat = -337331175, // Degrees (expressed in 1e-7 form)
            .lon = 1511143478, // Degrees (expressed in 1e-7 form)
            .alt = 144, // Altitude in metres
            // How often to send telemetry config (TODO: Move out to global level)
            .tel_enc_cycle = TIME_S2I(0)
        },
        // The base station identity
        .base = {
            // Tracker originated messages will be sent to this call sign if enabled
            .enabled = false,
            .call = "VK2GJ-7",
            .path = "WIDE2-1",
        },
    },

    // Global controls
    // Power control
    .keep_cam_switched_on = false,
    .gps_on_vbat = 1000, // mV
    .gps_off_vbat = 1000, // mV
    .gps_onper_vbat = 1000, // mV

    // GPS altitude model control (air pressure determined by on-board BME280)
    .gps_pressure = 90000, // Air pressure (Pa) threshold for alt model switch
    .gps_low_alt = GPS_STATIONARY,
    .gps_high_alt = GPS_AIRBORNE_1G,

    .magic = CONFIG_MAGIC_DEFAULT // Do not remove. This is the activation bit.
};
