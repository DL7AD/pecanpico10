/**
 * Put your configuration settings here. See description of all fields in types.h
 */


#include "config.h"
#include "aprs.h"
#include "geofence.h"

conf_t conf_sram;

const conf_t conf_flash_default = {
    // Primary position app
    .pos_pri = {
        .thread_conf = {
            .active = false,
            .cycle = TIME_S2I(60 * 5),
            .init_delay = TIME_S2I(30)
        },
        .radio_conf = {
            .pwr = 0x7F,
            .freq = FREQ_APRS_DYNAMIC,
            .mod = MOD_AFSK,
            .cca = 0x4F,
        },
        // App identity
        .call = "VK2GJ-12",
        .path = "WIDE1-1",
        .symbol = SYM_ANTENNA,
        .aprs_msg = true,
        // How often to send telemetry config
        .tel_enc_cycle = TIME_S2I(60 * 60)
    },

    // Secondary position app
    .pos_sec = {
        .thread_conf = {
            .active = false,
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
        .call = "VK2GJ-15",
        .path = "",
        .symbol = SYM_BALLOON,
        .aprs_msg = true,

        .tel_enc_cycle = TIME_S2I(0)
    },

    // Primary image app
    .img_pri = {
        .thread_conf = {
            .active = false,
            .cycle = CYCLE_CONTINUOUSLY,
            .init_delay = TIME_S2I(90),
            .send_spacing = TIME_S2I(0)
        },
        .radio_conf = {
            .pwr = 0x7F,
            .freq = 144800000,
            .mod = MOD_2FSK,
            .cca = 0x4F

        },
        // App identity
        .call = "VK2GJ-15",
        .path = "",

        // Image settings
        .res = RES_VGA,
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
            .freq = APRS_FREQ_AUSTRALIA,
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
            // The packet receive service is enabled if true
            // Receive is resumed after any transmission
            .active = true,
            .init_delay = TIME_S2I(20)
        },
        // The default APRS frequency when geofence is not resolved
        .freq = APRS_FREQ_AUSTRALIA,
        // The receive identity for APRS
        .rx = {
            // Receive radio configuration
            .radio_conf = {
                .freq = FREQ_APRS_DYNAMIC,
                .mod = MOD_AFSK,
                .rssi = 0x3F
            },
            // APRS identity used in message responses if digipeat is not enabled
            .call = "VK2GJ-4",
            .symbol = SYM_ANTENNA
        },
        // The digipeat transmit identity
        .digi = {
            .active = true,
            // Transmit radio configuration
            .radio_conf = {
                .freq = FREQ_APRS_RECEIVE,
                .pwr = 0x7F,
                .mod = MOD_AFSK,
                .cca = 0x4F
            },
            // Digipeat identity
            .call = "VK2GJ-5",
            .path = "WIDE2-1",
            .symbol = SYM_DIGIPEATER,
            // Set to have digi beacon position, telemetry & APRSD information.
            // This starts a BCN thread specifically for digi
            .beacon = true,
            .cycle = TIME_S2I(60 * 30), // Beacon interval
            // Set true to have digi use GPS for position
            // If valid position is not stored then default lat, lon and alt will be used.
            // If RTC time is invalid then GPS will be enabled to get time.
            // Once RTC is set then GPS is released and can be switched off.
            // This will be the case if no other position thread is using it.
            .gps = false,
            // A set location if GPS not enabled or unable to acquire lock.
            //.lat = -337331175, // Degrees (expressed in 1e-7 form)
            //.lon = 1511143478, // Degrees (expressed in 1e-7 form)
            //.alt = 144, // Altitude in metres
            // How often to send telemetry config (TODO: Move out to global level)
            .tel_enc_cycle = TIME_S2I(60 * 60 * 2)
        },
        // The base station identity
        .base = {
            // Tracker originated messages will be sent to this call sign if enabled
            .enabled = true,
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

    // A pre-set location if GPS never enabled or unable to acquire lock.
    .lat = -337331175, // Degrees (expressed in 1e-7 form)
    .lon = 1511143478, // Degrees (expressed in 1e-7 form)
    .alt = 144, // Altitude in metres

    .magic = CONFIG_MAGIC_DEFAULT // Do not remove. This is the activation bit.
};
