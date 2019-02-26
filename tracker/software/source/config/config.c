/**
 * Put your configuration settings here. See description of all fields in types.h
 */


#include "config.h"
#include "aprs.h"
#include "geofence.h"
#include "pktconf.h"

conf_t conf_sram useCCM;

const conf_t conf_flash_default = {
    // Primary position app
    .pos_pri = {
        .beacon = {
            .active = false,
            .cycle = TIME_S2I(60 * 30),
            .init_delay = TIME_S2I(5),
            .fixed = true, // Add lat, lon alt fields when enabling fixed
            .lat = -337331175, // Degrees (expressed in 1e-7 form)
            .lon = 1511143478, // Degrees (expressed in 1e-7 form)
            .alt = 144 // Altitude in metres
        },
        /* Altitude controlled settings. */
        .run_alt = 160,
        .radio_conf = {
            .pwr = 0x7F,
            .freq = FREQ_GEOFENCE,
            .mod = MOD_AFSK,
            .cca = 0x5F,
        },
        // App identity
        .call = "VK2GJ-9",
        .path = "WIDE2-1",
        .symbol = SYM_ANTENNA,
        .aprs_msg = true, // Enable APRS message reception on this app
    },

    // Secondary position app
    .pos_sec = {
        .beacon = {
            .active = true,
            .cycle = TIME_S2I(60 * 5), // Beacon interval
            .init_delay = TIME_S2I(10),
            .fixed = false
        },
        /* Altitude controlled settings. */
        .run_alt = 130,
        .arm_alt = 150,
        /* Radio configuration. */
        .radio_conf = {
            .pwr = 0x7F,
            .freq = FREQ_GEOFENCE,
            .mod = MOD_AFSK,
            .cca = 0x5F
        },
        // App identity
        .call = "VK2GJ-8",
        .path = "WIDE2-1",
        .symbol = SYM_ANTENNA,
        .aprs_msg = true, // Enable APRS message reception on this app
    },

    // Primary image app
    .img_pri = {
        .svc_conf = {
            .active = false,
            .cycle = TIME_S2I(60 * 15),
            .init_delay = TIME_S2I(60),
            .send_spacing = TIME_S2I(10)
        },
        .radio_conf = {
            .pwr = 0x7F,
            .freq = FREQ_GEOFENCE,
            .mod = MOD_AFSK,
            .cca = 0x5F
        },
        // App identity
        .call = "VK2GJ-6",
        .path = "",

        // Image settings
        .res = RES_QVGA,
        .quality = 4,
        .buf_size = 15 * 1024,
        .redundantTx = true
    },

    // Secondary image app
    .img_sec = {
        .svc_conf = {
            .active = false,
            .cycle = TIME_S2I(60 * 5),
            .init_delay = TIME_S2I(30),
            .send_spacing = TIME_S2I(0)
        },
        .radio_conf = {
            .pwr = 0x7F,
            .freq = 144800000,
            .mod = MOD_2FSK_9k6,
            .cca = 0x5F
        },
        // App identity
        .call = "VK2GJ-2",
        .path = "",

        // Image settings
        .res = RES_QVGA,
        .quality = 4,
        .buf_size = 20 * 1024,
        .redundantTx = false,
        .no_burst = true
    },

    // Log app
    .log = {
        .svc_conf = {
            .active = false,
            .cycle = TIME_S2I(10),
            .init_delay = TIME_S2I(5)
        },
        .radio_conf = {
            .pwr = 0x7F,
            .freq = FREQ_GEOFENCE,
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
        // The receive identity for APRS
        .rx = {
             .svc_conf = {
                 // The packet receive service is enabled if true
                 // Receive is paused and resumed by transmission on that radio
                 // Receive can have a schedule set by cycle and interval
                 // An interval will run receive for the specified time period
                 // If interval is TIME_IMMEDIATE the radio turns off then on immediately
                 // If interval is TIME_INFINITE then the radio stays on and cycle is ignored
                 // Cycle is checked after the interval has expired
                 // Cycle timing less than interval means the cycle will run immediately after interval
                 // If cycle is CYCLE_CONTINUOUSLY the radio turns off and on at each interval
                 .active = true,
                 .init_delay = TIME_S2I(10),
                 .cycle = TIME_S2I(60 * 5),
                 .interval = TIME_INFINITE //TIME_S2I((60 * 5) - 10)
             },
            // Receive radio configuration
            .radio_conf = {
                .freq = FREQ_APRS_AUSTRALIA,
                .mod = MOD_AFSK,
                .rssi = 0x4F
            },
            // APRS identity used in message responses if digipeat is not enabled
            .call = "VK2GJ-4",
            .symbol = SYM_ANTENNA
        },
        .aprs_msg = true, // Set true to enable messages to be accepted on RX call sign
        .digi = true,
        .tx = {
           // Transmit radio configuration
           .radio_conf = {
               .freq = FREQ_RX_APRS,
               .pwr = 0x7F,
               .mod = MOD_AFSK,
               .cca = 0x5F
           },
           // Digipeat transmission identity
           .call = "VK2GJ-6",
           .path = "WIDE2-1",
           .symbol = SYM_DIGIPEATER,
           // A digipeater beacon can be added using one of the POS apps
           // Set the POS identity the same as the dipipeater TX identity
           // Alternatively the digipeater can have its own .beacon entry here
           .beacon = {
               .active = true,
               .cycle = TIME_S2I(60 * 15), // Beacon interval
               .init_delay = TIME_S2I(10),
               .fixed = false
           },
       },
    },

    // Global controls

    // Power control
    .keep_cam_switched_on = false,
    .gps_on_vbat = 3300, // mV
    .gps_off_vbat = 3000, // mV
    .gps_onper_vbat = 3500, // mV

    // GPS altitude model control (air pressure controlled using on-board BME280)
    .gps_pressure = 90000, // Air pressure (Pa) threshold for alt model switch
    .gps_low_alt = GPS_AUTOMOTIVE,
    .gps_high_alt = GPS_AIRBORNE_1G,

    // APRS
    // How often to send telemetry config (global for beacons)
    .tel_enc_cycle = TIME_S2I(60 * 60),

    // The default APRS frequency when geofence is not resolved
    .freq = FREQ_APRS_AUSTRALIA,

    // The base station identity.
    .base = {
        // If enabled tracker initiated APRS messages are addressed to this call sign
       .enabled = true,
       .call = "VK2GJ-7",
       .path = "WIDE2-1",
    },

    .magic = CONFIG_MAGIC_DEFAULT // Do not remove. This is the activation bit.
};
