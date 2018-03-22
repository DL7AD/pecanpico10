/* trackuino copyright (C) 2010  EA5HAV Javi
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
#include "config.h"
#include "aprs.h"
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "debug.h"
#include "base91.h"
#include "digipeater.h"
#include "dedupe.h"
#include "radio.h"
#include "flash.h"
#include "image.h"

#define METER_TO_FEET(m) (((m)*26876) / 8192)

typedef struct {
	sysinterval_t time;
	char call[AX25_MAX_ADDR_LEN];
} heard_t;


static uint16_t msg_id;
char alias_re[] = "WIDE[4-7]-[1-7]|CITYD";
char wide_re[] = "WIDE[1-7]-[1-7]";
enum preempt_e preempt = PREEMPT_OFF;
static heard_t heard_list[20];
static bool dedupe_initialized;

const conf_command_t command_list[] = {
	{TYPE_INT,  "pos_pri.active",                sizeof(conf_sram.pos_pri.thread_conf.active),                &conf_sram.pos_pri.thread_conf.active               },
	{TYPE_TIME, "pos_pri.init_delay",            sizeof(conf_sram.pos_pri.thread_conf.init_delay),            &conf_sram.pos_pri.thread_conf.init_delay           },
	{TYPE_TIME, "pos_pri.packet_spacing",        sizeof(conf_sram.pos_pri.thread_conf.packet_spacing),        &conf_sram.pos_pri.thread_conf.packet_spacing       },
	{TYPE_INT,  "pos_pri.sleep_conf.type",       sizeof(conf_sram.pos_pri.thread_conf.sleep_conf.type),       &conf_sram.pos_pri.thread_conf.sleep_conf.type      },
	{TYPE_INT,  "pos_pri.sleep_conf.vbat_thres", sizeof(conf_sram.pos_pri.thread_conf.sleep_conf.vbat_thres), &conf_sram.pos_pri.thread_conf.sleep_conf.vbat_thres},
	{TYPE_INT,  "pos_pri.sleep_conf.vsol_thres", sizeof(conf_sram.pos_pri.thread_conf.sleep_conf.vsol_thres), &conf_sram.pos_pri.thread_conf.sleep_conf.vsol_thres},
	{TYPE_TIME, "pos_pri.cycle",                 sizeof(conf_sram.pos_pri.thread_conf.cycle),                 &conf_sram.pos_pri.thread_conf.cycle                },
	{TYPE_INT,  "pos_pri.pwr",                   sizeof(conf_sram.pos_pri.radio_conf.pwr),                    &conf_sram.pos_pri.radio_conf.pwr                   },
	{TYPE_INT,  "pos_pri.freq",                  sizeof(conf_sram.pos_pri.radio_conf.freq),                   &conf_sram.pos_pri.radio_conf.freq                  },
    {TYPE_INT,  "pos_pri.step",                  sizeof(conf_sram.pos_pri.radio_conf.step),                   &conf_sram.pos_pri.radio_conf.step                  },	{TYPE_INT,  "pos_pri.mod",                   sizeof(conf_sram.pos_pri.radio_conf.mod),                    &conf_sram.pos_pri.radio_conf.mod                   },
    {TYPE_INT,  "pos_pri.chan",                  sizeof(conf_sram.pos_pri.radio_conf.chan),                   &conf_sram.pos_pri.radio_conf.chan                  },	{TYPE_INT,  "pos_pri.preamble",              sizeof(conf_sram.pos_pri.radio_conf.preamble),               &conf_sram.pos_pri.radio_conf.preamble              },
	{TYPE_INT,  "pos_pri.speed",                 sizeof(conf_sram.pos_pri.radio_conf.speed),                  &conf_sram.pos_pri.radio_conf.speed                 },
	{TYPE_INT,  "pos_pri.redundantTx",           sizeof(conf_sram.pos_pri.radio_conf.redundantTx),            &conf_sram.pos_pri.radio_conf.redundantTx           },
	{TYPE_STR,  "pos_pri.call",                  sizeof(conf_sram.pos_pri.call),                              &conf_sram.pos_pri.call                             },
	{TYPE_STR,  "pos_pri.path",                  sizeof(conf_sram.pos_pri.path),                              &conf_sram.pos_pri.path                             },
	{TYPE_INT,  "pos_pri.symbol",                sizeof(conf_sram.pos_pri.symbol),                            &conf_sram.pos_pri.symbol                           },
	{TYPE_TIME, "pos_pri.tel_enc_cycle",         sizeof(conf_sram.pos_pri.tel_enc_cycle),                     &conf_sram.pos_pri.tel_enc_cycle                    },

	{TYPE_INT,  "pos_sec.active",                sizeof(conf_sram.pos_sec.thread_conf.active),                &conf_sram.pos_sec.thread_conf.active               },
	{TYPE_TIME, "pos_sec.init_delay",            sizeof(conf_sram.pos_sec.thread_conf.init_delay),            &conf_sram.pos_sec.thread_conf.init_delay           },
	{TYPE_TIME, "pos_sec.packet_spacing",        sizeof(conf_sram.pos_sec.thread_conf.packet_spacing),        &conf_sram.pos_sec.thread_conf.packet_spacing       },
	{TYPE_INT,  "pos_sec.sleep_conf.type",       sizeof(conf_sram.pos_sec.thread_conf.sleep_conf.type),       &conf_sram.pos_sec.thread_conf.sleep_conf.type      },
	{TYPE_INT,  "pos_sec.sleep_conf.vbat_thres", sizeof(conf_sram.pos_sec.thread_conf.sleep_conf.vbat_thres), &conf_sram.pos_sec.thread_conf.sleep_conf.vbat_thres},
	{TYPE_INT,  "pos_sec.sleep_conf.vsol_thres", sizeof(conf_sram.pos_sec.thread_conf.sleep_conf.vsol_thres), &conf_sram.pos_sec.thread_conf.sleep_conf.vsol_thres},
	{TYPE_TIME, "pos_sec.cycle",                 sizeof(conf_sram.pos_sec.thread_conf.cycle),                 &conf_sram.pos_sec.thread_conf.cycle                },
	{TYPE_INT,  "pos_sec.pwr",                   sizeof(conf_sram.pos_sec.radio_conf.pwr),                    &conf_sram.pos_sec.radio_conf.pwr                   },
	{TYPE_INT,  "pos_sec.freq",                  sizeof(conf_sram.pos_sec.radio_conf.freq),                   &conf_sram.pos_sec.radio_conf.freq                  },
    {TYPE_INT,  "pos_sec.step",                  sizeof(conf_sram.pos_sec.radio_conf.step),                   &conf_sram.pos_sec.radio_conf.step                  },
    {TYPE_INT,  "pos_sec.chan",                  sizeof(conf_sram.pos_sec.radio_conf.chan),                   &conf_sram.pos_sec.radio_conf.chan                  },
	{TYPE_INT,  "pos_sec.mod",                   sizeof(conf_sram.pos_sec.radio_conf.mod),                    &conf_sram.pos_sec.radio_conf.mod                   },
	{TYPE_INT,  "pos_sec.preamble",              sizeof(conf_sram.pos_sec.radio_conf.preamble),               &conf_sram.pos_sec.radio_conf.preamble              },
	{TYPE_INT,  "pos_sec.speed",                 sizeof(conf_sram.pos_sec.radio_conf.speed),                  &conf_sram.pos_sec.radio_conf.speed                 },
	{TYPE_INT,  "pos_sec.redundantTx",           sizeof(conf_sram.pos_sec.radio_conf.redundantTx),            &conf_sram.pos_sec.radio_conf.redundantTx           },
	{TYPE_STR,  "pos_sec.call",                  sizeof(conf_sram.pos_sec.call),                              &conf_sram.pos_sec.call                             },
	{TYPE_STR,  "pos_sec.path",                  sizeof(conf_sram.pos_sec.path),                              &conf_sram.pos_sec.path                             },
	{TYPE_INT,  "pos_sec.symbol",                sizeof(conf_sram.pos_sec.symbol),                            &conf_sram.pos_sec.symbol                           },
	{TYPE_TIME, "pos_sec.tel_enc_cycle",         sizeof(conf_sram.pos_sec.tel_enc_cycle),                     &conf_sram.pos_sec.tel_enc_cycle                    },

	{TYPE_INT,  "img_pri.active",                sizeof(conf_sram.img_pri.thread_conf.active),                &conf_sram.img_pri.thread_conf.active               },
	{TYPE_TIME, "img_pri.init_delay",            sizeof(conf_sram.img_pri.thread_conf.init_delay),            &conf_sram.img_pri.thread_conf.init_delay           },
	{TYPE_TIME, "img_pri.packet_spacing",        sizeof(conf_sram.img_pri.thread_conf.packet_spacing),        &conf_sram.img_pri.thread_conf.packet_spacing       },
	{TYPE_INT,  "img_pri.sleep_conf.type",       sizeof(conf_sram.img_pri.thread_conf.sleep_conf.type),       &conf_sram.img_pri.thread_conf.sleep_conf.type      },
	{TYPE_INT,  "img_pri.sleep_conf.vbat_thres", sizeof(conf_sram.img_pri.thread_conf.sleep_conf.vbat_thres), &conf_sram.img_pri.thread_conf.sleep_conf.vbat_thres},
	{TYPE_INT,  "img_pri.sleep_conf.vsol_thres", sizeof(conf_sram.img_pri.thread_conf.sleep_conf.vsol_thres), &conf_sram.img_pri.thread_conf.sleep_conf.vsol_thres},
	{TYPE_TIME, "img_pri.cycle",                 sizeof(conf_sram.img_pri.thread_conf.cycle),                 &conf_sram.img_pri.thread_conf.cycle                },
	{TYPE_INT,  "img_pri.pwr",                   sizeof(conf_sram.img_pri.radio_conf.pwr),                    &conf_sram.img_pri.radio_conf.pwr                   },
    {TYPE_INT,  "img_pri.freq",                  sizeof(conf_sram.img_pri.radio_conf.freq),                   &conf_sram.img_pri.radio_conf.freq                  },
    {TYPE_INT,  "img_pri.step",                  sizeof(conf_sram.img_pri.radio_conf.step),                   &conf_sram.img_pri.radio_conf.step                  },
    {TYPE_INT,  "img_pri.chan",                  sizeof(conf_sram.img_pri.radio_conf.chan),                   &conf_sram.img_pri.radio_conf.chan                  },
	{TYPE_INT,  "img_pri.mod",                   sizeof(conf_sram.img_pri.radio_conf.mod),                    &conf_sram.img_pri.radio_conf.mod                   },
	{TYPE_INT,  "img_pri.preamble",              sizeof(conf_sram.img_pri.radio_conf.preamble),               &conf_sram.img_pri.radio_conf.preamble              },
	{TYPE_INT,  "img_pri.speed",                 sizeof(conf_sram.img_pri.radio_conf.speed),                  &conf_sram.img_pri.radio_conf.speed                 },
	{TYPE_INT,  "img_pri.redundantTx",           sizeof(conf_sram.img_pri.radio_conf.redundantTx),            &conf_sram.img_pri.radio_conf.redundantTx           },
	{TYPE_STR,  "img_pri.call",                  sizeof(conf_sram.img_pri.call),                              &conf_sram.img_pri.call                             },
	{TYPE_STR,  "img_pri.path",                  sizeof(conf_sram.img_pri.path),                              &conf_sram.img_pri.path                             },
	{TYPE_INT,  "img_pri.res",                   sizeof(conf_sram.img_pri.res),                               &conf_sram.img_pri.res                              },
	{TYPE_INT,  "img_pri.quality",               sizeof(conf_sram.img_pri.quality),                           &conf_sram.img_pri.quality                          },
	{TYPE_INT,  "img_pri.buf_size",              sizeof(conf_sram.img_pri.buf_size),                          &conf_sram.img_pri.buf_size                         },

	{TYPE_INT,  "img_sec.active",                sizeof(conf_sram.img_sec.thread_conf.active),                &conf_sram.img_sec.thread_conf.active               },
	{TYPE_TIME, "img_sec.init_delay",            sizeof(conf_sram.img_sec.thread_conf.init_delay),            &conf_sram.img_sec.thread_conf.init_delay           },
	{TYPE_TIME, "img_sec.packet_spacing",        sizeof(conf_sram.img_sec.thread_conf.packet_spacing),        &conf_sram.img_sec.thread_conf.packet_spacing       },
	{TYPE_INT,  "img_sec.sleep_conf.type",       sizeof(conf_sram.img_sec.thread_conf.sleep_conf.type),       &conf_sram.img_sec.thread_conf.sleep_conf.type      },
	{TYPE_INT,  "img_sec.sleep_conf.vbat_thres", sizeof(conf_sram.img_sec.thread_conf.sleep_conf.vbat_thres), &conf_sram.img_sec.thread_conf.sleep_conf.vbat_thres},
	{TYPE_INT,  "img_sec.sleep_conf.vsol_thres", sizeof(conf_sram.img_sec.thread_conf.sleep_conf.vsol_thres), &conf_sram.img_sec.thread_conf.sleep_conf.vsol_thres},
	{TYPE_TIME, "img_sec.cycle",                 sizeof(conf_sram.img_sec.thread_conf.cycle),                 &conf_sram.img_sec.thread_conf.cycle                },
	{TYPE_INT,  "img_sec.pwr",                   sizeof(conf_sram.img_sec.radio_conf.pwr),                    &conf_sram.img_sec.radio_conf.pwr                   },
	{TYPE_INT,  "img_sec.freq",                  sizeof(conf_sram.img_sec.radio_conf.freq),                   &conf_sram.img_sec.radio_conf.freq                  },
    {TYPE_INT,  "img_sec.step",                  sizeof(conf_sram.img_sec.radio_conf.step),                   &conf_sram.img_sec.radio_conf.step                  },
    {TYPE_INT,  "img_sec.chan",                  sizeof(conf_sram.img_sec.radio_conf.chan),                   &conf_sram.img_sec.radio_conf.chan                  },
	{TYPE_INT,  "img_sec.mod",                   sizeof(conf_sram.img_sec.radio_conf.mod),                    &conf_sram.img_sec.radio_conf.mod                   },
	{TYPE_INT,  "img_sec.preamble",              sizeof(conf_sram.img_sec.radio_conf.preamble),               &conf_sram.img_sec.radio_conf.preamble              },
	{TYPE_INT,  "img_sec.speed",                 sizeof(conf_sram.img_sec.radio_conf.speed),                  &conf_sram.img_sec.radio_conf.speed                 },
	{TYPE_INT,  "img_sec.redundantTx",           sizeof(conf_sram.img_sec.radio_conf.redundantTx),            &conf_sram.img_sec.radio_conf.redundantTx           },
	{TYPE_STR,  "img_sec.call",                  sizeof(conf_sram.img_sec.call),                              &conf_sram.img_sec.call                             },
	{TYPE_STR,  "img_sec.path",                  sizeof(conf_sram.img_sec.path),                              &conf_sram.img_sec.path                             },
	{TYPE_INT,  "img_sec.res",                   sizeof(conf_sram.img_sec.res),                               &conf_sram.img_sec.res                              },
	{TYPE_INT,  "img_sec.quality",               sizeof(conf_sram.img_sec.quality),                           &conf_sram.img_sec.quality                          },
	{TYPE_INT,  "img_sec.buf_size",              sizeof(conf_sram.img_sec.buf_size),                          &conf_sram.img_sec.buf_size                         },

	{TYPE_INT,  "log.active",                    sizeof(conf_sram.log.thread_conf.active),                    &conf_sram.log.thread_conf.active                   },
	{TYPE_TIME, "log.init_delay",                sizeof(conf_sram.log.thread_conf.init_delay),                &conf_sram.log.thread_conf.init_delay               },
	{TYPE_TIME, "log.packet_spacing",            sizeof(conf_sram.log.thread_conf.packet_spacing),            &conf_sram.log.thread_conf.packet_spacing           },
	{TYPE_INT,  "log.sleep_conf.type",           sizeof(conf_sram.log.thread_conf.sleep_conf.type),           &conf_sram.log.thread_conf.sleep_conf.type          },
	{TYPE_INT,  "log.sleep_conf.vbat_thres",     sizeof(conf_sram.log.thread_conf.sleep_conf.vbat_thres),     &conf_sram.log.thread_conf.sleep_conf.vbat_thres    },
	{TYPE_INT,  "log.sleep_conf.vsol_thres",     sizeof(conf_sram.log.thread_conf.sleep_conf.vsol_thres),     &conf_sram.log.thread_conf.sleep_conf.vsol_thres    },
	{TYPE_TIME, "log.cycle",                     sizeof(conf_sram.log.thread_conf.cycle),                     &conf_sram.log.thread_conf.cycle                    },
	{TYPE_INT,  "log.pwr",                       sizeof(conf_sram.log.radio_conf.pwr),                        &conf_sram.log.radio_conf.pwr                       },
	{TYPE_INT,  "log.freq",                      sizeof(conf_sram.log.radio_conf.freq),                       &conf_sram.log.radio_conf.freq                      },
    {TYPE_INT,  "log.step",                      sizeof(conf_sram.log.radio_conf.step),                       &conf_sram.log.radio_conf.step                      },
    {TYPE_INT,  "log.chan",                      sizeof(conf_sram.log.radio_conf.chan),                       &conf_sram.log.radio_conf.chan                      },
	{TYPE_INT,  "log.mod",                       sizeof(conf_sram.log.radio_conf.mod),                        &conf_sram.log.radio_conf.mod                       },
	{TYPE_INT,  "log.preamble",                  sizeof(conf_sram.log.radio_conf.preamble),                   &conf_sram.log.radio_conf.preamble                  },
	{TYPE_INT,  "log.speed",                     sizeof(conf_sram.log.radio_conf.speed),                      &conf_sram.log.radio_conf.speed                     },
	{TYPE_INT,  "log.redundantTx",               sizeof(conf_sram.log.radio_conf.redundantTx),                &conf_sram.log.radio_conf.redundantTx               },
	{TYPE_STR,  "log.call",                      sizeof(conf_sram.log.call),                                  &conf_sram.log.call                                 },
	{TYPE_STR,  "log.path",                      sizeof(conf_sram.log.path),                                  &conf_sram.log.path                                 },
	{TYPE_INT,  "log.density",                   sizeof(conf_sram.log.density),                               &conf_sram.log.density                              },

	{TYPE_INT,  "rx.active",                     sizeof(conf_sram.rx.thread_conf.active),                     &conf_sram.rx.thread_conf.active                    },
	{TYPE_TIME, "rx.init_delay",                 sizeof(conf_sram.rx.thread_conf.init_delay),                 &conf_sram.rx.thread_conf.init_delay                },
	{TYPE_TIME, "rx.packet_spacing",             sizeof(conf_sram.rx.thread_conf.packet_spacing),             &conf_sram.rx.thread_conf.packet_spacing            },
	{TYPE_INT,  "rx.sleep_conf.type",            sizeof(conf_sram.rx.thread_conf.sleep_conf.type),            &conf_sram.rx.thread_conf.sleep_conf.type           },
	{TYPE_INT,  "rx.sleep_conf.vbat_thres",      sizeof(conf_sram.rx.thread_conf.sleep_conf.vbat_thres),      &conf_sram.rx.thread_conf.sleep_conf.vbat_thres     },
	{TYPE_INT,  "rx.sleep_conf.vsol_thres",      sizeof(conf_sram.rx.thread_conf.sleep_conf.vsol_thres),      &conf_sram.rx.thread_conf.sleep_conf.vsol_thres     },
	{TYPE_TIME, "rx.cycle",                      sizeof(conf_sram.rx.thread_conf.cycle),                      &conf_sram.rx.thread_conf.cycle                     },
	{TYPE_INT,  "rx.pwr",                        sizeof(conf_sram.rx.radio_conf.pwr),                         &conf_sram.rx.radio_conf.pwr                        },
	{TYPE_INT,  "rx.freq",                       sizeof(conf_sram.rx.radio_conf.freq),                        &conf_sram.rx.radio_conf.freq                       },
    {TYPE_INT,  "rx.step",                       sizeof(conf_sram.rx.radio_conf.step),                        &conf_sram.rx.radio_conf.step                       },
    {TYPE_INT,  "rx.chan",                       sizeof(conf_sram.rx.radio_conf.chan),                        &conf_sram.rx.radio_conf.chan                       },
	{TYPE_INT,  "rx.mod",                        sizeof(conf_sram.rx.radio_conf.mod),                         &conf_sram.rx.radio_conf.mod                        },
	{TYPE_INT,  "rx.preamble",                   sizeof(conf_sram.rx.radio_conf.preamble),                    &conf_sram.rx.radio_conf.preamble                   },
	{TYPE_INT,  "rx.speed",                      sizeof(conf_sram.rx.radio_conf.speed),                       &conf_sram.rx.radio_conf.speed                      },
	{TYPE_INT,  "rx.redundantTx",                sizeof(conf_sram.rx.radio_conf.redundantTx),                 &conf_sram.rx.radio_conf.redundantTx                },
	{TYPE_STR,  "rx.call",                       sizeof(conf_sram.rx.call),                                   &conf_sram.rx.call                                  },
	{TYPE_STR,  "rx.path",                       sizeof(conf_sram.rx.path),                                   &conf_sram.rx.path                                  },
	{TYPE_INT,  "rx.symbol",                     sizeof(conf_sram.rx.symbol),                                 &conf_sram.rx.symbol                                },

	{TYPE_INT,  "rssi",                          sizeof(conf_sram.rssi),                                      &conf_sram.rssi                                     },
	{TYPE_INT,  "dig_active",                    sizeof(conf_sram.dig_active),                                &conf_sram.dig_active                               },
	{TYPE_INT,  "keep_cam_switched_on",          sizeof(conf_sram.keep_cam_switched_on),                      &conf_sram.keep_cam_switched_on                     },
	{TYPE_INT,  "gps_on_vbat",                   sizeof(conf_sram.gps_on_vbat),                               &conf_sram.gps_on_vbat                              },
	{TYPE_INT,  "gps_off_vbat",                  sizeof(conf_sram.gps_off_vbat),                              &conf_sram.gps_off_vbat                             },
	{TYPE_INT,  "gps_onper_vbat",                sizeof(conf_sram.gps_onper_vbat),                            &conf_sram.gps_onper_vbat                           },

	{TYPE_NULL}
};

void aprs_debug_getPacket(packet_t pp, char* buf, uint32_t len)
{
	// Decode packet
	char rec[256];
	unsigned char *pinfo;
	ax25_format_addrs(pp, rec);
	if(ax25_get_info(pp, &pinfo) == 0)
	  return;

    // Print decoded packet
    uint32_t out = chsnprintf(buf, len, "%s", rec);
    for(uint32_t i=0; pinfo[i]; i++) {
        if(pinfo[i] < 32 || pinfo[i] > 126) {
            out += chsnprintf(&buf[out], len-out, "<0x%02x>", pinfo[i]);
        } else {
            out += chsnprintf(&buf[out], len-out, "%c", pinfo[i]);
        }
    }
}

/**
 * Transmit APRS position packet. The comments are filled with:
 * - Static comment (can be set in config.h)
 * - Battery voltage in mV
 * - Solar voltage in mW (if tracker is solar-enabled)
 * - Temperature in Celcius
 * - Air pressure in Pascal
 * - Number of satellites being used
 * - Number of cycles where GPS has been lost (if applicable in cycle)
 */
packet_t aprs_encode_position(const char *callsign, const char *path, uint16_t symbol, dataPoint_t *dataPoint)
{
	// Latitude
	uint32_t y = 380926 * (90 - dataPoint->gps_lat/10000000.0);
	uint32_t y3  = y   / 753571;
	uint32_t y3r = y   % 753571;
	uint32_t y2  = y3r / 8281;
	uint32_t y2r = y3r % 8281;
	uint32_t y1  = y2r / 91;
	uint32_t y1r = y2r % 91;

	// Longitude
	uint32_t x = 190463 * (180 + dataPoint->gps_lon/10000000.0);
	uint32_t x3  = x   / 753571;
	uint32_t x3r = x   % 753571;
	uint32_t x2  = x3r / 8281;
	uint32_t x2r = x3r % 8281;
	uint32_t x1  = x2r / 91;
	uint32_t x1r = x2r % 91;

	// Altitude
	uint32_t a = logf(METER_TO_FEET(dataPoint->gps_alt)) / logf(1.002f);
	uint32_t a1  = a / 91;
	uint32_t a1r = a % 91;

	uint8_t gpsFix = dataPoint->gps_state == GPS_LOCKED1 || dataPoint->gps_state == GPS_LOCKED2 ? GSP_FIX_CURRENT : GSP_FIX_OLD;
	uint8_t src = NMEA_SRC_GGA;
	uint8_t origin = ORIGIN_PICO;

	char xmit[256];
	uint32_t len = chsnprintf(xmit, sizeof(xmit), "%s>%s,%s:!", callsign, APRS_DEST_CALLSIGN, path);

	xmit[len+0]  = (symbol >> 8) & 0xFF;
	xmit[len+1]  = y3+33;
	xmit[len+2]  = y2+33;
	xmit[len+3]  = y1+33;
	xmit[len+4]  = y1r+33;
	xmit[len+5]  = x3+33;
	xmit[len+6]  = x2+33;
	xmit[len+7]  = x1+33;
	xmit[len+8]  = x1r+33;
	xmit[len+9]  = symbol & 0xFF;
	xmit[len+10] = a1+33;
	xmit[len+11] = a1r+33;
	xmit[len+12] = ((gpsFix << 5) | (src << 3) | origin) + 33;

	// Comments
	uint32_t len2 = base91_encode((uint8_t*)dataPoint, (uint8_t*)&xmit[len+13], sizeof(dataPoint_t));

	xmit[len+len2+13] = '|';

	// Sequence ID
	uint32_t t = dataPoint->id & 0x1FFF;
	xmit[len+len2+14] = t/91 + 33;
	xmit[len+len2+15] = t%91 + 33;

	// Telemetry parameter
	for(uint8_t i=0; i<5; i++) {
		switch(i) {
			case 0: t = dataPoint->adc_vbat;				break;
			case 1: t = dataPoint->adc_vsol;				break;
			case 2: t = dataPoint->pac_pbat+4096;			break;
			case 3: t = dataPoint->sen_i1_temp/10 + 1000;	break;
			case 4: t = dataPoint->sen_i1_press/125 - 40;	break;
		}

		xmit[len+len2+16+i*2]   = t/91 + 33;
		xmit[len+len2+16+i*2+1] = t%91 + 33;
	}

	xmit[len+len2+26] = '|';
	xmit[len+len2+27] = 0;

	return ax25_from_text(xmit, 1);
}

packet_t aprs_encode_data_packet(const char *callsign, const char *path, char packetType, uint8_t *data)
{
	char xmit[256];
	chsnprintf(xmit, sizeof(xmit), "%s>%s,%s:{{%c%s", callsign, APRS_DEST_CALLSIGN, path, packetType, data);

	return ax25_from_text(xmit, 1);
}

/**
 * Transmit message packet
 */
packet_t aprs_encode_message(const char *callsign, const char *path, const char *receiver, const char *text, const bool noCounter)
{
	char xmit[256];
	if(noCounter)
		chsnprintf(xmit, sizeof(xmit), "%s>%s,%s::%-9s:%s", callsign, APRS_DEST_CALLSIGN, path, receiver, text);
	else
		chsnprintf(xmit, sizeof(xmit), "%s>%s,%s::%-9s:%s{%d", callsign, APRS_DEST_CALLSIGN, path, receiver, text, ++msg_id);

	return ax25_from_text(xmit, 1);
}

packet_t aprs_encode_query_answer_aprsd(const char *callsign, const char *path, const char *receiver)
{
	char buf[256] = "Directs=";
	uint32_t out = 8;
	for(uint8_t i=0; i<20; i++) {
		if(heard_list[i].time && heard_list[i].time + TIME_S2I(600) >= chVTGetSystemTime() && heard_list[i].time <= chVTGetSystemTime())
			out += chsnprintf(&buf[out], sizeof(buf)-out, "%s ", heard_list[i].call);
	}
	buf[out-1] = 0; // Remove last space

	return aprs_encode_message(callsign, path, receiver, buf, true);
}

static bool aprs_decode_message(packet_t pp)
{
	// Get Info field
	char src[256];
	unsigned char *pinfo;
	if(ax25_get_info(pp, &pinfo) == 0)
	  return false;
	ax25_format_addrs(pp, src);

	// Decode destination callsign
	char dest[AX25_MAX_ADDR_LEN];
	uint8_t i=0;

	while(i < sizeof(dest)-1) {
		if(pinfo[i+1] == ':' || pinfo[i+1] == ' ') {
			dest[i++] = 0;
			break;
		}
		dest[i] = pinfo[i+1];
		i++;
	}

	// Decode source callsign
	for(uint32_t i=0; i < sizeof(src); i++) {
		if(src[i] == '>') {
			src[i] = 0;
			break;
		}
	}

	// Try to find out if this message is meant for us
	if(pinfo[10] == ':' && !strcmp(conf_sram.rx.call, dest))
	{
		char msg_id_rx[8];
		memset(msg_id_rx, 0, sizeof(msg_id_rx));

		// Cut off control chars
		for(uint16_t i=11; pinfo[i] != 0 && i<0xFFFF; i++) {
			if(pinfo[i] == '{') {
				// Copy ACK ID
				memcpy(msg_id_rx, &pinfo[i+1], sizeof(msg_id_rx)-1);
				// Cut off non-printable chars
				for(uint8_t j=0; j<sizeof(msg_id_rx); j++) {
					if(msg_id_rx[j] < 32 || msg_id_rx[j] > 126) {
						msg_id_rx[j] = 0;
						break;
					}
				}
				pinfo[i] = 0; // Mark end of message
			}
			if(pinfo[i] == '\r' || pinfo[i] == '\n') {
				pinfo[i] = 0;
			}
		}

		// Trace
		TRACE_INFO("RX   > Received message from %s (ID=%s): %s", src, msg_id_rx, &pinfo[11]);

		char *command = strlwr((char*)&pinfo[11]);

		// Do control actions
		if(!strcmp(command, "?gpio pa8:1")) { // Switch on pin

			TRACE_INFO("RX   > Message: GPIO query PA8 HIGH");
			palSetPadMode(GPIOA, 8, PAL_MODE_OUTPUT_PUSHPULL);
			palSetPad(GPIOA, 8);

		} else if(!strcmp(command, "?gpio pa8:0")) { // Switch off pin

			TRACE_INFO("RX   > Message: GPIO query PA8 LOW");
			palSetPadMode(GPIOA, 8, PAL_MODE_OUTPUT_PUSHPULL);
			palClearPad(GPIOA, 8);

		} else if(!strcmp(command, "?aprsp")) { // Transmit position

			TRACE_INFO("RX   > Message: Position query");
			dataPoint_t* dataPoint = getLastDataPoint();
			packet_t pp = aprs_encode_position(conf_sram.rx.call, conf_sram.rx.path, conf_sram.rx.symbol, dataPoint);
            transmitOnRadio(pp,
                            conf_sram.rx.radio_conf.freq,
                            conf_sram.rx.radio_conf.step,
                            conf_sram.rx.radio_conf.chan,
                            conf_sram.rx.radio_conf.pwr,
                            conf_sram.rx.radio_conf.mod);

		} else if(!strcmp(command, "?aprsd")) { // Transmit position

			TRACE_INFO("RX   > Message: Directs query");
			packet_t pp = aprs_encode_query_answer_aprsd(conf_sram.rx.call, conf_sram.rx.path, src);
            transmitOnRadio(pp,
                            conf_sram.rx.radio_conf.freq,
                            conf_sram.rx.radio_conf.step,
                            conf_sram.rx.radio_conf.chan,
                            conf_sram.rx.radio_conf.pwr,
                            conf_sram.rx.radio_conf.mod);

		} else if(!strcmp(command, "?reset")) { // Transmit position

			TRACE_INFO("RX   > Message: System Reset");
			char buf[16];
			chsnprintf(buf, sizeof(buf), "ack%s", msg_id_rx);
			packet_t pp = aprs_encode_message(conf_sram.rx.call, conf_sram.rx.path, src, buf, true);
            transmitOnRadio(pp,
                            conf_sram.rx.radio_conf.freq,
                            conf_sram.rx.radio_conf.step,
                            conf_sram.rx.radio_conf.chan,
                            conf_sram.rx.radio_conf.pwr,
                            conf_sram.rx.radio_conf.mod);
			chThdSleep(TIME_S2I(5)); // Give some time to send the message

			NVIC_SystemReset();

		} else if(!strcmp(command, "?save")) { // Transmit position

			TRACE_INFO("RX   > Message: Save");
			conf_sram.magic = CONFIG_MAGIC_UPDATED;
			flashSectorBegin(flashSectorAt(0x08060000));
			flashErase(0x08060000, 0x20000);
			flashWrite(0x08060000, (char*)&conf_sram, sizeof(conf_t));
			flashSectorEnd(flashSectorAt(0x08060000));

		} else if(!strcmp(command, "?img reject pri")) { // Reject image

			reject_pri = true;

		} else if(!strcmp(command, "?img reject sec")) { // Reject image

			reject_sec = true;

		} else if(!strncmp(command, "?img ", 5)) { // Repeat packets

			TRACE_INFO("RX   > Message: Image packet repeat request");

			char *pt;
			pt = strtok(&command[5], " ");
			while(pt != NULL) {
				uint32_t req = strtol(pt, NULL, 16);

				for(uint8_t i=0; i<16; i++) {
					if(!packetRepeats[i].n_done) {
						packetRepeats[i].image_id = (req >> 16) & 0xFF;
						packetRepeats[i].packet_id = req & 0xFFFF;
						packetRepeats[i].n_done = true;

						TRACE_INFO("RX   > ... Image %3d Packet %3d", packetRepeats[i].image_id, packetRepeats[i].packet_id);
						break;
					}
				}

				pt = strtok(NULL, " ");
			}

		} else if(!strncmp(command, "?conf ", 6)) { // Modify configuration

			for(uint8_t i=0; command_list[i].type != TYPE_NULL; i++)
			{
				if(!strncmp(&command[6], command_list[i].name, strlen(command_list[i].name))) {

					char *value = &command[strlen(command_list[i].name) + 6];
					TRACE_INFO("RX   > Message: Configuration Command");
					TRACE_INFO("RX   > %s => %s", &command[6], value);

					if(command_list[i].type == TYPE_INT && command_list[i].size == 1) {
						*((uint8_t*)command_list[i].ptr) = atoi(value);
					} else if(command_list[i].type == TYPE_INT && command_list[i].size == 2) {
						*((uint16_t*)command_list[i].ptr) = atoi(value);
					} else if(command_list[i].type == TYPE_INT && command_list[i].size == 4) {
						*((uint32_t*)command_list[i].ptr) = atoi(value);
					} else if(command_list[i].type == TYPE_TIME) {
						*((sysinterval_t*)command_list[i].ptr) = TIME_MS2I(atoi(value));
					} else if(command_list[i].type == TYPE_STR) {
						strncpy((char*)command_list[i].ptr, value, sizeof(command_list[i].size)-1);
					}
				}
			}

		} else {
			TRACE_INFO("RX   > Message does not contain a known command");
		}

		if(msg_id_rx[0]) { // Message ID has been sent which has to be acknowledged
			char buf[16];
			chsnprintf(buf, sizeof(buf), "ack%s", msg_id_rx);
			packet_t pp = aprs_encode_message(conf_sram.rx.call, conf_sram.rx.path, src, buf, true);
            transmitOnRadio(pp,
                            conf_sram.rx.radio_conf.freq,
                            conf_sram.rx.radio_conf.step,
                            conf_sram.rx.radio_conf.chan,
                            conf_sram.rx.radio_conf.pwr,
                            conf_sram.rx.radio_conf.mod);
		}

		return false; // Mark that message dont has to be digipeated
	}

	return true; // Mark that message has to be digipeated
}

static void aprs_digipeat(packet_t pp)
{
	if(!dedupe_initialized) {
		dedupe_init(TIME_S2I(10));
		dedupe_initialized = true;
	}

	if(!dedupe_check(pp, 0)) { // Last identical packet older than 10 seconds
		packet_t result = digipeat_match(0, pp, conf_sram.rx.call, conf_sram.rx.call, alias_re, wide_re, 0, preempt, NULL);
		if(result != NULL) { // Should be digipeated
			dedupe_remember(result, 0);
			transmitOnRadio(result,
			                conf_sram.rx.radio_conf.freq,
                            conf_sram.rx.radio_conf.step,
                            conf_sram.rx.radio_conf.chan,
                            conf_sram.rx.radio_conf.pwr,
                            conf_sram.rx.radio_conf.mod);
		}
	}
}

/**
 * Transmit APRS telemetry configuration
 */
packet_t aprs_encode_telemetry_configuration(const char *callsign, const char *path, uint8_t type)
{
	switch(type)
	{
		case 0:	return aprs_encode_message(callsign, path, callsign, "PARM.Vbat,Vsol,Pbat,Temperature,Airpressure", true);
		case 1: return aprs_encode_message(callsign, path, callsign, "UNIT.V,V,W,degC,Pa", true);
		case 2: return aprs_encode_message(callsign, path, callsign, "EQNS.0,.001,0,0,.001,0,0,.001,-4.096,0,.1,-100,0,12.5,500", true);
		case 3: return aprs_encode_message(callsign, path, callsign, "BITS.11111111,", true);
		default: return NULL;
	}
}

void aprs_decode_packet(packet_t pp)
{
	// Get heard callsign
	char call[AX25_MAX_ADDR_LEN];
	int8_t v = -1;
	do {
		v++;
		ax25_get_addr_with_ssid(pp, ax25_get_heard(pp)-v, call);
	} while(ax25_get_heard(pp)-v >= AX25_SOURCE && (!strncmp("WIDE", call, 4) || !strncmp("TRACE", call, 5)));

	// Fill/Update direct list
	sysinterval_t first_time = 0xFFFFFFFF;	// Timestamp of oldest heard list entry
	uint8_t first_id = 0;					// ID of oldest heard list entry

	for(uint8_t i=0; i<=20; i++) {
		if(i < 20) {
			// Search for callsign in list
			if(!strcmp(heard_list[i].call, call)) { // Callsign found in list
				heard_list[i].time = chVTGetSystemTime(); // Update time the callsign was last heard
				break;
			}

			// Find oldest entry
			if(first_time > heard_list[i].time) {
				first_time = heard_list[i].time;
				first_id = i;
			}
		} else { // Callsign not in list
			// Overwrite old entry/ use empty entry
			memcpy(heard_list[first_id].call, call, sizeof(heard_list[first_id].call));
			heard_list[first_id].time = chVTGetSystemTime();
		}
	}

	// Decode message packets
	bool digipeat = true;
	unsigned char *pinfo;
	if(ax25_get_info(pp, &pinfo) == 0)
	    return;
	if(pinfo[0] == ':') digipeat = aprs_decode_message(pp); // ax25_get_dti(pp)

	// Digipeat packet
	if(conf_sram.dig_active && digipeat) {
		aprs_digipeat(pp);
	}
}

