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
static heard_t heard_list[APRS_HEARD_LIST_SIZE];
static bool dedupe_initialized;

const conf_command_t command_list[] = {
	{TYPE_INT,  "pos_pri.active",                sizeof(conf_sram.pos_pri.thread_conf.active),                &conf_sram.pos_pri.thread_conf.active               },
	{TYPE_TIME, "pos_pri.init_delay",            sizeof(conf_sram.pos_pri.thread_conf.init_delay),            &conf_sram.pos_pri.thread_conf.init_delay           },
/*	{TYPE_TIME, "pos_pri.send_spacing",          sizeof(conf_sram.pos_pri.thread_conf.send_spacing),          &conf_sram.pos_pri.thread_conf.send_spacing       },*/
	{TYPE_INT,  "pos_pri.sleep_conf.type",       sizeof(conf_sram.pos_pri.thread_conf.sleep_conf.type),       &conf_sram.pos_pri.thread_conf.sleep_conf.type      },
	{TYPE_INT,  "pos_pri.sleep_conf.vbat_thres", sizeof(conf_sram.pos_pri.thread_conf.sleep_conf.vbat_thres), &conf_sram.pos_pri.thread_conf.sleep_conf.vbat_thres},
	{TYPE_INT,  "pos_pri.sleep_conf.vsol_thres", sizeof(conf_sram.pos_pri.thread_conf.sleep_conf.vsol_thres), &conf_sram.pos_pri.thread_conf.sleep_conf.vsol_thres},
	{TYPE_TIME, "pos_pri.cycle",                 sizeof(conf_sram.pos_pri.thread_conf.cycle),                 &conf_sram.pos_pri.thread_conf.cycle                },
	{TYPE_INT,  "pos_pri.pwr",                   sizeof(conf_sram.pos_pri.radio_conf.pwr),                    &conf_sram.pos_pri.radio_conf.pwr                   },
	{TYPE_INT,  "pos_pri.freq",                  sizeof(conf_sram.pos_pri.radio_conf.freq),                   &conf_sram.pos_pri.radio_conf.freq                  },
    {TYPE_INT,  "pos_pri.mod",                   sizeof(conf_sram.pos_pri.radio_conf.mod),                    &conf_sram.pos_pri.radio_conf.mod                   },
    {TYPE_INT,  "pos_pri.rssi",                  sizeof(conf_sram.pos_pri.radio_conf.rssi),                   &conf_sram.pos_pri.radio_conf.rssi                  },
/*    {TYPE_INT,  "pos_pri.speed",                 sizeof(conf_sram.pos_pri.radio_conf.speed),                  &conf_sram.pos_pri.radio_conf.speed                 },*/
/*	{TYPE_INT,  "pos_pri.redundantTx",           sizeof(conf_sram.pos_pri.radio_conf.redundantTx),            &conf_sram.pos_pri.radio_conf.redundantTx           },*/
	{TYPE_STR,  "pos_pri.call",                  sizeof(conf_sram.pos_pri.call),                              &conf_sram.pos_pri.call                             },
	{TYPE_STR,  "pos_pri.path",                  sizeof(conf_sram.pos_pri.path),                              &conf_sram.pos_pri.path                             },
	{TYPE_INT,  "pos_pri.symbol",                sizeof(conf_sram.pos_pri.symbol),                            &conf_sram.pos_pri.symbol                           },
    {TYPE_INT,  "pos_pri.aprs_msg",              sizeof(conf_sram.pos_pri.aprs_msg),                          &conf_sram.pos_pri.aprs_msg                         },
	{TYPE_TIME, "pos_pri.tel_enc_cycle",         sizeof(conf_sram.pos_pri.tel_enc_cycle),                     &conf_sram.pos_pri.tel_enc_cycle                    },

	{TYPE_INT,  "pos_sec.active",                sizeof(conf_sram.pos_sec.thread_conf.active),                &conf_sram.pos_sec.thread_conf.active               },
	{TYPE_TIME, "pos_sec.init_delay",            sizeof(conf_sram.pos_sec.thread_conf.init_delay),            &conf_sram.pos_sec.thread_conf.init_delay           },
/*	{TYPE_TIME, "pos_sec.send_spacing",          sizeof(conf_sram.pos_sec.thread_conf.send_spacing),          &conf_sram.pos_sec.thread_conf.send_spacing       },*/
	{TYPE_INT,  "pos_sec.sleep_conf.type",       sizeof(conf_sram.pos_sec.thread_conf.sleep_conf.type),       &conf_sram.pos_sec.thread_conf.sleep_conf.type      },
	{TYPE_INT,  "pos_sec.sleep_conf.vbat_thres", sizeof(conf_sram.pos_sec.thread_conf.sleep_conf.vbat_thres), &conf_sram.pos_sec.thread_conf.sleep_conf.vbat_thres},
	{TYPE_INT,  "pos_sec.sleep_conf.vsol_thres", sizeof(conf_sram.pos_sec.thread_conf.sleep_conf.vsol_thres), &conf_sram.pos_sec.thread_conf.sleep_conf.vsol_thres},
	{TYPE_TIME, "pos_sec.cycle",                 sizeof(conf_sram.pos_sec.thread_conf.cycle),                 &conf_sram.pos_sec.thread_conf.cycle                },
	{TYPE_INT,  "pos_sec.pwr",                   sizeof(conf_sram.pos_sec.radio_conf.pwr),                    &conf_sram.pos_sec.radio_conf.pwr                   },
	{TYPE_INT,  "pos_sec.freq",                  sizeof(conf_sram.pos_sec.radio_conf.freq),                   &conf_sram.pos_sec.radio_conf.freq                  },
	{TYPE_INT,  "pos_sec.mod",                   sizeof(conf_sram.pos_sec.radio_conf.mod),                    &conf_sram.pos_sec.radio_conf.mod                   },
    {TYPE_INT,  "pos_sec.rssi",                  sizeof(conf_sram.pos_sec.radio_conf.rssi),                   &conf_sram.pos_sec.radio_conf.rssi                  },
/*	{TYPE_INT,  "pos_sec.speed",                 sizeof(conf_sram.pos_sec.radio_conf.speed),                  &conf_sram.pos_sec.radio_conf.speed                 },*/
/*	{TYPE_INT,  "pos_sec.redundantTx",           sizeof(conf_sram.pos_sec.radio_conf.redundantTx),            &conf_sram.pos_sec.radio_conf.redundantTx           },*/
	{TYPE_STR,  "pos_sec.call",                  sizeof(conf_sram.pos_sec.call),                              &conf_sram.pos_sec.call                             },
	{TYPE_STR,  "pos_sec.path",                  sizeof(conf_sram.pos_sec.path),                              &conf_sram.pos_sec.path                             },
	{TYPE_INT,  "pos_sec.symbol",                sizeof(conf_sram.pos_sec.symbol),                            &conf_sram.pos_sec.symbol                           },
    {TYPE_INT,  "pos_sec.aprs_msg",              sizeof(conf_sram.pos_sec.aprs_msg),                          &conf_sram.pos_sec.aprs_msg                         },
	{TYPE_TIME, "pos_sec.tel_enc_cycle",         sizeof(conf_sram.pos_sec.tel_enc_cycle),                     &conf_sram.pos_sec.tel_enc_cycle                    },

	{TYPE_INT,  "img_pri.active",                sizeof(conf_sram.img_pri.thread_conf.active),                &conf_sram.img_pri.thread_conf.active               },
	{TYPE_TIME, "img_pri.init_delay",            sizeof(conf_sram.img_pri.thread_conf.init_delay),            &conf_sram.img_pri.thread_conf.init_delay           },
	{TYPE_TIME, "img_pri.send_spacing",          sizeof(conf_sram.img_pri.thread_conf.send_spacing),          &conf_sram.img_pri.thread_conf.send_spacing         },
	{TYPE_INT,  "img_pri.sleep_conf.type",       sizeof(conf_sram.img_pri.thread_conf.sleep_conf.type),       &conf_sram.img_pri.thread_conf.sleep_conf.type      },
	{TYPE_INT,  "img_pri.sleep_conf.vbat_thres", sizeof(conf_sram.img_pri.thread_conf.sleep_conf.vbat_thres), &conf_sram.img_pri.thread_conf.sleep_conf.vbat_thres},
	{TYPE_INT,  "img_pri.sleep_conf.vsol_thres", sizeof(conf_sram.img_pri.thread_conf.sleep_conf.vsol_thres), &conf_sram.img_pri.thread_conf.sleep_conf.vsol_thres},
	{TYPE_TIME, "img_pri.cycle",                 sizeof(conf_sram.img_pri.thread_conf.cycle),                 &conf_sram.img_pri.thread_conf.cycle                },
	{TYPE_INT,  "img_pri.pwr",                   sizeof(conf_sram.img_pri.radio_conf.pwr),                    &conf_sram.img_pri.radio_conf.pwr                   },
    {TYPE_INT,  "img_pri.freq",                  sizeof(conf_sram.img_pri.radio_conf.freq),                   &conf_sram.img_pri.radio_conf.freq                  },
	{TYPE_INT,  "img_pri.mod",                   sizeof(conf_sram.img_pri.radio_conf.mod),                    &conf_sram.img_pri.radio_conf.mod                   },
    {TYPE_INT,  "img_pri.rssi",                  sizeof(conf_sram.img_pri.radio_conf.rssi),                   &conf_sram.img_pri.radio_conf.rssi                  },
	{TYPE_INT,  "img_pri.speed",                 sizeof(conf_sram.img_pri.radio_conf.speed),                  &conf_sram.img_pri.radio_conf.speed                 },
	{TYPE_INT,  "img_pri.redundantTx",           sizeof(conf_sram.img_pri.radio_conf.redundantTx),            &conf_sram.img_pri.radio_conf.redundantTx           },
	{TYPE_STR,  "img_pri.call",                  sizeof(conf_sram.img_pri.call),                              &conf_sram.img_pri.call                             },
	{TYPE_STR,  "img_pri.path",                  sizeof(conf_sram.img_pri.path),                              &conf_sram.img_pri.path                             },
	{TYPE_INT,  "img_pri.res",                   sizeof(conf_sram.img_pri.res),                               &conf_sram.img_pri.res                              },
	{TYPE_INT,  "img_pri.quality",               sizeof(conf_sram.img_pri.quality),                           &conf_sram.img_pri.quality                          },
	{TYPE_INT,  "img_pri.buf_size",              sizeof(conf_sram.img_pri.buf_size),                          &conf_sram.img_pri.buf_size                         },

	{TYPE_INT,  "img_sec.active",                sizeof(conf_sram.img_sec.thread_conf.active),                &conf_sram.img_sec.thread_conf.active               },
	{TYPE_TIME, "img_sec.init_delay",            sizeof(conf_sram.img_sec.thread_conf.init_delay),            &conf_sram.img_sec.thread_conf.init_delay           },
	{TYPE_TIME, "img_sec.send_spacing",          sizeof(conf_sram.img_sec.thread_conf.send_spacing),          &conf_sram.img_sec.thread_conf.send_spacing       },
	{TYPE_INT,  "img_sec.sleep_conf.type",       sizeof(conf_sram.img_sec.thread_conf.sleep_conf.type),       &conf_sram.img_sec.thread_conf.sleep_conf.type      },
	{TYPE_INT,  "img_sec.sleep_conf.vbat_thres", sizeof(conf_sram.img_sec.thread_conf.sleep_conf.vbat_thres), &conf_sram.img_sec.thread_conf.sleep_conf.vbat_thres},
	{TYPE_INT,  "img_sec.sleep_conf.vsol_thres", sizeof(conf_sram.img_sec.thread_conf.sleep_conf.vsol_thres), &conf_sram.img_sec.thread_conf.sleep_conf.vsol_thres},
	{TYPE_TIME, "img_sec.cycle",                 sizeof(conf_sram.img_sec.thread_conf.cycle),                 &conf_sram.img_sec.thread_conf.cycle                },
	{TYPE_INT,  "img_sec.pwr",                   sizeof(conf_sram.img_sec.radio_conf.pwr),                    &conf_sram.img_sec.radio_conf.pwr                   },
	{TYPE_INT,  "img_sec.freq",                  sizeof(conf_sram.img_sec.radio_conf.freq),                   &conf_sram.img_sec.radio_conf.freq                  },
	{TYPE_INT,  "img_sec.mod",                   sizeof(conf_sram.img_sec.radio_conf.mod),                    &conf_sram.img_sec.radio_conf.mod                   },
    {TYPE_INT,  "img_sec.rssi",                  sizeof(conf_sram.img_sec.radio_conf.rssi),                   &conf_sram.img_sec.radio_conf.rssi                  },
	{TYPE_INT,  "img_sec.speed",                 sizeof(conf_sram.img_sec.radio_conf.speed),                  &conf_sram.img_sec.radio_conf.speed                 },
	{TYPE_INT,  "img_sec.redundantTx",           sizeof(conf_sram.img_sec.radio_conf.redundantTx),            &conf_sram.img_sec.radio_conf.redundantTx           },
	{TYPE_STR,  "img_sec.call",                  sizeof(conf_sram.img_sec.call),                              &conf_sram.img_sec.call                             },
	{TYPE_STR,  "img_sec.path",                  sizeof(conf_sram.img_sec.path),                              &conf_sram.img_sec.path                             },
	{TYPE_INT,  "img_sec.res",                   sizeof(conf_sram.img_sec.res),                               &conf_sram.img_sec.res                              },
	{TYPE_INT,  "img_sec.quality",               sizeof(conf_sram.img_sec.quality),                           &conf_sram.img_sec.quality                          },
	{TYPE_INT,  "img_sec.buf_size",              sizeof(conf_sram.img_sec.buf_size),                          &conf_sram.img_sec.buf_size                         },

	{TYPE_INT,  "log.active",                    sizeof(conf_sram.log.thread_conf.active),                    &conf_sram.log.thread_conf.active                   },
	{TYPE_TIME, "log.init_delay",                sizeof(conf_sram.log.thread_conf.init_delay),                &conf_sram.log.thread_conf.init_delay               },
	{TYPE_TIME, "log.send_spacing",              sizeof(conf_sram.log.thread_conf.send_spacing),              &conf_sram.log.thread_conf.send_spacing           },
	{TYPE_INT,  "log.sleep_conf.type",           sizeof(conf_sram.log.thread_conf.sleep_conf.type),           &conf_sram.log.thread_conf.sleep_conf.type          },
	{TYPE_INT,  "log.sleep_conf.vbat_thres",     sizeof(conf_sram.log.thread_conf.sleep_conf.vbat_thres),     &conf_sram.log.thread_conf.sleep_conf.vbat_thres    },
	{TYPE_INT,  "log.sleep_conf.vsol_thres",     sizeof(conf_sram.log.thread_conf.sleep_conf.vsol_thres),     &conf_sram.log.thread_conf.sleep_conf.vsol_thres    },
	{TYPE_TIME, "log.cycle",                     sizeof(conf_sram.log.thread_conf.cycle),                     &conf_sram.log.thread_conf.cycle                    },
	{TYPE_INT,  "log.pwr",                       sizeof(conf_sram.log.radio_conf.pwr),                        &conf_sram.log.radio_conf.pwr                       },
	{TYPE_INT,  "log.freq",                      sizeof(conf_sram.log.radio_conf.freq),                       &conf_sram.log.radio_conf.freq                      },
	{TYPE_INT,  "log.mod",                       sizeof(conf_sram.log.radio_conf.mod),                        &conf_sram.log.radio_conf.mod                       },
    {TYPE_INT,  "log.rssi",                      sizeof(conf_sram.log.radio_conf.rssi),                       &conf_sram.log.radio_conf.rssi                      },
	{TYPE_INT,  "log.speed",                     sizeof(conf_sram.log.radio_conf.speed),                      &conf_sram.log.radio_conf.speed                     },
	{TYPE_INT,  "log.redundantTx",               sizeof(conf_sram.log.radio_conf.redundantTx),                &conf_sram.log.radio_conf.redundantTx               },
	{TYPE_STR,  "log.call",                      sizeof(conf_sram.log.call),                                  &conf_sram.log.call                                 },
	{TYPE_STR,  "log.path",                      sizeof(conf_sram.log.path),                                  &conf_sram.log.path                                 },
	{TYPE_INT,  "log.density",                   sizeof(conf_sram.log.density),                               &conf_sram.log.density                              },

	{TYPE_INT,  "aprs.active",                   sizeof(conf_sram.aprs.thread_conf.active),                   &conf_sram.aprs.thread_conf.active                  },
	{TYPE_TIME, "aprs.init_delay",               sizeof(conf_sram.aprs.thread_conf.init_delay),               &conf_sram.aprs.thread_conf.init_delay              },
	/*
	{TYPE_TIME, "aprs.packet_spacing",           sizeof(conf_sram.aprs.rx.thread_conf.packet_spacing),        &conf_sram.aprs.rx.thread_conf.packet_spacing       },
	{TYPE_INT,  "aprs.sleep_conf.type",          sizeof(conf_sram.aprs.rx.thread_conf.sleep_conf.type),       &conf_sram.aprs.rx.thread_conf.sleep_conf.type      },
	{TYPE_INT,  "aprs.sleep_conf.vbat_thres",    sizeof(conf_sram.aprs.rx.thread_conf.sleep_conf.vbat_thres), &conf_sram.aprs.rx.thread_conf.sleep_conf.vbat_thres},
	{TYPE_INT,  "aprs.sleep_conf.vsol_thres",    sizeof(conf_sram.aprs.rx.thread_conf.sleep_conf.vsol_thres), &conf_sram.aprs.rx.thread_conf.sleep_conf.vsol_thres},
	{TYPE_TIME, "aprs.rx.cycle",                 sizeof(conf_sram.aprs.rx.thread_conf.cycle),                 &conf_sram.aprs.rx.thread_conf.cycle                },
    */
	{TYPE_INT,  "aprs.rx.freq",                  sizeof(conf_sram.aprs.rx.radio_conf.freq),                   &conf_sram.aprs.rx.radio_conf.freq                  },
	{TYPE_INT,  "aprs.rx.mod",                   sizeof(conf_sram.aprs.rx.radio_conf.mod),                    &conf_sram.aprs.rx.radio_conf.mod                   },
	{TYPE_INT,  "aprs.rx.speed",                 sizeof(conf_sram.aprs.rx.radio_conf.speed),                  &conf_sram.aprs.tx.radio_conf.speed                 },
    {TYPE_STR,  "aprs.rx.call",                  sizeof(conf_sram.aprs.rx.call),                              &conf_sram.aprs.rx.call                             },

    {TYPE_INT,  "aprs.base.freq",                sizeof(conf_sram.aprs.base.radio_conf.freq),                 &conf_sram.aprs.base.radio_conf.freq                },
    {TYPE_INT,  "aprs.base.pwr",                 sizeof(conf_sram.aprs.base.radio_conf.pwr),                  &conf_sram.aprs.base.radio_conf.pwr                 },
    {TYPE_INT,  "aprs.base.mod",                 sizeof(conf_sram.aprs.base.radio_conf.mod),                  &conf_sram.aprs.base.radio_conf.mod                 },
    {TYPE_INT,  "aprs.base.rssi",                sizeof(conf_sram.aprs.base.radio_conf.rssi),                 &conf_sram.aprs.base.radio_conf.rssi                },
    {TYPE_STR,  "aprs.base.call",                sizeof(conf_sram.aprs.base.call),                            &conf_sram.aprs.base.call                           },

	{TYPE_INT,  "aprs.tx.freq",                  sizeof(conf_sram.aprs.tx.radio_conf.freq),                   &conf_sram.aprs.tx.radio_conf.freq                  },
    {TYPE_INT,  "aprs.tx.pwr",                   sizeof(conf_sram.aprs.tx.radio_conf.pwr),                    &conf_sram.aprs.tx.radio_conf.pwr                   },
    {TYPE_INT,  "aprs.tx.mod",                   sizeof(conf_sram.aprs.tx.radio_conf.mod),                    &conf_sram.aprs.tx.radio_conf.mod                   },
	{TYPE_INT,  "aprs.tx.rssi",                  sizeof(conf_sram.aprs.tx.radio_conf.rssi),                   &conf_sram.aprs.tx.radio_conf.rssi                  },
    {TYPE_STR,  "aprs.tx.call",                  sizeof(conf_sram.aprs.tx.call),                              &conf_sram.aprs.tx.call                             },
    {TYPE_STR,  "aprs.tx.path",                  sizeof(conf_sram.aprs.tx.path),                              &conf_sram.aprs.tx.path                             },
    {TYPE_INT,  "aprs.tx.symbol",                sizeof(conf_sram.aprs.tx.symbol),                            &conf_sram.aprs.tx.symbol                           },
    {TYPE_INT,  "aprs.dig_active",               sizeof(conf_sram.aprs.dig_active),                           &conf_sram.aprs.dig_active                          },
    {TYPE_INT,  "keep_cam_switched_on",          sizeof(conf_sram.keep_cam_switched_on),                      &conf_sram.keep_cam_switched_on                     },
	{TYPE_INT,  "gps_on_vbat",                   sizeof(conf_sram.gps_on_vbat),                               &conf_sram.gps_on_vbat                              },
	{TYPE_INT,  "gps_off_vbat",                  sizeof(conf_sram.gps_off_vbat),                              &conf_sram.gps_off_vbat                             },
	{TYPE_INT,  "gps_onper_vbat",                sizeof(conf_sram.gps_onper_vbat),                            &conf_sram.gps_onper_vbat                           },

	{TYPE_NULL}
};

/*
 * Table of commands that can be embedded in a message.
 */
const APRSCommand aprs_commands[] = {
    {"?aprsd", aprs_send_aprsd_message},
    {"?aprsh", aprs_send_aprsh_message},
    {"?aprsp", aprs_send_position_beacon},
    {"?gpio", aprs_handle_gpio_command},
    {"?gps", aprs_handle_gps_command},
    {"?reset", aprs_execute_system_reset},
    {"?save", aprs_execute_config_save},
    {"?img", aprs_execute_img_command},
    {"?config", aprs_execute_config_command},
    {NULL, NULL}
};

/*
 * @brief       parse arguments from a command string.
 *
 * @return      pointer to next element in string.
 * @retval      NULL if end.
 */
static char *aprs_parse_arguments(char *str, char **saveptr) {
  char *p;

  if (str != NULL)
    *saveptr = str;

  p = *saveptr;
  if (!p) {
    return NULL;
  }

  /* Skipping white space.*/
  p += strspn(p, " \t");

  if (*p == '"') {
    /* If an argument starts with a double quote then its delimiter is another
       quote.*/
    p++;
    *saveptr = strpbrk(p, "\"");
  }
  else {
    /* The delimiter is white space.*/
    *saveptr = strpbrk(p, " \t");
  }

  /* Replacing the delimiter with a zero.*/
  if (*saveptr != NULL) {
    *(*saveptr)++ = '\0';
  }

  return *p != '\0' ? p : NULL;
}

/*
 * @brief       Execute a command in an APRS message.
 * @notes       Known commands are in APRS command table.
 * @notes       Commands themselves return only MSG_OK or MSG_ERROR.
 *
 * @return      result of command.
 * @retval      MSG_OK if the command completed.
 * @retval      MSG_ERROR if there was an error in command execution.
 * @retval      MSG_TIMEOUT if the command was not found in known commands.
 */
static msg_t aprs_cmd_exec(const APRSCommand *acp,
                          char *name,
                          aprs_identity_t *id,
                          int argc,
                          char *argv[]) {

  while (acp->ac_name != NULL) {
    if (strcmp(acp->ac_name, name) == 0) {
      return acp->ac_function(id, argc, argv);
    }
    acp++;
  }
  return MSG_TIMEOUT;
}

/*
 *
 */
void aprs_debug_getPacket(packet_t pp, char* buf, uint32_t len)
{
	// Decode packet
	char rec[127];
	unsigned char *pinfo;
	ax25_format_addrs(pp, rec, sizeof(rec));
	if(ax25_get_info(pp, &pinfo) == 0)
	  return;

    // Print decoded packet
    uint32_t out = chsnprintf(buf, len, "%s", rec);
    for(uint32_t i = 0; pinfo[i]; i++) {
        if(pinfo[i] < 32 || pinfo[i] > 126) {
            out += chsnprintf(&buf[out], len - out, "<0x%02x>", pinfo[i]);
        } else {
            out += chsnprintf(&buf[out], len - out, "%c", pinfo[i]);
        }
    }
}

/**
 * @brief  Transmit APRS position packet.
 * @notes  The comments are filled with:
 * @notes  - Static comment (can be set in config.h)
 * @notes  - Battery voltage in mV
 * @notes  - Solar voltage in mW (if tracker is solar-enabled)
 * @notes  - Temperature in Celcius
 * @notes  - Air pressure in Pascal
 * @notes  - Number of satellites being used
 * @notes  - Number of cycles where GPS has been lost (if applicable in cycle)
 *
 * @param[in] callsign  originator calls sign
 * @param[in] path      path to use
 * @param[in] symbol    symbol for originator
 * @param[in] dataPoint position data object
 *
 * @return    encoded packet object pointer
 * @retval    NULL if encoding failed
 */
packet_t aprs_encode_position(const char *callsign,
                              const char *path, uint16_t symbol,
                              dataPoint_t *dataPoint) {
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

	uint8_t gpsFix = dataPoint->gps_state == GPS_LOCKED1
	    || dataPoint->gps_state == GPS_LOCKED2 ? GSP_FIX_CURRENT : GSP_FIX_OLD;
	uint8_t src = NMEA_SRC_GGA;
	uint8_t origin = ORIGIN_PICO;

	char xmit[256];
	uint32_t len = chsnprintf(xmit, sizeof(xmit), "%s>%s,%s:!",
	                          callsign, APRS_DEVICE_CALLSIGN, path);

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
	uint32_t len2 = base91_encode((uint8_t*)dataPoint,
	                              (uint8_t*)&xmit[len+13],
	                              sizeof(dataPoint_t));

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

	return ax25_from_text(xmit, true);
}

packet_t aprs_encode_data_packet(const char *callsign, const char *path,
                                 char packetType, uint8_t *data)
{
	char xmit[256];
	chsnprintf(xmit, sizeof(xmit), "%s>%s,%s:{{%c%s", callsign,
	           APRS_DEVICE_CALLSIGN, path, packetType, data);

	return ax25_from_text(xmit, true);
}

/**
 * @brief       Transmit message packet
 *
 * @param[in]   originator  call sign of originator of this message
 * @param[in    path        path for message
 * @param[in]   recipient   call sign of recipient
 * @param[in    text        text of the message
 * @param[in]   ack         true if message acknowledgment requested
 */
packet_t aprs_encode_message(const char *originator, const char *path,
                             const char *recipient, const char *text,
                             const bool ack) {
	char xmit[256];
	if((strlen(text) > AX25_MAX_APRS_MSG_LEN)
	    || (strpbrk(text, "|~{") != NULL))
	  /* Invalid message. */
	  return NULL;
	if(!ack)
		chsnprintf(xmit, sizeof(xmit), "%s>%s,%s::%-9s:%s",
		                               originator,
                                       APRS_DEVICE_CALLSIGN,
                                       path,
                                       recipient,
                                       text);
	else
		chsnprintf(xmit, sizeof(xmit), "%s>%s,%s::%-9s:%s{%d",
		                               originator,
                                       APRS_DEVICE_CALLSIGN,
                                       path,
                                       recipient,
                                       text,
                                       ++msg_id);

	return ax25_from_text(xmit, true);
}

/*
 * @brief       Compose an APRSD message
 * @notes       Used by position service.
 *
 * @param[in]   originator  originator of this message
 * @param[in]   path        path to use
 * @param[in]   recipient   identity that requested the message
 */
packet_t aprs_compose_aprsd_message(const char *originator,
                                    const char *path,
                                    const char *recipient) {
	char buf[256] = "Directs=";
	uint32_t out = strlen(buf);
	for(uint8_t i = 0; i < APRS_HEARD_LIST_SIZE; i++) {
		if(heard_list[i].time
		    && heard_list[i].time + TIME_S2I(600) >= chVTGetSystemTime()
		    && heard_list[i].time <= chVTGetSystemTime())
			out += chsnprintf(&buf[out], sizeof(buf)-out, "%s ",
			                  heard_list[i].call);
	}
	buf[out-1] = 0; // Remove last space

	return aprs_encode_message(originator, path, recipient, buf, false);
}

/*
 * @brief       Encode and send an APRSD message
 *
 * @param[in]   id      aprs node identity
 * @param[in]   argc    number of parameters
 * @param[in]   argv    array of pointers to parameter strings
 *
 * @return      result of command
 * @retval      MSG_OK if the command completed.
 * @retval      MSG_ERROR if there was an error.
 */
msg_t aprs_send_aprsd_message(aprs_identity_t *id,
                                     int argc, char *argv[]) {
  (void)argc;
  (void)argv;

  packet_t pp = aprs_compose_aprsd_message(id->call, id->path, id->src);
  if(pp == NULL) {
    TRACE_WARN("RX   > No free packet objects or badly formed message");
    return MSG_ERROR;
  }
  if(!transmitOnRadio(pp,
                  id->freq,
                  0,
                  0,
                  id->pwr,
                  id->mod,
                  id->rssi)) {
    TRACE_ERROR("RX   > Transmit of APRSD failed");
    return MSG_ERROR;
  }
  return MSG_OK;
}

/*
 * @brief       Encode and send an APRSH message
 *
 * @param[in]   id      aprs node identity
 * @param[in]   argc    number of parameters
 * @param[in]   argv    array of pointers to parameter strings
 *
 * @return      result of command
 * @retval      MSG_OK if the command completed.
 * @retval      MSG_ERROR if there was an error.
 */
msg_t aprs_send_aprsh_message(aprs_identity_t *id,
                                 int argc, char *argv[]) {
  if(argc != 1)
    return MSG_ERROR;
  char buf[AX25_MAX_APRS_MSG_LEN + 1];
  uint32_t out = 0;
  strupr(argv[0]);
  for(uint8_t i = 0; i < APRS_HEARD_LIST_SIZE; i++) {
      if(heard_list[i].time && (strncmp(heard_list[i].call, argv[0],
                                        strlen(argv[0])) == 0)) {
        /* Convert time to human readable form. */
        time_secs_t diff = chTimeI2S(chVTTimeElapsedSinceX(heard_list[i].time));
        out = chsnprintf(buf, sizeof(buf),
                         "%s heard %02i:%02i ago",
                          heard_list[i].call, diff/60, diff % 60);
        break;
      }
      if(out == 0) {
        out = chsnprintf(buf, sizeof(buf),
                         "%s not heard", argv[0]);
      }
  }
  packet_t pp = aprs_encode_message(id->call, id->path, id->src, buf, false);
  if(pp == NULL) {
    TRACE_WARN("RX   > No free packet objects or badly formed message");
    return MSG_ERROR;
  }
  if(!transmitOnRadio(pp,
                  id->freq,
                  0,
                  0,
                  id->pwr,
                  id->mod,
                  id->rssi)) {
    TRACE_ERROR("RX   > Transmit of APRSH failed");
    return MSG_ERROR;
  }
  return MSG_OK;
}

/*
 * @brief       Handle GPIO set/clear/query
 *
 * @param[in]   id      aprs node identity
 * @param[in]   argc    number of parameters
 * @param[in]   argv    array of pointers to parameter strings
 *
 * @return      result of command
 * @retval      MSG_OK if the command completed.
 * @retval      MSG_ERROR if there was an error.
 */
msg_t aprs_handle_gpio_command(aprs_identity_t *id,
                                 int argc, char *argv[]) {
  if(argc != 1)
    return MSG_ERROR;

  if(!strcmp(argv[0], "pa8:1")) {
    TRACE_INFO("RX   > Message: GPIO query PA8 HIGH");
    palSetPadMode(GPIOA, 8, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPad(GPIOA, 8);
    return MSG_OK;
  }

  if(!strcmp(argv[0], "pa8:0")) {
    TRACE_INFO("RX   > Message: GPIO query PA8 LOW");
    palSetPadMode(GPIOA, 8, PAL_MODE_OUTPUT_PUSHPULL);
    palClearPad(GPIOA, 8);
    return MSG_OK;
  }

  if(!strcmp(argv[0], "pa8:?")) {
    char buf[AX25_MAX_APRS_MSG_LEN + 1];
    /* TODO: Need to read mode and if not output then report as "input" etc. */
    chsnprintf(buf, sizeof(buf),
                   "PA8 is %s ",
                   (palReadPad(GPIOA, 8) == PAL_HIGH) ? "HIGH" : "LOW");

    packet_t pp = aprs_encode_message(id->call, id->path, id->src, buf, false);
    if(pp == NULL) {
      TRACE_WARN("RX   > No free packet objects or badly formed message");
      return MSG_ERROR;
    }
    if(!transmitOnRadio(pp,
                id->freq,
                0,
                0,
                id->pwr,
                id->mod,
                id->rssi)) {
      TRACE_ERROR("RX   > Transmit of GPIO status failed");
      return MSG_ERROR;
    }
    return MSG_OK;
  }
  return MSG_ERROR;
}

/*
 * @brief       Handle GPS command
 *
 * @param[in]   id      aprs node identity
 * @param[in]   argc    number of parameters
 * @param[in]   argv    array of pointers to parameter strings
 *
 * @return      result of command
 * @retval      MSG_OK if the command completed.
 * @retval      MSG_ERROR if there was an error.
 */
msg_t aprs_handle_gps_command(aprs_identity_t *id,
                                 int argc, char *argv[]) {
  if(argc != 1)
    return MSG_ERROR;

  if(!strcmp(argv[0], "fixed")) {
    TRACE_INFO("RX   > Message: GPS set to fixed location");
    test_gps_enabled = true;
    return MSG_OK;
  }

  if(!strcmp(argv[0], "normal")) {
    TRACE_INFO("RX   > Message: GPS set to normal operation");
    test_gps_enabled = false;
    return MSG_OK;
  }

  if(!strcmp(argv[0], "status")) {
    char buf[AX25_MAX_APRS_MSG_LEN + 1];
    /* TODO: Need to read mode and if not output then report as "input" etc. */
    chsnprintf(buf, sizeof(buf),
                   "GPS is %s",
                   test_gps_enabled ? "fixed" : "normal");

    packet_t pp = aprs_encode_message(id->call, id->path, id->src, buf, false);
    if(pp == NULL) {
      TRACE_WARN("RX   > No free packet objects or badly formed message");
      return MSG_ERROR;
    }
    if(!transmitOnRadio(pp,
                id->freq,
                0,
                0,
                id->pwr,
                id->mod,
                id->rssi)) {
      TRACE_ERROR("RX   > Transmit of GPS status failed");
      return MSG_ERROR;
    }
    return MSG_OK;
  }
  return MSG_ERROR;
}

/**
* @brief       Request for position beacon to be sent
*
* @param[in]   id      aprs node identity
* @param[in]   argc    number of parameters
* @param[in]   argv    array of pointers to parameter strings
*
* @return      result of command
* @retval      MSG_OK if the command completed.
* @retval      MSG_ERROR if there was an error.
*/
msg_t aprs_send_position_beacon(aprs_identity_t *id,
                                int argc, char *argv[]) {
  (void)argv;

  if(argc != 0)
    return MSG_ERROR;

  TRACE_INFO("RX   > Message: Position query");
  dataPoint_t* dataPoint = getLastDataPoint();
  packet_t pp = aprs_encode_position(id->call,
                                     id->path,
                                     id->symbol,
                                     dataPoint);
  if(pp == NULL) {
    TRACE_WARN("RX   > No free packet objects or badly formed message");
    return MSG_ERROR;
  }
  if(!transmitOnRadio(pp,
                      id->freq,
                      0,
                      0,
                      id->pwr,
                      id->mod,
                      id->rssi)) {
    TRACE_ERROR("RX   > Transmit of APRSD failed");
    return MSG_ERROR;
  }
  return MSG_OK;
}

/**
* @brief       Request for system reset
*
* @param[in]   id      aprs node identity
* @param[in]   argc    number of parameters
* @param[in]   argv    array of pointers to parameter strings
*
* @return      result of command
* @retval      MSG_ERROR if there was an error.
*/
msg_t aprs_execute_system_reset(aprs_identity_t *id,
                                int argc, char *argv[]) {
  (void)argv;

  if(argc != 0)
    return MSG_ERROR;

  TRACE_INFO("RX   > Message: System Reset");
  char buf[16];
  chsnprintf(buf, sizeof(buf), "ack%s", id->num);
  packet_t pp = aprs_encode_message(id->call,
                                    id->path,
                                    id->src, buf, false);
  if(pp == NULL) {
    TRACE_WARN("RX   > No free packet objects");
    return MSG_ERROR;
  }
  transmitOnRadio(pp,
                  id->freq,
                  0,
                  0,
                  id->pwr,
                  id->mod,
                  id->rssi);

  chThdSleep(TIME_S2I(10));

  NVIC_SystemReset();
  /* We don't arrive here. */
  return MSG_OK;
}

/*
 * @brief       Handle config command
 *
 * @param[in]   id      aprs node identity
 * @param[in]   argc    number of parameters
 * @param[in]   argv    array of pointers to parameter strings
 *
 * @return      result of command
 * @retval      MSG_OK if the command completed.
 * @retval      MSG_ERROR if there was an error.
 */
msg_t aprs_execute_config_command(aprs_identity_t *id,
                                 int argc, char *argv[]) {
  (void)id;

  if(argc != 2)
    return MSG_ERROR;

  for(uint8_t i=0; command_list[i].type != TYPE_NULL; i++) {
    if(!strncmp(argv[1], command_list[i].name,
                strlen(command_list[i].name))) {

      /* Parameter being changed is in argv[0], new value is in argv[1]. */
      TRACE_INFO("RX   > Message: Configuration Command");
      TRACE_INFO("RX   > %s => %s", argv[1], argv[1]);

      if(command_list[i].type == TYPE_INT
          && command_list[i].size == 1) {
        *((uint8_t*)command_list[i].ptr) = atoi(argv[1]);
      } else if(command_list[i].type == TYPE_INT
          && command_list[i].size == 2) {
        *((uint16_t*)command_list[i].ptr) = atoi(argv[1]);
      } else if(command_list[i].type == TYPE_INT
          && command_list[i].size == 4) {
        *((uint32_t*)command_list[i].ptr) = atoi(argv[1]);
      } else if(command_list[i].type == TYPE_TIME) {
        *((sysinterval_t*)command_list[i].ptr) =
            TIME_MS2I(atoi(argv[2]));
      } else if(command_list[i].type == TYPE_STR) {
        strncpy((char*)command_list[i].ptr, argv[1],
                sizeof(command_list[i].size)-1);
      }
      return MSG_OK;
    } /* Next parameter. */
  } /* Parameter not found. */
  return MSG_ERROR;
}

/**
* @brief       Request configuration save to flash
*
* @param[in]   id      aprs node identity
* @param[in]   argc    number of parameters
* @param[in]   argv    array of pointers to parameter strings
*
* @return      result of command
* @retval      MSG_ERROR if there was an error.
*/
msg_t aprs_execute_config_save(aprs_identity_t *id,
                                int argc, char *argv[]) {
  (void)id;
  (void)argc;
  (void)argv;

  TRACE_INFO("RX   > Message: Config Save");
  conf_sram.magic = CONFIG_MAGIC_UPDATED;
  flashSectorBegin(flashSectorAt(0x08060000));
  flashErase(0x08060000, 0x20000);
  flashWrite(0x08060000, (char*)&conf_sram, sizeof(conf_t));
  flashSectorEnd(flashSectorAt(0x08060000));
  return MSG_OK;
}

/*
 * @brief       Handle Image command
 *
 * @param[in]   id      aprs node identity
 * @param[in]   argc    number of parameters
 * @param[in]   argv    array of pointers to parameter strings
 *
 * @return      result of command
 * @retval      MSG_OK if the command completed.
 * @retval      MSG_ERROR if there was an error.
 */
msg_t aprs_execute_img_command(aprs_identity_t *id,
                                 int argc, char *argv[]) {
  (void)id;
  if(argc < 2)
    return MSG_ERROR;

  if(!strcmp(argv[0], "reject") && argc == 2) {
    if(!strcmp(argv[1], "pri")) {
      reject_pri = true;
      TRACE_INFO("RX   > Message: Image reject pri");
      return MSG_OK;
    }
    if(!strcmp(argv[1], "sec")) {
      reject_sec = true;
      TRACE_INFO("RX   > Message: Image reject sec");
      return MSG_OK;
    }
    return MSG_ERROR;
  }

  if(!strcmp(argv[0], "repeat")) {
    TRACE_INFO("RX   > Message: Image packet repeat request");

    /* Start at arg 2. */
    int c = 2;
    while(c <= argc) {
      uint32_t req = strtol(argv[c++], NULL, 16);
      for(uint8_t i = 0; i < 16; i++) {
        /* Find an empty repeat slot. */
        if(!packetRepeats[i].n_done) {
          packetRepeats[i].image_id = (req >> 16) & 0xFF;
          packetRepeats[i].packet_id = req & 0xFFFF;
          packetRepeats[i].n_done = true;

          TRACE_INFO("RX   > ... Image %3d Packet %3d",
                     packetRepeats[i].image_id,
                     packetRepeats[i].packet_id);
          break;
        } /* Not an empty slot. */
      } /* No more slots. */
    } /* No more image IDs. */
    return MSG_OK;
  }
  /* Unknown parameter. */
  return MSG_ERROR;
}

/*
 * @brief       Decode APRS content and check for message
 *
 * @param[in]   pp    an APRS packet object
 *
 * @return      result of command
 * @retval      true if not a message or not addressed to any node on this device.
 *              in that case the APRS content can be digipeated.
 * @retval      false if this was a message for a node on this device.
 *              in that case the APRS content should not be digipeated.
 */
static bool aprs_decode_message(packet_t pp) {
  // Get Info field
  char src[127];
  unsigned char *pinfo;
  if(ax25_get_info(pp, &pinfo) == 0)
    return false;

  ax25_format_addrs(pp, src, sizeof(src));

  /* Decode destination call sign. */
  char dest[AX25_MAX_ADDR_LEN];
  uint8_t i = 0;

  while(i < sizeof(dest) - 1) {
    if(pinfo[i+1] == ':' || pinfo[i+1] == ' ') {
      dest[i++] = 0;
      break;
    }
    dest[i] = pinfo[i+1];
    i++;
  }
  /* Convert destination call sign to upper case. */
  strupr(dest);

  /* Decode source call sign. */
  for(uint32_t i = 0; i < sizeof(src); i++) {
    if(src[i] == '>') {
      src[i] = 0;
      break;
    }
  }
  /* Convert source call sign to upper case. */
  strupr(src);

  /*
   *  Setup default responding node identity.
   *  Default identity id set to tx.
   */

  aprs_identity_t identity = {0};

  strcpy(identity.src, src);
  strcpy(identity.call, conf_sram.aprs.tx.call);
  /* TODO: define a length for path. */
  strcpy(identity.path, conf_sram.aprs.tx.path);
  identity.symbol = conf_sram.aprs.tx.symbol;
  identity.freq = conf_sram.aprs.tx.radio_conf.freq;
  identity.pwr = conf_sram.aprs.tx.radio_conf.pwr;
  identity.mod = conf_sram.aprs.tx.radio_conf.mod;
  identity.rssi = conf_sram.aprs.tx.radio_conf.rssi;

  /* Check which nodes are enabled to accept aprs messages. */
  bool pos_pri = !strcmp(conf_sram.pos_pri.call, dest)
	        && (conf_sram.pos_pri.aprs_msg)
	        && (conf_sram.pos_pri.thread_conf.active);

  if(pos_pri) {
    strcpy(identity.call, conf_sram.pos_pri.call);
    strcpy(identity.path, conf_sram.pos_pri.path);
    identity.symbol = conf_sram.pos_pri.symbol;
  }

  bool pos_sec = !strcmp(conf_sram.pos_sec.call, dest)
            && (conf_sram.pos_sec.aprs_msg)
            && (conf_sram.pos_sec.thread_conf.active);
  if(pos_sec) {
    strcpy(identity.call, conf_sram.pos_sec.call);
    strcpy(identity.path, conf_sram.pos_sec.path);
    identity.symbol = conf_sram.pos_sec.symbol;
  }

  bool aprs_rx = !strcmp(conf_sram.aprs.rx.call, dest)
            && (conf_sram.aprs.thread_conf.active);
  if(aprs_rx) {
    /* Parameters come from tx. */
  }

  bool aprs_tx = !strcmp(conf_sram.aprs.tx.call, dest)
            && (conf_sram.aprs.thread_conf.active)
            && (conf_sram.aprs.dig_active);
  /* Default already set for tx. */

  /* Check if this is message and address is one of the nodes on this device. */
  if(!((pinfo[10] == ':') && (pos_pri || pos_sec || aprs_rx || aprs_tx))) {
    /*
     * Not a command or not addressed to one of the active nodes on this device.
     * Flag that message should be digipeated.
     */
    return true;
  }

  /* Proceed with command analysis. */
  char msg_id_rx[8] = {0};
  //memset(msg_id_rx, 0, sizeof(msg_id_rx));

  // Cut off control chars
  for(uint16_t i = 11; pinfo[i] != 0
  && i < (AX25_MAX_APRS_MSG_LEN + 11); i++) {
    /* FIXME: Trim trailing spaces before {. */
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

  strcpy(identity.num, msg_id_rx);

  /* Convert command string to lower case. */
  char *astrng = strlwr((char*)&pinfo[11]);

  // Trace
  TRACE_INFO("RX   > Received message from %s (ID=%s): %s [%s]",
             src, msg_id_rx, &pinfo[11], astrng);

  /* Parse arguments. */
  char *lp, *cmd, *tokp;
  char *args[APRS_MAX_MSG_ARGUMENTS];
  lp = aprs_parse_arguments(astrng, &tokp);
  /* The command itself. */
  cmd = lp;
  int n = 0;
  while ((lp = aprs_parse_arguments(NULL, &tokp)) != NULL) {
    if (n >= APRS_MAX_MSG_ARGUMENTS) {
      TRACE_INFO("RX   > Too many APRS command arguments");
      cmd = NULL;
      break;
    }
    args[n++] = lp;
  }

  /* Parse and execute command. */
  msg_t msg = aprs_cmd_exec(aprs_commands, cmd, &identity, n, args);

  if(msg == MSG_TIMEOUT) {
    TRACE_INFO("RX   > Command not found by parser");
  }

  if(msg_id_rx[0]) {
    /* Incoming message ID exists so an ACK or REJ has to be sent. */
    char buf[16];
    chsnprintf(buf, sizeof(buf), "%s%s",
               (msg == MSG_OK) ? "ack" : "rej", msg_id_rx);

    /*
     * Use the receiving node identity as sender.
     *  Don't request acknowledgment.
     */
    packet_t pp = aprs_encode_message(identity.call,
                                      identity.path,
                                      identity.src, buf, false);
    if(pp == NULL) {
      TRACE_WARN("RX   > No free packet objects");
      return false;
    }
    transmitOnRadio(pp,
                    identity.freq,
                    0,
                    0,
                    identity.pwr,
                    identity.mod,
                    identity.rssi);
  }
  /* Flag that the APRS content should not be digipeated. */
  return false;
}

/*
 * Transmit failure will release the packet memory.
 */
static void aprs_digipeat(packet_t pp) {
	if(!dedupe_initialized) {
		dedupe_init(TIME_S2I(10));
		dedupe_initialized = true;
	}

	if(!dedupe_check(pp, 0)) { // Last identical packet older than 10 seconds
		packet_t result = digipeat_match(0, pp, conf_sram.aprs.rx.call,
		                                 conf_sram.aprs.tx.call, alias_re,
		                                 wide_re, 0, preempt, NULL);
		if(result != NULL) { // Should be digipeated
			dedupe_remember(result, 0);
			/* If transmit fails the packet is freed. */
			transmitOnRadio(result,
			                conf_sram.aprs.tx.radio_conf.freq,
		                    0,
		                    0,
                            conf_sram.aprs.tx.radio_conf.pwr,
                            conf_sram.aprs.tx.radio_conf.mod,
                            conf_sram.aprs.tx.radio_conf.rssi);
		}
	}
}

/**
 * Transmit APRS telemetry configuration
 */
packet_t aprs_encode_telemetry_configuration(const char *callsign,
                                             const char *path,
                                             uint8_t type) {
	switch(type) {
		case 0:	return aprs_encode_message(callsign, path, callsign,
		       	        "PARM.Vbat,Vsol,Pbat,Temperature,Airpressure", false);
		case 1: return aprs_encode_message(callsign, path, callsign,
		                "UNIT.V,V,W,degC,Pa", false);
		case 2: return aprs_encode_message(callsign, path, callsign,
		                 "EQNS.0,.001,0,0,.001,0,0,.001,-4.096,0,.1,-100,0,12.5,500", false);
		case 3: return aprs_encode_message(callsign, path, callsign,
		                  "BITS.11111111,", false);
		default: return NULL;
	}
}

/*
 * 
 */
void aprs_decode_packet(packet_t pp) {
  // Get heard callsign
  char call[AX25_MAX_ADDR_LEN];
  int8_t v = -1;
  do {
    v++;
    ax25_get_addr_with_ssid(pp, ax25_get_heard(pp)-v, call);
  } while(ax25_get_heard(pp) - v >= AX25_SOURCE && (!strncmp("WIDE", call, 4) || !strncmp("TRACE", call, 5)));

  // Fill/Update direct list
  sysinterval_t first_time = 0xFFFFFFFF;	// Timestamp of oldest heard list entry
  uint8_t first_id = 0;					// ID of oldest heard list entry

  for(uint8_t i=0; i <= APRS_HEARD_LIST_SIZE; i++) {
    if(i < APRS_HEARD_LIST_SIZE) {
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
  /*
   * Check if the message is for us.
   * Execute any command found in the message.
   * If not then digipeat it.
   */
  if(pinfo[0] == ':') digipeat = aprs_decode_message(pp); // ax25_get_dti(pp)

  // Digipeat packet
  if(conf_sram.aprs.dig_active && digipeat) {
    aprs_digipeat(pp);
  }
}

