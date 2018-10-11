#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "debug.h"
#include <stdlib.h>
#include "ctype.h"
#include "image.h"
#include "aprs.h"
#include "radio.h"
#include "commands.h"
#include "pflash.h"
#include "ublox.h"
#include <string.h>
#include <time.h>
#include "ov5640.h"

//static uint8_t usb_buffer[16*1024] __attribute__((aligned(32))); // USB image buffer

const ShellCommand commands[] = {
    {"trace", usb_cmd_set_trace_level},
	{"picture", usb_cmd_printPicture},
	{"print_log", usb_cmd_printLog},
    {"log", usb_cmd_printLog},
	{"config", usb_cmd_printConfig},
	{"msg", usb_cmd_send_aprs_message},

#if SHELL_CMD_MEM_ENABLED == TRUE
    {"heap", usb_cmd_ccm_heap},
#else
    {"mem", usb_cmd_ccm_heap},
#endif
    {"sats", usb_cmd_get_gps_sat_info},
    {"error_list", usb_cmd_get_error_list},
    {"errors", usb_cmd_get_error_list},
    {"time", usb_cmd_time},
    {"radio", usb_cmd_radio},
	{NULL, NULL}
};

/**
 *
 */
void usb_cmd_get_gps_sat_info(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)argv;

  if(argc > 0) {
    shellUsage(chp, "sats");
    return;
  }
  chprintf(chp, "Checking for satellite information\r\n");
  gps_svinfo_t svinfo;
  if(!gps_get_sv_info(&svinfo, sizeof(svinfo))) {
    chprintf(chp, "No information available\r\n");
    return;
  }
  if(svinfo.numCh == 0) {
    chprintf(chp, "No satellites found\r\n");
    return;
  }
  chprintf(chp, "Space Vehicle info iTOW=%d numCh=%02d globalFlags=%d\r\n",
           svinfo.iTOW, svinfo.numCh, svinfo.globalFlags);
  uint8_t i;
  for(i = 0; i < svinfo.numCh; i++) {
    gps_svchn_t *sat = &svinfo.svinfo[i];
    chprintf(chp, "chn=%03d svid=%03d flags=0x%02x quality=%02d"
             " cno=%03d elev=%03d azim=%06d prRes=%06d\r\n",
             sat->chn, sat->svid, sat->flags, sat->flags,
             sat->quality, sat->cno, sat->elev, sat->azim, sat->prRes);
  }
}

/*
 *
 */
void usb_cmd_ccm_heap(BaseSequentialStream *chp, int argc, char *argv[]) {
  size_t n, total, largest;

  (void)argv;
  if (argc > 0) {
    shellUsage(chp, "heap");
    return;
  }

  n = chHeapStatus(NULL, &total, &largest);
  chprintf(chp, "Core free memory : %u bytes"SHELL_NEWLINE_STR,
                                             chCoreGetStatusX());
  chprintf(chp, SHELL_NEWLINE_STR"Core Heap"SHELL_NEWLINE_STR);
  chprintf(chp, "heap fragments   : %u"SHELL_NEWLINE_STR, n);
  chprintf(chp, "heap free total  : %u bytes"SHELL_NEWLINE_STR, total);
  chprintf(chp, "heap free largest: %u bytes"SHELL_NEWLINE_STR, largest);

  extern memory_heap_t *ccm_heap;
  if(ccm_heap == NULL) {
    chprintf(chp, SHELL_NEWLINE_STR"CCM Heap not enabled"SHELL_NEWLINE_STR);
    return;
  }
  extern uint8_t __ram4_free__[];
  extern uint8_t __ram4_end__[];

  chprintf(chp, SHELL_NEWLINE_STR"CCM heap : size %x starts at : %x"SHELL_NEWLINE_STR,
           (__ram4_end__ - __ram4_free__), __ram4_free__);
  n = chHeapStatus(ccm_heap, &total, &largest);
  chprintf(chp, SHELL_NEWLINE_STR"CCM Heap"SHELL_NEWLINE_STR);
  chprintf(chp, "heap fragments   : %u"SHELL_NEWLINE_STR, n);
  chprintf(chp, "heap free total  : %u bytes"SHELL_NEWLINE_STR, total);
  chprintf(chp, "heap free largest: %u bytes"SHELL_NEWLINE_STR, largest);
}

void usb_cmd_set_trace_level(BaseSequentialStream *chp, int argc, char *argv[])
{
	if(argc < 1)
	{
	  if(atoi(argv[0]) > 5) {
	      chprintf(chp, "Choose a level from 0 - 5\r\n");
      }
      chprintf(chp, "Current trace level: %i\r\n", current_trace_level);
      chprintf(chp, "Level 0: None\r\n");
      chprintf(chp, "Level 1: Errors\r\n");
      chprintf(chp, "Level 2: Errors + Warnings\r\n");
      chprintf(chp, "Level 3: Errors + Warnings + Monitor\r\n");
      chprintf(chp, "Level 4: Errors + Warnings + Monitor + Info\r\n");
      chprintf(chp, "Level 5: Errors + Warnings + Monitor + Info + Debug\r\n");
      return;
	}
	current_trace_level = atoi(argv[0]);
}

void usb_cmd_printPicture(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void)argc;
    (void)argv;
#define PKT_PICTURE_CMD_BUFFER_SIZE (16 * 1024)

    uint8_t *buffer = chHeapAllocAligned(NULL, PKT_PICTURE_CMD_BUFFER_SIZE,
                                         PDCMI_DMA_FIFO_BURST_ALIGN);
    if(buffer == NULL) {
      chprintf(chp, "DATA > image,jpeg,0\r\n");
      chprintf(chp, "DATA > error,no capture memory\r\n");
      return;
    }
    /*
     * Take picture.
     * Status is returned in msg.
     * MSG_OK = capture success.
     * MSG_RESET = no camera found
     * MSG_TIMEOUT = capture failed.
     */
    size_t size_sampled;
    msg_t msg = takePicture(buffer, PKT_PICTURE_CMD_BUFFER_SIZE, RES_QVGA,
                            &size_sampled, false);

    // Transmit image via USB
    if(msg == MSG_OK)
    {
        bool start_detected = false;
        for(size_t i = 0; i < size_sampled; i++)
        {
            // Look for APP0 instead of SOI because SOI is lost sometimes, but we can add SOI easily later on
            if(!start_detected && buffer[i] == 0xFF && buffer[i+1] == 0xE0) {
                start_detected = true;
                chprintf(chp, "DATA > image/jpeg,%d\r\n", size_sampled-i+2); // Flag the data on serial output
                streamPut(chp, 0xFF);
                streamPut(chp, 0xD8);
            }
            if(start_detected)
                streamPut(chp, buffer[i]);
        }
        if(!start_detected)
        {
            chprintf(chp, "DATA > image/jpeg,0\r\n");
            chprintf(chp, "DATA > text/trace,no SOI flag found\r\n");
            chHeapFree(buffer);
            return;
        }
        chprintf(chp, "DATA > image,jpeg,0\r\n");
        chprintf(chp, "DATA > end,image end\r\n");
        chHeapFree(buffer);
        return;

    } else if(msg == MSG_RESET) { // Camera error
        chprintf(chp, "DATA > image,jpeg,0\r\n");
        chprintf(chp, "DATA > error,no camera found\r\n");
        chHeapFree(buffer);
        return;
    } else if(msg == MSG_TIMEOUT) {
      /* MSG_TIMEOUT. */
      chprintf(chp, "DATA > image,jpeg,0\r\n");
      chprintf(chp, "DATA > error,capture failed\r\n");
      chHeapFree(buffer);
    }
    return;
}

void usb_cmd_printLog(BaseSequentialStream *chp, int argc, char *argv[])
{
	(void)argc;
	(void)argv;

	chprintf(chp,
		"reset,id,sys_time,gps_state,gps_time,gps_pdop,"
		"lat,lon,"
		"alt,sats,ttff,"
		"adc_vbat,adc_vsol,"
		"pac_vbat,pac_vsol,pac_pbat,pac_psol,"
		"press_i1,temp_i1,hum_i1,"
		"press_e1,temp_e1,hum_e1,"
		"press_e2,temp_e2,hum_e2,"
		"temp_stm32,temp_si446x,"
		"light,sys_error\r\n");

	dataPoint_t *dp;
	for(uint16_t i=0; (dp = flash_getLogBuffer(i)) != NULL; i++)
		if(dp->id != 0xFFFFFFFF)
		{
			chprintf(	chp,
						"%d,%d,%d,%d,%d,%d,"
						"%d.%05d,%d.%05d,"
						"%d,%d,%d,"
						"%d.%03d,%d.%03d,"
						"%d.%03d,%d.%03d,%d,%d,"
						"%d.%01d,%02d.%02d,%02d.%01d,"
						"%d.%01d,%02d.%02d,%02d.%01d,"
						"%d.%01d,%02d.%02d,%02d.%01d,"
						"%02d.%02d,%02d.%02d,"
						"%d,%08x\r\n",
						dp->reset, dp->id, dp->sys_time, dp->gps_state, dp->gps_time, dp->gps_pdop,
						dp->gps_lat/10000000, (dp->gps_lat > 0 ? 1:-1)*(dp->gps_lat/100)%100000, dp->gps_lon/10000000, (dp->gps_lon > 0 ? 1:-1)*(dp->gps_lon/100)%100000,
						dp->gps_alt, dp->gps_sats, dp->gps_ttff,
						dp->adc_vbat/1000, (dp->adc_vbat%1000), dp->adc_vsol/1000, (dp->adc_vsol%1000),
						dp->adc_vbat/1000, (dp->adc_vbat%1000), dp->adc_vsol/1000, (dp->adc_vsol%1000), dp->pac_pbat, dp->pac_psol,
						dp->sen_i1_press/10, dp->sen_i1_press%10, dp->sen_i1_temp/100, abs(dp->sen_i1_temp%100), dp->sen_i1_hum/10, dp->sen_i1_hum%10,
						dp->sen_e1_press/10, dp->sen_e1_press%10, dp->sen_e1_temp/100, abs(dp->sen_e1_temp%100), dp->sen_e1_hum/10, dp->sen_e1_hum%10,
						dp->sen_e2_press/10, dp->sen_e2_press%10, dp->sen_e2_temp/100, abs(dp->sen_e2_temp%100), dp->sen_e2_hum/10, dp->sen_e2_hum%10,
						dp->stm32_temp/100, dp->stm32_temp%100, dp->si446x_temp/100, abs(dp->si446x_temp%100),
						dp->light_intensity, dp->sys_error
			);
		}
}

void usb_cmd_printConfig(BaseSequentialStream *chp, int argc, char *argv[])
{
/*	if(argc < 1)
	{
		chprintf(chp, "Argument missing!\r\n");
		chprintf(chp, "Argument 1: Id of config\r\n");
		return;
	}

	uint8_t id = atoi(argv[0]);
	chprintf(chp, "Config ID=%d\r\n", id);

	chprintf(chp, "Power: %d\r\n", config[id].power);

	if(config[id].frequency.type == FREQ_STATIC) {
		uint32_t freq = config[id].frequency.hz;
		if((freq/1000)*1000 == freq)
			chprintf(chp, "Frequency: %d.%03d MHz\r\n", freq/1000000, (freq%1000000)/1000);
		else
			chprintf(chp, "Frequency: %d.%03d MHz\r\n", freq/1000000, (freq%1000000));
	} else {
		uint32_t freq = getFrequency(&config[id].frequency);
		chprintf(chp, "Frequency: APRS region dependent (currently %d.%03d MHz)\r\n", freq/1000000, (freq%1000000)/1000);
	}

	chprintf(chp, "Modulation: %d\r\n", config[id].modulation);
	chprintf(chp, "Initial Delay: %d\r\n", config[id].init_delay);
	chprintf(chp, "Packet Spacing: %d\r\n", config[id].packet_spacing);
	chprintf(chp, "Sleep config: xx\r\n");
	chprintf(chp, "Trigger config: xx\r\n");

	chprintf(chp, "Modulation config: xx\r\n");

	chprintf(chp, "Protocol config: xx\r\n");

	chprintf(chp, "SSDV config: xx\r\n");

	chprintf(chp, "Watchdog timeout: %d\r\n", config[id].wdg_timeout);*/

	(void)argc;
	(void)argv;
	chprintf(chp, "TODO: Not implemented\r\n");
}

void usb_cmd_send_aprs_message(BaseSequentialStream *chp, int argc, char *argv[])
{
	if(argc < 2)
	{
	    shellUsage(chp, "msg destination message");
		return;
	}

    char *s = argv[0];
    while(*s) {
      *s = toupper((uint8_t) *s);
      s++;
    }

	chprintf(chp, "Destination: %s\r\n", argv[0]);

	char m[50] = {'\0'};
	for(uint8_t i = 1; i < argc; i++) {
	  strcat(m, argv[i]);
	  if(i < argc - 1)
	    strcat(m, (char *)" ");
	}

	chprintf(chp, "Message: %s\r\n", m);

	/* Send with ack request (last arg true). */
	packet_t packet = aprs_format_transmit_message(conf_sram.aprs.tx.call,
	                                      conf_sram.aprs.tx.path,
	                                      argv[0], m, true);
    if(packet == NULL) {
      TRACE_WARN("CMD  > No free packet objects");
      return;
    }
	transmitOnRadio(packet,
	                conf_sram.aprs.tx.radio_conf.freq,
                    0,
                    0,
                    conf_sram.aprs.tx.radio_conf.pwr,
                    conf_sram.aprs.tx.radio_conf.mod,
                    conf_sram.aprs.tx.radio_conf.cca);

	chprintf(chp, "Message sent!\r\n");
}

void usb_cmd_get_error_list(BaseSequentialStream *chp, int argc, char *argv[])
{
	(void)argc;
	(void)argv;

	uint8_t cntr = 0;
	for(uint8_t i=0; i<ERROR_LIST_SIZE; i++)
	{
		if(strlen((char*)error_list[i]) > 0) {
			chprintf(chp, "%s\r\n", error_list[i]);
			cntr++;
		}
	}

	if(!cntr) {
		chprintf(chp, "No errors recorded\r\n");
	}
}


/**
 *
 */
void usb_cmd_time(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)argv;

  if(argc > 0 && argc != 2) {
    shellUsage(chp, "time [YYYY-MM-DD HH:MM:SS]");
    return;
  }
  /* Read time from RTC. */
  ptime_t time;
  getTime(&time);
  if(argc == 0) {
    chprintf(chp, "RTC time %04d-%02d-%02d %02d:%02d:%02d\r\n",
                            time.year, time.month, time.day,
                            time.hour, time.minute, time.day);
    chprintf(chp, "\r\nTo set time: time [YYYY-MM-DD HH:MM:SS]\r\n");
    return;
  }
  /*
   *  TODO:
   * - add error checking of values.
   * - allow just date or time as parameter?
   */
  struct tm cdate;
  struct tm ctime;
  strptime(argv[0], "%Y-%m-%d", &cdate);
  strptime(argv[1], "%T", &ctime);
  time.year = cdate.tm_year + 1900;
  time.month = cdate.tm_mon + 1;
  time.day = cdate.tm_mday;
  time.hour = ctime.tm_hour;
  time.minute = ctime.tm_min;
  time.second = ctime.tm_sec;
  setTime(&time);
/*  struct tm {
               int tm_sec;     Seconds (0-60)
               int tm_min;     Minutes (0-59)
               int tm_hour;    Hours (0-23)
               int tm_mday;    Day of the month (1-31)
               int tm_mon;     Month (0-11)
               int tm_year;    Year - 1900
               int tm_wday;    Day of the week (0-6, Sunday = 0)
               int tm_yday;    Day in the year (0-365, 1 Jan = 0)
               int tm_isdst;   Daylight saving time
           };*/
}

/**
 * List type, part ROM rev and patch for radio.
 */
void usb_cmd_radio(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)argv;

  if(argc > 1) {
    shellUsage(chp, "radio [number]");
    return;
  }
  radio_unit_t radio;
  if(argc == 0)
    radio = PKT_RADIO_1;
  else
    radio = atoi(argv[0]);

  int8_t num = pktGetNumRadios();
  if(radio == 0 || radio > num) {
    chprintf(chp, "Invalid radio number %d\r\n", radio);
    return;
  }
  packet_svc_t *handler = pktGetServiceObject(radio);

  chprintf(chp, "Radio %d info: part number %04x, rom revision %02x, "
                   "patch ID %04x\r\n",
                   radio, handler->radio_part,
                   handler->radio_rom_rev, handler->radio_patch);
}
