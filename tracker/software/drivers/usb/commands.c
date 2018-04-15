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

static uint8_t usb_buffer[16*1024] __attribute__((aligned(32))); // USB image buffer

const ShellCommand commands[] = {
	{"set_trace_level", usb_cmd_set_trace_level},
    {"trace", usb_cmd_set_trace_level}, /* Short form alias. */
	{"picture", usb_cmd_printPicture},
	{"print_log", usb_cmd_printLog},
	{"config", usb_cmd_printConfig},
	{"aprs_message", usb_cmd_send_aprs_message},
	{"msg", usb_cmd_send_aprs_message}, /* Short form alias. */
    {"test_gps", usb_cmd_set_test_gps},
#if SHELL_CMD_MEM_ENABLED == TRUE
    {"heap", usb_cmd_ccm_heap},
#else
    {"mem", usb_cmd_ccm_heap},
#endif
	{NULL, NULL}
};

void usb_cmd_set_test_gps(BaseSequentialStream *chp, int argc, char *argv[])
{
    if(argc < 1)
    {
        chprintf(chp, "Current test GPS: %s\r\n", test_gps_enabled ? "on" : "off");
        return;
    }
    extern bool test_gps_enabled;
    test_gps_enabled = atoi(argv[0]);
}

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

#if USE_CCM_FOR_PKT_HEAP == TRUE
  extern memory_heap_t *ccm_heap;

  n = chHeapStatus(ccm_heap, &total, &largest);
  chprintf(chp, SHELL_NEWLINE_STR"CCM Heap"SHELL_NEWLINE_STR);
  chprintf(chp, "heap fragments   : %u"SHELL_NEWLINE_STR, n);
  chprintf(chp, "heap free total  : %u bytes"SHELL_NEWLINE_STR, total);
  chprintf(chp, "heap free largest: %u bytes"SHELL_NEWLINE_STR, largest);
#endif
}

void usb_cmd_set_trace_level(BaseSequentialStream *chp, int argc, char *argv[])
{
	if(argc < 1)
	{
	  if(atoi(argv[0]) > 5) {
	      chprintf(chp, "Choose a level from 0 - 5\r\n");
      }
      chprintf(chp, "Current trace level: %i\r\n", usb_trace_level);
      chprintf(chp, "Level 0: None\r\n");
      chprintf(chp, "Level 1: Errors\r\n");
      chprintf(chp, "Level 2: Errors + Warnings\r\n");
      chprintf(chp, "Level 3: Errors + Warnings + Monitor\r\n");
      chprintf(chp, "Level 4: Errors + Warnings + Monitor + Info\r\n");
      chprintf(chp, "Level 5: Errors + Warnings + Monitor + Info + Debug\r\n");
      return;
	}
	usb_trace_level = atoi(argv[0]);
}

void usb_cmd_printPicture(BaseSequentialStream *chp, int argc, char *argv[])
{
	(void)argc;
	(void)argv;

	// Take picture
	uint32_t size_sampled = takePicture(usb_buffer, sizeof(usb_buffer), RES_QVGA, false);

	// Transmit image via USB
	if(size_sampled)
	{
		bool start_detected = false;
		for(uint32_t i=0; i<size_sampled; i++)
		{
			// Look for APP0 instead of SOI because SOI is lost sometimes, but we can add SOI easily later on
			if(!start_detected && usb_buffer[i] == 0xFF && usb_buffer[i+1] == 0xE0) {
				start_detected = true;
				TRACE_INFO("DATA > image/jpeg,%d", size_sampled-i+2); // Flag the data on serial output
				streamPut(chp, 0xFF);
				streamPut(chp, 0xD8);
			}
			if(start_detected)
				streamPut(chp, usb_buffer[i]);
		}
		if(!start_detected)
		{
			TRACE_INFO("DATA > image/jpeg,0");
			TRACE_INFO("DATA > text/trace,no SOI flag found");
		}

	} else { // Camera error

		TRACE_INFO("DATA > image,jpeg,0");
		TRACE_INFO("DATA > error,no camera found");

	}
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
		"pac_vbat,pac_vsol,pac_pbat,pac_psol"
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
						dp->sen_i1_press/10, dp->sen_i1_press%10, dp->sen_i1_temp/100, dp->sen_i1_temp%100, dp->sen_i1_hum/10, dp->sen_i1_hum%10,
						dp->sen_e1_press/10, dp->sen_e1_press%10, dp->sen_e1_temp/100, dp->sen_e1_temp%100, dp->sen_e1_hum/10, dp->sen_e1_hum%10,
						dp->sen_e2_press/10, dp->sen_e2_press%10, dp->sen_e2_temp/100, dp->sen_e2_temp%100, dp->sen_e2_hum/10, dp->sen_e2_hum%10,
						dp->stm32_temp/100, dp->stm32_temp%100, dp->si446x_temp/100, dp->si446x_temp%100,
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
		chprintf(chp, "Argument missing!\r\n");
		chprintf(chp, "Argument 1: Destination\r\n");
		chprintf(chp, "Argument 2: Message\r\n");
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

	/* Send with ack request (last arg false). */
	packet_t packet = aprs_encode_message(conf_sram.aprs.tx.call,
	                                      conf_sram.aprs.tx.path,
	                                      argv[0], m, false);
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
                    conf_sram.aprs.tx.radio_conf.rssi);

	chprintf(chp, "Message sent!\r\n");
/*
  (void)argc;
    (void)argv;
    chprintf(chp, "TODO: Not implemented\r\n");*/
}

