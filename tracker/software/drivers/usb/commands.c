#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "debug.h"
#include <stdlib.h>
#include "image.h"
#include "aprs.h"
#include "radio.h"
#include "commands.h"
#include "pflash.h"

static uint8_t usb_buffer[16*1024] __attribute__((aligned(32))); // USB image buffer

const ShellCommand commands[] = {
	{"debug", usb_cmd_debugOnUSB},
	{"picture", usb_cmd_printPicture},
	{"print_log", usb_cmd_printLog},
	{"config", usb_cmd_printConfig},
	{"aprs_message", usb_cmd_send_aprs_message},
	{NULL, NULL}
};


void usb_cmd_debugOnUSB(BaseSequentialStream *chp, int argc, char *argv[])
{
	if(argc < 1)
	{
		chprintf(chp, "Argument missing!\r\n");
		chprintf(chp, "Argument 1: 1 for switch on, 0 for switch off\r\n");
		return;
	}

	debug_on_usb = atoi(argv[0]);
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
				TRACE_USB("DATA > image/jpeg,%d", size_sampled-i+2); // Flag the data on serial output
				streamPut(chp, 0xFF);
				streamPut(chp, 0xD8);
			}
			if(start_detected)
				streamPut(chp, usb_buffer[i]);
		}
		if(!start_detected)
		{
			TRACE_USB("DATA > image/jpeg,0");
			TRACE_USB("DATA > text/trace,no SOI flag found");
		}

	} else { // Camera error

		TRACE_USB("DATA > image,jpeg,0");
		TRACE_USB("DATA > error,no camera found");

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
	chprintf(chp, "Destination: %s\r\n", argv[0]);
	chprintf(chp, "Message: %s\r\n", argv[1]);

	packet_t packet = aprs_encode_message(conf_sram.rx.call,
	                                      conf_sram.rx.path,
	                                      argv[0], argv[1], false);
	transmitOnRadio(packet,
	                conf_sram.rx.radio_conf.freq,
                    conf_sram.rx.radio_conf.step,
                    conf_sram.rx.radio_conf.chan,
                    conf_sram.rx.radio_conf.pwr,
                    conf_sram.rx.radio_conf.mod);

	chprintf(chp, "Message sent!\r\n");
}

