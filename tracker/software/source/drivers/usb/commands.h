#ifndef __COMMANDS_H__
#define __COMMANDS_H__

#include "ch.h"
#include "hal.h"

void usb_cmd_set_trace_level(BaseSequentialStream *chp, int argc, char *argv[]);
void usb_cmd_printConfig(BaseSequentialStream *chp, int argc, char *argv[]);
void usb_cmd_printPicture(BaseSequentialStream *chp, int argc, char *argv[]);
void usb_cmd_printLog(BaseSequentialStream *chp, int argc, char *argv[]);
void usb_cmd_command2Camera(BaseSequentialStream *chp, int argc, char *argv[]);
void usb_cmd_send_aprs_message(BaseSequentialStream *chp, int argc, char *argv[]);
void usb_cmd_set_test_gps(BaseSequentialStream *chp, int argc, char *argv[]);
void usb_cmd_ccm_heap(BaseSequentialStream *chp, int argc, char *argv[]);
void usb_cmd_get_gps_sat_info(BaseSequentialStream *chp, int argc, char *argv[]);
void usb_cmd_get_error_list(BaseSequentialStream *chp, int argc, char *argv[]);
void usb_cmd_time(BaseSequentialStream *chp, int argc, char *argv[]);
void usb_cmd_radio(BaseSequentialStream *chp, int argc, char *argv[]);

extern const ShellCommand commands[];

#endif

