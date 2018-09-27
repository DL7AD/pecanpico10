#ifndef __IMG_H__
#define __IMG_H__

#include "ch.h"
#include "hal.h"
#include "types.h"

#define PKT_SHOW_TX_THROTTLE_DEBUG  FALSE

typedef struct {
	uint16_t packet_id;
	uint8_t image_id;
	bool n_done;
} ssdv_packet_t;

extern ssdv_packet_t packetRepeats[16];
extern bool reject_pri;
extern bool reject_sec;

void start_image_thread(img_app_conf_t *conf, const char *name);
uint32_t takePicture(uint8_t* buffer, size_t size, resolution_t resolution,
                     size_t *size_sampled, bool enableJpegValidation);
extern mutex_t camera_mtx;
extern uint16_t gimage_id;

#endif

