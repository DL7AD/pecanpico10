#ifndef __IMG_H__
#define __IMG_H__

#include "ch.h"
#include "hal.h"
#include "types.h"

void start_image_thread(thd_img_conf_t *conf);
uint32_t takePicture(uint8_t* buffer, uint32_t size, resolution_t resolution, bool enableJpegValidation);
extern mutex_t camera_mtx;
extern uint8_t gimage_id;

#endif

