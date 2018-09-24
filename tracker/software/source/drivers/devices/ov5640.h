/**
  * This is the OV2640 driver
  */

/**
 * @file    ov5640.h
 * @brief   Driver for OV5640 camera.
 * @notyes  Implements a pseudo DCMI capture capability.
 *
 * @addtogroup drivers
 * @{
 */

#ifndef __OV5640_H__
#define __OV5640_H__

#include "ch.h"
#include "hal.h"
#include "types.h"

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

#define OV5640_I2C_ADR		        0x3C

#if !defined(PDCMI_USE_DMA_DBM)
#define PDCMI_USE_DMA_DBM           FALSE
#endif

#define PDCMI_DMA_DBM_PAGE_SIZE     512
#define PDCMI_DMA_FIFO_BURST_ALIGN  16
#define PDCMI_DMA_IRQ_PRIO          2

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

typedef enum {
  PDCMI_NOT_ACTIVE = 0,
  PDCMI_WAIT_VSYNC,
  PDCMI_CAPTURE_ACTIVE,
  PDCMI_DMA_ERROR,
  PDCMI_DMA_UNKNOWN_IRQ,
  PDCMI_DMA_END_BUFFER,
  PDCMI_DMA_COUNT_END,
  PDCMI_VSYNC_END,
  PDCMI_CAPTURE_TIMEOUT
} pdcmi_state_t;

typedef struct pdcmiControl {
  const stm32_dma_stream_t  *dmastp;
  TIM_TypeDef               *timer;
  uint16_t                  page_size;
  uint8_t                   *page_address;
  uint8_t                   *buffer_base;
  uint8_t                   *buffer_limit;
  int16_t                   page_count;
  volatile bool             terminate;
  uint32_t                  dma_flags;
  volatile pdcmi_state_t    pdcmi_state;
  uint32_t                  transfer_count;
  ioline_t                  vsync_line;
} pdcmi_capture_t;

extern binary_semaphore_t pdcmi_sem;

#ifdef __cplusplus
extern "C" {
#endif
uint32_t    OV5640_Snapshot2RAM(uint8_t* buffer, uint32_t size, resolution_t resolution);
uint32_t    OV5640_Capture(uint8_t* buffer, uint32_t size);
void        OV5640_InitGPIO(void);
void        OV5640_TransmitConfig(void);
void        OV5640_SetResolution(resolution_t res);
void        OV5640_init(void);
void        OV5640_deinit(void);
bool        OV5640_isAvailable(void);
void        OV5640_setLightIntensity(void);
uint32_t    OV5640_getLastLightIntensity(void);
uint8_t     OV5640_hasError(void);
msg_t       OV5640_LockPDCMI(void);
void        OV5640_UnlockPDCMI(void);
void        OV5640_GetPDCMILockStateI(void);

#ifdef __cplusplus
}
#endif

#endif /* __OV5640_H__ */

/** @} */
