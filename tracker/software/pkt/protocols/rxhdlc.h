/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

#ifndef PKT_PROTOCOLS_RXHDLC_H_
#define PKT_PROTOCOLS_RXHDLC_H_

/* HDLC bit pattern definitions. */
#define HDLC_CODE_MASK      0xFFU
#define HDLC_FLAG           0x7EU
#define HDLC_RESET          0x7FU
#define HDLC_IDLE           0xFFU
#define HDLC_ZERO           0x00U

/* Frame bounding. */
#define HDLC_FRAME_MASK_A   0x00FFFFFFU
#define HDLC_FRAME_OPEN_A   0x7E7E7E7EU
#define HDLC_FRAME_MASK_B   0x00FFFFFFU
#define HDLC_FRAME_OPEN_B   0x0000007EU
#define HDLC_FRAME_CLOSE    HDLC_FLAG

/* R$LL encoding. */
#define HDLC_RLL_MASK       0x3FU
#define HDLC_RLL_BIT        0x3EU

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

  #ifdef __cplusplus
  extern "C" {
  #endif
    bool pktExtractHDLCfromAFSK(AFSKDemodDriver *myDriver);
  #ifdef __cplusplus
  }
  #endif

#endif /* PKT_PROTOCOLS_RXHDLC_H_ */
