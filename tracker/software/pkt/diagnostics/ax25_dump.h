
/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

/**
 * @file    ax25_dump.h
 * @brief   Packet dump utility.
 *
 * @addtogroup DSP
 * @{
 */

#ifndef PKT_PROTOCOLS_AX25_DUMP_H_
#define PKT_PROTOCOLS_AX25_DUMP_H_

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

#define DUMP_LINE_LENGTH 60U

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

typedef enum AX25Dump {
  AX25_DUMP_NONE,
  AX25_DUMP_RAW,
  AX25_DUMP_APRS,
  AX25_DUMP_ALL
} ax25_select_t;

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void pktDumpAX25Frame(ax25char_t *frame_buffer, ax25size_t frame_size,
                        ax25_select_t which);
  void pktDiagnosticOutput(packet_svc_t *packetHandler,
                           pkt_data_object_t *pkt_buffer);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/


#endif /* PKT_PROTOCOLS_AX25_DUMP_H_ */

/** @} */
