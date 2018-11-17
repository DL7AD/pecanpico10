/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

#include "pktconf.h"

event_listener_t pkt_el;
event_listener_t afsk_el;
static bool trace_enabled = false;


void pktEnableServiceEventTrace(radio_unit_t radio) {
  packet_svc_t *handler = pktGetServiceObject(radio);
  chEvtRegisterMaskWithFlags(pktGetEventSource(handler), &pkt_el,
                EVENT_MASK(PKT_DIAGNOSTIC_EVENT_CODE),
                ALL_EVENTS);
  trace_enabled = true;
}

void pktDisableServiceEventTrace(radio_unit_t radio) {
  packet_svc_t *handler = pktGetServiceObject(radio);
  trace_enabled = false;
  chEvtUnregister(pktGetEventSource(handler), &pkt_el);
}

void pktEnableDecoderEventTrace(radio_unit_t radio) {
  packet_svc_t *handler = pktGetServiceObject(radio);
  chEvtRegisterMaskWithFlags(pktGetEventSource((AFSKDemodDriver *)handler->rx_link_control), &afsk_el,
                EVENT_MASK(AFSK_DIAGNOSTIC_EVENT_CODE),
                ALL_EVENTS);
}

void pktDisableDecoderEventTrace(radio_unit_t radio) {
  packet_svc_t *handler = pktGetServiceObject(radio);
  chEvtUnregister(pktGetEventSource((AFSKDemodDriver *)handler->rx_link_control), &afsk_el);
}

/*
 * TODO:
 * - Refactor and add severity categories filtering
 * - Add packet service listener object per radio.
 */
void pktTraceServiceEvents() {
  if(!trace_enabled)
    return;
eventmask_t evt = chEvtGetAndClearEvents(
                  EVENT_MASK(PKT_DIAGNOSTIC_EVENT_CODE));
  if(evt & EVENT_MASK(PKT_DIAGNOSTIC_EVENT_CODE)) {
    eventflags_t flags = chEvtGetAndClearFlags(&pkt_el);
    if(flags & EVT_RADIO_CCA_SPIKE) {
      TRACE_DEBUG("PKT  > CCA spike on receive");
    }
#if 0
    if(flags & EVT_RADIO_CCA_GLITCH) {
      TRACE_DEBUG("PKT  > CCA glitch on receive");
    }
#endif
    if(flags & EVT_PWM_ICU_ZERO) {
      TRACE_DEBUG("PKT  > PWM abort due to ICU pulse with zero width");
    }
#if 0
    if(flags & EVT_RAD_STREAM_OPEN) {
      TRACE_DEBUG("PKT  > Radio PWM stream open");
    }
    if(flags & EVT_PWM_ACTIVE) {
      TRACE_DEBUG("PKT  > Radio PWM stream active");
    }
    if(flags & EVT_RADIO_CCA_DROP) {
      TRACE_DEBUG("PKT  > Radio CCA has dropped (de-glitched 6.6 ms)");
    }
    if(flags & EVT_PWM_RADIO_TIMEOUT) {
      TRACE_DEBUG("PKT  > Radio PWM timeout");
    }
    if(flags & EVT_RAD_STREAM_CLOSE) {
      TRACE_DEBUG("PKT  > Radio PWM stream close");
    }
#endif
    if(flags & EVT_PWM_QUEUE_FULL) {
      TRACE_WARN("PKT  > PWM queue full. Possible jamming or RSSI set too low");
    }
    if(flags & EVT_PWM_JAMMING_RESET) {
      TRACE_INFO("PKT  > PWM jamming guard timeout released");
    }
    if(flags & EVT_PWM_FIFO_EMPTY) {
      TRACE_WARN("PKT  > PWM FIFO exhausted. Possible jamming or RSSI set too low");
    }
    if(flags & EVT_PKT_NO_BUFFER) {
      TRACE_WARN("PKT  > AX25 FIFO exhausted");
    }
    if(flags & EVT_PKT_BUFFER_FULL) {
      TRACE_WARN("PKT  > AX25 receive buffer full");
    }
    if(flags & EVT_PWM_QUEUE_ERROR) {
      TRACE_ERROR("PKT  > PWM queue handling error");
    }
    if(flags & EVT_PWM_INVALID_INBAND) {
      TRACE_ERROR("PKT  > Invalid PWM in-band message");
    }
    if(flags & EVT_PWM_NO_DATA) {
      TRACE_INFO("PKT  > RSSI validated but no PWM data from radio");
    }
    if(flags & EVT_PKT_FAILED_CB_THD) {
      TRACE_ERROR("PKT  > Failed to create RX callback thread");
    }
    if(flags & EVT_PWM_INVALID_SWAP) {
      TRACE_DEBUG("PKT  > Invalid in-band buffer swap");
    }
#if 0
    if(flags & EVT_PWM_STREAM_TIMEOUT) {
      TRACE_WARN("PKT  > PWM stream timeout");
    }
#endif
    if(flags & EVT_AFSK_START_FAIL) {
      TRACE_ERROR("PKT  > AFSK decoder failed to start");
    }
    if(flags & EVT_PKT_BUFFER_MGR_FAIL) {
      TRACE_ERROR("PKT  > Unable to start packet RX buffer");
    }
    if(flags & EVT_PKT_CBK_MGR_FAIL) {
      TRACE_ERROR("PKT  > Unable to start packet RX callback manager");
    }
#if 0
    if(flags & EVT_HDLC_RESET_RCVD) {
      TRACE_DEBUG("PKT  > HDLC Reset during frame reception");
    }
    if(flags & EVT_HDLC_OPENING_FLAG) {
      TRACE_DEBUG("PKT  > HDLC opening sequence for frame reception");
    }
    if(flags & EVT_AFSK_PWM_STOP) {
      TRACE_DEBUG("PKT  > PWM stopped stream to AFSK decoder");
    }
#endif
  } /* End if pkt evt. */
}

/*
 * TODO:
 * - Refactor and add severity categories filtering
 * - Add packet service listener object per radio.
 */
void pktTraceDecoderEvents() {
  if(!trace_enabled)
    return;
  eventmask_t evt = chEvtGetAndClearEvents(
                    EVENT_MASK(PKT_DIAGNOSTIC_EVENT_CODE));
  if(evt & EVENT_MASK(AFSK_DIAGNOSTIC_EVENT_CODE)) {
    eventflags_t flags = chEvtGetAndClearFlags(&afsk_el);
    if(flags & EVT_AFSK_PWM_START) {
      TRACE_DEBUG("AFSK > PWM stream active");
    }
  }
}
