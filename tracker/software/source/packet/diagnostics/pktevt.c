/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

#include "pktconf.h"

event_listener_t pkt_el;
static bool trace_enabled = false;


void pktEnableEventTrace(radio_unit_t radio) {
  packet_svc_t *handler = pktGetServiceObject(radio);
  chEvtRegister(pktGetEventSource(handler), &pkt_el,
                PKT_DIAGNOSTIC_EVENT_CODE);
  trace_enabled = true;
}

void pktDisableEventTrace(radio_unit_t radio) {
  packet_svc_t *handler = pktGetServiceObject(radio);
  trace_enabled = false;
  chEvtUnregister(pktGetEventSource(handler), &pkt_el);
}

/*
 * TODO:
 * - Refactor and add severity categories filtering
 * - Add packet service listener object per radio.
 */
void pktTraceEvents() {
  if(!trace_enabled)
    return;
eventmask_t evt = chEvtGetAndClearEvents(EVENT_MASK(PKT_DIAGNOSTIC_EVENT_CODE));
  if(evt) {
    eventflags_t flags = chEvtGetAndClearFlags(&pkt_el);
    if(flags & EVT_PWM_QUEUE_FULL) {
      TRACE_WARN("PKT  > PWM queue full. Possible jamming or RSSI set too low");
    }
    if(flags & EVT_PWM_JAMMING_RESET) {
      TRACE_INFO("PKT  > PWM jamming guard timeout released");
    }
    if(flags & EVT_PWM_FIFO_EMPTY) {
      TRACE_WARN("PKT  > PWM FIFO exhausted");
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
      TRACE_INFO("PKT  > RSSI validated but PWM data from radio failed");
    }
    if(flags & EVT_PKT_FAILED_CB_THD) {
      TRACE_ERROR("PKT  > Failed to create RX callback thread");
    }
    if(flags & EVT_PWM_INVALID_SWAP) {
      TRACE_DEBUG("PKT  > Invalid in-band buffer swap");
    }
    if(flags & EVT_PWM_STREAM_TIMEOUT) {
      TRACE_WARN("PKT  > PWM stream timeout");
    }
    if(flags & EVT_AFSK_START_FAIL) {
      TRACE_ERROR("PKT  > AFSK decoder failed to start");
    }
    if(flags & EVT_PKT_BUFFER_MGR_FAIL) {
      TRACE_ERROR("PKT  > Unable to start packet RX buffer");
    }
    if(flags & EVT_PKT_CBK_MGR_FAIL) {
      TRACE_ERROR("PKT  > Unable to start packet RX callback manager");
    }
  }
}
