/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "nrf52840-ieee.h"
#include "dev/radio.h"
#include "sys/energest.h"
#include "net/netstack.h"
#include "net/packetbuf.h"
#include "net/mac/tsch/tsch.h"
#include "nrf_radio.h"

#include <stdint.h>
#include <stdbool.h>
/*---------------------------------------------------------------------------*/
/* Log configuration */
#include "sys/log.h"

#define LOG_MODULE "nRF52840 IEEE"
#define LOG_LEVEL LOG_LEVEL_ERR
/*---------------------------------------------------------------------------*/
#define NRF52840_CCA_BUSY      0
#define NRF52840_CCA_CLEAR     1
/*---------------------------------------------------------------------------*/
#define NRF52840_RECEIVING_NO  0
#define NRF52840_RECEIVING_YES 1
/*---------------------------------------------------------------------------*/
#define NRF52840_PENDING_NO    0
#define NRF52840_PENDING_YES   1
/*---------------------------------------------------------------------------*/
#define NRF52840_COMMAND_ERR   0
#define NRF52840_COMMAND_OK    1
/*---------------------------------------------------------------------------*/
#define NRF52840_CHANNEL_MIN  11
#define NRF52840_CHANNEL_MAX  26
/*---------------------------------------------------------------------------*/
static uint8_t volatile poll_mode = 0;

/* Store the timestamp of the most recently received packet in rtimer ticks */
static volatile rtimer_clock_t last_frame_timestamp;
/*---------------------------------------------------------------------------*/
PROCESS(nrf52840_ieee_rf_process, "nRF52840 IEEE RF driver");
/*---------------------------------------------------------------------------*/
#ifndef NRF52840_CCA_MODE
#define NRF52840_CCA_MODE RADIO_CCACTRL_CCAMODE_CarrierOrEdMode
#endif

#ifndef NRF52840_CCA_ED_THRESHOLD
#define NRF52840_CCA_ED_THRESHOLD 0x14
#endif

#ifndef NRF52840_CCA_CORR_THRESHOLD
#define NRF52840_CCA_CORR_THRESHOLD 0x14
#endif

#ifndef NRF52840_CCA_CORR_COUNT
#define NRF52840_CCA_CORR_COUNT 0x02
#endif
/*---------------------------------------------------------------------------*/
static uint8_t
get_channel()
{
  return NRF_RADIO->FREQUENCY / 5 + 10;
}
/*---------------------------------------------------------------------------*/
static void
set_channel(uint8_t channel)
{
  NRF_RADIO->FREQUENCY = 5 * (channel - 10);
}
/*---------------------------------------------------------------------------*/
static void
cca_configure(void)
{
  uint32_t ccactrl;

  ccactrl = NRF52840_CCA_MODE;
  ccactrl |= NRF52840_CCA_ED_THRESHOLD << RADIO_CCACTRL_CCAEDTHRES_Pos;
  ccactrl |= NRF52840_CCA_CORR_COUNT << RADIO_CCACTRL_CCACORRCNT_Pos;
  ccactrl |= NRF52840_CCA_CORR_THRESHOLD << RADIO_CCACTRL_CCACORRTHRES_Pos;


  NRF_RADIO->CCACTRL = ccactrl;
}
/*---------------------------------------------------------------------------*/
static radio_value_t
cca_threshold_get(void)
{
  return (NRF_RADIO->CCACTRL = RADIO_CCACTRL_CCACORRTHRES_Msk)
      >> RADIO_CCACTRL_CCACORRTHRES_Pos;
}
/*---------------------------------------------------------------------------*/
/* Netstack API functions */
/*---------------------------------------------------------------------------*/
static int
init(void)
{
  set_channel(IEEE802154_DEFAULT_CHANNEL);

  last_frame_timestamp = 0;

  cca_configure();

  return RADIO_TX_OK;
}
/*---------------------------------------------------------------------------*/
static int
prepare(const void *payload, unsigned short payload_len)
{
  return RADIO_TX_OK;
}
/*---------------------------------------------------------------------------*/
static int
transmit(unsigned short transmit_len)
{
  return RADIO_TX_OK;
}
/*---------------------------------------------------------------------------*/
static int
send(const void *payload, unsigned short payload_len)
{
  prepare(payload, payload_len);
  return transmit(payload_len);
}
/*---------------------------------------------------------------------------*/
static int
read_frame(void *buf, unsigned short bufsize)
{
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
channel_clear(void)
{
  return NRF52840_CCA_CLEAR;
}
/*---------------------------------------------------------------------------*/
static int
receiving_packet(void)
{
  return NRF52840_RECEIVING_YES;
}
/*---------------------------------------------------------------------------*/
static int
pending_packet(void)
{
  return NRF52840_PENDING_YES;
}
/*---------------------------------------------------------------------------*/
static int
on(void)
{
  ENERGEST_ON(ENERGEST_TYPE_LISTEN);
  return NRF52840_COMMAND_OK;
}
/*---------------------------------------------------------------------------*/
static int
off(void)
{
  ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
  return NRF52840_COMMAND_OK;
}
/*---------------------------------------------------------------------------*/
static radio_result_t
get_value(radio_param_t param, radio_value_t *value)
{
  if(!value) {
    return RADIO_RESULT_INVALID_VALUE;
  }

  switch(param) {
  case RADIO_PARAM_POWER_MODE:
    return RADIO_RESULT_OK;
  case RADIO_PARAM_CHANNEL:
    *value = (radio_value_t)get_channel();
    return RADIO_RESULT_OK;
    return RADIO_RESULT_OK;
  case RADIO_PARAM_RX_MODE:
    return RADIO_RESULT_OK;
  case RADIO_PARAM_TX_MODE:
    return RADIO_RESULT_OK;
  case RADIO_PARAM_TXPOWER:
    *value = (radio_value_t)nrf_radio_txpower_get();
    return RADIO_RESULT_OK;
  case RADIO_PARAM_CCA_THRESHOLD:
    *value = cca_threshold_get();
    return RADIO_RESULT_OK;
  case RADIO_PARAM_RSSI:
    *value = (radio_value_t)nrf_radio_rssi_sample_get();
    return RADIO_RESULT_OK;
  case RADIO_PARAM_LAST_RSSI:
    return RADIO_RESULT_OK;
  case RADIO_PARAM_LAST_LINK_QUALITY:
    return RADIO_RESULT_OK;
  case RADIO_CONST_CHANNEL_MIN:
    *value = 11;
    return RADIO_RESULT_OK;
  case RADIO_CONST_CHANNEL_MAX:
    *value = 26;
    return RADIO_RESULT_OK;
  case RADIO_CONST_TXPOWER_MIN:
    *value = (radio_value_t)RADIO_TXPOWER_TXPOWER_Neg40dBm;
    return RADIO_RESULT_OK;
  case RADIO_CONST_TXPOWER_MAX:
    *value = (radio_value_t)RADIO_TXPOWER_TXPOWER_Pos8dBm;
    return RADIO_RESULT_OK;
  case RADIO_CONST_PHY_OVERHEAD:
    *value = (radio_value_t)RADIO_PHY_OVERHEAD;
    return RADIO_RESULT_OK;
  case RADIO_CONST_BYTE_AIR_TIME:
    *value = (radio_value_t)RADIO_BYTE_AIR_TIME;
    return RADIO_RESULT_OK;
  case RADIO_CONST_DELAY_BEFORE_TX:
    *value = (radio_value_t)RADIO_DELAY_BEFORE_TX;
    return RADIO_RESULT_OK;
  case RADIO_CONST_DELAY_BEFORE_RX:
    *value = (radio_value_t)RADIO_DELAY_BEFORE_RX;
    return RADIO_RESULT_OK;
  case RADIO_CONST_DELAY_BEFORE_DETECT:
    *value = (radio_value_t)RADIO_DELAY_BEFORE_DETECT;
    return RADIO_RESULT_OK;

  case RADIO_PARAM_PAN_ID:
  case RADIO_PARAM_16BIT_ADDR:
  default:
    return RADIO_RESULT_NOT_SUPPORTED;
  }
}
/*---------------------------------------------------------------------------*/
static radio_result_t
set_value(radio_param_t param, radio_value_t value)
{
  switch(param) {
  case RADIO_PARAM_POWER_MODE:
    if(value == RADIO_POWER_MODE_ON) {
      on();
      return RADIO_RESULT_OK;
    }
    if(value == RADIO_POWER_MODE_OFF) {
      off();
      return RADIO_RESULT_OK;
    }
    return RADIO_RESULT_INVALID_VALUE;
  case RADIO_PARAM_CHANNEL:
    if(value < NRF52840_CHANNEL_MIN ||
       value > NRF52840_CHANNEL_MAX) {
      return RADIO_RESULT_INVALID_VALUE;
    }
    set_channel(value);
    return RADIO_RESULT_OK;
  case RADIO_PARAM_RX_MODE:
    return RADIO_RESULT_OK;
  case RADIO_PARAM_TX_MODE:
    return RADIO_RESULT_OK;
  case RADIO_PARAM_TXPOWER:
    nrf_radio_txpower_set(value);
    return RADIO_RESULT_OK;
  case RADIO_PARAM_CCA_THRESHOLD:
    return RADIO_RESULT_OK;

  case RADIO_PARAM_SHR_SEARCH:
  case RADIO_PARAM_PAN_ID:
  case RADIO_PARAM_16BIT_ADDR:
  default:
    return RADIO_RESULT_NOT_SUPPORTED;
  }
}
/*---------------------------------------------------------------------------*/
static radio_result_t
get_object(radio_param_t param, void *dest, size_t size)
{
  if(param == RADIO_PARAM_LAST_PACKET_TIMESTAMP) {
    if(size != sizeof(rtimer_clock_t) || !dest) {
      return RADIO_RESULT_INVALID_VALUE;
    }
    *(rtimer_clock_t *)dest = last_frame_timestamp;
    return RADIO_RESULT_OK;
  }

#if MAC_CONF_WITH_TSCH
  if(param == RADIO_CONST_TSCH_TIMING) {
    if(size != sizeof(uint16_t *) || !dest) {
      return RADIO_RESULT_INVALID_VALUE;
    }
    /* Assigned value: a pointer to the TSCH timing in usec */
    *(const uint16_t **)dest = tsch_timeslot_timing_us_10000;
    return RADIO_RESULT_OK;
  }
#endif /* MAC_CONF_WITH_TSCH */

  /* The radio does not support h/w frame filtering based on addresses */
  return RADIO_RESULT_NOT_SUPPORTED;
}
/*---------------------------------------------------------------------------*/
static radio_result_t
set_object(radio_param_t param, const void *src, size_t size)
{
  /* The radio does not support h/w frame filtering based on addresses */
  return RADIO_RESULT_NOT_SUPPORTED;
}
/*---------------------------------------------------------------------------*/
const struct radio_driver nrf52840_ieee_driver = {
  init,
  prepare,
  transmit,
  send,
  read_frame,
  channel_clear,
  receiving_packet,
  pending_packet,
  on,
  off,
  get_value,
  set_value,
  get_object,
  set_object
};
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(nrf52840_ieee_rf_process, ev, data)
{
  int len;
  PROCESS_BEGIN();

  while(1) {
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    do {
      watchdog_periodic();
      packetbuf_clear();
      len = NETSTACK_RADIO.read(packetbuf_dataptr(), PACKETBUF_SIZE);

      if(len > 0) {
        packetbuf_set_datalen(len);

        NETSTACK_MAC.input();
      }
    } while(len > 0);
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
void
interrupt_handler(void)
{
  /*
   * ToDo: Needs to store the SFD reception timestamp, but only after a frame
   * has been received fully.
   */
  last_frame_timestamp = RTIMER_NOW();

  if(!poll_mode) {
    process_poll(&nrf52840_ieee_rf_process);
  }
}
/*---------------------------------------------------------------------------*/