/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "nrf52840-ieee.h"
#include "dev/radio.h"
#include "sys/energest.h"
#include "sys/int-master.h"
#include "sys/critical.h"
#include "net/netstack.h"
#include "net/packetbuf.h"
#include "net/mac/tsch/tsch.h"
#include "nrf_radio.h"
#include "nrf_ppi.h"
#include "nrf_timer.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
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
#define ED_RSSISCALE           4
/*---------------------------------------------------------------------------*/
#define FCS_LEN                2
#define MPDU_LEN             127
/*
 * The maximum number of bytes this driver can accept from the MAC layer for
 * transmission or will deliver to the MAC layer after reception. Includes
 * the MAC header and payload, but not the FCS.
 */
#define MAX_PAYLOAD_LEN      (MPDU_LEN - FCS_LEN)

#define ACK_MPDU_MIN_LEN      5
#define ACK_PAYLOAD_MIN_LEN  (ACK_MPDU_MIN_LEN - FCS_LEN)
/*---------------------------------------------------------------------------*/
static volatile bool poll_mode = false;

/*
 * The last frame's RSSI and LQI
 * ToDO: nrf52840 won't write this in the FCS like other radios will. Need to
 * figure out a way to find and store it
 */
static int8_t last_rssi;
static uint8_t last_lqi;

/* Perform CCA before TX */
static uint8_t send_on_cca = RADIO_TX_MODE_SEND_ON_CCA;
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
/*
 * .15.4-compliant CRC:
 *
 * Lenght 2, Initial value 0.
 *
 * Polynomial x^16 + x^12 + x^5 + 1
 * CRCPOLY: 1 00010000 00100001
 */
#define CRC_IEEE802154_LEN             2
#define CRC_IEEE802154_POLY      0x11021
#define CRC_IEEE802154_INIT            0
/*---------------------------------------------------------------------------*/
typedef struct tx_buf_s {
  uint8_t phr;
  uint8_t mpdu[MAX_PAYLOAD_LEN];
} tx_buf_t;

static tx_buf_t tx_buf;
/*---------------------------------------------------------------------------*/
typedef struct rx_buf_s {
  uint8_t phr;
  uint8_t mpdu[MPDU_LEN];
} rx_buf_t;

static rx_buf_t rx_buf;
/*---------------------------------------------------------------------------*/
typedef struct cca_cfg_s {
  uint8_t cca_mode;
  uint8_t cca_corr_threshold;
  uint8_t cca_corr_count;
  uint8_t ed_threshold;
} cca_cfg_t;

static cca_cfg_t cca_config = {
  .cca_mode = NRF52840_CCA_MODE,
  .cca_corr_threshold = NRF52840_CCA_CORR_THRESHOLD,
  .cca_corr_count = NRF52840_CCA_CORR_COUNT,
  .ed_threshold = NRF52840_CCA_ED_THRESHOLD,
};
/*---------------------------------------------------------------------------*/
static bool
radio_is_powered(void)
{
  return NRF_RADIO->POWER == 0 ? false : true;
}
/*---------------------------------------------------------------------------*/
static uint8_t
get_channel(void)
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
cca_reconfigure(void)
{
  uint32_t ccactrl;

  ccactrl = cca_config.cca_mode;
  ccactrl |= cca_config.ed_threshold << RADIO_CCACTRL_CCAEDTHRES_Pos;
  ccactrl |= cca_config.cca_corr_count << RADIO_CCACTRL_CCACORRCNT_Pos;
  ccactrl |= cca_config.cca_corr_threshold << RADIO_CCACTRL_CCACORRTHRES_Pos;

  NRF_RADIO->CCACTRL = ccactrl;
}
/*---------------------------------------------------------------------------*/
static void
crc_init(void)
{
  /*
   * Initialise the CRC engine in .15.4 mode:
   * - Length: 2 bytes
   * - Polynomial:
   * - Initial value: 0
   */
  nrf_radio_crc_configure(CRC_IEEE802154_LEN, NRF_RADIO_CRC_ADDR_IEEE802154,
                          CRC_IEEE802154_POLY);

  nrf_radio_crcinit_set(CRC_IEEE802154_INIT);
}
/*---------------------------------------------------------------------------*/
static void
packet_init(void)
{
  /* Configure packet format for .15.4 */
  nrf_radio_packet_conf_t conf;

  memset(&conf, 0, sizeof(conf));

  conf.lflen = 8; /* Length field, in bits */
  conf.s1incl = false;
  conf.plen = NRF_RADIO_PREAMBLE_LENGTH_32BIT_ZERO;

  /* The Nordic driver uses true for crcinc, but this does not make sense */
  conf.crcinc = false;
  conf.big_endian = false;
  conf.whiteen = false;
  conf.maxlen = MPDU_LEN;

  nrf_radio_packet_configure(&conf);
}
/*---------------------------------------------------------------------------*/
static void
setup_interrupts(void)
{
  int_master_status_t stat;
  nrf_radio_int_mask_t interrupts = 0;

  stat = critical_enter();

  if(!poll_mode) {
    nrf_radio_event_clear(NRF_RADIO_EVENT_CRCOK);
    interrupts |= NRF_RADIO_INT_CRCOK_MASK;
  }

  if(interrupts) {
    NVIC_ClearPendingIRQ(RADIO_IRQn);
    nrf_radio_int_enable(interrupts);
    NVIC_EnableIRQ(RADIO_IRQn);
  }

  critical_exit(stat);
}
/*---------------------------------------------------------------------------*/
static void
set_poll_mode(bool enable)
{
  poll_mode = enable;
  setup_interrupts();
}
/*---------------------------------------------------------------------------*/
static void
rx_buf_clear(void)
{
  memset(&rx_buf, 0, sizeof(rx_buf));
}
/*---------------------------------------------------------------------------*/
static void
rx_events_clear()
{
  nrf_radio_event_clear(NRF_RADIO_EVENT_FRAMESTART);
  nrf_radio_event_clear(NRF_RADIO_EVENT_END);
  nrf_radio_event_clear(NRF_RADIO_EVENT_CRCERROR);
  nrf_radio_event_clear(NRF_RADIO_EVENT_CRCOK);
}
/*---------------------------------------------------------------------------*/
/*
 * Powering off the peripheral will reset all registers to default values
 * This function here must be called at every power on to set the radio in a
 * known state
 */
static void
configure(void)
{
  nrf_radio_mode_set(NRF_RADIO_MODE_IEEE802154_250KBIT);

  set_channel(IEEE802154_DEFAULT_CHANNEL);

  cca_reconfigure();

  /* Initialise the CRC engine in .15.4 mode */
  crc_init();

  /* Initialise the packet format */
  packet_init();

  /*
   * MODECNF: Fast ramp up, DTX=center
   * The Nordic driver is using DTX=0, but this is against the PS (v1.1 p351)
   */
  nrf_radio_modecnf0_set(true, RADIO_MODECNF0_DTX_Center);

  /* Enabled interrupts, if applicable */
  setup_interrupts();
}
/*---------------------------------------------------------------------------*/
static void
power_on_and_configure(void)
{
  nrf_radio_power_set(true);
  configure();
}
/*---------------------------------------------------------------------------*/
/* The caller must first make sure the radio is powered and configured */
static void
enter_rx(void)
{
  /* Prepare the RX buffer */
  memset(&rx_buf, 0, sizeof(rx_buf));
  nrf_radio_packetptr_set(&rx_buf);

  /* Trigger RXEN */
  nrf_radio_task_trigger(NRF_RADIO_TASK_RXEN);

  /* Block till RXRU is done */
  while(!nrf_radio_event_check(NRF_RADIO_EVENT_RXREADY));

  /* Trigger the Start task */
  nrf_radio_task_trigger(NRF_RADIO_TASK_START);
}
/*---------------------------------------------------------------------------*/
/* Retrieve an RSSI sample. The radio must be in RX mode */
static int8_t
rssi_read(void)
{
  uint8_t rssi_sample;

  nrf_radio_task_trigger(NRF_RADIO_TASK_RSSISTART);

  while(nrf_radio_event_check(NRF_RADIO_EVENT_RSSIEND) == false);
  nrf_radio_event_clear(NRF_RADIO_EVENT_RSSIEND);

  rssi_sample = nrf_radio_rssi_sample_get();

  return -((int8_t)rssi_sample);
}
/*---------------------------------------------------------------------------*/
/* ToDo:
 * - Deal with the conversion between CC and rtimer ticks
 * - Deal with the time delta between SFD reception and EVENTS_END
 */
#define CONVERT(z) (z)

static rtimer_clock_t
last_timestamp_read(void)
{
  rtimer_clock_t timestamp;
  uint32_t timestamp_cc;

  timestamp_cc = nrf_timer_cc_read(NRF_TIMER0, NRF_TIMER_CC_CHANNEL2);
  timestamp = CONVERT(timestamp_cc);

  return timestamp;
}
/*---------------------------------------------------------------------------*/
/*
 * Convert the hardware-reported LQI to 802.15.4 range using an 8-bit
 * saturating multiplication by 4, as per the Product Spec.
 */
static uint8_t
lqi_convert_to_802154_scale(uint8_t lqi_hw)
{
  return (uint8_t)lqi_hw > 63 ? 255 : lqi_hw * ED_RSSISCALE;
}
/*---------------------------------------------------------------------------*/
/* Netstack API functions */
/*---------------------------------------------------------------------------*/
static int
on(void)
{
  if(radio_is_powered() == false) {
    power_on_and_configure();
  }

  nrf_timer_mode_set(NRF_TIMER0, NRF_TIMER_MODE_COUNTER);
  nrf_timer_task_trigger(NRF_TIMER0, NRF_TIMER_TASK_START);

  nrf_ppi_channel_enable(NRF_PPI_CHANNEL20);

  enter_rx();

  ENERGEST_ON(ENERGEST_TYPE_LISTEN);
  return NRF52840_COMMAND_OK;
}
/*---------------------------------------------------------------------------*/
static int
channel_clear(void)
{
  return NRF52840_CCA_CLEAR;
}
/*---------------------------------------------------------------------------*/
static int
init(void)
{
  last_rssi = 0;
  last_lqi = 0;

  /* Start the RF driver process */
  process_start(&nrf52840_ieee_rf_process, NULL);

  /* Power on the radio */
  power_on_and_configure();

  /* Set up initial state of poll mode. This will configure interrupts. */
  set_poll_mode(poll_mode);

  return RADIO_TX_OK;
}
/*---------------------------------------------------------------------------*/
static int
prepare(const void *payload, unsigned short payload_len)
{
  LOG_DBG("Prepare 0x%02x bytes\n", payload_len + FCS_LEN);

  if(payload_len > MAX_PAYLOAD_LEN) {
    LOG_ERR("Too long: %u bytes, max %u\n", payload_len, MAX_PAYLOAD_LEN);
    return RADIO_TX_ERR;
  }

  /* Populate the PHR. Packet length, including the FCS */
  tx_buf.phr = (uint8_t)payload_len + FCS_LEN;

  /* Copy the payload over */
  memcpy(tx_buf.mpdu, payload, payload_len);

  return RADIO_TX_OK;
}
/*---------------------------------------------------------------------------*/
static int
transmit(unsigned short transmit_len)
{
  if(transmit_len > MAX_PAYLOAD_LEN) {
    LOG_ERR("transmit: too long (%u bytes)\n", transmit_len);
    return RADIO_TX_ERR;
  }

  on();

  if(send_on_cca) {
    if(channel_clear() == NRF52840_CCA_CLEAR) {
      return RADIO_TX_COLLISION;
    }
  }

  /* Start the transmission */
  ENERGEST_SWITCH(ENERGEST_TYPE_LISTEN, ENERGEST_TYPE_TRANSMIT);

  nrf_radio_task_trigger(NRF_RADIO_TASK_TXEN);

  /* Pointer to the TX buffer in PACKETPTR before task START */
  nrf_radio_packetptr_set(&tx_buf);

  /* Wait for the rest of the TXRU, if needed */
  while(nrf_radio_event_check(NRF_RADIO_EVENT_TXREADY) == false);

  /* Trigger the Start task */
  nrf_radio_task_trigger(NRF_RADIO_TASK_START);

  /* Wait for TX to complete */
  while(nrf_radio_event_check(NRF_RADIO_EVENT_PHYEND) == false);

  ENERGEST_SWITCH(ENERGEST_TYPE_TRANSMIT, ENERGEST_TYPE_LISTEN);

  enter_rx();

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
  int payload_len;

  /* Clear all events */
  rx_events_clear();

  payload_len = rx_buf.phr - FCS_LEN;

  if(payload_len < ACK_PAYLOAD_MIN_LEN || payload_len > MAX_PAYLOAD_LEN) {
    LOG_ERR("Incorrect length: %d\n", payload_len);
    rx_buf_clear();
    return 0;
  }

  memcpy(buf, rx_buf.mpdu, payload_len);
  last_lqi = lqi_convert_to_802154_scale(rx_buf.mpdu[payload_len]);

  packetbuf_set_attr(PACKETBUF_ATTR_RSSI, last_rssi);
  packetbuf_set_attr(PACKETBUF_ATTR_LINK_QUALITY, last_lqi);

  rx_buf_clear();

  return payload_len;
}
/*---------------------------------------------------------------------------*/
static int
receiving_packet(void)
{
  LOG_DBG("Receiving\n");

  /*
   * Start of reception generates the FRAMESTART event. End of reception
   * generates the END event. If FRAMESTART is generated and END is not then
   * we are in the process of receiving.
   */
  if((nrf_radio_event_check(NRF_RADIO_EVENT_FRAMESTART) == true) &&
     (nrf_radio_event_check(NRF_RADIO_EVENT_END) == false)) {
    return NRF52840_RECEIVING_YES;
  }
  return NRF52840_RECEIVING_NO;
}
/*---------------------------------------------------------------------------*/
static int
pending_packet(void)
{
  LOG_DBG("Pending\n");

  /* END generated, CRCOK generated, and there are bytes in the RX Buf */
  if((rx_buf.phr > 0) &&
     (nrf_radio_event_check(NRF_RADIO_EVENT_END) == true) &&
     (nrf_radio_event_check(NRF_RADIO_EVENT_CRCOK) == true)) {
    return NRF52840_PENDING_YES;
  }

  return NRF52840_PENDING_NO;
}
/*---------------------------------------------------------------------------*/
static int
off(void)
{
  nrf_radio_power_set(false);

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
  case RADIO_PARAM_RX_MODE:
    if(poll_mode) {
      *value |= RADIO_RX_MODE_POLL_MODE;
    }
    return RADIO_RESULT_OK;
  case RADIO_PARAM_TX_MODE:
    *value = 0;
    if(send_on_cca) {
      *value |= RADIO_TX_MODE_SEND_ON_CCA;
    }
    return RADIO_RESULT_OK;
  case RADIO_PARAM_TXPOWER:
    *value = (radio_value_t)nrf_radio_txpower_get();
    return RADIO_RESULT_OK;
  case RADIO_PARAM_CCA_THRESHOLD:
    *value = (radio_value_t)cca_config.cca_corr_threshold;
    return RADIO_RESULT_OK;
  case RADIO_PARAM_RSSI:
    *value = (radio_value_t)rssi_read();
    return RADIO_RESULT_OK;
  case RADIO_PARAM_LAST_RSSI:
    *value = (radio_value_t)last_rssi;
    return RADIO_RESULT_OK;
  case RADIO_PARAM_LAST_LINK_QUALITY:
    *value = (radio_value_t)last_lqi;
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

    /* ToDo: Add RADIO_CONST_MAX_PAYLOAD_LEN after #974 */

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
    if(value & ~(RADIO_RX_MODE_ADDRESS_FILTER |
                 RADIO_RX_MODE_AUTOACK |
                 RADIO_RX_MODE_POLL_MODE)) {
      return RADIO_RESULT_INVALID_VALUE;
    }

    set_poll_mode((value & RADIO_RX_MODE_POLL_MODE) != 0);

    return RADIO_RESULT_OK;
  case RADIO_PARAM_TX_MODE:
    if(value & ~(RADIO_TX_MODE_SEND_ON_CCA)) {
      return RADIO_RESULT_INVALID_VALUE;
    }

    send_on_cca = (value & RADIO_TX_MODE_SEND_ON_CCA) != 0;
    return RADIO_RESULT_OK;
  case RADIO_PARAM_TXPOWER:
    nrf_radio_txpower_set(value);
    return RADIO_RESULT_OK;
  case RADIO_PARAM_CCA_THRESHOLD:
    cca_config.cca_corr_threshold = value;
    cca_reconfigure();
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
    *(rtimer_clock_t *)dest = last_timestamp_read();
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
      len = read_frame(packetbuf_dataptr(), PACKETBUF_SIZE);

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
RADIO_IRQHandler(void)
{
  if(!poll_mode) {
    nrf_radio_event_clear(NRF_RADIO_EVENT_CRCOK);
    process_poll(&nrf52840_ieee_rf_process);
  }
}
/*---------------------------------------------------------------------------*/
