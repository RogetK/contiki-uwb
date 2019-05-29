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
#include "nrf_clock.h"

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
 *
 * ToDO: Unlike other radios that write RSSI and LQI in the FCS, the nrf52840
 * only writes one value. This is a "hardware-reported" value, which needs
 * converted to the .15.4 standard LQI scale using an 8-bit saturating
 * multiplication by 4 (see the Product Spec). This value is based on the
 * median of three RSSI samples taken during frame reception. For now we treat
 * this value as the actual RSSI reported as negative dBm.
 */
static int8_t last_rssi;
static uint8_t last_lqi;

/* Perform CCA before TX */
static uint8_t send_on_cca = RADIO_TX_MODE_SEND_ON_CCA;
/*---------------------------------------------------------------------------*/
PROCESS(nrf52840_ieee_rf_process, "nRF52840 IEEE RF driver");
/*---------------------------------------------------------------------------*/
#ifndef NRF52840_CCA_MODE
#define NRF52840_CCA_MODE RADIO_CCACTRL_CCAMODE_CarrierAndEdMode
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
#define SYMBOL_DURATION_USEC          16
#define SYMBOL_DURATION_RTIMER         1
#define BYTE_DURATION_RTIMER          (SYMBOL_DURATION_RTIMER * 2)
/*---------------------------------------------------------------------------*/
/* Timestamping control */
#if MAC_CONF_WITH_TSCH
#define TIMESTAMPING_WITH_PPI 1
#endif

/* Timestamp in rtimer ticks of the reception of the PHR (FRAMESTART) */
static volatile rtimer_clock_t ts_dbg_framestart;

/* Timestamp in rtimer ticks of the CRCOK event */
static volatile rtimer_clock_t ts_dbg_crcok;

/*
 * Timestamp in rtimer ticks of the reception of the SFD
 * The SFD was received 1 byte before FRAMESTART, therefore all we need to do
 * is subtract 2 symbols (2 rtimer ticks) from ts_dbg_framestart.
 */
static volatile rtimer_clock_t ts_last_frame;
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
  conf.crcinc = true;
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
    nrf_radio_event_clear(NRF_RADIO_EVENT_FRAMESTART);
    interrupts |= NRF_RADIO_INT_CRCOK_MASK | NRF_RADIO_INT_FRAMESTART_MASK;
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
}
/*---------------------------------------------------------------------------*/
static void
power_on_and_configure(void)
{
  nrf_radio_power_set(true);
  configure();
}
/*---------------------------------------------------------------------------*/
/*
 * The caller must first make sure the radio is powered and configured.
 *
 * When we enter this function we can be in one of the following states:
 * -       STATE_RX: We were already in RX. Do nothing
 * -   STATE_RXIDLE: A reception just finished and we reverted to RXIDLE.
 *                   We just need to send the START task.
 * -   STATE_TXIDLE: A TX just finished and we reverted to TXIDLE.
 *                   We just need to send the START task.
 * - STATE_DISABLED: We just turned on. We need to request radio rampup
 */
static void
enter_rx(void)
{
  nrf_radio_state_t curr_state = nrf_radio_state_get();

  LOG_INFO("Enter RX, state=%u", curr_state);

  /* Do nothing if we are already in RX */
  if(curr_state == NRF_RADIO_STATE_RX) {
    LOG_DBG_(". Was in RX");
    LOG_INFO_("\n");
    return;
  }

  /* Prepare the RX buffer */
  rx_buf_clear();
  nrf_radio_packetptr_set(&rx_buf);

  /* Make sure the correct interrupts are enabled */
  rx_events_clear();
  setup_interrupts();

  if(curr_state != NRF_RADIO_STATE_RXIDLE) {
    /* Clear EVENTS_RXREADY and trigger RXEN */
    nrf_radio_event_clear(NRF_RADIO_EVENT_RXREADY);
    nrf_radio_task_trigger(NRF_RADIO_TASK_RXEN);

    /* Block till RXRU is done */
    while(!nrf_radio_event_check(NRF_RADIO_EVENT_RXREADY));
    LOG_INFO_("--->%u", nrf_radio_state_get());
  }

  /* Trigger the Start task */
  nrf_radio_task_trigger(NRF_RADIO_TASK_START);

  LOG_INFO_("--->%u\n", nrf_radio_state_get());

  LOG_DBG("PACKETPTR=0x%08lx (rx_buf @ 0x%08lx)\n",
          (uint32_t)nrf_radio_packetptr_get(), (uint32_t)&rx_buf);
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
  LOG_DBG("On\n");

  if(radio_is_powered() == false) {
    LOG_DBG("Not powered\n");
    power_on_and_configure();
  }

#if TIMESTAMPING_WITH_PPI
  nrf_timer_mode_set(NRF_TIMER0, NRF_TIMER_MODE_COUNTER);
  nrf_timer_task_trigger(NRF_TIMER0, NRF_TIMER_TASK_START);

  nrf_ppi_channel_enable(NRF_PPI_CHANNEL20);
#endif

  enter_rx();

  ENERGEST_ON(ENERGEST_TYPE_LISTEN);
  return NRF52840_COMMAND_OK;
}
/*---------------------------------------------------------------------------*/
static int
channel_clear(void)
{
  bool busy, idle;

  LOG_DBG("channel_clear\n");

  on();

  /* Clear previous CCA-related events, if any */
  nrf_radio_event_clear(NRF_RADIO_EVENT_CCABUSY);
  nrf_radio_event_clear(NRF_RADIO_EVENT_CCAIDLE);
  nrf_radio_event_clear(NRF_RADIO_EVENT_CCASTOPPED);

  LOG_DBG("channel_clear: CCACTRL=0x%08lx\n", NRF_RADIO->CCACTRL);

  /* We are now in RX. Send CCASTART */
  nrf_radio_task_trigger(NRF_RADIO_TASK_CCASTART);

  while((nrf_radio_event_check(NRF_RADIO_EVENT_CCABUSY) == false) &&
        (nrf_radio_event_check(NRF_RADIO_EVENT_CCAIDLE) == false));

  busy = nrf_radio_event_check(NRF_RADIO_EVENT_CCABUSY);
  idle = nrf_radio_event_check(NRF_RADIO_EVENT_CCAIDLE);

  LOG_DBG("channel_clear: I=%u, B=%u\n", idle, busy);

  if(busy) {
    return NRF52840_CCA_BUSY;
  }

  return NRF52840_CCA_CLEAR;
}
/*---------------------------------------------------------------------------*/
static int
init(void)
{
  LOG_DBG("Init\n");

  last_rssi = 0;
  last_lqi = 0;
  ts_dbg_framestart = 0;
  ts_dbg_crcok = 0;
  ts_last_frame = 0;

  /* Request the HF clock */
  nrf_clock_event_clear(NRF_CLOCK_EVENT_HFCLKSTARTED);
  nrf_clock_task_trigger(NRF_CLOCK_TASK_HFCLKSTART);

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
  LOG_INFO("Prepare %u bytes\n", payload_len);

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
  int i;

  LOG_INFO("TX %u bytes + FCS, channel=%u\n", transmit_len, get_channel());

  if(transmit_len > MAX_PAYLOAD_LEN) {
    LOG_ERR("TX: too long (%u bytes)\n", transmit_len);
    return RADIO_TX_ERR;
  }

  on();

  if(send_on_cca) {
    if(channel_clear() == NRF52840_CCA_BUSY) {
      LOG_DBG("TX: Busy\n");
      return RADIO_TX_COLLISION;
    }
  }

  LOG_DBG("Transmit: %u bytes=000000", tx_buf.phr);
  for(i = 0; i < tx_buf.phr - 2; i++) {
    LOG_DBG_(" %02x", tx_buf.mpdu[i]);
  }
  LOG_DBG_("\n");

  /* Start the transmission */
  ENERGEST_SWITCH(ENERGEST_TYPE_LISTEN, ENERGEST_TYPE_TRANSMIT);

  LOG_DBG("TX Start. State %u", nrf_radio_state_get());

  /* Pointer to the TX buffer in PACKETPTR before task START */
  nrf_radio_packetptr_set(&tx_buf);

  /* Clear TX-related events */
  nrf_radio_event_clear(NRF_RADIO_EVENT_END);
  nrf_radio_event_clear(NRF_RADIO_EVENT_PHYEND);
  nrf_radio_event_clear(NRF_RADIO_EVENT_TXREADY);

  nrf_radio_task_trigger(NRF_RADIO_TASK_TXEN);

  LOG_DBG_("--->%u", nrf_radio_state_get());

  /* Wait for the rest of the TXRU, if needed */
  while(nrf_radio_state_get() != NRF_RADIO_STATE_TXIDLE);

  LOG_DBG_("--->%u", nrf_radio_state_get());

  /* Trigger the Start task */
  nrf_radio_task_trigger(NRF_RADIO_TASK_START);

  LOG_DBG_("--->%u\n", nrf_radio_state_get());

  /* Wait for TX to complete */
  while(nrf_radio_state_get() == NRF_RADIO_STATE_TX);

  ENERGEST_SWITCH(ENERGEST_TYPE_TRANSMIT, ENERGEST_TYPE_LISTEN);

  LOG_DBG("TX: Done\n");

  /* Disable TX, enter RX */
  nrf_radio_task_trigger(NRF_RADIO_TASK_DISABLE);

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
  rtimer_clock_t diff;

  /* Clear all events */
  rx_events_clear();

  payload_len = rx_buf.phr - FCS_LEN;

  if(payload_len < ACK_PAYLOAD_MIN_LEN || payload_len > MAX_PAYLOAD_LEN) {
    LOG_ERR("Incorrect length: %d\n", payload_len);
    enter_rx();
    return 0;
  }

  memcpy(buf, rx_buf.mpdu, payload_len);
  last_lqi = lqi_convert_to_802154_scale(rx_buf.mpdu[payload_len]);
  last_rssi = -((int8_t)rx_buf.mpdu[payload_len]);

  packetbuf_set_attr(PACKETBUF_ATTR_RSSI, last_rssi);
  packetbuf_set_attr(PACKETBUF_ATTR_LINK_QUALITY, last_lqi);

  diff = ts_dbg_crcok - ts_dbg_framestart;

  LOG_DBG("Read frame, len=%d bytes (%u symbols):", rx_buf.phr, 2 * rx_buf.phr);
  LOG_DBG_("CRCOK - FRAMESTART = %lu-%lu=%lu symbols\n", ts_dbg_crcok,
           ts_dbg_framestart, diff);
  LOG_INFO("Read frame: len=%d, RSSI=%d, LQI=0x%02x\n", payload_len, last_rssi,
           last_lqi);

  /*
   * Timestamp in rtimer ticks of the reception of the SFD. The SFD was
   * received 1 byte before the PHR (FRAMESTART event), therefore all we need
   * to do is subtract 2 symbols (2 rtimer ticks) from ts_dbg_framestart.
   */
  ts_last_frame = ts_dbg_framestart - BYTE_DURATION_RTIMER;

  enter_rx();

  return payload_len;
}
/*---------------------------------------------------------------------------*/
static int
receiving_packet(void)
{
  LOG_DBG("Receiving: ");

  /*
   * Start of reception generates the FRAMESTART event. End of reception
   * generates the END event. If FRAMESTART is generated and END is not then
   * we are in the process of receiving.
   */
  if((nrf_radio_event_check(NRF_RADIO_EVENT_FRAMESTART) == true) &&
     (nrf_radio_event_check(NRF_RADIO_EVENT_END) == false)) {
    LOG_DBG_("Yes\n");
    return NRF52840_RECEIVING_YES;
  }

  LOG_DBG_("No\n");
  return NRF52840_RECEIVING_NO;
}
/*---------------------------------------------------------------------------*/
static int
pending_packet(void)
{
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
    *(rtimer_clock_t *)dest = ts_last_frame;
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

    LOG_DBG("Polled\n");

    watchdog_periodic();
    packetbuf_clear();
    len = read_frame(packetbuf_dataptr(), PACKETBUF_SIZE);
    if(len > 0) {
      packetbuf_set_datalen(len);
      NETSTACK_MAC.input();
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
void
RADIO_IRQHandler(void)
{
  if(!poll_mode) {
    if(nrf_radio_event_check(NRF_RADIO_EVENT_CRCOK)) {
      ts_dbg_crcok = RTIMER_NOW();
      nrf_radio_event_clear(NRF_RADIO_EVENT_CRCOK);
      nrf_radio_event_clear(NRF_RADIO_EVENT_FRAMESTART);
      process_poll(&nrf52840_ieee_rf_process);
    }

    if(nrf_radio_event_check(NRF_RADIO_EVENT_FRAMESTART)) {
      /* Unack the interrupt */
      ts_dbg_framestart = RTIMER_NOW();
      nrf_radio_int_disable(NRF_RADIO_INT_FRAMESTART_MASK);
    }
  }
}
/*---------------------------------------------------------------------------*/
