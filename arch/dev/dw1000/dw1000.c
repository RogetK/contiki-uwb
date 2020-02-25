#include "contiki.h"
#include "sys/energest.h"

#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_types.h"
#include "port_platform.h"
#include "mdek1001-def.h"

#include "nrf_ppi.h"
#include "nrf_drv_gpiote.h"
#include "nrf_gpiote.h"
#include "nrf_timer.h"
#include "nrf_clock.h"

#include "net/netstack.h"
#include "net/packetbuf.h"

#include "sys/log.h"

#define LOG_MODULE "DW1000 driver"
#define LOG_LEVEL LOG_LEVEL_ERR

/*---------------------------------------------------------------------------*/
PROCESS(dw1000_process, "DW1000 driver");
/*---------------------------------------------------------------------------*/

#define CRC_LEN 2
#define MAX_PAYLOAD_LEN (127 - CRC_LEN)
#define DW1000_RX_AFTER_TX_DELAY 0

static bool packet_pending;
static bool poll_mode = false;

/*------------------------------------------------------------------------
 * Function Declarations
 *----------------------------------------------------------------------*/

static int init(void);
static int prepare(const void *payload, unsigned short payload_len);
static int transmit(unsigned short transmit_len);
static int send(const void *payload, unsigned short payload_len);
static int read(void *buf, unsigned short buf_len);
static int channel_clear(void);
static int receiving_packet(void);
static int pending_packet(void);

int dw1000_on(void);
int dw1000_off(void);

static radio_result_t get_value(radio_param_t param, radio_value_t *value);
static radio_result_t set_value(radio_param_t param, radio_value_t value);

static radio_result_t get_object(radio_param_t param, void *dest, size_t size);
static radio_result_t set_object(radio_param_t param, const void *src, size_t size);


const struct radio_driver dw1000_driver =
{
  init,
  prepare,
  transmit,
  send,
  read,
  channel_clear,
  receiving_packet,
  pending_packet,
  dw1000_on,
  dw1000_off,
  get_value,
  set_value,
  get_object,
  set_object
};


static void dw_interrupt_init(void){
    ret_code_t err_code;
    nrf_drv_gpiote_init();
    nrf_drv_gpiote_in_config_t in_config = 
        GPIOTE_CONFIG_IN_SENSE_LOTOHI(true); 

    in_config.pull = NRF_GPIO_PIN_NOPULL;
    err_code = 
        nrf_drv_gpiote_in_init(DW1000_IRQ, &in_config, NULL);

    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_in_event_enable(DW1000_IRQ, true);
   
}



static void ppi_init(void) {

    nrf_ppi_channel_endpoint_setup(
        NRF_PPI_CHANNEL0,
        (uint32_t) nrf_drv_gpiote_in_event_addr_get(DW1000_IRQ), 
        (uint32_t) nrf_timer_task_address_get(NRF_TIMER0, NRF_TIMER_TASK_CAPTURE3)
    );

    nrf_ppi_channel_enable(NRF_PPI_CHANNEL0);
    nrf_ppi_channel_enable(NRF_PPI_CHANNEL27);
}

static const dwt_config_t default_config = {
    5,                /* Channel number. */
    DWT_PRF_64M,      /* Pulse repetition frequency. */
    DWT_PLEN_64,     /* Preamble length. Used in TX only. */
    DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
    10,               /* TX preamble code. Used in TX only. */
    10,               /* RX preamble code. Used in RX only. */
    0,                /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8,       /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    (65 + 8 - 8)     /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};


static dwt_config_t config = {
    5,                /* Channel number. */
    DWT_PRF_64M,      /* Pulse repetition frequency. */
    DWT_PLEN_64,     /* Preamble length. Used in TX only. */
    DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
    10,               /* TX preamble code. Used in TX only. */
    10,               /* RX preamble code. Used in RX only. */
    0,                /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8,       /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    (65 + 8 - 8)     /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

static int get_channel(dwt_config_t dw_config) {
    int val = 0;
    
    val = (2 * config.chan) + config.prf; 
    val = (config.chan == 7) ? val - 4 : val - 2;
    return val; 
}


/* 
 * DW1000 radio init function
 * 
 */

/*DW1000 config function*/

static int 
init(void) {
    // int result;
    dw_interrupt_init();
    /* Reset radio */
    reset_DW1000();
    /* Set 2MHz SPI for radio initialisation */
    port_set_dw1000_slowrate();
    /* Initialise the radio */
    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR) return -1;


    
    /* Set 8Mhz SPI for configuration and operation */
    port_set_dw1000_fastrate();
    /* Set default radio paramaters */
    dwt_configure(&config);
    /* Set radio indicator LEDs */
    dwt_setleds(1);

    dwt_setinterrupt(DWT_INT_RXSFD, 1);

    ppi_init();

    process_start(&dw1000_process, NULL);
    return RADIO_RESULT_OK;
}

/*
 * ------------------------------------------------------------------------
 */

static int 
prepare(const void *payload, unsigned short payload_len) {
    uint8_t frame_len = payload_len + CRC_LEN;
    if (frame_len > MAX_PAYLOAD_LEN) return RADIO_TX_ERR;

    dwt_writetxdata(payload_len, (uint8_t *)payload, 0);
    dwt_writetxfctrl(frame_len, 0, 0);

    return RADIO_RESULT_OK;
}

/*
 * ------------------------------------------------------------------------
 */

static int 
transmit(unsigned short transmit_len) {
    if (transmit_len > MAX_PAYLOAD_LEN) return RADIO_TX_ERR;
    
    // Ensure radio is in a IDLE state before sending.
    dwt_forcetrxoff(); 
    dwt_setrxaftertxdelay(DW1000_RX_AFTER_TX_DELAY);

    // start transmission
    if (dwt_starttx(0) != DWT_SUCCESS) return RADIO_TX_ERR;

    return RADIO_TX_OK;
}

/*
 * ------------------------------------------------------------------------
 */


static int 
send(const void *payload, unsigned short payload_len) {
    if (prepare(payload, payload_len) == 0) {
        return transmit(payload_len);
    } else {
        return RADIO_TX_ERR;
    }
}

/*
 * ------------------------------------------------------------------------
 */


static int 
read(void *buf, unsigned short bufsize) {
    dwt_readrxdata((uint8_t *) buf, bufsize, 0);
    return bufsize;
}

/*
 * ------------------------------------------------------------------------
 */


static int 
channel_clear(void) {
    return 0;
}

/*
 * ------------------------------------------------------------------------
 */


static int 
receiving_packet(void) {
    return 0;
}

/*
 * ------------------------------------------------------------------------
 */

static int 
pending_packet(void) {
    return packet_pending;
}

/*
 * ------------------------------------------------------------------------
 */

int 
dw1000_on(void){
    dwt_setrxtimeout(0);
    if (dwt_rxenable(0) != DWT_SUCCESS){
        return RADIO_RESULT_ERROR;
    }
    
    return RADIO_RESULT_OK;
}

/*
 * ------------------------------------------------------------------------
 */

int 
dw1000_off(void) {
    dwt_forcetrxoff();
    return 0;
}

/*
 * ------------------------------------------------------------------------
 */

static radio_result_t 
get_value(radio_param_t param, radio_value_t *value) {
    if (!value)  return RADIO_RESULT_INVALID_VALUE;

    switch(param) {
    case RADIO_PARAM_RX_MODE:
        *value = 0;
        if(poll_mode) {
            *value |= RADIO_RX_MODE_POLL_MODE;
        }
        return RADIO_RESULT_OK;

    case RADIO_PARAM_CHANNEL:
        *value = (radio_value_t) get_channel(config);
        return RADIO_RESULT_OK;

    default:
        return RADIO_RESULT_NOT_SUPPORTED;
    }


}

static radio_result_t 
set_value(radio_param_t param, radio_value_t value) {
    return RADIO_RESULT_NOT_SUPPORTED;
}

static radio_result_t 
get_object(radio_param_t param, void *dest, size_t size) {
    return RADIO_RESULT_NOT_SUPPORTED;
}
static radio_result_t 
set_object(radio_param_t param, const void *src, size_t size) {
    return RADIO_RESULT_NOT_SUPPORTED;
}

PROCESS_THREAD(dw1000_process, ev, data)
{
    int len;
    PROCESS_BEGIN();

    while(1) {
        PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

        /* Clear packetbuf to avoid having leftovers from previous receptions */
        packetbuf_clear();

        /* Copy the received frame to packetbuf */
        len  = read(packetbuf_dataptr(), PACKETBUF_SIZE);
        packetbuf_set_datalen(len);

        /* Re-enable RX to keep listening */
        dw1000_on();
         /*PRINTF("dw1000_process: calling recv cb, len %d\n", data_len); */
        NETSTACK_MAC.input();
  }

  PROCESS_END();
}
