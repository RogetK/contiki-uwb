#include "contiki.h"

#include "deca_device_api.h"
#include "port_platform.h"
#include "mdek1001-def.h"

#include "net/netstack.h"
#include "net/packetbuf.h"

#define MAX_PAYLOAD_LEN (127)

static int init(void);
static int prepare(const void *payload, unsigned short payload_len);
static int transmit(unsigned short transmit_len);
static int send(const void *payload, unsigned short payload_len);
static int read(void *buf, unsigned short buf_len);
static int channel_clear(void);
static int receiving_packet(void);
static int pending_packet(void);

int on(void);
int off(void);

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
        on,
        off, 
        get_value,
        set_value,
        get_object,
        set_object,
    };

/* 
 * DW1000 radio init function
 * 
 */

/*DW1000 config function*/
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


static int init(void) {

    int result;

    /* Reset radio */
    reset_DW1000();

    /* Set 2MHz SPI for radio initialisation */
    port_set_dw1000_slowrate();

    /* Initialise the radio */
    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR) 
        return -1;
    
    /* Set 8Mhz SPI for configuration and operation */
    port_set_dw1000_fastrate();
    dwt_configure(&config);

    
}