#include "contiki.h"
#include "deca_device_api.h"

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
static int init(void) {
    /* Rest radio */

}