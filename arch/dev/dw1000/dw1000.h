#ifndef DW1000_DRIVER_H_
#define DW1000_DRIVER_H_

#include "contiki.h"
#include "dev/radio.h"
#include "deca_device_api.h"

extern const struct radio_driver dw1000_driver;

int dw1000_on(void);
int dw1000_off(void);


#define DW1000_DEFAULT_CHANNEL 0

#endif