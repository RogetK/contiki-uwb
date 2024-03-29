/*
 * Copyright (c) 2015, Nordic Semiconductor
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */
/*---------------------------------------------------------------------------*/
#ifndef CONTIKI_CONF_H
#define CONTIKI_CONF_H

#include <stdint.h>
#include <inttypes.h>
/*---------------------------------------------------------------------------*/
/* Include Project Specific conf */
#ifdef PROJECT_CONF_PATH
#include PROJECT_CONF_PATH
#endif /* PROJECT_CONF_PATH */
/*---------------------------------------------------------------------------*/
/* Include platform peripherals configuration */
#include "mdek1001-def.h"
#include "nrf52832-def.h"

#define LOG_CONF_LEVEL_MAC  LOG_LEVEL_DBG
#define LOG_LEVEL_DW1000    LOG_LEVEL_DBG
#define LOG_CONF_LEVEL_RPL LOG_LEVEL_DBG
/*---------------------------------------------------------------------------*/
#ifndef NETSTACK_CONF_RADIO
#define NETSTACK_CONF_RADIO   dw1000_driver

#define TSCH_CONF_DEFAULT_HOPPING_SEQUENCE (uint8_t[]) {9}

#endif /* NETSTACK_CONF_RADIO */

// #ifndef SICSLOWPAN_CONF_FRAG
// #define SICSLOWPAN_CONF_FRAG                    1
// #endif
/*---------------------------------------------------------------------------*/
/* Include CPU-related configuration */
#include "nrf52832-conf.h"
/*---------------------------------------------------------------------------*/



/** @} */
#endif /* CONTIKI_CONF_H */
/**
 * @}
 * @}
 */
