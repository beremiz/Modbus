/*
 * Copyright (c) 2001,2016 Mario de Sousa (msousa@fe.up.pt)
 *
 * This file is part of the Modbus library for Beremiz and matiec.
 *
 * This Modbus library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser 
 * General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this Modbus library.  If not, see <http://www.gnu.org/licenses/>.
 *
 * This code is made available on the understanding that it will not be
 * used in safety-critical situations without a full and competent review.
 */


/* mb_slave.h */


#ifndef MODBUS_SLAVE_H
#define MODBUS_SLAVE_H

#include <time.h> /* struct timespec data structure */

#include "mb_types.h" /* get the data types */
#include "mb_addr.h"  /* get definition of common variable types and error codes */




/* Initialise the Modbus Library to work as Slave/Server only */
int mb_slave_init(int nd_count);
/* Shut down the Modbus Library */
int mb_slave_done(void);




/* Create a new slave/server...
 * 
 * This function creates a new node used to:
 *   - accept connection requests (TCP version)
 *   - receive frames from masters (RTU and ASCII versions)
 * 
 * The type of address (naf_tcp, naf_rtu, naf_ascii) will specify the lower
 * layer of modbus that will be used by the newly opened node.
 */
int mb_slave_new(node_addr_t node_addr);
/* close a node crreated by mb_slave_new() */
int mb_slave_close(int nd);




/***********************************************/
/***********************************************/
/**                                           **/
/**               Run the slave               **/
/**                                           **/
/***********************************************/
/***********************************************/


/* The following functions must return:
 *  -2  on attempt to read invalid address 
 *  -1  on all other errors... 
 *   0  on success.
 *
 *  Start_addr may start from 0, to 65535!
 *  In other words, we use 0 based addressing!  
 */ 
typedef struct {
  int (*read_inbits)   (void *arg, u16 start_addr, u16 bit_count,  u8  *data_bytes); /* bits are packed into bytes... */
  int (*read_outbits)  (void *arg, u16 start_addr, u16 bit_count,  u8  *data_bytes); /* bits are packed into bytes... */
  int (*write_outbits) (void *arg, u16 start_addr, u16 bit_count,  u8  *data_bytes); /* bits are packed into bytes... */
  int (*read_inwords)  (void *arg, u16 start_addr, u16 word_count, u16 *data_words);
  int (*read_outwords) (void *arg, u16 start_addr, u16 word_count, u16 *data_words);
  int (*write_outwords)(void *arg, u16 start_addr, u16 word_count, u16 *data_words);
  void *arg;
 } mb_slave_callback_t;

/* Execute the Slave and process requests coming from masters...
 * This function enters an infinite loop wating for new connection requests, 
 * and for modbus requests over previoulsy open connections...
 *
 * The frames are read from:
 *   -  the node descriptor nd, if nd >= 0
 *       When using TCP, if the referenced node nd was created to listen for new connections
 *       [mb_slave_listen()], then this function will also reply to Modbus data requests arriving
 *       on other nodes that were created as a consequence of accepting connections requests to
 *       the referenced node nd.
 *       All other nodes are ignored!
 *       
 *   -  any valid and initialised TCP node descriptor, if nd = -1
 *      In this case, will also accept connection requests arriving from a previously
 *       created node to listen for new connection requests [mb_slave_listen() ].
 *      NOTE: (only avaliable if using TCP)
 * 
 * slaveid identifies the address (RTU and ASCII) or slaveid (TCP) that we implement.
 *     Any requests that we receive sent with a slaveid different
 *     than the one specified, and also different to 0, will be silently ignored!
 *     Whatever the slaveid specified, we always reply to requests
 *     to slaveid 0 (the modbus broadcast address).
 *     Calling this function with a slaveid of 0 means to ignore this 
 *     parameter and to reply to all requests (whatever the slaveid
 *     used in the request). This should mostly be used by TCP servers... 
 */
int mb_slave_run(int nd, mb_slave_callback_t callback_functions, u8 slaveid);




#endif  /* MODBUS_SLAVE_H */








