/*
 * Copyright (c) 2002,2016 Mario de Sousa (msousa@fe.up.pt)
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


#ifndef MODBUS_LAYER2_H
#define MODBUS_LAYER2_H

#include <time.h> /* struct timespec data type */

//#include <sys/socket.h>
//#include <netinet/in.h>
#include <netinet/ip.h> /* superset of previous */ // Required for INADDR_ANY
       
       
/* Library Error codes */
#define PORT_FAILURE   -101
#define INTERNAL_ERROR -102
#define TIMEOUT        -103
#define INVALID_FRAME  -104
#define MODBUS_ERROR   -105

/* NOTE: Modbus error codes are defined in mb_util.h */




typedef enum {optimize_speed, optimize_size} optimization_t;


typedef enum {
        naf_ascii,
        naf_rtu,
        naf_tcp,
  } node_addr_family_t;

typedef struct {
        const char *host;
        const char *service;
        int         close_on_silence;  
  } node_addr_tcp_t;

typedef struct {
        const char *device;
        int         baud;       /* plain baud rate, eg 2400; zero for the default 9600 */
        int         parity;     /* 0 for none, 1 for odd, 2 for even                   */
        int         data_bits;
        int         stop_bits;
        int         ignore_echo; /* 1 => ignore echo; 0 => do not ignore echo */
  } node_addr_rtu_t;

typedef node_addr_rtu_t node_addr_ascii_t;

typedef union {
        node_addr_ascii_t ascii;
        node_addr_rtu_t   rtu;
        node_addr_tcp_t   tcp;
  } node_addr_common_t;

typedef struct {
        node_addr_family_t  naf;
        node_addr_common_t  addr;
  } node_addr_t;

#endif  /* MODBUS_LAYER2_H */








