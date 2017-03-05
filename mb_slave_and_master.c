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



#include <fcntl.h>	/* File control definitions */
#include <stdio.h>	/* Standard input/output */
#include <string.h>
#include <stdlib.h>
#include <termio.h>	/* POSIX terminal control definitions */
#include <sys/time.h>	/* Time structures for select() */
#include <unistd.h>	/* POSIX Symbolic Constants */
#include <errno.h>	/* Error definitions */

#include "mb_layer1.h"
#include "mb_slave_private.h"
#include "mb_master_private.h"
#include "mb_slave.h"
#include "mb_master.h"

//#define DEBUG 		/* uncomment to see the data sent and received */


#ifndef TRUE
#define TRUE  1
#endif

#ifndef FALSE
#define FALSE 0
#endif




layer1_funct_ptr_t fptr_[4] = {
  { /* WARNING: TCP functions MUST be the first, as we have this hardcoded in the code! */
    /*          more specifically, in the get_ttyfd() macro in mb_slave.c               */ 
    /*                             in the mb_slave_new() function in mb_slave.c         */
    /*                             in the mb_master_connect() function in mb_master.c   */    
    &modbus_tcp_write          
   ,&modbus_tcp_read           
   ,&modbus_tcp_init           
   ,&modbus_tcp_done           
   ,&modbus_tcp_connect        
   ,&modbus_tcp_listen         
   ,&modbus_tcp_close          
   ,&modbus_tcp_silence_init   
   ,&modbus_tcp_get_min_timeout
  },{
    &modbus_rtu_write           
   ,&modbus_rtu_read            
   ,&modbus_rtu_init            
   ,&modbus_rtu_done            
   ,&modbus_rtu_connect         
   ,&modbus_rtu_listen          
   ,&modbus_rtu_close           
   ,&modbus_rtu_silence_init    
   ,&modbus_rtu_get_min_timeout 
  },{
    &modbus_ascii_write          
   ,&modbus_ascii_read           
   ,&modbus_ascii_init           
   ,&modbus_ascii_done           
   ,&modbus_ascii_connect        
   ,&modbus_ascii_listen         
   ,&modbus_ascii_close          
   ,&modbus_ascii_silence_init   
   ,&modbus_ascii_get_min_timeout
  },{
    NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL
  }
};






/************************************************************************

	initialise / shutdown the library

	These functions sets up/shut down the library state
        (allocate memory for buffers, initialise data strcutures, etc)

**************************************************************************/
#define max(a,b) (((a)>(b))?(a):(b))

int mb_slave_and_master_init(int nd_count_tcp, int nd_count_rtu, int nd_count_ascii) {
        int extra_bytes, extra_bytes_tcp, extra_bytes_rtu, extra_bytes_ascii;

#ifdef DEBUG
	fprintf( stderr, "mb_slave_and_master_init()\n");
	fprintf( stderr, "creating %d nodes\n", nd_count);
#endif

            /* initialise layer 1 library */
	if (modbus_tcp_init  (nd_count_tcp,   DEF_OPTIMIZATION, &extra_bytes_tcp  ) < 0)
		goto error_exit_0;
	if (modbus_rtu_init  (nd_count_rtu,   DEF_OPTIMIZATION, &extra_bytes_rtu  ) < 0)
		goto error_exit_1;
	if (modbus_ascii_init(nd_count_ascii, DEF_OPTIMIZATION, &extra_bytes_ascii) < 0)
		goto error_exit_2;
	extra_bytes= max(extra_bytes_tcp, extra_bytes_rtu);
	extra_bytes= max(extra_bytes    , extra_bytes_ascii);

            /* initialise master and slave libraries... */
	if (mb_slave_init__(extra_bytes) < 0)
		goto error_exit_3;
	if (mb_master_init__(extra_bytes) < 0)
		goto error_exit_4;
	return 0;

/*
error_exit_3:
	modbus_master_done();
*/
error_exit_4:
	mb_slave_done__();
error_exit_3:
	modbus_ascii_done();
error_exit_2:
	modbus_rtu_done();
error_exit_1:
	modbus_tcp_done();
error_exit_0:
	return -1;
}




int mb_slave_and_master_done(void) {
	int res = 0;
	res |= mb_slave_done__  ();
	res |= mb_master_done__ ();
	res |= modbus_ascii_done();
	res |= modbus_rtu_done  ();
	res |= modbus_tcp_done  ();
	return res;
}

