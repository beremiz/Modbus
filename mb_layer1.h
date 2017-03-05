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



#ifndef MODBUS_LAYER1_H
#define MODBUS_LAYER1_H

#include <time.h> /* struct timespec data type */

#include "mb_types.h" /* get the data types */
#include "mb_addr.h"  /* get definitions of common variable types */


/* Define max registers and bits  */
#define MAX_READ_BITS           2000    /* Functions 0x01 and 0x02 */
#define MAX_READ_REGS           125     /* Functions 0x03 and 0x04 */
#define MAX_WRITE_COILS         1968    /* Function 0x0F */
#define MAX_WRITE_REGS          123     /* Function 0x10 */


/* Declare TCP layer1 functions */
#define modbus_write             modbus_tcp_write           
#define modbus_read              modbus_tcp_read            
#define modbus_init              modbus_tcp_init            
#define modbus_done              modbus_tcp_done            
#define modbus_connect           modbus_tcp_connect         
#define modbus_listen            modbus_tcp_listen          
#define modbus_close             modbus_tcp_close           
#define modbus_silence_init      modbus_tcp_silence_init    
#define modbus_get_min_timeout   modbus_tcp_get_min_timeout 

#include "mb_layer1_prototypes.h"

#undef modbus_write          
#undef modbus_read           
#undef modbus_init           
#undef modbus_done           
#undef modbus_connect        
#undef modbus_listen         
#undef modbus_close          
#undef modbus_silence_init   
#undef modbus_get_min_timeout





/* Declare RTU layer1 functions */
#define modbus_write             modbus_rtu_write           
#define modbus_read              modbus_rtu_read            
#define modbus_init              modbus_rtu_init            
#define modbus_done              modbus_rtu_done            
#define modbus_connect           modbus_rtu_connect         
#define modbus_listen            modbus_rtu_listen          
#define modbus_close             modbus_rtu_close           
#define modbus_silence_init      modbus_rtu_silence_init    
#define modbus_get_min_timeout   modbus_rtu_get_min_timeout 

#include "mb_layer1_prototypes.h"

#undef modbus_write          
#undef modbus_read           
#undef modbus_init           
#undef modbus_done           
#undef modbus_connect        
#undef modbus_listen         
#undef modbus_close          
#undef modbus_silence_init   
#undef modbus_get_min_timeout





/* Declare ASCII layer1 functions */
#define modbus_write             modbus_ascii_write           
#define modbus_read              modbus_ascii_read            
#define modbus_init              modbus_ascii_init            
#define modbus_done              modbus_ascii_done            
#define modbus_connect           modbus_ascii_connect         
#define modbus_listen            modbus_ascii_listen          
#define modbus_close             modbus_ascii_close           
#define modbus_silence_init      modbus_ascii_silence_init    
#define modbus_get_min_timeout   modbus_ascii_get_min_timeout 

#include "mb_layer1_prototypes.h"

#undef modbus_write          
#undef modbus_read           
#undef modbus_init           
#undef modbus_done           
#undef modbus_connect        
#undef modbus_listen         
#undef modbus_close          
#undef modbus_silence_init   
#undef modbus_get_min_timeout




#define modbus_write             (*modbus_write          ) 
#define modbus_read              (*modbus_read           ) 
#define modbus_init              (*modbus_init           ) 
#define modbus_done              (*modbus_done           ) 
#define modbus_connect           (*modbus_connect        ) 
#define modbus_listen            (*modbus_listen         ) 
#define modbus_close             (*modbus_close          ) 
#define modbus_silence_init      (*modbus_silence_init   ) 
#define modbus_get_min_timeout   (*modbus_get_min_timeout) 

typedef struct {
  #include "mb_layer1_prototypes.h"
} layer1_funct_ptr_t;

#undef modbus_write          
#undef modbus_read           
#undef modbus_init           
#undef modbus_done           
#undef modbus_connect        
#undef modbus_listen         
#undef modbus_close          
#undef modbus_silence_init   
#undef modbus_get_min_timeout



extern layer1_funct_ptr_t fptr_[4];





#endif  /* MODBUS_LAYER1_H */


