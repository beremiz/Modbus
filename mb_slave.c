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


/* mb_slave.c */

#include <fcntl.h>	/* File control definitions */
#include <stdio.h>	/* Standard input/output */
#include <string.h>
#include <stdlib.h>
#include <termio.h>	/* POSIX terminal control definitions */
#include <sys/time.h>	/* Time structures for select() */
#include <unistd.h>	/* POSIX Symbolic Constants */
#include <errno.h>	/* Error definitions */

#include <netinet/in.h> /* required for htons() and ntohs() */
#include "mb_layer1.h"
#include "mb_slave.h"
#include "mb_slave_private.h"

/* #define DEBUG */		/* uncomment to see the data sent and received */


#define modbus_write             fptr_[layer1_fin].modbus_write           
#define modbus_read              fptr_[layer1_fin].modbus_read            
#define modbus_init              fptr_[layer1_fin].modbus_init            
#define modbus_done              fptr_[layer1_fin].modbus_done            
#define modbus_connect           fptr_[layer1_fin].modbus_connect         
#define modbus_listen            fptr_[layer1_fin].modbus_listen          
#define modbus_close             fptr_[layer1_fin].modbus_close           
#define modbus_silence_init      fptr_[layer1_fin].modbus_silence_init    
#define modbus_get_min_timeout   fptr_[layer1_fin].modbus_get_min_timeout 

/* the lower two bits of ttyfd are used to store the index to layer1 function pointers */
/* layer1_fin index to fptr_[] is in lowest 2 bits of fd */
#define get_ttyfd()     int layer1_fin = fd & 3; int ttyfd = fd / 4;\
                        if (fd < 0) {ttyfd = fd; layer1_fin = 0; /* use modbusTCP */}




/******************************************/
/******************************************/
/**                                      **/
/**         Global Variables...          **/
/**                                      **/
/******************************************/
/******************************************/
/* The layer 1 (RTU, ASCII, TCP) implementations will be adding some 
 *  header and tail bytes (e.g. CRC) to the packet we build here. Since
 *  layer1 will re-use the same buffer allocated in this slave layer 
 *  (so as not to continuosly copy the same info from buffer to buffer),
 *  we need to allocate more bytes than those strictly required for this
 *  slave layer. Therefore, the extra_bytes parameter.
 *
 *  Note that we add one more extra byte to the response buffer.
 *  This is because some response packets will not be starting off
 *  at byte 0, but rather at byte 1 of the buffer. This is in order
 *  to guarantee that the data that is sent on the buffer is aligned
 *  on even bytes (the 16 bit words!). This will allow the application
 *  (layer above the one implemented in this file - i.e. the callback 
 *  functions) to reference this memory as an u16 *, without producing
 *  'bus error' messages in some embedded devices that do not allow
 *   acessing u16 on odd numbered addresses.
 */
static int buff_extra_bytes_;
#define RESP_BUFFER_SIZE       (MAX_L2_FRAME_LENGTH + buff_extra_bytes_ + 1)

/******************************************/
/******************************************/
/**                                      **/
/**       Local Utility functions...     **/
/**                                      **/
/******************************************/
/******************************************/


/*
 * Function to determine next transaction id.
 *
 * We use a library wide transaction id, which means that we
 * use a new transaction id no matter what slave to which we will
 * be sending the request...
 */
static inline u16 next_transaction_id(void) {
  static u16 next_id = 0;
  return next_id++;
}


/* Determine endianess of platform... */

/* WARNING: The following files are being included:
 *              <stdib.h>  -->  <endian.h>  -->  <bits/endian.h>
 * 
 *          endian.h defines the following constants as:
 *            #define __LITTLE_ENDIAN  and LITTLE_ENDIAN  as 1234
 *            #define    __BIG_ENDIAN  and    BIG_ENDIAN  as 4321
 *            #define    __PDP_ENDIAN  and    PDP_ENDIAN  as 3412
 * 
 *          bits/endian.h defines the constant BYTE_ORDER as:
 *              #define __BYTE_ORDER as __LITTLE_ENDIAN
 * 
 *          endian.h then sets the following constants
 *          (if __USE_BSD is set, which seems to be true):
 *            # define LITTLE_ENDIAN    __LITTLE_ENDIAN
 *            # define BIG_ENDIAN       __BIG_ENDIAN
 *            # define PDP_ENDIAN       __PDP_ENDIAN
 *            # define BYTE_ORDER       __BYTE_ORDER
 */ 

/* If we still don't know byte order, try to get it from <endian.h> */
#ifndef __BYTE_ORDER
#include <endian.h>
#endif


/* If we still don't know byte order => if using gcc, use it to determine byte order... */
#ifndef __BYTE_ORDER
#if defined(__GNUC__) 
  /* We have GCC, which should define __LITTLE_ENDIAN__ */ 
#  if defined(__LITTLE_ENDIAN__)
#    define __BYTE_ORDER __LITTLE_ENDIAN
#  else
#    define __BYTE_ORDER __BIG_ENDIAN
#  endif
#endif /* __GNUC__ */ 
#endif /* __BYTE_ORDER */


#ifndef __BYTE_ORDER
#  error   "Unable to determine platform's byte order. Aborting compilation."
#elif   __BYTE_ORDER == __BIG_ENDIAN
#  warning "Compiling for BIG endian platform."
#elif   __BYTE_ORDER == __LITTLE_ENDIAN
#  warning "Compiling for LITTLE endian platform."
#else
#  error   "Aborting compilation due to unsuported byte order (neither BIG not LITTLE endian)."
#endif



/*
 * Functions to convert u16 variables
 * between network and host byte order
 *
 * NOTE: Modbus uses MSByte first, just like
 *       tcp/ip, so we could be tempted to use the htons() and
 *       ntohs() functions to guarantee code portability.
 *
 *       However, on some embedded systems running Linux
 *       these functions only work if the 16 bit words are 
 *       stored on even addresses. This is not always the 
 *       case in our code, so we have to define our own
 *       conversion functions...
 */


#ifdef __BYTE_ORDER
# if __BYTE_ORDER == __LITTLE_ENDIAN

/**************************************************************/
/* u16 conversion functions to use on little endian platforms */
/**************************************************************/

/* NOTE: The input parameter must be the address 
 *          of an u16 passed as a pointer to u8
 * 
 *       We use u8 *ptr as input parameter and read both (ptr+0) and (ptr+1)
 *       instead of using u16 *ptr because we sometimes receive data in packtes
 *       that are not aligned on even addresses, so some compilers recognize that
 *       the given odd address cannot be used as a pointer to u16 and therefore 
 *       adjust the pointer by (+1) or (-1), basicaly breacking our code!
 *       So, we revert to u8 pointers... for u16 values.
 */
static inline void mb_hton(u8 *u16_from_ptr, u8 *u16_to_ptr) {
  u16_to_ptr[0] =  u16_from_ptr[1];
  u16_to_ptr[1] =  u16_from_ptr[0];
}
#define mb_ntoh(a, b) mb_hton(a, b)


static inline void mb_hton_count(u8 *u16_ptr, unsigned count) {
  unsigned i;
  for (i = 0; i < count*2; i += 2) {
    /* swap the bytes around... 
     *  a = a ^ b;
     *  b = a ^ b;
     *  a = a ^ b;
     */
    (u16_ptr+i)[0] ^= (u16_ptr+i)[1]; 
    (u16_ptr+i)[1] ^= (u16_ptr+i)[0]; 
    (u16_ptr+i)[0] ^= (u16_ptr+i)[1]; 
  }
}
#define mb_ntoh_count(w, count) mb_hton_count(w, count)



# else
#  if __BYTE_ORDER == __BIG_ENDIAN
/***********************************************************/
/* u16 conversion functions to use on big endian platforms */
/***********************************************************/

/* We don't need to swap the bytes around! */
static inline void mb_hton(u8 *u16_from_ptr, u8 *u16_to_ptr) {
  u16_to_ptr[0] =  u16_from_ptr[0];
  u16_to_ptr[1] =  u16_from_ptr[1];
}
#define mb_ntoh(a, b) mb_hton(a, b)

#define mb_hton_count(w, count) /* empty ! */
#define mb_ntoh_count(w, count) /* empty ! */


#  else
/********************************************************/
/* u16 conversion functions to use on generic platforms */
/********************************************************/


/* We can't determine endiannes at compile time, so we do it at runtime.
 * With any luck the compiler will be able to determine the result of the
 * comparison at compile time and end up discarding the non-used code
 * and the 'if' itself from the final executable.
 */

static union {u16 u16;
              u8  u8[2];} endian_ = 0x0102;


static inline void mb_hton(u8 *u16_from_ptr, u8 *u16_to_ptr) {
  if (endian_.u8[0] == 0x01) {
    /* machine is big endian -> no swapping */    
    u16_to_ptr[0] =  u16_from_ptr[0];
    u16_to_ptr[1] =  u16_from_ptr[1];
  } else {
    /* machine is little endian -> we swap bytes around */    
    u16_to_ptr[0] =  u16_from_ptr[1];
    u16_to_ptr[1] =  u16_from_ptr[0];
  }
}
#define mb_ntoh(a, b) mb_hton(a, b)


static inline void mb_hton_count(u8 *u16_ptr, unsigned count) {
  unsigned i;
  
  if (endian_.u8[0] == 0x01)
      /* machine is big endian. Nothing to do */
      return;
  
  /* machine is little endian -> we swap bytes around */    
  for (i = 0; i < count*2; i += 2) {
    /* swap the bytes around... 
     *  a = a ^ b;
     *  b = a ^ b;
     *  a = a ^ b;
     */
    (u16_ptr+i)[0] ^= (u16_ptr+i)[1]; 
    (u16_ptr+i)[1] ^= (u16_ptr+i)[0]; 
    (u16_ptr+i)[0] ^= (u16_ptr+i)[1]; 
  }
}
#define mb_ntoh_count(w, count) mb_hton_count(w, count)

#  endif
# endif
#endif /* __BYTE_ORDER */










/***********************************************/
/***********************************************/
/**                                           **/
/**    Handle requests from master/client     **/
/**                                           **/
/***********************************************/
/***********************************************/


/* Handle functions 0x01 and 0x02 */
typedef int (*read_bits_callback_t)(void *arg, u16 start_addr, u16 bit_count,  u8  *data_bytes);
static int handle_read_bits (u8 *query_packet,
                             u8 **resp_packet_ptr,
                             u8 *error_code,
                             read_bits_callback_t read_bits_callback,
                             void *callback_arg
                            ) {
  u16 start_addr, count;
  int res;
  u8 *resp_packet;
  
  /* If no callback, handle as if function is not supported... */
  if (read_bits_callback == NULL) 
    {*error_code = ERR_ILLEGAL_FUNCTION; return -1;}
  
  /* in oprder for the data in this packet to be aligned on even numbered addresses, this 
   *  response packet will start off at an odd numbered byte...
   *  We therefore add 1 to the address where the packet starts.
   */ 
  (*resp_packet_ptr)++;
  resp_packet = *resp_packet_ptr;
  
    /* NOTE:
     *  Modbus uses high level addressing starting off from 1, but
     *  this is sent as 0 on the wire!
     *  We could expect the user to specify high level addressing 
     *   starting at 1, and do the conversion to start off at 0 here.
     *   However, to do this we would then need to use an u32 data type
     *   to correctly hold the address supplied by the user (which could
     *   correctly be 65536, which does not fit in an u16), which would
     *   in turn require us to check whether the address supplied by the user
     *   is correct (i.e. <= 65536). 
     *  I decided to go with the other option of using an u16, and 
     *   requiring the user to use addressing starting off at 0! 
     */
  mb_ntoh(&(query_packet[2]), (u8 *)&start_addr);
  mb_ntoh(&(query_packet[4]), (u8 *)&count); 

  #ifdef DEBUG
  printf("handle_read_input_bits() called. slave=%d, function=%d, start_addr=%d, count=%d\n", 
          query_packet[0], query_packet[1], start_addr, count);
  #endif

  if ((count > MAX_READ_BITS) || (count < 1)) 
    {*error_code = ERR_ILLEGAL_DATA_VALUE; return -1;}
  
  /* Remember, we are using addressing starting off at 0, in the start_addr variable! */
  /*  This means that he highest acceptable address is 65535, when count=1 .... */
  /* Note the use of 65536 in the comparison will force automatic upgrade of u16 variables! */
  /*    => start_addr + count will nver overflow the u16 type!                              */
  if (start_addr + count > 65536) 
    {*error_code = ERR_ILLEGAL_DATA_ADDRESS; return -1;}
  
  /* start building response frame... */
  resp_packet[0] = query_packet[0]; /* slave */ 
  resp_packet[1] = query_packet[1]; /* function (either 0x01 or 0x02 ! */
  resp_packet[2] = (count + 7) / 8; /* number of data bytes = ceil(count/8) */
  
  res = read_bits_callback(callback_arg, start_addr, count, &(resp_packet[3]));
  if (res == -2) {*error_code = ERR_ILLEGAL_DATA_ADDRESS; return -1;}
  if (res  <  0) {*error_code = ERR_SLAVE_DEVICE_FAILURE; return -1;}
  
  return resp_packet[2] + 3; /* packet size is data length + 3 bytes -> slave, function, count */
}



/* Handle function 0x01 */
int handle_read_output_bits  (u8 *query_packet, u8 **resp_packet_ptr, u8 *error_code, mb_slave_callback_t *callbacks)
  {return handle_read_bits(query_packet, resp_packet_ptr, error_code, callbacks->read_outbits, callbacks->arg);}

/* Handle function 0x02 */
int handle_read_input_bits   (u8 *query_packet, u8 **resp_packet_ptr, u8 *error_code, mb_slave_callback_t *callbacks)
  {return handle_read_bits(query_packet, resp_packet_ptr, error_code, callbacks->read_inbits, callbacks->arg);}




/* Handle functions 0x03 and 0x04 */
typedef int (*read_words_callback_t)(void *arg, u16 start_addr, u16 word_count, u16 *data_words);
static int handle_read_words (u8 *query_packet, 
                              u8 **resp_packet_ptr, 
                              u8 *error_code, 
                              read_words_callback_t read_words_callback,
                              void *callback_arg
                             ) {
  u16 start_addr, count;
  int res;
  u8 *resp_packet;

  /* If no callback, handle as if function is not supported... */
  if (read_words_callback == NULL) 
    {*error_code = ERR_ILLEGAL_FUNCTION; return -1;}
  
  /* See equivalent comment in handle_read_bits() */ 
  (*resp_packet_ptr)++;
  resp_packet = *resp_packet_ptr;
  
  /* See equivalent comment in handle_read_bits() */ 
  mb_ntoh(&(query_packet[2]), (u8 *)&start_addr);
  mb_ntoh(&(query_packet[4]), (u8 *)&count);

  #ifdef DEBUG
  printf("handle_read_output_words() called. slave=%d, function=%d, start_addr=%d, count=%d\n", 
         query_packet[0], query_packet[1], start_addr, count);
  #endif

  if ((count > MAX_READ_REGS) || (count < 1))
    {*error_code = ERR_ILLEGAL_DATA_VALUE; return -1;}
  
  /* See equivalent comment in handle_read_bits() */ 
  if (start_addr + count > 65536)
    {*error_code = ERR_ILLEGAL_DATA_ADDRESS; return -1;}
  
  /* start building response frame... */
  resp_packet[0] = query_packet[0]; /* slave     */
  resp_packet[1] = query_packet[1]; /* function code, either 0x03 or 0x04 !!!*/
  resp_packet[2] = count * 2;       /* number of bytes of data... */
  
  res = read_words_callback(callback_arg, start_addr, count, (u16 *)&(resp_packet[3]));
  if (res == -2) {*error_code = ERR_ILLEGAL_DATA_ADDRESS; return -1;}
  if (res  <  0) {*error_code = ERR_SLAVE_DEVICE_FAILURE; return -1;}
  
  /* convert all data from host to network byte order. */
  mb_hton_count(&(resp_packet[3]), count);
  
  return resp_packet[2] + 3; /* packet size is data length + 3 bytes -> slave, function, count */
}




/* Handle function 0x03 */
int handle_read_output_words (u8 *query_packet, u8 **resp_packet_ptr, u8 *error_code, mb_slave_callback_t *callbacks) 
  {return handle_read_words(query_packet, resp_packet_ptr, error_code, callbacks->read_outwords, callbacks->arg);}

/* Handle function 0x04 */
int handle_read_input_words  (u8 *query_packet, u8 **resp_packet_ptr, u8 *error_code, mb_slave_callback_t *callbacks) 
  {return handle_read_words(query_packet, resp_packet_ptr, error_code, callbacks->read_inwords, callbacks->arg);}



/* Handle function 0x05 */
int handle_write_output_bit  (u8 *query_packet, u8 **resp_packet_ptr, u8 *error_code, mb_slave_callback_t *callbacks) {
  u16 start_addr;
  int res;
  u8 *resp_packet;
  
  /* If no callback, handle as if function is not supported... */
  if (callbacks->write_outbits == NULL)
    {*error_code = ERR_ILLEGAL_FUNCTION; return -1;}
  
  resp_packet = *resp_packet_ptr;
  
  /* See equivalent comment in handle_read_bits() */ 
  mb_ntoh(&(query_packet[2]), (u8 *)&start_addr);

  #ifdef DEBUG
  printf("handle_write_output_bit() called. slave=%d, function=%d, start_addr=%d\n", 
         query_packet[0], query_packet[1], start_addr);
  #endif

  // byte 5 Must be 0x00, byte 4 must be 0x00 or 0xFF !!
  if ( (query_packet[5] != 0) || 
      ((query_packet[4] != 0) && (query_packet[4] != 0xFF)))
    {*error_code = ERR_ILLEGAL_DATA_VALUE; return -1;}
  
  /* Address will always be valid, no need to check! */
  // if (start_addr > 65535) {*error_code = ERR_ILLEGAL_DATA_ADDRESS; return -1;}
  
  /* start building response frame... */
  resp_packet[0] = query_packet[0]; /* slave */ 
  resp_packet[1] = query_packet[1]; /* function */ 
  resp_packet[2] = query_packet[2]; /* start address - hi byte */
  resp_packet[3] = query_packet[3]; /* start address - lo byte */
  resp_packet[4] = query_packet[4]; /* value: 0x00 or 0xFF */
  resp_packet[5] = query_packet[5]; /* value: must be 0x00 */
  
  res = (callbacks->write_outbits)(callbacks->arg, start_addr, 1, &(query_packet[4]));
  if (res == -2) {*error_code = ERR_ILLEGAL_DATA_ADDRESS; return -1;}
  if (res  <  0) {*error_code = ERR_SLAVE_DEVICE_FAILURE; return -1;}
  
  return 6; /* response packet size, including slave id in byte 0 */
}



/* Handle function 0x06 */
int handle_write_output_word (u8 *query_packet, u8 **resp_packet_ptr, u8 *error_code, mb_slave_callback_t *callbacks) {
  u16 start_addr;
  int res;
  u8 *resp_packet;
  
  /* If no callback, handle as if function is not supported... */
  if (callbacks->write_outwords == NULL)
    {*error_code = ERR_ILLEGAL_FUNCTION; return -1;}
  
  resp_packet = *resp_packet_ptr;
  
  /* See equivalent comment in handle_read_bits() */ 
  mb_ntoh(&(query_packet[2]), (u8 *)&start_addr);
  
  #ifdef DEBUG
  printf("handle_write_output_word() called. slave=%d, function=%d, start_addr=%d\n", 
         query_packet[0], query_packet[1], start_addr);
  #endif

  /* Address will always be valid, no need to check! */
  // if (start_addr > 65535) {*error_code = ERR_ILLEGAL_DATA_ADDRESS; return -1;}
  
  /* start building response frame... */
  resp_packet[0] = query_packet[0]; /* slave           */
  resp_packet[1] = query_packet[1]; /* function        */ 
  resp_packet[2] = query_packet[2]; /* start address - hi byte */
  resp_packet[3] = query_packet[3]; /* start address - lo byte */
  resp_packet[4] = query_packet[4]; /* value - hi byte */
  resp_packet[5] = query_packet[5]; /* value - lo byte */
  
  /* convert data from network to host byte order */
  mb_ntoh_count(&(query_packet[4]), 1);
  
  res = (callbacks->write_outwords)(callbacks->arg, start_addr, 1, (u16 *)&(query_packet[4]));
  if (res == -2) {*error_code = ERR_ILLEGAL_DATA_ADDRESS; return -1;}
  if (res  <  0) {*error_code = ERR_SLAVE_DEVICE_FAILURE; return -1;}
  
  return 6; /* packet size is 6 -> slave, function, addr(2), value(2) */
}



/* Handle function 0x0F */
int handle_write_output_bits (u8 *query_packet, u8 **resp_packet_ptr, u8 *error_code, mb_slave_callback_t *callbacks) {
  u16 start_addr, count;
  int res;
  u8 *resp_packet;
  
  /* If no callback, handle as if function is not supported... */
  if (callbacks->write_outbits == NULL)
    {*error_code = ERR_ILLEGAL_FUNCTION; return -1;}
  
  resp_packet = *resp_packet_ptr;
  
  /* See equivalent comment in handle_read_bits() */ 
  mb_ntoh(&(query_packet[2]), (u8 *)&start_addr);
  mb_ntoh(&(query_packet[4]), (u8 *)&count); 

  #ifdef DEBUG
  printf("handle_write_output_bits() called. slave=%d, function=%d, start_addr=%d, count=%d\n", 
         query_packet[0], query_packet[1], start_addr, count);
  #endif

  if ((count > MAX_WRITE_COILS) || (count < 1) || ((count+7)/8 != query_packet[6]) )
    {*error_code = ERR_ILLEGAL_DATA_VALUE; return -1;}
  
  /* See equivalent comment in handle_read_bits() */ 
  if (start_addr + count > 65536)
    {*error_code = ERR_ILLEGAL_DATA_ADDRESS; return -1;}
  
  /* start building response frame... */
  resp_packet[0] = query_packet[0]; /* slave */ 
  resp_packet[1] = query_packet[1]; /* function */ 
  resp_packet[2] = query_packet[2]; /* start address - hi byte */
  resp_packet[3] = query_packet[3]; /* start address - lo byte */
  resp_packet[4] = query_packet[4]; /* count - hi byte */
  resp_packet[5] = query_packet[5]; /* count - lo byte */
  
  res = (callbacks->write_outbits)(callbacks->arg, start_addr, count, &(query_packet[7]));
  if (res == -2) {*error_code = ERR_ILLEGAL_DATA_ADDRESS; return -1;}
  if (res  <  0) {*error_code = ERR_SLAVE_DEVICE_FAILURE; return -1;}
  
  return 6; /* packet size is 6 -> slave, function, addr(2), count(2) */
}




/* Handle function 0x10 */
int handle_write_output_words(u8 *query_packet, u8 **resp_packet_ptr, u8 *error_code, mb_slave_callback_t *callbacks) {
  u16 start_addr, count;
  int res;
  u8 *resp_packet;
  
  /* If no callback, handle as if function is not supported... */
  if (callbacks->write_outwords == NULL)
    {*error_code = ERR_ILLEGAL_FUNCTION; return -1;}
  
  resp_packet = *resp_packet_ptr;
  
  /* See equivalent comment in handle_read_bits() */ 
  mb_ntoh(&(query_packet[2]), (u8 *)&start_addr);
  mb_ntoh(&(query_packet[4]), (u8 *)&count); 
  
  if ((count > MAX_WRITE_REGS) || (count < 1) || (count*2 != query_packet[6]) )
    {*error_code = ERR_ILLEGAL_DATA_VALUE; return -1;}
  
  /* See equivalent comment in handle_read_bits() */ 
  if (start_addr + count > 65536)
    {*error_code = ERR_ILLEGAL_DATA_ADDRESS; return -1;}
  
  /* start building response frame... */
  resp_packet[0] = query_packet[0]; /* slave           */
  resp_packet[1] = query_packet[1]; /* function        */ 
  resp_packet[2] = query_packet[2]; /* start address - hi byte */
  resp_packet[3] = query_packet[3]; /* start address - lo byte */
  resp_packet[4] = query_packet[4]; /* count - hi byte */
  resp_packet[5] = query_packet[5]; /* count - lo byte */
  
  /* convert all data from network to host byte order */
  mb_ntoh_count(&(query_packet[7]), count);
  
  res = (callbacks->write_outwords)(callbacks->arg, start_addr, count, (u16 *)&(query_packet[7]));
  if (res == -2) {*error_code = ERR_ILLEGAL_DATA_ADDRESS; return -1;}
  if (res  <  0) {*error_code = ERR_SLAVE_DEVICE_FAILURE; return -1;}
  
  return 6; /* packet size is 6 -> slave, function, addr(2), count(2) */
}








/***********************************************/
/***********************************************/
/**                                           **/
/**    initialise / shutdown the library      **/
/**                                           **/
/***********************************************/
/***********************************************/

int mb_slave_init__(int extra_bytes) {
  buff_extra_bytes_ = extra_bytes;
  return 0;
}


int mb_slave_done__(void) 
  {return 0;}


#if 0
int mb_slave_init(int nd_count) {
  int extra_bytes;

  #ifdef DEBUG
  fprintf( stderr, "mb_slave_init()\n");
  fprintf( stderr, "creating %d nodes\n", nd_count);
  #endif

  /* initialise layer 1 library */
  if (modbus_init(nd_count, DEF_OPTIMIZATION, &extra_bytes) < 0)
    goto error_exit_0;

  /* initialise this library */
  if (mb_slave_init__(extra_bytes) < 0)
    goto error_exit_1;

  return 0;

error_exit_1:
  modbus_done();
error_exit_0:
  return -1;
}


int mb_slave_done(void) {
  mb_slave_done__(void)
  return modbus_done();
}
#endif



/***********************************************/
/***********************************************/
/**                                           **/
/**        open/close slave connection        **/
/**                                           **/
/***********************************************/
/***********************************************/

/* Create a new slave/server */
/* NOTE: We use the lower 2 bits of the returned node id to identify which 
 *       layer1 implementation to use. 
 *           0 -> TCP 
 *           1 -> RTU 
 *           2 -> ASCII 
 *           4 -> unused 
 *       The node id used by the layer1 is shifted left 2 bits
 *       before returning the node id to the caller!
 */
int mb_slave_new(node_addr_t node_addr) {
  int res = -1;
  #ifdef DEBUG
  fprintf( stderr, "mb_slave_connect()\n");
  #endif

  /* call layer 1 library */
  switch(node_addr.naf) {
    case naf_tcp:  
      res = modbus_tcp_listen(node_addr);
      if (res >= 0) res = res*4 + 0 /* offset into fptr_ with TCP functions */;
      return res;
    case naf_rtu:  
      res = modbus_rtu_listen(node_addr);
      if (res >= 0) res = res*4 + 1 /* offset into fptr_ with RTU functions */;
      return res;
    case naf_ascii:  
      res = modbus_ascii_listen(node_addr);
      if (res >= 0) res = res*4 + 2 /* offset into fptr_ with ASCII functions */;
      return res;
  }

  return -1;
}




int mb_slave_close(int fd) {
  #ifdef DEBUG
  fprintf( stderr, "mb_slave_close(): nd = %d\n", fd);
  #endif
  get_ttyfd(); /* declare the ttyfd variable!! */
  /* call layer 1 library */
  /* will call one of modbus_tcp_close(), modbus_rtu_close(), modbus_ascii_close() */
  return modbus_close(ttyfd);
}





/***********************************************/
/***********************************************/
/**                                           **/
/**               Run the slave               **/
/**                                           **/
/***********************************************/
/***********************************************/

/* Execute infinite loop waiting and replying to requests coming from clients/master
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

int mb_slave_run(int fd, mb_slave_callback_t callback_functions, u8 slaveid) {
  int byte_count;
  u16 transaction_id;
  int nd;
  u8 function, error_code = 0;
  int resp_length;
  u8 *query_packet = NULL;
  u8 *resp_packet;
  u8  resp_buffer_[RESP_BUFFER_SIZE];
  u8  slave;
  
  get_ttyfd(); /* declare the ttyfd variable!! */

  #ifdef DEBUG  
  fprintf(stderr,"[%lu] mb_slave_run(): Called... fd=%d, ttyfd=%d\n", pthread_self(), fd, ttyfd);
  #endif

  while(1) {
    nd = ttyfd;
    /* will call one of modbus_tcp_read(), modbus_rtu_read(), modbus_ascii_read() */
    do {
        byte_count = modbus_read(&nd,              /* node descriptor          */
                                 &query_packet,    /* u8 **recv_data_ptr,      */
                                 &transaction_id,  /* u16 *transaction_id,     */
                                 NULL,             /* const u8 *send_data,     */
                                 0,                /* int send_length,         */
                                 NULL  /* wait indefenitely */ /* const struct timespec *recv_timeout); */
                                );
    } while (byte_count <= 2);

    #ifdef DEBUG
    {/* display the hex code of each character received */
      int i;
      printf("[%lu] mb_slave_run() received %d bytes (ptr=%p): \n", pthread_self(), byte_count, query_packet);
      for (i=0; i < byte_count; i++)
        printf("<0x%2X>", query_packet[i]);
      printf("\n");
    }
    #endif

    slave    = query_packet[0];
    function = query_packet[1];
    
    /* We only reply if:
     *       - request was sent to broadcast address   (slave   == 0)
     *  OR   - we were asked to reply to every request (slaveid == 0)
     *  OR   - request matches the slaveid we were asked to accept (slave == slaveid)
     * 
     * Otherwise, silently ignore the received request!!!
     */
    if ((slaveid == 0) || (slave == 0) || (slave == slaveid)) {
      resp_packet = resp_buffer_;
      
      switch(function) {
        case 0x01: resp_length = handle_read_output_bits  (query_packet, &resp_packet, &error_code, &callback_functions); break;
        case 0x02: resp_length = handle_read_input_bits   (query_packet, &resp_packet, &error_code, &callback_functions); break;
        case 0x03: resp_length = handle_read_output_words (query_packet, &resp_packet, &error_code, &callback_functions); break;
        case 0x04: resp_length = handle_read_input_words  (query_packet, &resp_packet, &error_code, &callback_functions); break;
        case 0x05: resp_length = handle_write_output_bit  (query_packet, &resp_packet, &error_code, &callback_functions); break;
        case 0x06: resp_length = handle_write_output_word (query_packet, &resp_packet, &error_code, &callback_functions); break;
        case 0x0F: resp_length = handle_write_output_bits (query_packet, &resp_packet, &error_code, &callback_functions); break;
        case 0x10: resp_length = handle_write_output_words(query_packet, &resp_packet, &error_code, &callback_functions); break;
        /* return exception code 0x01 -> function not supported! */
        default :  resp_length = -1; error_code = 0x01; break; 
      }; /* switch(function) */
      
      if (resp_length < 0) {
        /* return error... */
        /* build exception response frame... */
        resp_packet = resp_buffer_;
        resp_packet[0] = query_packet[0]; /* slave */ 
        resp_packet[1] = query_packet[1] | 0x80; /* function code with error bit activated! */ 
        resp_packet[2] = error_code; 	
        resp_length = 3;
      }
      modbus_write(nd, resp_packet, resp_length, transaction_id, NULL /*transmit_timeout*/);
    }; /* if not ignore request */
  }; /* while(1) */
  
  /* humour the compiler... */	
  return 0;
}













