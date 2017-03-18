/*
 * Copyright (c) 2001-2003,2016 Mario de Sousa (msousa@fe.up.pt)
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


/* mb_master.c */


#include <fcntl.h>	/* File control definitions */
#include <stdio.h>	/* Standard input/output */
#include <string.h>
#include <stdlib.h>
#include <termio.h>	/* POSIX terminal control definitions */
#include <sys/time.h>	/* Time structures for select() */
#include <unistd.h>	/* POSIX Symbolic Constants */
#include <errno.h>	/* Error definitions */

#include <pthread.h>    /* pthread_mutex_[un]lock() */

#include <netinet/in.h> /* required for htons() and ntohs() */
#include "mb_layer1.h"
#include "mb_master.h"
#include "mb_master_private.h"

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
#define get_ttyfd()     int layer1_fin = fd & 3; int ttyfd = fd / 4;



/******************************************/
/******************************************/
/**                                      **/
/**         Global Variables...          **/
/**                                      **/
/******************************************/
/******************************************/
   /* The layer 1 (RTU, ASCII, TCP) implementation will be adding some headers and CRC (at the end)
    *  of the packet we build here (actually currently it is only at the end). Since we want to 
    *  re-use the same buffer so as not to continuosly copy the same info from buffer to buffer,
    *  we need tp allocate more bytes than the ones we need for this layer. Therefore, the
    *  extra_bytes parameter.
    *
    *  Note that we add one more extra byte. This is because some packets will not be 
    *  starting off at byte 0, but rather at byte 1 of the buffer. This is in order to guarantee
    *  that the data that is sent on the buffer is aligned on even bytes (the 16 bit words!).
    *  This will allow us to reference this memory as an u16 *, without producing 'bus error'
    *  messages in some embedded devices that do not allow acessing u16 on odd numbered addresses.
    */ 
static int buff_extra_bytes_;
#define QUERY_BUFFER_SIZE       (MAX_L2_FRAME_LENGTH + buff_extra_bytes_ + 1)


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


/*
 * Functions to convert u16 variables
 * between network and host byte order
 *
 * NOTE: Modbus uses MSByte first, just like
 *       tcp/ip, so we use the htons() and
 *       ntoh() functions to guarantee
 *       code portability.
 */
static inline u16 mb_hton(u16 h_value) {return htons(h_value);}
static inline u16 mb_ntoh(u16 m_value) {return ntohs(m_value);}
static inline u8  msb    (u16   value) {return (value >> 8) & 0xFF;}
static inline u8  lsb    (u16   value) {return  value & 0xFF;}




/*************************************************/
/*************************************************/
/**                                             **/
/**   Common functions for Modbus Protocol.     **/
/**                                             **/
/*************************************************/
/*************************************************/

/* build the common elements of a query frame */
static inline int build_packet(u8  slave,
                               u8  function,
                               u16 start_addr,
                               u16 count,
                               u8 *packet) {
  union {
      u16 u16;
      u8  u8[2];
  } tmp;
  
  packet[0] = slave,
  packet[1] = function;
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
  /* NOTE: we do not use up casting - i.e. the following
   *       *((u16 *)(packet+2)) = mb_hton(start_addr);
   *       because packet+2 is NOT aligned with an even address, and would
   *       therefore result in 'bus error' when using compilers that do not 
   *       automatically do the required decomposing of this supposedly 
   *       single bus access into two distinct bus accesses.
   *       (Note that some compilers do do this decomposing automatically
   *       in which case the following is not necessary).
   *       At the moment, I (Mario de Sousa) know of at least one cross-compiler
   *       that does not do the decomposing automatically, i.e. the 
   *       AVR32 cross-compiler.
   */
  tmp.u16 = mb_hton(start_addr);
  packet[2] = tmp.u8[0];
  packet[3] = tmp.u8[1];
  tmp.u16 = mb_hton(count);
  packet[4] = tmp.u8[0];
  packet[5] = tmp.u8[1];
  
  return 6;
}



/* Execute a Query/Response transaction between client and server */ 
/* returns: <0    -> ERROR: error codes
 *          >2    -> SUCCESS: frame length
 *           0..2 -> will never be returned!
 */
static int mb_transaction(u8  *packet,
                          int query_length,
                          u8  **data,
                          int fd,
                          int send_retries,
                          u8  *error_code,
                          const struct timespec *response_timeout) {
  int error = TIMEOUT;
  int response_length = INTERNAL_ERROR;
  u16 send_transaction_id, recv_transaction_id;
  get_ttyfd(); /* declare the ttyfd variable, ... */
  
    /* We must also initialize the recv_transaction_id with the same value,
     * since some layer 1 protocols do not support transaction id's, so
     * simply return the recv_transaction_id variable without any changes...
     */
  /* NOTE: we re-use the same transaction id for all send re-tries., since, in truth, 
   * it is still the same transaction. This will also simplify re-synchronising with
   * some slaves that keep a buffer of outstanding requests, and will reply to all of
   * them, in FIFO order. In this case, once an error occurs we will be swamping the
   * slave with requests. By using the same transaction id, we may correctly consider
   * the reply to the first request sent as the reply to the third request! This means
   * we stop re-trying the sending of further requests, and no longer swamp the slave...
   */
  send_transaction_id = recv_transaction_id = next_transaction_id();

  for (send_retries++; send_retries > 0; send_retries--) {
    error = TIMEOUT;

    if (modbus_write(ttyfd, packet, query_length, send_transaction_id, response_timeout) < 0)
      {error = PORT_FAILURE; continue;}

      /* if we receive a correct response but with a wrong transaction id or wrong modbus function, we try to 
       * receive another frame instead of returning an error or re-sending the request! This first frame could 
       * have been a response to a previous request of ours that timed out waiting for a response, and the 
       * response we are waiting for could be coming 'any minute now'.
       */
    do {
      response_length = modbus_read(&ttyfd, data, &recv_transaction_id,
                                    packet, query_length, response_timeout);

      /* TIMEOUT condition */
      /* However, if we had previously received an invalid frame, or some other error,
       * we return that error instead!
       * Note that the 'error' variable was initialised with the TIMEOUT error
       * condition, so if no previous error ocurred, we will be returning the
       * TIMEOUT error condition here!
       */
      if(response_length == -2)  return error;
      /* NOTE we want to break out of this while loop without even running the while()
       * condition, as that condition is only valid if response_length > 3 !!
       */
      if(response_length  <  0)  {error = PORT_FAILURE; break;}
      /* This should never occur! Modbus_read() should only return valid frames! */
      if(response_length  <  3)  return INTERNAL_ERROR;

    } while (/* we have the wrong transaction id */
             (send_transaction_id != recv_transaction_id)
             /* not a response frame to _our_ query */
            ||
             (((*data)[1] & ~0x80) != packet[1])
            /* NOTE: no need to check whether (*data)[0] = slave!              */
            /*       This has already been done by the modbus_read() function! */
            );

    if(response_length < 0)  {error = PORT_FAILURE; continue;}

    /* Now check whether we received a Modbus Exception frame */
    if (((*data)[1] & 0x80) != 0) {       /* we have an exception frame! */
      /* NOTE: we have already checked above that data[2] exists! */
      if (error_code != NULL)  *error_code = (*data)[2];
      return MODBUS_ERROR;
    }
    /* success! Let's get out of the send retry loop... */
    return response_length;
  }
  /* reached the end of the retries... */
  return error;
}


/**************************************/
/**************************************/
/**                                  **/
/**   Modbus Protocol Functions.     **/
/**                                  **/
/**************************************/
/**************************************/



/* Execute a transaction for functions that READ BITS.
 * Bits are stored on an int array, one bit per int.
 * Called by:  read_input_bits()
 *             read_output_bits()
 */
static int read_bits(u8  function,
                     u8  slave,
                     u16 start_addr,
                     u16 count,
                     u16 *dest,
                     int dest_size,
                     int ttyfd,
                     int send_retries,
                     u8  *error_code,
                     const struct timespec *response_timeout,
                     pthread_mutex_t *data_access_mutex) {
  
  u8 packet[QUERY_BUFFER_SIZE];
  u8 *data;
  int response_length, query_length;
  int temp, i, bit, dest_pos = 0;
  int coils_processed = 0;
  
  query_length = build_packet(slave, function, start_addr, count, packet);
  if (query_length < 0)  return INTERNAL_ERROR;
  
  response_length = mb_transaction(packet, query_length, &data, ttyfd,
                                   send_retries, error_code, response_timeout);
  
  if (response_length  < 0)                  return response_length;
  /* NOTE: Integer division. (count+7)/8 is equivalent to ceil(count/8) */
  if (response_length != 3 + (count+7)/8)    return INVALID_FRAME;
  if (data[2]         !=     (count+7)/8)    return INVALID_FRAME;
  
  if (NULL != data_access_mutex) pthread_mutex_lock(data_access_mutex);
  for( i = 0; (i < data[2]) && (i < dest_size); i++ ) {
    temp = data[3 + i];
    for( bit = 0x01; (bit & 0xff) && (coils_processed < count); ) {
      dest[dest_pos] = (temp & bit)?1:0;
      coils_processed++;
      dest_pos++;
      bit = bit << 1;
    }
  }
  if (NULL != data_access_mutex) pthread_mutex_unlock(data_access_mutex);
  
  return response_length;
}



/* Execute a transaction for functions that READ BITS.
 * Bits are stored on an u32 array, 32 bits per u32.
 * Unused bits in last u32 are set to 0.
 * Called by:  read_input_bits_u32()
 *             read_output_bits_u32()
 */
static int read_bits_u32(u8  function,
                         u8  slave,
                         u16 start_addr,
                         u16 count, /* number of bits !! */
                         u32 *dest,
                         int ttyfd,
                         int send_retries,
                         u8  *error_code,
                         const struct timespec *response_timeout) {
  u8 packet[QUERY_BUFFER_SIZE];
  u8 *data;
  int response_length, query_length;                         
  int byte_count, i, dest_pos = 0;
  
  query_length = build_packet(slave, function, start_addr, count, packet);
  if (query_length < 0)  return INTERNAL_ERROR;
  
  response_length = mb_transaction(packet, query_length, &data, ttyfd,
                                   send_retries, error_code, response_timeout);
  
  if (response_length < 0)                   return response_length;  
  /* NOTE: Integer division. (count+7)/8 is equivalent to ceil(count/8) */
  if (response_length != 3 + (count+7)/8)    return INVALID_FRAME;
  if (data[2]         !=     (count+7)/8)    return INVALID_FRAME;
  
  byte_count = data[2];
  data += 3;
  /* handle groups of 4 bytes... */
  for(i = 0, dest_pos = 0; i + 3 < byte_count; i += 4, dest_pos++)
    dest[dest_pos] = data[i] + data[i+1]*0x100 + data[i+2]*0x10000 + data[i+3]*0x1000000;
  /* handle any remaining bytes... begining with the last! */
  if (i < byte_count) dest[dest_pos] = 0;
  for(byte_count--; i <= byte_count; byte_count--)
    dest[dest_pos] = dest[dest_pos]*0x100 + data[byte_count];
  
  return response_length;
}



/* FUNCTION 0x01   - Read Coils
 * Bits are stored on an int array, one bit per int.
 */
int read_output_bits(u8  slave,
                     u16 start_addr,
                     u16 count,
                     u16 *dest,
                     int dest_size,
                     int ttyfd,
                     int send_retries,
                     u8  *error_code,
                     const struct timespec *response_timeout,
                     pthread_mutex_t *data_access_mutex) {
  if( count > MAX_READ_BITS ) {
    count = MAX_READ_BITS;
    #ifdef DEBUG
    fprintf( stderr, "Too many coils requested.\n" );
    #endif
  }

  return read_bits(0x01 /* function */,
                   slave, start_addr, count, dest, dest_size, ttyfd, 
                   send_retries, error_code, response_timeout, data_access_mutex);
}



/* FUNCTION 0x01   - Read Coils
 * Bits are stored on an u32 array, 32 bits per u32.
 * Unused bits in last u32 are set to 0.
 */
int read_output_bits_u32(u8  slave,
                         u16 start_addr,
                         u16 count,
                         u32 *dest,
                         int ttyfd,
                         int send_retries,
                         u8  *error_code,
                         const struct timespec *response_timeout) {
  if( count > MAX_READ_BITS ) {
    count = MAX_READ_BITS;
    #ifdef DEBUG
    fprintf( stderr, "Too many coils requested.\n" );
    #endif
  }

  return read_bits_u32(0x01 /* function */,
                       slave, start_addr, count, dest, ttyfd,
                       send_retries, error_code, response_timeout);
}


/* FUNCTION 0x02   - Read Discrete Inputs
 * Bits are stored on an int array, one bit per int.
 */
int read_input_bits(u8  slave,
                    u16 start_addr,
                    u16 count,
                    u16 *dest,
                    int dest_size,
                    int ttyfd,
                    int send_retries,
                    u8  *error_code,
                    const struct timespec *response_timeout,
                    pthread_mutex_t *data_access_mutex) {
  if( count > MAX_READ_BITS ) {
    count = MAX_READ_BITS;
    #ifdef DEBUG
    fprintf( stderr, "Too many coils requested.\n" );
    #endif
  }

  return read_bits(0x02 /* function */,
                   slave, start_addr, count, dest, dest_size, ttyfd,
                   send_retries, error_code, response_timeout, data_access_mutex);
}



/* FUNCTION 0x02   - Read Discrete Inputs
 * Bits are stored on an u32 array, 32 bits per u32.
 * Unused bits in last u32 are set to 0.
 */
int read_input_bits_u32(u8  slave,
                        u16 start_addr,
                        u16 count,
                        u32 *dest,
                        int ttyfd,
                        int send_retries,
                        u8  *error_code,
                        const struct timespec *response_timeout) {
  if( count > MAX_READ_BITS ) {
    count = MAX_READ_BITS;
    #ifdef DEBUG
    fprintf( stderr, "Too many coils requested.\n" );
    #endif
  }

  return read_bits_u32(0x02 /* function */,
                       slave, start_addr, count, dest, ttyfd,
                       send_retries, error_code, response_timeout);
}






/* Execute a transaction for functions that READ REGISTERS.
 * Called by:  read_input_words()
 *             read_output_words()
 */
static int read_registers(u8  function,
                          u8  slave,
                          u16 start_addr,
                          u16 count,
                          u16 *dest,
                          int dest_size,
                          int ttyfd,
                          int send_retries,
                          u8  *error_code,
                          const struct timespec *response_timeout,
                          pthread_mutex_t *data_access_mutex) {
  u8 *data;
  u8 packet[QUERY_BUFFER_SIZE];
  int response_length;
  int query_length;
  int temp,i;
  
  query_length = build_packet(slave, function, start_addr, count, packet);
  if (query_length < 0)    return INTERNAL_ERROR;
  
  response_length = mb_transaction(packet, query_length, &data, ttyfd, 
                                   send_retries, error_code, response_timeout);
  
  if (response_length  < 0)              return response_length;  
  if (response_length != 3 + 2*count)    return INVALID_FRAME;  
  if (data[2]         !=     2*count)    return INVALID_FRAME;
  
  if (NULL != data_access_mutex) pthread_mutex_lock(data_access_mutex);
  for(i = 0; (i < (data[2]*2)) && (i < dest_size); i++ ) {
    temp = data[3 + i *2] << 8;    /* copy reg hi byte to temp hi byte*/
    temp = temp | data[4 + i * 2]; /* copy reg lo byte to temp lo byte*/
    dest[i] = temp;
  }
  if (NULL != data_access_mutex) pthread_mutex_unlock(data_access_mutex);
  
  return response_length;
}



/* Execute a transaction for functions that READ REGISTERS.
 * return the array with the data to the calling function
 * Called by:  read_input_words_u16_ref()
 *             read_output_words_u16_ref()
 */

static int read_registers_u16_ref(u8  function,
                                  u8  slave,
                                  u16 start_addr,
                                  u16 count,
                                  u16 **dest,
                                  int ttyfd,
                                  int send_retries,
                                  u8  *error_code,
                                  const struct timespec *response_timeout) {
  u8 *data;
  u8 packet[QUERY_BUFFER_SIZE];
  int response_length;
  int query_length;
  int i, byte_count;
  
  query_length = build_packet(slave, function, start_addr, count, packet);
  if (query_length < 0)    return INTERNAL_ERROR;
  
  response_length = mb_transaction(packet, query_length, &data, ttyfd,
                                   send_retries, error_code, response_timeout);
  
  if (response_length < 0)               return response_length;  
  if (response_length != 3 + 2*count)    return INVALID_FRAME;  
  if (data[2]         !=     2*count)    return INVALID_FRAME;
  
  byte_count = data[2];
  data = data + 3; /* & data[3] */
  
  if (ntohs(0x0102) != 0x0102) {
   /* little endian host... => we need to swap the bytes! */
    for(i = 0; i < byte_count; i++ ) {
      /* the following 3 lines result in the two values being exchanged! */ 
      data[i  ] = data[i] ^ data[i+1];
      data[i+1] = data[i] ^ data[i+1];
      data[i  ] = data[i] ^ data[i+1];
    }
  }
  *dest = (u16 *)data;  
  return byte_count;
}




/* Execute a transaction for functions that READ REGISTERS.
 * u16 registers are stored in array of u32, two registers per u32.
 * Unused bits of last u32 element are set to 0.
 * Called by:  read_input_words_u32()
 *             read_output_words_u32()
 */
static int read_registers_u32(u8  function,
                              u8  slave,
                              u16 start_addr,
                              u16 count,
                              u32 *dest,
                              int ttyfd,
                              int send_retries,
                              u8  *error_code,
                              const struct timespec *response_timeout) {
  u8 *data;
  u8 packet[QUERY_BUFFER_SIZE];
  int response_length;
  int query_length;
  int i, byte_count, dest_pos;
  
  query_length = build_packet(slave, function, start_addr, count, packet);
  if (query_length < 0)    return INTERNAL_ERROR;
  
  response_length = mb_transaction(packet, query_length, &data, ttyfd,
                                   send_retries, error_code, response_timeout);
  
  if (response_length  < 0)              return response_length;
  if (response_length != 3 + 2*count)    return INVALID_FRAME;
  if (data[2]         !=     2*count)    return INVALID_FRAME;
  
  byte_count = data[2];
  data += 3;
  
  if (ntohs(0x0102) == 0x0102) {
   /* big endian host... */
    /* handle groups of 4 bytes... */
    for(i = 0, dest_pos = 0; i + 3 < byte_count; i += 4, dest_pos++) {
      *(((u8 *)(dest + dest_pos))+ 0) = *(data+i+3);
      *(((u8 *)(dest + dest_pos))+ 1) = *(data+i+4);
      *(((u8 *)(dest + dest_pos))+ 2) = *(data+i+0);
      *(((u8 *)(dest + dest_pos))+ 3) = *(data+i+1);
    }
    /* handle any remaining bytes...
     * since byte_count is supposed to be multiple of 2,
     * (and has already been verified above 'if (data[2] != 2*count)')
     * this will be either 2, or none at all!
     */
    if (i + 1 < byte_count)
      *(((u8 *)(dest + dest_pos))+ 0) = 0;
      *(((u8 *)(dest + dest_pos))+ 1) = 0;
      *(((u8 *)(dest + dest_pos))+ 2) = *(data+i+0);
      *(((u8 *)(dest + dest_pos))+ 3) = *(data+i+1);
  } else {
   /* little endian host... */
    /* handle groups of 4 bytes... */
    for(i = 0, dest_pos = 0; i + 3 < byte_count; i += 4, dest_pos++) {
      *(((u8 *)(dest + dest_pos))+ 0) = *(data+i+1);
      *(((u8 *)(dest + dest_pos))+ 1) = *(data+i+0);
      *(((u8 *)(dest + dest_pos))+ 2) = *(data+i+3);
      *(((u8 *)(dest + dest_pos))+ 3) = *(data+i+2);
    }
    /* handle any remaining bytes...
     * since byte_count is supposed to be multiple of 2,
     * (and has already been verified above 'if (data[2] != 2*count)')
     * this will be either 2, or none at all!
     */
    if (i + 1 < byte_count)
      *(((u8 *)(dest + dest_pos))+ 0) = *(data+i+1);
      *(((u8 *)(dest + dest_pos))+ 1) = *(data+i+0);
      *(((u8 *)(dest + dest_pos))+ 2) = 0;
      *(((u8 *)(dest + dest_pos))+ 3) = 0;
  }
  
  return response_length;
}







/* FUNCTION 0x03   - Read Holding Registers */
int read_output_words(u8  slave,
                      u16 start_addr,
                      u16 count,
                      u16 *dest,
                      int dest_size,
                      int ttyfd,
                      int send_retries,
                      u8  *error_code,
                      const struct timespec *response_timeout,
                      pthread_mutex_t *data_access_mutex) {
  if( count > MAX_READ_REGS ) {
    count = MAX_READ_REGS;
    #ifdef DEBUG
    fprintf( stderr, "Too many registers requested.\n" );
    #endif
  }

  return read_registers(0x03 /* function */,
                        slave, start_addr, count, dest, dest_size, ttyfd,
                        send_retries, error_code, response_timeout, data_access_mutex);
}




/* FUNCTION 0x03   - Read Holding Registers
 * u16 registers are stored in array of u32, two registers per u32.
 * Unused bits of last u32 element are set to 0.
 */
int read_output_words_u32(u8  slave,
                          u16 start_addr,
                          u16 count,
                          u32 *dest,
                          int ttyfd,
                          int send_retries,
                          u8  *error_code,
                          const struct timespec *response_timeout) {
  if( count > MAX_READ_REGS ) {
    count = MAX_READ_REGS;
    #ifdef DEBUG
    fprintf( stderr, "Too many registers requested.\n" );
    #endif
  }
  
  return read_registers_u32(0x03 /* function */,
                            slave, start_addr, count, dest, ttyfd,
                            send_retries, error_code, response_timeout);
}




/* FUNCTION 0x03   - Read Holding Registers
 * return the array with the data to the calling function
 */
int read_output_words_u16_ref(u8  slave,
                              u16 start_addr,
                              u16 count,
                              u16 **dest,
                              int ttyfd,
                              int send_retries,
                              u8  *error_code,
                              const struct timespec *response_timeout) {
  if( count > MAX_READ_REGS ) {
    count = MAX_READ_REGS;
    #ifdef DEBUG
    fprintf( stderr, "Too many registers requested.\n" );
    #endif
  }
  
  return read_registers_u16_ref(0x03 /* function */,
                                slave, start_addr, count, dest, ttyfd, send_retries,
                                error_code, response_timeout);
}




/* FUNCTION 0x04   - Read Input Registers */
int read_input_words(u8  slave,
                     u16 start_addr,
                     u16 count,
                     u16 *dest,
                     int dest_size,
                     int ttyfd,
                     int send_retries,
                     u8  *error_code,
                     const struct timespec *response_timeout,
                     pthread_mutex_t *data_access_mutex) {
  if( count > MAX_READ_REGS ) {
    count = MAX_READ_REGS;
    #ifdef DEBUG
    fprintf( stderr, "Too many input registers requested.\n" );
    #endif
  }
  
  return read_registers(0x04 /* function */,
                        slave, start_addr, count, dest, dest_size, ttyfd, send_retries,
                        error_code, response_timeout, data_access_mutex);
}


/* FUNCTION 0x04   - Read Input Registers
 * u16 registers are stored in array of u32, two registers per u32.
 * Unused bits of last u32 element are set to 0.
 */
int read_input_words_u32(u8  slave,
                         u16 start_addr,
                         u16 count,
                         u32 *dest,
                         int ttyfd,
                         int send_retries,
                         u8  *error_code,
                         const struct timespec *response_timeout) {
  if( count > MAX_READ_REGS ) {
    count = MAX_READ_REGS;
    #ifdef DEBUG
    fprintf( stderr, "Too many input registers requested.\n" );
    #endif
  }
  
  return read_registers_u32(0x04 /* function */,
                            slave, start_addr, count, dest, ttyfd, send_retries,
                            error_code, response_timeout);
}




/* FUNCTION 0x04   - Read Input Registers
 * return the array with the data to the calling function
 */
int read_input_words_u16_ref(u8  slave,
                             u16 start_addr,
                             u16 count,
                             u16 **dest,
                             int ttyfd,
                             int send_retries,
                             u8  *error_code,
                             const struct timespec *response_timeout) {
  if( count > MAX_READ_REGS ) {
    count = MAX_READ_REGS;
    #ifdef DEBUG
    fprintf( stderr, "Too many input registers requested.\n" );
    #endif
  }
  
  return read_registers_u16_ref(0x04 /* function */,
                                slave, start_addr, count, dest, ttyfd, send_retries,
                                error_code, response_timeout);
}



/* Execute a transaction for functions that WRITE a sinlge BIT.
 * Called by:  write_output_bit()
 *             write_output_word()
 */
static int set_single(u8  function,
                      u8  slave,
                      u16 addr,
                      u16 value,
                      int ttyfd,
                      int send_retries,
                      u8  *error_code,
                      const struct timespec *response_timeout,
                      pthread_mutex_t *data_access_mutex) {
  u8 packet[QUERY_BUFFER_SIZE];
  u8 *data;
  int query_length, response_length;
  
  if (NULL != data_access_mutex) pthread_mutex_lock(data_access_mutex);
  query_length = build_packet(slave, function, addr, value, packet);
  if (NULL != data_access_mutex) pthread_mutex_unlock(data_access_mutex);
  if (query_length < 0)    return INTERNAL_ERROR;
  
  response_length = mb_transaction(packet, query_length, &data, ttyfd, send_retries,
                                   error_code, response_timeout);
  
  if (response_length  < 0)  return response_length;  
  if (response_length != 6)  return INVALID_FRAME;
  
  if ((data[2] != packet[2]) || (data[3] != packet[3]) ||
      (data[4] != packet[4]) || (data[5] != packet[5]))
    return INVALID_FRAME;
  
  return response_length;
}






/* FUNCTION 0x05   - Force Single Coil */
int write_output_bit(u8  slave,
                     u16 coil_addr,
                     u16 state,
                     int fd,
                     int send_retries,
                     u8  *error_code,
                     const struct timespec *response_timeout,
                     pthread_mutex_t *data_access_mutex) {
  if (state) state = 0xFF00;
  
  return set_single(0x05 /* function */,
                    slave, coil_addr, state, fd, send_retries,
                    error_code, response_timeout, data_access_mutex);
}





/* FUNCTION 0x06   - Write Single Register */
int write_output_word(u8  slave,
                      u16 reg_addr,
                      u16 value,
                      int fd,
                      int send_retries,
                      u8  *error_code,
                      const struct timespec *response_timeout,
                      pthread_mutex_t *data_access_mutex) {
  return set_single(0x06 /* function */, 
                    slave, reg_addr, value, fd, send_retries,
                    error_code, response_timeout, data_access_mutex);
}




/* FUNCTION 0x0F   - Force Multiple Coils */
int write_output_bits(u8  slave,
                      u16 start_addr,
                      u16 coil_count,
                      u16 *data,
                      int ttyfd,
                      int send_retries,
                      u8  *error_code,
                      const struct timespec *response_timeout,
                      pthread_mutex_t *data_access_mutex) {
  int byte_count, i;
  u8  bit;
  int coil_check = 0;
  int data_array_pos = 0;
  int query_length, response_length;
  u8 packet[QUERY_BUFFER_SIZE];
  u8  *rdata;
  
  if( coil_count > MAX_WRITE_COILS ) {
    coil_count = MAX_WRITE_COILS;
    #ifdef DEBUG
    fprintf( stderr, "Writing to too many coils.\n" );
    #endif
  }
  
  query_length = build_packet(slave, 0x0F /* function */,
                              start_addr, coil_count, packet);
  if (query_length < 0)    return INTERNAL_ERROR;
  
  /* NOTE: Integer division. (count+7)/8 is equivalent to ceil(count/8) */
  byte_count = (coil_count+7)/8;
  packet[query_length] = byte_count;
  
  if (NULL != data_access_mutex) pthread_mutex_lock(data_access_mutex);
  bit = 0x01;
  for(i = 0; i < byte_count; i++) {
    packet[++query_length] = 0;
    while((bit & 0xFF) && (coil_check++ < coil_count)) {
      if(data[data_array_pos++]) {packet[query_length] |=  bit;}
      else                       {packet[query_length] &= ~bit;}
      bit <<= 1;
    }
    bit = 0x01;
  }
  if (NULL != data_access_mutex) pthread_mutex_unlock(data_access_mutex);
  
  response_length = mb_transaction(packet, ++query_length, &rdata, ttyfd, send_retries,
                                   error_code, response_timeout);
  
  if (response_length  < 0)      return response_length;
  if (response_length != 6)      return INVALID_FRAME;
  if ((rdata[2] != packet[2]) || 
      (rdata[3] != packet[3]) ||
      (rdata[4] != packet[4]) ||
      (rdata[5] != packet[5]))   return INVALID_FRAME;
  
  return response_length;
}



/* FUNCTION 0x0F   - Force Multiple Coils
 * Bits should be stored on an u32 array, 32 bits per u32.
 * Unused bits in last u32 should be set to 0.
 */
int write_output_bits_u32(u8  slave,
                          u16 start_addr,
                          u16 coil_count,
                          u32 *data,
                          int ttyfd,
                          int send_retries,
                          u8  *error_code,
                          const struct timespec *response_timeout) {
  int org_pos, byte_count, i;
  int query_length, response_length;
  u8 packet[QUERY_BUFFER_SIZE];
  u8  *rdata;
  
  if( coil_count > MAX_WRITE_COILS ) {
    coil_count = MAX_WRITE_COILS;
    #ifdef DEBUG
    fprintf( stderr, "Writing to too many coils.\n" );
    #endif
  }
  
  query_length = build_packet(slave, 0x0F /* function */,
                              start_addr, coil_count, packet);
  if (query_length < 0)    return INTERNAL_ERROR;
  
  /* NOTE: Integer division. This is equivalent of determining the ceil(count/8) */
  byte_count = (coil_count+7)/8;
  packet[query_length] = byte_count;
  
  /* handle groups of 4 bytes... */
  for(i = 0, org_pos = 0; i + 3 < byte_count; i += 4, org_pos++) {
    packet[++query_length] = data[org_pos] & 0xFF; data[org_pos] >>= 8;
    packet[++query_length] = data[org_pos] & 0xFF; data[org_pos] >>= 8;
    packet[++query_length] = data[org_pos] & 0xFF; data[org_pos] >>= 8;
    packet[++query_length] = data[org_pos] & 0xFF;
  }
  /* handle any remaining bytes... */
  for(; i < byte_count; i++) {
    packet[++query_length] = data[org_pos] & 0xFF; 
    data[org_pos] >>= 8;
  }
  
  response_length = mb_transaction(packet, ++query_length, &rdata, ttyfd, send_retries,
                                   error_code, response_timeout);
  
  if (response_length  < 0)       return response_length;
  if (response_length != 6)       return INVALID_FRAME;
  if ((rdata[2] != packet[2]) ||
      (rdata[3] != packet[3]) ||
      (rdata[4] != packet[4]) ||
      (rdata[5] != packet[5]))    return INVALID_FRAME;
  
  return response_length;
}





/* FUNCTION 0x10   - Force Multiple Registers */
int write_output_words(u8  slave,
                       u16 start_addr,
                       u16 reg_count,
                       u16 *data,
                       int ttyfd,
                       int send_retries,
                       u8  *error_code,
                       const struct timespec *response_timeout,
                       pthread_mutex_t *data_access_mutex) {
  u8  byte_count;
  int i, query_length, response_length;
  u8 packet[QUERY_BUFFER_SIZE];
  u8  *rdata;
  
  if( reg_count > MAX_WRITE_REGS ) {
    reg_count = MAX_WRITE_REGS;
    #ifdef DEBUG
    fprintf( stderr, "Trying to write to too many registers.\n" );
    #endif
  }
  
  query_length = build_packet(slave, 0x10 /* function */,
                              start_addr, reg_count, packet);
  if (query_length < 0)    return INTERNAL_ERROR;
  
  byte_count = reg_count*2;
  packet[query_length] = byte_count;
  
  if (NULL != data_access_mutex) pthread_mutex_lock(data_access_mutex);
  for( i = 0; i < reg_count; i++ ) {
    packet[++query_length] = data[i] >> 8;
    packet[++query_length] = data[i] & 0x00FF;
  }
  if (NULL != data_access_mutex) pthread_mutex_unlock(data_access_mutex);
  
  response_length = mb_transaction(packet, ++query_length, &rdata, ttyfd, send_retries,
                                   error_code, response_timeout);
  
  if (response_length  < 0)       return response_length;  
  if (response_length != 6)       return INVALID_FRAME;  
  if ((rdata[2] != packet[2]) ||
      (rdata[3] != packet[3]) ||      
      (rdata[4] != packet[4]) ||
      (rdata[5] != packet[5]))    return INVALID_FRAME;
  
  return response_length;
}




/* FUNCTION 0x10   - Force Multiple Registers
 * u16 registers are stored in array of u32, two registers per u32.
 * Unused bits of last u32 element are set to 0.
 */
int write_output_words_u32(u8  slave,
                           u16 start_addr,
                             /* number of 16 bit registers packed in the u32 array! */
                           u16 reg_count,
                           u32 *data,
                           int ttyfd,
                           int send_retries,
                           u8  *error_code,
                           const struct timespec *response_timeout) {
  u8  byte_count;
  int i, query_length, response_length;
  u8 packet_[QUERY_BUFFER_SIZE];
  u8 *packet = packet_; /* remove the const'ness of packet_ */
  u8  *rdata;
  
  if( reg_count > MAX_WRITE_REGS ) {
    reg_count = MAX_WRITE_REGS;
    #ifdef DEBUG
    fprintf( stderr, "Trying to write to too many registers.\n" );
    #endif
  }
  
  /* Make sure that the de-referencing and up-casting going on later on in 
   * this function, i.e. code like the following line:
   * *((u16 *)packet) = XXX
   * will result in u16 words starting off on even addresses.
   * If we don't do this, some compilers (e.g. AVR32 cross-compiler) will 
   * generate code which, when executed, will result in 'bus error'.
   *
   * The following packet++ means that the first byte of the packet array is
   * essentially never used. Notice too that the size of thepacket array
   * already takes into account this un-used byte.
   */
  packet++;
  
  query_length = build_packet(slave, 0x10 /* function */,
                              start_addr, reg_count, packet);
  if (query_length < 0)  return INTERNAL_ERROR;
  
  byte_count = reg_count*2;
  packet[query_length] = byte_count;
  
  /* handle groups of 4 bytes... */
  for(i = 0; 4*i + 3 < byte_count; i++) {
    *((u16 *)(packet+(++query_length))) = mb_hton(data[i]);        ++query_length;
    *((u16 *)(packet+(++query_length))) = mb_hton(data[i] >> 16);  ++query_length;
  }
  
  /* handle any remaining bytes...
   * since byte_count is supposed to be multiple of 2,
   * (and has already been verified above 'if (data[2] != 2*count)')
   * this will be either 2, or none at all!
   */
  if (4*i + 1 < byte_count) {
    *((u16 *)(packet+(++query_length))) = mb_hton(data[i]);        ++query_length;
  }
  
  response_length = mb_transaction(packet, ++query_length, &rdata, ttyfd, send_retries,
                                   error_code, response_timeout);
    
  if (response_length  < 0)       return response_length;  
  if (response_length != 6)       return INVALID_FRAME;  
  if ((rdata[2] != packet[2]) ||
      (rdata[3] != packet[3]) ||
      (rdata[4] != packet[4]) ||
      (rdata[5] != packet[5]))    return INVALID_FRAME;
  
  return response_length;
}






/************************************************/
/************************************************/
/**                                            **/
/**   Modbus Library Management Functions.     **/
/**                                            **/
/************************************************/
/************************************************/



/* Initialise the Modbus Master Layer */
int mb_master_init__(int extra_bytes) {
  #ifdef DEBUG
  fprintf(stderr, "mb_master_init__(extra_bytes=%d), QUERY_BUFFER_SIZE=%d\n", extra_bytes, QUERY_BUFFER_SIZE);
  #endif
  buff_extra_bytes_ = extra_bytes;
  return 0;
}


/* Shut down the Modbus Master Layer */
int mb_master_done__(void) {
        return 0;
}


#if 0
int mb_master_init(int nd_count) {
  int extra_bytes;
  
  #ifdef DEBUG
  fprintf( stderr, "mb_master_init()\n");
  fprintf( stderr, "creating %d nodes\n", nd_count);
  #endif
  
  /* initialise layer 1 library */
  if (modbus_init(nd_count, DEF_OPTIMIZATION, &extra_bytes) < 0)
    goto error_exit_0;
  
  /* initialise this library */
  if (mb_master_init__(extra_bytes) < 0)
    goto error_exit_1;
  
  return 0;
  
error_exit_1:
  modbus_done();
error_exit_0:
  return -1;
}


int mb_master_done(void) {
  mb_master_done__();
  return modbus_done();
}
#endif


/* Establish a connection to a remote server/slave */
/* NOTE: We use the lower 2 bits of the returned node id to identify which 
 *       layer1 implementation to use. 
 *           0 -> TCP 
 *           1 -> RTU 
 *           2 -> ASCII 
 *           4 -> unused 
 *       The node id used by the layer1 is shifted left 2 bits
 *       before returning the node id to the caller!
 */
int mb_master_connect(node_addr_t node_addr) {
  int res = -1;
  
  #ifdef DEBUG
  fprintf( stderr, "mb_master_tcp connect()\n");
  #endif
  
  /* call layer 1 library */
  switch(node_addr.naf) {
    case naf_tcp:  
      res = modbus_tcp_connect(node_addr);
      if (res >= 0) res = res*4 + 0 /* offset into fptr_ with TCP functions */;
      return res;
    case naf_rtu:  
      res = modbus_rtu_connect(node_addr);
      if (res >= 0) res = res*4 + 1 /* offset into fptr_ with RTU functions */;
      return res;
    case naf_ascii:  
      res = modbus_ascii_connect(node_addr);
      if (res >= 0) res = res*4 + 2 /* offset into fptr_ with ASCII functions */;
      return res;
  }
  
  return -1;
}





/* Shut down a connection to a remote server/slave */
int mb_master_close(int fd) {
  #ifdef DEBUG
  fprintf( stderr, "mb_master_close(): nd = %d\n", fd);
  #endif
  get_ttyfd(); /* declare the ttyfd variable, ... */
  /* call layer 1 library */
  return modbus_close(ttyfd);
}






/* Tell the library that communications will be suspended for some time. */
/* RTU and ASCII versions ignore this function
 * TCP version closes all the open tcp connections (connections are automatically
 *   re-established the next time an IO function to the slave is requested).
 *   To be more precise, the TCP version makes an estimate of how long
 *   the silence will be based on previous invocations to this exact same
 *   function, and will only close the connections if this silence is
 *   expected to be longer than 1 second!
 *   (The closing of connections is specified in Modbus specification)
 */
int mb_master_tcp_silence_init(void) {
  #ifdef DEBUG
  fprintf( stderr, "mb_master_silence_init():\n");
  #endif
  /* call layer 1 library */
  return modbus_tcp_silence_init();
}


