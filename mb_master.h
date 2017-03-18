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


/* mb_master.h */


#ifndef MODBUS_MASTER_H
#define MODBUS_MASTER_H

#include <time.h> /* struct timespec data structure */

#include "mb_types.h" /* get the data types */
#include "mb_addr.h"  /* get definition of common variable types and error codes */



/***********************************************************************

	 Note: All functions used for sending or receiving data via
	       modbus return these return values.


	Returns:	string_length if OK
			-1 on internal error or port failure
			-2 on timeout
			-3 if a valid yet un-expected frame is received!
			-4 for modbus exception errors
			   (in this case exception code is returned in *error_code)

***********************************************************************/


/* FUNCTION 0x01   - Read Coils
 * Bits are stored on an int array, one bit per int.
 */
int read_output_bits(u8  slave,
                     u16 start_addr,
                     u16 count,
                     u16 *dest,
                     int dest_size,
                     int fd,
                     int send_retries,
                     u8  *error_code,
                     const struct timespec *response_timeout,
                     pthread_mutex_t *data_access_mutex);
#define read_coils(p1,p2,p3,p4,p5,p6,p7,p8,p9,p10) \
        read_output_bits(p1,p2,p3,p4,p5,p6,p7,p8,p9,p10)

 
/* FUNCTION 0x01   - Read Coils
 * Bits are stored on an u32 array, 32 bits per u32.
 * Unused bits in last u32 are set to 0.
 */
int read_output_bits_u32(u8  slave,
                         u16 start_addr,
                         u16 count,
                         u32 *dest,
                         int fd,
                         int send_retries,
                         u8  *error_code,
                         const struct timespec *response_timeout);
#define read_coils_u32(p1,p2,p3,p4,p5,p6,p7,p8) \
        read_output_bits_u32(p1,p2,p3,p4,p5,p6,p7,p8)




/* FUNCTION 0x02   - Read Discrete Inputs
 * Bits are stored on an int array, one bit per int.
 */
int read_input_bits(u8  slave,
                    u16 start_addr,
                    u16 count,
                    u16 *dest,
                    int dest_size,
                    int fd,
                    int send_retries,
                    u8  *error_code,
                    const struct timespec *response_timeout,
                    pthread_mutex_t *data_access_mutex);
#define read_discrete_inputs(p1,p2,p3,p4,p5,p6,p7,p8,p9,p10) \
        read_input_bits     (p1,p2,p3,p4,p5,p6,p7,p8,p9,p10)


/* FUNCTION 0x02   - Read Discrete Inputs
 * Bits are stored on an u32 array, 32 bits per u32.
 * Unused bits in last u32 are set to 0.
 */
int read_input_bits_u32(u8  slave,
                        u16 start_addr,
                        u16 count,
                        u32 *dest,
                        int fd,
                        int send_retries,
                        u8  *error_code,
                        const struct timespec *response_timeout);
#define read_discrete_inputs_u32(p1,p2,p3,p4,p5,p6,p7,p8) \
        read_input_bits_u32     (p1,p2,p3,p4,p5,p6,p7,p8)


        

/* FUNCTION 0x03   - Read Holding Registers */
int read_output_words(u8  slave,
                      u16 start_addr,
                      u16 count,
                      u16 *dest,
                      int dest_size,
                      int fd,
                      int send_retries,
                      u8  *error_code,
                      const struct timespec *response_timeout,
                      pthread_mutex_t *data_access_mutex);
#define read_holding_registers(p1,p2,p3,p4,p5,p6,p7,p8,p9,p10) \
        read_output_words     (p1,p2,p3,p4,p5,p6,p7,p8,p9,p10)


/* FUNCTION 0x03   - Read Holding Registers
 * u16 registers are stored in array of u32, two registers per u32.
 * Unused bits of last u32 element are set to 0.
 */
int read_output_words_u32(u8  slave,
                          u16 start_addr,
                          u16 count,
                          u32 *dest,
                          int fd,
                          int send_retries,
                          u8  *error_code,
                          const struct timespec *response_timeout);
#define read_holding_registers_u32(p1,p2,p3,p4,p5,p6,p7,p8) \
        read_output_words_u32     (p1,p2,p3,p4,p5,p6,p7,p8)


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
                              const struct timespec *response_timeout);
#define read_holding_registers_u16_ref(p1,p2,p3,p4,p5,p6,p7,p8) \
        read_output_words_u16_ref     (p1,p2,p3,p4,p5,p6,p7,p8)



/* FUNCTION 0x04   - Read Input Registers */
int read_input_words(u8  slave,
                     u16 start_addr,
                     u16 count,
                     u16 *dest,
                     int dest_size,
                     int fd,
                     int send_retries,
                     u8  *error_code,
                     const struct timespec *response_timeout,
                     pthread_mutex_t *data_access_mutex);
#define read_input_registers(p1,p2,p3,p4,p5,p6,p7,p8,p9,p10) \
        read_input_words    (p1,p2,p3,p4,p5,p6,p7,p8,p9,p10)



/* FUNCTION 0x04   - Read Input Registers
 * u16 registers are stored in array of u32, two registers per u32.
 * Unused bits of last u32 element are set to 0.
 */
int read_input_words_u32(u8  slave,
                         u16 start_addr,
                         u16 count,
                         u32 *dest,
                         int fd,
                         int send_retries,
                         u8  *error_code,
                         const struct timespec *response_timeout);
#define read_input_registers_u32(p1,p2,p3,p4,p5,p6,p7,p8) \
        read_input_words_u32    (p1,p2,p3,p4,p5,p6,p7,p8)



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
                             const struct timespec *response_timeout);
#define read_input_registers_u16_ref(p1,p2,p3,p4,p5,p6,p7,p8) \
        read_input_words_u16_ref    (p1,p2,p3,p4,p5,p6,p7,p8)




/* FUNCTION 0x05   - Force Single Coil */
int write_output_bit(u8  slave,
                     u16 coil_addr,
                     u16 state,
                     int fd,
                     int send_retries,
                     u8  *error_code,
                     const struct timespec *response_timeout,
                     pthread_mutex_t *data_access_mutex);
#define force_single_coil(p1,p2,p3,p4,p5,p6,p7,p8) \
        write_output_bit (p1,p2,p3,p4,p5,p6,p7,p8)






/* FUNCTION 0x06   - Write Single Register */
int write_output_word(u8  slave,
                      u16 reg_addr,
                      u16 value,
                      int fd,
                      int send_retries,
                      u8  *error_code,
                      const struct timespec *response_timeout,
                      pthread_mutex_t *data_access_mutex);
#define write_single_register(p1,p2,p3,p4,p5,p6,p7,p8) \
        write_output_word    (p1,p2,p3,p4,p5,p6,p7,p8)





/* FUNCTION 0x0F   - Force Multiple Coils */
int write_output_bits(u8  slave,
                      u16 start_addr,
                      u16 coil_count,
                      u16 *data,
                      int fd,
                      int send_retries,
                      u8  *error_code,
                      const struct timespec *response_timeout,
                      pthread_mutex_t *data_access_mutex);
#define force_multiple_coils(p1,p2,p3,p4,p5,p6,p7,p8,p9) \
        write_output_bits   (p1,p2,p3,p4,p5,p6,p7,p8,p9)


/* FUNCTION 0x0F   - Force Multiple Coils
 * Bits should be stored on an u32 array, 32 bits per u32.
 * Unused bits in last u32 should be set to 0.
 */
int write_output_bits_u32(u8  slave,
                          u16 start_addr,
                          u16 coil_count,
                          u32 *data,
                          int fd,
                          int send_retries,
                          u8  *error_code,
                          const struct timespec *response_timeout);
#define force_multiple_coils_u32(p1,p2,p3,p4,p5,p6,p7,p8) \
        write_output_bits_u32   (p1,p2,p3,p4,p5,p6,p7,p8)



/* FUNCTION 0x10   - Force Multiple Registers */
int write_output_words(u8  slave,
                       u16 start_addr,
                       u16 reg_count,
                       u16 *data,
                       int fd,
                       int send_retries,
                       u8  *error_code,
                       const struct timespec *response_timeout,
                       pthread_mutex_t *data_access_mutex);
#define force_multiple_registers(p1,p2,p3,p4,p5,p6,p7,p8,p9) \
        write_output_words      (p1,p2,p3,p4,p5,p6,p7,p8,p9)



/* FUNCTION 0x10   - Force Multiple Registers
 * u16 registers are stored in array of u32, two registers per u32.
 * Unused bits of last u32 element are set to 0.
 */
int write_output_words_u32(u8  slave,
                           u16 start_addr,
                           u16 reg_count,
                           u32 *data,
                           int fd,
                           int send_retries,
                           u8  *error_code,
                           const struct timespec *response_timeout);

#define force_multiple_registers_u32(p1,p2,p3,p4,p5,p6,p7,p8) \
        write_output_words_u32      (p1,p2,p3,p4,p5,p6,p7,p8)






/* Initialise the Modbus Library to work as Master only */
int mb_master_init(int nd_count);
/* Shut down the Modbus Library */
int mb_master_done(void);



/* Establish a connection to a remote server/slave.
 * The address type (naf_tcp, naf_rtu, naf_ascii) specifies the lower
 * layer to use for the newly opened node.
 */
int mb_master_connect(node_addr_t node_addr);
/* Shut down a connection to a remote server/slave */
int mb_master_close(int nd);




/* Tell the library that communications will be suspended for some time. */
/* RTU and ASCII versions ignore this function
 * TCP version closes all the open tcp connections (connections are automatically
 *   re-established the next time an IO function to the slave is requested).
 *   To be more precise, the TCP version makes an estimate of how long
 *   the silence will be based on previous invocations to this exact same
 *   function, and will only close the connections if this silence is
 *   expected to be longer than 1 second!
 */
int mb_master_tcp_silence_init(void);



#endif  /* MODBUS_MASTER_H */








