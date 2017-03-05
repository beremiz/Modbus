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




 /* write a modbus frame */
 /* WARNING: when calling this function, the *frame_data buffer
  *          must be allocated with an extra *extra_bytes
  *          beyond those required for the frame_length.
  *          This is because the extra bytes will be used
  *          to store the crc before sending the frame.
  *
  *          The *extra_bytes value will be returned by the
  *          modbus_init() function call.
  */
 /* NOTE: calling this function will flush the input stream,
  *       which means any frames that may have arrived
  *       but have not yet been read using modbus_read()
  *       will be permanently lost...
  */
int modbus_write(int    nd,
                 u8    *frame_data,
                 size_t frame_length,
                 u16    transaction_id,
                 const struct timespec *transmit_timeout
                );

 /* read a modbus frame */
/*
 * The frame is read from:
 *   -  the node descriptor nd, if nd >= 0
 *   -  any valid and initialised node descriptor, if nd = -1
 *      In this case, the node where the data is eventually read from
 *      is returned in *nd.
 *      NOTE: (only avaliable if using TCP)
 */
 /* NOTE: calling modbus_write() will flush the input stream,
  *       which means any frames that may have arrived
  *       but have not yet been read using modbus_read()
  *       will be permanently lost...
  *
  * NOTE: Ususal select semantics for (a: recv_timeout == NULL) and
  *       (b: *recv_timeout == 0) also apply.
  *       (a) Indefinite timeout
  *       (b) Try once, and and quit if no data available.
  */
 /* NOTE: send_data and send_length is used to pass to the modbus_read() function
  *        the frame that was previously sent over the same connection (node).
  *        This data is then allows the modbus_read() function to ignore any 
  *        data that is read but identical to the previously sent data. This
  *        is used when using serial ports that echoes back all the data that is 
  *        sent out over the same serial port. When using some RS232 to RS485 
  *        converters, this functionality is essential as not all these converters
  *        are capable of not echoing back the sent data.
  *       These parameters are ignored when using TCP! 
  */
 
 /* RETURNS: number of bytes read
  *          -1 on read from file/node error
  *          -2 on timeout
  */
int modbus_read(int *nd,                /* node descriptor */
                u8 **recv_data_ptr,
                u16 *transaction_id,
                const u8 *send_data,
                int send_length,      
                const struct timespec *recv_timeout);


 /* init the library */
int modbus_init(int nd_count,        /* maximum number of nodes... */
                optimization_t opt,
                int *extra_bytes);

 /* shutdown the library...*/
int modbus_done(void);


/* Open a node for master / slave operation.
 * Returns the node descriptor, or -1 on error.
 */
int modbus_connect(node_addr_t node_addr);
int modbus_listen(node_addr_t node_addr);

/* Close a node, needs a node descriptor as argument... */
int modbus_close(int nd);

/* Tell the library that the user will probably not be communicating
 * for some time...
 * This will allow the library to release any resources it will not
 * be needing during the silence.
 * NOTE: This is onlyused by the TCP version to close down tcp connections
 *       when the silence will going to be longer than  second.
 */
int modbus_silence_init(void);

 /* determine the minmum acceptable timeout... */
 /* NOTE: timeout values passed to modbus_read() lower than the value returned
  *       by this function may result in frames being aborted midway, since they
  *       take at least modbus_get_min_timeout() seconds to transmit.
  */
double modbus_get_min_timeout(int baud,
                              int parity,
                              int data_bits,
                              int stop_bits);










