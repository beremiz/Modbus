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



#ifndef MODBUS_TCP_PRIVATE_H
#define MODBUS_TCP_PRIVATE_H


#include "mb_util.h"


/* tcp port default configuration... */
#define DEF_SERVICE  "502"        /* port used by modbus */
#define DEF_PROTOCOL "tcp"        /* protocol used by modbus tcp */
#define DEF_TYPE     SOCK_STREAM  /* Quality of service required of the socket... */
#define DEF_MAX_PENDING_CONNECTION_REQUESTS 5
                                  /* maximum number of pending connection requests
                                   * that have not yet been accept()'ed
                                   */
#define DEF_CLOSE_ON_SILENCE 1    /* Used only by master nodes.
                                   * Flag indicating whether, by default, the connection
                                   * to the slave device should be closed whenever the
                                   * modbus_tcp_silence_init() function is called.
                                   *
                                   * 0  -> do not close connection
                                   * >0 -> close connection
                                   *
                                   * The spec sugests that connections that will not
                                   * be used for longer than 1 second should be closed.
                                   * Even though we expect most connections to have
                                   * silence intervals much shorted than 1 second, we
                                   * decide to use the default of shuting down the
                                   * connections because it is safer, and most other
                                   * implementations seem to do the same.
                                   * If we do not close we risk using up all the possible
                                   * connections that the slave can simultaneouly handle,
                                   * effectively locking out every other master that
                                   * wishes to communicate with that same slave.
                                   */

 /* Since the receive buffer is also re-used to store the frame header,
  * we set it to the larger of the two.
  */
#if     TCP_HEADER_LENGTH > MAX_L2_FRAME_LENGTH
#define RECV_BUFFER_SIZE    TCP_HEADER_LENGTH
#else
#define RECV_BUFFER_SIZE    MAX_L2_FRAME_LENGTH
#endif




#endif  /* MODBUS_TCP_PRIVATE_H */








