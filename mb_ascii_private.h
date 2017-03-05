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



#ifndef MODBUS_ASCII_PRIVATE_H
#define MODBUS_ASCII_PRIVATE_H



#include "mb_util.h"


/* serial port default configuration... */
#define DEF_DATA_BITS 7
#define DEF_STOP_BITS_PAR 1 /* default stop bits if parity is used     */
#define DEF_STOP_BITS_NOP 2 /* default stop bits if parity is not used */
#define DEF_BAUD_RATE 9600


/* Send retries of ascii frames... */
#define ASC_FRAME_SEND_RETRY 0
  /* NOTES:
   *  - the above are the retries at the layer1 level,
   *    higher layers may decide to retry for themselves!
   *  - For ascii frames, it doesn't make much sense to retry
   *    if the first try failed...
   */


/* Buffer sizes... */
#define SEND_BUFFER_SIZE_SMALL  (MAX_ASC_FRAME_LENGTH / 2)
#define SEND_BUFFER_SIZE_LARGE  (MAX_ASC_FRAME_LENGTH)
#define RECV_BUFFER_SIZE_SMALL  (MAX_ASC_FRAME_LENGTH / 2)
#define RECV_BUFFER_SIZE_LARGE  (MAX_ASC_FRAME_LENGTH)


/* Frame lengths... */

/* The smallest element in an ascii frame.
 * This is used later to define the smallest value of certain buffers...
 */
#define ASC_FRAME_MIN_ELE_LENGTH   ASC_FRAME_HEADER_LENGTH

#if     ASC_FRAME_MIN_ELE_LENGTH < ASC_FRAME_TAIL_LENGTH
#undef  ASC_FRAME_MIN_ELE_LENGTH
#define ASC_FRAME_MIN_ELE_LENGTH   ASC_FRAME_TAIL_LENGTH
#endif

#if     ASC_FRAME_MIN_ELE_LENGTH < ASC_FRAME_LRC_LENGTH
#undef  ASC_FRAME_MIN_ELE_LENGTH
#define ASC_FRAME_MIN_ELE_LENGTH   ASC_FRAME_LRC_LENGTH
#endif

#if     ASC_FRAME_MIN_ELE_LENGTH < L2_TO_ASC_CODING
#undef  ASC_FRAME_MIN_ELE_LENGTH
#define ASC_FRAME_MIN_ELE_LENGTH   L2_TO_ASC_CODING
#endif


#endif  /* MODBUS_ASCII_PRIVATE_H */








