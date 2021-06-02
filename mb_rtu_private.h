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



#ifndef MODBUS_RTU_PRIVATE_H
#define MODBUS_RTU_PRIVATE_H

#include "mb_types.h" /* get the data types */
#include "mb_util.h"


/* serial port default configuration... */
#define DEF_DATA_BITS 8
#define DEF_STOP_BITS_PAR 1 /* default stop bits if parity is used     */
#define DEF_STOP_BITS_NOP 2 /* default stop bits if parity is not used */
#define DEF_BAUD_RATE 9600


/* Send retries of rtu frames... */
#define RTU_FRAME_SEND_RETRY 1
  /* NOTES:
   *  - the above are the retries at the layer1 level,
   *    higher layers may decide to retry for themselves!
   */


/* Buffer sizes... */
 /* We use double the maximum frame length for the read buffer,
  * due to the algorithm used to work around aborted frames.
  */
#define RECV_BUFFER_SIZE_SMALL  (MAX_RTU_FRAME_LENGTH + 10)
#define RECV_BUFFER_SIZE_LARGE  (2 * (MAX_RTU_FRAME_LENGTH))


/* Frame lengths... */

 /* The number of bytes in each frame format, excluding CRC.
  *
  * BYTE_COUNT_3  denotes that third byte of frame contains the number of bytes;
  *                 - total number of bytes in frame is
  *                   BYTE_COUNT_3_HEADER + byte_count
  * BYTE_COUNT_34 denotes that third+fourth bytes of frame contain the number of bytes;
  *                 - total number of bytes in frame is
  *                   BYTE_COUNT_34_HEADER + byte_count
  * BYTE_COUNT_7  denotes that seventh byte of frame contain the number of bytes;
  *                 - total number of bytes in frame is
  *                   BYTE_COUNT_7_HEADER + byte_count
  * BYTE_COUNT_11 denotes that eleventh byte of frame contain the number of bytes;
  *                 - total number of bytes in frame is
  *                   BYTE_COUNT_11_HEADER + byte_count
  * BYTE_COUNT_U  denotes unknown number of bytes;
  */

#define BYTE_COUNT_3_HEADER   3
#define BYTE_COUNT_34_HEADER  4
#define BYTE_COUNT_7_HEADER   7
#define BYTE_COUNT_11_HEADER  11

#define BYTE_COUNT_3  (-3)
#define BYTE_COUNT_34 (-34)
#define BYTE_COUNT_7  (-7)
#define BYTE_COUNT_11 (-11)
#define BYTE_COUNT_U  (-128)


#define MAX_FUNCTION_CODE 0x18

#define MIN_FRAME_LENGTH       3
#define EXCEPTION_FRAME_LENGTH 3

static i8 query_frame_lengths[MAX_FUNCTION_CODE+1] = {
                /* 0x00 */ 0,             /* unused                    */
                /* 0x01 */ 6,             /* Read Coil Status          */
                /* 0x02 */ 6,             /* Read Input Status         */
                /* 0x03 */ 6,             /* Read Holding Registers    */
                /* 0x04 */ 6,             /* Read Input Registers      */
                /* 0x05 */ 6,             /* Force Single Coil         */
                /* 0x06 */ 6,             /* Preset Single Register    */
                /* 0x07 */ 2,             /* Read Exception Status     */
                /* 0x08 */ 4,             /* Diagnostics               */
                /* 0x09 */ BYTE_COUNT_U,  /* Program 484               */
                /* 0x0A */ BYTE_COUNT_U,  /* Poll 484                  */
                /* 0x0B */ 2,             /* Fetch Comm. Event Counter */
                /* 0x0C */ 2,             /* Fetch Comm. Event Log     */
                /* 0x0D */ BYTE_COUNT_U,  /* Program Controller        */
                /* 0x0E */ BYTE_COUNT_U,  /* Poll Controller           */
                /* 0x0F */ BYTE_COUNT_7,  /* Force Multiple Coils      */
                /* 0x10 */ BYTE_COUNT_7,  /* Preset Multiple Registers */
                /* 0x11 */ 2,             /* Report Slave ID           */
                /* 0x12 */ BYTE_COUNT_U,  /* Program 884/M84           */
                /* 0x13 */ BYTE_COUNT_U,  /* Reset. Comm. Link         */
                /* 0x14 */ BYTE_COUNT_3,  /* Read General Reference    */
                /* 0x15 */ BYTE_COUNT_3,  /* Write General Reference   */
                /* 0x16 */ 8,             /* Mask Write 4X Register    */
                /* 0x17 */ BYTE_COUNT_11, /* Read/Write 4x Register    */
                /* 0x18 */ 4              /* Read FIFO Queue           */
              };

static i8 response_frame_lengths[MAX_FUNCTION_CODE+1] = {
                /* 0x00 */ 0,             /* unused                    */
                /* 0x01 */ BYTE_COUNT_3,  /* Read Coil Status          */
                /* 0x02 */ BYTE_COUNT_3,  /* Read Input Status         */
                /* 0x03 */ BYTE_COUNT_3,  /* Read Holding Registers    */
                /* 0x04 */ BYTE_COUNT_3,  /* Read Input Registers      */
                /* 0x05 */ 6,             /* Force Single Coil         */
                /* 0x06 */ 6,             /* Preset Single Register    */
                /* 0x07 */ 3,             /* Read Exception Status     */
                /* 0x08 */ 6,/*see (1)*/  /* Diagnostics               */
                /* 0x09 */ BYTE_COUNT_U,  /* Program 484               */
                /* 0x0A */ BYTE_COUNT_U,  /* Poll 484                  */
                /* 0x0B */ 6,             /* Fetch Comm. Event Counter */
                /* 0x0C */ BYTE_COUNT_3,  /* Fetch Comm. Event Log     */
                /* 0x0D */ BYTE_COUNT_U,  /* Program Controller        */
                /* 0x0E */ BYTE_COUNT_U,  /* Poll Controller           */
                /* 0x0F */ 6,             /* Force Multiple Coils      */
                /* 0x10 */ 6,             /* Preset Multiple Registers */
                /* 0x11 */ BYTE_COUNT_3,  /* Report Slave ID           */
                /* 0x12 */ BYTE_COUNT_U,  /* Program 884/M84           */
                /* 0x13 */ BYTE_COUNT_U,  /* Reset. Comm. Link         */
                /* 0x14 */ BYTE_COUNT_3,  /* Read General Reference    */
                /* 0x15 */ BYTE_COUNT_3,  /* Write General Reference   */
                /* 0x16 */ 8,             /* Mask Write 4X Register    */
                /* 0x17 */ BYTE_COUNT_3,  /* Read/Write 4x Register    */
                /* 0x18 */ BYTE_COUNT_34  /* Read FIFO Queue           */
              };

/* NOTE (1):
 *    The diagnostic function (0x08) has sub-functions. In particular,
 *    sub-function 21 (0x15) has two sub-sub-functions. In the very
 *    particular case of *one* of these sub-sub-functions, the reply
 *    frame does *not* have a size of 4, but is variable in length
 *    and includes a byte counter.
 *    To take this into account in the table would require an extra two
 *    tables.
 *    The above length has been hardcoded into the frame_length() function
 *    (in file modbus_rtu.c)
 */


#define FALSE 0
#define TRUE 1


#endif  /* MODBUS_RTU_PRIVATE_H */








