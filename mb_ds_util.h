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



 /* Data structures used by the modbus protocols... */


#ifndef __MODBUS_DS_UTIL_H
#define __MODBUS_DS_UTIL_H


#include "mb_types.h"  /* get the data types */

/**************************************/
/**                                  **/
/** A data structure - linear buffer **/
/**                                  **/
/**************************************/

/* An unbounded FIFO data structure.
 *
 * The user/caller writes and reads directly from the data structure's buffer,
 * which eliminates slow copying of bytes between the user's and the structure's
 * local memory.
 *
 * The data structure stores the current data linearly in a single memory array,
 * i.e. the current data is stored from start to finish from a low address
 * to a high address, and does *not* circle back to the bottom of the address
 * space as is usual in a circular buffer. This allows the user/caller to
 * pass the structure's own byte array on to other functions such as
 * read() and write() for file operations.
 *
 * The FIFO is implemented by allocating more memory than the maximum number
 * of bytes it will ever hold, and using the extra bytes at the top of the
 * array as the bottom data bytes are released. When we run out of extra bytes,
 * (actually, when the number of un-used bytes at the beginning is larger than
 * a configured maximum), the whole data is moved down, freeing once again the
 * extra bytes at the top of the array.
 *
 * Remember that we can optimize the data structure so that whenever it becomes
 * empty, we can reset it to start off at the bottom of the byte array, i.e. we
 * can set the start = end = 0; instead of simply setting the start = end, which
 * may point to any position in the array.
 *
 * Taking the above into consideration, it would probably be a little more efficient
 * to implement it as a circular buffer with an additional linearize() function
 * the user could call whenever (s)he required the data to be stored linearly.
 * Nevertheless, since it has already been implemented as a linear buffer, and since
 * under normal circumstances the start and end pointers will be reset to 0 quite
 * often (and therefore we get no additional benefit under normal circumstances),
 * we will leave it as it is for the time being...
 *
 *
 * The user is expected to call
 *   lb_init() -> to initialize the structure
 *   lb_done() -> to free the data structure's memory
 *
 * The user can store data starting off from...
 *   lb_free() -> pointer to address of free memory
 *   lb_free_count() -> number of free bytes available
 * and then call
 *   lb_data_add()
 * to add the data to the data structure
 *
 * Likewise, the user can read the data directly from
 *   lb_data() -> pointer to address of data location
 *   lb_free_count() -> number of data bytes available
 * and free the data using
 *   lb_data_purge()
 * to remove the data from the data structure
 */


typedef struct {
        u8 *data;
        int data_size;      /* size of the *data buffer                   */
        int data_start;     /* offset within *data were valid data starts */
        int data_end;       /* offset within *data were valid data ends   */
        int max_data_start; /* optimization parameter! When should it be normalised? */
        } lb_buf_t;

/* NOTE: lb = Linear Buffer */

static inline u8 *lb_init(lb_buf_t *buf, int size, int max_data_start) {
  if (size <= 0)
    return NULL;

  if (max_data_start >= size)
    max_data_start = size - 1;

  buf->data_size  = size;
  buf->data_start = 0;
  buf->data_end   = 0;
  buf->max_data_start = max_data_start;
  buf->data = (u8 *)malloc(size);
  return buf->data;
}

static inline void lb_done(lb_buf_t *buf) {
  free(buf->data);
  buf->data = NULL;
}

static inline u8 *lb_normalize(lb_buf_t *buf) {
  return (u8 *)memmove(buf->data,
                       buf->data + buf->data_start,
                       buf->data_end - buf->data_start);
}

static inline u8 *lb_data(lb_buf_t *buf) {
  return buf->data + buf->data_start;
}

static inline int lb_data_count(lb_buf_t *buf) {
  return buf->data_end - buf->data_start;
}

static inline void lb_data_add(lb_buf_t *buf, int count) {
  if ((buf->data_end += count) >= buf->data_size)
    buf->data_end = buf->data_size - 1;
}

static inline u8 *lb_data_purge(lb_buf_t *buf, int count) {
  buf->data_start += count;
  if (buf->data_start > buf->data_end)
    buf->data_start = buf->data_end;

  if ((buf->data_end == buf->data_size) || (buf->data_start >= buf->max_data_start))
    return lb_normalize(buf);

  return buf->data + buf->data_start;
}

static inline void lb_data_purge_all(lb_buf_t *buf) {
  buf->data_start = buf->data_end = 0;
}

static inline u8 *lb_free(lb_buf_t *buf) {
  return buf->data + buf->data_end;
}

static inline int lb_free_count(lb_buf_t *buf) {
  return buf->data_size - buf->data_end;
}




#endif  /* __MODBUS_DS_UTIL_H */
