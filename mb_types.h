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



#ifndef __MB_TYPES_H
#define __MB_TYPES_H

#ifndef __PLC_TYPES_H
 /* if we have already included the MatPLC's type definitions, we don't need to delare the types ourselves... */


/* Tell the stdint.h file we want the limits of the data types defined. */
/* If being compiled by a C++ compiler, this is required!               */
/* If being compiled by a C   compiler, this is ignored!                */
#define __STDC_LIMIT_MACROS
#include <stdint.h>



/* We use the _leastX_t versions of the data types as these are guaranteed
 * to be the exact size we want.
 * The int8_t, etc..., may have been defined to be the same as the 
 * _fastX_t version, which may take up more space than what is really wanted
 * in order as to speed up memory access.
 */
typedef uint_least64_t      u64; /* 64-bit unsigned integer */
typedef  int_least64_t      i64; /* 64-bit signed integer   */

typedef uint_least32_t      u32; /* 32-bit unsigned integer */
typedef  int_least32_t      i32; /* 32-bit signed integer   */

typedef uint_least16_t      u16; /* 16-bit unsigned integer */
typedef  int_least16_t      i16; /* 16-bit signed integer   */

typedef uint_least8_t       u8;  /*  8-bit unsigned integer */
typedef  int_least8_t       i8;  /*  8-bit signed integer   */




#define u64_MAX UINT_LEAST64_MAX
#define u64_MIN UINT_LEAST64_MIN
#define i64_MAX  INT_LEAST64_MAX
#define i64_MIN  INT_LEAST64_MIN

#define u32_MAX UINT_LEAST32_MAX
#define u32_MIN UINT_LEAST32_MIN
#define i32_MAX  INT_LEAST32_MAX
#define i32_MIN  INT_LEAST32_MIN

#define u16_MAX UINT_LEAST16_MAX
#define u16_MIN UINT_LEAST16_MIN
#define i16_MAX  INT_LEAST16_MAX
#define i16_MIN  INT_LEAST16_MIN

#define u8_MAX UINT_LEAST8_MAX
#define u8_MIN UINT_LEAST8_MIN
#define i8_MAX  INT_LEAST8_MAX
#define i8_MIN  INT_LEAST8_MIN


#endif /* __PLC_TYPES_H */

#endif /* __MB_TYPES_H */

