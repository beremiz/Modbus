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



#include <fcntl.h>      /* File control definitions */
#include <stdio.h>      /* Standard input/output */
#include <string.h>
#include <stdlib.h>
#include <termio.h>     /* POSIX terminal control definitions */
#include <sys/time.h>   /* Time structures for select() */
#include <unistd.h>     /* POSIX Symbolic Constants */
#include <assert.h>
#include <errno.h>      /* Error definitions */
#include <ctype.h>
#include <time.h>       /* clock_gettime()   */
#include <limits.h>     /* required for INT_MAX */

#include "mb_layer1.h"
#include "mb_ascii_private.h"


/* #define DEBUG */         /* uncomment to see the data sent and received */




/************************************/
/**                                **/
/** Include common code...         **/
/**                                **/
/************************************/

#include "mb_ds_util.h"    /* data structures... */
#include "mb_time_util.h"  /* time conversion routines... */


/**************************************************************/
/**************************************************************/
/****                                                      ****/
/****                                                      ****/
/****                Purpose and Formats                   ****/
/****                                                      ****/
/****                                                      ****/
/**************************************************************/
/**************************************************************/
/*

   This file implements the ascii formating of the modbus protocol.
   Many values, protocol related, are hardcoded into the code, as it
   seems very unlikely this code will ever get re-used for anything
   else but this specific protocol.

   Modbus ASCII frames have no timing restrictions whatsoever, and
   abide by the following format:

   Header
   ------
     size : 1 byte
     value: ':' (i.e. '\0x3A')

   Body
   ----
     size : variable, multiple of 2
     value: binary data converted to ascii format,
            i.e. each binary data byte is converted into two ascii characters
            representing the byte in hexadecimal. Allowable characters are
            '0' to '9' and 'A' to 'D'

   LRC
   ---
     size : 2 bytes
     value: Longitudinal Redundancy Check of data, excluding any headers, tails,
            etc...

   Tail
   ----
     size : 2 bytes
     value: 'CR' + 'LF'  (i.e. '\0x0D' + '\0x0A')


*/



/**************************************************************/
/**************************************************************/
/****                                                      ****/
/****                                                      ****/
/****                Forward Declarations                  ****/
/****                    and Defaults                      ****/
/****                                                      ****/
/**************************************************************/
/**************************************************************/


typedef enum {fp_header, fp_body, fp_lrc, fp_tail, fp_done} frame_part_t;




/**************************************************************/
/**************************************************************/
/****                                                      ****/
/****                                                      ****/
/****              Local Utility functions...              ****/
/****                                                      ****/
/****                                                      ****/
/**************************************************************/
/**************************************************************/


/*****************************************/
/**                                     **/
/**             lrc functions           **/
/**                                     **/
/*****************************************/


static inline void lrc_init(u8 *lrc) {
  *lrc = 0;
}

static inline void lrc_add_single(u8 *lrc, u8 data) {
  *lrc += data;
}

static inline void lrc_add_many(u8 *lrc, u8 *data, int count) {
  for (; count > 0; count--, *lrc += *data++);
}

static inline void lrc_end(u8 *lrc) {
  *lrc = 1 + ~(*lrc);
}



/**************************************/
/**                                  **/
/**    Initialise a struct termios   **/
/**                                  **/
/**************************************/
static int termios_init(struct termios *tios,
                        int baud,
                        int parity,
                        int data_bits,
                        int stop_bits) {
  speed_t baud_rate;

  if (tios == NULL)
    return -1;

  /* reset all the values... */
  /* NOTE: the following are initialised later on...
  tios->c_iflag = 0;
  tios->c_oflag = 0;
  tios->c_cflag = 0;
  tios->c_lflag = 0;
  */
  tios->c_line  = 0;

 /* The minimum number of characters that should be received
  * to satisfy a call to read().
  */
  tios->c_cc[VMIN ] = 0;

 /* The maximum inter-arrival interval between two characters,
  * in deciseconds.
  *
  * NOTE: we could use this to detect the end of RTU frames,
  *       but we prefer to use select() that has higher resolution,
  *       even though this higher resolution is most probably not
  *       supported, and the effective resolution is 10ms,
  *       one tenth of a decisecond.
  */
  tios->c_cc[VTIME] = 0;

  /* configure the input modes... */
  tios->c_iflag =  IGNBRK |  /* ignore BREAK condition on input         */
                   IGNPAR |  /* ignore framing errors and parity errors */
                   IXANY;    /* enable any character to restart output  */
  /*               BRKINT       Only active if IGNBRK is not set.
   *                            generate SIGINT on BREAK condition,
   *                            otherwise read BREAK as character \0.
   *               PARMRK       Only active if IGNPAR is not set.
   *                            replace bytes with parity errors with
   *                            \377 \0, instead of \0.
   *               INPCK        enable input parity checking
   *               ISTRIP       strip off eighth bit
   *               IGNCR        ignore carriage return on input
   *               INLCR        only active if IGNCR is not set.
   *                            translate newline to carriage return  on input
   *               ICRNL        only active if IGNCR is not set.
   *                            translate carriage return to newline on input
   *               IUCLC        map uppercase characters to lowercase on input
   *               IXON         enable XON/XOFF flow control on output
   *               IXOFF        enable XON/XOFF flow control on input
   *               IMAXBEL      ring bell when input queue is full
   */

  /* configure the output modes... */
  tios->c_oflag = OPOST;     /* enable implementation-defined output processing */
  /*              ONOCR         don't output CR at column 0
   *              OLCUC         map lowercase characters to uppercase on output
   *              ONLCR         map NL to CR-NL on output
   *              OCRNL         map CR to NL on output
   *              OFILL         send fill characters for a delay, rather than
   *                            using a timed delay
   *              OFDEL         fill character is ASCII DEL. If unset, fill
   *                            character is ASCII NUL
   *              ONLRET        don't output CR
   *              NLDLY         NL delay mask. Values are NL0 and NL1.
   *              CRDLY         CR delay mask. Values are CR0, CR1, CR2, or CR3.
   *              TABDLY        horizontal tab delay mask. Values are TAB0, TAB1,
   *                            TAB2, TAB3, or XTABS. A value of XTABS expands
   *                            tabs to spaces (with tab stops every eight columns).
   *              BSDLY         backspace delay mask. Values are BS0 or BS1.
   *              VTDLY         vertical tab delay mask. Values are VT0 or VT1.
   *              FFDLY         form feed delay mask. Values are FF0 or FF1.
   */

  /* configure the control modes... */
  tios->c_cflag = CREAD |    /* enable receiver.               */
                  CLOCAL;    /* ignore modem control lines     */
  /*              HUPCL         lower modem control lines after last process
   *                            closes the device (hang up).
   *              CRTSCTS       flow control (Request/Clear To Send).
   */
       if (data_bits == 5) tios->c_cflag |= CS5;
  else if (data_bits == 6) tios->c_cflag |= CS6;
  else if (data_bits == 7) tios->c_cflag |= CS7;
  else if (data_bits == 8) tios->c_cflag |= CS8;
  else return -1;

       if (stop_bits == 1) tios->c_cflag &=~ CSTOPB;
  else if (stop_bits == 2) tios->c_cflag |= CSTOPB;
  else return -1;

         if(parity == 0) { /* none */
    tios->c_cflag &=~ PARENB;
    tios->c_cflag &=~ PARODD;
  } else if(parity == 2)  { /* even */
    tios->c_cflag |= PARENB;
    tios->c_cflag &=~ PARODD;
  } else if(parity == 1)  { /* odd */
    tios->c_cflag |= PARENB;
    tios->c_cflag |= PARODD;
  } else return -1;


  /* configure the local modes... */
  tios->c_lflag = IEXTEN;    /* enable implementation-defined input processing   */
  /*              ISIG          when any of the characters INTR, QUIT, SUSP, or DSUSP
   *                            are received, generate the corresponding signal.
   *              ICANON        enable canonical mode. This enables the special
   *                            characters EOF, EOL, EOL2, ERASE, KILL, REPRINT,
   *                            STATUS, and WERASE, and buffers by lines.
   *              ECHO          echo input characters.
   */

  /* Set the baud rate */
  /* Must be done before reseting all the values to 0! */
  switch(baud) {
    case 110:    baud_rate = B110;    break;
    case 300:    baud_rate = B300;    break;
    case 600:    baud_rate = B600;    break;
    case 1200:   baud_rate = B1200;   break;
    case 2400:   baud_rate = B2400;   break;
    case 4800:   baud_rate = B4800;   break;
    case 9600:   baud_rate = B9600;   break;
    case 19200:  baud_rate = B19200;  break;
    case 38400:  baud_rate = B38400;  break;
    case 57600:  baud_rate = B57600;  break;
    case 115200: baud_rate = B115200; break;
    default: return -1;
  } /* switch() */

  if ((cfsetispeed(tios, baud_rate) < 0) ||
      (cfsetospeed(tios, baud_rate) < 0))
    return -1;;

  return 0;
}





/*****************************************/
/**                                     **/
/**     u8/ascii conversion functions   **/
/**                                     **/
/*****************************************/

/* Convert binary data to ascii format.
 * Both xxx_data_lengths specify the available bytes in
 * in the respective arrays.
 */
/* NOTE: this function is only called from a single location
 *       so we might just as well make it inline...
 */
static inline int bin_2_asc(u8 *bin_data, int bin_data_length,
                            u8 *asc_data, int asc_data_length) {
  u8  nibble;
  int count = 0;
  u8  *last_bin_data = bin_data + bin_data_length;
    /* we need L2_TO_ASC_CODING ascii bytes for each bin byte, therefore the
     * '- (L2_TO_ASC_CODING - 1)'
     */
  u8  *last_asc_data = asc_data + asc_data_length - (L2_TO_ASC_CODING - 1);

  while ((bin_data < last_bin_data) &&
         (asc_data < last_asc_data)) {

    nibble = (*bin_data & 0xF0) >> 4;
    *(asc_data++) = (nibble <= 9)?nibble + '0':nibble - 10 + 'A';

    nibble = (*bin_data & 0x0F);
    *(asc_data++) = (nibble <= 9)?nibble + '0':nibble - 10 + 'A';

    count++;
    bin_data++;
  }

  /* return number of bytes converted... */
  return count;
}


/* Convert from lowercase to upper case. */
/* It probably does not make sense calling the generic toupper()
 * whose functionality depends on the current locale.
 * Our own specific function is most probably much faster...
 */
static inline u8 local_toupper(u8 val) {
  if ((val >= 'a') && (val <= 'z'))
    return val - 'a' + 'A';
  return val;
}

/* Convert ascii data to bin format.
 * *asc_data must be a two byte array.
 *
 * If a non-ascii character is found, returns -1
 */
/* NOTE: this function is only called from a single location
 *       so we might just as well make it inline...
 */
static inline int asc_2_bin(u8 *asc_data, u8 *bin_data) {
  if ((isxdigit(asc_data[0]) == 0) ||
      (isxdigit(asc_data[1]) == 0))
    return -1;

  asc_data[0] = local_toupper(asc_data[0]);
  asc_data[1] = local_toupper(asc_data[1]);

  /* hi */ *(bin_data) =  ((asc_data[0] <= '9')?
                           (asc_data[0] - '0'):(asc_data[0] - 'A' + 10)) * 0x10;
  /* lo */ *(bin_data) += (asc_data[1] <= '9')?
                           (asc_data[1] - '0'):(asc_data[1] - 'A' + 10);

  return 0;
}




/************************************/
/**                                **/
/** A data structure - send buffer **/
/**                                **/
/************************************/

/* data structure used to store the data to be sent in ascii format. */
/* The binary data is converted into ascii format before transmission. The
 * frame is not converted as a single whole, but is rather done in chunks.
 * The size of the chunks depends on the data size of the send_buffer.
 *
 * A lrc variable keeps a tab on the current value of the lrc as the data
 * is being converted.
 *
 * Three special functions add the header, lrc and tail to the ascii frame.
 */

/* NOTE: The algorithms in the insert functions require a minimum buffer
 *       size to work correctly...
 */
#define SEND_BUF_MIN_LENGTH   ASC_FRAME_MIN_ELE_LENGTH

typedef struct {
        lb_buf_t data_buf;

        u8 lrc; /* the current value of the lrc, in binary format */
        } send_buf_t;

/* A small auxiliary function... */
static inline u8 *send_buf_init(send_buf_t *buf, int size, int max_data_start) {
  /* The algorithms in other functions require a minimum size
   * to work correctly...
   */
  if (size < SEND_BUF_MIN_LENGTH)
    return NULL;

  lrc_init(&buf->lrc);
  return lb_init(&buf->data_buf, size, max_data_start);
}

/* A small auxiliary function... */
static inline void send_buf_done(send_buf_t *buf) {
  lb_done(&buf->data_buf);
}

/* A small auxiliary function... */
static inline void send_buf_reset(send_buf_t *buf) {
  lrc_init(&buf->lrc);
  lb_data_purge_all(&buf->data_buf);
}

/* A small auxiliary function... */
static inline int send_buf_data_count(send_buf_t *buf) {
  return lb_data_count(&buf->data_buf);
}

/* A small auxiliary function... */
static inline int send_buf_free_count(send_buf_t *buf) {
  return lb_free_count(&buf->data_buf);
}
/* A small auxiliary function... */
static inline u8 *send_buf_data(send_buf_t *buf) {
  return lb_data(&buf->data_buf);
}

/* A small auxiliary function... */
static inline u8 *send_buf_free(send_buf_t *buf) {
  return lb_free(&buf->data_buf);
}

/* A small auxiliary function... */
static inline int send_buf_data_add(send_buf_t *buf, u8 *data, int data_count) {
  int res = bin_2_asc(data,  data_count, send_buf_free(buf), send_buf_free_count(buf));
  if (res <=0) return res;
  lb_data_add(&buf->data_buf, L2_TO_ASC_CODING * res);
  lrc_add_many(&buf->lrc, data, res);
  return res;
}

/* A small auxiliary function... */
static inline void send_buf_data_purge(send_buf_t *buf, int count) {
  lb_data_purge(&buf->data_buf, count);
}

/* A small auxiliary function... */
static inline void send_buf_data_purge_all(send_buf_t *buf) {
  lb_data_purge_all(&buf->data_buf);
}

/* A small auxiliary function... */
static inline int send_buf_lrc_append(send_buf_t *buf) {
#if ASC_FRAME_LRC_LENGTH != 2
#error Code assumes LRC length of 2 bytes, but ASC_FRAME_LRC_LENGTH != 2
#endif
  if (lb_free_count(&buf->data_buf) < ASC_FRAME_LRC_LENGTH)
    return -1;
  lrc_end(&buf->lrc);
  bin_2_asc(&buf->lrc, sizeof(buf->lrc),
            lb_free(&buf->data_buf), ASC_FRAME_LRC_LENGTH);
  lb_data_add(&buf->data_buf, ASC_FRAME_LRC_LENGTH);
  return 0;
}

static inline int send_buf_header_append(send_buf_t *buf) {
#if ASC_FRAME_HEADER_LENGTH != 1
#error Code assumes HEADER length of 1 bytes, but ASC_FRAME_HEADER_LENGTH != 1
#endif
  if (lb_free_count(&buf->data_buf) < ASC_FRAME_HEADER_LENGTH)
    return -1;

  /* add the ':' frame header */
  *lb_free(&buf->data_buf) = ASC_FRAME_HEADER;
  lb_data_add(&buf->data_buf, ASC_FRAME_HEADER_LENGTH);

  return 0;
}

static inline int send_buf_tail_append(send_buf_t *buf) {
#if ASC_FRAME_TAIL_LENGTH != 2
#error Code assumes TAIL length of 2 bytes, but ASC_FRAME_TAIL_LENGTH != 2
#endif
  if (lb_free_count(&buf->data_buf) < ASC_FRAME_TAIL_LENGTH)
    return -1;

  /* add the CR+LF frame delimiter */
  lb_free(&buf->data_buf)[0] = ASC_FRAME_TAIL_0;
  lb_free(&buf->data_buf)[1] = ASC_FRAME_TAIL_1;
  lb_data_add(&buf->data_buf, ASC_FRAME_TAIL_LENGTH);

  return 0;
}




/************************************/
/**                                **/
/** A data structure - recv buffer **/
/**                                **/
/************************************/

/* data structure used to store the data received from the bus. */

/* The ascii data received from the bus is added to the buffer, and is
 * dynamically converted to binary format. Once a valid frame has been
 * converted, conversion stops until this valid frame is deleted/purged.
 */

/* NOTE: The algorithms in the insert functions require a minimum buffer
 *       size to work correctly...
 */
#define RECV_BUF_MIN_LENGTH   ASC_FRAME_MIN_ELE_LENGTH

#define RECV_BUF_BIN_BUF_SIZE (MAX_L2_FRAME_LENGTH +                     \
                               ASC_FRAME_LRC_LENGTH / L2_TO_ASC_CODING)

typedef struct {
        lb_buf_t asc_data_buf;
        u8       bin_data_buf[RECV_BUF_BIN_BUF_SIZE];
        int      bin_data_free_ofs;

        frame_part_t frame_part;
        u8 lrc; /* the current value of the lrc, in binary format */
        u8 lrc_1; /* the previous value of the lrc...             */
          /* NOTE: We do a running conversion between ascii and binary format,
           *       i.e. we start converting from ascii to binary before we
           *       have received the complete ascii frame. This means that
           *       we also do a running computation of our local version of
           *       the frame lrc.
           *       The lrc, transmitted at the end of the ascii frame,
           *       but before the frame tail, also gets converted to binary
           *       before we get a chance to realize that it is the lrc value,
           *       and should therefore not be taken into account when computing
           *       our local version of the lrc.
           *       So we keep the previous value of the running lrc, and use
           *       that to confirm whether we have a valid frame.
           */
        } recv_buf_t;

/* A small auxiliary function... */
static inline u8 *recv_buf_init(recv_buf_t *buf, int size, int max_data_start) {
  /* The algorithms in other functions require a minimum size
   * to work correctly...
   */
  if (size < RECV_BUF_MIN_LENGTH)
    return NULL;

  lrc_init(&buf->lrc);
  buf->bin_data_free_ofs = 0;
  buf->frame_part = fp_header;
  return lb_init(&buf->asc_data_buf, size, max_data_start);
}

/* A small auxiliary function... */
static inline void recv_buf_done(recv_buf_t *buf) {
  lb_done(&buf->asc_data_buf);
}

/* A small auxiliary function... */
static inline void recv_buf_reset(recv_buf_t *buf) {
  lrc_init(&buf->lrc);
  buf->bin_data_free_ofs = 0;
  buf->frame_part = fp_header;
  lb_data_purge_all(&buf->asc_data_buf);
}

/* A small auxiliary function... */
static inline u8 *recv_buf_data(recv_buf_t *buf) {
  return lb_data(&buf->asc_data_buf);
}

/* A small auxiliary function... */
static inline u8 *recv_buf_free(recv_buf_t *buf) {
  return lb_free(&buf->asc_data_buf);
}

/* The function that really does all the conversion work... */
/* It finds frame limits, converts the data into binary format,
 * and checks for correct lrc in the received frame.
 */
/* NOTE: called indirectly from various locations! Do NOT inline! */
static void recv_buf_data_parse(recv_buf_t *buf) {
  int count;
  u8  *data;

  data  = lb_data(&buf->asc_data_buf);
  count = lb_data_count(&buf->asc_data_buf);

  /* NOTE: We need at least ASC_FRAME_MIN_ELE_LENGTH bytes to
   *       to be able to find that element of minimum length
   */
  while ((count >= ASC_FRAME_MIN_ELE_LENGTH) && (buf->frame_part != fp_done)) {
    /* Check for frame header... */
 /* The following few lines of code assume that ASC_FRAME_HEADER_LENGTH is 1! */
#if ASC_FRAME_HEADER_LENGTH != 1
#error The code is written in such a way that can only handle ASC_FRAME_HEADER_LENGTH == 1
#endif
    if (data[0] == ASC_FRAME_HEADER) {
      /* found the beginning of a frame...
       * Even if we were previously converting a frame without errors,
       * if we receive a new frame header we discard the previous
       * frame that went unfinished!
       */
      data += ASC_FRAME_HEADER_LENGTH;
      count -= ASC_FRAME_HEADER_LENGTH;
      buf->frame_part = fp_body;
      lrc_init(&buf->lrc);
      buf->bin_data_free_ofs = 0;
      continue;
    }

    /* Check for frame tail... */
    /*
     * Note that the while() condition guarantees that we have at least
     * two ascii bytes to handle.
     */
 /* The following few lines of code assume that ASC_FRAME_TAIL_LENGTH is 2! */
#if ASC_FRAME_TAIL_LENGTH != 2
#error The code is written in such a way that can only handle ASC_FRAME_TAIL_LENGTH == 2
#endif
    if ((data[0] == ASC_FRAME_TAIL_0) &&
        (data[1] == ASC_FRAME_TAIL_1)) {
      /* end of binary data...  */
      data  += ASC_FRAME_TAIL_LENGTH;
      count -= ASC_FRAME_TAIL_LENGTH;

      /* let's check the lrc... */
      if (buf->bin_data_free_ofs <= 0)
        /* we have reached the end of a frame that did not include
         * any binary data, not even the lrc value itself!
         */
        goto frame_error;

      /* Remember that we do not use the most recent lrc value, as this
       * incorrectly includes the ascii bytes in the frame that code the
       * frame's lrc. (pls read the note in the recv_but_t typedef)
       */
 /* The following few lines of code assume that
  * (ASC_FRAME_LRC_LENGTH / L2_TO_ASC_CODING) is 1!
  */
#if L2_TO_ASC_CODING != ASC_FRAME_LRC_LENGTH
#error The code is written in such a way that can only handle L2_TO_ASC_CODING == ASC_FRAME_LRC_LENGTH
#endif
      lrc_end(&(buf->lrc_1));
      if (buf->lrc_1 == buf->bin_data_buf[buf->bin_data_free_ofs-1]) {
        /* we have received a correct frame... */
        buf->frame_part = fp_done;
        continue;
      } else {
        /* we have found a frame with an lrc error */
        goto frame_error;
      }
    }

    if (buf->frame_part == fp_header) {
      /* we are searching for beginning of a frame...
       * but we did not find it in the previous condition!
       * We continue looking in the next ascii byte
       */
      data++;
      count--;
      continue;
      }

    if (buf->frame_part == fp_body) {
      /* we have previously found the beginning of a frame,
       * and are now converting the body into binary format...
       *
       * Note that the while() condition guarantees that we have at least
       * two ascii bytes to convert into binary format.
       */

      /* this is normal data... let's convert... */
      if (asc_2_bin(data, buf->bin_data_buf + buf->bin_data_free_ofs) < 0)
        /* error converting from ascii to binary.
         * This must be due to an invalid ascii character in the ascii data.
         * We discard the current frame...
         *
         * Note that we *do not* increment the data pointer,
         * nor do we decrement the count variable. One of the ascii bytes could
         * be the begining of a new valid frame, and we don't want to skip it!
         */
        goto frame_error;

      buf->lrc_1 = buf->lrc;
      lrc_add_single(&(buf->lrc), *(buf->bin_data_buf + buf->bin_data_free_ofs));

      data += L2_TO_ASC_CODING;
      count -= L2_TO_ASC_CODING;
      if (++(buf->bin_data_free_ofs) >= RECV_BUF_BIN_BUF_SIZE)
        /* Whoops, this shouldn't be hapening!
         * The frame we are receiving is larger than the alocated buffer!
         * Our only alternative is to discard the frame
         * we are currently receiving...
         */
        goto frame_error;

      continue;
    }

    /* if none of the above, then it must be some transmission error */
    /* Actually, the code will never fall through since if we are in this loop,
     * (frame_part == header) || (frame_part == body) is always true!
     */
    data++;
    count--;
frame_error:
    lrc_init(&buf->lrc);
    buf->bin_data_free_ofs = 0;
    buf->frame_part = fp_header;
  } /* while () */

  lb_data_purge(&buf->asc_data_buf, lb_data_count(&buf->asc_data_buf) - count);
}

/* A small auxiliary function... */
static inline void recv_buf_search_frame(recv_buf_t *buf) {
  if (buf->frame_part != fp_done)
    recv_buf_data_parse(buf);
}

/* add ascii format data to buffer */
static inline void recv_buf_data_add(recv_buf_t *buf, int count) {
  lb_data_add(&buf->asc_data_buf, count);
}

/* A small auxiliary function... */
static inline int recv_buf_data_count(recv_buf_t *buf) {
  return lb_data_count(&buf->asc_data_buf);
}

/* A small auxiliary function... */
static inline int recv_buf_free_count(recv_buf_t *buf) {
  return lb_free_count(&buf->asc_data_buf);
}

/* Return pointer to frame, if a valid frame is available */
static inline u8 *recv_buf_frame(recv_buf_t *buf) {
  recv_buf_search_frame(buf);
  if (buf->frame_part == fp_done)
    /* we have found a frame...! */
    return buf->bin_data_buf;

  /* no frame... */
  return NULL;
}

/* Return number of bytes in frame, if a valid frame is available */
static inline int recv_buf_frame_count(recv_buf_t *buf) {
  recv_buf_search_frame(buf);
  if (buf->frame_part == fp_done)
    /* we have found a frame...! */
    return buf->bin_data_free_ofs - ASC_FRAME_LRC_LENGTH/L2_TO_ASC_CODING;

  /* no frame... */
  return -1;
}

/* Delete valid binary format frame! */
static inline void recv_buf_frame_purge(recv_buf_t *buf) {
  /* NOTE: we only delete valid frames!!
   *       Partially converted frames are not deleted, the
   *       remaining bytes may be received later!
   */
  if (buf->frame_part == fp_done) {
    buf->frame_part = fp_header;
    buf->bin_data_free_ofs = 0;
  }
}



/************************************/
/**                                **/
/**  A data structure - nd entry   **/
/**                                **/
/************************************/

/* NOTE: nd = node descriptor */

typedef struct {
         /* The file descriptor associated with this node */
         /* NOTE: if the node is not yet in use, i.e. if the node is free,
          *       then fd will be set to -1
          */
        int    fd;
        struct timeval time_15_char_;

         /* Modbus ascii frames are delimited by a ':' (colon) at the begining of
          * a frame, and CR-LF sequence at the end the frame.
          *
          * Unless we want to take 'ages' reading the data off the serial line
          * one byte at a time, we risk reading beyond the boundary of the
          * frame currently being interpreted. The extra data belongs to the
          * subsequent frame, and must therefore be buffered to be handled
          * when the next frame is being interpreted.
          *
          * The receive buffer is therefore a static variable.
          */
        recv_buf_t recv_buf_;

         /* The send ascii buffer could be a local function variable
          * instead of a static variable, but we might just as well
          * allocate the memory at startup and not risk running out
          * of memory while the module is running.
          */
        send_buf_t send_buf_;

          /* The old settings of the serial port, to be reset when the library is closed... */
        struct termios old_tty_settings_;

          /* ignore echo flag.
           * If set to 1, then it means that we will be reading every byte we
           * ourselves write out to the bus, so we must ignore those bytes read
           * before we really read the data sent by remote nodes.
           *
           * This comes in useful when using a RS232-RS485 converter that does
           * not correctly control the RTS-CTS lines...
           */
        int ignore_echo;
 } nd_entry_t;


static inline void nd_entry_init(nd_entry_t *nde) {
  nde->fd = -1; /* The node is free... */
}

static int nd_entry_connect(nd_entry_t *nde,
                            node_addr_t *node_addr,
                            optimization_t opt) {

  int parity_bits, start_bits, char_bits;
  struct termios settings;
  int buf_size;

  /*
  if (nde == NULL)
    goto error_exit_0;
  */
  if (nde->fd >= 0)
    goto error_exit_0;

  /* initialise the termios data structure */
  if (termios_init(&settings,
                   node_addr->addr.ascii.baud,
                   node_addr->addr.ascii.parity,
                   node_addr->addr.ascii.data_bits,
                   node_addr->addr.ascii.stop_bits)
      < 0) {
#ifdef DEBUG
    fprintf(stderr,   "Invalid serial line settings"
                      "(baud=%d, parity=%d, data_bits=%d, stop_bits=%d)",
                      node_addr->addr.ascii.baud,
                      node_addr->addr.ascii.parity,
                      node_addr->addr.ascii.data_bits,
                      node_addr->addr.ascii.stop_bits);
#endif
    goto error_exit_1;
  }

    /* set the ignore_echo flag */
  nde->ignore_echo = node_addr->addr.ascii.ignore_echo;

    /* initialise send buffer */
  buf_size = (opt == optimize_size)?SEND_BUFFER_SIZE_SMALL:
                                    SEND_BUFFER_SIZE_LARGE;
  if (send_buf_init(&nde->send_buf_, buf_size, buf_size - SEND_BUF_MIN_LENGTH)
      == NULL) {
#ifdef DEBUG
    fprintf(stderr, "Out of memory: error initializing send buffer");
#endif
    goto error_exit_1;
  }

    /* initialise recv buffer */
  buf_size = (opt == optimize_size)?RECV_BUFFER_SIZE_SMALL:
                                    RECV_BUFFER_SIZE_LARGE;
  if (recv_buf_init(&nde->recv_buf_, buf_size, buf_size - RECV_BUF_MIN_LENGTH)
      == NULL) {
#ifdef DEBUG
    fprintf(stderr, "Out of memory: error initializing receive buffer");
#endif
    goto error_exit_2;
  }

    /* open the serial port */
  if((nde->fd = open(node_addr->addr.ascii.device, O_RDWR | O_NOCTTY | O_NDELAY))
     < 0) {
#ifdef DEBUG
    fprintf(stderr, "Error opening device %s (errno=%d)",
                     node_addr->addr.ascii.device, errno);
#endif
    goto error_exit_3;
  }

  if(tcgetattr(nde->fd, &nde->old_tty_settings_) < 0) {
#ifdef DEBUG
    fprintf(stderr, "Error reading device's %s original settings.",
                      node_addr->addr.ascii.device);
#endif
    goto error_exit_4;
  }

  if(tcsetattr(nde->fd, TCSANOW, &settings) < 0) {
#ifdef DEBUG
    fprintf(stderr, "Error configuring device %s"
                    "(baud=%d, parity=%d, data_bits=%d, stop_bits=%d)",
                    node_addr->addr.ascii.device,
                    node_addr->addr.ascii.baud,
                    node_addr->addr.ascii.parity,
                    node_addr->addr.ascii.data_bits,
                    node_addr->addr.ascii.stop_bits);
#endif
    goto error_exit_4;
  }

  parity_bits   = (node_addr->addr.ascii.parity == 0)?0:1;
  start_bits    = 1;
  char_bits     = start_bits  + node_addr->addr.ascii.data_bits +
                  parity_bits + node_addr->addr.ascii.stop_bits;
/*  time_35_char_ = d_to_timeval(3.5*char_bits/baud); */
  nde->time_15_char_ = d_to_timeval(1.5*char_bits/node_addr->addr.ascii.baud);

#ifdef DEBUG
  printf("nd_entry_connect(): %s open\n", node_addr->addr.ascii.device );
  printf("nd_entry_connect(): returning fd=%d\n", nde->fd);
#endif
  return nde->fd;

  error_exit_4:
    close(nde->fd);
  error_exit_3:
    recv_buf_done(&nde->recv_buf_);
  error_exit_2:
    send_buf_done(&nde->send_buf_);
  error_exit_1:
    nde->fd = -1; /* set the node as free... */
  error_exit_0:
    return -1;
}



static int nd_entry_free(nd_entry_t *nde) {
  if (nde->fd < 0)
    /* already free */
    return -1;

  /* reset the tty device old settings... */
  if(tcsetattr(nde->fd, TCSANOW, &nde->old_tty_settings_) < 0)
    fprintf(stderr, "Error reconfiguring serial port to it's original settings.");

  recv_buf_done(&nde->recv_buf_);
  send_buf_done(&nde->send_buf_);
  close(nde->fd);
  nde->fd = -1;

  return 0;
}


static inline int nd_entry_is_free(nd_entry_t *nde) {
  return (nde->fd < 0);
}




/************************************/
/**                                **/
/**  A data structure - nd table   **/
/**                                **/
/************************************/

typedef struct {
      /* the array of node descriptors, and current size... */
    nd_entry_t *node;
    int        node_count;      /* total number of nodes in the node[] array */
} nd_table_t;



#if 1
/* nd_table_init()
 *   Version 1 of the nd_table_init() function. 
 *   If called more than once, 2nd and any subsequent calls will
 *   be interpreted as a request to confirm that it was already correctly 
 *   initialized with the requested number of nodes.
 */
static int nd_table_init(nd_table_t *ndt, int nd_count) {
  int count;

  if (ndt->node != NULL) {
    /* this function has already been called, and the node table is already initialised */
    return (ndt->node_count == nd_count)?0:-1;
  }

  /* initialise the node descriptor metadata array... */
  ndt->node = malloc(sizeof(nd_entry_t) * nd_count);
  if (ndt->node == NULL) {
#ifdef DEBUG
    fprintf(stderr, "Out of memory: error initializing node address buffer");
#endif
    return -1;
  }
  ndt->node_count = nd_count;

    /* initialise the state of each node in the array... */
  for (count = 0; count < ndt->node_count; count++) {
    nd_entry_init(&ndt->node[count]);
  } /* for() */

  return nd_count; /* number of succesfully created nodes! */
}
#else
/* nd_table_init()
 *   Version 2 of the nd_table_init() function. 
 *   If called more than once, 2nd and any subsequent calls will
 *   be interpreted as a request to reserve an extra new_nd_count
 *   number of nodes. This will be done using realloc().
 */
static int nd_table_init(nd_table_t *ndt, int new_nd_count) {
  int count;

  /* initialise the node descriptor metadata array... */
  ndt->node = realloc(ndt->node, sizeof(nd_entry_t) * (ndt->node_count + new_nd_count));
  if (ndt->node == NULL) {
#ifdef ERRMSG
    fprintf(stderr, ERRMSG_HEAD "Out of memory: error initializing node address buffer\n");
#endif
    return -1;
  }

  /* initialise the state of each newly added node in the array... */
  for (count = ndt->node_count; count < ndt->node_count + new_nd_count; count++) {
    nd_entry_init(&ndt->node[count]);
  } /* for() */
  ndt->node_count += new_nd_count;

  return new_nd_count; /* number of succesfully created nodes! */
}
#endif


static inline nd_entry_t *nd_table_get_nd(nd_table_t *ndt, int nd) {
  if ((nd < 0) || (nd >= ndt->node_count))
    return NULL;

  return &ndt->node[nd];
}


static inline void nd_table_done(nd_table_t *ndt) {
  int i;

  if (ndt->node == NULL) 
    return;

    /* close all the connections... */
  for (i = 0; i < ndt->node_count; i++)
    nd_entry_free(&ndt->node[i]);

  /* Free memory... */
  free(ndt->node);
  *ndt = (nd_table_t){.node=NULL, .node_count=0};
}



static inline int nd_table_get_free_nd(nd_table_t *ndt) {
  int count;

  for (count = 0; count < ndt->node_count; count++) {
    if (nd_entry_is_free(&ndt->node[count]))
      return count;
  }

   /* none found... */
  return -1;
}


static inline int nd_table_free_nd(nd_table_t *ndt, int nd) {
  if ((nd < 0) || (nd >= ndt->node_count))
    return -1;

  return nd_entry_free(&ndt->node[nd]);
}



/**************************************************************/
/**************************************************************/
/****                                                      ****/
/****                                                      ****/
/****                Global Library State                  ****/
/****                                                      ****/
/****                                                      ****/
/**************************************************************/
/**************************************************************/


 /* The node descriptor table... */
 /* NOTE: This variable must be correctly initialised here!! */
static nd_table_t nd_table_ = {.node=NULL, .node_count=0};

 /* The optimization choice... */
static optimization_t optimization_;



/**************************************************************/
/**************************************************************/
/****                                                      ****/
/****                                                      ****/
/****             Sending of Modbus ASCII Frames           ****/
/****                                                      ****/
/****                                                      ****/
/**************************************************************/
/**************************************************************/

/* 
 *     NOTE: for now the transmit_timeout is silently ignored in ASCII version!
 */
int modbus_ascii_write(int    nd,
                       u8    *data,
                       size_t data_length,
                       u16    transaction_id,
                       const struct timespec *transmit_timeout
                      )
 {
  fd_set rfds;
  struct timeval timeout;
  int res, bin_data_conv, send_retries;
  frame_part_t frame_part;
  nd_entry_t *nd_entry;

  /* check if nd is correct... */
  if ((nd_entry = nd_table_get_nd(&nd_table_, nd)) == NULL)
    return -1;

  /* check if nd is initialzed... */
  if (nd_entry->fd < 0)
    return -1;

  /* THE MAIN LOOP!!! */
  send_retries = ASC_FRAME_SEND_RETRY + 1; /* must try at least once... */
  while (send_retries > 0) {

    /*******************************
     * synchronise with the bus... *
     *******************************/
    /* Remember that a RS485 bus is half-duplex, so we have to wait until
     * nobody is transmitting over the bus for our turn to transmit.
     * This will never happen on a modbus network if the master and
     * slave state machines never get out of synch (granted, it probably
     * only has two states, but a state machine nonetheless), but we want
     * to make sure we can re-synchronise if they ever do get out of synch.
     *
     * The following lines are an attempt at re-synchronising with the bus.
     * Unfortunately, due to the completely asynchronous nature of the
     * modbus-ascii protocol (there are no time boundaries for sending a frame!),
     * it is impossible to guarantee that we will synchronise correctly.
     *
     * Use of RTS/CTS over a half-duplex coms chanel would eliminate this
     * 'feature', but not all RS232/RS485 converters delay activating the
     * CTS signal, even though there may be current activity on the bus.
     *
     * We first wait until the bus is silent for at least 1.5 character interval.
     * Note that we only get feedback from the device driver once a whole byte
     * has been received, so we must wait longer than 1 character interval to make
     * sure there is no current activity on the bus). We then flush the input and
     * output queues.
     */
      /* NOTES:
       *   - we do not need to reset the rfds with FD_SET(ttyfd, &rfds)
       *     before every call to select! We only wait on one file descriptor,
       *     so if select returns succesfully, it must have that same file
       *     decriptor set in the rdfs!
       *     If select returns with a timeout, then we do not get to call
       *     select again!
       *   - We do not reset the timeout value. Normally this value is left
       *     unchanged when select() returns, so we will be witing for longer
       *     than the desired period.
       *     On Linux the timeout is changed to reflect the time remaining.
       *     In this case, things will work more nicely.
       */
    FD_ZERO(&rfds);
    FD_SET(nd_entry->fd, &rfds);
    timeout = nd_entry->time_15_char_;
    while ((res = select(nd_entry->fd+1, &rfds, NULL, NULL, &timeout)) != 0) {
      if (res > 0) {
        /* we are receiving data over the serial port! */
        /* Throw the data away!                        */
        tcflush(nd_entry->fd, TCIFLUSH); /* flush the input stream */
        /* reset the timeout value! */
        timeout = nd_entry->time_15_char_;
      } else {
        /* some kind of error ocurred */
        if (errno != EINTR)
          /* we were not interrupted by a signal */
          return -1;
        /* We will be callind select() again.
         * We need to reset the FD SET !
         */
        FD_ZERO(&rfds);
        FD_SET(nd_entry->fd, &rfds);
      }
    } /* while (select()) */

    /* Flush both input and output streams... */
    /* NOTE: Due to the nature of the modbus protocol,
     *       when a frame is sent all previous
     *       frames that may have arrived at the sending node become
     *       irrelevant.
     */
    tcflush(nd_entry->fd, TCIOFLUSH);    /* flush the input & output streams */
    recv_buf_reset(&nd_entry->recv_buf_);   /* reset the recv buffer            */

    /**********************
     * write to output... *
     **********************/
    send_buf_reset(&nd_entry->send_buf_);

    frame_part = fp_header; /* start off by sending the header... */
    bin_data_conv = 0; /* binary format data already converted to ascii format... */
    while ((frame_part != fp_done) ||
           (send_buf_data_count(&nd_entry->send_buf_) > 0)) {

       /* build the frame we will send over the wire... */
       /* We use a state machine with four states... */
      if (frame_part == fp_header) {
        if (send_buf_header_append(&nd_entry->send_buf_) >= 0)
          frame_part = fp_body;
      }
      if (frame_part == fp_body) {
        res = send_buf_data_add(&nd_entry->send_buf_, data + bin_data_conv, data_length - bin_data_conv);
        bin_data_conv += res;
        if (bin_data_conv == data_length)
          frame_part = fp_lrc;
      }
      if (frame_part == fp_lrc) {
        if (send_buf_lrc_append(&nd_entry->send_buf_) >= 0)
          frame_part = fp_tail;
      }
      if (frame_part == fp_tail) {
        if (send_buf_tail_append(&nd_entry->send_buf_) >= 0)
          frame_part = fp_done;
      }

       /* send the frame... */
      if ((res = write(nd_entry->fd,
                       send_buf_data(&nd_entry->send_buf_),
                       send_buf_data_count(&nd_entry->send_buf_))) < 0) {
        if ((errno != EAGAIN ) && (errno != EINTR )) {
          break;
        } else {
          /* there is no harm if we get interrupted while writing the frame.
           * The ascii version of modbus does not place any timing
           * constraints whatsoever...
           *
           * We simply continue sending the frame...
           */
          res = 0;
        }
      }
#ifdef DEBUG
/* Print each character that has just been sent on the bus */
  { int i;
    printf("bytes sent -> [");
    for(i = 0; i < res; i++)
      printf("%c", send_buf_data(&nd_entry->send_buf_)[i]);
    printf("]\n");
  }
#endif
      send_buf_data_purge(&nd_entry->send_buf_, res);

      if ((frame_part == fp_done) &&
          (send_buf_data_count(&nd_entry->send_buf_) == 0))
        /* sent frame successfully! */
        return data_length;

    } /* while(frame_not_sent) */
    /* NOTE: Some error occured while trying to transmit. It will probably
     *       not go away by itself, but there is no harm in trying again
     *       if the upper layer so wishes...
     */
    send_retries--;
  } /* while()  MAIN LOOP */

   /* maximum retries exceeded */
  return -1;
}



/**************************************************************/
/**************************************************************/
/****                                                      ****/
/****                                                      ****/
/****             Receiving Modbus ASCII Frames            ****/
/****                                                      ****/
/****                                                      ****/
/**************************************************************/
/**************************************************************/


/************************************/
/**                                **/
/**          Read a frame          **/
/**                                **/
/************************************/

/* A function to read a valid frame off the modbus ascii bus.
 *
 * NOTES:
 *        - The returned frame is guaranteed to be a valid frame.
 *        - The returned length does *not* include the LRC.
 */
/* NOTE: This function is only called from one place in the rest of the code,
 * so we might just as well make it inline...
 */
/* RETURNS: number of bytes in received frame
 *          -1 on read file error
 *          -2 on timeout
 */
static inline int read_frame(nd_entry_t *nd_entry,
                             u8 **recv_data_ptr,
                             struct timespec *end_time)
{
  /* temporary variables... */
  fd_set rfds;
  int res;

  /* start by deleting any previously converted frames... */
  recv_buf_frame_purge(&nd_entry->recv_buf_);

  /*==============*
   * read a frame *
   *==============*/
#ifdef DEBUG
  printf("bytes read -> <");
#endif
  /* The main loop that reads one frame               */
  /*  (multiple calls to read() )                     */
  /* and jumps out as soon as it finds a valid frame. */
  /* NOTE: Do not change this into a do {...} until() loop!
   *       The while loop may find valid frames in the data read off the
   *       bus in previous invocations of this same fucntion, and may
   *       therefore not execute any loop iteration whatsoever,
   *       and not call read()
   */
  while ((*recv_data_ptr = recv_buf_frame(&nd_entry->recv_buf_)) == NULL) {
    /* We need more bytes... */

    /*-----------------------------*
     * check for data availability *
     *-----------------------------*/
    /* if we can't find a valid frame in the existing data, or no data
     * was left over, then we need to read more bytes!
     */
      FD_ZERO(&rfds);
      FD_SET(nd_entry->fd, &rfds);
      {int sel_res = my_select(nd_entry->fd + 1, &rfds, NULL, end_time);
        if (sel_res < 0)
          return -1;
        if (sel_res == 0)
          return -2;
      }

    /*------------------*
     * read frame bytes *
     *------------------*/
     /* Read in as many bytes as possible...  */
    res = read(nd_entry->fd,
               recv_buf_free(&nd_entry->recv_buf_),
               recv_buf_free_count(&nd_entry->recv_buf_));
    if (res < 0) {
      if (errno != EINTR)
        return -1;
      else
        res = 0;
    }
#ifdef DEBUG
      {/* display the hex code of each character received */
        int i;
        for (i=0; i < res; i++)
          printf("%c", recv_buf_free(&nd_entry->recv_buf_)[i]);
      }
#endif
    recv_buf_data_add(&nd_entry->recv_buf_, res);
  } /* while ()*/
#ifdef DEBUG
    printf(">\n");
#endif

  /* We have a frame! */
  return recv_buf_frame_count(&nd_entry->recv_buf_);
}





/**************************************/
/**                                  **/
/**    Read a Modbus ASCII frame     **/
/**                                  **/
/**************************************/

/* The public function that reads a valid modbus frame.
 *
 * The returned frame is guaranteed to be different to the
 * the frame stored in send_data, and to start with the
 * same slave address stored in send_data[0].
 *
 * If send_data is NULL, send_data_length = 0, or
 * ignore_echo == 0, then the first valid frame read off
 * the bus is returned.
 *
 * return value: The length (in bytes) of the valid frame,
 *               -1 on error
 *               -2 on timeout
 */

int modbus_ascii_read(int *nd,
                      u8 **recv_data_ptr,
                      u16 *transaction_id,
                      const u8 *send_data,
                      int send_length,
                      const struct timespec *recv_timeout) {

  struct timespec end_time, *ts_ptr;
  int res, recv_length;
  int iter;  /* Warning: if you change the var type of iter from int,
              *          then you must also change the INT_MAX constant
              *          further on in this function...
              */
  u8 *local_recv_data_ptr;
  nd_entry_t *nd_entry;

  /* Check input parameters... */
  if (nd == NULL)
    return -1;

  if (recv_data_ptr == NULL)
    recv_data_ptr = &local_recv_data_ptr;

  if ((send_data == NULL) && (send_length != 0))
    return -1;

  /* check if nd is correct... */
  if ((nd_entry = nd_table_get_nd(&nd_table_, *nd)) == NULL)
    return -1;

  /* check if nd is initialzed... */
  if (nd_entry->fd < 0)
    return -1;

  /* We will potentially read many frames, and we cannot reset the timeout
   * for every frame we read. We therefore determine the absolute time_out,
   * and use this as a parameter for each call to read_frame() instead of
   * using a relative timeout.
   *
   * NOTE: see also the timeout related comment in the read_frame()= function!
   */
  ts_ptr = NULL;
  if (recv_timeout != NULL) {
     ts_ptr = &end_time;
    *ts_ptr = timespec_add_curtime(*recv_timeout);
  }

  /* NOTE: When using a half-duplex RS-485 bus, some (most ?) RS232-485
   *       converters will send back to the RS232 port whatever we write,
   *       so we will read in whatever we write out onto the bus.
   *       We will therefore have to compare
   *       the first frame we read with the one we sent. If they are
   *       identical it is because we are in fact working on a RS-485
   *       bus and must therefore read in a second frame which will be
   *       the true response to our query.
   *       If the first frame we receive is different to the last frame we
   *       just sent, then we are *not* working on a RS-485 bus, and
   *       that is already the real response to our query.
   *
   *       Flushing the input cache immediately *after* sending a frame
   *       could solve this issue, but we have no guarantee that this
   *       process would not get swapped out between the write() and
   *       flush() calls, and we could therefore be flushing the response
   *       frame along with the last frame we sent!
   */

  iter = 0;
  while ((res = recv_length = read_frame(nd_entry, recv_data_ptr, ts_ptr)) >= 0) {
    if (iter < INT_MAX) iter++;

    if ((send_length <= 0) || (nd_entry->ignore_echo == 0))
      /* any valid frame will do... */
      return recv_length;

    if ((send_length > L2_FRAME_SLAVEID_OFS + 1) && (iter == 1))
     /* We have a frame in send_data,
      * so we must make sure we are not reading in the frame just sent...
      *
      * We must only do this for the first frame we read. Subsequent
      * frames are guaranteed not to be the previously sent frame
      * since the modbus_ascii_write() resets the recv buffer.
      * Remember too that valid modbus responses may be exactly the same
      * as the request frame!!
      */
      if (recv_length == send_length)
        if (memcmp(*recv_data_ptr, send_data, recv_length) == 0)
          /* recv == send !!! */
          /* read in another frame. */
          continue;

    /* The frame read is either:
     *  - different to the frame in send_data
     *  - or there is only the slave id in send_data[L2_FRAME_SLAVEID_OFS]
     *  - or both of the above...
     */
    if (send_length > L2_FRAME_SLAVEID_OFS)
      if (recv_length > L2_FRAME_SLAVEID_OFS)
        /* check that frame is from/to the correct slave... */
        if ((*recv_data_ptr)[L2_FRAME_SLAVEID_OFS] == send_data[L2_FRAME_SLAVEID_OFS])
          /* yep, it is... */
          return recv_length;

    /* The frame we have received is not acceptable...
     * Let's read a new frame.
     */
  } /* while(...) */

  /* error reading response! */
  /* Return the error returned by read_frame! */
  return res;
}





/**************************************************************/
/**************************************************************/
/****                                                      ****/
/****                                                      ****/
/****        Initialising and Shutting Down Library        ****/
/****                                                      ****/
/****                                                      ****/
/**************************************************************/
/**************************************************************/

/******************************/
/**                          **/
/**   Load Default Values    **/
/**                          **/
/******************************/

static void set_defaults(int *baud,
                         int *parity,
                         int *data_bits,
                         int *stop_bits) {
  /* Set the default values, if required... */
  if (*baud == 0)
    *baud = DEF_BAUD_RATE;
  if (*data_bits == 0)
    *data_bits = DEF_DATA_BITS;
  if (*stop_bits == 0) {
    if (*parity == 0)
      *stop_bits = DEF_STOP_BITS_NOP; /* no parity */
    else
      *stop_bits = DEF_STOP_BITS_PAR; /* parity used */
  }
}





/******************************/
/**                          **/
/**    Initialise Library    **/
/**                          **/
/******************************/

int modbus_ascii_init(int nd_count,
                      optimization_t opt,
                      int *extra_bytes)
{
#ifdef DEBUG
  printf("modbus_asc_init(): called...\n");
  printf("creating %d node descriptors\n", nd_count);
  if (opt == optimize_speed)
    printf("optimizing for speed\n");
  if (opt == optimize_size)
    printf("optimizing for size\n");
#endif

    /* check input parameters...*/
  if (0 == nd_count) {
    if (extra_bytes != NULL)
      // Not the corect value for this layer. 
      // What we set it to in case this layer is not used!
      *extra_bytes = 0; 
    return 0;
  }
  if (nd_count <= 0)
    goto error_exit_0;

  if (extra_bytes == NULL)
    goto error_exit_0;

    /* initialise nd table... */
  if (nd_table_init(&nd_table_, nd_count) < 0)
    goto error_exit_0;

    /* remember the optimization choice for later reference... */
  optimization_ = opt;

#ifdef DEBUG
  printf("modbus_asc_init(): returning succesfuly...\n");
#endif
  return 0;

error_exit_0:
  if (extra_bytes != NULL)
    *extra_bytes = 0; // The value we set this to in case of error.
  return -1;
}



/******************************/
/**                          **/
/**    Open node descriptor  **/
/**                          **/
/******************************/

/* Open a node for master or slave operation.
 * Returns the node descriptor, or -1 on error.
 *
 * This function is mapped onto both
 * modbus_connect() and modbus_listen()
 */
int modbus_ascii_connect(node_addr_t node_addr) {
  int node_descriptor;
  nd_entry_t *nd_entry;

#ifdef DEBUG
  printf("modbus_ascii_connect(): called...\n");
  printf("opening %s\n", node_addr.addr.ascii.device);
  printf("baud_rate = %d\n", node_addr.addr.ascii.baud);
  printf("parity = %d\n", node_addr.addr.ascii.parity);
  printf("data_bits = %d\n", node_addr.addr.ascii.data_bits);
  printf("stop_bits = %d\n", node_addr.addr.ascii.stop_bits);
  printf("ignore_echo = %d\n", node_addr.addr.ascii.ignore_echo);
#endif

  /* Check for valid address family */
  if (node_addr.naf != naf_ascii)
    /* wrong address type... */
    goto error_exit_0;

  /* find a free node descriptor */
  if ((node_descriptor = nd_table_get_free_nd(&nd_table_)) < 0)
    /* if no free nodes to initialize, then we are finished... */
    goto error_exit_0;
  if ((nd_entry = nd_table_get_nd(&nd_table_, node_descriptor)) == NULL)
    /* strange, this should not occur... */
    goto error_exit_0;

  /* set the default values... */
  set_defaults(&(node_addr.addr.ascii.baud),
               &(node_addr.addr.ascii.parity),
               &(node_addr.addr.ascii.data_bits),
               &(node_addr.addr.ascii.stop_bits));

  if (nd_entry_connect(nd_entry, &node_addr, optimization_) < 0)
    goto error_exit_0;

#ifdef DEBUG
  printf("modbus_ascii_connect(): %s open\n", node_addr.addr.ascii.device );
  printf("modbus_ascii_connect(): returning nd=%d\n", node_descriptor);
#endif
  return node_descriptor;

  error_exit_0:
    return -1;
}



int modbus_ascii_listen(node_addr_t node_addr) {
  return modbus_ascii_connect(node_addr);
}



/******************************/
/**                          **/
/**   Close node descriptor  **/
/**                          **/
/******************************/

int modbus_ascii_close(int nd) {
  return nd_table_free_nd(&nd_table_, nd);
}



/******************************/
/**                          **/
/**    Shutdown Library      **/
/**                          **/
/******************************/

int modbus_ascii_done(void) {
  nd_table_done(&nd_table_);
  return 0;
}





/******************************/
/**                          **/
/**                          **/
/**                          **/
/******************************/
int modbus_ascii_silence_init(void) {
  return 0;
}




/******************************/
/**                          **/
/**                          **/
/**                          **/
/******************************/


double modbus_ascii_get_min_timeout(int baud,
                                    int parity,
                                    int data_bits,
                                    int stop_bits) {
  int parity_bits, start_bits, char_bits;

  set_defaults(&baud, &parity, &data_bits, &stop_bits);
  parity_bits = (parity == 0)?0:1;
  start_bits  = 1;
  char_bits   = start_bits + data_bits + parity_bits + stop_bits;
  return (double)((MAX_ASC_FRAME_LENGTH * char_bits) / baud);
}



