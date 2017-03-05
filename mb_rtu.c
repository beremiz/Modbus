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



#include <fcntl.h>      /* File control definitions */
#include <stdio.h>      /* Standard input/output */
#include <string.h>
#include <stdlib.h>
#include <termio.h>     /* POSIX terminal control definitions */
#include <sys/time.h>   /* Time structures for select() */
#include <unistd.h>     /* POSIX Symbolic Constants */
#include <assert.h>
#include <errno.h>      /* Error definitions */
#include <time.h>       /* clock_gettime()   */
#include <limits.h>     /* required for INT_MAX */

#include <netinet/in.h> /* required for htons() and ntohs() */

#include "mb_layer1.h"  /* The public interface this file implements... */
#include "mb_rtu_private.h"


#define ERRMSG
#define ERRMSG_HEAD "ModbusRTU: "

// #define DEBUG   /* uncomment to see the data sent and received */

#ifdef DEBUG
#ifndef ERRMSG
#define ERRMSG
#endif
#endif


#define SAFETY_MARGIN 10

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
/****                Forward Declarations                  ****/
/****                    and Defaults                      ****/
/****                                                      ****/
/**************************************************************/
/**************************************************************/

 /* CRC funtions... */
typedef u16 (*crc_func_t)(u8 *buf, int cnt);
static u16 crc_slow(u8 *buf, int cnt);
static u16 crc_fast(u8 *buf, int cnt);

 /* slow version does not need to be initialised, so we use it as default. */
#define DEF_CRC_FUNCTION crc_slow


/**************************************************************/
/**************************************************************/
/****                                                      ****/
/****                                                      ****/
/****              Local Utility functions...              ****/
/****                                                      ****/
/****                                                      ****/
/**************************************************************/
/**************************************************************/

/************************************/
/**                                **/
/** Miscelaneous Utility functions **/
/**                                **/
/************************************/

/*
 * Functions to convert u16 variables
 * between network and host byte order
 *
 * NOTE: Modbus uses MSByte first, just like
 *       tcp/ip, so we use the htons() and
 *       ntoh() functions to guarantee
 *       code portability.
 */
static inline u16 mb_hton(u16 h_value) 
  {return htons(h_value);} /*  return h_value; */

static inline u16 mb_ntoh(u16 m_value)
  {return ntohs(m_value);} /*  return m_value; */

/*  return Most Significant Byte of value; */
static inline u8 msb(u16 value)
  {return (value >> 8) & 0xFF;}

/*  return Least Significant Byte of value; */
static inline u8 lsb(u16 value)
  {return value & 0xFF;}

#define u16_v(char_ptr)  (*((u16 *)(&(char_ptr))))



/**************************************/
/**                                  **/
/**    Initialise a termios struct   **/
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
    tios->c_cflag |=  PARENB;
    tios->c_cflag &=~ PARODD;
  } else if(parity == 1)  { /* odd */
    tios->c_cflag |=  PARENB;
    tios->c_cflag |=  PARODD;
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


/************************************/
/**                                **/
/** A data structure - recv buffer **/
/**                                **/
/************************************/

/* A data structutre used for the receive buffer, i.e. the buffer
 * that stores the bytes we receive from the bus.
 *
 * What we realy needed here is an unbounded buffer. This may be
 * implemented by:
 *   - a circular buffer the size of the maximum frame length
 *   - a linear buffer somewhat larger than the maximum frame length
 *
 * Due to the fact that this library's API hands over the frame data
 * in a linear buffer, and also reads the data (i,e, calls to read())
 * into a linear buffer:
 *  - the circular buffer would be more efficient in aborted frame
 *    situations
 *  - the linear is more efficient when no aborted frames are recieved.
 *
 * I have decided to optimize for the most often encountered situation,
 * i.e. when no aborted frames are received.
 *
 * The linear buffer has a size larger than the maximum
 * number of bytes we intend to store in it. We simply start ignoring
 * the first bytes in the buffer in which we are not interested in, and
 * continue with the extra bytes of the buffer. When we reach the limit
 * of these extra bytes, we shift the data down so it once again
 * uses the first bytes of the buffer. The more number of extra bytes,
 * the more efficient it will be.
 *
 * Note that if we don't receive any aborted frames, it will work as a
 * simple linear buffer, and no memory shifts will be required!
 */

typedef struct {
        lb_buf_t data_buf;
          /* Flag:
            *  1 => We have detected a frame boundary using 3.5 character silence
            *  0 => We have not yet detected any frame boundary
           */
        int found_frame_boundary; /* ==1 => valid data ends at a frame boundary. */
          /* Flag:
            *  Used in the call to search_for_frame() as the history parameter!
           */
        int frame_search_history;
        } recv_buf_t;

/* A small auxiliary function... */
static inline u8 *recv_buf_init(recv_buf_t *buf, int size, int max_data_start) {
  buf->found_frame_boundary = 0;
  buf->frame_search_history = 0;
  return lb_init(&buf->data_buf, size, max_data_start);
}


/* A small auxiliary function... */
static inline void recv_buf_done(recv_buf_t *buf) {
  buf->found_frame_boundary = 0;
  buf->frame_search_history = 0;
  lb_done(&buf->data_buf);
}


/* A small auxiliary function... */
static inline void recv_buf_reset(recv_buf_t *buf) {
  buf->found_frame_boundary = 0;
  buf->frame_search_history = 0;
  lb_data_purge_all(&buf->data_buf);
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

         /* the time it takes to transmit 1.5 characters at the current baud rate */
        struct timeval time_15_char_;
         /* the time it takes to transmit 3.5 characters at the current baud rate */
        struct timeval time_35_char_;

         /* Due to the algorithm used to work around aborted frames, the modbus_read()
          * function might read beyond the current modbus frame. The extra bytes
          * must be stored for the subsequent call to modbus_read().
          */
        recv_buf_t recv_buf_;

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
                   node_addr->addr.rtu.baud,
                   node_addr->addr.rtu.parity,
                   node_addr->addr.rtu.data_bits,
                   node_addr->addr.rtu.stop_bits)
      < 0) {
#ifdef ERRMSG
    fprintf(stderr, ERRMSG_HEAD "Invalid serial line settings"
                    "(baud=%d, parity=%d, data_bits=%d, stop_bits=%d)\n",
                    node_addr->addr.rtu.baud,
                    node_addr->addr.rtu.parity,
                    node_addr->addr.rtu.data_bits,
                    node_addr->addr.rtu.stop_bits);
#endif
    goto error_exit_1;
  }

    /* set the ignore_echo flag */
  nde->ignore_echo = node_addr->addr.rtu.ignore_echo;

    /* initialise recv buffer */
  buf_size = (opt == optimize_size)?RECV_BUFFER_SIZE_SMALL:
                                    RECV_BUFFER_SIZE_LARGE;
  if (recv_buf_init(&nde->recv_buf_, buf_size, buf_size - MAX_RTU_FRAME_LENGTH)
      == NULL) {
#ifdef ERRMSG
    fprintf(stderr, ERRMSG_HEAD "Out of memory: error initializing receive buffer\n");
#endif
    goto error_exit_2;
  }

    /* open the serial port */
  if((nde->fd = open(node_addr->addr.rtu.device, O_RDWR | O_NOCTTY | O_NDELAY))
     < 0) {
#ifdef ERRMSG
    perror("open()");
    fprintf(stderr, ERRMSG_HEAD "Error opening device %s\n",
                   node_addr->addr.rtu.device);
#endif
    goto error_exit_3;
  }

  if(tcgetattr(nde->fd, &nde->old_tty_settings_) < 0) {
#ifdef ERRMSG
    perror("tcgetattr()");
    fprintf(stderr, ERRMSG_HEAD "Error reading device's %s original settings.\n",
                      node_addr->addr.rtu.device);
#endif
    goto error_exit_4;
  }

  if(tcsetattr(nde->fd, TCSANOW, &settings) < 0) {
#ifdef ERRMSG
    perror("tcsetattr()");
    fprintf(stderr, ERRMSG_HEAD "Error configuring device %s "
                    "(baud=%d, parity=%d, data_bits=%d, stop_bits=%d)\n",
                    node_addr->addr.rtu.device,
                    node_addr->addr.rtu.baud,
                    node_addr->addr.rtu.parity,
                    node_addr->addr.rtu.data_bits,
                    node_addr->addr.rtu.stop_bits);
#endif
    goto error_exit_4;
  }

  parity_bits   = (node_addr->addr.rtu.parity == 0)?0:1;
  start_bits    = 1;
  char_bits     = start_bits  + node_addr->addr.rtu.data_bits +
                  parity_bits + node_addr->addr.rtu.stop_bits;
  nde->time_15_char_ = d_to_timeval(SAFETY_MARGIN*1.5*char_bits/node_addr->addr.rtu.baud);
  nde->time_35_char_ = d_to_timeval(SAFETY_MARGIN*3.5*char_bits/node_addr->addr.rtu.baud);

#ifdef DEBUG
  fprintf(stderr, "nd_entry_connect(): %s ope{.node=NULL, .node_count=0};n\n", node_addr->addr.rtu.device );
  fprintf(stderr, "nd_entry_connect(): returning fd=%d\n", nde->fd);
#endif
  return nde->fd;

  error_exit_4:
    close(nde->fd);
  error_exit_3:
    recv_buf_done(&nde->recv_buf_);
  error_exit_2:
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
#ifdef ERRMSG
  int res =
#endif
            tcsetattr(nde->fd, TCSANOW, &nde->old_tty_settings_);
#ifdef ERRMSG
  if(res < 0)
    fprintf(stderr, ERRMSG_HEAD "Error reconfiguring serial port to it's original settings.\n");
#endif

  recv_buf_done(&nde->recv_buf_);
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
#ifdef ERRMSG
    fprintf(stderr, ERRMSG_HEAD "Out of memory: error initializing node address buffer\n");
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

 /* the crc function currently in use... */
 /* This will depend on the optimisation choice... */
crc_func_t crc_calc = DEF_CRC_FUNCTION;



/**************************************************************/
/**************************************************************/
/****                                                      ****/
/****                                                      ****/
/****                      CRC functions                   ****/
/****                                                      ****/
/****                                                      ****/
/**************************************************************/
/**************************************************************/

#if     RTU_FRAME_CRC_LENGTH < 2
#error  The CRC on modbus RTU frames requires at least 2 bytes in the frame length.
#endif


/************************************/
/**                                **/
/**     Read the CRC of a frame    **/
/**                                **/
/************************************/

/* NOTE: cnt is number of bytes in the frame _excluding_ CRC! */
static inline u16 crc_read(u8 *buf, int cnt) {
  /* For some strange reason, the crc is transmited
   * LSB first, unlike all other values...
   */
  return (buf[cnt + 1] << 8) | buf[cnt];
}


/************************************/
/**                                **/
/**    Write the CRC of a frame    **/
/**                                **/
/************************************/

/* NOTE: cnt is number of bytes in the frame _excluding_ CRC! */
static inline void crc_write(u8 *buf, int cnt) {
  /* For some strange reason, the crc is transmited
   * LSB first, unlike all other values...
   *
   * u16_v(query[string_length]) = mb_hton(temp_crc);  -> This is wrong !!
   */
  /* NOTE: We have already checked above that RTU_FRAME_CRC_LENGTH is >= 2 */
  u16 crc = crc_calc(buf, cnt);
  buf[cnt]   = lsb(crc);
  buf[cnt+1] = msb(crc);
}



/************************************/
/**                                **/
/**     A slow version of the      **/
/**          CRC function          **/
/**                                **/
/************************************/

/* crc optimized for smallest memory footprint */
static u16 crc_slow(u8 *buf, int cnt)
{
  int bit;
  u16 temp,flag;

  temp=0xFFFF;

  while (cnt-- != 0) {
    temp=temp ^ *buf++;
    for (bit=1; bit<=8; bit++) {
      flag = temp & 0x0001;
      /* NOTE:
       *  - since temp is unsigned, we are guaranteed a zero in MSbit;
       *  - if it were signed, the value placed in the MSbit would be
       *      compiler dependent!
       */
      temp >>= 1;
      if (flag)
        temp=temp ^ 0xA001;
    }
  }
  return(temp);
}




/************************************/
/**                                **/
/**     A fast version of the      **/
/**          CRC function          **/
/**                                **/
/************************************/
static u8 *crc_fast_buf = NULL;

/* crc optimized for speed */
static u16 crc_fast(u8 *buf, int cnt)
{
/* NOTE: The following arrays have been replaced by an equivalent
 *      array (crc_fast_buf[]) initialised at run-time.
 */
/*
  static u8 buf_lsb[] = {0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41,
                         0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40,
                         0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40,
                         0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41,
                         0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40,
                         0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41,
                         0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41,
                         0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40,
                         0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40,
                         0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41,
                         0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41,
                         0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40,
                         0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41,
                         0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40,
                         0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40,
                         0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41,
                         0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40,
                         0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41,
                         0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41,
                         0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40,
                         0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41,
                         0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40,
                         0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40,
                         0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41,
                         0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41,
                         0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40,
                         0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40,
                         0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41,
                         0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40,
                         0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41,
                         0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41,
                         0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40
                        };

  static u8 buf_msb[] = {0x00, 0xc0, 0xc1, 0x01, 0xc3, 0x03, 0x02, 0xc2,
                         0xc6, 0x06, 0x07, 0xc7, 0x05, 0xc5, 0xc4, 0x04,
                         0xcc, 0x0c, 0x0d, 0xcd, 0x0f, 0xcf, 0xce, 0x0e,
                         0x0a, 0xca, 0xcb, 0x0b, 0xc9, 0x09, 0x08, 0xc8,
                         0xd8, 0x18, 0x19, 0xd9, 0x1b, 0xdb, 0xda, 0x1a,
                         0x1e, 0xde, 0xdf, 0x1f, 0xdd, 0x1d, 0x1c, 0xdc,
                         0x14, 0xd4, 0xd5, 0x15, 0xd7, 0x17, 0x16, 0xd6,
                         0xd2, 0x12, 0x13, 0xd3, 0x11, 0xd1, 0xd0, 0x10,
                         0xf0, 0x30, 0x31, 0xf1, 0x33, 0xf3, 0xf2, 0x32,
                         0x36, 0xf6, 0xf7, 0x37, 0xf5, 0x35, 0x34, 0xf4,
                         0x3c, 0xfc, 0xfd, 0x3d, 0xff, 0x3f, 0x3e, 0xfe,
                         0xfa, 0x3a, 0x3b, 0xfb, 0x39, 0xf9, 0xf8, 0x38,
                         0x28, 0xe8, 0xe9, 0x29, 0xeb, 0x2b, 0x2a, 0xea,
                         0xee, 0x2e, 0x2f, 0xef, 0x2d, 0xed, 0xec, 0x2c,
                         0xe4, 0x24, 0x25, 0xe5, 0x27, 0xe7, 0xe6, 0x26,
                         0x22, 0xe2, 0xe3, 0x23, 0xe1, 0x21, 0x20, 0xe0,
                         0xa0, 0x60, 0x61, 0xa1, 0x63, 0xa3, 0xa2, 0x62,
                         0x66, 0xa6, 0xa7, 0x67, 0xa5, 0x65, 0x64, 0xa4,
                         0x6c, 0xac, 0xad, 0x6d, 0xaf, 0x6f, 0x6e, 0xae,
                         0xaa, 0x6a, 0x6b, 0xab, 0x69, 0xa9, 0xa8, 0x68,
                         0x78, 0xb8, 0xb9, 0x79, 0xbb, 0x7b, 0x7a, 0xba,
                         0xbe, 0x7e, 0x7f, 0xbf, 0x7d, 0xbd, 0xbc, 0x7c,
                         0xb4, 0x74, 0x75, 0xb5, 0x77, 0xb7, 0xb6, 0x76,
                         0x72, 0xb2, 0xb3, 0x73, 0xb1, 0x71, 0x70, 0xb0,
                         0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
                         0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54,
                         0x9c, 0x5c, 0x5d, 0x9d, 0x5f, 0x9f, 0x9e, 0x5e,
                         0x5a, 0x9a, 0x9b, 0x5b, 0x99, 0x59, 0x58, 0x98,
                         0x88, 0x48, 0x49, 0x89, 0x4b, 0x8b, 0x8a, 0x4a,
                         0x4e, 0x8e, 0x8f, 0x4f, 0x8d, 0x4d, 0x4c, 0x8c,
                         0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86,
                         0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40
                        };
*/
  u8 crc_msb = 0xFF;
  u8 crc_lsb = 0xFF;
  int index;

  if (cnt <= 0) {
    fprintf(stderr, "\nInternal program error in file %s at line %d\n\n\n", __FILE__, __LINE__);
    exit(EXIT_FAILURE);
  }

  while (cnt-- != 0) {
    index = 2 * (crc_lsb ^ *buf++);
    crc_lsb = crc_msb ^ crc_fast_buf[index]/* buf_lsb[index/2] */;
    crc_msb = crc_fast_buf[index + 1] /* buf_msb[index/2] */;
  }

  return crc_msb*0x0100 + crc_lsb;
}


/************************************/
/**                                **/
/**  init() and done() functions   **/
/**      of fast CRC version       **/
/**                                **/
/************************************/

static inline int crc_fast_init(void) {
  int i;
  u8  data[2];
  u16 tmp_crc;

  if ((crc_fast_buf = (u8 *)malloc(256 * 2)) == NULL)
    return -1;

  for (i = 0x00; i < 0x100; i++) {
    data[0] = 0xFF;
    data[1] = i;
    data[1] = ~data[1];
    tmp_crc = crc_slow(data, 2);
    crc_fast_buf[2*i    ] = lsb(tmp_crc);
    crc_fast_buf[2*i + 1] = msb(tmp_crc);
  }

  return 0;
}


static inline void crc_fast_done(void) {
  free(crc_fast_buf);
}


/************************************/
/**                                **/
/**  init() and done() functions   **/
/**         of generic CRC         **/
/**                                **/
/************************************/

static inline int crc_init(optimization_t opt) {
  switch (opt) {
    case optimize_speed:
      if (crc_fast_init() < 0)
        return -1;
      crc_calc = crc_fast;
      return 0;
    case optimize_size :
      crc_calc = crc_slow;
      return 0;
    default:
      return -1;
  }

  /* humour the compiler */
  return -1;
}


static inline int crc_done(void) {
  if (crc_calc == crc_fast)
    crc_fast_done();

  crc_calc = DEF_CRC_FUNCTION;
  return 0;
}



/**************************************************************/
/**************************************************************/
/****                                                      ****/
/****                                                      ****/
/****              Sending of Modbus RTU Frames            ****/
/****                                                      ****/
/****                                                      ****/
/**************************************************************/
/**************************************************************/

/*  W A R N I N G
 *  =============
 *     The modbus_rtu_write() function assumes that the caller
 *     has allocated a few bytes extra for the buffer containing
 *     the data. These bytes will be used to write the crc.
 *
 *     The caller of this function MUST make sure that the data
 *     buffer, although only containing data_length bytes, has
 *     been allocated with a size equal to or larger than
 *     data_length + RTU_FRAME_CRC_LENGTH bytes
 *
 *     I know, this is a very ugly hack, but we don't have much
 *     choice (please read other comments further on for more
 *     explanations)
 *
 *     We will nevertheless try and make this explicit by having the
 *     library initialisation function (modbus_rtu_init() ) return a
 *     value specifying how many extra bytes this buffer should have.
 *     Maybe this way this very ugly hack won't go unnoticed, and we
 *     won't be having any segmentation faults...!
 * 
 *     NOTE: for now the transmit_timeout is silently ignored in RTU version!
 */
int modbus_rtu_write(int    nd,
                     u8    *data,
                     size_t data_length,
                     u16    transaction_id,
                     const struct timespec *transmit_timeout
                    )
{
  fd_set rfds;
  struct timeval timeout;
  int res, send_retries;
  nd_entry_t *nd_entry;

#ifdef DEBUG
  fprintf(stderr, "modbus_rtu_write(fd=%d) called...\n", nd);
#endif
  /* check if nd is correct... */
  if ((nd_entry = nd_table_get_nd(&nd_table_, nd)) == NULL)
    return -1;

  /* check if nd is initialzed... */
  if (nd_entry->fd < 0)
    return -1;

  /**************************
  * append crc to frame... *
  **************************/
/* WARNING:
 *     The crc_write() function assumes that we have an extra
 *     RTU_FRAME_CRC_LENGTH free bytes at the end of the *data
 *     buffer.
 *     The caller of this function had better make sure he has
 *     allocated those extra bytes, or a segmentation fault will
 *     occur.
 *     Please read on why we leave this as it is...
 *
 * REASONS:
 *     We want to write the data and the crc in a single call to
 *     the OS. This is the only way we can minimally try to gurantee
 *     that we will not be introducing a silence of more than 1.5
 *     character transmission times between any two characters.
 *
 *     We could do the above using one of two methods:
 *       (a) use a special writev() call in which the data
 *           to be sent is stored in two buffers (one for the
 *           data and the other for the crc).
 *       (b) place all the data in a single linear buffer and
 *           use the normal write() function.
 *
 *     We cannot use (a) since the writev(2) function does not seem
 *     to be POSIX compliant...
 *     (b) has the drawback that we would need to allocate a new buffer,
 *      and copy all the data into that buffer. We have enough copying of
 *      data between buffers as it is, so we won't be doing it here
 *      yet again!
 *
 *      The only option that seems left over is to have the caller
 *      of this function allocate a few extra bytes. Let's hope he
 *      does not forget!
*/
  crc_write(data, data_length);
  data_length += RTU_FRAME_CRC_LENGTH;

#ifdef DEBUG
/* Print the hex value of each character that is about to be
 * sent over the bus.
 */
  { int i;
    for(i = 0; i < data_length; i++)
      fprintf(stderr, "[0x%2X]", data[i]);
    fprintf(stderr, "\n");
  }
#endif
   /* THE MAIN LOOP!!! */
  /* NOTE: The modbus standard specifies that the message must
   *       be sent continuosly over the wire with maximum
   *       inter-character delays of 1.5 character intervals.
   *
   *       If the write() call is interrupted by a signal, then
   *       this delay will most probably be exceeded. We should then
   *       re-start writing the query from the begining.
   *
   *       BUT, can we really expect the write() call to return
   *       query_length on every platform when no error occurs?
   *       The write call would still be correct if it only wrote
   *       1 byte at a time!
   *
   *       To protect ourselves getting into an infinte loop in the
   *       above cases, we specify a maximum number of retries, and
   *       hope for the best...! The worst will now be we simply do
   *       not get to send out a whole frame, and will therefore always
   *       fail on writing a modbus frame!
   */
  send_retries = RTU_FRAME_SEND_RETRY + 1; /* must try at least once... */
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
     * The following lines will guarantee that we will re-synchronise our
     * state machine with the current state of the bus.
     *
     * We first wait until the bus has been silent for at least
     * char_interval_timeout (i.e. 3.5 character interval). We then flush
     * any input and output that might be on the cache.
     */
      /* NOTES:
       *   - we do not need to reset the rfds with FD_SET(ttyfd, &rfds)
       *     before every call to select! We only wait on one file descriptor,
       *     so if select returns succesfully, it must have that same file
       *     decriptor set in the rdfs!
       *     If select returns with a timeout, then we do not get to call
       *     select again!
       *   -  On Linux, timeout (i.e. timeout) is modified by select() to
       *      reflect the amount of time not slept; most other implementations
       *      do not do this. In the cases in which timeout is not modified,
       *      we will simply have to wait for longer periods if select is
       *      interrupted by a signal.
       */
    FD_ZERO(&rfds);
    FD_SET(nd_entry->fd, &rfds);
    timeout = nd_entry->time_35_char_;
    while ((res = select(nd_entry->fd+1, &rfds, NULL, NULL, &timeout)) != 0) {
      if (res > 0) {
        /* we are receiving data over the serial port! */
        /* Throw the data away!                        */
        tcflush(nd_entry->fd, TCIFLUSH); /* flush the input stream */
        /* reset the timeout value! */
        timeout = nd_entry->time_35_char_;
        /* We do not need to reset the FD SET here! */
      } else {
        /* some kind of error ocurred */
        if (errno != EINTR)
          /* we were not interrupted by a signal */
          return -1;
        /* We will be calling select() again.
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
    tcflush(nd_entry->fd, TCIOFLUSH);       /* flush the input & output streams */
    recv_buf_reset(&nd_entry->recv_buf_);   /* reset the recv buffer            */

    /**********************
     * write to output... *
     **********************/
     /* Please see the comment just above the main loop!! */
    if ((res = write(nd_entry->fd, data, data_length)) != data_length) {
      if ((res < 0) && (errno != EAGAIN ) && (errno != EINTR ))
        return -1;
    } else {
      /* query succesfully sent! */
      /* res == query_length     */

      /*  NOTE: We do not flush the input stream after sending the frame!
       *        If the process gets swapped out between the end of writing
       *        to the serial port, and the call to flush the input of the
       *        same serial port, the response to the modbus query may be
       *        sent over between those two calls. This would result in the
       *        tcflush(ttyfd, TCIFLUSH) call flushing out the response
       *        to the query we have just sent!
       *        Not a good thing at all... ;-)
       */
      return data_length - RTU_FRAME_CRC_LENGTH;
    }
    /* NOTE: The maximum inter-character delay of 1.5 character times
     *       has most probably been exceeded, so we abort the frame and
     *       retry again...
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
/****              Receiving Modbus RTU Frames             ****/
/****                                                      ****/
/****                                                      ****/
/**************************************************************/
/**************************************************************/

#if     MIN_FRAME_LENGTH < 2
#error  Modbus RTU frames have a minimum length larger than MIN_FRAME_LENGTH.
#endif

/************************************/
/**                                **/
/**     Guess length of frame      **/
/**         being read.            **/
/**                                **/
/************************************/

/*  Auxiliary function to the search_for_frame() function.
 *
 *  NOTE: data_byte_count must be >=2 for correct operation, therefore
 *        the #error condition above.
 *
 *  Function to determine the length of the frame currently being read,
 *  assuming it is a query/response frame.
 *
 *  The guess is obtained by analysing the bytes that have already been
 *  read. Sometimes we cannot be sure what is the frame length, because
 *  not enough bytes of the frame have been read yet (for example, frames
 *  that have a byte_count value which has not yet been read). In these
 *  cases we return not the frame length, but an error (-1).
 *
 *  If we find the data does not make any sense (i.e. it cannot be a valid
 *  modbus frame), we return -1.
 */
static int frame_length(u8 *frame_data,
                        int frame_data_length,
                          /* The array containing the lengths of frames. */
                          /*   - query_frame_length[]
                           *   - response_frame_length[]
                           */
                        i8  *frame_length_array) {

  u8  function_code;
  int res;

   /* check consistency of input parameters... */
 /*
  if ((frame_data == NULL) || (frame_length_array == NULL) || (frame_data_length < 2))
    return -1;
  */

  function_code = frame_data[L2_FRAME_FUNCTION_OFS];

   /* hard code the length of response to diagnostic function 8 (0x08), with
    * subfunction 21 (0x15), and sub-sub-function (a.k.a. operation) 3 (0x03),
    * which contains a byte count...
    */
  if ((function_code == 0x08) && (frame_length_array == response_frame_lengths)) {
    if (frame_data_length < 4) {
      /* not enough info to determine the sub-function... */
      return -1;
    } else {
      if ((frame_data[2] == 0x00) && (frame_data[3] == 0x15)) {
        /* we need a couple more bytes to figure out the sub-sub-function... */
        if (frame_data_length < 6) {
          /* not enough info to determine the sub-sub-function... */
          return -1;
        } else {
          if ((frame_data[4] == 0x00) && (frame_data[5] == 0x03)) {
            /* We have found a response frame to diagnostic sub-function ... */
            if (frame_data_length < 8) {
              /* not enough info to determine the frame length */
              return -1;
            } else {
              return /*HEADER*/ 6 + mb_ntoh(u16_v(frame_data[6])) + RTU_FRAME_CRC_LENGTH;
            }
          }
        }
      }
    }
  }

  res = frame_length_array[function_code];

  switch(res) {
    case BYTE_COUNT_3 :
      if (frame_data_length >= 3)
        return BYTE_COUNT_3_HEADER  + frame_data[2] + RTU_FRAME_CRC_LENGTH;
      break;
    case BYTE_COUNT_34:
      if (frame_data_length >= 4)
        return BYTE_COUNT_34_HEADER + mb_ntoh(u16_v(frame_data[2])) + RTU_FRAME_CRC_LENGTH;
      break;
    case BYTE_COUNT_7 :
      if (frame_data_length >= 7)
        return BYTE_COUNT_7_HEADER  + frame_data[6] + RTU_FRAME_CRC_LENGTH;
      break;
    case BYTE_COUNT_11:
      if (frame_data_length >= 11)
        return BYTE_COUNT_11_HEADER + frame_data[10] + RTU_FRAME_CRC_LENGTH;
      break;
    case BYTE_COUNT_U :
      return -1;
    default:
      return res + RTU_FRAME_CRC_LENGTH;
  } /* switch() */

  /* unknown frame length */
  return -1;
}



/************************************/
/**                                **/
/**      Search for a frame        **/
/**                                **/
/************************************/

/* Search for a valid frame in the current data.
 * If no valid frame is found, then we return -1.
 *
 * NOTE: Since frame verification is done by calculating the CRC, which is rather
 *       CPU intensive, and this function may be called several times with the same,
 *       data, we keep state regarding the result of previous invocations...
 *       That is the reason for the *search_history parameter!
 */
static int search_for_frame(u8 *frame_data,
                            int frame_data_length,
                            int *search_history) {
  int query_length, resp_length;
  u8  function_code;
    /* *search_history flag will have or'ed of following values... */
#define SFF_HIST_NO_QUERY_FRAME     0x01
#define SFF_HIST_NO_RESPONSE_FRAME  0x02
#define SFF_HIST_NO_FRAME  (SFF_HIST_NO_RESPONSE_FRAME + SFF_HIST_NO_QUERY_FRAME)

  if ((*search_history == SFF_HIST_NO_FRAME) ||
      (frame_data_length < MIN_FRAME_LENGTH) ||
      (frame_data_length > MAX_RTU_FRAME_LENGTH))
    return -1;

  function_code = frame_data[L2_FRAME_FUNCTION_OFS];

  /* check for exception frame... */
  if ((function_code && 0x80) == 0x80) {
    if (frame_data_length >= EXCEPTION_FRAME_LENGTH + RTU_FRAME_CRC_LENGTH) {
      /* let's check CRC for valid frame. */
      if (   crc_calc(frame_data, EXCEPTION_FRAME_LENGTH)
          == crc_read(frame_data, EXCEPTION_FRAME_LENGTH))
        return EXCEPTION_FRAME_LENGTH + RTU_FRAME_CRC_LENGTH;
      else
        /* We have checked the CRC, and it is not a valid frame! */
        *search_history |= SFF_HIST_NO_FRAME;
    }
    return -1;
  }

  /* check for valid function code */
  if ((function_code > MAX_FUNCTION_CODE) || (function_code < 1)) {
    /* This is an invalid frame!!! */
    *search_history |= SFF_HIST_NO_FRAME;
    return -1;
  }

  /* let's guess the frame length */
  query_length = resp_length = -1;
  if ((*search_history & SFF_HIST_NO_QUERY_FRAME) == 0)
    query_length = frame_length(frame_data, frame_data_length, query_frame_lengths);
  if ((*search_history & SFF_HIST_NO_RESPONSE_FRAME) == 0)
    resp_length  = frame_length(frame_data, frame_data_length, response_frame_lengths);

  /* let's check whether any of the lengths are valid...*/
  /* If any of the guesses coincides with the available data length
   * we check that length first...
   */
  if ((frame_data_length == query_length) || (frame_data_length == resp_length)) {
    if (   crc_calc(frame_data, frame_data_length - RTU_FRAME_CRC_LENGTH)
        == crc_read(frame_data, frame_data_length - RTU_FRAME_CRC_LENGTH))
      return frame_data_length;
    /* nope, wrong guess...*/
    if (frame_data_length == query_length)
      *search_history |= SFF_HIST_NO_QUERY_FRAME;
    if (frame_data_length == resp_length)
      *search_history |= SFF_HIST_NO_RESPONSE_FRAME;
  }

  /* let's shoot for a query frame */
  if ((*search_history & SFF_HIST_NO_QUERY_FRAME) == 0) {
    if (query_length >= 0) {
      if (frame_data_length >= query_length) {
        /* let's check if we have a valid frame */
        if (   crc_calc(frame_data, query_length - RTU_FRAME_CRC_LENGTH)
            == crc_read(frame_data, query_length - RTU_FRAME_CRC_LENGTH))
          return query_length;
        else
          /* We have checked the CRC, and it is not a valid frame! */
          *search_history |= SFF_HIST_NO_QUERY_FRAME;
      }
    }
  }

  /* let's shoot for a response frame */
  if ((*search_history & SFF_HIST_NO_RESPONSE_FRAME) == 0) {
    if (resp_length >= 0) {
      if (frame_data_length >= resp_length) {
        /* let's check if we have a valid frame */
        if (   crc_calc(frame_data, resp_length - RTU_FRAME_CRC_LENGTH)
            == crc_read(frame_data, resp_length - RTU_FRAME_CRC_LENGTH))
          return resp_length;
        else
          *search_history |= SFF_HIST_NO_RESPONSE_FRAME;
      }
    }
  }

  /* Could not find valid frame... */
  return -1;
}



/************************************/
/**                                **/
/**          Read a frame          **/
/**                                **/
/************************************/

/* A small auxiliary function, just to make the code easier to read... */
static inline void next_frame_offset(recv_buf_t *buf, u8 *slave_id) {
  buf->frame_search_history = 0;
  lb_data_purge(&(buf->data_buf), 1 /* skip one byte */);

  if (slave_id == NULL)
    return;

  /* keep ignoring bytes, until we find one == *slave_id,
   * or no more bytes...
   */
  while (lb_data_count(&(buf->data_buf)) != 0) {
    if (*lb_data(&(buf->data_buf)) == *slave_id)
      return;
    lb_data_purge(&(buf->data_buf), 1 /* skip one byte */);
  }
}

/* A small auxiliary function, just to make the code easier to read... */
static inline int return_frame(recv_buf_t *buf,
                               int frame_length,
                               u8 **recv_data_ptr) {
#ifdef DEBUG
  fprintf(stderr, "\n" );
  fprintf(stderr, "returning valid frame of %d bytes.\n", frame_length);
#endif
    /* set the data pointer */
  *recv_data_ptr = lb_data(&(buf->data_buf));
    /* remove the frame bytes off the buffer */
  lb_data_purge(&(buf->data_buf), frame_length);
    /* reset the search_history flag */
  buf->frame_search_history = 0;
    /* if the buffer becomes empty, then reset boundary flag */
  if (lb_data_count(&(buf->data_buf)) <= 0)
    buf->found_frame_boundary = 0;
    /* return the frame length, excluding CRC */
  return frame_length - RTU_FRAME_CRC_LENGTH;
}

/* A function to read a valid frame off the rtu bus.
 *
 * NOTES:
 *        - The returned frame is guaranteed to be a valid frame.
 *        - The returned length does *not* include the CRC.
 *        - The returned frame is not guaranteed to have the same
 *          slave id as that stored in (*slave_id). This value is used
 *          merely in optimizing the search for wanted valid frames
 *          after reading an aborted frame. Only in this situation do
 *          we limit our search for frames with a slvae id == (*slave_id).
 *          Under normal circumstances, the value in (*slave_id) is
 *          simply ignored...
 *          If any valid frame is desired, then slave_id should be NULL.
 *
 */

/* NOTE: We cannot relly on the 3.5 character interval between frames to detect
 *       end of frame. We are reading the bytes from a user process, so in
 *       essence the bytes we are reading are coming off a cache.
 *       Any inter-character delays between the arrival of the bytes are
 *       lost as soon as they were placed in the cache.
 *
 *       Our only recourse is to analyse the frame we are reading in real-time,
 *       and check if it is a valid frame by checking it's CRC.
 *       To optimise this, we must be able to figure out the length
 *       of the frame currently being received by analysing the first bytes
 *       of that frame. Unfortunately, we have three problems with this:
 *         1) The spec does not specify the format of every possible modbus
 *            frame. For ex.functions 9, 10, 13, 14, 18 and 19(?).
 *         2) It is not possible to figure out whether a frame is a query
 *            or a response by just analysing the frame, and query and response
 *            frames have different sizes...
 *         3) A frame may be aborted in the middle! We have no easy way of telling
 *            if what we are reading is a partial (aborted) frame, followed by a
 *            correct frame.
 *       Possible solutions to:
 *         1) We could try to reverse engineer, but at the moment I have no
 *            PLCs that will generate the required frames.
 *            The chosen method is to verify the CRC if we are lucky enough to
 *            detect the 3.5 frame boundary imediately following one of these
 *            frames of unknown length.
 *            If we do not detect any frame boundary, then our only option
 *            is to consider it an aborted frame.
 *         2) We aim for the query frame (usually the shortest), and check
 *            it's CRC. If it matches, we accept, the frame, otherwise we try
 *            a response frame.
 *         3) The only way is to consider a frame boundary after each byte,
 *            (i.e. ignore one bye at a time) and verify if the following bytes
 *            constitue a valid frame (by checking the CRC).
 *
 *       When reading an aborted frame followed by two or more valid frames, if
 *       we are unlucky and do not detetect any frame boundary using the 3.5
 *       character interval, then we will most likely be reading in bytes
 *       beyond the first valid frame. This means we will have to store the extra
 *       bytes we have already read, so they may be handled the next time the
 *       read_frame() function is called.
 */
 /*
  * NOTE: The modbus RTU spec is inconsistent on how to handle
  *       inter-character delays larger than 1.5 characters.
  *       - On one paragraph it is stated that any delay larger than
  *         1.5 character times aborts the current frame, and a new
  *         frame is started.
  *       - On another paragraph it is stated that a frame must begin
  *         with a silence of 3.5 character times.
  *
  * We will therefore consider that any delay larger than 1.5 character
  * times terminates a valid frame. All the above references to the 3.5 character
  * interval should therefore be read as a 1.5 character interval.
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
                             struct timespec *end_time,
                             u8 *slave_id)
{
  /* temporary variables... */
  fd_set rfds;
  struct timeval timeout;
  int res, read_stat;
  int frame_length;
  recv_buf_t *recv_buf = &nd_entry->recv_buf_;

    /* Flag:
     *       1 => we are reading in an aborted frame, so we must
     *            start ignoring bytes...
     */
  int found_aborted_frame;

  /* assume error... */
  *recv_data_ptr = NULL;

  /*===================================*
   * Check for frame in left over data *
   *===================================*/
  /* If we have any data left over from previous call to read_frame()
   * (i.e. this very same function), then we try to interpret that
   * data, and do not wait for any extra bytes...
   */
  frame_length = search_for_frame(lb_data(&recv_buf->data_buf),
                                  lb_data_count(&recv_buf->data_buf),
                                  &recv_buf->frame_search_history);
  if (frame_length > 0)
    /* We found a valid frame! */
    return return_frame(recv_buf, frame_length, recv_data_ptr);

  /* If the left over data finished at a frame boundary, and since it
   * doesn't contain any valid frame, we discard those bytes...
   */
  if (recv_buf->found_frame_boundary == 1)
    recv_buf_reset(recv_buf);

  /*============================*
   * wait for data availability *
   *============================*/
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

  /*==============*
   * read a frame *
   *==============*/
  /* The main loop that reads one frame               */
  /*  (multiple calls to read() )                     */
  /* and jumps out as soon as it finds a valid frame. */

  found_aborted_frame = 0;
  FD_ZERO(&rfds);
  FD_SET(nd_entry->fd, &rfds);
  while (1) {

    /*------------------*
     * read frame bytes *
     *------------------*/
     /* Read in as many bytes as possible...
      * But only if we have not found a frame boundary. Once we find
      *  a frame boundary, we do not want to read in any more bytes
      *  and mix them up with the current frame's bytes.
      */
    if (recv_buf->found_frame_boundary == 0) {
      read_stat = read(nd_entry->fd,
                       lb_free(&recv_buf->data_buf),
                       lb_free_count(&recv_buf->data_buf));
      if (read_stat < 0) {
        if (errno != EINTR)
          return -1;
        else
          read_stat = 0;
      }
#ifdef DEBUG
      {/* display the hex code of each character received */
        int i;
        fprintf(stderr, "-");
        for (i=0; i < read_stat; i++)
          fprintf(stderr, "<0x%2X>", *(lb_free(&recv_buf->data_buf) + i));
      }
#endif
      lb_data_add(&recv_buf->data_buf, read_stat);
    }

    /*-----------------------*
     * check for valid frame *
     *-----------------------*/
    frame_length = search_for_frame(lb_data(&recv_buf->data_buf),
                                    lb_data_count(&recv_buf->data_buf),
                                    &recv_buf->frame_search_history);
    if (frame_length > 0)
      /* We found a valid frame! */
      return return_frame(recv_buf, frame_length, recv_data_ptr);

    /* if we reach this point, we are sure we do not have valid frame
     * of known length in the current data with the current offset...
     */

    /*---------------------------------*
     * Have we found an aborted frame? *
     *---------------------------------*/
    if (lb_data_count(&recv_buf->data_buf) >= MAX_RTU_FRAME_LENGTH)
      found_aborted_frame = 1;

    /*---------------------------------*
     * Must we try a new frame_offset? *
     *---------------------------------*/
    if (found_aborted_frame == 1) {
      /* Note that the found_aborted_frame flag is only set if:
       *   1 - we have previously detected a frame_boundary,
       *       (i.e. found_frame_boundary is == 1 !!) so we won't be
       *       reading in more bytes;
       *   2 - we have read more bytes than the maximum frame length
       *
       * Considering we have just failed finding a valid frame, and the above
       * points (1) and (2), then there is no way we are still going to
       * find a valid frame in the current data.
       * We must therefore try a new first byte for the frame...
       */
      next_frame_offset(recv_buf, slave_id);
    }

    /*-----------------------------*
     * check for data availability *
     *-----------------------------*/
    if (recv_buf->found_frame_boundary == 0) {
      /* We need more bytes!! */
      /*
       * if no character at the buffer, then we wait time_15_char_
       * before accepting end of frame
       */
      /* NOTES:
       *   - On Linux, timeout is modified by select() to reflect
       *     the amount of time not slept; most other implementations do
       *     not do this. On those platforms we will simply have to wait
       *     longer than we wished if select() is by any chance interrupted
       *     by a signal...
       */
      timeout = nd_entry->time_15_char_;
      while ((res = select(nd_entry->fd+1, &rfds, NULL, NULL, &timeout)) < 0) {
        if (errno != EINTR)
          return -1;
        /* We will be calling select() again.
         * We need to reset the FD SET !
         */
        FD_ZERO(&rfds);
        FD_SET(nd_entry->fd, &rfds);
      }

      if (res == 0) {
        int frame_length = lb_data_count(&recv_buf->data_buf);
        /* We have detected an end of frame using timing boundaries... */
        recv_buf->found_frame_boundary = 1; /* => stop trying to read any more bytes! */

        /* Let's check if we happen to have a correct frame... */
        if ((frame_length <= MAX_RTU_FRAME_LENGTH) &&
	    (frame_length - RTU_FRAME_CRC_LENGTH > 0)) {
          if (   crc_calc(lb_data(&recv_buf->data_buf), frame_length - RTU_FRAME_CRC_LENGTH)
              == crc_read(lb_data(&recv_buf->data_buf), frame_length - RTU_FRAME_CRC_LENGTH)) {
            /* We have found a valid frame. Let's get out of here! */
            return return_frame(recv_buf, frame_length, recv_data_ptr);
          }
	}

        /* We have detected a frame boundary, but the frame we read
         * is not valid...
         *
         * One of the following reasons must be the cause:
         *   1 - we are reading a single aborted frame.
         *   2 - we are reading more than one frame. The first frame,
         *       followed by any number of valid and/or aborted frames,
         *       may be one of:
         *       a - a valid frame whose length is unknown to us,
         *           i.e. it is not specified in the public Modbus spec.
         *       b - an aborted frame.
         *
         * Due to the complexity of reading 2a as a correct frame, we will
         * consider it as an aborted frame. (NOTE: it is possible, but
         * we will ignore it until the need arises... hopefully, never!)
         *
         * To put it succintly, what wee now have is an 'aborted' frame
         * followed by one or more aborted and/or valid frames. To get to
         * any valid frames, and since we do not know where they begin,
         * we will have to consider every byte as the possible begining
         * of a valid frame. For this permutation, we ignore the first byte,
         * and carry on from there...
         */
        found_aborted_frame = 1;
        lb_data_purge(&recv_buf->data_buf, 1 /* skip one byte */);
        recv_buf->frame_search_history = 0;
      }
    }

    /*-------------------------------*
     * check for data yet to process *
     *-------------------------------*/
    if ((lb_data_count(&recv_buf->data_buf) < MIN_FRAME_LENGTH) &&
        (recv_buf->found_frame_boundary == 1)) {
      /* We have no more data to process, and will not read anymore! */
      recv_buf_reset(recv_buf);
      /* Return TIMEOUT error */
      return -2;
    }
  } /* while (1)*/

  /* humour the compiler... */
  return -1;
}





/************************************/
/**                                **/
/**    Read a Modbus RTU frame     **/
/**                                **/
/************************************/

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

int modbus_rtu_read(int *nd,
                    u8 **recv_data_ptr,
                    u16 *transaction_id,
                    const u8 *send_data,
                    int send_length,
                    const struct timespec *recv_timeout) {
  struct timespec end_time, *ts_ptr;
  int res, recv_length, iter;
  u8 *local_recv_data_ptr;
  u8 *slave_id, local_slave_id;
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

  slave_id = NULL;
  if (send_length > L2_FRAME_SLAVEID_OFS) {
    local_slave_id = send_data[L2_FRAME_SLAVEID_OFS];
    slave_id = &local_slave_id;
  }

  /* We will potentially read many frames, and we cannot reset the timeout
   * for every frame we read. We therefore determine the absolute time_out,
   * and use this as a parameter for each call to read_frame() instead of
   * using a relative timeout.
   *
   * NOTE: see also the timeout related comment in the read_frame()= function!
   */
  /* get the current time... */
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
   *       If the first frame we receive is different to the query we
   *       just sent, then we are *not* working on a RS-485 bus, and
   *       that is already the real response to our query.
   *
   *       Flushing the input cache immediately after sending the query
   *       could solve this issue, but we have no guarantee that this
   *       process would not get swapped out between the write() and
   *       flush() calls, and we could therefore be flushing the response
   *       frame!
   */

  iter = 0;
  while ((res = recv_length = read_frame(nd_entry, recv_data_ptr, ts_ptr, slave_id)) >= 0) {
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
      * since the modbus_rtu_write() resets the recv buffer.
      * Remember too that valid modbus responses may be exactly the same
      * as the request frame!!
      */
      if (recv_length == send_length)
        if (memcmp(*recv_data_ptr, send_data, recv_length) == 0)
          /* recv == send !!! */
          /* read in another frame.  */
          continue;

    /* The frame read is either:
     *  - different to the frame in send_data
     *  - or there is only the slave id in send_data[0]
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

int modbus_rtu_init(int nd_count,
                    optimization_t opt,
                    int *extra_bytes)
{
#ifdef DEBUG
  fprintf(stderr, "modbus_rtu_init(): called...\n");
  fprintf(stderr, "creating %d node descriptors\n", nd_count);
  if (opt == optimize_speed)
    fprintf(stderr, "optimizing for speed\n");
  if (opt == optimize_size)
    fprintf(stderr, "optimizing for size\n");
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

  if (crc_init(opt) < 0) {
#ifdef ERRMSG
    fprintf(stderr, ERRMSG_HEAD "Out of memory: error initializing crc buffers\n");
#endif
    goto error_exit_0;
  }

    /* set the extra_bytes value... */
    /* Please see note before the modbus_rtu_write() function for a
     * better understanding of this extremely ugly hack...
     *
     * The number of extra bytes that must be allocated to the data buffer
     * before calling modbus_rtu_write()
     */
  *extra_bytes = RTU_FRAME_CRC_LENGTH;

    /* initialise nd table... */
  if (nd_table_init(&nd_table_, nd_count) < 0)
    goto error_exit_0;

    /* remember the optimization choice for later reference... */
  optimization_ = opt;

#ifdef DEBUG
  fprintf(stderr, "modbus_rtu_init(): returning succesfuly...\n");
#endif
  return 0;

error_exit_0:
  if (extra_bytes != NULL)
    // Not the corect value for this layer. 
    // What we set it to in case of error!
    *extra_bytes = 0; 
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
int modbus_rtu_connect(node_addr_t node_addr) {
  int node_descriptor;
  nd_entry_t *nd_entry;

#ifdef DEBUG
  fprintf(stderr, "modbus_rtu_connect(): called...\n");
  fprintf(stderr, "opening %s\n", node_addr.addr.rtu.device);
  fprintf(stderr, "baud_rate = %d\n", node_addr.addr.rtu.baud);
  fprintf(stderr, "parity = %d\n", node_addr.addr.rtu.parity);
  fprintf(stderr, "data_bits = %d\n", node_addr.addr.rtu.data_bits);
  fprintf(stderr, "stop_bits = %d\n", node_addr.addr.rtu.stop_bits);
  fprintf(stderr, "ignore_echo = %d\n", node_addr.addr.rtu.ignore_echo);
#endif

  /* Check for valid address family */
  if (node_addr.naf != naf_rtu)
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
  set_defaults(&(node_addr.addr.rtu.baud),
               &(node_addr.addr.rtu.parity),
               &(node_addr.addr.rtu.data_bits),
               &(node_addr.addr.rtu.stop_bits));

#ifdef DEBUG
  fprintf(stderr, "modbus_rtu_connect(): calling nd_entry_connect()\n");
#endif
  if (nd_entry_connect(nd_entry, &node_addr, optimization_) < 0)
    goto error_exit_0;

#ifdef DEBUG
  fprintf(stderr, "modbus_rtu_connect(): %s open\n", node_addr.addr.rtu.device);
  fprintf(stderr, "modbus_rtu_connect(): returning nd=%d\n", node_descriptor);
#endif
  return node_descriptor;

  error_exit_0:
#ifdef DEBUG
  fprintf(stderr, "modbus_rtu_connect(): error!\n");
#endif
    return -1;
}



int modbus_rtu_listen(node_addr_t node_addr) {
  return modbus_rtu_connect(node_addr);
}



/******************************/
/**                          **/
/**   Close node descriptor  **/
/**                          **/
/******************************/

int modbus_rtu_close(int nd) {
  return nd_table_free_nd(&nd_table_, nd);
}



/******************************/
/**                          **/
/**    Shutdown Library      **/
/**                          **/
/******************************/

int modbus_rtu_done(void) {
  nd_table_done(&nd_table_);
  crc_done();

  return 0;
}




/******************************/
/**                          **/
/**                          **/
/**                          **/
/******************************/
int modbus_rtu_silence_init(void) {
  return 0;
}




/******************************/
/**                          **/
/**                          **/
/**                          **/
/******************************/


double modbus_rtu_get_min_timeout(int baud,
                                  int parity,
                                  int data_bits,
                                  int stop_bits) {
  int parity_bits, start_bits, char_bits;

  set_defaults(&baud, &parity, &data_bits, &stop_bits);
  parity_bits = (parity == 0)?0:1;
  start_bits  = 1;
  char_bits   = start_bits + data_bits + parity_bits + stop_bits;
  return (double)((MAX_RTU_FRAME_LENGTH * char_bits) / baud);
}


