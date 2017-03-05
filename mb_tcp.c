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
#include <time.h>       /* clock_gettime()   */
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>  /* required for htons() and ntohs() */
#include <netinet/tcp.h> /* TCP level socket options */
#include <netinet/ip.h>  /* IP  level socket options */

#include <pthread.h>
#include <sched.h>       /* sched_yield() */



#include "sin_util.h"   /* internet socket utility functions... */
#include "mb_layer1.h"  /* The public interface this file implements... */
#include "mb_tcp_private.h"



/************************************/
/**                                **/
/** Include common code...         **/
/**                                **/
/************************************/

#include "mb_time_util.h"


//#define ERRMSG
#define ERRMSG_HEAD "Modbus/TCP: "


// #define DEBUG       /* uncomment to see the data sent and received */


#ifdef DEBUG
#ifndef ERRMSG
#define ERRMSG
#endif
#endif



/**************************************************************/
/**************************************************************/
/****                                                      ****/
/****                                                      ****/
/****                Forward Declarations                  ****/
/****                    and Defaults                      ****/
/****                                                      ****/
/**************************************************************/
/**************************************************************/


  /* A Node Descriptor metadata,
   *   Due to the fact that modbus TCP is connection oriented,
   *   and that if the client detects an error the connection
   *   must be shut down and re-established automatically,
   *   the modbus TCP layer needs to keep the address of the remote server.
   *
   * We do this by implementing a node descriptor table, in which each
   *   entry will have the remote address, and the file descriptor
   *   of the socket currently in use.
   *
   * We do not pass the file descriptor up to the next higher layer. We
   *   send them the node descriptor instead...
   */
#define MB_MASTER_NODE 12
#define MB_LISTEN_NODE 14
#define MB_SLAVE_NODE  16
#define MB_FREE_NODE   18
typedef sa_family_t nd_type_t;

typedef struct {
    int    fd;                 /* socket descriptor == file descriptor */
                               /* NOTE:
                                *   Modbus TCP says that on error, we should close
                                *   a connection and retry with a new connection.
                                *   Since it takes time for a socket to close
                                *   a connection if the remote server is down,
                                *   we close the connection on the socket, close the
                                *   socket itself, and create a new one for the new
                                *   connection. There will be times when the node will
                                *   not have any valid socket, and it will have to
                                *   be created on the fly.
                                *   When the node does not have a valid socket,
                                *   fd will be set to -1
                                */
    int    node_type;          /*   What kind of use we are giving to this node...
                                *   If node_type == MB_MASTER_NODE
                                *      The node descriptor was initialised by the
                                *      modbus_connect() function.
                                *      The node descriptor is being used by a master
                                *      device, and the addr contains the address of the slave.
                                *      Remember that in this case fd may be >= 0 while
                                *      we have a valid connection, or it may be < 0 when
                                *      the connection needs to be reset.
                                *   If node_type == MB_LISTEN_NODE
                                *      The node descriptor was initialised by the
                                *      modbus_listen() function.
                                *      The node is merely used to accept() new connection
                                *      requests. The new slave connections will use another
                                *      node to transfer data.
                                *      In this case fd must be >= 0.
                                *      fd < 0 is an ilegal state and should never occur.
                                *   If node_type == MB_SLAVE_NODE
                                *      The node descriptor was initialised when a new
                                *      connection request arrived on a MB_LISTEN type node.
                                *      The node descriptor is being used by a slave device,
                                *      and is currently being used to connect to a master.
                                *      In this case fd must be >= 0.
                                *      fd < 0 is an ilegal state and should never occur.
                                *   If node_type == FREE_ND
                                *      The node descriptor is currently not being used.
                                *      In this case fd is set to -1, but is really irrelevant.
                                */
    struct sockaddr_in addr;   /* The internet adress we are using.
                                *   If node_type == MB_MASTER_NODE
                                *      addr will be the address of the remote slave
                                *   If node_type == MB_LISTEN_NODE
                                *      addr will be the address of the local listening port and network interface
                                *   If node_type == MB_SLAVE_NODE
                                *      addr will be the address of the local port and network interface
                                *       of the connection to the specific client.
                                */
    int listen_node;           /* When a slave accepts a connection through a MB_LISTEN_NODE, it will
                                * will use an empty node for the new connection, and configure this new node
                                * to use the type MB_SLAVE_NODE.
                                * The listen_node entry is only used by nodes of type MB_SLAVE_NODE.
                                * In this case, listen_node will be the node of type MB_LISTEN_NODE through
                                * which the connection request came through...
                                */ 
    int close_on_silence;      /* A flag used only by Master Nodes.
                                * When (close_on_silence > 0), then the connection to the
                                * slave device will be shut down whenever the
                                * modbus_tcp_silence_init() function is called.
                                * Remember that the connection will be automatically
                                * re-established the next time the user wishes to communicate
                                * with the same slave (using this same node descripto).
                                * If the user wishes to comply with the sugestion
                                * in the OpenModbus Spec, (s)he should set this flag
                                * if a silence interval longer than 1 second is expected.
                                */
    int print_connect_error;   /* flag to guarantee we only print an error the first time we
                                * attempt to connect to a emote server.
                                * Stops us from generting a cascade of errors while the slave
                                * is down.
                                * Flag will get reset every time we successfully
                                * establish a connection, so a message is once again generated 
                                * on the next error.
                                */
    u8 *recv_buf;              /* This node's receive buffer
                                * The library supports multiple simultaneous connections,
                                * and may need to receive multiple frames through mutiple nodes concurrently.
                                * To make the library thread-safe, we use one buffer for each node.
                                */
} nd_entry_t;


/* please make sure to lock the node table mutex before calling this function */
static int nd_entry_init(nd_entry_t *nde) {
  nde->addr.sin_family = AF_INET  ;
  nde->node_type = MB_FREE_NODE;
  nde->fd = -1; /* not currently connected... */
  /* initialise recv buffer */
  nde->recv_buf = malloc(sizeof(u8) * RECV_BUFFER_SIZE);
  if (nde->recv_buf == NULL)
    return -1;
  return 0;
}

/* please make sure to lock the node table mutex before calling this function */
static int nd_entry_done(nd_entry_t *nde) {
  free(nde->recv_buf);
  return 0;
}



typedef struct {
      /* the array of node descriptors, and current size... */
    nd_entry_t      *node;           /* array of node entries. if NULL => node table not initialized */
    int             node_count;      /* total number of nodes in the node[] array */
    int             free_node_count; /* number of free nodes in the node[] array */
    pthread_mutex_t mutex;
} nd_table_t;



static int nd_table_done(nd_table_t *ndt) {
  int count;

  if (ndt->node == NULL) 
    return 0;

  /* lock the mutex */
  while (pthread_mutex_lock(&ndt->mutex) != 0) sched_yield();
  
  /* initialise the state of each node in the array... */
  for (count = 0; count < ndt->node_count; count++) {
    nd_entry_done(&ndt->node[count]);
  } /* for() */

  free(ndt->node);
  pthread_mutex_unlock (&ndt->mutex);
  pthread_mutex_destroy(&ndt->mutex);
  *ndt = (nd_table_t){.node=NULL, .node_count=0, .free_node_count=0};
  
  return 0;
}




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
  
  /* initialise the node table mutex... */
  pthread_mutex_init(&ndt->mutex, NULL);
  if (pthread_mutex_lock(&ndt->mutex) != 0) {
#ifdef DEBUG
    perror("pthread_mutex_lock()");
    fprintf(stderr, "[%lu] Unable to lock newly crated mutex while creating new node table!\n", pthread_self());
#endif
    pthread_mutex_destroy(&ndt->mutex);
    return -1;
  }
  
  /* initialise the node descriptor metadata array... */
  ndt->node = malloc(sizeof(nd_entry_t) * nd_count);
  if (ndt->node == NULL) {
#ifdef DEBUG
    perror("malloc()");
    fprintf(stderr, "[%lu] Out of memory: error initializing node address buffer\n", pthread_self());
#endif
#ifdef ERRMSG
    perror("malloc()");
    fprintf(stderr, ERRMSG_HEAD "Out of memory. Error initializing node address buffer\n");
#endif
    pthread_mutex_unlock (&ndt->mutex);
    pthread_mutex_destroy(&ndt->mutex);
    return -1;
  }

  /* initialise the state of each node in the array... */
  for (count = 0; count < nd_count; count++) {
    if (nd_entry_init(&ndt->node[count]) < 0) {
      pthread_mutex_unlock(&ndt->mutex);
      nd_table_done(ndt);
      return -1;
    }
    ndt->node_count = count+1;
    ndt->free_node_count = count+1;
  } /* for() */

  ndt->node_count = nd_count;
  ndt->free_node_count = nd_count;

  pthread_mutex_unlock(&ndt->mutex);
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

  if (ndt->node == NULL) {
    /* Node table nt yet initialized => we must initialise the node table mutex... */
    pthread_mutex_init(&ndt->mutex, NULL);
  }
  
  /* lock the mutex */
  while (pthread_mutex_lock(&ndt->mutex) != 0) sched_yield();
  
  /* initialise the node descriptor metadata array... */
  ndt->node = realloc(ndt->node, sizeof(nd_entry_t) * (ndt->node_count + new_nd_count));
  if (ndt->node == NULL) {
#ifdef DEBUG
    perror("malloc()");
    fprintf(stderr, "[%lu] Out of memory: error initializing node address buffer\n", pthread_self());
#endif
#ifdef ERRMSG
    perror("malloc()");
    fprintf(stderr, ERRMSG_HEAD "Out of memory. Error initializing node address buffer\n");
#endif
    pthread_mutex_unlock (&ndt->mutex);
    pthread_mutex_destroy(&ndt->mutex);
    return -1;
  }

  /* initialise the state of each newly added node in the array... */
  for (count = ndt->node_count; count < ndt->node_count + new_nd_count; count++) {
    if (nd_entry_init(&ndt->node[count]) < 0) {
      pthread_mutex_unlock(&ndt->mutex);
      return -1;
    }
  } /* for() */
  ndt->node_count      += new_nd_count;
  ndt->free_node_count += new_nd_count;

  pthread_mutex_unlock(&ndt->mutex);
  return new_nd_count; /* number of succesfully created nodes! */
}
#endif


static int nd_table_get_free_node(nd_table_t *ndt, nd_type_t nd_type) {
  int count;

  /* lock the mutex */
  while (pthread_mutex_lock(&ndt->mutex) != 0) sched_yield();

  /* check for free nodes... */
  if (ndt->free_node_count <= 0) {
    /* no free nodes... */
    pthread_mutex_unlock(&ndt->mutex);
    return -1;
  }

  /* Decrement the free node counter...*/
  ndt->free_node_count--;

  /* search for a free node... */
  for (count = 0; count < ndt->node_count; count++) {
    if(ndt->node[count].node_type == MB_FREE_NODE) {
      /* found one!! Allocate it to the new type! */
      ndt->node[count].node_type = nd_type;
      pthread_mutex_unlock(&ndt->mutex);
      return count;
    }
  } /* for() */

  /* Strange... We should have free nodes, but we didn't finda any! */
  /* Let's try to get into a consistent state, and return an error! */
  ndt->free_node_count = 0;
  pthread_mutex_unlock(&ndt->mutex);
  return -1;
}



static void nd_table_close_node(nd_table_t *ndt, int nd) {

  /* lock the mutex */
  while (pthread_mutex_lock(&ndt->mutex) != 0) sched_yield();

  if(ndt->node[nd].node_type == MB_FREE_NODE) {
    /* Node already free... */
    pthread_mutex_unlock(&ndt->mutex);
    return;
  }

  /* Increment the free node counter...*/
  ndt->free_node_count++;
  /* Mark the node as being free. */
  ndt->node[nd].node_type = MB_FREE_NODE;

  pthread_mutex_unlock(&ndt->mutex);
  return;
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
 /* NOTE: The node_table_ Must be initialized correctly here! */
static nd_table_t nd_table_ = {.node=NULL, .node_count=0, .free_node_count=0};


/**************************************************************/
/**************************************************************/
/****                                                      ****/
/****                                                      ****/
/****              Local Utility functions...              ****/
/****                                                      ****/
/****                                                      ****/
/**************************************************************/
/**************************************************************/


#define min(a,b) ((a<b)?a:b)
#define max(a,b) ((a>b)?a:b)

/************************************/
/**                                **/
/**  Configure socket for Modbus   **/
/**                                **/
/************************************/


static int configure_socket(int socket_id) {

  /* configure the socket */
    /* Set it to be non-blocking. This is safe because we always use select() before reading from it! 
     * It is also required for the connect() call. The default timeout in the TCP stack is much too long
     * (typically blocks for 128 s ??) when the connect does not succedd imediately!
     */
  if (fcntl(socket_id, F_SETFL, O_NONBLOCK) < 0) {
#ifdef ERRMSG
    perror("fcntl()");
    fprintf(stderr, ERRMSG_HEAD "Error configuring socket 'non-blocking' option.\n");
#endif
    return -1;
  }

  /* configure the socket */
    /* set the TCP no delay flag. */
  {int bool_opt = 1;
  if (setsockopt(socket_id, SOL_TCP, TCP_NODELAY,
                 (const void *)&bool_opt, sizeof(bool_opt))
      < 0) {
#ifdef ERRMSG
    perror("setsockopt()");
    fprintf(stderr, ERRMSG_HEAD "Error configuring socket 'TCP no delay' option.\n");
#endif
    return -1;
  }
  }

    /* set the IP low delay option. */
  {int priority_opt = IPTOS_LOWDELAY;
  if (setsockopt(socket_id, SOL_IP, IP_TOS,
                 (const void *)&priority_opt, sizeof(priority_opt))
      < 0) {
#ifdef ERRMSG
    perror("setsockopt()");
    fprintf(stderr, ERRMSG_HEAD "Error configuring socket 'IP low delay' option.\n");
#endif
    return -1;
  }
  }

#if 0
    /* send buffer */
    /* NOTE: For slave devices, that may be receiving multiple
     *       requests before they have a chance to reply to the first,
     *       it probably is a good idea to have a large receive buffer.
     *       So it is best to leave it with the default configuration, as it is
     *       larger than the largest Modbus TCP frame.
     *
     *       For the send buffer, a smaller buffer should suffice.
     *       However, it probably does not make sense to
     *       waste time asking for a smaller buffer, since the larger
     *       default buffer has already been allocated (the socket has already
     *       been created!)
     *
     *       We might just as well leave out the configuration of the socket
     *       buffer size...
     */
#define SOCK_BUF_SIZE 300 /* The size proposed in the Modbus TCP spec. */
  {int sock_buf_size;
  sock_buf_size = SOCK_BUF_SIZE;
  if (setsockopt(socket_id, SOL_SOCKET, SO_SNDBUF,
                 (const void *)&sock_buf_size, sizeof(sock_buf_size))
      < 0)
    return -1;
    /* recv buffer */
  sock_buf_size = SOCK_BUF_SIZE;
  if (setsockopt(socket_id, SOL_SOCKET, SO_RCVBUF,
             (const void *)&sock_buf_size, sizeof(sock_buf_size))
      < 0)
    return -1;
  }
#endif

  return 0;
}


/************************************/
/**                                **/
/** Connect socket to remote host  **/
/**                                **/
/************************************/

/* This function will create a new socket, and connect it to a remote host... */
static inline int open_connection(int nd, const struct timespec *timeout) {
  int socket_id, con_res;

#ifdef DEBUG
        printf("[%lu] open_connection(): called, nd = %d\n", pthread_self(), nd);
#endif

  if (nd_table_.node[nd].fd >= 0)
    /* nd already connected) */
    return nd_table_.node[nd].fd;

  if (nd_table_.node[nd].addr.sin_family != AF_INET)
    /* invalid remote address, or invalid nd */
    return -1;

  /* lets try to connect... */
    /* create the socket */
  if ((socket_id = socket(PF_INET, DEF_TYPE, 0 /* protocol_num */)) < 0) {
#ifdef DEBUG
    perror("socket()");
    fprintf(stderr, "[%lu] Error creating socket\n", pthread_self());
#endif
#ifdef ERRMSG
    perror("socket()");
    fprintf(stderr, ERRMSG_HEAD "Error creating socket\n");
#endif
    return -1;
  }

  /* configure the socket - includes setting non-blocking option! */
  if (configure_socket(socket_id) < 0) {
    close(socket_id);
    return -1;
  };
 
  /* establish the connection to remote host */
  con_res = connect(socket_id,
                    (struct sockaddr *)&(nd_table_.node[nd].addr),
                    sizeof(nd_table_.node[nd].addr));

  /* The following condition is not strictly necessary 
   * (we could let the code fall through)
   * but it does make the code easier to read/understand...
   */
  if (con_res >= 0)
    goto success_exit; /* connected succesfully on first try! */
    
  if (con_res < 0) {
    if ((errno != EINPROGRESS) && (errno != EALREADY))
      goto error_exit; /* error in connection request! */

    /* connection request is ongoing */
    /* EINPROGRESS -> first call to connect, EALREADY -> subsequent calls to connect */
    /* Must wait for connect to complete at most 'timeout' seconds */
    {fd_set fdset;
     int res, so_error;
     socklen_t len;
     struct timespec end_time, *et_ptr;
     
     et_ptr = NULL;
     if (timeout != NULL) {
        et_ptr = &end_time;
       *et_ptr = timespec_add_curtime(*timeout);
     }
      
     FD_ZERO(&fdset);
     FD_SET(socket_id, &fdset);
     
     res = my_select(socket_id+1, NULL, &fdset, et_ptr);
     if (res  < 0) goto error_exit; /* error on call to select */
     if (res == 0) goto error_exit; /* timeout */
     /* (res  > 0) -> connection attemt completed. May have been success or failure! */
     
     len = sizeof(so_error);
     res = getsockopt(socket_id, SOL_SOCKET, SO_ERROR, &so_error, &len);
     if (res  < 0)      goto error_exit; /* error on call to getsockopt */
     if (so_error != 0) goto error_exit; /* error on connection attempt */
     goto success_exit; /* succesfully completed connection attempt! */
                        /* goto sucess_exit is not strcitly necessary - we could let the code fall through! */
    }
  }

success_exit:
  nd_table_.node[nd].fd = socket_id;
  /* Succesfully established connection => print a message next time we have error. */
  nd_table_.node[nd].print_connect_error = 1;  

#ifdef DEBUG
  printf("[%lu] open_connection(): returning...\n", pthread_self());
#endif
  return socket_id;

error_exit:
#ifdef ERRMSG
    if (nd_table_.node[nd].print_connect_error > 0) {
      perror("connect()");
      fprintf(stderr, ERRMSG_HEAD "Error establishing socket connection.\n");
      /* do not print more error messages for this node... */
      nd_table_.node[nd].print_connect_error = 0;
    }
#endif
    close(socket_id);
    return -1;
}


/* This function will accept a new connection request, and attribute it to a new node... */
static inline int accept_connection(int nd) {
  int socket_id, new_nd;

#ifdef DEBUG
        printf("[%lu] accept_connection(): called, nd = %d\n", pthread_self(), nd);
#endif

  /* NOTE: We MUST accccept8) all connection requests, even if no new node is available.
   *       => We first accept the connection request, and only later look for a node.
   *          If no node is free/available for this new connections request, the 
   *          connection will be accepted and immediately closed.
   *       Reason:
   *       When the library is used for a Modbus/TCP server and no free node is 
   *        available, if we do not accept() all newly arrived connection requests
   *        we would enter an infinite loop calling
   *           - select() (in modbus_tcp_read()) 
   *           - and accept_connection().
   *        Note that select() will continue to return immediately if the 
   *        connection request is not accept()ted!
   */
  /* lets accept new connection request... */
  if ((socket_id = accept(nd_table_.node[nd].fd, NULL, NULL)) < 0) {
#ifdef ERRMSG
    perror("accept()");
    fprintf(stderr, ERRMSG_HEAD "Error while waiting for connection request from new client\n");
#endif
    /* error establishing new connection... */
    return -1;
  }

  /* find a free node */ 
  if ((new_nd = nd_table_get_free_node(&nd_table_, MB_SLAVE_NODE)) < 0) {
    /* no available free nodes for the new connection... */
    close(socket_id);    
    return -1;
  }

  /* configure the socket - includes setting the non-blocking option! */
  if (configure_socket(socket_id) < 0) {
    nd_table_close_node(&nd_table_, new_nd);  /* first free up the un-used node. */
    close(socket_id);
    return -1;
  }

  /* set up the node entry and update the fd sets */
  nd_table_.node[new_nd].fd = socket_id;
  nd_table_.node[new_nd].listen_node = nd;

#ifdef DEBUG
        printf("[%lu] accept_connection(): returning new_nd = %d\n", pthread_self(), new_nd);
#endif
  return new_nd;
}


static inline void close_connection(int nd) {
  if (nd_table_.node[nd].fd >= 0) {
    /* disconnect the tcp connection */
    shutdown(nd_table_.node[nd].fd, SHUT_RDWR);
#ifdef ERRMSG
    int res =
#endif
    close(nd_table_.node[nd].fd);
#ifdef ERRMSG
    if (res < 0) {
      perror("close()");
      fprintf(stderr, ERRMSG_HEAD "Error closing socket\n");
    }
#endif
    nd_table_.node[nd].fd = -1;
  }

  if (nd_table_.node[nd].node_type == MB_SLAVE_NODE) {
    /* If it is a slave node, we will not be receiving any more data over this disconnected node,
     * (MB_SLAVE_NODE do not get re-connected!), so we free the node...
     */
    nd_table_close_node(&nd_table_, nd);
  }
}



/************************************/
/**                                **/
/**     Data format conversion     **/
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

static inline u16 mb_hton(u16 h_value) {
/*  return h_value; */
  return htons(h_value);
}

static inline u16 mb_ntoh(u16 m_value) {
/*  return m_value; */
  return ntohs(m_value);
}

static inline u8 msb(u16 value) {
/*  return Most Significant Byte of value; */
  return (value >> 8) & 0xFF;
}

static inline u8 lsb(u16 value) {
/*  return Least Significant Byte of value; */
  return value & 0xFF;
}

#define u16_v(char_ptr)  (*((u16 *)(&(char_ptr))))


/************************************/
/**                                **/
/**   Build/Check a frame header   **/
/**                                **/
/************************************/

/* A modbus TCP frame header has 6 bytes...
 *   header[0-1] -> transaction id
 *   header[2-3] -> must be 0
 *   header[4-5] -> frame data length (must be <= 255)
 */
#if TCP_HEADER_LENGTH < 6
#error This code assumes a header size of 6 bytes, but TCP_HEADER_LENGTH < 6
#endif

static inline void build_header(u8 *header,
                                u16 transaction_id,
                                u16 byte_count)
{
  u16_v(header[0]) = mb_hton(transaction_id);
  header[2] = 0;
  header[3] = 0;
  u16_v(header[4]) = mb_hton(byte_count);
}


static inline int check_header(u8  *header,
                               u16 *transaction_id,
                               u16 *byte_count)
{
  if ((header[2] != 0) || (header[3] != 0))
    return -1;

  *transaction_id = mb_ntoh(*(u16 *)(header + 0));
  *byte_count     = mb_ntoh(*(u16 *)(header + 4));

  if (*byte_count > MAX_L2_FRAME_LENGTH)
    return -1;

  return 0;
}





/**************************************************************/
/**************************************************************/
/****                                                      ****/
/****                                                      ****/
/****              Sending of Modbus TCP Frames            ****/
/****                                                      ****/
/****                                                      ****/
/**************************************************************/
/**************************************************************/

// pthread_mutex_t sendmsg_mutex = PTHREAD_MUTEX_INITIALIZER; 

/* NOTE: this function MUST be thread safe!! */
int modbus_tcp_write(int nd,  /* node descriptor */
                     u8 *data,
                     size_t data_length,
                     u16 transaction_id,
                     const struct timespec *transmit_timeout
                     )
{
#define data_vector_size 2

  u8            header[TCP_HEADER_LENGTH];
  struct iovec  data_vector[data_vector_size] = {
                         {(void *)header, TCP_HEADER_LENGTH},
                         {NULL, 0}};
  struct msghdr msg = {NULL, 0, data_vector, data_vector_size, NULL, 0, 0};
  int res, bytes_sent;

#ifdef DEBUG
  printf("[%lu] modbus_tcp_write(): called...  nd=%d\n", pthread_self(), nd);
#endif

  if ((nd >= nd_table_.node_count) || (nd < 0))
    /* invalid node descriptor... */
    return -1;

#ifdef DEBUG
//  printf("[%lu] locking mutex...\n", pthread_self());
#endif
//  while (pthread_mutex_lock(&sendmsg_mutex) != 0);

  /*************************
  * prepare the header...  *
  *************************/
  build_header(header, transaction_id, data_length);
#ifdef DEBUG
/* Print the hex value of each character that is about to be
 * sent over the bus.
 */
  { int i;
    printf("modbus_tcp_write(): sending data...\n");
    for(i = 0; i < TCP_HEADER_LENGTH; i++)
      printf("[0x%2X]", header[i]);
    for(i = 0; i < data_length; i++)
      printf("[0x%2X]", data[i]);
    printf("\n");
  }
#endif

  /******************************************
   * do we need to re-establish connection? *
   ******************************************/
  if (open_connection(nd, transmit_timeout) < 0) {
#ifdef DEBUG
    fprintf(stderr, "[%lu] modbus_tcp_write(): could not establish connection...\n", pthread_self());
#endif
#ifdef ERRMSG
    fprintf(stderr, ERRMSG_HEAD "could not establish connection...\n");
#endif
    return -1;
  }

  /**********************
   * write to output... *
   **********************/
   /* TWO ALTERNATIVE IMPLEMENTATIONS !!! */
#if 0
    /* write header */
  bytes_sent = 0;
  while (1) {
    res = write(nd_table_.node[nd].fd, header+bytes_sent, TCP_HEADER_LENGTH-bytes_sent);
    if (res < 0) {
      if ((errno != EAGAIN ) && (errno != EINTR )) {
        /* error sending message... */
        close_connection(nd);
        return -1;
      } else {
        continue;
      }
    } else {
      /* res >= 0 */
      bytes_sent += res;
      if (bytes_sent >= TCP_HEADER_LENGTH) {
        break;
      }
	}
  }

      /* write data */
  bytes_sent = 0;
  while (1) {
    res = write(nd_table_.node[nd].fd, data+bytes_sent, data_length-bytes_sent);
    if (res < 0) {
      if ((errno != EAGAIN ) && (errno != EINTR )) {
        /* error sending message... */
        close_connection(nd);
        return -1;
      } else {
        continue;
      }
    } else {
      /* res >= 0 */
      bytes_sent += res;
      if (bytes_sent >= data_length) {
        /* query succesfully sent! */
#ifdef DEBUG
        printf("[%lu] modbus_tcp_write(): sent %d bytes\n", pthread_self(), TCP_HEADER_LENGTH+data_length);
#endif
        return data_length;
      }
	}
  }

   /**********************
   * write to output... *
   **********************/
#else
  /* We are optimising for the most likely case, and in doing that
   * we are making the least likely case have worse behaviour!
   * Read on for an explanation...
   *
   * - The optimised behaviour for the most likely case:
   * We have set the NO_DELAY flag on the socket, so the IP datagram
   * is not delayed and is therefore sent as soon as any data is written to
   * the socket.
   * In order to send the whole message in a single IP datagram, we have to
   * write both the the header and the data with a single call to write()
   * In order to not to have to copy the data around just to add the
   * message header, we use sendmsg() instead of write()!
   *
   * - The worse behaviour for the least likely case:
   * If for some reason only part of the data is sent with the first call to
   * write(), a datagram is sent right away, and the subsequent data will
   * be sent in another datagram. :-(
   */
   /* NOTE: since snedmsg() is not thread safe, we use a mutex to protect access to this function... */

  data_vector[data_vector_size - 1].iov_base = data;
  data_vector[data_vector_size - 1].iov_len  = data_length;
  data_vector[                   0].iov_base = header;
  data_vector[                   0].iov_len  = TCP_HEADER_LENGTH;
  bytes_sent = 0;
  while (1) {
    int sendmsg_errno;
     /* Please see the comment just above the main loop!! */
    res = sendmsg(nd_table_.node[nd].fd, &msg, 0);
    sendmsg_errno = errno;
    if (res < 0) {
      if ((sendmsg_errno != EAGAIN ) && (sendmsg_errno != EINTR )) {
        /* error sending message... */
        close_connection(nd);
        return -1;
      } else {
        continue;
      }
    } else {
      /* res >= 0 */
      bytes_sent += res;
      if (bytes_sent >= data_length + TCP_HEADER_LENGTH) {
        /* query succesfully sent! */
#ifdef DEBUG
        printf("[%lu] modbus_tcp_write(): sent %d bytes\n", pthread_self(), bytes_sent);
#endif
//        pthread_mutex_unlock(&sendmsg_mutex);
#ifdef DEBUG
//        printf("[%lu] unlocked  mutex...\n", pthread_self());
#endif
        return data_length;
      }

      /* adjust the data_vector... */
      if (res < data_vector[0].iov_len) {
        u8* tmp = data_vector[0].iov_base;
        tmp += res; 
        data_vector[0].iov_len -= res;
        data_vector[0].iov_base = tmp;
      } else {
        u8* tmp = data_vector[1].iov_base;
        tmp += res; 
        res -= data_vector[0].iov_len;
        data_vector[0].iov_len  = 0;
        data_vector[1].iov_len -= res;
        data_vector[1].iov_base = tmp;
      }
    }
  } /* while (1) */
#endif

  /* humour the compiler... */
//  pthread_mutex_unlock(&sendmsg_mutex);
#ifdef DEBUG
//  printf("[%lu] unlocked  mutex...\n", pthread_self());
#endif
  return -1;
}



/**************************************************************/
/**************************************************************/
/****                                                      ****/
/****                                                      ****/
/****              Receiving Modbus TCP Frames             ****/
/****                                                      ****/
/****                                                      ****/
/**************************************************************/
/**************************************************************/


/* A helper function to modbus_tcp_read()
 *
 * WARNING: The semantics of this function are not what you would expect!
 *
 *          if (data_already_available != 0)
 *          It assumes that select() has already been called before
 *          this function got called, and we are therefore guaranteed
 *          to have at least one byte to read off the socket (the fd).
 *
 *          if (data_already_available == 0)
 *          it starts off by calling select()!
 *
 *
 * NOTE: Ususal select semantics for (a: end_time == NULL) and
 *       (b: *end_time == 0) also apply.
 *
 *       (a) Indefinite timeout
 *       (b) Try once, and and quit if no data available.
 */
/* RETURNS: number of bytes read
 *          -1 read error!
 *          -2 timeout
 */
static int read_bytes(int fd,
                      u8 *data,
                      int max_data_count,
                      const struct timespec *end_time,
                      int data_already_available)
{
  fd_set rfds;
  int res, data_count;

  data_count = 0;

  while (data_count < max_data_count) {
    /*============================*
     * wait for data availability *
     *============================*/
    if (data_already_available == 0) {
      int sel_res;
      FD_ZERO(&rfds);
      FD_SET(fd, &rfds);
      sel_res = my_select(fd + 1, &rfds, NULL, end_time);
      if (sel_res < 0)
        return -1;
      if (sel_res == 0)
        /* timeout! */
        return -2;
    }

    /*============================*
     * read the available data... *
     *============================*/
    res = read(fd, data + data_count, max_data_count - data_count);
    if (res == 0) {
      /* We are guaranteed to have data to read off the fd since we called
       * select(), but read() returned 0 bytes.
       * This means that the remote process has closed down the connection,
       * so we return 0.
       */
      return 0;
    }

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
        printf("<0x%2X>", *(data + data_count + i));
    }
#endif
    data_count += res;
    data_already_available = 0;
  } /* while ()*/

  /* data read succesfully... */
  return data_count;
}



/***************************************/
/**                                   **/
/**    Read a Modbus TCP frame        **/
/**    off a single identified node.  **/
/**                                   **/
/***************************************/

/* This private function will read a Modbus TCP frame off a single identified node
 * that we know before hand that has data ready to be read off it. The data may or may not be
 * a valid Modbus TCP frame. It is up to this function to figure that out.
 */
/* NOTES:
 *  - We re-use the recv_buf_ to load the frame header, so we have to make
 *    sure that the buffer is large enough to take it...
 */
 /* RETURNS: number of bytes read
  *          -1 on read from file/node error
  *          -2 on timeout
  */
#if RECV_BUFFER_SIZE < TCP_HEADER_LENGTH
#error The receive buffer is smaller than the frame header length.
#endif

static int modbus_tcp_read_frame(int nd,
                                 u16 *transaction_id,
                                 struct timespec *ts_ptr) {
  int fd, res;
  u16 frame_length;

#ifdef DEBUG
  printf("[%lu] modbus_tcp_read_frame(): reading off nd=%d\n", pthread_self(), nd);
#endif
  /*=========================*
   * read a Modbus TCP frame *
   *=========================*/
  /* assume error... */
  fd = nd_table_.node[nd].fd;

  /*-------------*
   * read header *
   *-------------*/
  if ((res = read_bytes(fd, nd_table_.node[nd].recv_buf, TCP_HEADER_LENGTH, ts_ptr, 1)) != TCP_HEADER_LENGTH) { 
#ifdef DEBUG
    printf("[%lu] modbus_tcp_read_frame(): frame with insuficient bytes for a valid header...\n", pthread_self());
#endif
    if (res < 0) return res;
    return -1;
  }

  /* let's check for header consistency... */
  if (check_header(nd_table_.node[nd].recv_buf, transaction_id, &frame_length) < 0) {
#ifdef DEBUG
    printf("[%lu] modbus_tcp_read_frame(): frame with non valid header...\n", pthread_self());
#endif
    return -1;
  }

  /*-----------*
   * read data *
   *-----------*/
  if ((res = read_bytes(fd, nd_table_.node[nd].recv_buf, frame_length, ts_ptr, 0)) != frame_length) { 
#ifdef DEBUG
    printf("[%lu] modbus_tcp_read_frame(): frame with non valid frame length...\n", pthread_self());
#endif
    if (res < 0) return res;
    return -1;
  }

  /* frame received succesfully... */
#ifdef DEBUG
  printf("\n");
#endif
  return frame_length;
}




/***************************************/
/**                                   **/
/**    Read a Modbus TCP frame        **/
/**    OR Accept connection requests  **/
/**    off possibly multiple node...  **/
/**                                   **/
/***************************************/

/* The public function that reads a valid modbus frame.
 * The frame is read from...:
 *   -  if (nd >= 0) and (nd is of type MB_MASTER_NODE or MB_SLAVE_NODE)
 *          The frame is read from the node descriptor nd 
 *   -  if (nd >= 0) and (nd is of type MB_LISTEN_NODE)
 *          The frame is read from the all node descriptors of type MB_SLAVE_NODE that were
 *          opened as a consequence of a connection request to the nd slave.
 *          In this case, new connection requests to nd will also be accepted! 
 *   -  if (nd == -1)
 *          The frame is read from any valid and initialised node descriptor.
 *          In this case, new connection requests to any nd of type MB_LISTEN_NODE will also be accepted! 
 *          In this case, the node where the data is eventually read from is returned in *nd.
 *
 * The send_data and send_length parameters are ignored...
 *  (However, these parameters must stay in order to keep the function 
 *   interface identical to the ASCII and RTU versons!)
 *
 * return value: The length (in bytes) of the valid frame,
 *               -1 on error
 *
 * NOTE: Ususal select semantics for (a: recv_timeout == NULL) and
 *       (b: *recv_timeout == 0) also apply.
 *
 *       (a) Indefinite timeout
 *       (b) Try once, and and quit if no data available.
 */

 /* RETURNS: number of bytes read
  *          -1 on read from file/node error
  *          -2 on timeout
  */
int modbus_tcp_read(int *nd,                /* node descriptor */
                    u8 **recv_data_ptr,
                    u16 *transaction_id,
                    const u8 *send_data,   /* ignored ! */
                    int send_length,       /* ignored ! */
                    const struct timespec *recv_timeout) {

  struct timespec end_time, *ts_ptr;
  u8 *local_recv_data_ptr;
  u16 local_transaction_id = 0;

#ifdef DEBUG
  printf("[%lu] modbus_tcp_read(): called...  nd=%d\n", pthread_self(), *nd);
#endif

  if (nd == NULL)
    return -1;

  if (*nd >= nd_table_.node_count)
    /* invalid *nd                      */
    /* remember that *nd < 0 is valid!! */
    return -1;

  if (recv_data_ptr == NULL)
    recv_data_ptr = &local_recv_data_ptr;
  if (transaction_id == NULL)
    transaction_id = &local_transaction_id;

  /* We will potentially call read() multiple times to read in a single frame.
   * We therefore determine the absolute time_out, and use this as a parameter
   * for each call to read_bytes() instead of using a relative timeout.
   *
   * NOTE: see also the timeout related comment in the read_bytes() function!
   */
  ts_ptr = NULL;
  if (recv_timeout != NULL) {
     ts_ptr = &end_time;
    *ts_ptr = timespec_add_curtime(*recv_timeout);
  }

  /* If we must read off a single node... */
  if (*nd >= 0)
    /* but the node does not have a valid fd */
    if ((nd_table_.node[*nd].node_type == MB_FREE_NODE) ||
        (nd_table_.node[*nd].fd < 0))
      /* then we return an error... */
      return -1;

  /* We will loop forever...
   * We jump out of the loop and return from the function as soon as:
   *  - we receive a valid modbus message;
   *    OR
   *  - we time out.
   * 
   *  NOTE: This loop will close connections through which we receive invalid frames.
   *        This means that the set of nodes through which we may receive data may change with each
   *        loop iteration.  => We need to re-calculate the fds in each loop iteration! 
   */

  while (1) {
    int nd_count, fd_high;
    fd_set rfds;

    /* We prepare our fd sets here so we can later call select() */
    FD_ZERO(&rfds);
    fd_high = -1;

    for (nd_count = 0; nd_count < nd_table_.node_count; nd_count++) {
      if (nd_table_.node[nd_count].node_type != MB_FREE_NODE)
      {
        if ((*nd < 0)  // we select from all nodes 
            || (*nd == nd_count)  // we select from this specific node
              // we are listening on a MB_LISTEN_NODE, so we must also receive requests sent to slave nodes
              // whose connection requests arrived through this MB_LISTEN_NDODE 
            || ((nd_table_.node[nd_count].node_type == MB_SLAVE_NODE) && (nd_table_.node[nd_count].listen_node == *nd))) 
        {
          /* check if valid fd */
          if (nd_table_.node[nd_count].fd >= 0) {
            /* Add the descriptor to the fd set... */
            FD_SET(nd_table_.node[nd_count].fd, &rfds);
            fd_high = max(fd_high, nd_table_.node[nd_count].fd);
          }
        }
      }
    } /* for(;;) */

#ifdef DEBUG
    printf("[%lu] modbus_tcp_read(): while(1) looping. fd_high = %d, nd=%d\n", pthread_self(), fd_high, *nd);
#endif

    if (fd_high == -1)
      /* we will not be reading from any node! */
      return -1;

    /* We now call select and wait for activity on the nodes we are listening to */
    { int sel_res = my_select(fd_high + 1, &rfds, NULL, ts_ptr);
      if (sel_res < 0)
        return -1;
      if (sel_res == 0)
        /* timeout! */
        return -2;
    }

    /* figure out which nd is ready to be read... */
    for (nd_count = 0; nd_count < nd_table_.node_count; nd_count++) {
      if ((nd_table_.node[nd_count].node_type != MB_FREE_NODE) &&
          (nd_table_.node[nd_count].fd >= 0)) {
        if (FD_ISSET(nd_table_.node[nd_count].fd, &rfds)) {
          /* Found the node descriptor... */
#ifdef DEBUG
          printf("[%lu] modbus_tcp_read(): my_select() returned due to activity on node nd=%d\n", pthread_self(), nd_count);
#endif
          if (nd_table_.node[nd_count].node_type == MB_LISTEN_NODE) {
            /* We must accept a new connection...
             * No need to check for errors.
             * If one occurs, there is nothing we can do...
             */
            accept_connection(nd_count);
          } else {
            /* it is a MB_SLAVE_NODE or a MB_MASTER_NODE */ 
            /* We will read a frame off this nd */
            int res;
            res = modbus_tcp_read_frame(nd_count, transaction_id, ts_ptr);
            if (res > 0) {
              *nd = nd_count;
              *recv_data_ptr = nd_table_.node[nd_count].recv_buf;
              return res;
            } 
            if (res < 0) {
                /* We had an error reading the frame...
                 * We handle it by closing the connection, as specified by
                 * the modbus TCP protocol!
                 *
                 * NOTE: The error may have been a timeout, which means this function should return immediately.
                 *       However, in this case we let the execution loop once again
                 *       in the while(1) loop. My_select() will be called again
                 *       and the timeout detected. The timeout error code (-2)
                 *       will then be returned correctly!
                 */
#ifdef DEBUG
              printf("[%lu] modbus_tcp_read(): error reading frame. Closing connection...\n", pthread_self());
#endif
              /* We close the socket... */
              close_connection(nd_count);
            }
          }
          /* we have found the node descriptor, so let's jump out of the for(;;) loop */
          break;
        }
      }
    } /* for(;;) */

    /* We were unsuccesfull reading a frame, so we try again... */
  } /* while (1) */

  /* humour the compiler... */
  return -1;
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


/* Ugly hack...
 *  Beremiz will be calling modbus_tcp_init() multiple times (through modbus_init() )
 *    (once for each plugin instance)
 *  It will also be calling modbus_tcp_done() the same number of times
 *  We only want to really shutdown the library the last time it is called.
 *  We therefore keep a counter of how many times modbus_tcp_init() is called,
 *  and decrement it in modbus_tcp_done()
 */
int modbus_tcp_init_counter = 0;

/******************************/
/**                          **/
/**   Load Default Values    **/
/**                          **/
/******************************/

static void set_defaults(const char **service) {
  /* Set the default values, if required... */
  if (*service == NULL)
    *service = DEF_SERVICE;
}


/******************************/
/**                          **/
/**    Initialise Library    **/
/**                          **/
/******************************/
/* returns the number of nodes succesfully initialised...
 * returns -1 on error.
 */
int modbus_tcp_init(int nd_count,
                    optimization_t opt /* ignored... */,
                    int *extra_bytes) {
#ifdef DEBUG
  printf("[%lu] modbus_tcp_init(): called...\n", pthread_self());
  printf("[%lu] creating %d nodes:\n", pthread_self(), nd_count);
#endif

  modbus_tcp_init_counter++;
  
    /* set the extra_bytes value... */
    /* Please see note before the modbus_rtu_write() function for a
     * better understanding of this extremely ugly hack... This will be
     * in the mb_rtu.c file!!
     *
     * The number of extra bytes that must be allocated to the data buffer
     * before calling modbus_tcp_write()
     */
  if (extra_bytes != NULL)
    *extra_bytes = 0;

  if (0 == nd_count)
    /* no need to initialise this layer! */
    return 0;
  if (nd_count <= 0)
    /* invalid node count... */
    goto error_exit_1;

  /* initialise the node table... */
  if (nd_table_init(&nd_table_, nd_count) < 0)
    goto error_exit_1;

#ifdef DEBUG
  printf("[%lu] modbus_tcp_init(): %d node(s) opened succesfully\n", pthread_self(), nd_count);
#endif
  return nd_count; /* number of succesfully created nodes! */

/*
error_exit_2:
  nd_table_done(&nd_table_);
*/
error_exit_1:
  if (extra_bytes != NULL)
    *extra_bytes = 0;
  return -1;
}






/******************************/
/**                          **/
/**    Open a Master Node    **/
/**                          **/
/******************************/
int modbus_tcp_connect(node_addr_t node_addr) {
  int node_descriptor;
  struct sockaddr_in tmp_addr;

#ifdef DEBUG
  printf("[%lu] modbus_tcp_connect(): called...\n", pthread_self());
  printf("[%lu]        %s:%s\n", pthread_self(),
         node_addr.addr.tcp.host,
         node_addr.addr.tcp.service);
#endif

  /* Check for valid address family */
  if (node_addr.naf != naf_tcp)
    /* wrong address type... */
    return -1;

  /* set the default values... */
  set_defaults(&(node_addr.addr.tcp.service));

  /* Check the parameters we were passed... */
  if(sin_initaddr(&tmp_addr,
                  node_addr.addr.tcp.host,    0,
                  node_addr.addr.tcp.service, 0,
                  DEF_PROTOCOL)
       < 0) {
#ifdef ERRMSG
    fprintf(stderr, ERRMSG_HEAD "Error parsing/resolving address %s:%s\n",
                   node_addr.addr.tcp.host,
                   node_addr.addr.tcp.service);
#endif
    return -1;
  }

  /* find a free node descriptor */
  if ((node_descriptor = nd_table_get_free_node(&nd_table_, MB_MASTER_NODE)) < 0)
    /* if no free nodes to initialize, then we are finished... */
    return -1;

  nd_table_.node[node_descriptor].addr = tmp_addr;
  nd_table_.node[node_descriptor].fd   = -1; /* not currently connected... */
  nd_table_.node[node_descriptor].close_on_silence = node_addr.addr.tcp.close_on_silence;

  if (nd_table_.node[node_descriptor].close_on_silence < 0)
    nd_table_.node[node_descriptor].close_on_silence = DEF_CLOSE_ON_SILENCE;
  
  /* WE have never tried to connect, so print an error the next time we try to connect */
  nd_table_.node[node_descriptor].print_connect_error = 1;

#ifdef DEBUG
  printf("[%lu] modbus_tcp_connect(): returning nd=%d\n", pthread_self(), node_descriptor);
#endif
  return node_descriptor;
}



/******************************/
/**                          **/
/**    Open a Slave Node     **/
/**                          **/
/******************************/

int modbus_tcp_listen(node_addr_t node_addr) {
  int fd, nd;

#ifdef DEBUG
  printf("[%lu] modbus_tcp_listen(): called...\n", pthread_self());
  printf("[%lu]        %s:%s\n", pthread_self(),
         node_addr.addr.tcp.host,
         node_addr.addr.tcp.service);
#endif

  /* Check for valid address family */
  if (node_addr.naf != naf_tcp)
    /* wrong address type... */
    goto error_exit_0;

  /* set the default values... */
  set_defaults(&(node_addr.addr.tcp.service));

  /* create a socket and bind it to the appropriate port... */
  fd = sin_bindsock(node_addr.addr.tcp.host,
                    node_addr.addr.tcp.service,
                    DEF_PROTOCOL);
  if (fd < 0) {
#ifdef ERRMSG
    fprintf(stderr, ERRMSG_HEAD "Could not bind to socket %s:%s\n", 
                    ((node_addr.addr.tcp.host==NULL)?"#ANY#":node_addr.addr.tcp.host),
                    node_addr.addr.tcp.service);
#endif
    goto error_exit_0;
  }
  if (listen(fd, DEF_MAX_PENDING_CONNECTION_REQUESTS) < 0)
    goto error_exit_0;

  /* find a free node descriptor */
  if ((nd = nd_table_get_free_node(&nd_table_, MB_LISTEN_NODE)) < 0) {
    /* if no free nodes to initialize, then we are finished... */
    goto error_exit_1;
  }

  /* nd_table_.node[nd].addr = tmp_addr; */ /* does not apply for MB_LISTEN_NODE */
  nd_table_.node[nd].fd = fd; /* not currently connected... */

#ifdef DEBUG
  printf("[%lu] modbus_tcp_listen(): returning nd=%d\n", pthread_self(), nd);
#endif
  return nd;

error_exit_1:
  close(fd);
error_exit_0:
  return -1;
}



/******************************/
/**                          **/
/**       Close a node       **/
/**                          **/
/******************************/

int modbus_tcp_close(int nd) {
#ifdef DEBUG
  fprintf(stderr, "[%lu] modbus_tcp_close(): called... nd=%d\n", pthread_self(), nd);
#endif

  if ((nd < 0) || (nd >= nd_table_.node_count)) {
    /* invalid nd */
#ifdef DEBUG
    fprintf(stderr, "[%lu] modbus_tcp_close(): invalid node %d. Should be < %d\n", pthread_self(), nd, nd_table_.node_count);
#endif
    return -1;
  }

  if (nd_table_.node[nd].node_type == MB_FREE_NODE)
    /* already free node */
    return 0;

  close_connection(nd);

  nd_table_close_node(&nd_table_, nd);

  return 0;
}



/**********************************/
/**                              **/
/**  Close all open connections  **/
/**                              **/
/**********************************/

int modbus_tcp_silence_init(void) {
  int nd;

#ifdef DEBUG
  printf("[%lu] modbus_tcp_silence_init(): called...\n", pthread_self());
#endif

  /* close all master connections that remain open... */
  for (nd = 0; nd < nd_table_.node_count; nd++)
    if (nd_table_.node[nd].node_type == MB_MASTER_NODE)
      if (nd_table_.node[nd].close_on_silence > 0)
        /* node is is being used for a master device,
         * and wishes to be closed...   ...so we close it!
         */
         close_connection(nd);

  return 0;
}



/******************************/
/**                          **/
/**   Shutdown the Library   **/
/**                          **/
/******************************/

int modbus_tcp_done(void) {
  int i;
  
  modbus_tcp_init_counter--;
  if (modbus_tcp_init_counter != 0) return 0; /* ignore this request */
  
    /* close all the connections... */
  for (i = 0; i < nd_table_.node_count; i++)
    modbus_tcp_close(i);

  /* Free memory... */
  nd_table_done(&nd_table_);

  return 0;
}




double modbus_tcp_get_min_timeout(int baud,
                                  int parity,
                                  int data_bits,
                                  int stop_bits) {
  return 0;
}
