/*
 * Copyright (c) 2016 Mario de Sousa (msousa@fe.up.pt)
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


/* sin_util.c */

#include "sin_util.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>   // gethostbyname(), ...
#include <errno.h>   // errno
#include <stdlib.h>  // strtoll()
#include <ctype.h>   // isspace()
#include <string.h>  // memcpy(), memset()
#include <strings.h> // strcasecmp()
#include <stdio.h>   // perror()

#ifndef INADDR_NONE
#define INADDR_NONE       0xffffffff
#endif

/* Last time I (msousa) checked, this was not being defined in QNX */
#ifndef socklen_t
typedef unsigned int socklen_t;
#endif


static inline int str_is_whitespace(const char *str) {
  const char *s;
  for (s = str; *s; s++) if (!isspace(*s)) return 0; // non whitespace char found
  return 1; // all whitespace, or empty ""
}

/* Convert a string to a number, allowing for leading and trailing whitespace */
/* Number may be in decimal, hexadecimal (leading '0x') or octal (leading '0') format */
static long long str_to_portnum(const char *str) {
  long long port = 0;
  char *errstr;
  #define PORT_MIN 0
  #define PORT_MAX 65535
  errno = 0; // strtoll() does not set errno to 0 on success!!
  port = strtoll(str, &errstr, 0 /* accept base 8, 10 and 16 */);
  if (str   == errstr)                        return -1; // not a number
  if (errno == ERANGE)                        return -2; // out of range
  if (!str_is_whitespace(errstr))             return -1; // not a number (has trailing characters)
  if ((port < PORT_MIN) || (port > PORT_MAX)) return -2; // out of range  
  return (port);
}



static int gettypebyname(const char *protocol) {
   if (strcasecmp(protocol, "tcp") == 0) return SOCK_STREAM;
   if (strcasecmp(protocol, "udp") == 0) return SOCK_DGRAM;
   if (strcasecmp(protocol, "raw") == 0) return SOCK_RAW;
   return SOCK_PACKET; // a deprecated service type, we use here as error.
}


static int getportbyname(in_port_t *port, const char *service, const char *protocol) {
   struct servent *se;
   int32_t tmp;   
   // if service is NULL, "" or string of whitespace, then set port to 0, and return -1.
   // Used when binding to a random port on the local host...
   if (    port == NULL)                                 {                              return -1;}
   if ( service == NULL)                                 {*port = 0;                    return -1;}
   if (str_is_whitespace(service))                       {*port = 0;                    return -1;}
   if ((se  = getservbyname(service, protocol)) != NULL) {*port = se->s_port;           return  0;}
   if ((tmp = str_to_portnum(service)) >= 0)             {*port = htons((uint16_t)tmp); return  0;}
   return -2;
}


static int getipbyname(struct in_addr *ip_addr, const char *host) {
   struct hostent *he;
   // if host is NULL, "", or "*", then set ip_addr to INADDR_ANY, and return -1.
   // Used when binding to all interfaces on the local host...
   if ( host == NULL)                                 {ip_addr->s_addr = INADDR_ANY; return -1;}
   if (str_is_whitespace(host))                       {ip_addr->s_addr = INADDR_ANY; return -1;}
   if (strcmp(host, "*") == 0)                        {ip_addr->s_addr = INADDR_ANY; return -1;}
   if ((he = gethostbyname(host)) != NULL) {memcpy((char *)ip_addr, he->h_addr, he->h_length); return 0;}
   if ((ip_addr->s_addr = inet_addr(host)) != INADDR_NONE) {return 0;}
   return -2;
}





int sin_initaddr(struct sockaddr_in *sin,
                  const char *host,    int allow_null_host, // 1 => allow host NULL, "" or "*" -> INADDR_ANY
                  const char *service, int allow_null_serv, // 1 => allow serivce NULL or ""   -> port = 0
                  const char *protocol) {
  int he = allow_null_host?-1:0;
  int se = allow_null_serv?-1:0;

  memset((void *)sin, 0, sizeof(sin));
  sin->sin_family = AF_INET;
  
  if (getportbyname(&(sin->sin_port), service, protocol) < se) return -1; 
  if (getipbyname  (&(sin->sin_addr), host)              < he) return -1;
  return 0;
}



/* Create a socket for the IP protocol family, and connect to remote host. */
int sin_connsock(const char *host, const char *service, const char *protocol) {
  struct sockaddr_in sin;
  int s, type;

  if (sin_initaddr(&sin, host, 0, service, 0, protocol) < 0) return -1;
  if ((type = gettypebyname(protocol)) == SOCK_PACKET)       return -1;
  /* create the socket */
  if ((s = socket(PF_INET, type, 0)) < 0)                   {perror("socket()");  return -1;}   
  /* connect the socket */
  if (connect(s, (struct sockaddr *)&sin, sizeof(sin)) < 0) {perror("connect()"); return -1;}

  return(s);
}




/* Create a socket for the IP protocol family, and bind to local host's port. */
int sin_bindsock(const char *host, const char *service, const char *protocol) {
  struct sockaddr_in sin;
  int s, type;

  // TODO allow random port... Needs new input parameter to function interface!
  if (sin_initaddr(&sin, host, 1, service, 0, protocol) < 0) return -1;
  if ((type = gettypebyname(protocol)) == SOCK_PACKET)       return -1;
  /* create the socket */
  if ((s = socket(PF_INET, type, 0)) < 0)                   {perror("socket()");  return -1;}   
  /* bind the socket */
  if (bind(s, (struct sockaddr *)&sin, sizeof (sin)) < 0)   {perror("bind()");    return -1;}
  
  return(s);
}




