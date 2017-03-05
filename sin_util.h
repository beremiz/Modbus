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


#ifndef SIN_UTIL_H
#define SIN_UTIL_H


#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>



/* Create a socket for the IP protocol family, and connect to remote host. */
int sin_connsock(const char *host, const char *service, const char *protocol);

/* Create a socket for the IP protocol family, and bind to local host's port. */
int sin_bindsock(const char *host, const char *service, const char *protocol);

/* Initialize a in_addr structure */
int sin_initaddr(struct sockaddr_in *sin,
                 const char *host,    int allow_null_host, // 1 => allow host NULL, "" or "*" -> INADDR_ANY
                 const char *service, int allow_null_serv, // 1 => allow serivce NULL or ""   -> port = 0
                 const char *protocol);
#endif
