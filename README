/*
 * Copyright (c) 2001-2003,2016 Mario de Sousa (msousa@fe.up.pt)
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



  Modbus Protocol Libraries
  =========================

  This directory contains the libararies that implement the modbus protocol
  stack.

  This protocol has been implemented as a two layer stack. Layer two includes
  the mb_master and the mb_slave protocols. Layer one is composed of
  the mb_rtu, mb_ascii and mb_tcp protocols.

  Layer1 protocols all implement the same interface, defined in mb_layer1.h
  Layer2 protocols implement different interfaces, defined in mb_master.h
  and mb_slave.h

  Which layer1 protocol that will be used by the program will depend on which 
  layer1 protocol implementation is linked to the final binary/executable.
  It is not possible to define during run-time which layer1 protocol is to
  be used. Each compiled program can only support a single layer1 protocol.

  Users of these libraries should only use functions defined in the layer2
  protocol header files (i.e. mb_master.h and mb_slave.h)

  If writing a program that will simultaneously be a master and a slave,
  then only use the mb_slave_and_master.h header file!
  In this case, do not forget to link the final binary to both the
  master and slave protocol implementations (as well as the chosen
  layer1 protocol implementation).



            ------------------------------------------
            |                    |                   |
  layer 2   |    mb_master.h     |    mb_slave.h     |
            |    mb_master.c     |    mb_slave.c     |
            |                    |                   |
            |----------------------------------------|
            |              mb_layer1.h               |
  Layer 1   |            |              |            |
            |  mb_rtu.c  |  mb_ascii.c  |  mb_tcp.c  |
            |            |              |            |
            ------------------------------------------
