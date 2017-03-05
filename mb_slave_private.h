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



#ifndef MODBUS_SLAVE_PRIVATE_H
#define MODBUS_SLAVE_PRIVATE_H

#include "mb_slave.h"
#include "mb_util.h"


#define DEF_LAYER2_SEND_RETRIES 1

#define DEF_IGNORE_ECHO         0

#define DEF_OPTIMIZATION        optimize_speed




int mb_slave_init__(int extra_bytes);
int mb_slave_done__(void);


#endif  /* MODBUS_SLAVE_PRIVATE_H */








