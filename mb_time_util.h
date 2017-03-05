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



 /* Time handling functions used by the modbus protocols... */


#ifndef __MODBUS_TIME_UTIL_H
#define __MODBUS_TIME_UTIL_H


/************************************/
/**                                **/
/**     Time format conversion     **/
/**                                **/
/************************************/

/* Function to load a struct timeval correctly from a double. */
static inline struct timeval d_to_timeval(double time) {
  struct timeval tmp;

  tmp.tv_sec  = time;
  tmp.tv_usec = 1e6*(time - tmp.tv_sec);
  return tmp;
}


/* Function to load a struct timespec correctly from a double. */
static inline struct timespec d_to_timespec(double time) {
  struct timespec tmp;

  tmp.tv_sec  = time;
  tmp.tv_nsec = 1e9*(time - tmp.tv_sec);
  return tmp;
}



/* Function to ... */
static inline struct timespec timespec_dif(struct timespec ts1, struct timespec ts2) {
  struct timespec ts;

  ts.tv_sec  = ts1.tv_sec  - ts2.tv_sec;
  if(ts1.tv_nsec > ts2.tv_nsec) {
    ts.tv_nsec = ts1.tv_nsec - ts2.tv_nsec;
  } else {
    ts.tv_nsec = 1000000000 + ts1.tv_nsec - ts2.tv_nsec;
    ts.tv_sec--;
  }

  if (ts.tv_sec < 0)
    ts.tv_sec = ts.tv_nsec = 0;

  return ts;
}

/* Function to ... */
static inline struct timespec timespec_add(struct timespec ts1, struct timespec ts2) {
  struct timespec ts;

  ts.tv_sec  = ts1.tv_sec  + ts2.tv_sec;
  ts.tv_nsec = ts1.tv_nsec + ts2.tv_nsec;
  ts.tv_sec += ts.tv_nsec / 1000000000;
  ts.tv_nsec = ts.tv_nsec % 1000000000;
  return ts;
}

/* Function to convert a struct timespec to a struct timeval. */
static inline struct timeval timespec_to_timeval(struct timespec ts) {
  struct timeval tv;

  tv.tv_sec  = ts.tv_sec;
  tv.tv_usec = ts.tv_nsec/1000;
  return tv;
}


/*
 * NOTE: clock_gettime() is rather expensive, between 7000 and 7500 clock
 *       cycles (measured with rdtsc on an Intel Pentium)
 *       gettimeofday() is half as expensive (3000 to 3500 clock cycles),
 *       but is not POSIX compliant... :-(
 *       Nevertheless this is peanuts (20 us on a 350 MHz cpu) compared to
 *       the timescales required to read a modbus frame over a serial bus
 *       (aprox. 10 ms for a 10 byte frame on a 9600 baud bus!)
 */
static inline struct timespec timespec_add_curtime(struct timespec ts) {
  struct timespec  res     = {.tv_sec = 0, .tv_nsec = 0};
  
  /* is ts = 0 also return 0 !! */
  if ((ts.tv_sec != 0) || (ts.tv_nsec != 0))
    if (clock_gettime(CLOCK_MONOTONIC, &res) >= 0)
      res = timespec_add(res, ts);
  return res;
}
  
/************************************/
/**                                **/
/** select() with absolute timeout **/
/**                                **/
/************************************/




/* My private version of select using an absolute timeout, instead of the
 * usual relative timeout.
 *
 * NOTE: Ususal select semantics for (a: end_time == NULL) and
 *       (b: *end_time == 0) also apply.
 *
 *       (a) Indefinite timeout
 *       (b) Try once, and and quit if no data available.
 */
/* Returns: -1 on error
 *           0 on timeout
 *          >0 on success
 */
static int my_select(int fd, fd_set *rfds, fd_set *wfds, const struct timespec *end_time) {

  int res;
  struct timespec cur_time;
  struct timeval timeout, *tv_ptr;
  fd_set tmp_rfds, *tmp_rfds_ptr, tmp_wfds, *tmp_wfds_ptr;
  
  tmp_rfds_ptr = NULL;
  tmp_wfds_ptr = NULL;
  if (rfds != NULL) tmp_rfds_ptr = &tmp_rfds;
  if (wfds != NULL) tmp_wfds_ptr = &tmp_wfds;

  /*============================*
   * wait for data availability *
   *============================*/
  do {
    if (rfds != NULL) tmp_rfds = *rfds;
    if (wfds != NULL) tmp_wfds = *wfds;
      /* NOTE: To do the timeout correctly we would have to revert to timers
       *       and asociated signals. That is not very thread friendly, and is
       *       probably too much of a hassle trying to figure out which signal
       *       to use. What if we don't have any free signals?
       *
       *       The following solution is not correct, as it includes a race
       *       condition. The following five lines of code should really
       *       be atomic!
       *
       * NOTE: see also the timeout related comment in the
       *       modbus_tcp_read() function!
       */
    if (end_time == NULL) {
      tv_ptr = NULL;
    } else {
      tv_ptr = &timeout;
      if ((end_time->tv_sec == 0) && (end_time->tv_nsec == 0)) {
        timeout.tv_sec = timeout.tv_usec = 0;
      } else {
        /* ATOMIC - start */
        if (clock_gettime(CLOCK_MONOTONIC, &cur_time) < 0)
          return -1;
        timeout = timespec_to_timeval(timespec_dif(*end_time, cur_time));
      }
    }

    res = select(fd, tmp_rfds_ptr, tmp_wfds_ptr, NULL, tv_ptr);
  /* ATOMIC - end */

#ifdef DEBUG
  {int i;
   if (tmp_rfds_ptr != NULL)
     for (i = 0; i < fd; i++)
       if (FD_ISSET(i, tmp_rfds_ptr))
         fprintf(stderr,"fd=%d is ready for reading\n", i);
   if (tmp_wfds_ptr != NULL)
     for (i = 0; i < fd; i++)
       if (FD_ISSET(i, tmp_wfds_ptr))
         fprintf(stderr,"fd=%d is ready for writing\n", i);
  }
#endif
    if (res == 0) {
#ifdef DEBUG
      printf("Comms time out\n");
#endif
      return 0;
    }
    if ((res < 0) && (errno != EINTR)) {
      return -1;
    }
  } while (res <= 0);

  if (rfds != NULL) *rfds = tmp_rfds;
  if (wfds != NULL) *wfds = tmp_wfds;
  return res;
}






#endif  /* __MODBUS_TIME_UTIL_H */
