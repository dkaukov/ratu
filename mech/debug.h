/*
 * debug.h
 *
 *  Created on: 15Nov.,2017
 *      Author: ziss
 */

#ifndef DEBUG_H_
#define DEBUG_H_

#include <stdio.h>

#if defined(MECH_DEBUG)

#define __ASSERT_USE_STDERR
#include <assert.h>

#define dprintf(format, ...) printf_P(PSTR( format ), ##__VA_ARGS__)

#if defined(__AVR__)
static FILE debugout = {0};

static int debug_putchar (char c, FILE *stream) {
  //CLI_serial_write(c);
  icsc.send(ID_HAL100, (char) cmdDebug, c);
  return 1;
}

inline void debugInit() {
  fdev_setup_stream (&debugout, debug_putchar, NULL, _FDEV_SETUP_WRITE);
  stdout = &debugout;
  stderr = &debugout;
}
#else

inline void debugInit() {
}

#endif

#else

inline void debugInit() {
}

#define dprintf(format, ...)
#define assert(expression)

#endif

#endif /* DEBUG_H_ */
