/*
 * name: crx
 * description: Regular Expression Engine (light weight version) for C Language, using double-recursion and function pointers.
 * author: ken (hexabox) seto
 * date: 2009.08~09
 * license: LGPL
 * version: 0.13.13.cpp.1
 *
 * usage:
 *     int   len;
 *     char  output[24];
 *     char* found = regex("\\d*", "abc123", &len);
 *     if (found) {
 *         strncpy(output, found, len);
 *         output[len] = 0x00;
 *     }
 */

#include <stdio.h>

#ifndef __CRX_H
#define __CRX_H

#ifndef bool
#define bool int
#endif

#ifndef true
#define false (0)
#define true (!0)
#endif

#ifndef NULL
#define NULL (0)
#endif

char* regex(char* pattern, char* string, int* found_len);

#endif
