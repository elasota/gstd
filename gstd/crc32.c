/* crc32.c -- compute the CRC-32 of a data stream
 * Copyright (C) 1995-2022 Mark Adler
 * For conditions of distribution and use, see copyright notice in zlib.h
 *
 * This interleaved implementation of a CRC makes use of pipelined multiple
 * arithmetic-logic units, commonly found in modern CPU cores. It is due to
 * Kadatch and Jenkins (2010). See doc/crc-doc.1.0.pdf in this distribution.
 */

#include <stdint.h>
#include <stddef.h>

#include "crc32.h"
  
uint32_t crc32(uint32_t crc, const void *vbuf, size_t len)
{
    const uint8_t *buf = (const uint8_t *)vbuf;

    crc = crc ^ 0xffffffff;

    while (len)
    {
        len--;
        crc = (crc >> 8) ^ crc_table[(crc ^ *buf++) & 0xff];
    }

    return crc ^ 0xffffffff;
}
