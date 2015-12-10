#pragma once

#include "client.h"
#include <climits>

// Error flags
#define ERROR_PES_GENERAL   0x01
#define ERROR_PES_SCRAMBLE  0x02
#define ERROR_PES_STARTCODE 0x04
#define ERROR_DEMUX_NODATA  0x10

#define TS_SYNC_BYTE          0x47
#define TS_SIZE               188
#define TS_ERROR              0x80
#define TS_PAYLOAD_START      0x40
#define TS_TRANSPORT_PRIORITY 0x20
#define TS_PID_MASK_HI        0x1F
#define TS_SCRAMBLING_CONTROL 0xC0
#define TS_ADAPT_FIELD_EXISTS 0x20
#define TS_PAYLOAD_EXISTS     0x10
#define TS_CONT_CNT_MASK      0x0F
#define TS_ADAPT_DISCONT      0x80
#define TS_ADAPT_RANDOM_ACC   0x40 // would be perfect for detecting independent frames, but unfortunately not used by all broadcasters
#define TS_ADAPT_ELEM_PRIO    0x20
#define TS_ADAPT_PCR          0x10
#define TS_ADAPT_OPCR         0x08
#define TS_ADAPT_SPLICING     0x04
#define TS_ADAPT_TP_PRIVATE   0x02
#define TS_ADAPT_EXTENSION    0x01

#define PATPID 0x0000 // PAT PID (constant 0)
#define CATPID 0x0001 // CAT PID (constant 1)
#define MAXPID 0x2000 // for arrays that use a PID as the index

#define PTSTICKS  90000 // number of PTS ticks per second
#define PCRFACTOR 300 // conversion from 27MHz PCR extension to 90kHz PCR base
#define MAX33BIT  0x00000001FFFFFFFFLL // max. possible value with 33 bit
#define MAX27MHZ  ((MAX33BIT + 1) * PCRFACTOR - 1) // max. possible PCR value

#define MAXLANGCODE1 4 // a 3 letter language code, zero terminated
#define MAXLANGCODE2 8 // up to two 3 letter language codes, separated by '+' and zero terminated

#define MAX_SECTION_SIZE 4096 // maximum size of an SI section
#define MAX_PMT_TS  (MAX_SECTION_SIZE / TS_SIZE + 1)

#define MAXAPIDS 32 // audio
#define MAXDPIDS 16 // dolby (AC3 + DTS)
#define MAXSPIDS 32 // subtitles
#define MAXCAIDS 12 // conditional access

inline bool TsHasPayload(const unsigned char *p)
{
  return p[3] & TS_PAYLOAD_EXISTS;
}

inline bool TsHasAdaptationField(const unsigned char *p)
{
  return p[3] & TS_ADAPT_FIELD_EXISTS;
}

inline bool TsPayloadStart(const unsigned char *p)
{
  return p[1] & TS_PAYLOAD_START;
}

inline bool TsError(const unsigned char *p)
{
  return p[1] & TS_ERROR;
}

inline int TsPid(const unsigned char *p)
{
  return (p[1] & TS_PID_MASK_HI) * 256 + p[2];
}

inline bool TsIsScrambled(const unsigned char *p)
{
  return p[3] & TS_SCRAMBLING_CONTROL;
}

inline unsigned char TsGetContinuityCounter(const unsigned char *p)
{
  return p[3] & TS_CONT_CNT_MASK;
}

inline void TsSetContinuityCounter(unsigned char *p, unsigned char Counter)
{
  p[3] = (p[3] & ~TS_CONT_CNT_MASK) | (Counter & TS_CONT_CNT_MASK);
}

inline int TsPayloadOffset(const unsigned char *p)
{
  int o = TsHasAdaptationField(p) ? p[4] + 5 : 4;
  return o <= TS_SIZE ? o : TS_SIZE;
}

inline int TsGetPayload(const unsigned char **p)
{
  if (TsHasPayload(*p)) {
     int o = TsPayloadOffset(*p);
     *p += o;
     return TS_SIZE - o;
     }
  return 0;
}

inline int TsContinuityCounter(const unsigned char *p)
{
  return p[3] & TS_CONT_CNT_MASK;
}

inline int64_t TsGetPcr(const unsigned char *p)
{
  if (TsHasAdaptationField(p)) {
     if (p[4] >= 7 && (p[5] & TS_ADAPT_PCR)) {
        return ((((int64_t)p[ 6]) << 25) |
                (((int64_t)p[ 7]) << 17) |
                (((int64_t)p[ 8]) <<  9) |
                (((int64_t)p[ 9]) <<  1) |
                (((int64_t)p[10]) >>  7)) * PCRFACTOR +
               (((((int)p[10]) & 0x01) << 8) |
                ( ((int)p[11])));
        }
     }
  return -1;
}
