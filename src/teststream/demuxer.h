#pragma once
/*
 *      Copyright (C) 2005-2012 Team XBMC
 *      Copyright (C) 2015 Team KODI
 *
 *      http://kodi.tv
 *
 *  This Program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2, or (at your option)
 *  any later version.
 *
 *  This Program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with KODI; see the file COPYING.  If not, see
 *  <http://www.gnu.org/licenses/>.
 *
 */

/*
 * Taken from vdr-plugin-vnsi (KODI server plugin for VDR) to use his test
 * stream support.
 */

#include <list>
#include "config.h"
#include "parser.h"
#include "platform/threads/threads.h"

struct sStreamPacket;
class cTSStream;
class cVideoBuffer;

struct sStreamInfo
{
  int pID;
  eStreamType type;
  eStreamContent content;
  char language[MAXLANGCODE2];
  int subtitlingType;
  int compositionPageId;
  int ancillaryPageId;
  bool handleRDS;
  void SetLanguage(const char* lang)
  {
    language[0] = lang[0];
    language[1] = lang[1];
    language[2] = lang[2];
    language[3] = 0;
  }
};


// PAT/PMT Parser:

#define MAX_PMT_PIDS 32

class cPatPmtParser {
private:
  unsigned char pmt[MAX_SECTION_SIZE];
  int pmtSize;
  int patVersion;
  int pmtVersion;
  int pmtPids[MAX_PMT_PIDS + 1]; // list is zero-terminated
  int vpid;
  int ppid;
  int vtype;
  int apids[MAXAPIDS + 1]; // list is zero-terminated
  int atypes[MAXAPIDS + 1]; // list is zero-terminated
  char alangs[MAXAPIDS][MAXLANGCODE2];
  int dpids[MAXDPIDS + 1]; // list is zero-terminated
  int dtypes[MAXDPIDS + 1]; // list is zero-terminated
  char dlangs[MAXDPIDS][MAXLANGCODE2];
  int spids[MAXSPIDS + 1]; // list is zero-terminated
  char slangs[MAXSPIDS][MAXLANGCODE2];
  unsigned char subtitlingTypes[MAXSPIDS];
  uint16_t compositionPageIds[MAXSPIDS];
  uint16_t ancillaryPageIds[MAXSPIDS];
  bool updatePrimaryDevice;
protected:
  int SectionLength(const unsigned char *Data, int Length) { return (Length >= 3) ? ((int(Data[1]) & 0x0F) << 8)| Data[2] : 0; }
public:
  cPatPmtParser(bool UpdatePrimaryDevice = false);
  void Reset(void);
       ///< Resets the parser. This function must be called whenever a new
       ///< stream is parsed.
  void ParsePat(const unsigned char *Data, int Length);
       ///< Parses the PAT data from the single TS packet in Data.
       ///< Length is always TS_SIZE.
  void ParsePmt(const unsigned char *Data, int Length);
       ///< Parses the PMT data from the single TS packet in Data.
       ///< Length is always TS_SIZE.
       ///< The PMT may consist of several TS packets, which
       ///< are delivered to the parser through several subsequent calls to
       ///< ParsePmt(). The whole PMT data will be processed once the last packet
       ///< has been received.
  bool ParsePatPmt(const unsigned char *Data, int Length);
       ///< Parses the given Data (which may consist of several TS packets, typically
       ///< an entire frame) and extracts the PAT and PMT.
       ///< Returns true if a valid PAT/PMT has been detected.
  bool GetVersions(int &PatVersion, int &PmtVersion) const;
       ///< Returns true if a valid PAT/PMT has been parsed and stores
       ///< the current version numbers in the given variables.
  bool IsPmtPid(int Pid) const { for (int i = 0; pmtPids[i]; i++) if (pmtPids[i] == Pid) return true; return false; }
       ///< Returns true if Pid the one of the PMT pids as defined by the current PAT.
       ///< If no PAT has been received yet, false will be returned.
  int Vpid(void) const { return vpid; }
       ///< Returns the video pid as defined by the current PMT, or 0 if no video
       ///< pid has been detected, yet.
  int Ppid(void) const { return ppid; }
       ///< Returns the PCR pid as defined by the current PMT, or 0 if no PCR
       ///< pid has been detected, yet.
  int Vtype(void) const { return vtype; }
       ///< Returns the video stream type as defined by the current PMT, or 0 if no video
       ///< stream type has been detected, yet.
  const int *Apids(void) const { return apids; }
  const int *Dpids(void) const { return dpids; }
  const int *Spids(void) const { return spids; }
  int Apid(int i) const { return (0 <= i && i < MAXAPIDS) ? apids[i] : 0; }
  int Dpid(int i) const { return (0 <= i && i < MAXDPIDS) ? dpids[i] : 0; }
  int Spid(int i) const { return (0 <= i && i < MAXSPIDS) ? spids[i] : 0; }
  int Atype(int i) const { return (0 <= i && i < MAXAPIDS) ? atypes[i] : 0; }
  int Dtype(int i) const { return (0 <= i && i < MAXDPIDS) ? dtypes[i] : 0; }
  const char *Alang(int i) const { return (0 <= i && i < MAXAPIDS) ? alangs[i] : ""; }
  const char *Dlang(int i) const { return (0 <= i && i < MAXDPIDS) ? dlangs[i] : ""; }
  const char *Slang(int i) const { return (0 <= i && i < MAXSPIDS) ? slangs[i] : ""; }
  unsigned char SubtitlingType(int i) const { return (0 <= i && i < MAXSPIDS) ? subtitlingTypes[i] : 0; }
  uint16_t CompositionPageId(int i) const { return (0 <= i && i < MAXSPIDS) ? compositionPageIds[i] : uint16_t(0); }
  uint16_t AncillaryPageId(int i) const { return (0 <= i && i < MAXSPIDS) ? ancillaryPageIds[i] : uint16_t(0); }
};



class cDemuxer
{
public:
  cDemuxer();
  virtual ~cDemuxer();
  int Read(sStreamPacket *packet, sStreamPacket *packet_side_data);
  cTSStream *GetFirstStream();
  cTSStream *GetNextStream();
  void Open(cVideoBuffer *videoBuffer);
  void Close();
  bool SeekTime(int64_t time);
  uint32_t GetSerial() { return m_MuxPacketSerial; }
  void SetSerial(uint32_t serial) { m_MuxPacketSerial = serial; }
  void BufferStatus(bool &timeshift, uint32_t &start, uint32_t &end);
  uint16_t GetError();

protected:
  bool EnsureParsers();
  void ResetParsers();
  void SetChannelStreams();
  void SetChannelPids(cPatPmtParser *patPmtParser);
  cTSStream *FindStream(int Pid);
  void AddStreamInfo(sStreamInfo &stream);
  bool GetTimeAtPos(off_t *pos, int64_t *time);
  std::list<cTSStream*> m_Streams;
  std::list<cTSStream*>::iterator m_StreamsIterator;
  std::list<sStreamInfo> m_StreamInfos;
  cPatPmtParser m_PatPmtParser;
  int m_OldPmtVersion;
  bool m_WaitIFrame;
  cVideoBuffer *m_VideoBuffer;
  PLATFORM::CMutex m_Mutex;
  uint32_t m_MuxPacketSerial;
  sPtsWrap m_PtsWrap;
  uint16_t m_Error;
  bool m_SetRefTime;
  time_t m_refTime, m_endTime, m_wrapTime;

  int m_vpid;
  int m_ppid;
  int m_vtype;
  int m_Apids[MAXAPIDS + 1];
  int m_Atypes[MAXAPIDS + 1];
  int m_Dpids[MAXDPIDS + 1];
  int m_Dtypes[MAXDPIDS + 1];
  int m_Spids[MAXSPIDS + 1];
  char m_ALangs[MAXAPIDS][MAXLANGCODE2];
  char m_DLangs[MAXDPIDS][MAXLANGCODE2];
  char m_SLangs[MAXSPIDS][MAXLANGCODE2];

  int m_tpid;
};
