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

#include "config.h"
#include "demuxer.h"
#include "parser.h"
#include "videobuffer.h"

#include "libsi/si.h"
#include "libsi/section.h"
#include "libsi/descriptor.h"

using namespace ADDON;
using namespace PLATFORM;

//#define dbgpatpmt(a...) fprintf(stderr, a)
#define dbgpatpmt(a...)

cPatPmtParser::cPatPmtParser(bool UpdatePrimaryDevice)
{
  updatePrimaryDevice = UpdatePrimaryDevice;
  Reset();
}

void cPatPmtParser::Reset(void)
{
  pmtSize = 0;
  patVersion = pmtVersion = -1;
  pmtPids[0] = 0;
  vpid = vtype = 0;
  ppid = 0;
}

void cPatPmtParser::ParsePat(const unsigned char *Data, int Length)
{
  // Unpack the TS packet:
  int PayloadOffset = TsPayloadOffset(Data);
  Data += PayloadOffset;
  Length -= PayloadOffset;
  // The PAT is always assumed to fit into a single TS packet
  if ((Length -= Data[0] + 1) <= 0)
    return;
  Data += Data[0] + 1; // process pointer_field
  SI::PAT Pat(Data, false);
  if (Pat.CheckCRCAndParse())
  {
    dbgpatpmt("PAT: TSid = %d, c/n = %d, v = %d, s = %d, ls = %d\n", Pat.getTransportStreamId(), Pat.getCurrentNextIndicator(), Pat.getVersionNumber(), Pat.getSectionNumber(), Pat.getLastSectionNumber());
    if (patVersion == Pat.getVersionNumber())
      return;
    int NumPmtPids = 0;
    SI::PAT::Association assoc;
    for (SI::Loop::Iterator it; Pat.associationLoop.getNext(assoc, it); )
    {
      dbgpatpmt("     isNITPid = %d\n", assoc.isNITPid());
      if (!assoc.isNITPid())
      {
        if (NumPmtPids <= MAX_PMT_PIDS)
          pmtPids[NumPmtPids++] = assoc.getPid();
        dbgpatpmt("     service id = %d, pid = %d\n", assoc.getServiceId(), assoc.getPid());
      }
    }
    pmtPids[NumPmtPids] = 0;
    patVersion = Pat.getVersionNumber();
  }
  else
    XBMC->Log(LOG_ERROR, "ERROR: can't parse PAT");
}

void cPatPmtParser::ParsePmt(const unsigned char *Data, int Length)
{
  // Unpack the TS packet:
  bool PayloadStart = TsPayloadStart(Data);
  int PayloadOffset = TsPayloadOffset(Data);
  Data += PayloadOffset;
  Length -= PayloadOffset;
  // The PMT may extend over several TS packets, so we need to assemble them
  if (PayloadStart)
  {
    pmtSize = 0;
    if ((Length -= Data[0] + 1) <= 0)
      return;
    Data += Data[0] + 1; // this is the first packet
    if (SectionLength(Data, Length) > Length)
    {
      if (Length <= int(sizeof(pmt)))
      {
        memcpy(pmt, Data, Length);
        pmtSize = Length;
      }
      else
        XBMC->Log(LOG_ERROR, "ERROR: PMT packet length too big (%d byte)!", Length);
      return;
    }
    // the packet contains the entire PMT section, so we run into the actual parsing
  }
  else if (pmtSize > 0)
  {
    // this is a following packet, so we add it to the pmt storage
    if (Length <= int(sizeof(pmt)) - pmtSize)
    {
      memcpy(pmt + pmtSize, Data, Length);
      pmtSize += Length;
    }
    else
    {
      XBMC->Log(LOG_ERROR, "ERROR: PMT section length too big (%d byte)!", pmtSize + Length);
      pmtSize = 0;
    }
    if (SectionLength(pmt, pmtSize) > pmtSize)
      return; // more packets to come
    // the PMT section is now complete, so we run into the actual parsing
    Data = pmt;
  }
  else
    return; // fragment of broken packet - ignore
  SI::PMT Pmt(Data, false);
  if (Pmt.CheckCRCAndParse())
  {
    dbgpatpmt("PMT: sid = %d, c/n = %d, v = %d, s = %d, ls = %d\n", Pmt.getServiceId(), Pmt.getCurrentNextIndicator(), Pmt.getVersionNumber(), Pmt.getSectionNumber(), Pmt.getLastSectionNumber());
    dbgpatpmt("     pcr = %d\n", Pmt.getPCRPid());
    if (pmtVersion == Pmt.getVersionNumber())
      return;
    int NumApids = 0;
    int NumDpids = 0;
    int NumSpids = 0;
    vpid = vtype = 0;
    ppid = 0;
    apids[0] = 0;
    dpids[0] = 0;
    spids[0] = 0;
    atypes[0] = 0;
    dtypes[0] = 0;
    SI::PMT::Stream stream;
    for (SI::Loop::Iterator it; Pmt.streamLoop.getNext(stream, it); )
    {
      dbgpatpmt("     stream type = %02X, pid = %d", stream.getStreamType(), stream.getPid());
      switch (stream.getStreamType())
      {
        case 0x01: // STREAMTYPE_11172_VIDEO
        case 0x02: // STREAMTYPE_13818_VIDEO
        case 0x1B: // H.264
                      vpid = stream.getPid();
                      vtype = stream.getStreamType();
                      ppid = Pmt.getPCRPid();
                      break;
        case 0x03: // STREAMTYPE_11172_AUDIO
        case 0x04: // STREAMTYPE_13818_AUDIO
        case 0x0F: // ISO/IEC 13818-7 Audio with ADTS transport syntax
        case 0x11: // ISO/IEC 14496-3 Audio with LATM transport syntax
                      {
                      if (NumApids < MAXAPIDS) {
                         apids[NumApids] = stream.getPid();
                         atypes[NumApids] = stream.getStreamType();
                         *alangs[NumApids] = 0;
                         SI::Descriptor *d;
                         for (SI::Loop::Iterator it; (d = stream.streamDescriptors.getNext(it)); ) {
                             switch (d->getDescriptorTag()) {
                               case SI::ISO639LanguageDescriptorTag: {
                                    SI::ISO639LanguageDescriptor *ld = (SI::ISO639LanguageDescriptor *)d;
                                    SI::ISO639LanguageDescriptor::Language l;
                                    char *s = alangs[NumApids];
                                    int n = 0;
                                    for (SI::Loop::Iterator it; ld->languageLoop.getNext(l, it); ) {
                                        if (*ld->languageCode != '-') { // some use "---" to indicate "none"
                                           dbgpatpmt(" '%s'", l.languageCode);
                                           if (n > 0)
                                              *s++ = '+';
                                           strncpy(s, l.languageCode, MAXLANGCODE1);
                                           s += strlen(s);
                                           if (n++ > 1)
                                              break;
                                           }
                                        }
                                    }
                                    break;
                               default: ;
                               }
                             delete d;
                             }
                         NumApids++;
                         apids[NumApids] = 0;
                         }
                      }
                      break;
        case 0x06: // STREAMTYPE_13818_PES_PRIVATE
                      {
                      int dpid = 0;
                      int dtype = 0;
                      char lang[MAXLANGCODE1] = "";
                      SI::Descriptor *d;
                      for (SI::Loop::Iterator it; (d = stream.streamDescriptors.getNext(it)); ) {
                          switch (d->getDescriptorTag()) {
                            case SI::AC3DescriptorTag:
                            case SI::EnhancedAC3DescriptorTag:
                                 dbgpatpmt(" AC3");
                                 dpid = stream.getPid();
                                 dtype = d->getDescriptorTag();
                                 break;
                            case SI::SubtitlingDescriptorTag:
                                 dbgpatpmt(" subtitling");
                                 if (NumSpids < MAXSPIDS) {
                                    spids[NumSpids] = stream.getPid();
                                    *slangs[NumSpids] = 0;
                                    subtitlingTypes[NumSpids] = 0;
                                    compositionPageIds[NumSpids] = 0;
                                    ancillaryPageIds[NumSpids] = 0;
                                    SI::SubtitlingDescriptor *sd = (SI::SubtitlingDescriptor *)d;
                                    SI::SubtitlingDescriptor::Subtitling sub;
                                    char *s = slangs[NumSpids];
                                    int n = 0;
                                    for (SI::Loop::Iterator it; sd->subtitlingLoop.getNext(sub, it); ) {
                                        if (sub.languageCode[0]) {
                                           dbgpatpmt(" '%s'", sub.languageCode);
                                           subtitlingTypes[NumSpids] = sub.getSubtitlingType();
                                           compositionPageIds[NumSpids] = sub.getCompositionPageId();
                                           ancillaryPageIds[NumSpids] = sub.getAncillaryPageId();
                                           if (n > 0)
                                              *s++ = '+';
                                           strncpy(s, sub.languageCode, MAXLANGCODE1);
                                           s += strlen(s);
                                           if (n++ > 1)
                                              break;
                                           }
                                        }
                                    NumSpids++;
                                    spids[NumSpids] = 0;
                                    }
                                 break;
                            case SI::ISO639LanguageDescriptorTag: {
                                 SI::ISO639LanguageDescriptor *ld = (SI::ISO639LanguageDescriptor *)d;
                                 dbgpatpmt(" '%s'", ld->languageCode);
                                 strncpy(lang, ld->languageCode, MAXLANGCODE1);
                                 }
                                 break;
                            default: ;
                            }
                          delete d;
                          }
                      if (dpid) {
                         if (NumDpids < MAXDPIDS) {
                            dpids[NumDpids] = dpid;
                            dtypes[NumDpids] = dtype;
                            strncpy(dlangs[NumDpids], lang, sizeof(dlangs[NumDpids]));
                            NumDpids++;
                            dpids[NumDpids] = 0;
                            }
                         }
                      }
                      break;
        case 0x81: // STREAMTYPE_USER_PRIVATE - AC3 audio for ATSC and BD
        case 0x82: // STREAMTYPE_USER_PRIVATE - DTS audio for BD
                      {
                      dbgpatpmt(" %s",
                          stream.getStreamType() == 0x81 ? "AC3" :
                          stream.getStreamType() == 0x82 ? "DTS" : "");
                      char lang[MAXLANGCODE1] = { 0 };
                      SI::Descriptor *d;
                      for (SI::Loop::Iterator it; (d = stream.streamDescriptors.getNext(it)); ) {
                          switch (d->getDescriptorTag()) {
                            case SI::ISO639LanguageDescriptorTag: {
                                 SI::ISO639LanguageDescriptor *ld = (SI::ISO639LanguageDescriptor *)d;
                                 dbgpatpmt(" '%s'", ld->languageCode);
                                 strncpy(lang, ld->languageCode, MAXLANGCODE1);
                                 }
                                 break;
                            default: ;
                            }
                         delete d;
                         }
                      if (NumDpids < MAXDPIDS) {
                         dpids[NumDpids] = stream.getPid();
                         dtypes[NumDpids] = SI::AC3DescriptorTag;
                         strncpy(dlangs[NumDpids], lang, sizeof(dlangs[NumDpids]));
                         NumDpids++;
                         dpids[NumDpids] = 0;
                         }
                      }
                      break;
        case 0x90: // PGS subtitles for BD
        {
          dbgpatpmt(" subtitling");
          char lang[MAXLANGCODE1] = { 0 };
          SI::Descriptor *d;
          for (SI::Loop::Iterator it; (d = stream.streamDescriptors.getNext(it)); )
          {
            switch (d->getDescriptorTag())
            {
              case SI::ISO639LanguageDescriptorTag:
              {
                SI::ISO639LanguageDescriptor *ld = (SI::ISO639LanguageDescriptor *)d;
                dbgpatpmt(" '%s'", ld->languageCode);
                strncpy(lang, ld->languageCode, MAXLANGCODE1);
                if (NumSpids < MAXSPIDS)
                {
                  spids[NumSpids] = stream.getPid();
                  *slangs[NumSpids] = 0;
                  subtitlingTypes[NumSpids] = 0;
                  compositionPageIds[NumSpids] = 0;
                  ancillaryPageIds[NumSpids] = 0;
                  NumSpids++;
                  spids[NumSpids] = 0;
                }
              }
              break;
              default: ;
            }
            delete d;
          }
        }
        break;
        default: ;
      }
      dbgpatpmt("\n");
    }
    pmtVersion = Pmt.getVersionNumber();
  }
  else
    XBMC->Log(LOG_ERROR, "ERROR: can't parse PMT");
  pmtSize = 0;
}

bool cPatPmtParser::ParsePatPmt(const unsigned char *Data, int Length)
{
  while (Length >= TS_SIZE)
  {
    if (*Data != TS_SYNC_BYTE)
      break; // just for safety
    int Pid = TsPid(Data);
    if (Pid == PATPID)
      ParsePat(Data, TS_SIZE);
    else if (IsPmtPid(Pid))
    {
      ParsePmt(Data, TS_SIZE);
      if (patVersion >= 0 && pmtVersion >= 0)
        return true;
    }
    Data += TS_SIZE;
    Length -= TS_SIZE;
  }
  return false;
}

bool cPatPmtParser::GetVersions(int &PatVersion, int &PmtVersion) const
{
  PatVersion = patVersion;
  PmtVersion = pmtVersion;
  return patVersion >= 0 && pmtVersion >= 0;
}


cDemuxer::cDemuxer()
{
  m_OldPmtVersion = -1;
}

cDemuxer::~cDemuxer()
{

}

void cDemuxer::Open(cVideoBuffer *videoBuffer)
{
  const CLockObject lock(m_Mutex);

  m_VideoBuffer = videoBuffer;
  m_OldPmtVersion = -1;

//  if (m_CurrentChannel.Vpid())
//    m_WaitIFrame = true;
//  else
    m_WaitIFrame = false;

  m_PtsWrap.m_Wrap = false;
  m_PtsWrap.m_NoOfWraps = 0;
  m_PtsWrap.m_ConfirmCount = 0;
  m_MuxPacketSerial = 0;
  m_Error = ERROR_DEMUX_NODATA;
  m_SetRefTime = true;
}

void cDemuxer::Close()
{
  const CLockObject lock(m_Mutex);

  for (std::list<cTSStream*>::iterator it = m_Streams.begin(); it != m_Streams.end(); ++it)
  {
    dbgpatpmt("Deleting stream parser for pid=%i and type=%i", (*it)->GetPID(), (*it)->Type());
    delete (*it);
  }
  m_Streams.clear();
  m_StreamInfos.clear();
}

int cDemuxer::Read(sStreamPacket *packet, sStreamPacket *packet_side_data)
{
  uint8_t *buf;
  int len;
  cTSStream *stream;

  const CLockObject lock(m_Mutex);

  // clear packet
  if (!packet)
    return -1;
  packet->data = NULL;
  packet->streamChange = false;
  packet->pmtChange = false;

  // read TS Packet from buffer
  len = m_VideoBuffer->Read(&buf, TS_SIZE, m_endTime, m_wrapTime);
  // eof
  if (len == -2)
    return -2;
  else if (len != TS_SIZE)
    return -1;

  m_Error &= ~ERROR_DEMUX_NODATA;

  int ts_pid = TsPid(buf);

  // parse PAT/PMT
  if (ts_pid == PATPID)
  {
    m_PatPmtParser.ParsePat(buf, TS_SIZE);
  }
  else if (m_PatPmtParser.IsPmtPid(ts_pid))
  {
    int patVersion, pmtVersion;
    m_PatPmtParser.ParsePmt(buf, TS_SIZE);
    if (m_PatPmtParser.GetVersions(patVersion, pmtVersion))
    {
      if (pmtVersion != m_OldPmtVersion)
      {
        SetChannelPids(&m_PatPmtParser);
        SetChannelStreams();
        m_PatPmtParser.Reset();
        m_OldPmtVersion = pmtVersion;
        if (EnsureParsers())
        {
          packet->pmtChange = true;
            return 1;
        }
      }
    }
  }
  else if (stream = FindStream(ts_pid))
  {
    int error = stream->ProcessTSPacket(buf, packet, packet_side_data, m_WaitIFrame);
    if (error == 0)
    {
      m_WaitIFrame = false;

      packet->serial = m_MuxPacketSerial;
      packet_side_data->serial = m_MuxPacketSerial;
      if (m_SetRefTime)
      {
        m_refTime = m_VideoBuffer->GetRefTime();
        packet->reftime = m_refTime;
        m_SetRefTime = false;
      }
      return 1;
    }
    else if (error < 0)
    {
      m_Error |= abs(error);
    }
  }

  return 0;
}

bool cDemuxer::SeekTime(int64_t time)
{
  off_t pos, pos_min, pos_max, pos_limit, start_pos;
  int64_t ts, ts_min, ts_max, last_ts;
  int no_change;

  if (!m_VideoBuffer->HasBuffer())
    return false;

  const CLockObject lock(m_Mutex);

  // rescale to 90khz
  time = cTSStream::Rescale(time, 90000, DVD_TIME_BASE);

  m_VideoBuffer->GetPositions(&pos, &pos_min, &pos_max);

  if (!GetTimeAtPos(&pos_min, &ts_min))
  {
    ResetParsers();
    m_WaitIFrame = true;
    return false;
  }

  if (ts_min >= time)
  {
    m_VideoBuffer->SetPos(pos_min);
    ResetParsers();
    m_WaitIFrame = true;
    m_MuxPacketSerial++;
    return true;
  }

  int64_t timecur;
  GetTimeAtPos(&pos, &timecur);

  // get time at end of buffer
  unsigned int step= 1024;
  bool gotTime;
  do
  {
    pos_max -= step;
    gotTime = GetTimeAtPos(&pos_max, &ts_max);
    step += step;
  } while (!gotTime && pos_max >= step);

  if (!gotTime)
  {
    ResetParsers();
    m_WaitIFrame = true;
    return false;
  }

  if (ts_max <= time)
  {
    ResetParsers();
    m_WaitIFrame = true;
    m_MuxPacketSerial++;
    return true;
  }

  // bisect seek
  if(ts_min > ts_max)
  {
    ResetParsers();
    m_WaitIFrame = true;
    return false;
  }
  else if (ts_min == ts_max)
  {
    pos_limit = pos_min;
  }
  else
    pos_limit = pos_max;

  no_change = 0;
  ts = time;
  last_ts = 0;
  while (pos_min < pos_limit)
  {
    if (no_change==0)
    {
      // interpolate position
      pos = cTSStream::Rescale(time - ts_min, pos_max - pos_min, ts_max - ts_min)
          + pos_min - (pos_max - pos_limit);
    }
    else if (no_change==1)
    {
      // bisection, if interpolation failed to change min or max pos last time
      pos = (pos_min + pos_limit) >> 1;
    }
    else
    {
      // linear search if bisection failed
      pos = pos_min;
    }

    // clamp calculated pos into boundaries
    if( pos <= pos_min)
      pos = pos_min + 1;
    else if (pos > pos_limit)
      pos = pos_limit;
    start_pos = pos;

    // get time stamp at pos
    if (!GetTimeAtPos(&pos, &ts))
    {
      ResetParsers();
      m_WaitIFrame = true;
      return false;
    }
    pos = m_VideoBuffer->GetPosCur();

    // determine method for next calculation of pos
    if ((last_ts == ts) || (pos >= pos_max))
      no_change++;
    else
      no_change=0;

    // 0.4 sec is close enough
    if (abs(time - ts) <= 36000)
    {
      break;
    }
    // target is to the left
    else if (time <= ts)
    {
      pos_limit = start_pos - 1;
      pos_max = pos;
      ts_max = ts;
    }
    // target is to the right
    if (time >= ts)
    {
      pos_min = pos;
      ts_min = ts;
    }
    last_ts = ts;
  }

  m_VideoBuffer->SetPos(pos);

  ResetParsers();
  m_WaitIFrame = true;
  m_MuxPacketSerial++;
  return true;
}

void cDemuxer::BufferStatus(bool &timeshift, uint32_t &start, uint32_t &end)
{
  timeshift = m_VideoBuffer->HasBuffer();

  if (timeshift)
  {
    if (!m_wrapTime)
    {
      start = m_refTime;
    }
    else
    {
      start = m_endTime - (m_wrapTime - m_refTime);
    }
    end = m_endTime;
  }
  else
  {
    start = 0;
    end = 0;
  }
}

cTSStream *cDemuxer::GetFirstStream()
{
  m_StreamsIterator = m_Streams.begin();
  if (m_StreamsIterator != m_Streams.end())
    return *m_StreamsIterator;
  else
    return NULL;
}

cTSStream *cDemuxer::GetNextStream()
{
  ++m_StreamsIterator;
  if (m_StreamsIterator != m_Streams.end())
    return *m_StreamsIterator;
  else
    return NULL;
}

cTSStream *cDemuxer::FindStream(int Pid)
{
  for (std::list<cTSStream*>::iterator it = m_Streams.begin(); it != m_Streams.end(); ++it)
  {
    if (Pid == (*it)->GetPID())
      return *it;
  }
  return NULL;
}

void cDemuxer::ResetParsers()
{
  for (std::list<cTSStream*>::iterator it = m_Streams.begin(); it != m_Streams.end(); ++it)
  {
    (*it)->ResetParser();
  }
}

void cDemuxer::AddStreamInfo(sStreamInfo &stream)
{
  m_StreamInfos.push_back(stream);
}

bool cDemuxer::EnsureParsers()
{
  bool streamChange = false;

  std::list<cTSStream*>::iterator it = m_Streams.begin();
  while (it != m_Streams.end())
  {
    std::list<sStreamInfo>::iterator its;
    for (its = m_StreamInfos.begin(); its != m_StreamInfos.end(); ++its)
    {
      if ((its->pID == (*it)->GetPID()) && (its->type == (*it)->Type()))
      {
        break;
      }
    }
    if (its == m_StreamInfos.end())
    {
      XBMC->Log(LOG_INFO, "Deleting stream for pid=%i and type=%i", (*it)->GetPID(), (*it)->Type());
      m_Streams.erase(it);
      it = m_Streams.begin();
      streamChange = true;
    }
    else
      ++it;
  }

  for (std::list<sStreamInfo>::iterator it = m_StreamInfos.begin(); it != m_StreamInfos.end(); ++it)
  {
    cTSStream *stream = FindStream(it->pID);
    if (stream)
    {
      // TODO: check for change in lang
      stream->SetLanguage(it->language);
      continue;
    }

    if (it->type == stH264)
    {
      stream = new cTSStream(stH264, it->pID, &m_PtsWrap);
    }
    else if (it->type == stMPEG2VIDEO)
    {
      stream = new cTSStream(stMPEG2VIDEO, it->pID, &m_PtsWrap);
    }
    else if (it->type == stMPEG2AUDIO)
    {
      stream = new cTSStream(stMPEG2AUDIO, it->pID, &m_PtsWrap, it->handleRDS);
      stream->SetLanguage(it->language);
    }
    else if (it->type == stAACADTS)
    {
      stream = new cTSStream(stAACADTS, it->pID, &m_PtsWrap);
      stream->SetLanguage(it->language);
    }
    else if (it->type == stAACLATM)
    {
      stream = new cTSStream(stAACLATM, it->pID, &m_PtsWrap);
      stream->SetLanguage(it->language);
    }
    else if (it->type == stAC3)
    {
      stream = new cTSStream(stAC3, it->pID, &m_PtsWrap);
      stream->SetLanguage(it->language);
    }
    else if (it->type == stEAC3)
    {
      stream = new cTSStream(stEAC3, it->pID, &m_PtsWrap);
      stream->SetLanguage(it->language);
    }
    else if (it->type == stDVBSUB)
    {
      stream = new cTSStream(stDVBSUB, it->pID, &m_PtsWrap);
      stream->SetLanguage(it->language);
      stream->SetSubtitlingDescriptor(it->subtitlingType, it->compositionPageId, it->ancillaryPageId);
    }
    else if (it->type == stTELETEXT)
    {
      stream = new cTSStream(stTELETEXT, it->pID, &m_PtsWrap);
    }
    else
      continue;

    m_Streams.push_back(stream);
    XBMC->Log(LOG_INFO, "Created stream for pid=%i and type=%i", stream->GetPID(), stream->Type());
    streamChange = true;
  }
  m_StreamInfos.clear();

  return streamChange;
}

void cDemuxer::SetChannelStreams()
{
  sStreamInfo newStream;
  bool containsVideo = false;
  int index = 0;
  if (m_vpid)
  {
    newStream.pID = m_vpid;
    if (m_vtype == 0x1B)
      newStream.type = stH264;
    else
      newStream.type = stMPEG2VIDEO;

    AddStreamInfo(newStream);
    containsVideo = true;
  }

  const int *DPids = m_Dpids;
  index = 0;
  for ( ; *DPids; DPids++)
  {
    if (!FindStream(*DPids))
    {
      newStream.pID = *DPids;
      newStream.type = stAC3;
#if APIVERSNUM >= 10715
      if (m_Dtypes[index] == SI::EnhancedAC3DescriptorTag)
        newStream.type = stEAC3;
#endif
      newStream.SetLanguage(m_DLangs[index]);
      AddStreamInfo(newStream);
    }
    index++;
  }

  const int *APids = m_Apids;
  index = 0;
  for ( ; *APids; APids++)
  {
    if (!FindStream(*APids))
    {
      newStream.pID = *APids;
      newStream.type = stMPEG2AUDIO;
      if (m_Atypes[index] == 0x0F)
        newStream.type = stAACADTS;
      else if (m_Atypes[index] == 0x11)
        newStream.type = stAACLATM;
      newStream.handleRDS = newStream.type == stMPEG2AUDIO && !containsVideo ? true : false; // Relevant for RDS, if present only on mpeg 2 audio, use only if RDS is allowed
      newStream.SetLanguage(m_ALangs[index]);
      AddStreamInfo(newStream);
    }
    index++;
  }

  const int *SPids = m_Spids;
  if (SPids)
  {
    index = 0;
    for ( ; *SPids; SPids++)
    {
      if (!FindStream(*SPids))
      {
        newStream.pID = *SPids;
        newStream.type = stDVBSUB;
        newStream.SetLanguage(m_SLangs[index]);
        AddStreamInfo(newStream);
      }
      index++;
    }
  }
/*
  if (channel->Tpid())
  {
    newStream.pID = channel->Tpid();
    newStream.type = stTELETEXT;
    AddStreamInfo(newStream);
  }*/
}

void cDemuxer::SetChannelPids(cPatPmtParser *patPmtParser)
{
  m_Apids[0] = { 0 };
  m_Atypes[0] = { 0 };
  m_Dpids[0] = { 0 };
  m_Dtypes[0] = { 0 };
  m_Spids[0] = { 0 };
  int index = 0;

  const int *aPids = patPmtParser->Apids();
  index = 0;
  for ( ; *aPids; aPids++)
  {
    m_Apids[index] = patPmtParser->Apid(index);
    m_Atypes[index] = patPmtParser->Atype(index);
    strncpy(m_ALangs[index], patPmtParser->Alang(index), MAXLANGCODE2);
    index++;
  }

  const int *dPids = patPmtParser->Dpids();
  index = 0;
  for ( ; *dPids; dPids++)
  {
    m_Dpids[index] = patPmtParser->Dpid(index);
    m_Dtypes[index] = patPmtParser->Dtype(index);
    strncpy(m_DLangs[index], patPmtParser->Dlang(index), MAXLANGCODE2);
    index++;
  }

  const int *sPids = patPmtParser->Spids();
  index = 0;
  for ( ; *sPids; sPids++)
  {
    m_Spids[index] = patPmtParser->Spid(index);
    strncpy(m_SLangs[index], patPmtParser->Slang(index), MAXLANGCODE2);
    index++;
  }

  m_vpid = patPmtParser->Vpid();
  m_ppid = patPmtParser->Ppid();
  m_vtype = patPmtParser->Vtype();
//  m_tpid = m_CurrentChannel.Tpid();
}

bool cDemuxer::GetTimeAtPos(off_t *pos, int64_t *time)
{
  uint8_t *buf;
  int len;
  cTSStream *stream;
  int ts_pid;

  m_VideoBuffer->SetPos(*pos);
  ResetParsers();
  while (len = m_VideoBuffer->Read(&buf, TS_SIZE, m_endTime, m_wrapTime) == TS_SIZE)
  {
    ts_pid = TsPid(buf);
    if (stream = FindStream(ts_pid))
    {
      // only consider video or audio streams
      if ((stream->Content() == scVIDEO || stream->Content() == scAUDIO) &&
          stream->ReadTime(buf, time))
      {
        return true;
      }
    }
  }
  return false;
}

uint16_t cDemuxer::GetError()
{
  uint16_t ret = m_Error;
  m_Error = ERROR_DEMUX_NODATA;
  return ret;
}
