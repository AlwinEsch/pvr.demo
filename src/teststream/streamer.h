#pragma once
/*
 *      Copyright (C) 2010 Alwin Esch (Team XBMC)
 *      Copyright (C) 2010, 2011 Alexander Pipelka
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

#include "parser.h"
#include "demuxer.h"

#include "platform/threads/mutex.h"
#include "kodi/xbmc_stream_utils.hpp"

#include <vector>
#include <list>

class cVideoBuffer;
class PVRDemoChannel;

class cLiveStreamer
{
private:
  friend class cParser;

  void sendStreamChange();
  void sendStreamStatus();

  uint32_t           m_scanTimeout;                  /*!> Channel scanning timeout (in seconds) */
  PLATFORM::CTimeout m_last_tick;
  cDemuxer           m_Demuxer;
  cVideoBuffer      *m_VideoBuffer;

protected:
  bool Open(int serial = -1);

public:
  cLiveStreamer();
  virtual ~cLiveStreamer();

  bool OpenChannel(const PVR_CHANNEL &channelinfo, PVRDemoChannel *currentChannel);
  void Close();
  void Abort();
  bool GetStreamProperties(PVR_STREAM_PROPERTIES* props);
  DemuxPacket* Read();
  time_t GetPlayingTime();

private:
  std::string                 m_streamFile;
  ADDON::XbmcStreamProperties m_streams;
  PVR_CHANNEL                 m_channelinfo;
  time_t                      m_ReferenceTime;
  double                      m_ReferenceDTS;
  double                      m_CurrentDTS;
  sStreamPacket               m_pkt_data;
  sStreamPacket               m_pkt_side_data; // Additional data
  DemuxPacket*                m_RDSPacket;
  bool                        m_requestChange;
};
