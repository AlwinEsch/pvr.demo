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

#include "config.h"
#include "streamer.h"
#include "videobuffer.h"
#include "PVRDemoData.h"

#include "xbmc_codec_descriptor.hpp"
#include "kodi/xbmc_stream_utils.hpp"

#include <stdlib.h>
#include <time.h>
#include <p8-platform/util/StringUtils.h>

using namespace ADDON;
using namespace P8PLATFORM;

cLiveStreamer::cLiveStreamer()
 : m_scanTimeout(10)
{
  m_VideoBuffer     = NULL;
  m_RDSPacket       = NULL;
  m_requestChange   = false;
}

cLiveStreamer::~cLiveStreamer()
{
  Close();
}

bool cLiveStreamer::OpenChannel(const PVR_CHANNEL &channelinfo, PVRDemoChannel *currentChannel)
{
  XBMC->Log(LOG_DEBUG, "changing to channel %d", channelinfo.iChannelNumber);

  m_streamFile = currentChannel->strStreamFile;
  if (m_streamFile == "")
  {
    XBMC->Log(LOG_ERROR, "%s - No local stream file in 'PVRDemoAddonSettings.xml' defined!", __FUNCTION__);
    return false;
  }

  if (StringUtils::StartsWith(m_streamFile, "special://xbmc/addons/pvr.demo"))
    StringUtils::Replace(m_streamFile, "special://xbmc/addons/pvr.demo", g_strClientPath);
  else if (StringUtils::StartsWith(m_streamFile, "special://home/addons/pvr.demo"))
    StringUtils::Replace(m_streamFile, "special://home/addons/pvr.demo", g_strUserPath);

  if (!Open(0))
  {
    XBMC->Log(LOG_ERROR, "%s - failed to set channel", __FUNCTION__);
    return false;
  }

  m_channelinfo = channelinfo;
  m_streams.Clear();
  m_ReferenceTime = 0;
  m_requestChange = true;

  return true;
}

bool cLiveStreamer::GetStreamProperties(PVR_STREAM_PROPERTIES* props)
{
  return m_streams.GetProperties(props);
}

void cLiveStreamer::Abort()
{
  m_streams.Clear();
}

DemuxPacket* cLiveStreamer::Read()
{
  unsigned int iSize = 0;
  DemuxPacket* pPacket = NULL;

  if (m_RDSPacket)
  {
    pPacket = m_RDSPacket;
    m_RDSPacket = NULL;
    return pPacket;
  }

  int ret = m_Demuxer.Read(&m_pkt_data, &m_pkt_side_data);
  if (ret > 0)
  {
    // Process normal data if present
    if (m_pkt_data.data)
    {
      if (m_pkt_data.pmtChange)
        m_requestChange = true;

      // If some additional data is present inside the stream, it is written there
      // (currently RDS inside MPEG2-Audio) and stored for the next call to send
      // before new data becomes loaded,
      if (m_pkt_side_data.data)
      {
        int iStreamId = m_streams.GetStreamId(m_pkt_side_data.id);

        // stream found ?
        if(iStreamId != -1 && m_pkt_side_data.serial == m_Demuxer.GetSerial())
        {
          m_RDSPacket = PVR->AllocateDemuxPacket(m_pkt_side_data.size);
          if (m_RDSPacket)
          {
            uint8_t *data = (uint8_t *)m_RDSPacket->pData;
            for (unsigned int i = 0; i < m_pkt_side_data.size; ++i)
              data[i] = m_pkt_side_data.data[i];

            m_RDSPacket->iStreamId  = iStreamId;
            m_RDSPacket->iSize      = m_pkt_side_data.size;
            m_RDSPacket->dts        = (double)m_pkt_side_data.pts * DVD_TIME_BASE / 1000000;
            m_RDSPacket->pts        = (double)m_pkt_side_data.dts * DVD_TIME_BASE / 1000000;
            m_RDSPacket->duration   = (double)m_pkt_data.duration * DVD_TIME_BASE / 1000000;
          }
        }
        m_pkt_side_data.data = NULL;
      }

      if (m_pkt_data.streamChange || m_requestChange)
      {
        sendStreamChange();
        m_requestChange = false;
        pPacket = PVR->AllocateDemuxPacket(0);
        pPacket->iStreamId = DMX_SPECIALID_STREAMCHANGE;
        return pPacket;
      }
      if (m_pkt_data.reftime)
      {
        m_ReferenceTime = m_pkt_data.reftime;
        m_ReferenceDTS = (double)m_pkt_data.pts * DVD_TIME_BASE / 1000000;

        m_pkt_data.reftime = 0;
      }

      int iStreamId = m_streams.GetStreamId(m_pkt_data.id);

      // stream found ?
      if(iStreamId != -1 && m_pkt_data.serial == m_Demuxer.GetSerial())
      {
        pPacket = PVR->AllocateDemuxPacket(m_pkt_data.size);
        if (!pPacket)
          return NULL;
        uint8_t *data = (uint8_t *)pPacket->pData;
        for (unsigned int i = 0; i < m_pkt_data.size; ++i)
          data[i] = m_pkt_data.data[i];
        pPacket->iStreamId  = iStreamId;
        pPacket->iSize      = m_pkt_data.size;
        pPacket->dts        = (double)m_pkt_data.pts * DVD_TIME_BASE / 1000000;
        pPacket->pts        = (double)m_pkt_data.dts * DVD_TIME_BASE / 1000000;
        pPacket->duration   = (double)m_pkt_data.duration * DVD_TIME_BASE / 1000000;

        XbmcPvrStream *stream = m_streams.GetStreamById(m_pkt_data.id);
        xbmc_codec_type_t type = XBMC_CODEC_TYPE_UNKNOWN;
        if (stream != NULL)
          type = stream->iCodecType;
        if (type == XBMC_CODEC_TYPE_VIDEO || type == XBMC_CODEC_TYPE_AUDIO)
        {
          if (pPacket->dts != DVD_NOPTS_VALUE)
            m_CurrentDTS = pPacket->dts;
          else if (pPacket->pts != DVD_NOPTS_VALUE)
            m_CurrentDTS = pPacket->pts;
        }

        return pPacket;
      }
    }
    else if (ret == -1)
    {
      // no data
      CEvent::Sleep(1000);

      if(m_last_tick.TimeLeft() == 0)
      {
        sendStreamStatus();
        m_last_tick.Init(m_scanTimeout*1000);
      }
    }
    else if (ret == -2)
    {
      if (!Open(m_Demuxer.GetSerial()))
        return NULL;
    }
  }

  return PVR->AllocateDemuxPacket(0);
}

time_t cLiveStreamer::GetPlayingTime()
{
  time_t ret = 0;
  if (m_ReferenceTime)
    ret = m_ReferenceTime + (m_CurrentDTS - m_ReferenceDTS) / DVD_TIME_BASE;
  return ret;
}

bool cLiveStreamer::Open(int serial)
{
  Close();

  m_VideoBuffer = cVideoBuffer::Create(m_streamFile);
  if (!m_VideoBuffer)
    return false;

  m_Demuxer.Open(m_VideoBuffer);
  if (serial >= 0)
    m_Demuxer.SetSerial(serial);

  return true;
}

void cLiveStreamer::Close(void)
{
  XBMC->Log(LOG_INFO, "LiveStreamer::Close - close");

  m_Demuxer.Close();
  if (m_VideoBuffer)
  {
    delete m_VideoBuffer;
    m_VideoBuffer = NULL;
  }
  if (m_RDSPacket)
  {
    PVR->FreeDemuxPacket(m_RDSPacket);
    m_RDSPacket = NULL;
  }
}

void cLiveStreamer::sendStreamChange()
{
  std::vector<XbmcPvrStream> newStreams;

  uint32_t FpsScale, FpsRate, Height, Width;
  double Aspect;
  int Channels, SampleRate, BitRate, BitsPerSample, BlockAlign;
  for (cTSStream* stream = m_Demuxer.GetFirstStream(); stream; stream = m_Demuxer.GetNextStream())
  {
    uint32_t pid = stream->GetPID();

    XbmcPvrStream newStream;
    m_streams.GetStreamData(pid, &newStream);

    const char *language = stream->GetLanguage();
    newStream.strLanguage[0]  = language[0];
    newStream.strLanguage[1]  = language[1];
    newStream.strLanguage[2]  = language[2];
    newStream.strLanguage[3]  = 0;
    newStream.iIdentifier     = -1;
    newStream.iPhysicalId     = pid;

    if (stream->Type() == stMPEG2AUDIO)
    {
      CodecDescriptor codecId = CodecDescriptor::GetCodecByName("MPEG2AUDIO");
      if (codecId.Codec().codec_type == XBMC_CODEC_TYPE_UNKNOWN)
        continue;

      newStream.iCodecType      = codecId.Codec().codec_type;
      newStream.iCodecId        = codecId.Codec().codec_id;

      stream->GetAudioInformation(newStream.iChannels, newStream.iSampleRate, newStream.iBitRate, newStream.iBitsPerSample, newStream.iBlockAlign);

      for (unsigned int i = 0; i < stream->GetSideDataTypes()->size(); i++)
      {
        uint32_t pid = stream->GetSideDataTypes()->at(i).first;

        XbmcPvrStream newSideStream;
        m_streams.GetStreamData(pid, &newSideStream);

        if (stream->GetSideDataTypes()->at(i).second == scRDS)
        {
          CodecDescriptor codecId = CodecDescriptor::GetCodecByName("RDS");
          if (codecId.Codec().codec_type == XBMC_CODEC_TYPE_UNKNOWN)
            continue;

          newSideStream.iPhysicalId     = pid;
          newSideStream.iCodecType      = codecId.Codec().codec_type;
          newSideStream.iCodecId        = codecId.Codec().codec_id;

          newStreams.push_back(newSideStream);
        }
      }
    }
    else if (stream->Type() == stMPEG2VIDEO)
    {
      CodecDescriptor codecId = CodecDescriptor::GetCodecByName("MPEG2VIDEO");
      if (codecId.Codec().codec_type == XBMC_CODEC_TYPE_UNKNOWN)
        continue;

      newStream.iCodecType      = codecId.Codec().codec_type;
      newStream.iCodecId        = codecId.Codec().codec_id;

      stream->GetVideoInformation(newStream.iFPSScale, newStream.iFPSRate, newStream.iHeight, newStream.iWidth, newStream.fAspect);

      newStream.strLanguage[0]  = 0;
      newStream.strLanguage[1]  = 0;
      newStream.strLanguage[2]  = 0;
      newStream.strLanguage[3]  = 0;
    }
    else if (stream->Type() == stAC3)
    {
      CodecDescriptor codecId = CodecDescriptor::GetCodecByName("AC3");
      if (codecId.Codec().codec_type == XBMC_CODEC_TYPE_UNKNOWN)
        continue;

      newStream.iCodecType      = codecId.Codec().codec_type;
      newStream.iCodecId        = codecId.Codec().codec_id;

      stream->GetAudioInformation(newStream.iChannels, newStream.iSampleRate, newStream.iBitRate, newStream.iBitsPerSample, newStream.iBlockAlign);
    }
    else if (stream->Type() == stH264)
    {
      CodecDescriptor codecId = CodecDescriptor::GetCodecByName("H264");
      if (codecId.Codec().codec_type == XBMC_CODEC_TYPE_UNKNOWN)
        continue;

      newStream.iCodecType      = codecId.Codec().codec_type;
      newStream.iCodecId        = codecId.Codec().codec_id;

      stream->GetVideoInformation(newStream.iFPSScale, newStream.iFPSRate, newStream.iHeight, newStream.iWidth, newStream.fAspect);

      newStream.strLanguage[0]  = 0;
      newStream.strLanguage[1]  = 0;
      newStream.strLanguage[2]  = 0;
      newStream.strLanguage[3]  = 0;
    }
    else if (stream->Type() == stDVBSUB)
    {
      CodecDescriptor codecId = CodecDescriptor::GetCodecByName("DVBSUB");
      if (codecId.Codec().codec_type == XBMC_CODEC_TYPE_UNKNOWN)
        continue;

      newStream.iCodecType      = codecId.Codec().codec_type;
      newStream.iCodecId        = codecId.Codec().codec_id;
      newStream.iIdentifier     = (stream->CompositionPageId() & 0xffff) | ((stream->AncillaryPageId() & 0xffff) << 16);
    }
    else if (stream->Type() == stTELETEXT)
    {
      CodecDescriptor codecId = CodecDescriptor::GetCodecByName("TELETEXT");
      if (codecId.Codec().codec_type == XBMC_CODEC_TYPE_UNKNOWN)
        continue;

      newStream.iCodecType      = codecId.Codec().codec_type;
      newStream.iCodecId        = codecId.Codec().codec_id;
      newStream.iIdentifier     = (stream->CompositionPageId() & 0xffff) | ((stream->AncillaryPageId() & 0xffff) << 16);
    }
    else if (stream->Type() == stAACADTS)
    {
      CodecDescriptor codecId = CodecDescriptor::GetCodecByName("AAC");
      if (codecId.Codec().codec_type == XBMC_CODEC_TYPE_UNKNOWN)
        continue;

      newStream.iCodecType      = codecId.Codec().codec_type;
      newStream.iCodecId        = codecId.Codec().codec_id;

      stream->GetAudioInformation(newStream.iChannels, newStream.iSampleRate, newStream.iBitRate, newStream.iBitsPerSample, newStream.iBlockAlign);
    }
    else if (stream->Type() == stAACLATM)
    {
      CodecDescriptor codecId = CodecDescriptor::GetCodecByName("AAC_LATM");
      if (codecId.Codec().codec_type == XBMC_CODEC_TYPE_UNKNOWN)
        continue;

      newStream.iCodecType      = codecId.Codec().codec_type;
      newStream.iCodecId        = codecId.Codec().codec_id;

      stream->GetAudioInformation(newStream.iChannels, newStream.iSampleRate, newStream.iBitRate, newStream.iBitsPerSample, newStream.iBlockAlign);
    }
    else if (stream->Type() == stEAC3)
    {
      CodecDescriptor codecId = CodecDescriptor::GetCodecByName("EAC3");
      if (codecId.Codec().codec_type == XBMC_CODEC_TYPE_UNKNOWN)
        continue;

      newStream.iCodecType      = codecId.Codec().codec_type;
      newStream.iCodecId        = codecId.Codec().codec_id;

      stream->GetAudioInformation(newStream.iChannels, newStream.iSampleRate, newStream.iBitRate, newStream.iBitsPerSample, newStream.iBlockAlign);
    }
    else if (stream->Type() == stDTS)
    {
      CodecDescriptor codecId = CodecDescriptor::GetCodecByName("DTS");
      if (codecId.Codec().codec_type == XBMC_CODEC_TYPE_UNKNOWN)
        continue;

      newStream.iCodecType      = codecId.Codec().codec_type;
      newStream.iCodecId        = codecId.Codec().codec_id;

      stream->GetAudioInformation(newStream.iChannels, newStream.iSampleRate, newStream.iBitRate, newStream.iBitsPerSample, newStream.iBlockAlign);
    }

    newStreams.push_back(newStream);
  }

  m_streams.UpdateStreams(newStreams);
}

void cLiveStreamer::sendStreamStatus()
{
  uint16_t error = m_Demuxer.GetError();
  if (error & ERROR_PES_SCRAMBLE)
  {
    XBMC->Log(LOG_INFO, "Channel: scrambled %d", error);
    XBMC->QueueNotification(QUEUE_INFO, "Channel: scrambled (%d)", error);
  }
  else if (error & ERROR_PES_STARTCODE)
  {
    XBMC->Log(LOG_INFO, "Channel: startcode %d", error);
    XBMC->QueueNotification(QUEUE_INFO, "Channel: encrypted? (%d)", error);
  }
  else if (error & ERROR_DEMUX_NODATA)
  {
    XBMC->Log(LOG_INFO, "Channel: no data %d", error);
    XBMC->QueueNotification(QUEUE_INFO, "Channel: no data");
  }
  else
  {
    XBMC->Log(LOG_INFO, "Channel: unknown error %d", error);
    XBMC->QueueNotification(QUEUE_INFO, "Channel: unknown error (%d)", error);
  }
}
