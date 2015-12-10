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

#include "videobuffer.h"
#include "config.h"
#include "platform/threads/threads.h"
#include "platform/util/StringUtils.h"

using namespace ADDON;
using namespace PLATFORM;

//-----------------------------------------------------------------------------

#define MARGIN 40000

class cVideoBufferTimeshift : public cVideoBuffer
{
friend class cVideoBuffer;
public:
  virtual off_t GetPosMin();
  virtual off_t GetPosMax();
  virtual off_t GetPosCur();
  virtual void GetPositions(off_t *cur, off_t *min, off_t *max);
  virtual bool HasBuffer() { return true; };

protected:
  cVideoBufferTimeshift();
  virtual bool Init() = 0;
  virtual off_t Available();
  off_t m_BufferSize;
  off_t m_WritePtr;
  off_t m_ReadPtr;
  bool m_BufferFull;
  unsigned int m_Margin;
  unsigned int m_BytesConsumed;
  PLATFORM::CMutex m_Mutex;
};

cVideoBufferTimeshift::cVideoBufferTimeshift()
{
  m_Margin = TS_SIZE*2;
  m_BufferFull = false;
  m_ReadPtr = 0;
  m_WritePtr = 0;
  m_BytesConsumed = 0;
}

off_t cVideoBufferTimeshift::GetPosMin()
{
  off_t ret;
  if (!m_BufferFull)
    return 0;

  ret = m_WritePtr + MARGIN * 2;
  if (ret >= m_BufferSize)
    ret -= m_BufferSize;

  return ret;
}

off_t cVideoBufferTimeshift::GetPosMax()
{
   off_t ret = m_WritePtr;
   if (ret < GetPosMin())
     ret += m_BufferSize;
   return ret;
}

off_t cVideoBufferTimeshift::GetPosCur()
{
  off_t ret = m_ReadPtr;
  if (ret < GetPosMin())
    ret += m_BufferSize;
  return ret;
}

void cVideoBufferTimeshift::GetPositions(off_t *cur, off_t *min, off_t *max)
{
  const CLockObject lock(m_Mutex);

  *cur = GetPosCur();
  *min = GetPosMin();
  *min = (*min > *cur) ? *cur : *min;
  *max = GetPosMax();
}

off_t cVideoBufferTimeshift::Available()
{
  const CLockObject lock(m_Mutex);

  off_t ret;
  if (m_ReadPtr <= m_WritePtr)
    ret = m_WritePtr - m_ReadPtr;
  else
    ret = m_BufferSize - (m_ReadPtr - m_WritePtr);

  return ret;
}

//-----------------------------------------------------------------------------

class cVideoBufferFile : public cVideoBufferTimeshift
{
friend class cVideoBuffer;
public:
  virtual off_t GetPosMax();
  virtual int ReadBlock(uint8_t **buf, unsigned int size, time_t &endTime, time_t &wrapTime);
  virtual void SetPos(off_t pos);

protected:
  cVideoBufferFile();
  virtual ~cVideoBufferFile();
  virtual bool Init();
  virtual int ReadBytes(uint8_t *buf, off_t pos, unsigned int size);
  int m_ClientID;
  std::string m_Filename;
  uint8_t *m_ReadCache;
  unsigned int m_ReadCachePtr;
  unsigned int m_ReadCacheSize;
  unsigned int m_ReadCacheMaxSize;
  void *m_file;
};

cVideoBufferFile::cVideoBufferFile()
  : m_file(nullptr)
{

}

cVideoBufferFile::~cVideoBufferFile()
{
  if (m_file)
  {
    XBMC->CloseFile(m_file);
    m_file = nullptr;
  }
  if (m_ReadCache)
    free(m_ReadCache);
}

bool cVideoBufferFile::Init()
{
  return true;
}

void cVideoBufferFile::SetPos(off_t pos)
{
  const CLockObject lock(m_Mutex);

  m_ReadPtr = pos;
  if (m_ReadPtr >= m_BufferSize)
    m_ReadPtr -= m_BufferSize;
  m_BytesConsumed = 0;
  m_ReadCacheSize = 0;
}

off_t cVideoBufferFile::GetPosMax()
{
  off_t posMax = cVideoBufferTimeshift::GetPosMax();
  if (posMax >= m_ReadCacheMaxSize)
    posMax -= m_ReadCacheMaxSize;
  else
    posMax = 0;
  return posMax;
}

int cVideoBufferFile::ReadBytes(uint8_t *buf, off_t pos, unsigned int size)
{
  ssize_t p = 0;
  XBMC->SeekFile(m_file, pos, 0);
  while (p < size)
  {
    p += XBMC->ReadFile(m_file, buf, size);
  }
  return p;
}

int cVideoBufferFile::ReadBlock(uint8_t **buf, unsigned int size, time_t &endTime, time_t &wrapTime)
{
  // move read pointer
  if (m_BytesConsumed)
  {
    const CLockObject lock(m_Mutex);
    m_ReadPtr += m_BytesConsumed;
    if (m_ReadPtr >= m_BufferSize)
      m_ReadPtr -= m_BufferSize;
    m_ReadCachePtr += m_BytesConsumed;

    endTime = m_bufferEndTime;
    wrapTime = m_bufferWrapTime;
  }
  m_BytesConsumed = 0;

  // check if we have anything to read
  off_t readBytes;
  if (m_ReadCacheSize && ((m_ReadCachePtr + m_Margin) <= m_ReadCacheSize))
  {
    readBytes = m_ReadCacheSize - m_ReadCachePtr;
    *buf = m_ReadCache + m_ReadCachePtr;
  }
  else if ((readBytes = Available()) >= m_ReadCacheMaxSize)
  {
    if (m_ReadPtr + m_ReadCacheMaxSize <= m_BufferSize)
    {
      m_ReadCacheSize = ReadBytes(m_ReadCache, m_ReadPtr, m_ReadCacheMaxSize);
      if (m_ReadCacheSize < 0)
      {
        XBMC->Log(LOG_ERROR, "Could not read file: %s", m_Filename.c_str());
        return 0;
      }
      if (m_ReadCacheSize < m_Margin)
      {
        XBMC->Log(LOG_ERROR, "Could not read file (margin): %s , read: %d", m_Filename.c_str(), m_ReadCacheSize);
        m_ReadCacheSize = 0;
        return 0;
      }
      readBytes = m_ReadCacheSize;
      *buf = m_ReadCache;
      m_ReadCachePtr = 0;
    }
    else
    {
      m_ReadCacheSize = ReadBytes(m_ReadCache, m_ReadPtr, m_BufferSize - m_ReadPtr);
      if ((m_ReadCacheSize < m_Margin) && (m_ReadCacheSize != (m_BufferSize - m_ReadPtr)))
      {
        XBMC->Log(LOG_ERROR, "Could not read file (end): %s", m_Filename.c_str());
        m_ReadCacheSize = 0;
        return 0;
      }
      readBytes = ReadBytes(m_ReadCache + m_ReadCacheSize, 0, m_ReadCacheMaxSize - m_ReadCacheSize);
      if (readBytes < 0)
      {
        XBMC->Log(LOG_ERROR, "Could not read file (end): %s", m_Filename.c_str());
        m_ReadCacheSize = 0;
        return 0;
      }
      m_ReadCacheSize += readBytes;
      if (m_ReadCacheSize < m_Margin)
      {
        XBMC->Log(LOG_ERROR, "Could not read file (margin): %s", m_Filename.c_str());
        m_ReadCacheSize = 0;
        return 0;
      }
      readBytes = m_ReadCacheSize;
      *buf = m_ReadCache;
      m_ReadCachePtr = 0;
    }
  }
  else
    return 0;

  // Make sure we are looking at a TS packet
  while (readBytes > TS_SIZE)
  {
    if ((*buf)[0] == TS_SYNC_BYTE && (*buf)[TS_SIZE] == TS_SYNC_BYTE)
      break;
    m_BytesConsumed++;
    (*buf)++;
    readBytes--;
  }

  if ((*buf)[0] != TS_SYNC_BYTE)
  {
    return 0;
  }

  m_BytesConsumed += TS_SIZE;
  return TS_SIZE;
}

//-----------------------------------------------------------------------------

class cVideoBufferTest : public cVideoBufferFile
{
friend class cVideoBuffer;
public:
  virtual off_t GetPosMax();

protected:
  cVideoBufferTest(std::string filename);
  virtual ~cVideoBufferTest();
  virtual bool Init();
  virtual off_t Available();
  off_t GetPosEnd();
};

cVideoBufferTest::cVideoBufferTest(std::string filename)
{
  m_Filename = filename;
  m_ReadCacheSize = 0;
  m_file = nullptr;
}

cVideoBufferTest::~cVideoBufferTest()
{
  if (m_file)
  {
    XBMC->CloseFile(m_file);
    m_file = nullptr;
  }
}

off_t cVideoBufferTest::GetPosMax()
{
  m_WritePtr = GetPosEnd();
  return cVideoBufferTimeshift::GetPosMax();
}

off_t cVideoBufferTest::GetPosEnd()
{
  return XBMC->GetFileLength(m_file);
}

bool cVideoBufferTest::Init()
{
  m_ReadCache = 0;
  m_ReadCacheMaxSize = 8000;

  m_ReadCache = (uint8_t*)malloc(m_ReadCacheMaxSize);
  if (!m_ReadCache)
    return false;

  m_file = XBMC->OpenFile(m_Filename.c_str(), 0);
  if (m_file == nullptr)
  {
    XBMC->Log(LOG_ERROR, "Could not open file: %s", m_Filename.c_str());
    return false;
  }

  m_WritePtr = 0;
  m_ReadPtr = 0;
  m_ReadCacheSize = 0;
  m_InputAttached = false;
  return true;
}

off_t cVideoBufferTest::Available()
{
  m_BufferSize = m_WritePtr = GetPosEnd();
  return cVideoBufferTimeshift::Available();
}

//-----------------------------------------------------------------------------

cVideoBuffer::cVideoBuffer()
{
  m_CheckEof = false;
  m_InputAttached = true;
  m_bufferEndTime = 0;
  m_bufferWrapTime = 0;
}

cVideoBuffer::~cVideoBuffer()
{
}

cVideoBuffer* cVideoBuffer::Create(std::string filename)
{
  XBMC->Log(LOG_INFO, "Open recording: %s", filename.c_str());
  cVideoBufferTest *buffer = new cVideoBufferTest(filename);
  if (!buffer->Init())
  {
    delete buffer;
    return NULL;
  }
  else
    return buffer;
}

int cVideoBuffer::Read(uint8_t **buf, unsigned int size, time_t &endTime, time_t &wrapTime)
{
  int count = ReadBlock(buf, size, endTime, wrapTime);

  // check for end of file
  if (!m_InputAttached && count != TS_SIZE)
  {
    if (m_CheckEof && m_Timer.TimeLeft() == 0)
    {
      XBMC->Log(LOG_INFO, "Recoding - end of file");
      return -2;
    }
    else if (!m_CheckEof)
    {
      m_CheckEof = true;
      m_Timer.Init(3000);
    }
  }
  else
    m_CheckEof = false;

  return count;
}

void cVideoBuffer::AttachInput(bool attach)
{
  m_InputAttached = attach;
}

time_t cVideoBuffer::GetRefTime()
{
  time_t t;
  time(&t);
  return t;
}
