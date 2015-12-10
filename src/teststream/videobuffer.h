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

#include <stdint.h>
#include <stdlib.h>
#include <string>

#include "platform/util/timeutils.h"

class cRecording;

class cVideoBuffer
{
public:
  virtual ~cVideoBuffer();
  static cVideoBuffer* Create(std::string filename);

  virtual int ReadBlock(uint8_t **buf, unsigned int size, time_t &endTime, time_t &wrapTime) = 0;
  virtual off_t GetPosMin() { return 0; };
  virtual off_t GetPosMax() { return 0; };
  virtual off_t GetPosCur() { return 0; };
  virtual void GetPositions(off_t *cur, off_t *min, off_t *max) {};
  virtual void SetPos(off_t pos) {};
  virtual void SetCache(bool on) {};
  virtual bool HasBuffer() { return false; };
  virtual time_t GetRefTime();
  int Read(uint8_t **buf, unsigned int size, time_t &endTime, time_t &wrapTime);
  void AttachInput(bool attach);
protected:
  cVideoBuffer();
  PLATFORM::CTimeout m_Timer;
  bool m_CheckEof;
  bool m_InputAttached;
  time_t m_bufferEndTime;
  time_t m_bufferWrapTime;
};
