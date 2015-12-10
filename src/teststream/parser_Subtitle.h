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

#include "parser.h"

// --- cParserSubtitle -------------------------------------------------

class cParserSubtitle : public cParser
{
public:
  cParserSubtitle(int pID, cTSStream *stream, sPtsWrap *ptsWrap, bool observePtsWraps);
  virtual ~cParserSubtitle();

  virtual void Parse(sStreamPacket *pkt, sStreamPacket *pkt_side_data);
};
