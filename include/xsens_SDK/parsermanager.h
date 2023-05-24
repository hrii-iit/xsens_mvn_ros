/*! \file
	\section FileCopyright Copyright Notice
	This is free and unencumbered software released into the public domain.

	Anyone is free to copy, modify, publish, use, compile, sell, or
	distribute this software, either in source code form or as a compiled
	binary, for any purpose, commercial or non-commercial, and by any
	means.

	In jurisdictions that recognize copyright laws, the author or authors
	of this software dedicate any and all copyright interest in the
	software to the public domain. We make this dedication for the benefit
	of the public at large and to the detriment of our heirs and
	successors. We intend this dedication to be an overt act of
	relinquishment in perpetuity of all present and future rights to this
	software under copyright law.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
	EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
	MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
	IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
	ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
	OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef PARSERMANAGER_H
#define PARSERMANAGER_H

#include "datagram.h"
#include "eulerdatagram.h"
#include "scaledatagram.h"
#include "metadatagram.h"
#include "quaterniondatagram.h"
#include "angularsegmentkinematicsdatagram.h"
#include "centerofmassdatagram.h"
#include "jointanglesdatagram.h"
#include "linearsegmentkinematicsdatagram.h"
#include "positiondatagram.h"
#include "timecodedatagram.h"
#include "trackerkinematicsdatagram.h"

class ParserManager
{
public:
  ParserManager(bool printData = false, bool printHeader = false);
  ~ParserManager();
  void readDatagram(const char* data);

  AngularSegmentKinematicsDatagram* getAngularSegmentKinematicsDatagram();
  //CenterOfMassDatagram* getCenterOfMassDatagram() {return NULL;};

  CenterOfMassDatagram* getCenterOfMassDatagram();

  EulerDatagram* getEulerDatagram() {return NULL;};
  JointAnglesDatagram* getJointAnglesDatagram();
  LinearSegmentKinematicsDatagram* getLinearSegmentKinematicsDatagram();
  MetaDatagram* getMetaDatagram() {return NULL;};
  PositionDatagram* getPositionDatagram() {return NULL;};
  QuaternionDatagram* getQuaternionDatagram();
  ScaleDatagram* getScaleDatagram() {return NULL;};
//  TimeCodeDatagram* getTimeCodeDatagram() {return NULL;};

  TimeCodeDatagram* getTimeCodeDatagram();
  TrackerKinematicsDatagram* getTrackerKinematicsDatagram() {return NULL;};

private:
  Datagram* createDgram(StreamingProtocol proto);
  bool m_printdata;   // Print or not the data of the received datagram
  bool m_printheader; // Print or not the header of the received datagram

  Datagram* m_datagram;
  StreamingProtocol m_type;
};

#endif
