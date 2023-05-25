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

#include "xsens_SDK/parsermanager.h"


ParserManager::ParserManager(bool printData /*=false*/, bool printHeader /*=false*/) : m_printdata(false), m_printheader(false), m_datagram(NULL)
{
  if (printData)
    m_printdata = true;

  if (printHeader)
    m_printheader = true;

}

/*! Destructor */
ParserManager::~ParserManager()
{
   if (m_datagram != NULL)
      delete m_datagram;
}

Datagram* ParserManager::createDgram(StreamingProtocol proto)
{
  switch (proto)
  {
  case SPPoseEuler:   return new EulerDatagram;
  case SPPoseQuaternion:return new QuaternionDatagram;
  case SPPosePositions: return new PositionDatagram;
  case SPMetaScaling:   return new ScaleDatagram;
  case SPMetaMoreMeta:  return new MetaDatagram;
  case SPJointAngles:   return new JointAnglesDatagram;
  case SPLinearSegmentKinematics:   return new LinearSegmentKinematicsDatagram;
  case SPAngularSegmentKinematics:  return new AngularSegmentKinematicsDatagram;
  case SPTrackerKinematics:   return new TrackerKinematicsDatagram;
  case SPCenterOfMass:        return new CenterOfMassDatagram;
  case SPTimeCode:            return new TimeCodeDatagram;

  default:
    return NULL;
  }
}

/*! Read single datagram from the incoming stream */
void ParserManager::readDatagram(const char* data)
{
  m_type = static_cast<StreamingProtocol>(Datagram::messageType(data));

  if (m_datagram != NULL) {
     delete m_datagram;
     m_datagram = NULL;
  }

  m_datagram = createDgram(m_type);

  if (m_datagram != NULL)
  {
    m_datagram->deserialize(data);

    if (m_printheader) {
      m_datagram->printHeader();
    }
    if (m_printdata) {
      m_datagram->printData();
    }
  }
}

CenterOfMassDatagram* ParserManager::getCenterOfMassDatagram()
{
    if (m_type == SPCenterOfMass) return static_cast<CenterOfMassDatagram*>(m_datagram);
    return NULL;
}


TimeCodeDatagram* ParserManager::getTimeCodeDatagram() {
   if (m_type == SPTimeCode) return static_cast<TimeCodeDatagram*>(m_datagram);
   return NULL;
}

/* Return JointAnglesDatagram else return null */
JointAnglesDatagram* ParserManager::getJointAnglesDatagram() {
   if (m_type == SPJointAngles) return static_cast<JointAnglesDatagram*>(m_datagram);
   return NULL;
}

/* Return QuaternionDatagram else return null */
QuaternionDatagram* ParserManager::getQuaternionDatagram() {
  if (m_type == SPPoseQuaternion) return static_cast<QuaternionDatagram*>(m_datagram);
  return NULL;
}

/* Return AngularSegmentKinematicsDatagram else return null */
AngularSegmentKinematicsDatagram* ParserManager::getAngularSegmentKinematicsDatagram() {
   if (m_type == SPAngularSegmentKinematics) return static_cast<AngularSegmentKinematicsDatagram*>(m_datagram);
   return NULL;
}

/* Return LinearSegmentKinematicsDatagram else return null */
LinearSegmentKinematicsDatagram *ParserManager::getLinearSegmentKinematicsDatagram() {
  if (m_type == SPLinearSegmentKinematics) return static_cast<LinearSegmentKinematicsDatagram*>(m_datagram);
  return NULL;
}
