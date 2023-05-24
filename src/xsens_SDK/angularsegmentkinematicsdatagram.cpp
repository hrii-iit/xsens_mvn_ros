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

#include "xsens_SDK/angularsegmentkinematicsdatagram.h"

/*! \class AngularSegmentKinematicsDatagram
	\brief a Angular Kinematics datagram (type 0x22)

	Information about each segment is sent as follows.

	4 bytes segment ID, in the range 1-30
	4 bytes q1 segment orientation quaternion component 1 (re)
	4 bytes q2 segment orientation quaternion component 1 (i)
	4 bytes q3 segment orientation quaternion component 1 (j)
	4 bytes q4 segment orientation quaternion component 1 (k)
	4 bytes angular velocity around segment x axis
	4 bytes angular velocity around segment y axis
	4 bytes angular velocity around segment z axis
	4 bytes angular acceleration around segment x axis
	4 bytes angular acceleration around segment y axis
	4 bytes angular acceleration around segment z axis

	Total: 44 bytes per segment

	The coordinates use a Z-Up, right-handed coordinate system.
*/

/*! Constructor */
AngularSegmentKinematicsDatagram::AngularSegmentKinematicsDatagram()
	: Datagram()
{
	setType(SPAngularSegmentKinematics);
}

/*! Destructor */
AngularSegmentKinematicsDatagram::~AngularSegmentKinematicsDatagram()
{
}

/*! Deserialize the data from \a arr
	\sa serializeData
*/
void AngularSegmentKinematicsDatagram::deserializeData(Streamer &inputStreamer)
{
	Streamer* streamer = &inputStreamer;

	for (int i = 0; i < dataCount(); i++)
	{
		angularSegmentKinematics kin;

		// Store the segement Id -> 4 byte 
		streamer->read(kin.segmentId);

		// Store the Segment orientation in a Vector -> 16 byte	(4 x 4 byte)
		for (int k = 0; k < 4; k++)
			streamer->read(kin.segmentOrien[k]);
		// trasform in degrees
		for (int k = 0; k < 4; k++)
			kin.segmentOrien[k] = rad2Deg(kin.segmentOrien[k]);

		// Store the Angular Velocity in a Vector -> 12 byte	(3 x 4 byte)
		for (int k = 0; k < 3; k++)
			streamer->read(kin.angularVeloc[k]);
		// trasform in degrees
		for (int k = 0; k < 3; k++)
			kin.angularVeloc[k] = rad2Deg(kin.angularVeloc[k]);

		// Store the Angular Acceleration in a Vector -> 12 byte	(3 x 4 byte)
		for (int k = 0; k < 3; k++)
			streamer->read(kin.angularAccel[k]);
		// trasform in degrees
		for (int k = 0; k < 3; k++)
			kin.angularAccel[k] = rad2Deg(kin.angularAccel[k]);

		m_data.push_back(kin);
	}
}

/*! Print Data datagram in a formated why
*/
void AngularSegmentKinematicsDatagram::printData() const
{
	std::cout << "*********************** DATA CONTENT ***********************" <<  std::endl <<  std::endl;

	for (int i = 0; i < m_data.size(); i++)
	{
		std::cout << "Segment ID: " << m_data.at(i).segmentId << std::endl;
		// Segment orientation quaternion 
		std::cout << "Segment orientation: " << "(";
		std::cout << "re: " << m_data.at(i).segmentOrien[0] << ", ";
		std::cout << "i: " << m_data.at(i).segmentOrien[1] << ", ";
		std::cout << "j: " << m_data.at(i).segmentOrien[1] << ", ";
		std::cout << "k: " << m_data.at(i).segmentOrien[2] << ")"<< std::endl;

		// Angular Velocity
		std::cout << "Angular velocity: " << "(";
		std::cout << "x: " << m_data.at(i).angularVeloc[0] << ", ";
		std::cout << "y: " << m_data.at(i).angularVeloc[1] << ", ";
		std::cout << "z: " << m_data.at(i).angularVeloc[2] << ")"<< std::endl;

		// Angular acceleration
		std::cout << "Angular acceleration: " << "(";
		std::cout << "x: " << m_data.at(i).angularAccel[0] << ", ";
		std::cout << "y: " << m_data.at(i).angularAccel[1] << ", ";
		std::cout << "z: " << m_data.at(i).angularAccel[2] << ")"<< std::endl << std::endl;
	}
}

/*! Print Data datagram in a CSV format
*/
void AngularSegmentKinematicsDatagram::printCSVData() const
{
  for (int i = 0; i < m_data.size(); i++)
	{
	   for(int j=0; j<4; j++) {
	       std::cout << m_data[i].segmentOrien[j] << ",";
	   }
	   for(int j=0; j<3; j++) {
	      std::cout << m_data[i].angularVeloc[j] << ",";
	   }
	   for(int j=0; j<3; j++) {
	      std::cout << m_data[i].angularAccel[j] << ",";
	   }
  }
  std::cout << std::endl;
}

/*! Get the datagram value
*/
std::vector<angularSegmentKinematics> AngularSegmentKinematicsDatagram::getData()
{
  return m_data;
}

angularSegmentKinematics AngularSegmentKinematicsDatagram::getItem(int32_t segmentID)
{
    angularSegmentKinematics req_angular_segment_kinematics;
    std::vector<angularSegmentKinematics>::iterator it;

    struct FindAngSegKinByID {
        int32_t segment_id;
        FindAngSegKinByID(int32_t sID) : segment_id(sID) {}
        bool operator()(const angularSegmentKinematics& j) const {
            return (j.segmentId == segment_id);
        }
    };

    int32_t segment_id = segmentID; // Check why segmentID is not declared in scope
    it = std::find_if(m_data.begin(), m_data.end(), FindAngSegKinByID(segment_id));

    if (it != m_data.end())
        req_angular_segment_kinematics = *it;
    else
        std::memset(&req_angular_segment_kinematics, 0, sizeof req_angular_segment_kinematics);

    return req_angular_segment_kinematics;
}
