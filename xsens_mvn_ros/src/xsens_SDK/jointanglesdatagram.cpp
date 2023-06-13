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

#include "xsens_SDK/jointanglesdatagram.h"
#include <boost/concept_check.hpp>


JointAngle::JointAngle(int32_t parent, int32_t child, float rotx, float roty, float rotz)
  : parent(parent), child(child)
{
  rotation[0] = rotx;
  rotation[1] = roty;
  rotation[2] = rotz;
}

JointAngle JointAngle::operator+(const JointAngle& j) const
  { 
    if ((child != j.parent) && (parent != j.child))
      return JointAngle(parent,child,rotation[0],rotation[1],rotation[2]);
    else {
      return JointAngle(
              (child == j.parent) ? parent : j.parent,
              (parent == j.child) ? child : j.child,
              rotation[0]+j.rotation[0],
              rotation[1]+j.rotation[1],
              rotation[2]+j.rotation[2]);
    }
  }


/*! \class JointAnglesDatagram
	\brief a Joint Angle datagram (type 0x20)

	Information about each joint is sent as follows.

	4 bytes parent connection identifier: 256 * segment ID + point ID
	4 bytes child connection identifer: 256 * segment ID + point ID
	4 bytes x rotation
	4 bytes y rotation
	4 bytes z rotation

	Total: 20 bytes per joint

  The coordinates use a Z-Up, right-handed coordinate system.
*/

/*! Constructor */
JointAnglesDatagram::JointAnglesDatagram()
  : Datagram()
{
  setType(SPJointAngles);
}

/*! Destructor */
JointAnglesDatagram::~JointAnglesDatagram()
{
}

/*! Deserialize the data from \a arr
  \sa serializeData
*/
void JointAnglesDatagram::deserializeData(Streamer &inputStreamer)
{
  Streamer* streamer = &inputStreamer;

  for (int i = 0; i < dataCount(); i++)
  {
    JointAngle joint;

    // Parent Connection ID  -> 4 byte 
    streamer->read(joint.parent);
    joint.parentSegmentId = joint.parent / 256;

    // Child Connection ID -> 4 byte
    streamer->read(joint.child);
    joint.childSegmentId = joint.child / 256;

    // Store the Rotation in a Vector -> 12 byte	(3 x 4 byte)
    for (int k = 0; k < 3; k++)
      streamer->read(joint.rotation[k]);

    m_data.push_back(joint);
  }
}

/*! Print Data datagram in a formated why
*/
void JointAnglesDatagram::printData() const
{
  std::cout << "*********************** DATA CONTENT ***********************" <<  std::endl <<  std::endl; 

  for (int i = 0; i < m_data.size(); i++)
  {
    std::cout << "Parent Connection ID (256 * segment ID + point ID): " << m_data.at(i).parent << std::endl;
    std::cout << "Child Connection ID (256 * segment ID + point ID): " << m_data.at(i).child << std::endl;
    // Rotation
    std::cout << "Rotation: " << "(";
    std::cout << "x: " << m_data.at(i).rotation[0] << ", ";
    std::cout << "y: " << m_data.at(i).rotation[1] << ", ";
    std::cout << "z: " << m_data.at(i).rotation[2] << ")"<< std::endl << std::endl;
  }
}

/*! Print Data datagram in CSV format
*/
void JointAnglesDatagram::printCSVData() const
{
  for (int i = 0; i < m_data.size(); i++)
  {
    // Rotation
    std::cout << m_data.at(i).rotation[0] << ",";
    std::cout << m_data.at(i).rotation[1] << ",";
    std::cout << m_data.at(i).rotation[2] << ",";
  }
  std::cout << std::endl;
}

/*! Get the datagram value
*/
std::vector<JointAngle> JointAnglesDatagram::getData()
{
  return m_data;
}

/*! Get the requested item of the datagram data
*/
JointAngle JointAnglesDatagram::getItem(int32_t parentSegmentId, int32_t childSegmentId)
{  
  JointAngle req_joint_angle;
  std::vector<JointAngle>::iterator it;
  
  //TODO: For now we are considering that the input is segmentID and not pointID.
  //TODO: We are not considering the possibility of other pointID, only pointID 0. 
	//Identifier = 256 * segment ID + point ID
  /*pointparent = pointparent*256 + 0;
  pointchild = pointchild*256 + 0;*/
  
  
  struct FindByParentAndChild {
      int32_t point_parent;
      int32_t point_child;
      FindByParentAndChild(int32_t pparent, int32_t pchild) : point_parent(pparent), point_child(pchild) {}
      bool operator()(const JointAngle& j) const { 
          return (j.parent/256 == point_parent && j.child/256 == point_child); 
      }
  };
  
  it = std::find_if(m_data.begin(), m_data.end(), FindByParentAndChild(parentSegmentId, childSegmentId));
  
  if (it != m_data.end())
    req_joint_angle = *it;
  else
    std::memset(&req_joint_angle, 0, sizeof req_joint_angle);
  
  return req_joint_angle;
}
