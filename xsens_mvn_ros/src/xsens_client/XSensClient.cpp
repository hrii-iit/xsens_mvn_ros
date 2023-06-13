#include "xsens_mvn_ros/XSensClient.h"
#include <ros/ros.h>

XSensClient::XSensClient(const int& udp_port) :
    udp_port_(udp_port),
    client_active_(false)
{}

bool XSensClient::init()
{
    // Initialize human data model
    human_data_ = std::make_shared<hrii::ergonomics::HumanDataHandler>();

    // Initialize UDP communication
    udp_socket_ = boost::make_shared<Socket>(IP_UDP);
    if(!udp_socket_->bind(udp_port_))
    {
        std::cout << "Error binding XSens port.";
        return false;
    }
    
    data_acquisition_thread_ = boost::thread(&XSensClient::dataAcquisitionCallback, this);

    // Wait for XSens data acquisition activation
    while (!client_active_)
    {
        boost::this_thread::sleep_for(boost::chrono::milliseconds(500));
    }
    std::cout << "XSens client initialized." << std::endl;

    return true;
}

void XSensClient::dataAcquisitionCallback()
{
    std::cout << "Xsens client start reading." << std::endl << std::endl;
    
    if (!buildHumanModel())
    {
        std::cerr << "Failure building human model." << std::endl;
        client_active_ = false;
    }
    else
    {
        std::cout << "Human model built." << std::endl;
        client_active_ = true;
    }

    // fromHumanModelToURDF();

    while (client_active_)
    {
        if (readData())
        {
            updateJointAngles();
            updateLinkPoses();
            updateLinkLinearTwists();
            updateLinkAngularTwists();
            updateCOM();
        }
    }
}

bool XSensClient::readData()
{
    // Read data from UDP socket
    int readed_bytes = udp_socket_->read(data_buffer_, MAX_MVN_DATAGRAM_SIZE);
    if (readed_bytes > 0)
    {
        parser_manager_.readDatagram(data_buffer_);
        return true;
    }
    std::cerr << "Error reading data" << std::endl;
    client_active_ = false;
    return false;
}

bool XSensClient::buildHumanModel()
{
    link_name_list_.clear();
    std::vector<std::string> full_link_name_list;
    full_link_name_list.clear();
    full_link_name_list.push_back("pelvis");
    full_link_name_list.push_back("l5");
    full_link_name_list.push_back("l3");
    full_link_name_list.push_back("t12");
    full_link_name_list.push_back("t8");
    full_link_name_list.push_back("neck");
    full_link_name_list.push_back("head");
    full_link_name_list.push_back("right_shoulder");
    full_link_name_list.push_back("right_upper_arm");
    full_link_name_list.push_back("right_forearm");
    full_link_name_list.push_back("right_hand");
    full_link_name_list.push_back("left_shoulder");
    full_link_name_list.push_back("left_upper_arm");
    full_link_name_list.push_back("left_forearm");
    full_link_name_list.push_back("left_hand");
    full_link_name_list.push_back("right_upper_leg");
    full_link_name_list.push_back("right_lower_leg");
    full_link_name_list.push_back("right_foot");
    full_link_name_list.push_back("right_toe");
    full_link_name_list.push_back("left_upper_leg");
    full_link_name_list.push_back("left_lower_leg");
    full_link_name_list.push_back("left_foot");
    full_link_name_list.push_back("left_toe");
    full_link_name_list.push_back("generic_link");

    joint_name_list_.clear();

    QuaternionDatagram quaternion_datagram;
    std::cout << "Waiting for quaternion datagram received..." << std::endl;
    while (quaternion_datagram.getData().size() == 0)
    {
        if(!readData()) return false;
        if (parser_manager_.getQuaternionDatagram() != NULL)
            quaternion_datagram = *(parser_manager_.getQuaternionDatagram());
    }
    std::cout << "Quaternion datagram received." << std::endl;

    std::cout << "Waiting for joint angles..." << std::endl;
    JointAnglesDatagram joint_angles_datagram;
    while (joint_angles_datagram.getData().size() == 0)
    {
        if(!readData()) return false;
        if (parser_manager_.getJointAnglesDatagram() != NULL)
            joint_angles_datagram = *(parser_manager_.getJointAnglesDatagram());
    }
    std::cout << "Joint angles received." << std::endl;

    urdf_model_ = std::make_shared<urdf::ModelInterface>();
    urdf_model_->clear();

    // Get human model name
    urdf_model_->name_ = "skeleton";

    // Set material
    urdf::MaterialSharedPtr material(new urdf::Material);

    urdf_model_->materials_.insert(make_pair(material->name,material));

    // Get links
    for (int link_cnt = 0; link_cnt < quaternion_datagram.getData().size(); link_cnt++)
    {
        auto xsens_link = quaternion_datagram.getItem(link_cnt);
        urdf::LinkSharedPtr link(new urdf::Link);
        if (urdf_model_->getLink(full_link_name_list[xsens_link.segmentId]))
        {
            std::cerr << "Link " << full_link_name_list[xsens_link.segmentId] << " is not unique." << std::endl;

            return false;
        }
        else
        {
            link_name_list_.push_back(full_link_name_list[xsens_link.segmentId]);
        }
        //         if (link->visual)
//         {
//           assignMaterial(link->visual, model, link->name.c_str());
//         }
//         for (const auto& visual : link->visual_array)
//         {
//           assignMaterial(visual, model, link->name.c_str());
//         }

        urdf_model_->links_.insert(make_pair(link_name_list_[xsens_link.segmentId], link));
        std::cout << "Successfully added a new link " << xsens_link.segmentId << ": " << link_name_list_[xsens_link.segmentId] << "" << std::endl;
    }
    if (urdf_model_->links_.empty())
    {
        std::cerr << "No link elements found." << std::endl;
        return false;
    }

    // Get all joints
    for (int joint_cnt = 0; joint_cnt < joint_angles_datagram.getData().size(); joint_cnt++)
    {
        auto xsens_joint = joint_angles_datagram.getData()[joint_cnt];
        urdf::JointSharedPtr joint(new urdf::Joint);
        // std::cout << static_cast<int>(xsens_joint.parentSegmentId) << "_" << static_cast<int>(xsens_joint.childSegmentId) << std::endl;
        std::string joint_name = link_name_list_[xsens_joint.parentSegmentId-1] + "_" + link_name_list_[xsens_joint.childSegmentId-1];
        joint_name_list_.push_back(joint_name);
        std::cout << static_cast<int>(xsens_joint.parentSegmentId) << "_" << static_cast<int>(xsens_joint.childSegmentId) << std::endl;
        if (urdf_model_->getJoint(joint_name))
        {
            std::cout << "Joint " << joint_name << " is not unique." << std::endl;
            // return false;
        }
        else
        {   
            urdf_model_->joints_.insert(make_pair(joint_name, joint));
            std::cout << "Successfully added a new joint (" << xsens_joint.parentSegmentId-1 << 
                " -> " << xsens_joint.childSegmentId-1 << "):" << joint_name << "" << std::endl;
        }
        
    }
    if (urdf_model_->joints_.empty())
    {
        std::cerr << "No joint elements found." << std::endl;
        return false;
    }

    if (!human_data_->init(link_name_list_, joint_name_list_))
        return false;

    return true;
}
    
void XSensClient::updateJointAngles()
{
    auto joint_angles = parser_manager_.getJointAnglesDatagram();
    if(joint_angles != NULL)
    {
        // Update joint angles according to N-pose
        human_data_->setJointAngles("l5_s1",             jointAngleToEigenVector3d(joint_angles->getItem(1, 2), 1, 1, 1));
        human_data_->setJointAngles("l4_l3",             jointAngleToEigenVector3d(joint_angles->getItem(2, 3), 1, 1, 1));
        human_data_->setJointAngles("l1_t12",            jointAngleToEigenVector3d(joint_angles->getItem(3, 4), 1, 1, 1));
        human_data_->setJointAngles("t9_t8",             jointAngleToEigenVector3d(joint_angles->getItem(4, 5), 1, 1, 1));
        human_data_->setJointAngles("t1_c7",             jointAngleToEigenVector3d(joint_angles->getItem(5, 6), 1, 1, 1));
        human_data_->setJointAngles("c1_head",           jointAngleToEigenVector3d(joint_angles->getItem(6, 7), 1, 1, 1));
        human_data_->setJointAngles("right_c7_shoulder", jointAngleToEigenVector3d(joint_angles->getItem(5, 8), -1, 1, -1));
        human_data_->setJointAngles("right_shoulder",    jointAngleToEigenVector3d(joint_angles->getItem(8, 9), -1, 1, -1));
        human_data_->setJointAngles("right_elbow",       jointAngleToEigenVector3d(joint_angles->getItem(9, 10), -1, 1, -1));
        human_data_->setJointAngles("right_wrist",       jointAngleToEigenVector3d(joint_angles->getItem(10, 11), -1, 1, -1));
        human_data_->setJointAngles("left_c7_shoulder",  jointAngleToEigenVector3d(joint_angles->getItem(5, 12), 1, -1, -1));
        human_data_->setJointAngles("left_shoulder",     jointAngleToEigenVector3d(joint_angles->getItem(12, 13), 1, -1, -1));
        human_data_->setJointAngles("left_elbow",        jointAngleToEigenVector3d(joint_angles->getItem(13, 14), 1, -1, -1));
        human_data_->setJointAngles("left_wrist",        jointAngleToEigenVector3d(joint_angles->getItem(14, 15), 1, -1, -1));
        human_data_->setJointAngles("right_hip",         jointAngleToEigenVector3d(joint_angles->getItem(1, 16), -1, 1, -1));
        human_data_->setJointAngles("right_knee",        jointAngleToEigenVector3d(joint_angles->getItem(16, 17), -1, 1, 1));
        human_data_->setJointAngles("right_ankle",       jointAngleToEigenVector3d(joint_angles->getItem(17, 18), -1, 1, -1));
        human_data_->setJointAngles("right_ball_foot",   jointAngleToEigenVector3d(joint_angles->getItem(18, 19), -1, 1, -1));
        human_data_->setJointAngles("left_hip",          jointAngleToEigenVector3d(joint_angles->getItem(1, 20), 1, -1, -1));
        human_data_->setJointAngles("left_knee",         jointAngleToEigenVector3d(joint_angles->getItem(20, 21), 1, -1, 1));
        human_data_->setJointAngles("left_ankle",        jointAngleToEigenVector3d(joint_angles->getItem(21, 22), 1, -1, -1));
        human_data_->setJointAngles("left_ball_foot",    jointAngleToEigenVector3d(joint_angles->getItem(22, 23), 1, -1, -1));

        // Add +90° to elbows angle
        hrii::ergonomics::Joint elbow_joint;
        human_data_->getJoint("right_elbow", elbow_joint);
        elbow_joint.state.angles[2] -= 90.0;
        human_data_->setJoint("right_elbow", elbow_joint);
        human_data_->getJoint("left_elbow", elbow_joint);
        elbow_joint.state.angles[2] += 90.0;
        human_data_->setJoint("left_elbow", elbow_joint);
    }
}

void XSensClient::updateLinkPoses()
{
    auto quaternion_datagram = parser_manager_.getQuaternionDatagram();
    if(quaternion_datagram != NULL)
    {
        std::vector<std::string> link_names = human_data_->getLinkNames();
        // if (quaternion_datagram->getData().size() != link_names.size())
        // {
        //     std::cerr << "Quaternion datagram returns an array of " << quaternion_datagram->getData().size() << 
        //         " elements, while it was expected an array of " << link_names.size() << " elements." << std::endl;
        //     return;
        // }

        for (int link_cnt = 0; link_cnt < link_names.size(); link_cnt++)
        {
            auto link_pose = quaternion_datagram->getItem(link_cnt+1);
            Eigen::Vector3d link_pos;
            link_pos << link_pose.sensorPos[0], link_pose.sensorPos[1], link_pose.sensorPos[2];
            Eigen::Quaterniond link_orient(link_pose.quatRotation[0], link_pose.quatRotation[1], link_pose.quatRotation[2], link_pose.quatRotation[3]);
            human_data_->setLinkPose(link_names[link_cnt], link_pos, link_orient);
        }

        Eigen::Quaterniond quat_x_rot_90neg(0.7071068, -0.7071068, 0.0, 0.0);
        rotateLink("right_upper_arm", quat_x_rot_90neg);
        rotateLink("right_forearm", quat_x_rot_90neg);
        rotateLink("right_hand", quat_x_rot_90neg);

        Eigen::Quaterniond quat_x_rot_90pos(0.7071068, 0.7071068, 0.0, 0.0);
        rotateLink("left_upper_arm", quat_x_rot_90pos);
        rotateLink("left_forearm", quat_x_rot_90pos);
        rotateLink("left_hand", quat_x_rot_90pos);

    }
}

void XSensClient::rotateLink(const std::string& link_name, const Eigen::Quaterniond& quat)
{
    hrii::ergonomics::Link link_to_rotate;
    human_data_->getLink(link_name, link_to_rotate);
    link_to_rotate.state.orientation = link_to_rotate.state.orientation * quat;
    human_data_->setLink(link_name, link_to_rotate);
}

void XSensClient::updateLinkLinearTwists()
{
    auto linear_segment_kinematics_datagram = parser_manager_.getLinearSegmentKinematicsDatagram();

    if (linear_segment_kinematics_datagram != NULL)
    {
        std::vector<std::string> link_names = human_data_->getLinkNames();

        for (int link_cnt = 0; link_cnt < link_names.size(); link_cnt++)
        {
            // Get linear kinematics from datagram structure
            auto link_linear_kinematics = linear_segment_kinematics_datagram->getItem(link_cnt+1);

            // Get link
            hrii::ergonomics::Link link;
            if (!human_data_->getLink(link_names[link_cnt], link))
            {
                std::cerr << "Link " << link_names[link_cnt] << "not found" << std::endl;
            }
            else
            {
                link.state.velocity.linear <<   link_linear_kinematics.velocity[0], 
                                            link_linear_kinematics.velocity[1], 
                                            link_linear_kinematics.velocity[2];
                link.state.acceleration.linear << link_linear_kinematics.acceleration[0], 
                                                    link_linear_kinematics.acceleration[1], 
                                                    link_linear_kinematics.acceleration[2];
        
                human_data_->setLinkState(link_names[link_cnt], link.state);
            }
        }
    }
}

void XSensClient::updateLinkAngularTwists()
{
    auto angular_segment_kinematics_datagram = parser_manager_.getAngularSegmentKinematicsDatagram();

    if (angular_segment_kinematics_datagram != NULL)
    {
        std::vector<std::string> link_names = human_data_->getLinkNames();

        for (int link_cnt = 0; link_cnt < link_names.size(); link_cnt++)
        {
            // Get angular kinematics from datagram structure
            auto link_angular_kinematics = angular_segment_kinematics_datagram->getItem(link_cnt+1);

            // Get link
            hrii::ergonomics::Link link;
            if (!human_data_->getLink(link_names[link_cnt], link))
            {
                std::cerr << "Link " << link_names[link_cnt] << "not found" << std::endl;
            }
            else
            {
                link.state.velocity.angular <<  link_angular_kinematics.angularVeloc[0]*M_PI/180, 
                                                link_angular_kinematics.angularVeloc[1]*M_PI/180, 
                                                link_angular_kinematics.angularVeloc[2]*M_PI/180;
                link.state.acceleration.angular <<  link_angular_kinematics.angularAccel[0]*M_PI/180, 
                                                    link_angular_kinematics.angularAccel[1]*M_PI/180, 
                                                    link_angular_kinematics.angularAccel[2]*M_PI/180;

                human_data_->setLinkState(link_names[link_cnt], link.state);
            }
        }
    }
}

void XSensClient::updateCOM()
{
    auto com_datagram = parser_manager_.getCenterOfMassDatagram();

    if (com_datagram != NULL)
    {
        Eigen::Vector3d com;
        com << com_datagram->getData()[0], com_datagram->getData()[1], com_datagram->getData()[2];
        // std::cout << com << std::endl;
        human_data_->setCOM(com);
    }
}

Eigen::Vector3d XSensClient::jointAngleToEigenVector3d(const JointAngle& joint_angle, 
                                                        const double& x_axis, 
                                                        const double& y_axis, 
                                                        const double& z_axis)
{
    // Convert JointAngle object to a Eigen Vector3d rotating axes to align in resting position N-Pose
    Eigen::Vector3d joint_angle_eigen_vec;
    joint_angle_eigen_vec[0] = x_axis * joint_angle.rotation[0];
    // TEMPORARY HORRIBLE SOLUTION
    joint_angle_eigen_vec[1] = z_axis * joint_angle.rotation[2];
    joint_angle_eigen_vec[2] = y_axis * joint_angle.rotation[1];
    return joint_angle_eigen_vec;
}

hrii::ergonomics::HumanDataHandler::Ptr XSensClient::getHumanData()
{
    return human_data_;
}


void XSensClient::fromHumanModelToURDF()
{
    
    urdf_str_.append("<?xml version=\"1.0\"?><robot name=\"xsens_model\">");
    for (std::map<std::string, std::shared_ptr<urdf::Joint>>::iterator joint_it = urdf_model_->joints_.begin(); joint_it != urdf_model_->joints_.end(); joint_it++)
    {
        // std::cout << "--- " << std::endl;
        // std::cout << joint_it->first << "" << std::endl;
        // std::cout << joint_it->second->name << "" << std::endl;
        // urdf_str.append(jointToUrdf(joint_it->second->name, "revolute", "0 0 1", 
                                    // "0 0 0", "0 0 0", "child_link", "parent_link", "0", "10", "0", "10"));
    }
    // urdf_str.append(jointToUrdf("root_joint", "revolute", "0 0 1", 
    //                                 "0 0 0", "0 0 0", "child_link", "parent_link", "0", "10", "0", "10"));
    urdf_str_.append(linkToUrdf("child_link", "0", "0 0 1", 
                                    "0 0 0", "0 0 0", "child_link", "parent_link", "0", "10", "0", "10"));
    // urdf_str.append(linkToUrdf("parent_link", "0", "0 0 1", 
    //                                 "0 0 0", "0 0 0", "child_link", "parent_link", "0", "10", "0", "10"));
    urdf_str_.append("</robot>");

}

std::string XSensClient::getURDFString()
{
    return urdf_str_;
}

std::string XSensClient::jointToUrdf(const std::string& name, const std::string& type, const std::string& axis,
                        const std::string& xyz, const std::string& rpy, const std::string& child_link,
                        const std::string& parent_link, const std::string& effort, const std::string& velocity,
                        const std::string& lower_limit, const std::string& upper_limit)
{
    std::string joint_urdf = "";
    joint_urdf.append("<joint name=\""+name + "\" type=\"" + type + "\">");
    joint_urdf.append("<origin xyz=\"" + xyz + "\" type=\"" + rpy + "\"/>");
    joint_urdf.append("<axis xyz=\"" + axis + "\"/>");
    joint_urdf.append("<parent link=\"" + parent_link + "\"/>");
    joint_urdf.append("<child link=\"" + child_link + "\"/>");
    joint_urdf.append("<limit effort=\"" + effort + "\" velocity=\"" + velocity + 
                        " lower=\"" + lower_limit + "\" upper=\"" + upper_limit + "\"/>");
    joint_urdf.append("</joint>");
    return joint_urdf;
}

std::string XSensClient::linkToUrdf(const std::string& name, const std::string& mass, const std::string& axis,
                        const std::string& xyz, const std::string& rpy, const std::string& child_link,
                        const std::string& parent_link, const std::string& effort, const std::string& velocity,
                        const std::string& lower_limit, const std::string& upper_limit)
{
    std::string link_urdf = "";
    link_urdf.append("<link name=\"" + name + "\">");
    // link_urdf.append("<inertial>");
    // link_urdf.append("<mass xyz=\"" + mass + "\"/>");
    // link_urdf.append("</inertial>");

    // link_urdf.append("<origin xyz=\"" + xyz + "\" type=\"" + rpy + "\"/>");
    // link_urdf.append("<axis xyz=\"" + axis + "\"/>");
    // link_urdf.append("<parent link=\"" + parent_link + "\"/>");
    // link_urdf.append("<child link=\"" + child_link + "\"/>");
    // link_urdf.append("<limit effort=\"" + effort + "\" velocity=\"" + velocity + 
    //                     " lower=\"" + limit_lower + "\" upper=\"" + limit_upper + "\"/>");
    link_urdf.append("</link>");
    return link_urdf;
}

// <xacro:macro name="m_link" params="name mass origin_xyz origin_rpy ixx ixy ixz iyy iyz izz meshfile meshscale color">
//     <link name="${name}">
//       <inertial>
//         <mass value="${mass}" />
//         <origin rpy="${origin_rpy}" xyz="${origin_xyz}" />
//         <inertia ixx="${ixx}" ixy="${ixy}" ixz="${ixz}" iyy="${iyy}" iyz="${iyz}" izz="${izz}" />
//       </inertial>
//       <collision>
//         <origin rpy="${origin_rpy}" xyz="${origin_xyz}" />
//         <geometry>
//           <mesh filename="${meshfile}" scale="${meshscale}"/>
//         </geometry>
//       </collision>
//       <visual>
//         <origin rpy="${origin_rpy}" xyz="${origin_xyz}" />
//         <geometry>
//             <mesh filename="${meshfile}" scale="${meshscale}"/>

//         </geometry>
//         <material name="${color}"/>
//       </visual>

//     </link>
//   </xacro:macro>
  
//   <xacro:macro name="m_fake_link" params="name">
//     <link name="${name}">
//     </link>
//   </xacro:macro>

//   <xacro:macro name="m_joint" params="name type axis_xyz origin_rpy origin_xyz parent child effort velocity limit_lower limit_upper">
//     
//   </xacro:macro>


//   for (TiXmlElement* joint_xml = robot_xml->FirstChildElement("joint"); joint_xml; joint_xml = joint_xml->NextSiblingElement("joint"))
//   {
//     JointSharedPtr joint;
//     joint.reset(new Joint);

//     if (parseJoint(*joint, joint_xml))
//     {
//       if (model->getJoint(joint->name))
//       {
//         CONSOLE_BRIDGE_logError("joint '%s' is not unique.", joint->name.c_str());
//         model.reset();
//         return model;
//       }
//       else
//       {
//         model->joints_.insert(make_pair(joint->name,joint));
//         CONSOLE_BRIDGE_logDebug("urdfdom: successfully added a new joint '%s'", joint->name.c_str());
//       }
//     }
//     else
//     {
//       CONSOLE_BRIDGE_logError("joint xml is not initialized correctly");
//       model.reset();
//       return model;
//     }
//   }


//   // every link has children links and joints, but no parents, so we create a
//   // local convenience data structure for keeping child->parent relations
//   std::map<std::string, std::string> parent_link_tree;
//   parent_link_tree.clear();

//   // building tree: name mapping
//   try 
//   {
//     model->initTree(parent_link_tree);
//   }
//   catch(ParseError &e)
//   {
//     CONSOLE_BRIDGE_logError("Failed to build tree: %s", e.what());
//     model.reset();
//     return model;
//   }

//   // find the root link
//   try
//   {
//     model->initRoot(parent_link_tree);
//   }
//   catch(ParseError &e)
//   {
//     CONSOLE_BRIDGE_logError("Failed to find root link: %s", e.what());
//     model.reset();
//     return model;
//   }
  
//   return model;
// }
    

XSensClient::~XSensClient()
{
    client_active_ = false;
    data_acquisition_thread_.join();
    std::cout << "~XSensClient()" << std::endl;
}

// ModelInterfaceSharedPtr  parseURDF(const std::string &xml_string)
// {
//   ModelInterfaceSharedPtr model(new ModelInterface);
//   model->clear();

//   TiXmlDocument xml_doc;
//   xml_doc.Parse(xml_string.c_str());
//   if (xml_doc.Error())
//   {
//     CONSOLE_BRIDGE_logError(xml_doc.ErrorDesc());
//     xml_doc.ClearError();
//     model.reset();
//     return model;
//   }

//   TiXmlElement *robot_xml = xml_doc.FirstChildElement("robot");
//   if (!robot_xml)
//   {
//     CONSOLE_BRIDGE_logError("Could not find the 'robot' element in the xml file");
//     model.reset();
//     return model;
//   }

//   // Get robot name
//   const char *name = robot_xml->Attribute("name");
//   if (!name)
//   {
//     CONSOLE_BRIDGE_logError("No name given for the robot.");
//     model.reset();
//     return model;
//   }
//   model->name_ = std::string(name);

//   try
//   {
//     urdf_export_helpers::URDFVersion version(robot_xml->Attribute("version"));
//     if (!version.equal(1, 0))
//     {
//       throw std::runtime_error("Invalid 'version' specified; only version 1.0 is currently supported");
//     }
//   }
//   catch (const std::runtime_error & err)
//   {
//     CONSOLE_BRIDGE_logError(err.what());
//     model.reset();
//     return model;
//   }

//   // Get all Material elements
//   for (TiXmlElement* material_xml = robot_xml->FirstChildElement("material"); material_xml; material_xml = material_xml->NextSiblingElement("material"))
//   {
//     MaterialSharedPtr material;
//     material.reset(new Material);

//     try {
//       parseMaterial(*material, material_xml, false); // material needs to be fully defined here
//       if (model->getMaterial(material->name))
//       {
//         CONSOLE_BRIDGE_logError("material '%s' is not unique.", material->name.c_str());
//         material.reset();
//         model.reset();
//         return model;
//       }
//       else
//       {
//         model->materials_.insert(make_pair(material->name,material));
//         CONSOLE_BRIDGE_logDebug("urdfdom: successfully added a new material '%s'", material->name.c_str());
//       }
//     }
//     catch (ParseError &/*e*/) {
//       CONSOLE_BRIDGE_logError("material xml is not initialized correctly");
//       material.reset();
//       model.reset();
//       return model;
//     }
//   }

//   // Get all Link elements
//   for (TiXmlElement* link_xml = robot_xml->FirstChildElement("link"); link_xml; link_xml = link_xml->NextSiblingElement("link"))
//   {
//     LinkSharedPtr link;
//     link.reset(new Link);

//     try {
//       parseLink(*link, link_xml);
//       if (model->getLink(link->name))
//       {
//         CONSOLE_BRIDGE_logError("link '%s' is not unique.", link->name.c_str());
//         model.reset();
//         return model;
//       }
//       else
//       {
//         // set link visual(s) material
//         CONSOLE_BRIDGE_logDebug("urdfdom: setting link '%s' material", link->name.c_str());
//         if (link->visual)
//         {
//           assignMaterial(link->visual, model, link->name.c_str());
//         }
//         for (const auto& visual : link->visual_array)
//         {
//           assignMaterial(visual, model, link->name.c_str());
//         }

//         model->links_.insert(make_pair(link->name,link));
//         CONSOLE_BRIDGE_logDebug("urdfdom: successfully added a new link '%s'", link->name.c_str());
//       }
//     }
//     catch (ParseError &/*e*/) {
//       CONSOLE_BRIDGE_logError("link xml is not initialized correctly");
//       model.reset();
//       return model;
//     }
//   }
//   if (model->links_.empty()){
//     CONSOLE_BRIDGE_logError("No link elements found in urdf file");
//     model.reset();
//     return model;
//   }

//   // Get all Joint elements
//   for (TiXmlElement* joint_xml = robot_xml->FirstChildElement("joint"); joint_xml; joint_xml = joint_xml->NextSiblingElement("joint"))
//   {
//     JointSharedPtr joint;
//     joint.reset(new Joint);

//     if (parseJoint(*joint, joint_xml))
//     {
//       if (model->getJoint(joint->name))
//       {
//         CONSOLE_BRIDGE_logError("joint '%s' is not unique.", joint->name.c_str());
//         model.reset();
//         return model;
//       }
//       else
//       {
//         model->joints_.insert(make_pair(joint->name,joint));
//         CONSOLE_BRIDGE_logDebug("urdfdom: successfully added a new joint '%s'", joint->name.c_str());
//       }
//     }
//     else
//     {
//       CONSOLE_BRIDGE_logError("joint xml is not initialized correctly");
//       model.reset();
//       return model;
//     }
//   }


//   // every link has children links and joints, but no parents, so we create a
//   // local convenience data structure for keeping child->parent relations
//   std::map<std::string, std::string> parent_link_tree;
//   parent_link_tree.clear();

//   // building tree: name mapping
//   try 
//   {
//     model->initTree(parent_link_tree);
//   }
//   catch(ParseError &e)
//   {
//     CONSOLE_BRIDGE_logError("Failed to build tree: %s", e.what());
//     model.reset();
//     return model;
//   }

//   // find the root link
//   try
//   {
//     model->initRoot(parent_link_tree);
//   }
//   catch(ParseError &e)
//   {
//     CONSOLE_BRIDGE_logError("Failed to find root link: %s", e.what());
//     model.reset();
//     return model;
//   }
  
//   return model;
// }
    