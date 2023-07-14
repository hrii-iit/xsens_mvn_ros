#include "xsens_mvn_ros/XSensClient.h"
#include "xsens_mvn_ros/XSensModel.h"

XSensClient::XSensClient(const int& udp_port) :
    udp_port_(udp_port),
    client_active_(false)
{}

bool XSensClient::init()
{
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
    std::cout << "XSens client start reading." << std::endl << std::endl;
    
    if (!buildXSensModel())
    {
        std::cerr << "Failure building human model." << std::endl;
        client_active_ = false;
    }
    else
    {
        std::cout << "Human model built." << std::endl;
        client_active_ = true;
    }

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

bool XSensClient::buildXSensModel()
{
    // Initialize human data handler
    human_data_ = std::make_shared<hrii::ergonomics::HumanDataHandler>();
    link_name_list_.clear();
    joint_name_list_.clear();

    XSensModelNames xsens_model_names;

    auto quaternion_datagram = waitForQuaternionDatagram();
    auto joint_angles_datagram = waitForJointAnglesDatagram();

    // Get links
    std::cout << "Available links: " << quaternion_datagram.getData().size() << std::endl;
    for (auto xsens_link: quaternion_datagram.getData())
    {
        if(!human_data_->setLink(xsens_model_names.links[xsens_link.segmentId], hrii::ergonomics::Link(xsens_model_names.links[xsens_link.segmentId])))
        {
            std::cerr << "Error inserting link " << xsens_model_names.links[xsens_link.segmentId] << ". Exiting...";
            return false;
        }
        else
        {
            link_name_list_.push_back(xsens_model_names.links[xsens_link.segmentId]);
        }
    }
    if (human_data_->getLinks().size() == 0)
    {
        std::cerr << "No link elements found." << std::endl;
        return false;
    }

    std::cout << "Available joints: " << joint_angles_datagram.getData().size() << std::endl;
    for (size_t joint_cnt = 0; joint_cnt < joint_angles_datagram.getData().size(); joint_cnt++)
    {
        auto xsens_joint = joint_angles_datagram.getData()[joint_cnt];
        std::cout << joint_cnt << ") " << link_name_list_[xsens_joint.parentSegmentId-1] << "(" << xsens_joint.parentSegmentId-1 << ") -> " << 
                                          link_name_list_[xsens_joint.childSegmentId-1] << "(" << xsens_joint.childSegmentId-1 << ")" << std::endl; 
        std::cout << xsens_model_names.joints[joint_cnt] << std::endl;
        if(!human_data_->setJoint(xsens_model_names.joints[joint_cnt], hrii::ergonomics::Joint(xsens_model_names.joints[joint_cnt])))
        {
            std::cerr << "Error inserting joint " << xsens_model_names.joints[joint_cnt] << ". Exiting...";
            return false;
        }
        else
        {
            joint_name_list_.push_back(xsens_model_names.joints[joint_cnt]);
        }
    }
    if (human_data_->getJoints().size() == 0)
    {
        std::cerr << "No joint elements found." << std::endl;
        return false;
    }

    return true;
}

QuaternionDatagram XSensClient::waitForQuaternionDatagram()
{
    std::cout << "Waiting for quaternion datagram received..." << std::endl;
    QuaternionDatagram quaternion_datagram;
    while (parser_manager_.getQuaternionDatagram() == NULL)
    {
        if(!readData()) return QuaternionDatagram();
        if (parser_manager_.getQuaternionDatagram() != NULL)
        {
            std::cout << "Quaternion datagram received." << std::endl;
            quaternion_datagram = *(parser_manager_.getQuaternionDatagram());
        }
    }
    return quaternion_datagram;
}

JointAnglesDatagram XSensClient::waitForJointAnglesDatagram()
{
    std::cout << "Waiting for joint angles..." << std::endl;
    JointAnglesDatagram joint_angles_datagram;
    while (parser_manager_.getJointAnglesDatagram() == NULL)
    {
        if(!readData()) return JointAnglesDatagram();
        if (parser_manager_.getJointAnglesDatagram() != NULL)
        {
            std::cout << "Joint angles datagram received." << std::endl;
            joint_angles_datagram = *(parser_manager_.getJointAnglesDatagram());
        }
    }
    return joint_angles_datagram;
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
        human_data_->setJointAngles("right_ballfoot",    jointAngleToEigenVector3d(joint_angles->getItem(18, 19), -1, 1, -1));
        human_data_->setJointAngles("left_hip",          jointAngleToEigenVector3d(joint_angles->getItem(1, 20), 1, -1, -1));
        human_data_->setJointAngles("left_knee",         jointAngleToEigenVector3d(joint_angles->getItem(20, 21), 1, -1, 1));
        human_data_->setJointAngles("left_ankle",        jointAngleToEigenVector3d(joint_angles->getItem(21, 22), 1, -1, -1));
        human_data_->setJointAngles("left_ballfoot",     jointAngleToEigenVector3d(joint_angles->getItem(22, 23), 1, -1, -1));

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
        // std::vector<std::string> link_names = human_data_->getLinkNames();
        // if (quaternion_datagram->getData().size() != link_names.size())
        // {
        //     std::cerr << "Quaternion datagram returns an array of " << quaternion_datagram->getData().size() << 
        //         " elements, while it was expected an array of " << link_names.size() << " elements." << std::endl;
        //     return;
        // }

        for (int link_cnt = 0; link_cnt < link_name_list_.size(); link_cnt++)
        {
            auto link_pose = quaternion_datagram->getItem(link_cnt+1);
            Eigen::Vector3d link_pos;
            link_pos << link_pose.sensorPos[0], link_pose.sensorPos[1], link_pose.sensorPos[2];
            Eigen::Quaterniond link_orient(link_pose.quatRotation[0], link_pose.quatRotation[1], link_pose.quatRotation[2], link_pose.quatRotation[3]);
            human_data_->setLinkPose(link_name_list_[link_cnt], link_pos, link_orient);
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
        // std::vector<std::string> link_names = human_data_->getLinkNames();

        for (int link_cnt = 0; link_cnt < link_name_list_.size(); link_cnt++)
        {
            // Get linear kinematics from datagram structure
            auto link_linear_kinematics = linear_segment_kinematics_datagram->getItem(link_cnt+1);

            // Get link
            hrii::ergonomics::Link link;
            if (!human_data_->getLink(link_name_list_[link_cnt], link))
            {
                std::cerr << "Link " << link_name_list_[link_cnt] << "not found" << std::endl;
            }
            else
            {
                link.state.velocity.linear << link_linear_kinematics.velocity[0], 
                                              link_linear_kinematics.velocity[1], 
                                              link_linear_kinematics.velocity[2];
                link.state.acceleration.linear <<   link_linear_kinematics.acceleration[0], 
                                                    link_linear_kinematics.acceleration[1], 
                                                    link_linear_kinematics.acceleration[2];
        
                human_data_->setLinkState(link_name_list_[link_cnt], link.state);
            }
        }
    }
}

void XSensClient::updateLinkAngularTwists()
{
    auto angular_segment_kinematics_datagram = parser_manager_.getAngularSegmentKinematicsDatagram();

    if (angular_segment_kinematics_datagram != NULL)
    {
        // std::vector<std::string> link_names = human_data_->getLinkNames();

        for (int link_cnt = 0; link_cnt < link_name_list_.size(); link_cnt++)
        {
            // Get angular kinematics from datagram structure
            auto link_angular_kinematics = angular_segment_kinematics_datagram->getItem(link_cnt+1);

            // Get link
            hrii::ergonomics::Link link;
            if (!human_data_->getLink(link_name_list_[link_cnt], link))
            {
                std::cerr << "Link " << link_name_list_[link_cnt] << "not found" << std::endl;
            }
            else
            {
                link.state.velocity.angular <<  link_angular_kinematics.angularVeloc[0]*M_PI/180, 
                                                link_angular_kinematics.angularVeloc[1]*M_PI/180, 
                                                link_angular_kinematics.angularVeloc[2]*M_PI/180;
                link.state.acceleration.angular <<  link_angular_kinematics.angularAccel[0]*M_PI/180, 
                                                    link_angular_kinematics.angularAccel[1]*M_PI/180, 
                                                    link_angular_kinematics.angularAccel[2]*M_PI/180;

                human_data_->setLinkState(link_name_list_[link_cnt], link.state);
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



XSensClient::~XSensClient()
{
    client_active_ = false;
    data_acquisition_thread_.join();
    std::cout << "~XSensClient()" << std::endl;
}
