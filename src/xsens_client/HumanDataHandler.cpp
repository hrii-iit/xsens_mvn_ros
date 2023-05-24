#include "xsens_awinda_ros/HumanDataHandler.h"
#include <iostream>

namespace hrii {

namespace ergonomics {

HumanDataHandler::HumanDataHandler()
{
    com_ << 0.0, 0.0, 0.0;
}

bool HumanDataHandler::init(const std::vector<std::string>& link_name_list, 
                            const std::vector<std::string>& joint_name_list)
{
    link_name_list_ = link_name_list;
    joint_name_list_ = joint_name_list;

    link_map_.clear();
    for (std::string link_name : link_name_list_)
        setLink(link_name, hrii::ergonomics::Link(link_name));

    joint_map_.clear();
    for (std::string joint_name : joint_name_list_)
        setJoint(joint_name, hrii::ergonomics::Joint(joint_name));

    return true;
}

bool HumanDataHandler::setJoint(const std::string& joint_name, const hrii::ergonomics::Joint& joint)
{
    if (joint_map_.count(joint_name) == 0)
    {
        joint_map_.insert(std::pair<std::string, hrii::ergonomics::Joint>(joint_name, joint));
    }
    else
    {
        joint_map_[joint_name] = joint;
    }
    
    return true;
}

bool HumanDataHandler::setJointAngles(const std::string& joint_name, const Eigen::Vector3d& joint_angles)
{
    if (joint_map_.count(joint_name) == 0)
    {
        hrii::ergonomics::Joint joint;
        joint.name = joint_name;
        joint.state.angles = joint_angles;
        joint_map_.insert(std::pair<std::string, hrii::ergonomics::Joint>(joint_name, joint));
    }
    else
    {
        joint_map_[joint_name].state.angles = joint_angles;
    }
    
    return true;
}

bool HumanDataHandler::getJoint(const std::string& joint_name, hrii::ergonomics::Joint& joint)
{
    if (joint_map_.count(joint_name) == 0)
    {
        std::cerr << "Joint \"" << joint_name << "\" not found.";
        return false;
    }
    
    joint = joint_map_[joint_name];
    
    return true;
}

bool HumanDataHandler::setLink(const std::string& link_name, const hrii::ergonomics::Link& link)
{
    if (link_map_.count(link_name) == 0)
    {
        link_map_.insert(std::pair<std::string, hrii::ergonomics::Link>(link_name, link));
    }
    else
    {
        link_map_[link_name] = link;
    }

    return true;
}

bool HumanDataHandler::setLinkPose(const std::string& link_name, const Eigen::Vector3d& link_pos, const Eigen::Quaterniond& link_orient)
{
    if (link_map_.count(link_name) == 0)
    {
        hrii::ergonomics::Link link;
        link.name = link_name;
        link.state.position = link_pos;
        link.state.orientation = link_orient.normalized();
        link_map_.insert(std::pair<std::string, hrii::ergonomics::Link>(link_name, link));
    }
    else
    {
        link_map_[link_name].state.position = link_pos;
        link_map_[link_name].state.orientation = link_orient.normalized();
    }

    return true;
}

bool HumanDataHandler::setLinkState(const std::string& link_name, const hrii::ergonomics::LinkState& link_state)
{
    if (link_map_.count(link_name) == 0)
    {
        hrii::ergonomics::Link link;
        link.name = link_name;
        link.state = link_state;
        link_map_.insert(std::pair<std::string, hrii::ergonomics::Link>(link_name, link));
    }
    else
    {
        link_map_[link_name].state = link_state;
        link_map_[link_name].state.orientation = link_map_[link_name].state.orientation.normalized();
    }

    return true;
}

bool HumanDataHandler::getLink(const std::string& link_name, hrii::ergonomics::Link& link)
{
    if (link_map_.count(link_name) == 0)
        return false;
    
    link = link_map_[link_name];
    
    return true;
}

std::vector<std::string> HumanDataHandler::getLinkNames()
{
    return link_name_list_;
}

std::vector<std::string> HumanDataHandler::getJointNames()
{
    return joint_name_list_;
}

std::map<std::string, hrii::ergonomics::Link> HumanDataHandler::getLinks()
{
    return link_map_;
}

std::map<std::string, hrii::ergonomics::Joint> HumanDataHandler::getJoints()
{
    return joint_map_;
}


Eigen::Vector3d HumanDataHandler::getCOM()
{
    return com_;
}

void HumanDataHandler::setCOM(const Eigen::Vector3d& com)
{
    com_ = com;
}

} // namespace ergonomics

} // namespace hrii
