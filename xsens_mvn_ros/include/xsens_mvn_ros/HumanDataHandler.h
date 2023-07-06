#ifndef _XSENS_MVN_ROS_HUMAN_DATA_HANDLER_H_
#define _XSENS_MVN_ROS_HUMAN_DATA_HANDLER_H_

#include <string>
#include <map>
#include <memory>
#include <vector>

#include <Eigen/Dense>

namespace hrii {

namespace ergonomics {

class JointState
{
    public:
        // Eigen::Affine3d pose;
        // Eigen::Vector3d velocity;
        // Eigen::Vector3d acceleration;
        Eigen::Vector3d angles;
};

class Joint
{
    public:
        hrii::ergonomics::JointState state;
        std::string name;
        std::string parent_link;
        std::string child_link;

        Joint() = default;
        Joint(std::string joint_name) : name(joint_name) {};
};

class Twist
{
    public:
        Eigen::Vector3d linear;
        Eigen::Vector3d angular;
};

class LinkState
{
    public:
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;
        hrii::ergonomics::Twist velocity;
        hrii::ergonomics::Twist acceleration;
};

class Link
{
    public:
        Eigen::Vector3d length;
        hrii::ergonomics::LinkState state;
        std::string name;
        std::string parent_joint;

        Link() = default;
        Link(std::string link_name) : name(link_name) {};
};

class HumanDataHandler
{
    public:
        /**
         * @brief Shared pointer to a HumanDataHandler object
         */
        typedef std::shared_ptr<HumanDataHandler> Ptr;
        typedef std::shared_ptr<const HumanDataHandler> ConstPtr;

        /**
         * @brief HumanDataHandler constructor
         */
        HumanDataHandler();

        /**
         * @brief init Initialize the XSens data handler
         * 
         * @return bool Initialization result
         */                                       
        bool init(const std::vector<std::string>& link_name_list, 
                  const std::vector<std::string>& joint_name_list);

        /**
         * @brief Set the state of a desired joint. If it does not exist, it will be add to the map joint_name-joint_state
         * 
         * @param joint_name name of the joint to set. 
         * @param joint_state state of the joint to set. 
         * 
         * @return bool: Setting result
         */
        bool setJoint(const std::string& joint_name, const hrii::ergonomics::Joint& joint);

        /**
         * @brief Set the joint angles vector of a desired joint. If it does not exist, it will be add to the map joint_name-joint_state
         * 
         * @param joint_name name of the joint to set. 
         * @param joint_angles joint rotations x y z. 
         * 
         * @return bool: Setting result
         */
        bool setJointAngles(const std::string& joint_name, const Eigen::Vector3d& joint_angles);

        /**
         * @brief Get the state of a desired joint. If it does not exist it return false;
         * 
         * @param joint_name name of the joint to get.
         * 
         * @return bool: Getting result
         */
        bool getJoint(const std::string& joint_name, hrii::ergonomics::Joint& joint);

        /**
         * @brief Set the state of a desired link. If it does not exist, it will be add to the map link_name-link_state
         * 
         * @param link_name name of the link to set. 
         * @param link_state state of the link to set. 
         * 
         * @return bool: Setting result
         */
        bool setLink(const std::string& link_name, const hrii::ergonomics::Link& link);

        /**
         * @brief Set the link pose of a desired link. If it does not exist, it will be add to the map link_name-link_state
         * 
         * @param link_name name of the link to set. 
         * @param link_pos link position vector.
         * @param link_orient link quaternions. 
         * 
         * @return bool: Setting result
         */
        bool setLinkPose(const std::string& link_name, const Eigen::Vector3d& link_pos, const Eigen::Quaterniond& link_orient);

        /**
         * @brief Set the link state of a desired link. If it does not exist, it will be add to the map link_name-link_state
         * 
         * @param link_name name of the link to set. 
         * @param link_state link state. 
         * 
         * @return bool: Setting result
         */
        bool setLinkState(const std::string& link_name, const hrii::ergonomics::LinkState& link_state);

        /**
         * @brief Get the state of a desired link. If it does not exist it return false;
         * 
         * @param link_name name of the link to get.
         * 
         * @return bool: Getting result
         */
        bool getLink(const std::string& link_name, hrii::ergonomics::Link& link);

        // /**
        //  * @brief Get the link name list;
        //  * 
        //  * @return std::vector<std::string>: Getting link name list
        //  */
        // std::vector<std::string> getLinkNames();

        // /**
        //  * @brief Get the joint name list;
        //  * 
        //  * @return std::vector<std::string>: Getting joint name list
        //  */
        // std::vector<std::string> getJointNames();

        /**
         * @brief Get the link map;
         * 
         * @return std::vector<std::string>: Getting link map
         */
        std::map<std::string, hrii::ergonomics::Link> getLinks();

        /**
         * @brief Get the joint map;
         * 
         * @return std::vector<std::string>: Getting joint map
         */
        std::map<std::string, hrii::ergonomics::Joint> getJoints();

        /**
         * @brief Get the center of mass;
         * 
         * @return Eigen::Vector3d: Getting the center of mass
         */
        Eigen::Vector3d getCOM();

        /**
         * @brief Set the center of mass;
         * 
         * @param Eigen::Vector3d: center of mass position with respect to the reference frame
         */
        void setCOM(const Eigen::Vector3d& com);


    private:

        std::map<std::string, hrii::ergonomics::Joint> joint_map_;
        std::map<std::string, hrii::ergonomics::Link> link_map_;

        Eigen::Vector3d com_;

        std::vector<std::string> link_name_list_, joint_name_list_;


};

} // namespace ergonomics

} // namespace hrii

#endif //_XSENS_MVN_ROS_HUMAN_DATA_HANDLER_H_