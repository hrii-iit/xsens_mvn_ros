#ifndef __XSENS_MVN_CLIENT_H__
#define __XSENS_MVN_CLIENT_H__

#define MAX_MVN_DATAGRAM_SIZE 5000

#include <boost/thread.hpp>
#include <xsens_mvn_ros/Socket.h>
#include <xsens_mvn_ros/HumanDataHandler.h>
#include <xsens_mvn_sdk/parsermanager.h>
#include "urdf_parser/urdf_parser.h"

class XSensClient
{
    public:
        XSensClient(const int& udp_port);
        virtual ~XSensClient();
        bool init();

        hrii::ergonomics::HumanDataHandler::Ptr getHumanData();

        std::string getURDFString();

    private:
        int udp_port_;
        boost::shared_ptr<Socket> udp_socket_;
        ParserManager parser_manager_;
        hrii::ergonomics::HumanDataHandler::Ptr human_data_;
        urdf::ModelInterfaceSharedPtr urdf_model_;
        std::vector<std::string> link_name_list_, joint_name_list_;
        std::string urdf_str_;
        
        char data_buffer_[MAX_MVN_DATAGRAM_SIZE];

        boost::thread data_acquisition_thread_;
        void dataAcquisitionCallback();
        bool client_active_;

        bool readData();
        bool buildHumanModel();
        void fromHumanModelToURDF();
        void updateJointAngles();
        void updateLinkPoses();
        void updateLinkLinearTwists();
        void updateLinkAngularTwists();
        void updateCOM();

        Eigen::Vector3d jointAngleToEigenVector3d(const JointAngle& joint_angle, 
                                                const double& x_axis, 
                                                const double& y_axis, 
                                                const double& z_axis);
        void rotateLink(const std::string& link_name, const Eigen::Quaterniond& quat);
        // std::string jointToUrdf(const std::string& name, const std::string& type, const std::string& axis,
        //                 const std::string& xyz, const std::string& rpy, const std::string& child_link,
        //                 const std::string& parent_link, const std::string& effort, const std::string& velocity,
        //                 const std::string& lower_limit, const std::string& upper_limit);
        // std::string linkToUrdf(const std::string& name, const std::string& mass, const std::string& axis,
        //                 const std::string& xyz, const std::string& rpy, const std::string& child_link,
        //                 const std::string& parent_link, const std::string& effort, const std::string& velocity,
        //                 const std::string& lower_limit, const std::string& upper_limit);

};

#endif // __XSENS_MVN_CLIENT_H__