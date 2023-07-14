#ifndef __XSENS_MVN_CLIENT_H__
#define __XSENS_MVN_CLIENT_H__

#define MAX_MVN_DATAGRAM_SIZE 5000

#include <boost/thread.hpp>
#include <xsens_mvn_ros/Socket.h>
#include <xsens_mvn_ros/HumanDataHandler.h>
#include <xsens_mvn_sdk/parsermanager.h>
// #include "urdf_parser/urdf_parser.h"

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
        std::vector<std::string> link_name_list_, joint_name_list_;
        
        boost::thread data_acquisition_thread_;
        void dataAcquisitionCallback();
        char data_buffer_[MAX_MVN_DATAGRAM_SIZE];
        bool client_active_;
        
        bool buildXSensModel();
        QuaternionDatagram waitForQuaternionDatagram();
        JointAnglesDatagram waitForJointAnglesDatagram();

        bool readData();
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
};

#endif // __XSENS_MVN_CLIENT_H__