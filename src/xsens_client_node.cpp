#include <xsens_awinda_ros/XSensClient.h>

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <eigen_conversions/eigen_msg.h>

#include <sensor_msgs/JointState.h>
#include <xsens_awinda_ros/LinkStateArray.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "xsens_client");

    ros::NodeHandle nh("~");

    tf2_ros::TransformBroadcaster tf_broadcaster;

    // Data parser initialization
    std::string model_name, reference_frame;
    nh.param<std::string>("model_name", model_name, "skeleton");
    nh.param<std::string>("reference_frame", reference_frame, "world");
    int xsens_udp_port;
    nh.param<int>("udp_port", xsens_udp_port, 8001);

    // ROS publishers
    ros::Publisher joint_state_publisher = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
    ros::Publisher link_state_publisher = nh.advertise<xsens_awinda_ros::LinkStateArray>("link_states", 10);
    ros::Publisher com_publisher = nh.advertise<geometry_msgs::Point>("com", 10);

    boost::shared_ptr<XSensClient> xsens_client_ptr;
    try
    {
        xsens_client_ptr = boost::make_shared<XSensClient>(xsens_udp_port);
    }
    catch(const std::exception& err)
    {
        ROS_ERROR_STREAM(err.what());
        return -1;
    }
    
    if(!xsens_client_ptr->init())
    {
        ROS_ERROR_STREAM("XSens client initialization failed.");
        return -1;
    }
    
    ros::Rate loop_rate(120);

    while(ros::ok())
    {
        // Publish joint state
        if (joint_state_publisher.getNumSubscribers() > 0)
        {
            sensor_msgs::JointState joint_state_msg;
            joint_state_msg.header.stamp = ros::Time::now();

            auto joints = xsens_client_ptr->getHumanData()->getJoints();
            for (std::map<std::string, hrii::ergonomics::Joint>::iterator joint_it = joints.begin(); joint_it != joints.end(); joint_it++)
            {
                joint_state_msg.name.push_back(model_name+"_"+joint_it->first+"_x");
                joint_state_msg.name.push_back(model_name+"_"+joint_it->first+"_y");
                joint_state_msg.name.push_back(model_name+"_"+joint_it->first+"_z");
                joint_state_msg.position.push_back(joint_it->second.state.angles[0]/180*3.1415);
                joint_state_msg.position.push_back(joint_it->second.state.angles[1]/180*3.1415);
                joint_state_msg.position.push_back(joint_it->second.state.angles[2]/180*3.1415);
            }
            
            joint_state_publisher.publish(joint_state_msg);
        }
        
        // Publish link tf and state
        xsens_awinda_ros::LinkStateArray link_state_msg;
        auto links = xsens_client_ptr->getHumanData()->getLinks();
        for (std::map<std::string, hrii::ergonomics::Link>::iterator link_it = links.begin(); link_it != links.end(); link_it++)
        {
            // Publish link tf
            if (!(link_it->second.state.orientation.x() == 0 && link_it->second.state.orientation.y() == 0 
                && link_it->second.state.orientation.z() == 0 && link_it->second.state.orientation.w()))
            {
                geometry_msgs::TransformStamped transform_stamped;
                transform_stamped.header.stamp = ros::Time::now();
                transform_stamped.header.frame_id = reference_frame;
                transform_stamped.child_frame_id = link_it->first;
                transform_stamped.transform.translation.x = link_it->second.state.position[0];
                transform_stamped.transform.translation.y = link_it->second.state.position[1];
                transform_stamped.transform.translation.z = link_it->second.state.position[2];
                transform_stamped.transform.rotation.x = link_it->second.state.orientation.x();
                transform_stamped.transform.rotation.y = link_it->second.state.orientation.y();
                transform_stamped.transform.rotation.z = link_it->second.state.orientation.z();
                transform_stamped.transform.rotation.w = link_it->second.state.orientation.w();
                tf_broadcaster.sendTransform(transform_stamped);
            }

            if (link_state_publisher.getNumSubscribers() > 0)
            {
                // Publish link state
                xsens_awinda_ros::LinkState link_state;
                link_state.header.frame_id = link_it->first;
                link_state.header.stamp = ros::Time::now();
                tf::pointEigenToMsg(link_it->second.state.position, link_state.pose.position);
                tf::quaternionEigenToMsg(link_it->second.state.orientation, link_state.pose.orientation);
                Eigen::Matrix<double, 6, 1> link_twist;
                link_twist << link_it->second.state.velocity.linear, link_it->second.state.velocity.angular;
                tf::twistEigenToMsg(link_twist , link_state.twist);
                link_state.accel.linear.x = link_it->second.state.acceleration.linear[0];
                link_state.accel.linear.y = link_it->second.state.acceleration.linear[1];
                link_state.accel.linear.z = link_it->second.state.acceleration.linear[2];
                link_state.accel.angular.x = link_it->second.state.acceleration.angular[0];
                link_state.accel.angular.y = link_it->second.state.acceleration.angular[1];
                link_state.accel.angular.z = link_it->second.state.acceleration.angular[2];

                link_state_msg.states.push_back(link_state);
            }

        }

        if (link_state_publisher.getNumSubscribers() > 0)
            link_state_publisher.publish(link_state_msg);

        
        if (com_publisher.getNumSubscribers() > 0)
        {
            geometry_msgs::Point com_msg;
            tf::pointEigenToMsg(xsens_client_ptr->getHumanData()->getCOM(), com_msg);
            com_publisher.publish(com_msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
