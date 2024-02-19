Build
=====

Create a Catkin workspace
-------------------------

Execute the following steps to create a catkin workspace, clone the xsens_mvn_ros ROS package and compile it.

#. Create a catkin workspace and change directory to the source code:

    .. code:: bash

        mkdir -p path_to_catkin_ws/src
        cd path_to_catkin_ws/src


#. Clone the xsens_mvn_ros repository:

    .. code:: bash

        git clone https://github.com/hrii-iit/xsens_mvn_ros.git --recursive

#. Build your catkin workspace

    .. code:: bash

        cd path_to_catkin_ws
        catkin build
