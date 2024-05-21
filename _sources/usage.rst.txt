Usage
=====

In order to stream data through the ROS middleware, install MVN Studio software to stream the data from a Windows computer. An Linux Ubuntu client computer is required to receive the streamed data and publish it to a ROS network. 

Server (Microsoft Windows computer)
-----------------------------------

Open the MVN Studio 2023 software on your Windows computer and prepare the server software as described in the setup section of this guide.

Client (Linux Ubuntu computer)
------------------------------

Before start the client, set it up as described in the setup section of this guide.

#. Change directory to your workspace and source the setup.bash on your workspace devel folder:

    .. code:: bash

        cd path_to_catkin_ws
        source devel/setup.bash

#. Run the launch file as following:

    .. code:: bash

        reset; ros launch xsens_mvn_ros xsens_client.launch