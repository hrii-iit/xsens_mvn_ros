Setup
=====

In order to stream data through the ROS middleware, install MVN Studio software to stream the data from a Windows computer. An Linux Ubuntu client computer is required to receive the streamed data and publish it to a ROS network. 

.. note::

   The Windows server computer and the Ubuntu client computer have to be on the same network.

.. _setup-server:

Server (Microsoft Windows computer)
-----------------------------------

Install `MVN Awinda Studio 2023 software <https://www.xsens.com/products/mtw-awinda>`_.

If you need further information about the MVN software, please download the `MVN User Manual <https://www.xsens.com/hubfs/Downloads/usermanual/MVN_User_Manual.pdf>`_.

Open a *Live Configuration* by clicking on "File -> Edit Live Configuration..."
Load the desired body measurements.

.. image:: ./imgs/mvn_studio/awinda_active.png
  :width: 500
  :alt: 

After the calibration, you should be able to visualise the tracked human as in the following picture: 

.. image:: ./imgs/mvn_studio/awinda_skeleton_result.png
  :width: 500
  :alt: 

Open the *Network Streamer* window, add a new option and select to the following options:

#. Position + Orientation (Quaternion)
#. Linear Segment Kinematics
#. Angular Segment Kinematics
#. Time Code
#. Center of Mass
#. Joint Angles

Select UDP as protocol and 8001 as port.

.. image:: ./imgs/mvn_studio/network_streamer.png
  :width: 500
  :alt:

.. _setup-client:

Client (Linux Ubuntu computer)
------------------------------

Requirements:

#. GNU/Linux Ubuntu 20.04
#. `ROS Noetic <http://wiki.ros.org/noetic>`_
#. `Git <https://git-scm.com/download/linux>`_
#. `Catkin Tools <https://catkin-tools.readthedocs.io/en/latest/installing.html>`_

On your Linux Ubuntu machine install ROS noetic and git.

Execute the following steps to create a catkin workspace, clone the `xsens_mvn_ros <https://github.com/hrii-iit/xsens_mvn_ros>`_ ROS package and compile it.

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
