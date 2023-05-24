# Repo: xsens_awinda_ros
xsens_awinda_ros contains the relevant packages for the sxens software development tools. (alongside with the ROS middleware)

**NOTE**: refer to the dedicated README files of the packages for further information about their use cases.

## Table of Contents
- [Dependencies](#dependencies)
    - [Server side](#server-side)
    - [Client side](#client-side)
- [Build](#build)
- [Kinematics chain: indexes and definition](#definitions)

## Dependencies
### Server side
- Microsoft Windows
- [MVN Link or Awinda](https://www.xsens.com/products/mtw-awinda)

### Client side
- GNU/Linux Ubuntu 20.04
- ROS Noetic
- Git

## Build
1- Open the terminal emulator and change the current directory to the **src** directory of **catkin_ws** as follows:
```
cd path_to_catkin_ws/src
```

2- Clone the Git repository:

```
git clone git@github.com:hrii-iit/xsens_awinda_ros.git
# If you use https: git clone https://github.com/hrii-iit/xsens_awinda_ros.git
```

3- Build the catkin workspace:

```
cd path_to_catkin_ws
catkin_make
```
## Kinematics chain: indexes and definition
