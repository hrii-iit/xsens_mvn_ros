#ifndef _XSENS_MVN_ROS_MODEL_H_
#define _XSENS_MVN_ROS_MODEL_H_

#include <vector>
#include <string>

struct XSensModelNames
{
    const std::vector<std::string> links = {"base_link",
                                            "pelvis",
                                            "l5",
                                            "l3",
                                            "t12",
                                            "t8",
                                            "neck",
                                            "head",
                                            "right_shoulder",
                                            "right_upper_arm",
                                            "right_forearm",
                                            "right_hand",
                                            "left_shoulder",
                                            "left_upper_arm",
                                            "left_forearm",
                                            "left_hand",
                                            "right_upper_leg",
                                            "right_lower_leg",
                                            "right_foot",
                                            "right_toe",
                                            "left_upper_leg",
                                            "left_lower_leg",
                                            "left_foot",
                                            "left_toe",
                                            "generic_link"};

    const std::vector<std::string> joints = {"l5_s1",
                                            "l4_l3",
                                            "l1_t12",
                                            "t9_t8",
                                            "t1_c7",
                                            "c1_head",
                                            "right_c1_shoulder",
                                            "right_shoulder",
                                            "right_elbow",
                                            "right_wrist",
                                            "left_c1_shoulder",
                                            "left_shoulder",
                                            "left_elbow",
                                            "left_wrist",
                                            "right_hip",
                                            "right_knee",
                                            "right_ankle",
                                            "right_ballfoot",
                                            "left_hip",
                                            "left_knee",
                                            "left_ankle",
                                            "left_ballfoot",
                                            "t8_head_NA",
                                            "t8_left_upper_arm_NA",
                                            "t8_right_upper_arm_NA",
                                            "pelvis_t8_NA",
                                            "pelvis_pelvis_NA",
                                            "pelvis_t8_v2_NA"};

};

#endif //_XSENS_MVN_ROS_MODEL_H_