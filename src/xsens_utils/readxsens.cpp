#include "xsens_utils/readxsens.h"

ReadXsens::ReadXsens(int port, std::string Stream_IP, std::string Local_IP)
 : jointAngles_(NULL), XsensCOM(NULL)
{

    parserManager.reset(new ParserManager(false, false));
    socket.reset(new Socket(IP_UDP));
    hostDestinationAddress = Stream_IP;

    bool res = socket->bind(port);

    std::cout << "Waiting to receive packets from Xense ..." << std::endl << std::endl;

    if (res == false){
        std::cout << "error binging xsens port" << std::endl;
    }
    else
    {
        /* Initialize the variables */
        XsensTimeCode = TimeCode();
        XsensAngle = AngleJoint();
        XsensAngle_prev = AngleJoint3D();
        XsensAngle3D = AngleJoint3D();
        XsensAngleMatrix = Rotation3f();
        XsensAngleMatrix3D = Rotation3f();
        qKinematics_ =  qKinematics();
        linearSegtKinematics_ = linearSegtKinematics();
        angSegtKinematics_ = angSegtKinematics();
        XsensPosition = JointPosition();
        XsensQuaternion = QuaternionJoint();
        XsensQuaternionMatrix = Rotation3f();
        XsensLength = LinkLength();
        XsensSegLinearVelocity = SegmentLinearVelocity();
        XsensSegLinearAcceleration = SegmentLinearAcceleration();
        XsensSegAngularVelocity = SegmentAngularVelocity();
        XsensSegAngularAcceleration = SegmentAngularAcceleration();
        XsensJointVelocity = VelocityJoint();
        XsensJointVelocity_deriv = VelocityJoint();
        XsensJointVelocity_prev = VelocityJoint();
        XsensJointVelocity_deriv_prev = VelocityJoint();
        XsensJointAcceleration = AccelerationJoint();
        XsensJointAcceleration_deriv = AccelerationJoint();

        for (int i = 0; i < 3; i++){
            CalValue[i] = 0.0;
        }
        count_c = 0;
        Cal = false;
        OptitrackFlag = false;
        OptitrackPos << 0.0, 0.0, 0.0;

        XsensThread = new std::thread(&ReadXsens::BufferReadXsens, this);
    }

    /* Filter */
    for (int i = 0; i < DOF; i++){
        xv0[i] = 0.0; xv1[i] = 0.0; xv2[i] = 0.0;
        yv0[i] = 0.0; yv1[i] = 0.0; yv2[i] = 0.0;
    }
    for (int i = 0; i < DOF; i++){
        xd0[i] = 0.0; xd1[i] = 0.0; xd2[i] = 0.0;
        yd0[i] = 0.0; yd1[i] = 0.0; yd2[i] = 0.0;
    }
    for (int i = 0; i < DOF; i++){
        xc0[i] = 0.0; xc1[i] = 0.0; xc2[i] = 0.0;
        yc0[i] = 0.0; yc1[i] = 0.0; yc2[i] = 0.0;
    }

    TimeStep = 0;
    check_optitrack_ = false;
}

ReadXsens::~ReadXsens(){
}

void ReadXsens::BufferReadXsens(){
    std::cout << "[XSENS] Start reading." << std::endl << std::endl;
    while(true)
    {
        int rv = socket->read(buffer, MAX_MVN_DATAGRAM_SIZE);

        if (rv > 0)
        {
            parserManager->readDatagram(buffer);
        }

        std::memset(buffer, 0, MAX_MVN_DATAGRAM_SIZE);

        jointAngles_ = parserManager->getJointAnglesDatagram();
        quaternionDatagram_ = parserManager->getQuaternionDatagram();
        LinearSegKinDatagram_ = parserManager->getLinearSegmentKinematicsDatagram();
        AngSegKinDatagram_ = parserManager->getAngularSegmentKinematicsDatagram();

        com_ = parserManager->getCenterOfMassDatagram();
        time_code_ = parserManager->getTimeCodeDatagram();

        if (com_ != NULL)
        {
            XsensCOM = com_->getData();
            XsensCoM_ << XsensCOM[0], XsensCOM[1], XsensCOM[2];
        }
        if (time_code_ != NULL)
        {
            XsensTimeCode = time_code_->getData();
        }

        if ((quaternionDatagram_ != NULL) || (jointAngles_ != NULL) || (LinearSegKinDatagram_ != NULL) || (AngSegKinDatagram_ != NULL))
        {

            if (jointAngles_ != NULL)
            {
                XsensAngle_prev = XsensAngle3D;
                XsensAngle = ReadXsensAngle();
                XsensAngle3D = ReadXsensAngle3D();
            }
            else if (quaternionDatagram_ != NULL)
            {
                qKinematics_ = ReadXsensQuaternion();
            }
            else if (LinearSegKinDatagram_ != NULL)
            {
                linearSegtKinematics_= ReadXsensLinearKinematics();
            }
            else if (AngSegKinDatagram_ != NULL)
            {
                angSegtKinematics_ = ReadXsensAngularKinematics();
            }

            XsensPosition = WriteXsensPosition(qKinematics_);

            XsensQuaternion = WriteXsensQuaternion(qKinematics_);
            XsensQuaternionMatrix = WriteXsensQuaternionMatrix(XsensQuaternion);

            XsensSegLinearVelocity = WriteXsensLinearVelocity(linearSegtKinematics_);
            XsensSegLinearAcceleration = WriteXsensLinearAcceleration(linearSegtKinematics_);
            XsensSegAngularVelocity = WriteXsensAngularVelocity(angSegtKinematics_);
            XsensSegAngularAcceleration = WriteXsensAngularAcceleration(angSegtKinematics_);

            // Define Tool position (mid of the hand palm)
            Vector3f Tool_pos; Tool_pos << 0.0, 0.0, -0.1;
            XsensPosition.LeftTool_Pos = XsensPosition.LeftWrist_Pos + XsensQuaternionMatrix.A13*Tool_pos;
            XsensPosition.RightTool_Pos = XsensPosition.RightWrist_Pos + XsensQuaternionMatrix.A16*Tool_pos;

            // PREVIOUS VERSION OF q0 calculations
            // Compute base joint angles from quaternion Q0
            // Vector3f q0_rpy = XsensQuaternionMatrix.A0.eulerAngles(0, 1, 2);  // Roll, Pitch, Yaw
            // XsensAngle3D.q0_x = q0_rpy[0];
            // XsensAngle3D.q0_y = q0_rpy[1];
            // XsensAngle3D.q0_z = q0_rpy[2];
//                 std::cout <<  "q0  = " << RAD2DEG(XsensAngle3D.q0_x) << "\t" << RAD2DEG(XsensAngle3D.q0_y) << "\t" << RAD2DEG(XsensAngle3D.q0_z)  << std::endl;

            Vector3d q0_Q_d;
            q0_Q_d = getRPY(XsensQuaternionMatrix.A0.cast<double>());
            Vector3f q0_Q;
            q0_Q = q0_Q_d.cast<float>();
            XsensAngle3D.q0_x = q0_Q[0];
            XsensAngle3D.q0_y = q0_Q[1];
            XsensAngle3D.q0_z = q0_Q[2];

            XsensAngleMatrix = WriteXsensAngleMatrix(XsensAngle);
            XsensAngleMatrix3D = WriteXsensAngleMatrix3D(XsensAngle3D);

            // Check matrices
//             std::cout <<  "QuaternionMatrix  = " << std::endl;
//             std::cout <<  "   " << XsensQuaternionMatrix.A14(0,0) << "\t" << XsensQuaternionMatrix.A14(0,1) << "\t" << XsensQuaternionMatrix.A14(0,2) << std::endl;
//             std::cout <<  "   " << XsensQuaternionMatrix.A14(1,0) << "\t" << XsensQuaternionMatrix.A14(1,1) << "\t" << XsensQuaternionMatrix.A14(1,2) << std::endl;
//             std::cout <<  "   " << XsensQuaternionMatrix.A14(2,0) << "\t" << XsensQuaternionMatrix.A14(2,1) << "\t" << XsensQuaternionMatrix.A14(2,2) << std::endl;
//
//             std::cout <<  "AngleMatrix3D  = " << std::endl;
//             std::cout <<  "   " << XsensAngleMatrix3D.A14(0,0) << "\t" << XsensAngleMatrix3D.A14(0,1) << "\t" << XsensAngleMatrix3D.A14(0,2) << std::endl;
//             std::cout <<  "   " << XsensAngleMatrix3D.A14(1,0) << "\t" << XsensAngleMatrix3D.A14(1,1) << "\t" << XsensAngleMatrix3D.A14(1,2) << std::endl;
//             std::cout <<  "   " << XsensAngleMatrix3D.A14(2,0) << "\t" << XsensAngleMatrix3D.A14(2,1) << "\t" << XsensAngleMatrix3D.A14(2,2) << std::endl;

            // Compute joint velocities from segment linear and angular velocities
            XsensJointVelocity = computeJointVelocity(XsensQuaternionMatrix, XsensPosition, XsensSegLinearVelocity, XsensSegAngularVelocity);

            // Compute joint velocities through derivative
//             TimeStep = 0.004167*5; //240Hz/5 because 4 data are sended in series
            TimeStep = 0.01667*2;
            VelocityJoint jointVelocity_tmp;
            jointVelocity_tmp.qdot_0  = (XsensAngle3D.q0_y  - XsensAngle_prev.q0_y )/TimeStep;
            jointVelocity_tmp.qdot_1  = (XsensAngle3D.q1_y  - XsensAngle_prev.q1_y )/TimeStep;
            jointVelocity_tmp.qdot_2  = (XsensAngle3D.q2_y  - XsensAngle_prev.q2_y )/TimeStep;
            jointVelocity_tmp.qdot_3  = (XsensAngle3D.q3_y  - XsensAngle_prev.q3_y )/TimeStep;
            jointVelocity_tmp.qdot_3t = (XsensAngle3D.q3t_y - XsensAngle_prev.q3t_y)/TimeStep;
            jointVelocity_tmp.qdot_4  = (XsensAngle3D.q4_y  - XsensAngle_prev.q4_y )/TimeStep;
            jointVelocity_tmp.qdot_5  = (XsensAngle3D.q5_y  - XsensAngle_prev.q5_y )/TimeStep;
            jointVelocity_tmp.qdot_6  = (XsensAngle3D.q6_y  - XsensAngle_prev.q6_y )/TimeStep;
            jointVelocity_tmp.qdot_6t = (XsensAngle3D.q6t_y - XsensAngle_prev.q6t_y)/TimeStep;
            jointVelocity_tmp.qdot_7  = (XsensAngle3D.q7_y  - XsensAngle_prev.q7_y )/TimeStep;
            jointVelocity_tmp.qdot_8  = (XsensAngle3D.q8_y  - XsensAngle_prev.q8_y )/TimeStep;
            jointVelocity_tmp.qdot_9  = (XsensAngle3D.q9_y  - XsensAngle_prev.q9_y )/TimeStep;
            jointVelocity_tmp.qdot_10 = (XsensAngle3D.q10_y - XsensAngle_prev.q10_y)/TimeStep;
            jointVelocity_tmp.qdot_11 = (XsensAngle3D.q11_y - XsensAngle_prev.q11_y)/TimeStep;
            jointVelocity_tmp.qdot_12 = (XsensAngle3D.q12_y - XsensAngle_prev.q12_y)/TimeStep;
            jointVelocity_tmp.qdot_13 = (XsensAngle3D.q13_y - XsensAngle_prev.q13_y)/TimeStep;
            jointVelocity_tmp.qdot_14 = (XsensAngle3D.q14_y - XsensAngle_prev.q14_y)/TimeStep;
            jointVelocity_tmp.qdot_15 = (XsensAngle3D.q15_y - XsensAngle_prev.q15_y)/TimeStep;
            jointVelocity_tmp.qdot_16 = (XsensAngle3D.q16_y - XsensAngle_prev.q16_y)/TimeStep;
            jointVelocity_tmp.qdot_17 = (XsensAngle3D.q17_y - XsensAngle_prev.q17_y)/TimeStep;
            jointVelocity_tmp.qdot_18 = (XsensAngle3D.q18_y - XsensAngle_prev.q18_y)/TimeStep;

            // Filtering the q_dot values
            float JointVel[DOF];
            JointVel[0]  = jointVelocity_tmp.qdot_0;   JointVel[1]  = jointVelocity_tmp.qdot_1;   JointVel[2] = jointVelocity_tmp.qdot_2;
            JointVel[3]  = jointVelocity_tmp.qdot_3;   JointVel[4]  = jointVelocity_tmp.qdot_3t;  JointVel[5] = jointVelocity_tmp.qdot_4;
            JointVel[6]  = jointVelocity_tmp.qdot_5;   JointVel[7]  = jointVelocity_tmp.qdot_6;   JointVel[8] = jointVelocity_tmp.qdot_6t;
            JointVel[9]  = jointVelocity_tmp.qdot_7;   JointVel[10] = jointVelocity_tmp.qdot_8;  JointVel[11] = jointVelocity_tmp.qdot_9;
            JointVel[12] = jointVelocity_tmp.qdot_10;  JointVel[13] = jointVelocity_tmp.qdot_11; JointVel[14] = jointVelocity_tmp.qdot_12;
            JointVel[15] = jointVelocity_tmp.qdot_13;  JointVel[16] = jointVelocity_tmp.qdot_14; JointVel[17] = jointVelocity_tmp.qdot_15;
            JointVel[18] = jointVelocity_tmp.qdot_16;  JointVel[19] = jointVelocity_tmp.qdot_17; JointVel[20] = jointVelocity_tmp.qdot_18;
            float JointVel_filt[DOF];
            for (int i = 0; i < DOF; i++){
                JointVel_filt[i] = 0.0;
            }
            float fac1, fac2;
//             fac1 = -0.9115944966; fac2 = 1.9075016260; // 240/5=48 Hz cut: 0.5Hz
            fac1 = -0.9636529842; fac2 = 1.9629800894; // 240/5=48 Hz cut: 0.2Hz
            for (int i = 0; i < DOF; i++){
                xv0[i] = xv1[i]; xv1[i] = xv2[i];
                xv2[i] = JointVel[i] / F_GAIN;
                yv0[i] = yv1[i]; yv1[i] = yv2[i];
                yv2[i] =   (xv0[i] + xv2[i]) + 2 * xv1[i] + ( fac1 * yv0[i]) + (  fac2 * yv1[i]);
                JointVel_filt[i] = yv2[i];
//                 JointVel_filt[i] = JointVel[i];
            }
            XsensJointVelocity_prev = XsensJointVelocity_deriv;

            XsensJointVelocity_deriv.qdot_0  = JointVel_filt[0];
            XsensJointVelocity_deriv.qdot_1  = JointVel_filt[1];
            XsensJointVelocity_deriv.qdot_2  = JointVel_filt[2];
            XsensJointVelocity_deriv.qdot_3  = JointVel_filt[3];
            XsensJointVelocity_deriv.qdot_3t = JointVel_filt[4];
            XsensJointVelocity_deriv.qdot_4  = JointVel_filt[5];
            XsensJointVelocity_deriv.qdot_5  = JointVel_filt[6];
            XsensJointVelocity_deriv.qdot_6  = JointVel_filt[7];
            XsensJointVelocity_deriv.qdot_6t = JointVel_filt[8];
            XsensJointVelocity_deriv.qdot_7  = JointVel_filt[9];
            XsensJointVelocity_deriv.qdot_8  = JointVel_filt[10];
            XsensJointVelocity_deriv.qdot_9  = JointVel_filt[11];
            XsensJointVelocity_deriv.qdot_10 = JointVel_filt[12];
            XsensJointVelocity_deriv.qdot_11 = JointVel_filt[13];
            XsensJointVelocity_deriv.qdot_12 = JointVel_filt[14];
            XsensJointVelocity_deriv.qdot_13 = JointVel_filt[15];
            XsensJointVelocity_deriv.qdot_14 = JointVel_filt[16];
            XsensJointVelocity_deriv.qdot_15 = JointVel_filt[17];
            XsensJointVelocity_deriv.qdot_16 = JointVel_filt[18];
            XsensJointVelocity_deriv.qdot_17 = JointVel_filt[19];
            XsensJointVelocity_deriv.qdot_18 = JointVel_filt[20];

            // Compute joint acceleration through derivative
            AccelerationJoint jointAcceleration_tmp;
            jointAcceleration_tmp.qddot_0  = (XsensJointVelocity_deriv.qdot_0  - XsensJointVelocity_prev.qdot_0 )/TimeStep;
            jointAcceleration_tmp.qddot_1  = (XsensJointVelocity_deriv.qdot_1  - XsensJointVelocity_prev.qdot_1 )/TimeStep;
            jointAcceleration_tmp.qddot_2  = (XsensJointVelocity_deriv.qdot_2  - XsensJointVelocity_prev.qdot_2 )/TimeStep;
            jointAcceleration_tmp.qddot_3  = (XsensJointVelocity_deriv.qdot_3  - XsensJointVelocity_prev.qdot_3 )/TimeStep;
            jointAcceleration_tmp.qddot_3t = (XsensJointVelocity_deriv.qdot_3t - XsensJointVelocity_prev.qdot_3t)/TimeStep;
            jointAcceleration_tmp.qddot_4  = (XsensJointVelocity_deriv.qdot_4  - XsensJointVelocity_prev.qdot_4 )/TimeStep;
            jointAcceleration_tmp.qddot_5  = (XsensJointVelocity_deriv.qdot_5  - XsensJointVelocity_prev.qdot_5 )/TimeStep;
            jointAcceleration_tmp.qddot_6  = (XsensJointVelocity_deriv.qdot_6  - XsensJointVelocity_prev.qdot_6 )/TimeStep;
            jointAcceleration_tmp.qddot_6t = (XsensJointVelocity_deriv.qdot_6t - XsensJointVelocity_prev.qdot_6t)/TimeStep;
            jointAcceleration_tmp.qddot_7  = (XsensJointVelocity_deriv.qdot_7  - XsensJointVelocity_prev.qdot_7 )/TimeStep;
            jointAcceleration_tmp.qddot_8  = (XsensJointVelocity_deriv.qdot_8  - XsensJointVelocity_prev.qdot_8 )/TimeStep;
            jointAcceleration_tmp.qddot_9  = (XsensJointVelocity_deriv.qdot_9  - XsensJointVelocity_prev.qdot_9 )/TimeStep;
            jointAcceleration_tmp.qddot_10 = (XsensJointVelocity_deriv.qdot_10 - XsensJointVelocity_prev.qdot_10)/TimeStep;
            jointAcceleration_tmp.qddot_11 = (XsensJointVelocity_deriv.qdot_11 - XsensJointVelocity_prev.qdot_11)/TimeStep;
            jointAcceleration_tmp.qddot_12 = (XsensJointVelocity_deriv.qdot_12 - XsensJointVelocity_prev.qdot_12)/TimeStep;
            jointAcceleration_tmp.qddot_13 = (XsensJointVelocity_deriv.qdot_13 - XsensJointVelocity_prev.qdot_13)/TimeStep;
            jointAcceleration_tmp.qddot_14 = (XsensJointVelocity_deriv.qdot_14 - XsensJointVelocity_prev.qdot_14)/TimeStep;
            jointAcceleration_tmp.qddot_15 = (XsensJointVelocity_deriv.qdot_15 - XsensJointVelocity_prev.qdot_15)/TimeStep;
            jointAcceleration_tmp.qddot_16 = (XsensJointVelocity_deriv.qdot_16 - XsensJointVelocity_prev.qdot_16)/TimeStep;
            jointAcceleration_tmp.qddot_17 = (XsensJointVelocity_deriv.qdot_17 - XsensJointVelocity_prev.qdot_17)/TimeStep;
            jointAcceleration_tmp.qddot_18 = (XsensJointVelocity_deriv.qdot_18 - XsensJointVelocity_prev.qdot_18)/TimeStep;

            // Filtering the q_ddot values
            float JointAcc[DOF];
            JointAcc[0]  = jointAcceleration_tmp.qddot_0;  JointAcc[1]  = jointAcceleration_tmp.qddot_1;   JointAcc[2] = jointAcceleration_tmp.qddot_2;
            JointAcc[3]  = jointAcceleration_tmp.qddot_3;  JointAcc[4]  = jointAcceleration_tmp.qddot_3t;  JointAcc[5] = jointAcceleration_tmp.qddot_4;
            JointAcc[6]  = jointAcceleration_tmp.qddot_5;  JointAcc[7]  = jointAcceleration_tmp.qddot_6;   JointAcc[8] = jointAcceleration_tmp.qddot_6t;
            JointAcc[9]  = jointAcceleration_tmp.qddot_7;  JointAcc[10] = jointAcceleration_tmp.qddot_8;  JointAcc[11] = jointAcceleration_tmp.qddot_9;
            JointAcc[12] = jointAcceleration_tmp.qddot_10; JointAcc[13] = jointAcceleration_tmp.qddot_11; JointAcc[14] = jointAcceleration_tmp.qddot_12;
            JointAcc[15] = jointAcceleration_tmp.qddot_13; JointAcc[16] = jointAcceleration_tmp.qddot_14; JointAcc[17] = jointAcceleration_tmp.qddot_15;
            JointAcc[18] = jointAcceleration_tmp.qddot_16; JointAcc[19] = jointAcceleration_tmp.qddot_17; JointAcc[20] = jointAcceleration_tmp.qddot_18;
            float JointAcc_filt[DOF];
            for (int i = 0; i < DOF; i++){
                JointAcc_filt[i] = 0.0;
            }
            for (int i = 0; i < DOF; i++){
                xd0[i] = xd1[i]; xd1[i] = xd2[i];
                xd2[i] = JointAcc[i] / F_GAIN;
                yd0[i] = yd1[i]; yd1[i] = yd2[i];
                yd2[i] =   (xd0[i] + xd2[i]) + 2 * xd1[i] + ( fac1 * yd0[i]) + (  fac2 * yd1[i]);
                JointAcc_filt[i] = yd2[i];
        //         JointAcc_filt[i] = JointAcc[i];
            }

            XsensJointAcceleration.qddot_0  = JointAcc_filt[0];
            XsensJointAcceleration.qddot_1  = JointAcc_filt[1];
            XsensJointAcceleration.qddot_2  = JointAcc_filt[2];
            XsensJointAcceleration.qddot_3  = JointAcc_filt[3];
            XsensJointAcceleration.qddot_3t = JointAcc_filt[4];
            XsensJointAcceleration.qddot_4  = JointAcc_filt[5];
            XsensJointAcceleration.qddot_5  = JointAcc_filt[6];
            XsensJointAcceleration.qddot_6  = JointAcc_filt[7];
            XsensJointAcceleration.qddot_6t = JointAcc_filt[8];
            XsensJointAcceleration.qddot_7  = JointAcc_filt[9];
            XsensJointAcceleration.qddot_8  = JointAcc_filt[10];
            XsensJointAcceleration.qddot_9  = JointAcc_filt[11];
            XsensJointAcceleration.qddot_10 = JointAcc_filt[12];
            XsensJointAcceleration.qddot_11 = JointAcc_filt[13];
            XsensJointAcceleration.qddot_12 = JointAcc_filt[14];
            XsensJointAcceleration.qddot_13 = JointAcc_filt[15];
            XsensJointAcceleration.qddot_14 = JointAcc_filt[16];
            XsensJointAcceleration.qddot_15 = JointAcc_filt[17];
            XsensJointAcceleration.qddot_16 = JointAcc_filt[18];
            XsensJointAcceleration.qddot_17 = JointAcc_filt[19];
            XsensJointAcceleration.qddot_18 = JointAcc_filt[20];

            // Calibration of position & length
            if (Cal && (count_c == 0))
            {
                //std::cout << "=========================== CALIBRATION =========================" << std::endl;
                for (int i = 0; i < 3; i++)
                {
                    CalValue[i] = qKinematics_.RightToeP_.sensorPos[i] - OptitrackPos(i); // save the first capture data when put the 'c'
                }
                std::cout << "\n- RightToe position  [x, y, z]:\n " 
                    << "[" << qKinematics_.RightToeP_.sensorPos[0] << ",  " 
                    << qKinematics_.RightToeP_.sensorPos[1] << ",  " 
                    << qKinematics_.RightToeP_.sensorPos[2] << "]" << std::endl << std::endl;
                std::cout << "- Optitrack position [x, y, z]:\n " 
                    << "[" << OptitrackPos(0) << ",  " 
                    << OptitrackPos(1) << ",  " 
                    << OptitrackPos(2) << "]" << std::endl << std::endl;
                std::cout << "- Calibration value  [x, y, z]:\n "  
                    << "[" << CalValue[0] << ",  " 
                    << CalValue[1] << ",  " 
                    << CalValue[2] << "]" << std::endl << std::endl;

//                 for (int i = 0; i < 3; i++)
//                 {
//                     CalValue[i] = qKinematics_.RightToeP_.sensorPos[i] - OptitrackPos(i); // save the first capture data when put the 'c'
//                     std::cout << "RightToePos [" << i  << "] : "<< '\t' << qKinematics_.RightToeP_.sensorPos[i] << std::endl;
//                     std::cout << "OptitrackPos [" << i  << "] : "<< '\t' << OptitrackPos(i) << std::endl;
//                     std::cout << "CalValue [" << i  << "] : "<< '\t' << CalValue[i] << std::endl;
//                 }
                count_c ++;
            }
//             if (Cal && (count_c == 1))
//             {
// 
//                 XsensLength = WriteXsensLength(XsensPosition, XsensQuaternionMatrix);
//                 std::cout << "======================================================================" << std::endl;
//                 std::cout << "Length  "          << '\t' << "[x] [y] [z]" << std::endl;
//                 std::cout << "Pelvis = "         << '\t' << XsensLength.Pelvislength.transpose() << std::endl;
//                 std::cout << "Left Pelvis = "    << '\t' << XsensLength.LPVlinklength.transpose() << std::endl;
//                 std::cout << "Left UpperLeg "    << '\t' << XsensLength.LTHlength.transpose() << std::endl;
//                 std::cout << "Left LowerLeg "    << '\t' << XsensLength.LSKlength.transpose() << std::endl;
//                 std::cout << "Left Foot = "      << '\t' << XsensLength.LFTlength.transpose() << std::endl;
//                 std::cout << "Right Pelvis = "   << '\t' << XsensLength.RPVlinklength.transpose() << std::endl;
//                 std::cout << "Right UpperLeg "   << '\t' << XsensLength.RTHlength.transpose() << std::endl;
//                 std::cout << "Right LowerLeg "   << '\t' << XsensLength.RSKlength.transpose() << std::endl;
//                 std::cout << "Right Foot = "     << '\t' << XsensLength.RFTlength.transpose() << std::endl;
//                 std::cout << "L5 = "             << '\t' << XsensLength.L5length.transpose() << std::endl;
//                 std::cout << "L3 = "             << '\t' << XsensLength.L3length.transpose() << std::endl;
//                 std::cout << "T12 = "            << '\t' << XsensLength.T12length.transpose() << std::endl;
//                 std::cout << "T8 = "             << '\t' << XsensLength.T8length.transpose() << std::endl;
//                 std::cout << "Left Clavicle = "  << '\t' << XsensLength.T8LSHlength.transpose() << std::endl;
//                 std::cout << "Left UpperArm = "  << '\t' << XsensLength.LUAlength.transpose() << std::endl;
//                 std::cout << "Left ForeArm = "   << '\t' << XsensLength.LFAlength.transpose() << std::endl;
//                 std::cout << "Left Hand = "      << '\t' << XsensLength.LHlength.transpose() << std::endl;
//                 std::cout << "Right Clavicle = " << '\t' << XsensLength.T8RSHlength.transpose() << std::endl;
//                 std::cout << "Right UpperArm = " << '\t' << XsensLength.RUAlength.transpose() << std::endl;
//                 std::cout << "Right ForeArm = "  << '\t' << XsensLength.RFAlength.transpose() << std::endl;
//                 std::cout << "Right Hand = "     << '\t' << XsensLength.RHlength.transpose() << std::endl;
//                 std::cout << "Neck = "           << '\t' << XsensLength.NKlength.transpose() << std::endl;
//                 std::cout << "Head = "           << '\t' << XsensLength.HDlength.transpose() << std::endl;
//                 std::cout << "======================================================================" << std::endl;
// 
//                 count_c ++;
//             }

        }
    }
}

/*
TimeCode ReadXsens::ReadXsensTimeCode(){
        //struct TimeCode tmp;
    //
    //time_code_->getData();
    //return time_code_->getData();
}
*/

AngleJoint ReadXsens::ReadXsensAngle()const{

    JointAngle Lhip_, Lknee_, Lankle_, Ltoe_, Rhip_, Rknee_, Rankle_, Rtoe_, L5_, L3_, T12_, T8_, Neck_, Head_,
    Lclavicle_, Lshoulder_, Lelbow_, Lwrist_, Rclavicle_, Rshoulder_, Relbow_, Rwrist_;

    // Angle Joint are calculated in the "jointangledatagram.cpp" function getItem(ParentLink, ChildLink) according to XSens MVN manual

    L5_        = jointAngles_->getItem(1, 2);
    L3_        = jointAngles_->getItem(2, 3);
    T12_       = jointAngles_->getItem(3, 4);
    T8_        = jointAngles_->getItem(4, 5);
    Neck_      = jointAngles_->getItem(5, 6);
    Head_      = jointAngles_->getItem(6, 7);
    Rclavicle_ = jointAngles_->getItem(5, 8);
    Rshoulder_ = jointAngles_->getItem(8, 9);
    Relbow_    = jointAngles_->getItem(9, 10);
    Rwrist_    = jointAngles_->getItem(10, 11);
    Lclavicle_ = jointAngles_->getItem(5, 12);
    Lshoulder_ = jointAngles_->getItem(12, 13);
    Lelbow_    = jointAngles_->getItem(13, 14);
    Lwrist_    = jointAngles_->getItem(14, 15);
    Rhip_      = jointAngles_->getItem(1, 16);
    Rknee_     = jointAngles_->getItem(16, 17);
    Rankle_    = jointAngles_->getItem(17, 18);
    Rtoe_      = jointAngles_->getItem(18, 19);
    Lhip_      = jointAngles_->getItem(1, 20);
    Lknee_     = jointAngles_->getItem(20, 21);
    Lankle_    = jointAngles_->getItem(21, 22);
    Ltoe_      = jointAngles_->getItem(22, 23);

    // Transform the axis to obtain an alignment of all of them in resting position (N-pose)
    // Correspondence with rotation [0][0] [1][1] [2][2]
    int LSCaxis[3]  = {1, -1, -1};
    int LSaxis[3]   = {1, -1, -1};
    int LEaxis[3]   = {1, -1, -1};
    int LWaxis[3]   = {1, -1, -1};

    int RSCaxis[3]  = {-1, 1, -1};
    int RSaxis[3]   = {-1, 1, -1};
    int REaxis[3]   = {-1, 1, -1};
    int RWaxis[3]   = {-1, 1, -1};

    int L5axis[3]   = {1, 1, 1};
    int L3axis[3]   = {1, 1, 1};
    int T12axis[3]  = {1, 1, 1};
    int T8axis[3]   = {1, 1, 1};
    int NKaxis[3]   = {1, 1, 1};
    int HDaxis[3]   = {1, 1, 1};

    int LHaxis[3]   = {1, -1, -1};
    int LKaxis[3]   = {1, -1, 1};
    int LAaxis[3]   = {1, -1, -1};
    int LTaxis[3]   = {1, -1, -1};

    int RHaxis[3]   = {-1, 1, -1};
    int RKaxis[3]   = {-1, 1, 1};
    int RAaxis[3]   = {-1, 1, -1};
    int RTaxis[3]   = {-1, 1, -1};

    VectorQf th_angle;

    th_angle(0)  = LHaxis[2]*Lhip_.rotation[2];
    th_angle(1)  = LKaxis[2]*Lknee_.rotation[2];
    th_angle(2)  = LAaxis[2]*Lankle_.rotation[2];
    th_angle(3)  = RTaxis[2]*Ltoe_.rotation[2];
    th_angle(4)  = RHaxis[2]*Rhip_.rotation[2];
    th_angle(5)  = RKaxis[2]*Rknee_.rotation[2];
    th_angle(6)  = RAaxis[2]*Rankle_.rotation[2];
    th_angle(7)  = RTaxis[2]*Rtoe_.rotation[2];

    th_angle(8)  = L5axis[2]*L5_.rotation[2];
    th_angle(9)  = L3axis[2]*L3_.rotation[2];
    th_angle(10) = T12axis[2]*T12_.rotation[2];
    th_angle(11) = T8axis[2]*T8_.rotation[2];

    th_angle(12) = LSCaxis[2]*Lclavicle_.rotation[2];
    th_angle(13) = LSaxis[2]*Lshoulder_.rotation[2];
    th_angle(14) = LEaxis[2]*Lelbow_.rotation[2];
    th_angle(15) = LWaxis[2]*Lwrist_.rotation[2];
    th_angle(16) = RSCaxis[2]*Rclavicle_.rotation[2];
    th_angle(17) = RSaxis[2]*Rshoulder_.rotation[2];
    th_angle(18) = REaxis[2]*Relbow_.rotation[2];
    th_angle(19) = RWaxis[2]*Rwrist_.rotation[2];
    th_angle(20) = NKaxis[2]*Neck_.rotation[2];
    th_angle(21) = HDaxis[2]*Head_.rotation[2];

    th_angle = DEG2RAD(th_angle);

    struct AngleJoint readxsensangle;

    readxsensangle.q0  = -(th_angle(0) + th_angle(1) + th_angle(2));     // Pelvis
    readxsensangle.q1  = th_angle(0);         // L-Hip
    readxsensangle.q2  = th_angle(1);         // L-Knee
    readxsensangle.q3  = th_angle(2);         // L-Ankle
    readxsensangle.q3t = th_angle(3);        // L-Toe

    readxsensangle.q4  = th_angle(4);         // R-Hip
    readxsensangle.q5  = th_angle(5);         // R-Knee
    readxsensangle.q6  = th_angle(6);         // R-Ankle
    readxsensangle.q6t = th_angle(7);        // R-Toe

    readxsensangle.q7  = th_angle(8);        // L5S1
    readxsensangle.q8  = th_angle(9);        // L4L3
    readxsensangle.q9  = th_angle(10);        // T12L1
    readxsensangle.q10 = th_angle(11);       // T9T8

    readxsensangle.q11 = th_angle(12) + th_angle(13);        // L-Shoulder
    readxsensangle.q12 = th_angle(14);                       // L-Elbow
    readxsensangle.q13 = th_angle(15);                       // L-Wrist
    readxsensangle.q14 = th_angle(16) + th_angle(17);        // R-Shoulder
    readxsensangle.q15 = th_angle(18);                       // L-Elbow
    readxsensangle.q16 = th_angle(19);                       // R-Wrist
    readxsensangle.q17 = th_angle(20);                       // Neck
    readxsensangle.q18 = th_angle(21);                       // Head

    return readxsensangle;
}

AngleJoint3D ReadXsens::ReadXsensAngle3D()const{

    JointAngle Lhip_, Lknee_, Lankle_, Ltoe_, Rhip_, Rknee_, Rankle_, Rtoe_, L5_, L3_, T12_, T8_, Neck_, Head_,
    Lclavicle_, Lshoulder_, Lelbow_, Lwrist_, Rclavicle_, Rshoulder_, Relbow_, Rwrist_;
    float ang_elbow = 90.0;

    // Angle Joint are calculated in the "jointangledatagram.cpp" function getItem(ParentLink, ChildLink) according to XSens MVN manual

    L5_        = jointAngles_->getItem(1, 2);
    L3_        = jointAngles_->getItem(2, 3);
    T12_       = jointAngles_->getItem(3, 4);
    T8_        = jointAngles_->getItem(4, 5);
    Neck_      = jointAngles_->getItem(5, 6);
    Head_      = jointAngles_->getItem(6, 7);
    Rclavicle_ = jointAngles_->getItem(5, 8);
    Rshoulder_ = jointAngles_->getItem(8, 9);
    Relbow_    = jointAngles_->getItem(9, 10);
    Rwrist_    = jointAngles_->getItem(10, 11);
    Lclavicle_ = jointAngles_->getItem(5, 12);
    Lshoulder_ = jointAngles_->getItem(12, 13);
    Lelbow_    = jointAngles_->getItem(13, 14);
    Lwrist_    = jointAngles_->getItem(14, 15);
    Rhip_      = jointAngles_->getItem(1, 16);
    Rknee_     = jointAngles_->getItem(16, 17);
    Rankle_    = jointAngles_->getItem(17, 18);
    Rtoe_      = jointAngles_->getItem(18, 19);
    Lhip_      = jointAngles_->getItem(1, 20);
    Lknee_     = jointAngles_->getItem(20, 21);
    Lankle_    = jointAngles_->getItem(21, 22);
    Ltoe_      = jointAngles_->getItem(22, 23);

    // Transform the axis to obtain an alignment of all of them in resting position (N-pose)
    // Correspondence with rotation [0][0] [1][1] [2][2]
    int LSCaxis[3]  = {1, -1, -1};
    int LSaxis[3]   = {1, -1, -1};
    int LEaxis[3]   = {1, -1, -1};
    int LWaxis[3]   = {1, -1, -1};

    int RSCaxis[3]  = {-1, 1, -1};
    int RSaxis[3]   = {-1, 1, -1};
    int REaxis[3]   = {-1, 1, -1};
    int RWaxis[3]   = {-1, 1, -1};

    int L5axis[3]   = {1, 1, 1};
    int L3axis[3]   = {1, 1, 1};
    int T12axis[3]  = {1, 1, 1};
    int T8axis[3]   = {1, 1, 1};
    int NKaxis[3]   = {1, 1, 1};
    int HDaxis[3]   = {1, 1, 1};

    int LHaxis[3]   = {1, -1, -1};
    int LKaxis[3]   = {1, -1, 1};
    int LAaxis[3]   = {1, -1, -1};
    int LTaxis[3]   = {1, -1, -1};

    int RHaxis[3]   = {-1, 1, -1};
    int RKaxis[3]   = {-1, 1, 1};
    int RAaxis[3]   = {-1, 1, -1};
    int RTaxis[3]   = {-1, 1, -1};

    VectorQf th_angle_x, th_angle_y, th_angle_z;

    th_angle_x(0)  = LHaxis[0]*Lhip_.rotation[0];
    th_angle_x(1)  = LKaxis[0]*Lknee_.rotation[0];
    th_angle_x(2)  = LAaxis[0]*Lankle_.rotation[0];
    th_angle_x(3)  = LTaxis[0]*Ltoe_.rotation[0];
    th_angle_x(4)  = RHaxis[0]*Rhip_.rotation[0];
    th_angle_x(5)  = RKaxis[0]*Rknee_.rotation[0];
    th_angle_x(6)  = RAaxis[0]*Rankle_.rotation[0];
    th_angle_x(7)  = RTaxis[0]*Rtoe_.rotation[0];

    th_angle_x(8)  = L5axis[0]*L5_.rotation[0];
    th_angle_x(9)  = L3axis[0]*L3_.rotation[0];
    th_angle_x(10) = T12axis[0]*T12_.rotation[0];
    th_angle_x(11) = T8axis[0]*T8_.rotation[0];

    th_angle_x(12) = LSCaxis[0]*Lclavicle_.rotation[0];
    th_angle_x(13) = LSaxis[0]*Lshoulder_.rotation[0];
    th_angle_x(14) = LEaxis[0]*Lelbow_.rotation[0];
    th_angle_x(15) = LWaxis[0]*Lwrist_.rotation[0];
    th_angle_x(16) = RSCaxis[0]*Rclavicle_.rotation[0];
    th_angle_x(17) = RSaxis[0]*Rshoulder_.rotation[0];
    th_angle_x(18) = REaxis[0]*Relbow_.rotation[0];
    th_angle_x(19) = RWaxis[0]*Rwrist_.rotation[0];

    th_angle_x(20) = NKaxis[0]*Neck_.rotation[0];
    th_angle_x(21) = HDaxis[0]*Head_.rotation[0];

    th_angle_x = DEG2RAD(th_angle_x);

    th_angle_y(0)  = LHaxis[2]*Lhip_.rotation[2];
    th_angle_y(1)  = LKaxis[2]*Lknee_.rotation[2];
    th_angle_y(2)  = LAaxis[2]*Lankle_.rotation[2];
    th_angle_y(3)  = LTaxis[2]*Ltoe_.rotation[2];
    th_angle_y(4)  = RHaxis[2]*Rhip_.rotation[2];
    th_angle_y(5)  = RKaxis[2]*Rknee_.rotation[2];
    th_angle_y(6)  = RAaxis[2]*Rankle_.rotation[2];
    th_angle_y(7)  = RTaxis[2]*Rtoe_.rotation[2];

    th_angle_y(8)  = L5axis[2]*L5_.rotation[2];
    th_angle_y(9)  = L3axis[2]*L3_.rotation[2];
    th_angle_y(10) = T12axis[2]*T12_.rotation[2];
    th_angle_y(11) = T8axis[2]*T8_.rotation[2];

    th_angle_y(12) = LSCaxis[2]*Lclavicle_.rotation[2];
    th_angle_y(13) = LSaxis[2]*Lshoulder_.rotation[2];
    th_angle_y(14) = LEaxis[2]*Lelbow_.rotation[2];
    th_angle_y(15) = LWaxis[2]*Lwrist_.rotation[2];
    th_angle_y(16) = RSCaxis[2]*Rclavicle_.rotation[2];
    th_angle_y(17) = RSaxis[2]*Rshoulder_.rotation[2];
    th_angle_y(18) = REaxis[2]*Relbow_.rotation[2];
    th_angle_y(19) = RWaxis[2]*Rwrist_.rotation[2];

    th_angle_y(20) = NKaxis[2]*Neck_.rotation[2];
    th_angle_y(21) = HDaxis[2]*Head_.rotation[2];

    th_angle_y = DEG2RAD(th_angle_y);

    th_angle_z(0)  = LHaxis[1]*Lhip_.rotation[1];
    th_angle_z(1)  = LKaxis[1]*Lknee_.rotation[1];
    th_angle_z(2)  = LAaxis[1]*Lankle_.rotation[1];
    th_angle_z(3)  = LTaxis[1]*Ltoe_.rotation[1];
    th_angle_z(4)  = RHaxis[1]*Rhip_.rotation[1];
    th_angle_z(5)  = RKaxis[1]*Rknee_.rotation[1];
    th_angle_z(6)  = RAaxis[1]*Rankle_.rotation[1];
    th_angle_z(7)  = RTaxis[1]*Rtoe_.rotation[1];

    th_angle_z(8)  = L5axis[1]*L5_.rotation[1];
    th_angle_z(9)  = L3axis[1]*L3_.rotation[1];
    th_angle_z(10) = T12axis[1]*T12_.rotation[1];
    th_angle_z(11) = T8axis[1]*T8_.rotation[1];

    th_angle_z(12) = LSCaxis[1]*Lclavicle_.rotation[1];
    th_angle_z(13) = LSaxis[1]*Lshoulder_.rotation[1];
    th_angle_z(14) = LEaxis[1]*Lelbow_.rotation[1] + ang_elbow; // A 90 degrees angle must be added to align the reference frame with the N-pose
    th_angle_z(15) = LWaxis[1]*Lwrist_.rotation[1];
    th_angle_z(16) = RSCaxis[1]*Rclavicle_.rotation[1];
    th_angle_z(17) = RSaxis[1]*Rshoulder_.rotation[1];
    th_angle_z(18) = REaxis[1]*Relbow_.rotation[1] - ang_elbow; // A -90 degrees angle must be added to align the reference frame with the N-pose
    th_angle_z(19) = RWaxis[1]*Rwrist_.rotation[1];

    th_angle_z(20) = NKaxis[1]*Neck_.rotation[1];
    th_angle_z(21) = HDaxis[1]*Head_.rotation[1];

    th_angle_z = DEG2RAD(th_angle_z);

    struct AngleJoint3D readxsensangle3D;

    readxsensangle3D.q1_x   = th_angle_x(0);      // L-Hip
    readxsensangle3D.q2_x   = th_angle_x(1);      // L-Knee
    readxsensangle3D.q3_x   = th_angle_x(2);      // L-Ankle
    readxsensangle3D.q3t_x  = th_angle_x(3);      // L-Toe
    readxsensangle3D.q4_x   = th_angle_x(4);      // R-Hip
    readxsensangle3D.q5_x   = th_angle_x(5);      // R-Knee
    readxsensangle3D.q6_x   = th_angle_x(6);      // R-Ankle
    readxsensangle3D.q6t_x  = th_angle_x(7);      // R-Toe
    readxsensangle3D.q7_x   = th_angle_x(8);      // L5S1
    readxsensangle3D.q8_x   = th_angle_x(9);      // L4L3
    readxsensangle3D.q9_x   = th_angle_x(10);     // T12L1
    readxsensangle3D.q10_x  = th_angle_x(11);     // T9T8
    readxsensangle3D.q11sc_x  = th_angle_x(12);   // L-Clavicle
    readxsensangle3D.q11_x  = th_angle_x(13);     // L-Shoulder
    readxsensangle3D.q12_x  = th_angle_x(14);     // L-Elbow
    readxsensangle3D.q13_x  = th_angle_x(15);     // L-Wrist
    readxsensangle3D.q14sc_x  = th_angle_x(16);   // R-Clavicle
    readxsensangle3D.q14_x  = th_angle_x(17);     // R-Shoulder
    readxsensangle3D.q15_x  = th_angle_x(18);     // R-Elbow
    readxsensangle3D.q16_x  = th_angle_x(19);     // R-Wrist
    readxsensangle3D.q17_x  = th_angle_x(20);     // Neck
    readxsensangle3D.q18_x  = th_angle_x(21);     // Head

    readxsensangle3D.q1_y   = th_angle_y(0);      // L-Hip
    readxsensangle3D.q2_y   = th_angle_y(1);      // L-Knee
    readxsensangle3D.q3_y   = th_angle_y(2);      // L-Ankle
    readxsensangle3D.q3t_y  = th_angle_y(3);      // L-Toe
    readxsensangle3D.q4_y   = th_angle_y(4);      // R-Hip
    readxsensangle3D.q5_y   = th_angle_y(5);      // R-Knee
    readxsensangle3D.q6_y   = th_angle_y(6);      // R-Ankle
    readxsensangle3D.q6t_y  = th_angle_y(7);      // R-Toe
    readxsensangle3D.q7_y   = th_angle_y(8);      // L5S1
    readxsensangle3D.q8_y   = th_angle_y(9);      // L4L3
    readxsensangle3D.q9_y   = th_angle_y(10);     // T12L1
    readxsensangle3D.q10_y  = th_angle_y(11);     // T9T8
    readxsensangle3D.q11sc_y  = th_angle_y(12);   // L-Clavicle
    readxsensangle3D.q11_y  = th_angle_y(13);     // L-Shoulder
    readxsensangle3D.q12_y  = th_angle_y(14);     // L-Elbow
    readxsensangle3D.q13_y  = th_angle_y(15);     // L-Wrist
    readxsensangle3D.q14sc_y  = th_angle_y(16);   // R-Clavicle
    readxsensangle3D.q14_y  = th_angle_y(17);     // R-Shoulder
    readxsensangle3D.q15_y  = th_angle_y(18);     // R-Elbow
    readxsensangle3D.q16_y  = th_angle_y(19);     // R-Wrist
    readxsensangle3D.q17_y  = th_angle_y(20);     // Neck
    readxsensangle3D.q18_y  = th_angle_y(21);     // Head

    readxsensangle3D.q1_z   = th_angle_z(0);      // L-Hip
    readxsensangle3D.q2_z   = th_angle_z(1);      // L-Knee
    readxsensangle3D.q3_z   = th_angle_z(2);      // L-Ankle
    readxsensangle3D.q3t_z  = th_angle_z(3);      // L-Toe
    readxsensangle3D.q4_z   = th_angle_z(4);      // R-Hip
    readxsensangle3D.q5_z   = th_angle_z(5);      // R-Knee
    readxsensangle3D.q6_z   = th_angle_z(6);      // R-Ankle
    readxsensangle3D.q6t_z  = th_angle_z(7);      // R-Toe
    readxsensangle3D.q7_z   = th_angle_z(8);      // L5S1
    readxsensangle3D.q8_z   = th_angle_z(9);      // L4L3
    readxsensangle3D.q9_z   = th_angle_z(10);     // T12L1
    readxsensangle3D.q10_z  = th_angle_z(11);     // T9T8
    readxsensangle3D.q11sc_z  = th_angle_z(12);   // L-Clavicle
    readxsensangle3D.q11_z  = th_angle_z(13);     // L-Shoulder
    readxsensangle3D.q12_z  = th_angle_z(14);     // L-Elbow
    readxsensangle3D.q13_z  = th_angle_z(15);     // L-Wrist
    readxsensangle3D.q14sc_z  = th_angle_z(16);   // R-Clavicle
    readxsensangle3D.q14_z  = th_angle_z(17);     // R-Shoulder
    readxsensangle3D.q15_z  = th_angle_z(18);     // R-Elbow
    readxsensangle3D.q16_z  = th_angle_z(19);     // R-Wrist
    readxsensangle3D.q17_z  = th_angle_z(20);     // Neck
    readxsensangle3D.q18_z  = th_angle_z(21);     // Head

    std::cout << "Right shoulder: " << th_angle_x(17) << " " << th_angle_y(17) << " " << th_angle_z(17)  << "\n";

    return readxsensangle3D;
}

Rotation3f ReadXsens::WriteXsensAngleMatrix(AngleJoint readxsensangle)const{

    RotationMatrix *xsens;
    Rotation3f writexsensanglematrix = xsens->AngleMatrix(readxsensangle);

    return writexsensanglematrix;
}

Rotation3f ReadXsens::WriteXsensAngleMatrix3D(AngleJoint3D readxsensangle3D)const{

    RotationMatrix *xsens;
    Rotation3f writexsensanglematrix3D = xsens->AngleMatrix3D(readxsensangle3D);

    return writexsensanglematrix3D;
}

qKinematics ReadXsens::ReadXsensQuaternion() const{

    struct qKinematics qKinematics;

    qKinematics.Pelvis_         = quaternionDatagram_->getItem(1);
    qKinematics.L5_             = quaternionDatagram_->getItem(2);
    qKinematics.L3_             = quaternionDatagram_->getItem(3);
    qKinematics.T12_            = quaternionDatagram_->getItem(4);
    qKinematics.T8_             = quaternionDatagram_->getItem(5);
    qKinematics.NeckP_          = quaternionDatagram_->getItem(6);
    qKinematics.HeadP_          = quaternionDatagram_->getItem(7);
    qKinematics.RightClavicleP_ = quaternionDatagram_->getItem(8);
    qKinematics.RightShoulderP_ = quaternionDatagram_->getItem(9);
    qKinematics.RightElbowP_    = quaternionDatagram_->getItem(10);
    qKinematics.RightWristP_    = quaternionDatagram_->getItem(11);
    qKinematics.LeftClavicleP_  = quaternionDatagram_->getItem(12);
    qKinematics.LeftShoulderP_  = quaternionDatagram_->getItem(13);
    qKinematics.LeftElbowP_     = quaternionDatagram_->getItem(14);
    qKinematics.LeftWristP_     = quaternionDatagram_->getItem(15);
    qKinematics.RightHipP_      = quaternionDatagram_->getItem(16);
    qKinematics.RightKneeP_     = quaternionDatagram_->getItem(17);
    qKinematics.RightAnkleP_    = quaternionDatagram_->getItem(18);
    qKinematics.RightToeP_      = quaternionDatagram_->getItem(19);
    qKinematics.LeftHipP_       = quaternionDatagram_->getItem(20);
    qKinematics.LeftKneeP_      = quaternionDatagram_->getItem(21);
    qKinematics.LeftAnkleP_     = quaternionDatagram_->getItem(22);
    qKinematics.LeftToeP_       = quaternionDatagram_->getItem(23);

    return qKinematics;
}

JointPosition ReadXsens::WriteXsensPosition(qKinematics qKin) {

    float xsensCal[3] = {};
    struct JointPosition readxsensposition;
//     std::cout << "Test Optitrack: " << qKin.RightToeP_.sensorPos[0] << "\t" << qKin.RightToeP_.sensorPos[1] << "\t" << qKin.RightToeP_.sensorPos[2] << std::endl;
//     std::cout << "CalValue: " << CalValue[0] << "\t" << CalValue[1] << "\t" << CalValue[2] << std::endl;

    for (int i = 0; i < 3; i++){
        if(OptitrackFlag) {
            xsensCal[i] = (qKin.RightToeP_.sensorPos[i] - CalValue[i]  - OptitrackPos(i) );  // Define the offset value
        } else {
        // -- TO BE USED -- when Optitrack is not activated
            xsensCal[0] = 0; xsensCal[1] = 0; xsensCal[2] = 0;
                if (Cal && !check_optitrack_)
                {
                    std::cout << "\n- optitrack is NOT active!" << std::endl;
                    check_optitrack_ = true;
                }
                    
            
        }
        

//         std::cout << "OptitrackPos: " << OptitrackPos[0] << "\t" << OptitrackPos[1] << "\t" << OptitrackPos[2] << std::endl;
//         std::cout << "xsensCal: " << xsensCal[0] << "\t" << xsensCal[1] << "\t" << xsensCal[2] << std::endl;

        readxsensposition.Pelvis_Pos(i)        = qKin.Pelvis_.sensorPos[i] - xsensCal[i];
        readxsensposition.LeftHip_Pos(i)       = qKin.LeftHipP_.sensorPos[i] - xsensCal[i];
        readxsensposition.LeftKnee_Pos(i)      = qKin.LeftKneeP_.sensorPos[i] - xsensCal[i];
        readxsensposition.LeftAnkle_Pos(i)     = qKin.LeftAnkleP_.sensorPos[i] - xsensCal[i];
        readxsensposition.LeftToe_Pos(i)       = qKin.LeftToeP_.sensorPos[i] - xsensCal[i];
        readxsensposition.RightHip_Pos(i)      = qKin.RightHipP_.sensorPos[i] - xsensCal[i];
        readxsensposition.RightKnee_Pos(i)     = qKin.RightKneeP_.sensorPos[i] - xsensCal[i];
        readxsensposition.RightAnkle_Pos(i)    = qKin.RightAnkleP_.sensorPos[i] - xsensCal[i];
        readxsensposition.RightToe_Pos(i)      = qKin.RightToeP_.sensorPos[i] - xsensCal[i];
        readxsensposition.L5_Pos(i)            = qKin.L5_.sensorPos[i] - xsensCal[i];
        readxsensposition.L3_Pos(i)            = qKin.L3_.sensorPos[i] - xsensCal[i];
        readxsensposition.T12_Pos(i)           = qKin.T12_.sensorPos[i] - xsensCal[i];
        readxsensposition.T8_Pos(i)            = qKin.T8_.sensorPos[i] - xsensCal[i];
        readxsensposition.LeftClavicle_Pos(i)  = qKin.LeftClavicleP_.sensorPos[i] - xsensCal[i];
        readxsensposition.LeftShoulder_Pos(i)  = qKin.LeftShoulderP_.sensorPos[i] - xsensCal[i];
        readxsensposition.LeftElbow_Pos(i)     = qKin.LeftElbowP_.sensorPos[i] - xsensCal[i];
        readxsensposition.LeftWrist_Pos(i)     = qKin.LeftWristP_.sensorPos[i] - xsensCal[i];
        readxsensposition.RightClavicle_Pos(i) = qKin.RightClavicleP_.sensorPos[i] - xsensCal[i];
        readxsensposition.RightShoulder_Pos(i) = qKin.RightShoulderP_.sensorPos[i] - xsensCal[i];
        readxsensposition.RightElbow_Pos(i)    = qKin.RightElbowP_.sensorPos[i] - xsensCal[i];
        readxsensposition.RightWrist_Pos(i)    = qKin.RightWristP_.sensorPos[i] - xsensCal[i];
        readxsensposition.Neck_Pos(i)          = qKin.NeckP_.sensorPos[i] - xsensCal[i];
        readxsensposition.Head_Pos(i)          = qKin.HeadP_.sensorPos[i] - xsensCal[i];

    }

    return readxsensposition;
}

QuaternionJoint ReadXsens::WriteXsensQuaternion(qKinematics qKin) const{

    struct QuaternionJoint readxsensquaternion;
    Quaternionf Qx_m90, Qx_p90;

    // NB: The axes of each body frame are aligned with the Global reference frame (G) when the subject is standing in the T-pose
    // To express the quaternions orientation wrt the N-pose we multiply by Q_90
    Qx_m90.w() = 0.70711;
    Qx_m90.x() = -0.70711;
    Qx_m90.y() = 0.0;
    Qx_m90.z() = 0.0;

    Qx_p90 = Qx_m90;
    Qx_p90.x() = 0.70711;

    readxsensquaternion.Q0    = vec2quat(qKin.Pelvis_.quatRotation);
    readxsensquaternion.Q1    = vec2quat(qKin.LeftHipP_.quatRotation);
    readxsensquaternion.Q2    = vec2quat(qKin.LeftKneeP_.quatRotation);
    readxsensquaternion.Q3    = vec2quat(qKin.LeftAnkleP_.quatRotation);
    readxsensquaternion.Q3t   = vec2quat(qKin.LeftToeP_.quatRotation);
    readxsensquaternion.Q4    = vec2quat(qKin.RightHipP_.quatRotation);
    readxsensquaternion.Q5    = vec2quat(qKin.RightKneeP_.quatRotation);
    readxsensquaternion.Q6    = vec2quat(qKin.RightAnkleP_.quatRotation);
    readxsensquaternion.Q6t   = vec2quat(qKin.RightToeP_.quatRotation);

    readxsensquaternion.Q7    = vec2quat(qKin.L5_.quatRotation);
    readxsensquaternion.Q8    = vec2quat(qKin.L3_.quatRotation);
    readxsensquaternion.Q9    = vec2quat(qKin.T12_.quatRotation);
    readxsensquaternion.Q10   = vec2quat(qKin.T8_.quatRotation);

    readxsensquaternion.Q11sc = vec2quat(qKin.LeftClavicleP_.quatRotation);
    readxsensquaternion.Q11   = vec2quat(qKin.LeftShoulderP_.quatRotation)*Qx_p90;
    readxsensquaternion.Q12   = vec2quat(qKin.LeftElbowP_.quatRotation)*Qx_p90;
    readxsensquaternion.Q13   = vec2quat(qKin.LeftWristP_.quatRotation)*Qx_p90;

    readxsensquaternion.Q14sc = vec2quat(qKin.RightClavicleP_.quatRotation);
    readxsensquaternion.Q14   = vec2quat(qKin.RightShoulderP_.quatRotation)*Qx_m90;
    readxsensquaternion.Q15   = vec2quat(qKin.RightElbowP_.quatRotation)*Qx_m90;
    readxsensquaternion.Q16   = vec2quat(qKin.RightWristP_.quatRotation)*Qx_m90;

    readxsensquaternion.Q17   = vec2quat(qKin.NeckP_.quatRotation);
    readxsensquaternion.Q18   = vec2quat(qKin.HeadP_.quatRotation);

//     std::cout << "TestQuarternion: " << readxsensquaternion.Q0.coeffs() << std::endl;

    return readxsensquaternion;
}


Rotation3f ReadXsens::WriteXsensQuaternionMatrix(QuaternionJoint qRotation) const{
    RotationMatrix xsens;

    Rotation3f writexsensanglematrix  = xsens.QuaternionMatrix(qRotation);
//     std::cout <<  "LM5  = " << writexsensanglematrix.A0 << std::endl;
//     std::cout <<  "LM5  = " << writexsensanglematrix.A0(1,0) << "\t" << writexsensanglematrix.A0(1,1) << "\t" << writexsensanglematrix.A0(1,2) << std::endl;
//     std::cout <<  "LM5  = " << writexsensanglematrix.A0(2,0) << "\t" << writexsensanglematrix.A0(2,1) << "\t" << writexsensanglematrix.A0(2,2) << std::endl;

    return writexsensanglematrix;
}

LinkLength ReadXsens::WriteXsensLength(JointPosition Pjoint, Rotation3f Rotation) const{

    struct LinkLength writexsenslength;
    // The length is set manually
    Vector3f HandLength, HeadLength;
    HandLength << 0.0, 0.0, -0.15;
    HeadLength << 0.0, 0.0, 0.25;

    writexsenslength.Pelvislength  = Rotation.A0.inverse()*( Pjoint.L5_Pos - Pjoint.Pelvis_Pos );
    writexsenslength.LPVlinklength = Rotation.A0.inverse()*( Pjoint.LeftHip_Pos - Pjoint.Pelvis_Pos);
    writexsenslength.LTHlength     = Rotation.A1.inverse()*( Pjoint.LeftKnee_Pos - Pjoint.LeftHip_Pos );
    writexsenslength.LSKlength     = Rotation.A2.inverse()*( Pjoint.LeftAnkle_Pos - Pjoint.LeftKnee_Pos );
    writexsenslength.LFTlength     = Rotation.A3.inverse()*( Pjoint.LeftToe_Pos - Pjoint.LeftAnkle_Pos );
    writexsenslength.LFTlength[0] = writexsenslength.LFTlength[0] + 0.05; // Adding Toe length
    writexsenslength.RPVlinklength = Rotation.A0.inverse()*( Pjoint.RightHip_Pos - Pjoint.Pelvis_Pos );
    writexsenslength.RTHlength     = Rotation.A4.inverse()*( Pjoint.RightKnee_Pos - Pjoint.RightHip_Pos );
    writexsenslength.RSKlength     = Rotation.A5.inverse()*( Pjoint.RightAnkle_Pos - Pjoint.RightKnee_Pos );
    writexsenslength.RFTlength     = Rotation.A6.inverse()*( Pjoint.RightToe_Pos - Pjoint.RightAnkle_Pos );
    writexsenslength.RFTlength[0] = writexsenslength.RFTlength[0] + 0.05; // Adding Toe length
    writexsenslength.L5length      = Rotation.A7.inverse()*( Pjoint.L3_Pos - Pjoint.L5_Pos);
    writexsenslength.L3length      = Rotation.A8.inverse()*( Pjoint.T12_Pos - Pjoint.L3_Pos);
    writexsenslength.T12length     = Rotation.A9.inverse()*( Pjoint.T8_Pos - Pjoint.T12_Pos);
    writexsenslength.T8length      = Rotation.A10.inverse()*( Pjoint.Neck_Pos - Pjoint.T8_Pos );
    writexsenslength.T8LSHlength   = Rotation.A10.inverse()*( Pjoint.LeftShoulder_Pos - Pjoint.T8_Pos );
    writexsenslength.LUAlength     = Rotation.A11.inverse()*( Pjoint.LeftElbow_Pos - Pjoint.LeftShoulder_Pos );
    writexsenslength.LFAlength     = Rotation.A12.inverse()*( Pjoint.LeftWrist_Pos - Pjoint.LeftElbow_Pos );
    writexsenslength.LHlength      = HandLength;
    writexsenslength.T8RSHlength   = Rotation.A10.inverse()*( Pjoint.RightShoulder_Pos - Pjoint.T8_Pos );
    writexsenslength.RUAlength     = Rotation.A14.inverse()*( Pjoint.RightElbow_Pos - Pjoint.RightShoulder_Pos );
    writexsenslength.RFAlength     = Rotation.A15.inverse()*( Pjoint.RightWrist_Pos - Pjoint.RightElbow_Pos );
    writexsenslength.RHlength      = HandLength;
    writexsenslength.NKlength      = Rotation.A17.inverse()*( Pjoint.Head_Pos - Pjoint.Neck_Pos );
    writexsenslength.HDlength      = HeadLength;

   // Wansoo
//     writexsenslength.Pelvislength << -0.0097, 0.0, 0.1153;
//     writexsenslength.LPVlinklength << 0.0, 0.0943, 0.0;
//     writexsenslength.LTHlength << 0.0, 0.0, -0.4411;
//     writexsenslength.LSKlength << 0.0, 0.0, -0.3992;
//     writexsenslength.LFTlength << 0.1656, 0.0, -0.0945;
//     writexsenslength.RPVlinklength << 0.0, -0.0943, 0.0;
//     writexsenslength.RTHlength << 0.0, 0.0, -0.4411;
//     writexsenslength.RSKlength << 0.0, 0.0, -0.3995;
//     writexsenslength.RFTlength << 0.1656, 0.0, -0.0945;
//     writexsenslength.L5length << -0.0041, 0.0, 0.1123 ;
//     writexsenslength.L3length << -0.0038, 0.0, 0.1015;
//     writexsenslength.T12length << -0.0028, 0.0, 0.1015;
//     writexsenslength.T8length << -0.0018, 0.0, 0.08;
//     writexsenslength.T8LSHlength << -0.0008, 0.1859, 0.0862;
//     writexsenslength.LUAlength << 0.0, 0.0, -0.3017;
//     writexsenslength.LFAlength << 0.0, 0.0, -0.2481;
//     writexsenslength.LHlength << 0.0, 0.0, -0.15;
//     writexsenslength.T8RSHlength << 0.0010, -0.1825, 0.0871;
//     writexsenslength.RUAlength << 0.0, 0.0, -0.3017;
//     writexsenslength.RFAlength << 0.0, 0.0, -0.2481;
//     writexsenslength.RHlength << 0.0, 0.0, -0.15;

    return writexsenslength;
}

linearSegtKinematics ReadXsens::ReadXsensLinearKinematics() const
{
    struct linearSegtKinematics linearSegtKinematics;

    linearSegtKinematics.Pelvis_         = LinearSegKinDatagram_->getItem(1);
    linearSegtKinematics.L5_             = LinearSegKinDatagram_->getItem(2);
    linearSegtKinematics.L3_             = LinearSegKinDatagram_->getItem(3);
    linearSegtKinematics.T12_            = LinearSegKinDatagram_->getItem(4);
    linearSegtKinematics.T8_             = LinearSegKinDatagram_->getItem(5);
    linearSegtKinematics.NeckP_          = LinearSegKinDatagram_->getItem(6);
    linearSegtKinematics.HeadP_          = LinearSegKinDatagram_->getItem(7);
    linearSegtKinematics.RightClavicleP_ = LinearSegKinDatagram_->getItem(8);
    linearSegtKinematics.RightShoulderP_ = LinearSegKinDatagram_->getItem(9);
    linearSegtKinematics.RightElbowP_    = LinearSegKinDatagram_->getItem(10);
    linearSegtKinematics.RightWristP_    = LinearSegKinDatagram_->getItem(11);
    linearSegtKinematics.LeftClavicleP_  = LinearSegKinDatagram_->getItem(12);
    linearSegtKinematics.LeftShoulderP_  = LinearSegKinDatagram_->getItem(13);
    linearSegtKinematics.LeftElbowP_     = LinearSegKinDatagram_->getItem(14);
    linearSegtKinematics.LeftWristP_     = LinearSegKinDatagram_->getItem(15);
    linearSegtKinematics.RightHipP_      = LinearSegKinDatagram_->getItem(16);
    linearSegtKinematics.RightKneeP_     = LinearSegKinDatagram_->getItem(17);
    linearSegtKinematics.RightAnkleP_    = LinearSegKinDatagram_->getItem(18);
    linearSegtKinematics.RightToeP_      = LinearSegKinDatagram_->getItem(19);
    linearSegtKinematics.LeftHipP_       = LinearSegKinDatagram_->getItem(20);
    linearSegtKinematics.LeftKneeP_      = LinearSegKinDatagram_->getItem(21);
    linearSegtKinematics.LeftAnkleP_     = LinearSegKinDatagram_->getItem(22);
    linearSegtKinematics.LeftToeP_       = LinearSegKinDatagram_->getItem(23);

    return linearSegtKinematics;

}

SegmentLinearVelocity ReadXsens::WriteXsensLinearVelocity(linearSegtKinematics lsKin) const
{

    SegmentLinearVelocity readsegmentlinearvelocity;

     for (int i = 0; i < 3; i++){

        readsegmentlinearvelocity.Pelvis_LinVel(i)        = lsKin.Pelvis_.velocity[i];
        readsegmentlinearvelocity.LeftHip_LinVel(i)       = lsKin.LeftHipP_.velocity[i];
        readsegmentlinearvelocity.LeftKnee_LinVel(i)      = lsKin.LeftKneeP_.velocity[i];
        readsegmentlinearvelocity.LeftAnkle_LinVel(i)     = lsKin.LeftAnkleP_.velocity[i];
        readsegmentlinearvelocity.LeftToe_LinVel(i)       = lsKin.LeftToeP_.velocity[i];
        readsegmentlinearvelocity.RightHip_LinVel(i)      = lsKin.RightHipP_.velocity[i];
        readsegmentlinearvelocity.RightKnee_LinVel(i)     = lsKin.RightKneeP_.velocity[i];
        readsegmentlinearvelocity.RightAnkle_LinVel(i)    = lsKin.RightAnkleP_.velocity[i];
        readsegmentlinearvelocity.RightToe_LinVel(i)      = lsKin.RightToeP_.velocity[i];
        readsegmentlinearvelocity.L5_LinVel(i)            = lsKin.L5_.velocity[i];
        readsegmentlinearvelocity.L3_LinVel(i)            = lsKin.L3_.velocity[i];
        readsegmentlinearvelocity.T12_LinVel(i)           = lsKin.T12_.velocity[i];
        readsegmentlinearvelocity.T8_LinVel(i)            = lsKin.T8_.velocity[i];
        readsegmentlinearvelocity.LeftClavicle_LinVel(i)  = lsKin.LeftClavicleP_.velocity[i];
        readsegmentlinearvelocity.LeftShoulder_LinVel(i)  = lsKin.LeftShoulderP_.velocity[i];
        readsegmentlinearvelocity.LeftElbow_LinVel(i)     = lsKin.LeftElbowP_.velocity[i];
        readsegmentlinearvelocity.LeftWrist_LinVel(i)     = lsKin.LeftWristP_.velocity[i];
        readsegmentlinearvelocity.RightClavicle_LinVel(i) = lsKin.RightClavicleP_.velocity[i];
        readsegmentlinearvelocity.RightShoulder_LinVel(i) = lsKin.RightShoulderP_.velocity[i];
        readsegmentlinearvelocity.RightElbow_LinVel(i)    = lsKin.RightElbowP_.velocity[i];
        readsegmentlinearvelocity.RightWrist_LinVel(i)    = lsKin.RightWristP_.velocity[i];
        readsegmentlinearvelocity.Neck_LinVel(i)          = lsKin.NeckP_.velocity[i];
        readsegmentlinearvelocity.Head_LinVel(i)          = lsKin.HeadP_.velocity[i];

    }

    return readsegmentlinearvelocity;
}

SegmentLinearAcceleration ReadXsens::WriteXsensLinearAcceleration(linearSegtKinematics lsKin) const
{

    SegmentLinearAcceleration readsegmentlinearacceleration;

     for (int i = 0; i < 3; i++){

        readsegmentlinearacceleration.Pelvis_LinAcc(i)        = lsKin.Pelvis_.acceleration[i];
        readsegmentlinearacceleration.LeftHip_LinAcc(i)       = lsKin.LeftHipP_.acceleration[i];
        readsegmentlinearacceleration.LeftKnee_LinAcc(i)      = lsKin.LeftKneeP_.acceleration[i];
        readsegmentlinearacceleration.LeftAnkle_LinAcc(i)     = lsKin.LeftAnkleP_.acceleration[i];
        readsegmentlinearacceleration.LeftToe_LinAcc(i)       = lsKin.LeftToeP_.acceleration[i];
        readsegmentlinearacceleration.RightHip_LinAcc(i)      = lsKin.RightHipP_.acceleration[i];
        readsegmentlinearacceleration.RightKnee_LinAcc(i)     = lsKin.RightKneeP_.acceleration[i];
        readsegmentlinearacceleration.RightAnkle_LinAcc(i)    = lsKin.RightAnkleP_.acceleration[i];
        readsegmentlinearacceleration.RightToe_LinAcc(i)      = lsKin.RightToeP_.acceleration[i];
        readsegmentlinearacceleration.L5_LinAcc(i)            = lsKin.L5_.acceleration[i];
        readsegmentlinearacceleration.L3_LinAcc(i)            = lsKin.L3_.acceleration[i];
        readsegmentlinearacceleration.T12_LinAcc(i)           = lsKin.T12_.acceleration[i];
        readsegmentlinearacceleration.T8_LinAcc(i)            = lsKin.T8_.acceleration[i];
        readsegmentlinearacceleration.LeftClavicle_LinAcc(i)  = lsKin.LeftClavicleP_.acceleration[i];
        readsegmentlinearacceleration.LeftShoulder_LinAcc(i)  = lsKin.LeftShoulderP_.acceleration[i];
        readsegmentlinearacceleration.LeftElbow_LinAcc(i)     = lsKin.LeftElbowP_.acceleration[i];
        readsegmentlinearacceleration.LeftWrist_LinAcc(i)     = lsKin.LeftWristP_.acceleration[i];
        readsegmentlinearacceleration.RightClavicle_LinAcc(i) = lsKin.RightClavicleP_.acceleration[i];
        readsegmentlinearacceleration.RightShoulder_LinAcc(i) = lsKin.RightShoulderP_.acceleration[i];
        readsegmentlinearacceleration.RightElbow_LinAcc(i)    = lsKin.RightElbowP_.acceleration[i];
        readsegmentlinearacceleration.RightWrist_LinAcc(i)    = lsKin.RightWristP_.acceleration[i];
        readsegmentlinearacceleration.Neck_LinAcc(i)          = lsKin.NeckP_.acceleration[i];
        readsegmentlinearacceleration.Head_LinAcc(i)          = lsKin.HeadP_.acceleration[i];

    }

    return readsegmentlinearacceleration;
}

angSegtKinematics ReadXsens::ReadXsensAngularKinematics() const
{
    struct angSegtKinematics angSegtKinematics;

    angSegtKinematics.Pelvis_         = AngSegKinDatagram_->getItem(1);
    angSegtKinematics.L5_             = AngSegKinDatagram_->getItem(2);
    angSegtKinematics.L3_             = AngSegKinDatagram_->getItem(3);
    angSegtKinematics.T12_            = AngSegKinDatagram_->getItem(4);
    angSegtKinematics.T8_             = AngSegKinDatagram_->getItem(5);
    angSegtKinematics.NeckP_          = AngSegKinDatagram_->getItem(6);
    angSegtKinematics.HeadP_          = AngSegKinDatagram_->getItem(7);
    angSegtKinematics.RightClavicleP_ = AngSegKinDatagram_->getItem(8);
    angSegtKinematics.RightShoulderP_ = AngSegKinDatagram_->getItem(9);
    angSegtKinematics.RightElbowP_    = AngSegKinDatagram_->getItem(10);
    angSegtKinematics.RightWristP_    = AngSegKinDatagram_->getItem(11);
    angSegtKinematics.LeftClavicleP_  = AngSegKinDatagram_->getItem(12);
    angSegtKinematics.LeftShoulderP_  = AngSegKinDatagram_->getItem(13);
    angSegtKinematics.LeftElbowP_     = AngSegKinDatagram_->getItem(14);
    angSegtKinematics.LeftWristP_     = AngSegKinDatagram_->getItem(15);
    angSegtKinematics.RightHipP_      = AngSegKinDatagram_->getItem(16);
    angSegtKinematics.RightKneeP_     = AngSegKinDatagram_->getItem(17);
    angSegtKinematics.RightAnkleP_    = AngSegKinDatagram_->getItem(18);
    angSegtKinematics.RightToeP_      = AngSegKinDatagram_->getItem(19);
    angSegtKinematics.LeftHipP_       = AngSegKinDatagram_->getItem(20);
    angSegtKinematics.LeftKneeP_      = AngSegKinDatagram_->getItem(21);
    angSegtKinematics.LeftAnkleP_     = AngSegKinDatagram_->getItem(22);
    angSegtKinematics.LeftToeP_       = AngSegKinDatagram_->getItem(23);
    return angSegtKinematics;

}

SegmentAngularVelocity ReadXsens::WriteXsensAngularVelocity(angSegtKinematics asKin) const
{

    SegmentAngularVelocity readsegmentangularvelocity;

     for (int i = 0; i < 3; i++){

        readsegmentangularvelocity.Pelvis_AngVel(i)        = asKin.Pelvis_.angularVeloc[i];
        readsegmentangularvelocity.LeftHip_AngVel(i)       = asKin.LeftHipP_.angularVeloc[i];
        readsegmentangularvelocity.LeftKnee_AngVel(i)      = asKin.LeftKneeP_.angularVeloc[i];
        readsegmentangularvelocity.LeftAnkle_AngVel(i)     = asKin.LeftAnkleP_.angularVeloc[i];
        readsegmentangularvelocity.LeftToe_AngVel(i)       = asKin.LeftToeP_.angularVeloc[i];
        readsegmentangularvelocity.RightHip_AngVel(i)      = asKin.RightHipP_.angularVeloc[i];
        readsegmentangularvelocity.RightKnee_AngVel(i)     = asKin.RightKneeP_.angularVeloc[i];
        readsegmentangularvelocity.RightAnkle_AngVel(i)    = asKin.RightAnkleP_.angularVeloc[i];
        readsegmentangularvelocity.RightToe_AngVel(i)      = asKin.RightToeP_.angularVeloc[i];
        readsegmentangularvelocity.L5_AngVel(i)            = asKin.L5_.angularVeloc[i];
        readsegmentangularvelocity.L3_AngVel(i)            = asKin.L3_.angularVeloc[i];
        readsegmentangularvelocity.T12_AngVel(i)           = asKin.T12_.angularVeloc[i];
        readsegmentangularvelocity.T8_AngVel(i)            = asKin.T8_.angularVeloc[i];
        readsegmentangularvelocity.LeftClavicle_AngVel(i)  = asKin.LeftClavicleP_.angularVeloc[i];
        readsegmentangularvelocity.LeftShoulder_AngVel(i)  = asKin.LeftShoulderP_.angularVeloc[i];
        readsegmentangularvelocity.LeftElbow_AngVel(i)     = asKin.LeftElbowP_.angularVeloc[i];
        readsegmentangularvelocity.LeftWrist_AngVel(i)     = asKin.LeftWristP_.angularVeloc[i];
        readsegmentangularvelocity.RightClavicle_AngVel(i) = asKin.RightClavicleP_.angularVeloc[i];
        readsegmentangularvelocity.RightShoulder_AngVel(i) = asKin.RightShoulderP_.angularVeloc[i];
        readsegmentangularvelocity.RightElbow_AngVel(i)    = asKin.RightElbowP_.angularVeloc[i];
        readsegmentangularvelocity.RightWrist_AngVel(i)    = asKin.RightWristP_.angularVeloc[i];
        readsegmentangularvelocity.Neck_AngVel(i)          = asKin.NeckP_.angularVeloc[i];
        readsegmentangularvelocity.Head_AngVel(i)          = asKin.HeadP_.angularVeloc[i];

    }
    return readsegmentangularvelocity;
}

SegmentAngularAcceleration ReadXsens::WriteXsensAngularAcceleration(angSegtKinematics asKin) const
{

    SegmentAngularAcceleration readsegmentangularacceleration;

     for (int i = 0; i < 3; i++){

        readsegmentangularacceleration.Pelvis_AngAcc(i)        = asKin.Pelvis_.angularAccel[i];
        readsegmentangularacceleration.LeftHip_AngAcc(i)       = asKin.LeftHipP_.angularAccel[i];
        readsegmentangularacceleration.LeftKnee_AngAcc(i)      = asKin.LeftKneeP_.angularAccel[i];
        readsegmentangularacceleration.LeftAnkle_AngAcc(i)     = asKin.LeftAnkleP_.angularAccel[i];
        readsegmentangularacceleration.LeftToe_AngAcc(i)       = asKin.LeftToeP_.angularAccel[i];
        readsegmentangularacceleration.RightHip_AngAcc(i)      = asKin.RightHipP_.angularAccel[i];
        readsegmentangularacceleration.RightKnee_AngAcc(i)     = asKin.RightKneeP_.angularAccel[i];
        readsegmentangularacceleration.RightAnkle_AngAcc(i)    = asKin.RightAnkleP_.angularAccel[i];
        readsegmentangularacceleration.RightToe_AngAcc(i)      = asKin.RightToeP_.angularAccel[i];
        readsegmentangularacceleration.L5_AngAcc(i)            = asKin.L5_.angularAccel[i];
        readsegmentangularacceleration.L3_AngAcc(i)            = asKin.L3_.angularAccel[i];
        readsegmentangularacceleration.T12_AngAcc(i)           = asKin.T12_.angularAccel[i];
        readsegmentangularacceleration.T8_AngAcc(i)            = asKin.T8_.angularAccel[i];
        readsegmentangularacceleration.LeftClavicle_AngAcc(i)  = asKin.LeftClavicleP_.angularAccel[i];
        readsegmentangularacceleration.LeftShoulder_AngAcc(i)  = asKin.LeftShoulderP_.angularAccel[i];
        readsegmentangularacceleration.LeftElbow_AngAcc(i)     = asKin.LeftElbowP_.angularAccel[i];
        readsegmentangularacceleration.LeftWrist_AngAcc(i)     = asKin.LeftWristP_.angularAccel[i];
        readsegmentangularacceleration.RightClavicle_AngAcc(i) = asKin.RightClavicleP_.angularAccel[i];
        readsegmentangularacceleration.RightShoulder_AngAcc(i) = asKin.RightShoulderP_.angularAccel[i];
        readsegmentangularacceleration.RightElbow_AngAcc(i)    = asKin.RightElbowP_.angularAccel[i];
        readsegmentangularacceleration.RightWrist_AngAcc(i)    = asKin.RightWristP_.angularAccel[i];
        readsegmentangularacceleration.Neck_AngAcc(i)          = asKin.NeckP_.angularAccel[i];
        readsegmentangularacceleration.Head_AngAcc(i)          = asKin.HeadP_.angularAccel[i];

    }
    return readsegmentangularacceleration;
}

extern Matrix66f computeAdJointMatrix(Matrix3f Rot, Vector3f Pos)
{
    Matrix66f A_Vtrans_B;
    Eigen::Matrix3f Pos_hat;

    // A_Vtrans_B is the adjoint transformation matrix associated with A_Trans_B
    // where A_Trans_B = | A_R_B A_p_B0 |
    //                   |   0     1    |
    // is the homogeneous transformation matrix from RF A to RF B
    // The input of this function are Rot = A_R_B and Pos = A_p_B0
    // A_Vtrans_B is a 6×6 matrix which transforms twist coordinates(∈R6) from RF A to RF B
    // A_Vtrans_B = | A_R_B  A_p_hat_B0*A_R_B |
    //              |   0         A_R_B       |
    // where A_p_hat_B0 is the skew-matrix associated with A_p_B0

    // A_Vel = | A_v | = | A_R_B  A_p_hat_B0*A_R_B |*| B_v |
    //         | A_w |   |   0         A_R_B       | | B_w |
    // where v is the linear velocity and w is the angular velocity

    A_Vtrans_B = MatrixXf::Zero(6,6);
    A_Vtrans_B.block<3,3>(0,0) = Rot;
    A_Vtrans_B.block<3,3>(3,3) = Rot;
    Pos_hat << 0,   -Pos(2),  Pos(1),
             Pos(2),   0,    -Pos(0),
            -Pos(1), Pos(0),     0;
    A_Vtrans_B.block<3,3>(0,3) = Pos_hat*Rot;

    return A_Vtrans_B;

}


VelocityJoint ReadXsens::computeJointVelocity(Rotation3f XsensRot, JointPosition XsensPos, SegmentLinearVelocity XLinVel, SegmentAngularVelocity XAngVel)
{

    VelocityJoint jointVelocity, jointVelocity_tmp;

    Vector6f g_Pelvis_Vel, g_LeftHip_Vel, g_LeftKnee_Vel, g_LeftAnkle_Vel, g_LeftToe_Vel;
    Vector6f g_RightHip_Vel, g_RightKnee_Vel, g_RightAnkle_Vel, g_RightToe_Vel;
    Vector6f g_L5_Vel, g_L3_Vel, g_T12_Vel, g_T8_Vel, g_Neck_Vel, g_Head_Vel;
    Vector6f g_LeftClavicle_Vel, g_LeftShoulder_Vel, g_LeftElbow_Vel, g_LeftWrist_Vel;
    Vector6f g_RightClavicle_Vel, g_RightShoulder_Vel, g_RightElbow_Vel, g_RightWrist_Vel;

    Vector6f Pelvis_Vel, LeftHip_Vel, LeftKnee_Vel, LeftAnkle_Vel, LeftToe_Vel;
    Vector6f RightHip_Vel, RightKnee_Vel, RightAnkle_Vel, RightToe_Vel;
    Vector6f L5_Vel, L3_Vel, T12_Vel, T8_Vel, Neck_Vel, Head_Vel;
    Vector6f LeftClavicle_Vel, LeftShoulder_Vel, LeftElbow_Vel, LeftWrist_Vel;
    Vector6f RightClavicle_Vel, RightShoulder_Vel, RightElbow_Vel, RightWrist_Vel;

    // Transform the axis to obtain an alignment of all of them in resting position (N-pose)
    // Correspondence with rotation [0][0] [1][1] [2][2]
    int LSCaxis[3]  = {1, -1, -1};
    int LSaxis[3]   = {1, -1, -1};
    int LEaxis[3]   = {1, -1, -1};
    int LWaxis[3]   = {1, -1, -1};

    int RSCaxis[3]  = {-1, 1, -1};
    int RSaxis[3]   = {-1, 1, -1};
    int REaxis[3]   = {-1, 1, -1};
    int RWaxis[3]   = {-1, 1, -1};

    int L5axis[3]   = {1, 1, 1};
    int L3axis[3]   = {1, 1, 1};
    int T12axis[3]  = {1, 1, 1};
    int T8axis[3]   = {1, 1, 1};
    int NKaxis[3]   = {1, 1, 1};
    int HDaxis[3]   = {1, 1, 1};

    int LHaxis[3]   = {1, -1, -1};
    int LKaxis[3]   = {1, -1, 1};
    int LAaxis[3]   = {1, -1, -1};
    int LTaxis[3]   = {1, -1, -1};

    int RHaxis[3]   = {-1, 1, -1};
    int RKaxis[3]   = {-1, 1, 1};
    int RAaxis[3]   = {-1, 1, -1};
    int RTaxis[3]   = {-1, 1, -1};

    // Preparing twist vectors (∈R6) composed by linear and angular velocity
    // Xsens segment linear and angular velocity are expressed in the global RF
    g_Pelvis_Vel     << XLinVel.Pelvis_LinVel[0]    , XLinVel.Pelvis_LinVel[1]    , XLinVel.Pelvis_LinVel[2]    , L5axis[0]*XAngVel.Pelvis_AngVel[0]    , L5axis[2]*XAngVel.Pelvis_AngVel[2]    , L5axis[1]*XAngVel.Pelvis_AngVel[1];
    g_LeftHip_Vel    << XLinVel.LeftHip_LinVel[0]   , XLinVel.LeftHip_LinVel[1]   , XLinVel.LeftHip_LinVel[2]   , LHaxis[0]*XAngVel.LeftHip_AngVel[0]   , LHaxis[2]*XAngVel.LeftHip_AngVel[2]   , LHaxis[1]*XAngVel.LeftHip_AngVel[1];
    g_LeftKnee_Vel   << XLinVel.LeftKnee_LinVel[0]  , XLinVel.LeftKnee_LinVel[1]  , XLinVel.LeftKnee_LinVel[2]  , LKaxis[0]*XAngVel.LeftKnee_AngVel[0]  , LKaxis[2]*XAngVel.LeftKnee_AngVel[2]  , LKaxis[1]*XAngVel.LeftKnee_AngVel[1];
    g_LeftAnkle_Vel  << XLinVel.LeftAnkle_LinVel[0] , XLinVel.LeftAnkle_LinVel[1] , XLinVel.LeftAnkle_LinVel[2] , LAaxis[0]*XAngVel.LeftAnkle_AngVel[0] , LAaxis[2]*XAngVel.LeftAnkle_AngVel[2] , LAaxis[1]*XAngVel.LeftAnkle_AngVel[1];
    g_LeftToe_Vel    << XLinVel.LeftToe_LinVel[0]   , XLinVel.LeftToe_LinVel[1]   , XLinVel.LeftToe_LinVel[2]   , LTaxis[0]*XAngVel.LeftToe_AngVel[0]   , LTaxis[2]*XAngVel.LeftToe_AngVel[2]   , LTaxis[1]*XAngVel.LeftToe_AngVel[1];
    g_RightHip_Vel   << XLinVel.RightHip_LinVel[0]  , XLinVel.RightHip_LinVel[1]  , XLinVel.RightHip_LinVel[2]  , RHaxis[0]*XAngVel.RightHip_AngVel[0]  , RHaxis[2]*XAngVel.RightHip_AngVel[2]  , RHaxis[1]*XAngVel.RightHip_AngVel[1];
    g_RightKnee_Vel  << XLinVel.RightKnee_LinVel[0] , XLinVel.RightKnee_LinVel[1] , XLinVel.RightKnee_LinVel[2] , RKaxis[0]*XAngVel.RightKnee_AngVel[0] , RKaxis[2]*XAngVel.RightKnee_AngVel[2] , RKaxis[1]*XAngVel.RightKnee_AngVel[1];
    g_RightAnkle_Vel << XLinVel.RightAnkle_LinVel[0], XLinVel.RightAnkle_LinVel[1], XLinVel.RightAnkle_LinVel[2], RAaxis[0]*XAngVel.RightAnkle_AngVel[0], RAaxis[2]*XAngVel.RightAnkle_AngVel[2], RAaxis[1]*XAngVel.RightAnkle_AngVel[1];
    g_RightToe_Vel   << XLinVel.RightToe_LinVel[0]  , XLinVel.RightToe_LinVel[1]  , XLinVel.RightToe_LinVel[2]  , RTaxis[0]*XAngVel.RightToe_AngVel[0]  , RTaxis[2]*XAngVel.RightToe_AngVel[2]  , RTaxis[1]*XAngVel.RightToe_AngVel[1];

    g_L5_Vel            << XLinVel.L5_LinVel[0] , XLinVel.L5_LinVel[1] , XLinVel.L5_LinVel[2] , L5axis[0]*XAngVel.L5_AngVel[0]  , L5axis[2]*XAngVel.L5_AngVel[2]  , L5axis[1]*XAngVel.L5_AngVel[1];
    g_L3_Vel            << XLinVel.L3_LinVel[0] , XLinVel.L3_LinVel[1] , XLinVel.L3_LinVel[2] , L3axis[0]*XAngVel.L3_AngVel[0]  , L3axis[2]*XAngVel.L3_AngVel[2]  , RHaxis[1]*XAngVel.L3_AngVel[1];
    g_T12_Vel           << XLinVel.T12_LinVel[0], XLinVel.T12_LinVel[1], XLinVel.T12_LinVel[2], T12axis[0]*XAngVel.T12_AngVel[0], T12axis[2]*XAngVel.T12_AngVel[2], L3axis[1]*XAngVel.T12_AngVel[1];
    g_T8_Vel            << XLinVel.T8_LinVel[0] , XLinVel.T8_LinVel[1] , XLinVel.T8_LinVel[2] , T8axis[0]*XAngVel.T8_AngVel[0]  , T8axis[2]*XAngVel.T8_AngVel[2]  , T12axis[1]*XAngVel.T8_AngVel[1];

    g_LeftClavicle_Vel  << XLinVel.LeftClavicle_LinVel[0] , XLinVel.LeftClavicle_LinVel[1] , XLinVel.LeftClavicle_LinVel[2] , LSaxis[0]*XAngVel.LeftClavicle_AngVel[0]  , LSaxis[2]*XAngVel.LeftClavicle_AngVel[2]  , LSaxis[1]*XAngVel.LeftClavicle_AngVel[1];
    g_LeftShoulder_Vel  << XLinVel.LeftShoulder_LinVel[0] , XLinVel.LeftShoulder_LinVel[1] , XLinVel.LeftShoulder_LinVel[2] , LSCaxis[0]*XAngVel.LeftShoulder_AngVel[0] , LSCaxis[2]*XAngVel.LeftShoulder_AngVel[2] , LSCaxis[1]*XAngVel.LeftShoulder_AngVel[1];
    g_LeftElbow_Vel     << XLinVel.LeftElbow_LinVel[0]    , XLinVel.LeftElbow_LinVel[1]    , XLinVel.LeftElbow_LinVel[2]    , LEaxis[0]*XAngVel.LeftElbow_AngVel[0]     , LEaxis[2]*XAngVel.LeftElbow_AngVel[2]     , LEaxis[1]*XAngVel.LeftElbow_AngVel[1];
    g_LeftWrist_Vel     << XLinVel.LeftWrist_LinVel[0]    , XLinVel.LeftWrist_LinVel[1]    , XLinVel.LeftWrist_LinVel[2]    , LWaxis[0]*XAngVel.LeftWrist_AngVel[0]     , LWaxis[2]*XAngVel.LeftWrist_AngVel[2]     , LWaxis[1]*XAngVel.LeftWrist_AngVel[1];
    g_RightClavicle_Vel << XLinVel.RightClavicle_LinVel[0], XLinVel.RightClavicle_LinVel[1], XLinVel.RightClavicle_LinVel[2], RSaxis[0]*XAngVel.RightClavicle_AngVel[0] , RSaxis[2]*XAngVel.RightClavicle_AngVel[2] , RSaxis[1]*XAngVel.RightClavicle_AngVel[1];
    g_RightShoulder_Vel << XLinVel.RightShoulder_LinVel[0], XLinVel.RightShoulder_LinVel[1], XLinVel.RightShoulder_LinVel[2], RSCaxis[0]*XAngVel.RightShoulder_AngVel[0], RSCaxis[2]*XAngVel.RightShoulder_AngVel[2], RSCaxis[1]*XAngVel.RightShoulder_AngVel[1];
    g_RightElbow_Vel    << XLinVel.RightElbow_LinVel[0]   , XLinVel.RightElbow_LinVel[1]   , XLinVel.RightElbow_LinVel[2]   , REaxis[0]*XAngVel.RightElbow_AngVel[0]    , REaxis[2]*XAngVel.RightElbow_AngVel[2]    , REaxis[1]*XAngVel.RightElbow_AngVel[1];
    g_RightWrist_Vel    << XLinVel.RightWrist_LinVel[0]   , XLinVel.RightWrist_LinVel[1]   , XLinVel.RightWrist_LinVel[2]   , RWaxis[0]*XAngVel.RightWrist_AngVel[0]    , RWaxis[2]*XAngVel.RightWrist_AngVel[2]    , RWaxis[1]*XAngVel.RightWrist_AngVel[1];

    g_Neck_Vel   << XLinVel.Neck_LinVel[0], XLinVel.Neck_LinVel[1], XLinVel.Neck_LinVel[2], NKaxis[0]*XAngVel.Neck_AngVel[0], NKaxis[2]*XAngVel.Neck_AngVel[2], NKaxis[1]*XAngVel.Neck_AngVel[1];
    g_Head_Vel   << XLinVel.Head_LinVel[0], XLinVel.Head_LinVel[1], XLinVel.Head_LinVel[2], HDaxis[0]*XAngVel.Head_AngVel[0], HDaxis[2]*XAngVel.Head_AngVel[2], HDaxis[1]*XAngVel.Head_AngVel[1];

    Matrix66f adJoint_Pelvis, adJoint_L5, adJoint_L3, adJoint_T12, adJoint_T8, adJoint_NK, adJoint_HD;
    Matrix66f adJoint_RSH, adJoint_RUA, adJoint_RFA, adJoint_RW;
    Matrix66f adJoint_LSH, adJoint_LUA, adJoint_LFA, adJoint_LW;
    Matrix66f adJoint_RTH, adJoint_RSK, adJoint_RFT, adJoint_RTO;
    Matrix66f adJoint_LTH, adJoint_LSK, adJoint_LFT, adJoint_LTO;

    // Compute adjoint transformation matrix associated to L_Trans_G
    // L_Trans_G is the transformation matrix from the global to the local RF
    // where L_Trans_G = | L_R_G L_p_G0 | =  | (G_R_L)^(-1) -(G_R_L)^(-1)*G_p_L0 |
    //                   |   0     1    |    |        0               1          |
    // to transform Xsens velocity from the global to the local RF

    adJoint_Pelvis = computeAdJointMatrix(XsensRot.A0.transpose(), -XsensRot.A0.transpose()*XsensPos.Pelvis_Pos);
    adJoint_LTH    = computeAdJointMatrix(XsensRot.A1.transpose(), -XsensRot.A1.transpose()*XsensPos.LeftHip_Pos);
    adJoint_LSK    = computeAdJointMatrix(XsensRot.A2.transpose(), -XsensRot.A2.transpose()*XsensPos.LeftKnee_Pos);
    adJoint_LFT    = computeAdJointMatrix(XsensRot.A3.transpose(), -XsensRot.A3.transpose()*XsensPos.LeftAnkle_Pos);
//     adJoint_LTO = computeAdJointMatrix(XsensRot.A3t.transpose(), -XsensRot.A3t.transpose()*XsensPos.LeftToe_Pos);
    adJoint_RTH    = computeAdJointMatrix(XsensRot.A4.transpose(), -XsensRot.A4.transpose()*XsensPos.RightHip_Pos);
    adJoint_RSK    = computeAdJointMatrix(XsensRot.A5.transpose(), -XsensRot.A5.transpose()*XsensPos.RightKnee_Pos);
    adJoint_RFT    = computeAdJointMatrix(XsensRot.A6.transpose(), -XsensRot.A6.transpose()*XsensPos.RightAnkle_Pos);
//     adJoint_RTO = computeAdJointMatrix(XsensRot.A6t.transpose(), -XsensRot.A6t.transpose()*XsensPos.RightToe_Pos);

    adJoint_L5     = computeAdJointMatrix(XsensRot.A7.transpose(), -XsensRot.A7.transpose()*XsensPos.L5_Pos);
    adJoint_L3     = computeAdJointMatrix(XsensRot.A8.transpose(), -XsensRot.A8.transpose()*XsensPos.L3_Pos);
    adJoint_T12    = computeAdJointMatrix(XsensRot.A9.transpose(), -XsensRot.A9.transpose()*XsensPos.T12_Pos);
    adJoint_T8     = computeAdJointMatrix(XsensRot.A10.transpose(), -XsensRot.A10.transpose()*XsensPos.T8_Pos);

//     adJoint_LSH = computeAdJointMatrix(XsensRot.A11sc.transpose(), -XsensRot.A11sc.transpose()*XsensPos.LeftClavicle_Pos);
    adJoint_LUA    = computeAdJointMatrix(XsensRot.A11.transpose(), -XsensRot.A11.transpose()*XsensPos.LeftShoulder_Pos);
    adJoint_LFA    = computeAdJointMatrix(XsensRot.A12.transpose(), -XsensRot.A12.transpose()*XsensPos.LeftElbow_Pos);
//     adJoint_LW  = computeAdJointMatrix(XsensRot.A13.transpose(), -XsensRot.A13.transpose()*XsensPos.LeftWrist_Pos);
//     adJoint_RSH = computeAdJointMatrix(XsensRot.A14sc.transpose(), -XsensRot.A14sc.transpose()*XsensPos.RightClavicle_Pos);
    adJoint_RUA    = computeAdJointMatrix(XsensRot.A14.transpose(), -XsensRot.A14.transpose()*XsensPos.RightShoulder_Pos);
    adJoint_RFA    = computeAdJointMatrix(XsensRot.A15.transpose(), -XsensRot.A15.transpose()*XsensPos.RightElbow_Pos);
//     adJoint_RW  = computeAdJointMatrix(XsensRot.A16.transpose(), -XsensRot.A16.transpose()*XsensPos.RightWrist_Pos);

    adJoint_NK     = computeAdJointMatrix(XsensRot.A17.transpose(), -XsensRot.A17.transpose()*XsensPos.Neck_Pos);
//     adJoint_HD  = computeAdJointMatrix(XsensRot.A18.transpose(), -XsensRot.A18.transpose()*XsensPos.Head_Pos);

//     std::cout << adJoint_Pelvis << std::endl;

    Pelvis_Vel        = g_Pelvis_Vel;
    LeftHip_Vel       = adJoint_Pelvis*g_LeftHip_Vel;
    LeftKnee_Vel      = adJoint_LTH*g_LeftKnee_Vel;
    LeftAnkle_Vel     = adJoint_LSK*g_LeftAnkle_Vel;
    LeftToe_Vel       = adJoint_LFT*g_LeftToe_Vel;
    RightHip_Vel      = adJoint_Pelvis*g_RightHip_Vel;
    RightKnee_Vel     = adJoint_RTH*g_RightKnee_Vel;
    RightAnkle_Vel    = adJoint_RSK*g_RightAnkle_Vel;
    RightToe_Vel      = adJoint_RFT*g_RightToe_Vel;
    L5_Vel            = adJoint_Pelvis*g_L5_Vel;
    L3_Vel            = adJoint_L5*g_L3_Vel;
    T12_Vel           = adJoint_L3*g_T12_Vel;
    T8_Vel            = adJoint_T12*g_T8_Vel;
    RightClavicle_Vel = adJoint_T8*g_RightClavicle_Vel;
    RightShoulder_Vel = adJoint_RSH*g_RightShoulder_Vel;
    RightElbow_Vel    = adJoint_RUA*g_RightElbow_Vel;
    RightWrist_Vel    = adJoint_RFA*g_RightWrist_Vel;
    LeftClavicle_Vel  = adJoint_T8*g_LeftClavicle_Vel;
    LeftShoulder_Vel  = adJoint_LSH*g_LeftShoulder_Vel;
    LeftElbow_Vel     = adJoint_LUA*g_LeftElbow_Vel;
    LeftWrist_Vel     = adJoint_LFA*g_LeftWrist_Vel;
    Neck_Vel          = adJoint_T8*g_Neck_Vel;
    Head_Vel          = adJoint_NK*g_Head_Vel;

    jointVelocity_tmp.qdot_0  = Pelvis_Vel[5];
    jointVelocity_tmp.qdot_1  = LeftHip_Vel[5];
    jointVelocity_tmp.qdot_2  = LeftKnee_Vel[5];
    jointVelocity_tmp.qdot_3  = LeftAnkle_Vel[5];
    jointVelocity_tmp.qdot_3t = LeftToe_Vel[5];
    jointVelocity_tmp.qdot_4  = RightHip_Vel[5];
    jointVelocity_tmp.qdot_5  = RightKnee_Vel[5];
    jointVelocity_tmp.qdot_6  = RightAnkle_Vel[5];
    jointVelocity_tmp.qdot_6t = RightToe_Vel[5];
    jointVelocity_tmp.qdot_7  = L5_Vel[5];
    jointVelocity_tmp.qdot_8  = L3_Vel[5];
    jointVelocity_tmp.qdot_9  = T12_Vel[5];
    jointVelocity_tmp.qdot_10 = T8_Vel[5];
    jointVelocity_tmp.qdot_11 = LeftClavicle_Vel[5] + LeftShoulder_Vel[5];
    // The velocity on the shoulder is the sum of two joints: T8-Shoulder (Clavicle) and Shoulder-UpperArm (Shoulder)
    jointVelocity_tmp.qdot_12 = LeftElbow_Vel[5];
    jointVelocity_tmp.qdot_13 = LeftWrist_Vel[5];
    jointVelocity_tmp.qdot_14 = RightClavicle_Vel[5] + RightShoulder_Vel[5];
    // The velocity on the shoulder is the sum of two joints: T8-Shoulder (Clavicle) and Shoulder-UpperArm (Shoulder)
    jointVelocity_tmp.qdot_15 = RightElbow_Vel[5];
    jointVelocity_tmp.qdot_16 = RightWrist_Vel[5];
    jointVelocity_tmp.qdot_17 = Neck_Vel[5];
    jointVelocity_tmp.qdot_18 = Head_Vel[5];

    // CHECK - sending global velocities
//     jointVelocity_tmp.qdot_0 = g_Pelvis_Vel[5];
//     jointVelocity_tmp.qdot_1 = g_LeftHip_Vel[5];
//     jointVelocity_tmp.qdot_2 = g_LeftKnee_Vel[5];
//     jointVelocity_tmp.qdot_3 = g_LeftAnkle_Vel[5];
//     jointVelocity_tmp.qdot_3t = g_LeftToe_Vel[5];
//     jointVelocity_tmp.qdot_4 = g_RightHip_Vel[5];
//     jointVelocity_tmp.qdot_5 = g_RightKnee_Vel[5];
//     jointVelocity_tmp.qdot_6 = g_RightAnkle_Vel[5];
//     jointVelocity_tmp.qdot_6t = g_RightToe_Vel[5];
//     jointVelocity_tmp.qdot_7 = g_L5_Vel[5];
//     jointVelocity_tmp.qdot_8 = g_L3_Vel[5];
//     jointVelocity_tmp.qdot_9 = g_T12_Vel[5];
//     jointVelocity_tmp.qdot_10 = g_T8_Vel[5];
//     jointVelocity_tmp.qdot_11 = g_LeftClavicle_Vel[5] + LeftShoulder_Vel[5];
    // The velocity on the shoulder is the sum of two joints: T8-Shoulder (Clavicle) and Shoulder-UpperArm (Shoulder)
//     jointVelocity_tmp.qdot_12 = g_LeftElbow_Vel[5];
//     jointVelocity_tmp.qdot_13 = g_LeftWrist_Vel[5];
//     jointVelocity_tmp.qdot_14 = g_LeftClavicle_Vel[5] + RightShoulder_Vel[5];
    // The velocity on the shoulder is the sum of two joints: T8-Shoulder (Clavicle) and Shoulder-UpperArm (Shoulder)
//     jointVelocity_tmp.qdot_15 = g_RightElbow_Vel[5];
//     jointVelocity_tmp.qdot_16 = g_RightWrist_Vel[5];
//     jointVelocity_tmp.qdot_17 = g_Neck_Vel[5];
//     jointVelocity_tmp.qdot_18 = g_Head_Vel[5];

    // Filtering the q_dot values
    float JointVel[DOF];
    JointVel[0] = jointVelocity_tmp.qdot_0;   JointVel[1] = jointVelocity_tmp.qdot_1;   JointVel[2] = jointVelocity_tmp.qdot_2;
    JointVel[3] = jointVelocity_tmp.qdot_3;   JointVel[4] = jointVelocity_tmp.qdot_3t;  JointVel[5] = jointVelocity_tmp.qdot_4;
    JointVel[6] = jointVelocity_tmp.qdot_5;   JointVel[7] = jointVelocity_tmp.qdot_6;   JointVel[8] = jointVelocity_tmp.qdot_6t;
    JointVel[9] = jointVelocity_tmp.qdot_7;   JointVel[10] = jointVelocity_tmp.qdot_8;  JointVel[11] = jointVelocity_tmp.qdot_9;
    JointVel[12] = jointVelocity_tmp.qdot_10; JointVel[13] = jointVelocity_tmp.qdot_11; JointVel[14] = jointVelocity_tmp.qdot_12;
    JointVel[15] = jointVelocity_tmp.qdot_13; JointVel[16] = jointVelocity_tmp.qdot_14; JointVel[17] = jointVelocity_tmp.qdot_15;
    JointVel[18] = jointVelocity_tmp.qdot_16; JointVel[19] = jointVelocity_tmp.qdot_17; JointVel[20] = jointVelocity_tmp.qdot_18;
    float JointVel_filt[DOF];
    for (int i = 0; i < DOF; i++){
        JointVel_filt[i] = 0.0;
    }
    float fac1, fac2;
//     fac1 = -0.9115944966; fac2 = 1.9075016260; // 240/5=48 Hz cut: 0.5Hz
    fac1 = -0.9636529842; fac2 = 1.9629800894; // 240/5=48 Hz cut: 0.2Hz
    for (int i = 0; i < DOF; i++){
        xc0[i] = xc1[i]; xc1[i] = xc2[i];
        xc2[i] = JointVel[i] / F_GAIN;
        yc0[i] = yc1[i]; yc1[i] = yc2[i];
        yc2[i] =   (xc0[i] + xc2[i]) + 2 * xc1[i] + ( fac1 * yc0[i]) + (  fac2 * yc1[i]);
        JointVel_filt[i] = yc2[i];
//         JointVel_filt[i] = JointVel[i];
    }

    jointVelocity.qdot_0  = JointVel_filt[0];
    jointVelocity.qdot_1  = JointVel_filt[1];
    jointVelocity.qdot_2  = JointVel_filt[2];
    jointVelocity.qdot_3  = JointVel_filt[3];
    jointVelocity.qdot_3t = JointVel_filt[4];
    jointVelocity.qdot_4  = JointVel_filt[5];
    jointVelocity.qdot_5  = JointVel_filt[6];
    jointVelocity.qdot_6  = JointVel_filt[7];
    jointVelocity.qdot_6t = JointVel_filt[8];
    jointVelocity.qdot_7  = JointVel_filt[9];
    jointVelocity.qdot_8  = JointVel_filt[10];
    jointVelocity.qdot_9  = JointVel_filt[11];
    jointVelocity.qdot_10 = JointVel_filt[12];
    jointVelocity.qdot_11 = JointVel_filt[13];
    jointVelocity.qdot_12 = JointVel_filt[14];
    jointVelocity.qdot_13 = JointVel_filt[15];
    jointVelocity.qdot_14 = JointVel_filt[16];
    jointVelocity.qdot_15 = JointVel_filt[17];
    jointVelocity.qdot_16 = JointVel_filt[18];
    jointVelocity.qdot_17 = JointVel_filt[19];
    jointVelocity.qdot_18 = JointVel_filt[20];

    return jointVelocity;

}

Eigen::Vector3d ReadXsens::getRPY(Eigen::Matrix3d R)
{
    Eigen::Vector3d rpy;
    double roll, pitch, yaw;

    tf2::Matrix3x3 R_tf2 = getTf2MatrixFromEigen(R);
    R_tf2.getRPY(roll, pitch, yaw);

    rpy(0) = roll;
    rpy(1) = pitch;
    rpy(2) = yaw;

    return rpy;
}

Eigen::Vector3d ReadXsens::getRPY(Eigen::Quaterniond q)
{
    return getRPY(Eigen::Matrix3d(q));
}

tf2::Matrix3x3 ReadXsens::getTf2MatrixFromEigen(Eigen::Matrix3d R)
{
  tf2::Matrix3x3 R_tf2(R(0,0), R(0,1), R(0,2),
                       R(1,0), R(1,1), R(1,2),
                       R(2,0), R(2,1), R(2,2));
  return R_tf2;
}
