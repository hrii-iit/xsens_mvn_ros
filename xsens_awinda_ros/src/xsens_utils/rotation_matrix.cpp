#include <xsens_utils/rotation_matrix.h>

#include <iostream>
#include <fstream>

extern Matrix3f rotx (float rad) {
    s = sinf (rad);
    c = cosf (rad);
    tmp << 1., 0., 0.,
           0.,  c,  -s,
           0., s,  c;
    return tmp;
}

extern Matrix3f roty (float rad) {
    s = sinf (rad);
    c = cosf (rad);
    tmp <<  c, 0., s,
            0., 1., 0.,
            -s, 0., c;
    return tmp;
}

extern Matrix3f rotz (float rad) {
    s = sinf (rad);
    c = cosf (rad);
    tmp << c, -s, 0.,
           s, c, 0.,
           0., 0., 1.;
    return tmp;
}


extern Quaternionf rot2quat(Matrix3f rotation_matrix) {

  Quaternionf quaternion(rotation_matrix);
  quaternion.normalize();
  
  return quaternion;
}

extern Quaternionf vec2quat(const float *vector){
    q.w() = vector[0];
    q.x() = vector[1];
    q.y() = vector[2];
    q.z() = vector[3];
    q.normalize();

    return q;
}

Rotation3f RotationMatrix::QuaternionMatrix(QuaternionJoint QuaternionJoint_) {
    
    struct Rotation3f R;

    R.A0    = QuaternionJoint_.Q0.toRotationMatrix();        // Pelvis     
    R.A1    = QuaternionJoint_.Q1.toRotationMatrix();        // L-Hip
    R.A2    = QuaternionJoint_.Q2.toRotationMatrix();        // L-Knee
    R.A3    = QuaternionJoint_.Q3.toRotationMatrix();        // L-Ankle
    R.A3t   = QuaternionJoint_.Q3t.toRotationMatrix();       // L-Toe
    R.A4    = QuaternionJoint_.Q4.toRotationMatrix();        // R-Hip
    R.A5    = QuaternionJoint_.Q5.toRotationMatrix();        // R-Knee
    R.A6    = QuaternionJoint_.Q6.toRotationMatrix();        // R-Ankle
    R.A6t   = QuaternionJoint_.Q6t.toRotationMatrix();       // R-Toe
    R.A7    = QuaternionJoint_.Q7.toRotationMatrix();        // L5S1
    R.A8    = QuaternionJoint_.Q8.toRotationMatrix();        // L4L3
    R.A9    = QuaternionJoint_.Q9.toRotationMatrix();        // T12L1
    R.A10   = QuaternionJoint_.Q10.toRotationMatrix();       // T9T8
    R.A11sc = QuaternionJoint_.Q11.toRotationMatrix();       // L-Clavicle
    R.A11   = QuaternionJoint_.Q11.toRotationMatrix();       // L-UpperArm
    R.A12   = QuaternionJoint_.Q12.toRotationMatrix();       // L-Elbow
    R.A13   = QuaternionJoint_.Q13.toRotationMatrix();       // L-Wrist
    R.A14sc = QuaternionJoint_.Q11.toRotationMatrix();       // R-Clavicle
    R.A14   = QuaternionJoint_.Q14.toRotationMatrix();       // R-UpperArm
    R.A15   = QuaternionJoint_.Q15.toRotationMatrix();       // R-Elbow
    R.A16   = QuaternionJoint_.Q16.toRotationMatrix();       // R-Wrist
    R.A17   = QuaternionJoint_.Q17.toRotationMatrix();       // Neck
    R.A18   = QuaternionJoint_.Q18.toRotationMatrix();       // Head
    
//     std::cout << "TestRotation A1: " << R.A1 << std::endl;

    return R;
}


 Rotation3f RotationMatrix::AngleMatrix(AngleJoint AngleJoint_)const{
     
    struct Rotation3f R;
    //Global to Base frame
    R.A0= AngleAxisf(AngleJoint_.q0, Vector3f::UnitY());
    // Base frame to left leg
    R.A1 = R.A0*AngleAxisf(AngleJoint_.q1, Vector3f::UnitY());
    R.A2 = R.A1*AngleAxisf(AngleJoint_.q2, Vector3f::UnitY());
    R.A3 = R.A2*AngleAxisf(AngleJoint_.q3, Vector3f::UnitY());
    // Base frame to right leg
    R.A4 = R.A0*AngleAxisf(AngleJoint_.q4, Vector3f::UnitY());
    R.A5 = R.A4*AngleAxisf(AngleJoint_.q5, Vector3f::UnitY());
    R.A6 = R.A5*AngleAxisf(AngleJoint_.q6, Vector3f::UnitY());    
    // Base frame to T8
    R.A7 = R.A0*AngleAxisf(AngleJoint_.q7, Vector3f::UnitY());
    R.A8 = R.A7*AngleAxisf(AngleJoint_.q8, Vector3f::UnitY());
    R.A9 = R.A8*AngleAxisf(AngleJoint_.q9, Vector3f::UnitY());
    R.A10 = R.A9*AngleAxisf(AngleJoint_.q10, Vector3f::UnitY());
    // Base frame to left arm
    R.A11 = R.A10*AngleAxisf(AngleJoint_.q11, Vector3f::UnitY());
    R.A12 = R.A11*AngleAxisf(AngleJoint_.q12, Vector3f::UnitY());
    // Base frame to right arm
    R.A14 = R.A10*AngleAxisf(AngleJoint_.q14, Vector3f::UnitY());
    R.A15 = R.A14*AngleAxisf(AngleJoint_.q15, Vector3f::UnitY());
    // Base frame to head
    R.A17 = R.A10*AngleAxisf(AngleJoint_.q17, Vector3f::UnitY());
    R.A18 = R.A17*AngleAxisf(AngleJoint_.q18, Vector3f::UnitY());

    return R;
}
 
Rotation3f RotationMatrix::AngleMatrix3D(AngleJoint3D AngleJoint3D_)const{
   
    struct Rotation3f R;
    // Euler sequence ZXY the rotation matrix is R = R_y*R_x*R_z
    //Global to Base frame
    R.A0 = AngleAxisf(AngleJoint3D_.q0_x, Vector3f::UnitX())
         * AngleAxisf(AngleJoint3D_.q0_y, Vector3f::UnitY())
         * AngleAxisf(AngleJoint3D_.q0_z, Vector3f::UnitZ());
     
    // Base frame to left leg
    R.A1 = R.A0
         * AngleAxisf(AngleJoint3D_.q1_y, Vector3f::UnitY())
         * AngleAxisf(AngleJoint3D_.q1_x, Vector3f::UnitX())
         * AngleAxisf(AngleJoint3D_.q1_z, Vector3f::UnitZ());
    R.A2 = R.A1
         * AngleAxisf(AngleJoint3D_.q2_y, Vector3f::UnitY())
         * AngleAxisf(AngleJoint3D_.q2_x, Vector3f::UnitX())
         * AngleAxisf(AngleJoint3D_.q2_z, Vector3f::UnitZ());
    R.A3 = R.A2
         * AngleAxisf(AngleJoint3D_.q3_y, Vector3f::UnitY())
         * AngleAxisf(AngleJoint3D_.q3_x, Vector3f::UnitX())
         * AngleAxisf(AngleJoint3D_.q3_z, Vector3f::UnitZ());
    R.A3t = R.A3
         * AngleAxisf(AngleJoint3D_.q3t_y, Vector3f::UnitY())
         * AngleAxisf(AngleJoint3D_.q3t_x, Vector3f::UnitX())
         * AngleAxisf(AngleJoint3D_.q3t_z, Vector3f::UnitZ());
         
    // Base frame to right leg
    R.A4 = R.A0
         * AngleAxisf(AngleJoint3D_.q4_y, Vector3f::UnitY())
         * AngleAxisf(AngleJoint3D_.q4_x, Vector3f::UnitX())
         * AngleAxisf(AngleJoint3D_.q4_z, Vector3f::UnitZ());
    R.A5 = R.A4
         * AngleAxisf(AngleJoint3D_.q5_y, Vector3f::UnitY())
         * AngleAxisf(AngleJoint3D_.q5_x, Vector3f::UnitX())
         * AngleAxisf(AngleJoint3D_.q5_z, Vector3f::UnitZ());
    R.A6 = R.A5
         * AngleAxisf(AngleJoint3D_.q6_y, Vector3f::UnitY())
         * AngleAxisf(AngleJoint3D_.q6_x, Vector3f::UnitX())
         * AngleAxisf(AngleJoint3D_.q6_z, Vector3f::UnitZ());
    R.A6t = R.A6
         * AngleAxisf(AngleJoint3D_.q6t_y, Vector3f::UnitY())
         * AngleAxisf(AngleJoint3D_.q6t_x, Vector3f::UnitX())
         * AngleAxisf(AngleJoint3D_.q6t_z, Vector3f::UnitZ());

    // Base frame to T8
    R.A7 = R.A0
         * AngleAxisf(AngleJoint3D_.q7_y, Vector3f::UnitY())
         * AngleAxisf(AngleJoint3D_.q7_x, Vector3f::UnitX())
         * AngleAxisf(AngleJoint3D_.q7_z, Vector3f::UnitZ());
    R.A8 = R.A7
         * AngleAxisf(AngleJoint3D_.q8_y, Vector3f::UnitY())
         * AngleAxisf(AngleJoint3D_.q8_x, Vector3f::UnitX())
         * AngleAxisf(AngleJoint3D_.q8_z, Vector3f::UnitZ());
    R.A9 = R.A8
         * AngleAxisf(AngleJoint3D_.q9_y, Vector3f::UnitY())
         * AngleAxisf(AngleJoint3D_.q9_x, Vector3f::UnitX())
         * AngleAxisf(AngleJoint3D_.q9_z, Vector3f::UnitZ());  
    R.A10 = R.A9
         * AngleAxisf(AngleJoint3D_.q10_y, Vector3f::UnitY())
         * AngleAxisf(AngleJoint3D_.q10_x, Vector3f::UnitX())
         * AngleAxisf(AngleJoint3D_.q10_z, Vector3f::UnitZ());
         
    // T8 to left arm
    R.A11sc = R.A10
         * AngleAxisf(AngleJoint3D_.q11sc_y, Vector3f::UnitY())
         * AngleAxisf(AngleJoint3D_.q11sc_x, Vector3f::UnitX())
         * AngleAxisf(AngleJoint3D_.q11sc_z, Vector3f::UnitZ());
    R.A11 = R.A11sc
         * AngleAxisf(AngleJoint3D_.q11_y, Vector3f::UnitY())
         * AngleAxisf(AngleJoint3D_.q11_x, Vector3f::UnitX())
         * AngleAxisf(AngleJoint3D_.q11_z, Vector3f::UnitZ());
    R.A12 = R.A11
         * AngleAxisf(AngleJoint3D_.q12_y, Vector3f::UnitY())
         * AngleAxisf(AngleJoint3D_.q12_x, Vector3f::UnitX())
         * AngleAxisf(AngleJoint3D_.q12_z, Vector3f::UnitZ()); 
    R.A13 = R.A12
         * AngleAxisf(AngleJoint3D_.q13_y, Vector3f::UnitY())
         * AngleAxisf(AngleJoint3D_.q13_x, Vector3f::UnitX())
         * AngleAxisf(AngleJoint3D_.q13_z, Vector3f::UnitZ());

    // T8 to right arm
    R.A14sc = R.A10
         * AngleAxisf(AngleJoint3D_.q14sc_y, Vector3f::UnitY())
         * AngleAxisf(AngleJoint3D_.q14sc_x, Vector3f::UnitX())
         * AngleAxisf(AngleJoint3D_.q14sc_z, Vector3f::UnitZ());
    R.A14 = R.A14sc
         * AngleAxisf(AngleJoint3D_.q14_y, Vector3f::UnitY())
         * AngleAxisf(AngleJoint3D_.q14_x, Vector3f::UnitX())
         * AngleAxisf(AngleJoint3D_.q14_z, Vector3f::UnitZ());
    R.A15 = R.A14
         * AngleAxisf(AngleJoint3D_.q15_y, Vector3f::UnitY())
         * AngleAxisf(AngleJoint3D_.q15_x, Vector3f::UnitX())
         * AngleAxisf(AngleJoint3D_.q15_z, Vector3f::UnitZ()); 
    R.A16 = R.A15
         * AngleAxisf(AngleJoint3D_.q16_y, Vector3f::UnitY())
         * AngleAxisf(AngleJoint3D_.q16_x, Vector3f::UnitX())
         * AngleAxisf(AngleJoint3D_.q16_z, Vector3f::UnitZ());
         
    // T8 to Head
    R.A17 = R.A10
         * AngleAxisf(AngleJoint3D_.q17_y, Vector3f::UnitY())
         * AngleAxisf(AngleJoint3D_.q17_x, Vector3f::UnitX())
         * AngleAxisf(AngleJoint3D_.q17_z, Vector3f::UnitZ());
    R.A18 = R.A17
         * AngleAxisf(AngleJoint3D_.q18_y, Vector3f::UnitY())
         * AngleAxisf(AngleJoint3D_.q18_x, Vector3f::UnitX())
         * AngleAxisf(AngleJoint3D_.q18_z, Vector3f::UnitZ());
         
    return R;
}
