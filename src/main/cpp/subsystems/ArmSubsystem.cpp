// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ArmSubsystem.h"

using namespace ArmConstants;
using namespace Eigen;

ArmSubsystem::ArmSubsystem()
{
    m_J1.ConfigFactoryDefault();
    m_J2.ConfigFactoryDefault();
    m_J3.ConfigFactoryDefault();
    m_J4.ConfigFactoryDefault();

    m_J1.SetNeutralMode(kNeutralMode);
    m_J2.SetNeutralMode(kNeutralMode);
    m_J3.SetNeutralMode(kNeutralMode);
    m_J4.SetNeutralMode(kNeutralMode);
};

units::degree_t ArmSubsystem::GetAngleJ1()
{
    return units::degree_t{m_encoderJ1.GetRaw()};
}

units::degree_t ArmSubsystem::GetAngleJ2()
{
    return units::degree_t{m_encoderJ1.GetRaw()};
}
units::degree_t ArmSubsystem::GetAngleJ3()
{
    return units::degree_t{m_encoderJ1.GetRaw()};
}
units::degree_t ArmSubsystem::GetAngleJ4()
{
    return units::degree_t{m_encoderJ1.GetRaw()};
}

frc::Pose3d ArmSubsystem::GetPosition(double theta_1, double theta_2, double theta_3, double theta_4)
{
    // T_1 = [[c_1, -s_1, 0, 0], [s_1, c_1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    // T_2 = [[1, 0, 0, 0], [0, c_2, -s_2, 0], [0, s_2, c_2, L2], [0, 0, 0, 1]]
    // T_3 = [[1, 0, 0, 0], [0, c_3, -s_3, 0], [0, s_3, c_3, L3], [0, 0, 0, 1]]
    // T_4 = [[1, 0, 0, 0], [0, c_4, -s_4, 0], [0, s_4, c_4, L4], [0, 0, 0, 1]]
    // T_5 = [[1, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, L5], [0, 0, 0, 1]]
    // T_1*T_2*T_3*T_4*T_5 = 
    // Matrix<double, 4, 4> M {
    //     {0, 0, -1, 0},
    //     {0, 1, 0, 0},
    //     {-1, 0, 0, 2},
    //     {0, 0, 0, 1},
    // };
    // Matrix<double, 6, 1> S1{
    //     0,
    //     0,
    //     1,
    //     0,
    //     0,
    //     0,
    // };
    // Matrix<double, 6, 1> S2{
    //     0,
    //     -1,
    //     0,
    //     -3, // this is the length of the first arm segment
    //     0,
    //     0,
    // };
    // Matrix<double, 6, 1> S3{
    //     0,
    //     -1,
    //     0,
    //     -5, // this is the length of the first arm segment + the length of the second arm segment
    //     0,
    //     0,
    // };
    // Matrix<double, 6, 1> S4{
    //     0,
    //     -1,
    //     0,
    //     -7, // this is the length of the first arm segment + the length of the second arm segment + the length of the third arm segment
    //     0,
    //     0,
    // };



    // (theta_1*S1).exp()*(theta_2*S2).exp();//*(theta_3*S3).exp()*(theta_4*S4).exp()*M;
}
// This method will be called once per scheduler run
void ArmSubsystem::Periodic() {}
