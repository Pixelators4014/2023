// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ArmSubsystem.h"

using namespace ArmConstants;

ArmSubsystem::ArmSubsystem()
{
    m_J1.ConfigFactoryDefault();

    m_J1.SetNeutralMode(kFalconNeutralMode);
    m_J2.SetIdleMode(kRevNeutralMode);
    m_J3.SetIdleMode(kRevNeutralMode);
    m_J4.SetIdleMode(kRevNeutralMode);
};


void ArmSubsystem::moveTo(double theta_1, double theta_2, double theta_3, double theta_4)
{
    double MatrixA[64];
    double MatrixB[32];
    A(MatrixA, theta_1, theta_2, theta_3, theta_4, 0, 0, 0, 0, 0, 0, 0, 0);
    B(MatrixB, theta_1, theta_2, theta_3, theta_4, 0, 0, 0, 0, 0, 0, 0, 0);
}
// frc::Pose3d ArmSubsystem::GetPosition(double θ1, double θ2, double θ3, double θ4)
// {
//     units::meter_t x=sin(θ1)*sin(θ2)*kL2+(cos(θ3)*sin(θ1)*sin(θ2)+cos(θ2)*sin(θ1)*sin(θ3))*kL3+(cos(θ4)*(cos(θ3)*sin(θ1)*sin(θ2)+cos(θ2)*sin(θ1)*sin(θ3))-(-cos(θ2)*cos(θ3)*sin(θ1)+sin(θ1)*sin(θ2)*sin(θ3)))*kL4;
//     units::meter_t y=-cos(θ1)*sin(θ2)*kL2+(-cos(θ1)*cos(θ3)*sin(θ2)-cos(θ1)*cos(θ2)*sin(θ3))*kL3+(cos(θ4)*(-cos(θ1)*cos(θ3)*sin(θ2)-cos(θ1)*cos(θ2)*sin(θ3))-(cos(θ1)*cos(θ2)*cos(θ3)-cos(θ1)*sin(θ2)*sin(θ3))*sin(θ4))*kL4;
//     units::meter_t z=kL1+cos(θ2)*kL2+(cos(θ2)*cos(θ3)-sin(θ2)*sin(θ3))*kL2+(cos(θ4)*cos(θ2)*cos(θ3)-sin(θ2)*sin(θ3)-(cos(θ3)*sin(θ2)+cos(θ2)*sin(θ3))*sin(θ4))*kL4;
//     Translation3d T{x, y, z};
// }

// This method will be called once per scheduler run
void ArmSubsystem::Periodic() {}
