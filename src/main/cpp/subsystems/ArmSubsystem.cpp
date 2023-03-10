// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ArmSubsystem.h"

using namespace ArmConstants;

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

// frc::Pose3d ArmSubsystem::GetPosition(double θ1, double θ2, double θ3, double θ4)
// {
//     units::meter_t x=sin(θ1)*sin(θ2)*kL2+(cos(θ3)*sin(θ1)*sin(θ2)+cos(θ2)*sin(θ1)*sin(θ3))*kL3+(cos(θ4)*(cos(θ3)*sin(θ1)*sin(θ2)+cos(θ2)*sin(θ1)*sin(θ3))-(-cos(θ2)*cos(θ3)*sin(θ1)+sin(θ1)*sin(θ2)*sin(θ3)))*kL4;
//     units::meter_t y=-cos(θ1)*sin(θ2)*kL2+(-cos(θ1)*cos(θ3)*sin(θ2)-cos(θ1)*cos(θ2)*sin(θ3))*kL3+(cos(θ4)*(-cos(θ1)*cos(θ3)*sin(θ2)-cos(θ1)*cos(θ2)*sin(θ3))-(cos(θ1)*cos(θ2)*cos(θ3)-cos(θ1)*sin(θ2)*sin(θ3))*sin(θ4))*kL4;
//     units::meter_t z=kL1+cos(θ2)*kL2+(cos(θ2)*cos(θ3)-sin(θ2)*sin(θ3))*kL2+(cos(θ4)*cos(θ2)*cos(θ3)-sin(θ2)*sin(θ3)-(cos(θ3)*sin(θ2)+cos(θ2)*sin(θ3))*sin(θ4))*kL4;
//     Translation3d T{x, y, z};
// }
// This method will be called once per scheduler run
void ArmSubsystem::Periodic() {}
