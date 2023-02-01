// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ArmSubsystem.h"

using namespace ArmConstants;

ArmSubsystem::ArmSubsystem() {
    m_J1.ConfigFactoryDefault();
    m_J2.ConfigFactoryDefault();
    m_J3.ConfigFactoryDefault();
    m_J4.ConfigFactoryDefault();

    m_J1.SetNeutralMode(kNeutralMode);
    m_J2.SetNeutralMode(kNeutralMode);
    m_J3.SetNeutralMode(kNeutralMode);
    m_J4.SetNeutralMode(kNeutralMode);
};

units::degree_t ArmSubsystem::GetAngleJ1() {
    return units::degree_t{m_encoderJ1.GetRaw()};
}

units::degree_t ArmSubsystem::GetAngleJ2() {
    return units::degree_t{m_encoderJ1.GetRaw()};
}
units::degree_t ArmSubsystem::GetAngleJ3() {
    return units::degree_t{m_encoderJ1.GetRaw()};
}
units::degree_t ArmSubsystem::GetAngleJ4() {
    return units::degree_t{m_encoderJ1.GetRaw()};
}

frc::Pose3d ArmSubsystem::GetPosition() {
    frc::Pose3d pose{frc::Translation3d{0_m, 0_m, 0_m}, frc::Rotation3d{0_deg,0_deg,0_deg}};

    // first joint

    pose.Rotation().Z() += GetAngleJ1();
    pose.Translation().Z() += kJ1Length;

    // second joint

    pose.Rotation().X() += GetAngleJ2() * sin(pose.Rotation().Z());
    pose.Rotation().Y() += GetAngleJ2() * cos(pose.Rotation().Z());
    pose.Translation().X() += kJ2Length * sin(pose.Rotation().X());
    pose.Translation().Y() += kJ2Length * sin(pose.Rotation().Y());
    pose.Translation().Z() += kJ2Length * sin(pose.Rotation().Z());

    // third joint

    pose.Rotation().Y() += GetAngleJ3();
    pose.Translation().X() += kJ3Length * sin(pose.Rotation().X());
    pose.Translation().Y() += kJ3Length * sin(pose.Rotation().Y());
    pose.Translation().Z() += kJ3Length * sin(pose.Rotation().Z());

    // fourth joint
    
    pose.Rotation().Y() += GetAngleJ4();
    pose.Translation().X() += kJ4Length * sin(pose.Rotation().X());
    pose.Translation().Y() += kJ4Length * sin(pose.Rotation().Y());
    pose.Translation().Z() += kJ4Length * sin(pose.Rotation().Z());

// This method will be called once per scheduler run
void ArmSubsystem::Periodic() {}

