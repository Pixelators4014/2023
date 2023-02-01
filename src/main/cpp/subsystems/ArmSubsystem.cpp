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



// This method will be called once per scheduler run
void ArmSubsystem::Periodic() {}

