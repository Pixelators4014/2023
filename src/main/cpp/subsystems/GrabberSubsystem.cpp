// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/GrabberSubsystem.h"

using namespace GrabberConstants;

GrabberSubsystem::GrabberSubsystem() {
    m_leftMotor.SetInverted(kLeftInverted);
    m_rightMotor.SetInverted(kRightInverted);

    m_leftMotor.SetIdleMode(kNeutralMode);
    m_rightMotor.SetIdleMode(kNeutralMode);
}

void GrabberSubsystem::setSpeed(double speed){
    m_leftMotor.Set(speed);
    m_rightMotor.Set(speed);
}

void GrabberSubsystem::setPiston(frc::DoubleSolenoid::Value state){
    m_leftPiston.Set(state);
    m_rightPiston.Set(state);
}

// This method will be called once per scheduler run
void GrabberSubsystem::Periodic() {}