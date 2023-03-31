// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <frc/DoubleSolenoid.h>

#include "Constants.h"

class GrabberSubsystem : public frc2::SubsystemBase {
 public:
  GrabberSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void setSpeed(double speed);

  void setPiston(frc::DoubleSolenoid::Value state);

 private:
  rev::CANSparkMax m_leftMotor{GrabberConstants::kLeftID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightMotor{GrabberConstants::kRightID, rev::CANSparkMax::MotorType::kBrushless};
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  frc::DoubleSolenoid m_leftPiston{frc::PneumaticsModuleType::REVPH, GrabberConstants::kLeftPiston1ID, GrabberConstants::kLeftPiston2ID};
  frc::DoubleSolenoid m_rightPiston{frc::PneumaticsModuleType::REVPH, GrabberConstants::kRightPiston1ID, GrabberConstants::kRightPiston2ID};
};
