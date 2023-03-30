// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <frc/Solenoid.h>

#include "Constants.h"

class GrabberSubsystem : public frc2::SubsystemBase {
 public:
  GrabberSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void setSpeed(double speed);

  void setPiston(bool state);

 private:
  rev::CANSparkMax m_leftMotor{GrabberConstants::kLeftID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightMotor{GrabberConstants::kRightID, rev::CANSparkMax::MotorType::kBrushless};
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  frc::Solenoid m_piston1{frc::PneumaticsModuleType::REVPH, GrabberConstants::kPiston1ID};
  frc::Solenoid m_piston2{frc::PneumaticsModuleType::REVPH, GrabberConstants::kPiston2ID};
};
