// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

class DrivetrainSubsystem : public frc2::SubsystemBase {
 public:
  DrivetrainSubsystem();

      void ArcadeDrive (double forward, double rotation, bool squareInputs);
      void ResetEncoders();
      double GetLeftEncoderDistance();
      double GetRightEncoderDistance();
      double GetLeftEncoderVelocity();
      double GetRightEncoderVelocity();


  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
      WPI_TalonFX m_leftFront;
      WPI_TalonFX m_leftMiddle;
      WPI_TalonFX m_leftBack;
      WPI_TalonFX m_rightFront;
      WPI_TalonFX m_rightMiddle;
      WPI_TalonFX m_rightBack;

      frc::DifferentialDrive m_drive{m_leftFront, m_rightFront};
};
