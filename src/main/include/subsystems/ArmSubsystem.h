// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc/geometry/Pose3d.h>
#include <frc/Encoder.h>

#include <ctre/Phoenix.h>

#include <Eigen/Core>

#include <Constants.h>

class ArmSubsystem : public frc2::SubsystemBase {
 public:
  ArmSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  units::degree_t GetAngleJ1();
  units::degree_t GetAngleJ2();
  units::degree_t GetAngleJ3();
  units::degree_t GetAngleJ4();

  units::degrees_per_second GetVelocityJ1();
  units::degrees_per_second GetVelocityJ2();
  units::degrees_per_second GetVelocityJ3();
  units::degrees_per_second GetVelocityJ4();

  frc::Pose3d GetPosition();

  void GoToAngleJ1(units::degree_t angle);
  void GoToAngleJ2(units::degree_t angle);
  void GoToAngleJ3(units::degree_t angle);
  void GoToAngleJ4(units::degree_t angle);

  void GoToPosition(frc::Pose3d position);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  frc::Encoder m_encoderJ1{0, 1};
  frc::Encoder m_encoderJ2{0, 1};
  frc::Encoder m_encoderJ3{0, 1};
  frc::Encoder m_encoderJ4{0, 1};

  WPI_TalonFX m_J1{ArmConstants::kJ1ID};
  WPI_TalonFX m_J2{ArmConstants::kJ2ID};
  WPI_TalonFX m_J3{ArmConstants::kJ3ID};
  WPI_TalonFX m_J4{ArmConstants::kJ4ID};
};
