// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

RobotContainer::RobotContainer() {
  ConfigureBindings();

  m_drive.SetDefaultCommand(frc2::RunCommand([this] {
    m_drive.ArcadeDrive(m_driverController.GetY(),m_driverController.GetX());
  },
  {&m_drive}));
}

void RobotContainer::ConfigureBindings() {}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  frc::TrajectoryConfig config(AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration);

  auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      frc::Pose2d{1_m, 2_m, 0_deg},
      {frc::Translation2d{2_m, 3_m}, frc::Translation2d{3_m, 1_m}},
      frc::Pose2d{4_m, 2_m, 0_deg},
      config);
  
  frc2::CommandPtr ramseteCommand{frc2::RamseteCommand(
      trajectory, [this] { return m_drive.GetPose(); },
      frc::RamseteController{AutoConstants::kRamseteB,
                             AutoConstants::kRamseteZeta},
      frc::SimpleMotorFeedforward<units::meters>{
          DriveConstants::ks, DriveConstants::kv, DriveConstants::ka},
      DriveConstants::kDriveKinematics,
      [this] { return m_drive.GetWheelSpeeds(); },
      frc2::PIDController{DriveConstants::kPDriveVel, 0, 0},
      frc2::PIDController{DriveConstants::kPDriveVel, 0, 0},
      [this](auto left, auto right) { m_drive.TankDriveVolts(left, right); },
      {&m_drive})};

  return ramseteCommand;
}
