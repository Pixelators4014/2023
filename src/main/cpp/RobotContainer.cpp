// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

RobotContainer::RobotContainer() {
  ConfigureBindings();

  m_drive.SetDefaultCommand(frc2::RunCommand([this] {
    m_drive.ArcadeDrive(-m_driverController.GetY(),-m_driverController.GetX()*(m_driverController.GetRawButton(OIConstants::driverControllerHalfRotationSpeedButton)?1:.5));
  },
  {&m_drive}));
}

void RobotContainer::ConfigureBindings() {
  frc2::JoystickButton(&m_driverController,OIConstants::driverControllerBrakeModeButton).OnFalse(frc2::cmd::Run([this]{m_drive.SetNeutralMode(NeutralMode::Brake);}));
  frc2::JoystickButton(&m_driverController,OIConstants::driverControllerCoastModeButton).OnFalse(frc2::cmd::Run([this]{m_drive.SetNeutralMode(NeutralMode::Coast);}));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {

  m_drive.ResetOdometry();

  frc::TrajectoryConfig config(AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration);

  auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      frc::Pose2d{0_m, 0_m, 0_deg},
      {},
      frc::Pose2d{2_m, 0_m, 0_deg},
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


  // z axis

  // frc2::CommandPtr RPIDCommand{frc2::PIDCommand(
  //   frc::PIDController{AutoConstants::kRP,AutoConstants::kRI,AutoConstants::kRD},
  //   [this] { return m_drive.GetAngle().value(); },
  //   [this] { return 0.0; },
  //   [this](auto r) { m_drive.ArcadeDriveR(r); },
  //   {&m_drive}
  // )};

  m_drive.SetYawAxis(frc::ADIS16470_IMU::IMUAxis::kY);

  frc2::CommandPtr FPIDCommand{frc2::PIDCommand(
    frc::PIDController{AutoConstants::kFP,AutoConstants::kFI,AutoConstants::kFD},
    [this] { return m_drive.GetAngle().value(); },
    [this] { return 0.0; },
    [this](auto f) { m_drive.ArcadeDriveF(f - AutoConstants::kMotorStrength*sin(m_drive.GetAngle().value()/(180.0/M_PI))); },
    {&m_drive}
  )};
  return frc2::cmd::Sequence(
    std::move(ramseteCommand),
    std::move(FPIDCommand)
  );
}
