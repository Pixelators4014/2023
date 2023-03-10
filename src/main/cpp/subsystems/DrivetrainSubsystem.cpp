// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DrivetrainSubsystem.h"

using namespace DriveConstants;

DrivetrainSubsystem::DrivetrainSubsystem() {
    m_leftMaster.ConfigFactoryDefault();
    m_leftFollower1.ConfigFactoryDefault();
    m_leftFollower2.ConfigFactoryDefault();
    m_rightMaster.ConfigFactoryDefault();
    m_rightFollower1.ConfigFactoryDefault();
    m_rightFollower2.ConfigFactoryDefault();

    m_leftMaster.SetInverted(kLeftMotorInverted);
    m_leftFollower1.SetInverted(kLeftMotorInverted);
    m_leftFollower2.SetInverted(kLeftMotorInverted);
    m_rightMaster.SetInverted(kRightMotorInverted);
    m_rightFollower1.SetInverted(kRightMotorInverted);
    m_rightFollower2.SetInverted(kRightMotorInverted);

    m_leftFollower1.Follow(m_leftMaster);
    m_leftFollower2.Follow(m_leftMaster);
    m_rightFollower1.Follow(m_rightMaster);
    m_rightFollower2.Follow(m_rightMaster);

    m_leftMaster.SetNeutralMode(kNeutralMode);
    m_leftFollower1.SetNeutralMode(kNeutralMode);
    m_leftFollower2.SetNeutralMode(kNeutralMode);
    m_rightMaster.SetNeutralMode(kNeutralMode);
    m_rightFollower1.SetNeutralMode(kNeutralMode);
    m_rightFollower2.SetNeutralMode(kNeutralMode);

    ResetEncoders();

    frc::SmartDashboard::PutData("Field", &m_fieldSim);
};

// This method will be called once per scheduler run
void DrivetrainSubsystem::Periodic() {
    m_odometry.Update(m_IMU.GetAngle(),
        units::meter_t{m_leftMaster.GetSelectedSensorPosition() * kWheelEncoderMetersPerUnit},
        units::meter_t{m_rightMaster.GetSelectedSensorPosition() * kWheelEncoderMetersPerUnit});
    m_fieldSim.SetRobotPose(m_odometry.GetPose());
}

void DrivetrainSubsystem::SimulationPeriodic() {
    m_drivetrainSimulator.SetInputs(units::volt_t{m_leftMaster.Get()} *
            frc::RobotController::GetInputVoltage(),
        units::volt_t{m_rightMaster.Get()} *
            frc::RobotController::GetInputVoltage());
    m_drivetrainSimulator.Update(20_ms);

    m_IMUSim.SetGyroAngleY(-m_drivetrainSimulator.GetHeading().Degrees()); // make sure this is the right axis
}

units::ampere_t DrivetrainSubsystem::GetCurrentDraw() const {
  return m_drivetrainSimulator.GetCurrentDraw();
}

void DrivetrainSubsystem::ArcadeDrive(double fwd, double rot, bool squaredInputs) {
  m_drive.ArcadeDrive(fwd, rot, squaredInputs);
}

void DrivetrainSubsystem::ResetEncoders() {
  m_leftMaster.SetSelectedSensorPosition(0);
  m_rightMaster.SetSelectedSensorPosition(0);
}

units::degree_t DrivetrainSubsystem::GetHeading() const {
  return m_IMU.GetAngle();
}

units::angular_velocity::degrees_per_second_t DrivetrainSubsystem::GetTurnRate() {
  return m_IMU.GetRate();
}

frc::Pose2d DrivetrainSubsystem::GetPose() {
  return m_odometry.GetPose();
}

frc::DifferentialDriveWheelSpeeds DrivetrainSubsystem::GetWheelSpeeds() {
  return {units::meters_per_second_t{m_leftMaster.GetSelectedSensorVelocity()},
          units::meters_per_second_t{m_rightMaster.GetSelectedSensorVelocity()}};
}

void DrivetrainSubsystem::ResetOdometry(frc::Pose2d pose) {
  ResetEncoders();
  m_drivetrainSimulator.SetPose(pose);
  m_odometry.ResetPosition(m_IMU.GetAngle(),
                           m_leftMaster.GetSelectedSensorPosition() * kWheelEncoderMetersPerUnit,
                           m_rightMaster.GetSelectedSensorPosition() * kWheelEncoderMetersPerUnit, pose);
}