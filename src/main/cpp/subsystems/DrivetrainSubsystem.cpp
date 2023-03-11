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
        m_leftMaster.GetSelectedSensorPosition() * kWheelEncoderMetersPerUnit,
        m_rightMaster.GetSelectedSensorPosition() * kWheelEncoderMetersPerUnit);
    m_fieldSim.SetRobotPose(m_odometry.GetPose());
}

void DrivetrainSubsystem::SimulationPeriodic() {
  m_leftMasterSim.SetBusVoltage(frc::RobotController::GetInputVoltage());
  m_rightMasterSim.SetBusVoltage(frc::RobotController::GetInputVoltage());

  m_drivetrainSimulator.SetInputs(
    units::volt_t{m_leftMasterSim.GetMotorOutputLeadVoltage()},
    units::volt_t{m_rightMasterSim.GetMotorOutputLeadVoltage()}
  );
  m_drivetrainSimulator.Update(20_ms);

  m_leftMasterSim.SetIntegratedSensorRawPosition(m_drivetrainSimulator.GetLeftPosition()/kWheelEncoderMetersPerUnit);
  m_leftMasterSim.SetIntegratedSensorVelocity(m_drivetrainSimulator.GetLeftVelocity()*1_s/kWheelEncoderMetersPerUnit);
  m_rightMasterSim.SetIntegratedSensorRawPosition(m_drivetrainSimulator.GetRightPosition()/kWheelEncoderMetersPerUnit);
  m_rightMasterSim.SetIntegratedSensorVelocity(m_drivetrainSimulator.GetRightVelocity()*1_s/kWheelEncoderMetersPerUnit);

  m_IMUSim.SetGyroAngleY(m_drivetrainSimulator.GetHeading().Degrees());
}

void DrivetrainSubsystem::ArcadeDrive(double forward, double rotation) {
  m_drive.ArcadeDrive(forward, rotation, false);
}

void DrivetrainSubsystem::ArcadeDriveF(double f) {
  forward = f;
  m_drive.ArcadeDrive(forward, rotation, false);
}

void DrivetrainSubsystem::ArcadeDriveR(double r) {
  rotation = r;
  m_drive.ArcadeDrive(forward, rotation, false);
}

void DrivetrainSubsystem::SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode NeutralMode) {
  m_leftMaster.SetNeutralMode(NeutralMode);
  m_leftFollower1.SetNeutralMode(NeutralMode);
  m_leftFollower2.SetNeutralMode(NeutralMode);
  m_rightMaster.SetNeutralMode(NeutralMode);
  m_rightFollower1.SetNeutralMode(NeutralMode);
  m_rightFollower2.SetNeutralMode(NeutralMode);
}

void DrivetrainSubsystem::ResetEncoders() {
  m_leftMaster.SetSelectedSensorPosition(0);
  m_rightMaster.SetSelectedSensorPosition(0);
}

void DrivetrainSubsystem::SetYawAxis(frc::ADIS16470_IMU::IMUAxis yaw_axis) {
  m_IMU.SetYawAxis(yaw_axis);
}

units::degree_t DrivetrainSubsystem::GetAngle() {
  return m_IMU.GetAngle();
}

frc::Pose2d DrivetrainSubsystem::GetPose() {
  return m_odometry.GetPose();
}

frc::DifferentialDriveWheelSpeeds DrivetrainSubsystem::GetWheelSpeeds() {
  return {m_leftMaster.GetSelectedSensorVelocity()*kWheelEncoderMetersPerUnit/1_s,
          m_rightMaster.GetSelectedSensorVelocity()*kWheelEncoderMetersPerUnit/1_s};
}

void DrivetrainSubsystem::TankDriveVolts(units::volt_t left, units::volt_t right) {
  m_leftMaster.SetVoltage(left);
  m_rightMaster.SetVoltage(left);
  m_drive.Feed();
}

void DrivetrainSubsystem::ResetOdometry(){
  ResetEncoders();
  m_odometry.ResetPosition(0_deg,0_m,0_m,frc::Pose2d{0_m,0_m,0_deg});
}