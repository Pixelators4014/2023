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

driveTrain::driveTrain() {
  // We need to invert one side of the drivetrain so that positive voltages
  // result in both sides moving forward. Depending on how your robot's
  // gearbox is constructed, you might have to invert the left side instead.
  m_rightMotors.SetInverted(true);

  // Set the distance per pulse for the encoders
  m_leftEncoder.SetDistancePerPulse(kEncoderDistancePerPulse);
  m_rightEncoder.SetDistancePerPulse(kEncoderDistancePerPulse);

  ResetEncoders();
  frc::SmartDashboard::PutData("Field", &m_fieldSim);
}


// This method will be called once per scheduler run
<<<<<<< HEAD
void driveTrain::Periodic() {
    // Implementation of subsystem periodic method goes here.
  m_odometry.Update(m_gyro.GetRotation2d(),
                    units::meter_t{m_leftEncoder.GetDistance()},
                    units::meter_t{m_rightEncoder.GetDistance()});
  m_fieldSim.SetRobotPose(m_odometry.GetPose());

}

void driveTrain::SimulationPeriodic() {
  // To update our simulation, we set motor voltage inputs, update the
  // simulation, and write the simulated positions and velocities to our
  // simulated encoder and gyro. We negate the right side so that positive
  // voltages make the right side move forward.
  m_drivetrainSimulator.SetInputs(units::volt_t{m_leftMotors.Get()} *
                                      frc::RobotController::GetInputVoltage(),
                                  units::volt_t{m_rightMotors.Get()} *
                                      frc::RobotController::GetInputVoltage());
  m_drivetrainSimulator.Update(20_ms);

  m_leftEncoderSim.SetDistance(m_drivetrainSimulator.GetLeftPosition().value());
  m_leftEncoderSim.SetRate(m_drivetrainSimulator.GetLeftVelocity().value());
  m_rightEncoderSim.SetDistance(
      m_drivetrainSimulator.GetRightPosition().value());
  m_rightEncoderSim.SetRate(m_drivetrainSimulator.GetRightVelocity().value());
  m_gyroSim.SetAngle(-m_drivetrainSimulator.GetHeading().Degrees());
}

units::ampere_t driveTrain::GetCurrentDraw() const {
  return m_drivetrainSimulator.GetCurrentDraw();
}

void driveTrain::ArcadeDrive(double fwd, double rot) {
  m_drive.ArcadeDrive(fwd, rot);
}

void driveTrain::TankDriveVolts(units::volt_t left, units::volt_t right) {
  m_leftMotors.SetVoltage(left);
  m_rightMotors.SetVoltage(right);
  m_drive.Feed();
}

void driveTrain::ResetEncoders() {
  m_leftEncoder.Reset();
  m_rightEncoder.Reset();
}

double driveTrain::GetAverageEncoderDistance() {
  return (m_leftEncoder.GetDistance() + m_rightEncoder.GetDistance()) / 2.0;
}

frc::Encoder& driveTrain::GetLeftEncoder() {
  return m_leftEncoder;
}

frc::Encoder& driveTrain::GetRightEncoder() {
  return m_rightEncoder;
}

void driveTrain::SetMaxOutput(double maxOutput) {
  m_drive.SetMaxOutput(maxOutput);
}

units::degree_t driveTrain::GetHeading() const {
  return m_gyro.GetRotation2d().Degrees();
}

double driveTrain::GetTurnRate() {
  return -m_gyro.GetRate();
}

frc::Pose2d driveTrain::GetPose() {
  return m_odometry.GetPose();
}

frc::DifferentialDriveWheelSpeeds driveTrain::GetWheelSpeeds() {
  return {units::meters_per_second_t{m_leftEncoder.GetRate()},
          units::meters_per_second_t{m_rightEncoder.GetRate()}};
}

void driveTrain::ResetOdometry(frc::Pose2d pose) {
  ResetEncoders();
  m_drivetrainSimulator.SetPose(pose);
  m_odometry.ResetPosition(m_gyro.GetRotation2d(),
                           units::meter_t{m_leftEncoder.GetDistance()},
                           units::meter_t{m_rightEncoder.GetDistance()}, pose);
}


=======
void DrivetrainSubsystem::Periodic() {
    m_odometry.Update(m_IMU.GetRotation2d(),
        units::meter_t{m_leftMaster.GetSelectedSensorPosition() / kEncoderUnitsPerInch},
        units::meter_t{m_rightMaster.GetSelectedSensorPosition() / kEncoderUnitsPerInch});
    m_fieldSim.SetRobotPose(m_odometry.GetPose());
}

void DrivetrainSubsystem::SimulationPeriodic() {
    m_drivetrainSimulator.SetInputs(units::volt_t{m_leftMaster.Get()} *
            frc::RobotController::GetInputVoltage(),
        units::volt_t{m_rightMaster.Get()} *
            frc::RobotController::GetInputVoltage());
    m_drivetrainSimulator.Update(20_ms);

    m_leftEncoderSim.SetDistance(m_drivetrainSimulator.GetLeftPosition().value());
    m_leftEncoderSim.SetRate(m_drivetrainSimulator.GetLeftVelocity().value());
    m_rightEncoderSim.SetDistance(m_drivetrainSimulator.GetRightPosition().value());
    m_rightEncoderSim.SetRate(m_drivetrainSimulator.GetRightVelocity().value());
    m_IMUSim.SetAngle(-m_drivetrainSimulator.GetHeading().Degrees());
}

units::ampere_t DrivetrainSubsystem::GetCurrentDraw() const {
  return m_drivetrainSimulator.GetCurrentDraw();
}

void DrivetrainSubsystem::ArcadeDrive(double fwd, double rot) {
  m_drive.ArcadeDrive(fwd, rot);
}

void DrivetrainSubsystem::ResetEncoders() {
  m_leftMaster.SetSelectedSensorPosition(0);
  m_rightMaster.SetSelectedSensorPosition(0);
}

void DrivetrainSubsystem::SetMaxOutput(double maxOutput) {
  m_drive.SetMaxOutput(maxOutput);
}

units::degree_t DrivetrainSubsystem::GetHeading() const {
  return m_IMU.GetRotation2d().Degrees();
}

double DrivetrainSubsystem::GetTurnRate() {
  return -m_IMU.GetRate();
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
  m_odometry.ResetPosition(m_gyro.GetRotation2d(),
                           units::meter_t{m_leftMaster.GetSelectedSensorPosition() / kEncoderUnitsPerInch},
                           units::meter_t{m_rightMaster.GetSelectedSensorPosition() / kEncoderUnitsPerInch}, pose);
}
>>>>>>> 0e048e474707d950337c44a3603c6b99123eb24f
