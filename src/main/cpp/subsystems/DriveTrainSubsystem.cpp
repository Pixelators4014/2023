// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveTrainSubsystem.h"

driveTrain::driveTrain() = default;

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


