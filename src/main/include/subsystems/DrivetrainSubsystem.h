// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <Constants.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/ADIS16470_IMU.h>
#include <frc/simulation/ADIS16470_IMUSim.h>
#include <frc/Encoder.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/simulation/DifferentialDrivetrainSim.h>
#include <frc/simulation/EncoderSim.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/RobotController.h>
#include <frc2/command/SubsystemBase.h>
#include <units/voltage.h>

#include <ctre/Phoenix.h>

class DrivetrainSubsystem : public frc2::SubsystemBase {
 public:
  DrivetrainSubsystem();

    units::ampere_t GetCurrentDraw() const;

    /**
    * Drives the robot using arcade controls.
    *
    * @param forward the commanded forward movement
    * @param rotation the commanded rotation
    */
    void ArcadeDrive(double forward, double rotation, bool squareInputs);

    /**
    * Resets the drive encoders to currently read a position of 0.
    */
    void ResetEncoders();

    /**
    * Returns the heading of the robot.
    *
    * @return the robot's heading in degrees, from -180 to 180
    */
    units::degree_t GetHeading() const;

    /**
    * Returns the turn rate of the robot.
    *
    * @return The turn rate of the robot, in degrees per second
    */
    units::angular_velocity::degrees_per_second_t GetTurnRate();

    /**
    * Returns the currently-estimated pose of the robot.
    *
    * @return The pose.
    */
    frc::Pose2d GetPose();

    /**
    * Returns the current wheel speeds of the robot.
    *
    * @return The current wheel speeds.
    */
    frc::DifferentialDriveWheelSpeeds GetWheelSpeeds();

    /**
    * Resets the odometry to the specified pose.
    *
    * @param pose The pose to which to set the odometry.
    */
    void ResetOdometry(frc::Pose2d pose);

    /**
    * Will be called periodically whenever the CommandScheduler runs.
    */
    void Periodic() override;

    /**
    * Will be called periodically during simulation.
    */
    void SimulationPeriodic() override;

 private:
   WPI_TalonFX m_leftMaster{DriveConstants::kLeftMasterID};
   WPI_TalonFX m_leftFollower1{DriveConstants::kLeftFollower1ID};
   WPI_TalonFX m_leftFollower2{DriveConstants::kLeftFollower2ID};
   WPI_TalonFX m_rightMaster{DriveConstants::kRightMasterID};
   WPI_TalonFX m_rightFollower1{DriveConstants::kRightFollower1ID};
   WPI_TalonFX m_rightFollower2{DriveConstants::kRightFollower2ID};

   TalonFXSimCollection m_leftMasterSim{m_leftMaster};
	TalonFXSimCollection m_rightMasterSim{m_rightMaster};

   frc::DifferentialDrive m_drive{m_leftMaster, m_rightMaster};

   frc::ADIS16470_IMU m_IMU;

   frc::sim::ADIS16470_IMUSim m_IMUSim{m_IMU};

   frc::DifferentialDriveOdometry m_odometry{
      m_IMU.GetAngle(), m_leftMaster.GetSelectedSensorPosition() * DriveConstants::kWheelEncoderMetersPerUnit, m_rightMaster.GetSelectedSensorPosition() * DriveConstants::kWheelEncoderMetersPerUnit
   };

   frc::sim::DifferentialDrivetrainSim m_drivetrainSimulator{ // come back to drivetrain plant later
      frc::DCMotor::Falcon500(3),
      DriveConstants::kDrivetrainGearing,
      DriveConstants::kMOI,
      DriveConstants::kMass,
      DriveConstants::kWheelDiameter / 2,
      DriveConstants::kTrackwidth,
      {0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005}};

   frc::Field2d m_fieldSim;
};
