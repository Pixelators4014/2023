// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/ADIS16470_IMU.h>
#include <frc/simulation/ADIS16470_IMUSim.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/simulation/DifferentialDrivetrainSim.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/RobotController.h>
#include <frc2/command/button/JoystickButton.h>

#include "Constants.h"

class DrivetrainSubsystem : public frc2::SubsystemBase {
 public:
  DrivetrainSubsystem();

    /**
    * Drives the robot using arcade controls.
    *
    * @param forward the commanded forward movement
    * @param rotation the commanded rotation
    */
    void ArcadeDrive(double forward, double rotation);

    void ArcadeDriveF(double forward);
    
    void ArcadeDriveR(double rotation);
    
    void ResetOdometry();

    /**
    * Resets the drive encoders to a position of 0.
    */
    void ResetEncoders();

   /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
   frc::Pose2d GetPose();

   void SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode NeutralMode);

   /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
   frc::DifferentialDriveWheelSpeeds GetWheelSpeeds();

   /**
   * Controls each side of the drive directly with a voltage.
   *
   * @param left the commanded left output
   * @param right the commanded right output
   */
   void TankDriveVolts(units::volt_t left, units::volt_t right);

   void SetYawAxis(frc::ADIS16470_IMU::IMUAxis yaw_axis);

   units::degree_t GetAngle();

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
      DriveConstants::kTrackWidth,
      {0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005}};

   frc::Field2d m_fieldSim;

   double forward, rotation;
};
