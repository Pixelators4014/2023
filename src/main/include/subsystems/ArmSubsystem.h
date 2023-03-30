// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/Encoder.h>
#include <frc/controller/LinearQuadraticRegulator.h>

#include <rev/CANSparkMax.h>

#include "Constants.h"
#include "A.h"
#include "B.h"

class ArmSubsystem : public frc2::SubsystemBase {
 public:
  ArmSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  // frc::Pose3d GetPosition(double theta_1, double theta_2, double theta_3, double theta_4);

  void moveTo(double theta_1, double theta_2, double theta_3, double theta_4);

  void moveTo(double x, double y, double z);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  frc::Encoder m_encoderJ2{0, 1};
  frc::Encoder m_encoderJ3{0, 1};
  frc::Encoder m_encoderJ4{0, 1};

  WPI_TalonFX m_J1{ArmConstants::kJ1ID};
  rev::CANSparkMax m_J2{ArmConstants::kJ2ID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_J3{ArmConstants::kJ3ID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_J4{ArmConstants::kJ4ID, rev::CANSparkMax::MotorType::kBrushless};

  frc::Vectord<4> tau;
};
