// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Joystick.h>
#include <frc2/command/Commands.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/controller/PIDController.h>
#include <frc2/command/RamseteCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc/PneumaticHub.h>
#include <math.h>


#include "Constants.h"
#include "subsystems/DrivetrainSubsystem.h"
#include "subsystems/GrabberSubsystem.h"

class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

 private:

  frc::Joystick m_driverController{OIConstants::driverControllerPort};
  frc::Joystick m_operatorController{OIConstants::operatorControllerPort};
  DrivetrainSubsystem m_drive;
  // ArmSubsystem m_arm;
  GrabberSubsystem m_grabber;
  frc::PneumaticHub m_ph{Pnumatics::kPHID};
  void ConfigureBindings();
};
