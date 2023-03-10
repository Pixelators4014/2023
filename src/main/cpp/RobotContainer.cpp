// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {
  ConfigureBindings();

  m_drive.SetDefaultCommand(frc2::RunCommand([this] {
    m_drive.ArcadeDrive(m_driverController.GetY(),m_driverController.GetX(), m_driverController.GetRawButton(1));
  },
  {&m_drive}));
}

void RobotContainer::ConfigureBindings() {}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
