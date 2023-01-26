// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>

#include "subsystems/ArmSubsystem.h"

class RobotContainer
{
public:
  RobotContainer() noexcept;

  frc2::CommandPtr GetAutonomousCommand() noexcept;

private:
  ArmSubsystem arm_;

  void ConfigureBindings() noexcept;
};
