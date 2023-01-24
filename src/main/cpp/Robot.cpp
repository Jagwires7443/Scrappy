// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>

void Robot::RobotInit() noexcept {}

void Robot::RobotPeriodic() noexcept
{
  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() noexcept {}

void Robot::DisabledPeriodic() noexcept {}

void Robot::DisabledExit() noexcept {}

void Robot::AutonomousInit() noexcept
{
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand)
  {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() noexcept {}

void Robot::AutonomousExit() noexcept {}

void Robot::TeleopInit() noexcept
{
  if (m_autonomousCommand)
  {
    m_autonomousCommand->Cancel();
  }
}

void Robot::TeleopPeriodic() noexcept {}

void Robot::TeleopExit() noexcept {}

void Robot::TestInit() noexcept
{
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() noexcept {}

void Robot::TestExit() noexcept {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
