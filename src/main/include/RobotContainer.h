// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/GenericHID.h>
#include <frc/XboxController.h>
#include <frc2/command/CommandPtr.h>

#include "subsystems/ArmSubsystem.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/InfrastructureSubsystem.h"

#include <memory>
#include <tuple>

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer
{
public:
  RobotContainer() noexcept;

  RobotContainer(const RobotContainer &) = delete;
  RobotContainer &operator=(const RobotContainer &) = delete;

  std::optional<frc2::CommandPtr> GetAutonomousCommand() noexcept;

  void TestInit() noexcept;
  void TestExit() noexcept;
  void TestPeriodic() noexcept;

  void DisabledInit() noexcept;
  void DisabledExit() noexcept;
  void AutonomousInit() noexcept;
  void TeleopInit() noexcept;

private:
  static frc2::CommandPtr DriveCommandFactory(RobotContainer *container) noexcept;
  static frc2::CommandPtr PointCommandFactory(RobotContainer *container) noexcept;

  std::tuple<double, double, double, bool> GetDriveTeleopControls() noexcept;

  void ConfigureBindings() noexcept;

  bool m_fieldOriented{false};
  bool m_lock{false};
  bool m_slow{false};
  double m_shooterVelocity{0.0};
  uint32_t m_LEDPattern{29};
  uint32_t m_LEDPatternCount{0};

  // The robot's subsystems and commands are defined here...
  ArmSubsystem arm_;
  DriveSubsystem m_driveSubsystem;
  InfrastructureSubsystem m_infrastructureSubsystem;

  frc::XboxController m_xbox{0};
  frc::GenericHID m_buttonBoard{1};
};
