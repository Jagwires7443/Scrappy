// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Constants.h"
#include "RobotContainer.h"
#include "commands/TestModeCommands.h"

#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/POVButton.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>

#include <cmath>
#include <cstdio>
#include <functional>
#include <string>

RobotContainer::RobotContainer() noexcept
{
  // Initialize all of your commands and subsystems here
  m_LEDPatternCount = m_infrastructureSubsystem.GetLEDPatternCount();

  // Configure the button bindings
  ConfigureBindings();

  printf("Axis: %i; Button: %i\n", m_buttonBoard.GetAxisCount(), m_buttonBoard.GetButtonCount());
  m_buttonBoard.SetOutputs(0x0fff);
}

frc2::CommandPtr RobotContainer::DriveCommandFactory(RobotContainer *container) noexcept
{
  // Set up default drive command; non-owning pointer is passed by value.
  auto driveRequirements = {dynamic_cast<frc2::Subsystem *>(&container->m_driveSubsystem)};

  // Drive, as commanded by operator joystick controls.
  return frc2::CommandPtr{std::make_unique<frc2::RunCommand>(
      [container]() -> void
      {
        if (container->m_lock)
        {
          (void)container->m_driveSubsystem.SetLockWheelsX();

          return;
        }

        const auto controls = container->GetDriveTeleopControls();

        container->m_driveSubsystem.Drive(
            std::get<0>(controls) * physical::kMaxDriveSpeed,
            std::get<1>(controls) * physical::kMaxDriveSpeed,
            std::get<2>(controls) * physical::kMaxTurnRate,
            std::get<3>(controls));
      },
      driveRequirements)};
}

frc2::CommandPtr RobotContainer::PointCommandFactory(RobotContainer *container) noexcept
{
  // Set up default drive command; non-owning pointer is passed by value.
  auto driveRequirements = {dynamic_cast<frc2::Subsystem *>(&container->m_driveSubsystem)};

  // Point swerve modules, but do not actually drive.
  return frc2::CommandPtr{std::make_unique<frc2::RunCommand>(
      [container]() -> void
      {
        const auto controls = container->GetDriveTeleopControls();

        units::angle::radian_t angle{std::atan2(std::get<0>(controls), std::get<1>(controls))};

        // Ingnore return (bool); no need to check that commanded angle has
        // been reached.
        (void)container->m_driveSubsystem.SetTurningPosition(angle);
      },
      driveRequirements)};
}

frc2::CommandPtr RobotContainer::ArmCommandFactory(RobotContainer *container) noexcept
{
  // Set up default drive command; non-owning pointer is passed by value.
  auto armRequirements = {dynamic_cast<frc2::Subsystem *>(&container->arm_)};

  // Point swerve modules, but do not actually drive.
  return frc2::CommandPtr{std::make_unique<frc2::RunCommand>(
      [container]() -> void
      {
        const auto controls = container->GetDriveTeleopControls();

        container->arm_.IncrementXY(std::get<0>(controls) * 6.0_in, std::get<1>(controls) * 6.0_in);
      },
      armRequirements)};
}

void RobotContainer::AutonomousInit() noexcept
{
  arm_.ClearFaults();
  m_driveSubsystem.ClearFaults();

  m_driveSubsystem.ResetEncoders();

  m_driveSubsystem.SetDefaultCommand(frc2::RunCommand([&]() -> void {},
                                                      {&m_driveSubsystem}));

  m_infrastructureSubsystem.SetDefaultCommand(frc2::RunCommand([&]() -> void {},
                                                               {&m_infrastructureSubsystem}));
  m_infrastructureSubsystem.SetNumberLights(true);
}

void RobotContainer::TeleopInit() noexcept
{
  arm_.ClearFaults();
  m_driveSubsystem.ClearFaults();

  m_driveSubsystem.ResetEncoders();

  m_driveSubsystem.SetDefaultCommand(DriveCommandFactory(this));

  m_infrastructureSubsystem.SetDefaultCommand(frc2::RunCommand([&]() -> void
                                                               { m_infrastructureSubsystem.SetLEDPattern(m_LEDPattern); },
                                                               {&m_infrastructureSubsystem}));
  m_infrastructureSubsystem.SetNumberLights(true);
}

void RobotContainer::ConfigureBindings() noexcept
{
  m_buttonBoard.Button(1).WhileFalse(frc2::InstantCommand([&]() -> void
                                                          { printf("On\n"); m_buttonBoard.SetOutputs(0x0555); },
                                                          {})
                                         .ToPtr());
  m_buttonBoard.Button(1).WhileTrue(frc2::InstantCommand([&]() -> void
                                                         { printf("Off\n"); m_buttonBoard.SetOutputs(0x0aaa); },
                                                         {})
                                        .ToPtr());

#if 0
  m_xbox.A().OnTrue(frc2::InstantCommand([&]() -> void
                                         { m_slow = true; },
                                         {})
                        .ToPtr());
  m_xbox.B().OnTrue(frc2::InstantCommand([&]() -> void
                                         { m_slow = false; },
                                         {})
                        .ToPtr());

  m_xbox.X().OnTrue(frc2::InstantCommand([&]() -> void
                                         { m_fieldOriented = false; },
                                         {})
                        .ToPtr());
  m_xbox.Y().OnTrue(frc2::InstantCommand([&]() -> void
                                         { m_driveSubsystem.ZeroHeading();
                                           m_fieldOriented = true; },
                                         {&m_driveSubsystem})
                        .ToPtr());

  m_buttonBoard.Button(5).OnTrue(frc2::InstantCommand([&]() -> void
                                                      { m_shooterVelocity = -500.0; },
                                                      {})
                                     .ToPtr());

  m_buttonBoard.Button(6).OnTrue(frc2::InstantCommand([&]() -> void
                                                      { m_lock = true; },
                                                      {})
                                     .ToPtr());

  m_buttonBoard.Button(8).OnTrue(frc2::InstantCommand([&]() -> void
                                                      { m_lock = false; },
                                                      {})
                                     .ToPtr());

  m_buttonBoard.Button(10).OnTrue(frc2::InstantCommand([&]() -> void
                                                       { m_shooterVelocity = 1320.0; },
                                                       {})
                                      .ToPtr());

  m_buttonBoard.Button(11).OnTrue(frc2::InstantCommand([&]() -> void
                                                       { m_shooterVelocity = 930.0; },
                                                       {})
                                      .ToPtr());

  m_buttonBoard.Button(12).OnTrue(frc2::InstantCommand([&]() -> void
                                                       { m_shooterVelocity = 400.0; },
                                                       {})
                                      .ToPtr());
  m_buttonBoard.Button(7).OnTrue(frc2::InstantCommand([&]() -> void
                                                      { ++m_LEDPattern;
                                                                      if (m_LEDPattern >= m_LEDPatternCount) { m_LEDPattern = 0; }
                                                                      std::printf("LED Pattern[%u]: %s\n", m_LEDPattern, std::string(m_infrastructureSubsystem.GetLEDPatternDescription(m_LEDPattern)).c_str()); },
                                                      {})
                                     .ToPtr());

  m_xbox.LeftBumper().OnTrue(frc2::InstantCommand([&]() -> void
                                                  { m_infrastructureSubsystem.Disable(); },
                                                  {&m_infrastructureSubsystem})
                                 .ToPtr());

  m_xbox.RightBumper().OnTrue(frc2::InstantCommand([&]() -> void
                                                   { m_infrastructureSubsystem.Enable(); },
                                                   {&m_infrastructureSubsystem})
                                  .ToPtr());
#endif

#if 0
  m_xbox.A().OnTrue(frc2::InstantCommand([&]() -> void
                                         { arm_.SetShoulder(+0.25); },
                                         {&arm_})
                        .ToPtr());
  m_xbox.A().OnFalse(frc2::InstantCommand([&]() -> void
                                          { arm_.SetShoulder(0.0); },
                                          {&arm_})
                         .ToPtr());

  m_xbox.B().OnTrue(frc2::InstantCommand([&]() -> void
                                         { arm_.SetShoulder(-0.25); },
                                         {&arm_})
                        .ToPtr());
  m_xbox.B().OnFalse(frc2::InstantCommand([&]() -> void
                                          { arm_.SetShoulder(0.0); },
                                          {&arm_})
                         .ToPtr());

  m_xbox.X().OnTrue(frc2::InstantCommand([&]() -> void
                                         { arm_.SetElbow(+0.25); },
                                         {&arm_})
                        .ToPtr());
  m_xbox.X().OnFalse(frc2::InstantCommand([&]() -> void
                                          { arm_.SetElbow(0.0); },
                                          {&arm_})
                         .ToPtr());

  m_xbox.Y().OnTrue(frc2::InstantCommand([&]() -> void
                                         { arm_.SetElbow(-0.25); },
                                         {&arm_})
                        .ToPtr());
  m_xbox.Y().OnFalse(frc2::InstantCommand([&]() -> void
                                          { arm_.SetElbow(0.0); },
                                          {&arm_})
                         .ToPtr());
#endif

#if 0
  m_xbox.A().OnTrue(frc2::InstantCommand([&]() -> void
                                         { arm_.SetAngles(arm::shoulderPositiveStopLimit, arm::elbowNegativeStopLimit); },
                                         {&arm_})
                        .ToPtr());
  m_xbox.B().OnTrue(frc2::InstantCommand([&]() -> void
                                         { arm_.SetAngles(+146.3_deg, -153.3_deg); },
                                         {&arm_})
                        .ToPtr());
  m_xbox.X().OnTrue(frc2::InstantCommand([&]() -> void
                                         { arm_.SetAngles(+150.4_deg, -142.8_deg); },
                                         {&arm_})
                        .ToPtr());
  m_xbox.Y().OnTrue(frc2::InstantCommand([&]() -> void
                                         { arm_.SetAngles(-130.0_deg, +20.0_deg); },
                                         {&arm_})
                        .ToPtr());
#endif

  m_xbox.A().OnTrue(frc2::InstantCommand([&]() -> void
                                         { arm_.SetAngles(arm::shoulderPositiveStopLimit, arm::elbowNegativeStopLimit); },
                                         {&arm_})
                        .ToPtr());
  m_xbox.B().OnTrue(frc2::InstantCommand([&]() -> void
                                         { arm_.SetXY(25.0_in, 25.0_in, true); },
                                         {&arm_})
                        .ToPtr());
  m_xbox.X().OnTrue(frc2::InstantCommand([&]() -> void
                                         { arm_.SetXY(25.0_in, 25.0_in, false); },
                                         {&arm_})
                        .ToPtr());
  m_xbox.Y().OnTrue(frc2::InstantCommand([&]() -> void
                                         { arm_.SetXY(-25.0_in, 25.0_in, true); },
                                         {&arm_})
                        .ToPtr());

  m_xbox.Back().WhileTrue(frc2::InstantCommand([&]() -> void
                                               { arm_.OpenGrip(); },
                                               {&arm_})
                              .ToPtr());
  m_xbox.Back().OnFalse(frc2::InstantCommand([&]() -> void
                                             { arm_.RelaxGrip(); },
                                             {&arm_})
                            .ToPtr());
  m_xbox.Start().WhileTrue(frc2::InstantCommand([&]() -> void
                                                { arm_.CloseGrip(); },
                                                {&arm_})
                               .ToPtr());
  m_xbox.Start().OnFalse(frc2::InstantCommand([&]() -> void
                                              { arm_.RelaxGrip(); },
                                              {&arm_})
                             .ToPtr());

  frc2::POVButton(&m_xbox, 90).WhileTrue(frc2::InstantCommand([&]() -> void
                                                              { arm_.IncrementXY(+0.0_in, +6.0_in); },
                                                              {&arm_})
                                             .ToPtr());

  frc2::POVButton(&m_xbox, 270).WhileTrue(frc2::InstantCommand([&]() -> void
                                                               { arm_.IncrementXY(+0.0_in, -6.0_in); },
                                                               {&arm_})
                                              .ToPtr());

  frc2::POVButton(&m_xbox, 0).WhileTrue(frc2::InstantCommand([&]() -> void
                                                             { arm_.IncrementXY(+6.0_in, +0.0_in); },
                                                             {&arm_})
                                            .ToPtr());

  frc2::POVButton(&m_xbox, 180).WhileTrue(frc2::InstantCommand([&]() -> void
                                                               { arm_.IncrementXY(-6.0_in, +0.0_in); },
                                                               {&arm_})
                                              .ToPtr());
}

std::optional<frc2::CommandPtr> RobotContainer::GetAutonomousCommand() noexcept
{
#if 0
  if (m_buttonBoard.GetRawButton(9))
  {
  }
  else
  {
  }
#endif

  return std::nullopt;
}

std::tuple<double, double, double, bool> RobotContainer::GetDriveTeleopControls() noexcept
{
  // The robot's frame of reference is the standard unit circle, from
  // trigonometry.  However, the front of the robot is facing along the positve
  // X axis.  This means the poitive Y axis extends outward from the left (or
  // port) side of the robot.  Poitive rotation is counter-clockwise.  On the
  // other hand, as the controller is held, the Y axis is aligned with forward.
  // And, specifically, it is the negative Y axis which extends forward.  So,
  // the robot's X is the controllers inverted Y.  On the controller, the X
  // axis lines up with the robot's Y axis.  And, the controller's positive X
  // extends to the right.  So, the robot's Y is the controller's inverted X.
  // Finally, the other controller joystick is used for commanding rotation and
  // things work out so that this is also an inverted X axis.
  double x = -m_xbox.GetLeftY();
  double y = -m_xbox.GetLeftX();
  double z = -m_xbox.GetRightX();

  // PlayStation controllers seem to do this strange thing with the rotation:
  // double z = -m_xbox.GetLeftTriggerAxis();
  // Note: there is now a PS4Controller class.

  // Add some deadzone, so the robot doesn't drive when the joysticks are
  // released and return to "zero".  These implement a continuous deadband, one
  // in which the full range of outputs may be generated, once joysticks move
  // outside the deadband.

  // Also, cube the result, to provide more opertor control.  Just cubing the
  // raw value does a pretty good job with the deadband, but doing both is easy
  // and guarantees no movement in the deadband.  Cubing makes it easier to
  // command smaller/slower movements, while still being able to command full
  // power.  The 'mixer` parameter is used to shape the `raw` input, some mix
  // between out = in^3.0 and out = in.
  auto shape = [](double raw, double mixer = 0.75) -> double
  {
    // Input deadband around 0.0 (+/- range).
    constexpr double range = 0.05;

    constexpr double slope = 1.0 / (1.0 - range);

    if (raw >= -range && raw <= +range)
    {
      raw = 0.0;
    }
    else if (raw < -range)
    {
      raw += range;
      raw *= slope;
    }
    else if (raw > +range)
    {
      raw -= range;
      raw *= slope;
    }

    return mixer * std::pow(raw, 3.0) + (1.0 - mixer) * raw;
  };

  x = shape(x);
  y = shape(y);
  z = shape(z, 0.0);

  if (m_slow)
  {
    x *= 0.50;
    y *= 0.50;
    z *= 0.40;
  }
  else
  {
    // x *= 1.0;
    // y *= 1.0;
    z *= 0.8;
  }

  return std::make_tuple(x, y, z, m_fieldOriented);
}

void RobotContainer::TestInit() noexcept
{
  frc2::CommandScheduler::GetInstance().CancelAll();

  arm_.ClearFaults();
  m_driveSubsystem.ClearFaults();

  m_driveSubsystem.ResetEncoders();

  m_driveSubsystem.TestInit();
  arm_.TestInit();

  frc::SendableChooser<std::function<frc2::CommandPtr()>> *chooser{m_driveSubsystem.TestModeChooser()};

  chooser->SetDefaultOption("Zero", std::bind(ZeroCommand::ZeroCommandFactory, &m_driveSubsystem));
  chooser->AddOption("Turning Max", std::bind(MaxVAndATurningCommand::MaxVAndATurningCommandFactory, &m_driveSubsystem));
  chooser->AddOption("Drive Max", std::bind(MaxVAndADriveCommand::MaxVAndADriveCommandFactory, &m_driveSubsystem));
  chooser->AddOption("Xs and Os", std::bind(XsAndOsCommand::XsAndOsCommandFactory, &m_driveSubsystem));
  chooser->AddOption("RotateModules", std::bind(RotateModulesCommand::RotateModulesCommandFactory, &m_driveSubsystem));
  chooser->AddOption("Point", std::bind(PointCommandFactory, this));
  chooser->AddOption("Square", std::bind(SquareCommand::SquareCommandFactory, &m_driveSubsystem));
  chooser->AddOption("Spirograph", std::bind(SpirographCommand::SpirographCommandFactory, &m_driveSubsystem));
  chooser->AddOption("Orbit", std::bind(OrbitCommand::OrbitCommandFactory, &m_driveSubsystem));
  chooser->AddOption("Pirouette", std::bind(PirouetteCommand::PirouetteCommandFactory, &m_driveSubsystem));
  chooser->AddOption("Drive", std::bind(DriveCommandFactory, this));
  chooser->AddOption("Spin", std::bind(SpinCommand::SpinCommandFactory, &m_driveSubsystem));

  frc2::CommandScheduler::GetInstance().Enable();

  m_infrastructureSubsystem.SetNumberLights(true);
}

void RobotContainer::TestExit() noexcept
{
  frc2::CommandScheduler::GetInstance().CancelAll();

  arm_.ClearFaults();
  m_driveSubsystem.ClearFaults();

  m_driveSubsystem.ResetEncoders();

  // arm_.BurnConfig();
  // m_driveSubsystem.BurnConfig();

  m_driveSubsystem.TestExit();
  arm_.TestExit();
}

void RobotContainer::TestPeriodic() noexcept
{
  m_driveSubsystem.TestPeriodic();
  arm_.TestPeriodic();
}

void RobotContainer::DisabledInit() noexcept
{
  frc2::CommandScheduler::GetInstance().CancelAll();

  arm_.ClearFaults();
  m_driveSubsystem.ClearFaults();

  m_driveSubsystem.ResetEncoders();

  // Useful things may be done disabled... (construct, config, dashboard, etc.)
  frc2::CommandScheduler::GetInstance().Enable();

  m_driveSubsystem.DisabledInit();
  arm_.DisabledInit();

  m_infrastructureSubsystem.SetNumberLights(false);
}

void RobotContainer::DisabledExit() noexcept
{
  arm_.ClearFaults();
  m_driveSubsystem.ClearFaults();

  m_driveSubsystem.ResetEncoders();

  m_driveSubsystem.DisabledExit();
  arm_.DisabledExit();
}
