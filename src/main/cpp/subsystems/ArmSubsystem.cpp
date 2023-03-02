// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ArmSubsystem.h"

#include "Constants.h"

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardLayout.h>
#include <frc/shuffleboard/ShuffleboardTab.h>

#include <units/angle.h>

ArmSubsystem::ArmSubsystem() noexcept
{
  shoulderSensor_ = std::make_unique<AngleSensor>(nonDrive::kShoulderEncoderPort, nonDrive::kShoulderAlignmentOffset);
  elbowSensor_ = std::make_unique<AngleSensor>(nonDrive::kElbowEncoderPort, nonDrive::kElbowAlignmentOffset);
  shoulderMotorBase_ = SparkMaxFactory::CreateSparkMax("Shoulder", nonDrive::kShoulderMotorCanID, nonDrive::kShoulderMotorInverted);
  shoulderMotor_ = std::make_unique<SmartMotor<units::angle::degrees>>(*shoulderMotorBase_);
  elbowMotorBase_ = SparkMaxFactory::CreateSparkMax("Elbow", nonDrive::kElbowMotorCanID, nonDrive::kElbowMotorInverted);
  elbowMotor_ = std::make_unique<SmartMotor<units::angle::degrees>>(*elbowMotorBase_);

  // XXX Need real constants here!!!
  shoulderPIDController_ = std::make_unique<frc::ProfiledPIDController<units::angle::degrees>>(
      pidf::kTurningPositionP,
      pidf::kTurningPositionI,
      pidf::kTurningPositionD,
      std::move(frc::TrapezoidProfile<units::angle::degrees>::Constraints{
          pidf::kTurningPositionMaxVelocity,
          pidf::kTurningPositionMaxAcceleration}));

  // XXX Need real constants here!!!
  elbowPIDController_ = std::make_unique<frc::ProfiledPIDController<units::angle::degrees>>(
      pidf::kTurningPositionP,
      pidf::kTurningPositionI,
      pidf::kTurningPositionD,
      std::move(frc::TrapezoidProfile<units::angle::degrees>::Constraints{
          pidf::kTurningPositionMaxVelocity,
          pidf::kTurningPositionMaxAcceleration}));
}

void ArmSubsystem::Periodic() noexcept
{
  shoulderSensor_->Periodic();
  elbowSensor_->Periodic();
  shoulderMotor_->Periodic();
  elbowMotor_->Periodic();

  const auto shoulderSensor = shoulderSensor_->GetAbsolutePosition();
  const auto elbowSensor = elbowSensor_->GetAbsolutePosition();

  // If there is no valid reading from either sensor, stop everything and bail!
  if (!shoulderSensor || !elbowSensor)
  {
    shoulderMotor_->Stop();
    elbowMotor_->Stop();

    return;
  }

  printf("**** Arm Shoulder: %lf; Elbow: %lf\n", shoulderSensor.value().value(), elbowSensor.value().value());
}

void ArmSubsystem::TestInit() noexcept
{
  frc::ShuffleboardTab &shuffleboardShoulderTab = frc::Shuffleboard::GetTab("Shoulder");
  frc::ShuffleboardTab &shuffleboardElbowTab = frc::Shuffleboard::GetTab("Elbow");

  frc::ShuffleboardLayout &shuffleboardLayoutShoulderSensor =
      shuffleboardShoulderTab.GetLayout("Sensor",
                                        frc::BuiltInLayouts::kGrid)
          .WithPosition(0, 0)
          .WithSize(8, 13)
          .WithProperties(wpi::StringMap<nt::Value>{
              std::make_pair("Number of columns", nt::Value::MakeDouble(3.0)),
              std::make_pair("Number of rows", nt::Value::MakeDouble(3.0))});

  frc::ShuffleboardLayout &shuffleboardLayoutElbowSensor =
      shuffleboardElbowTab.GetLayout("Sensor",
                                     frc::BuiltInLayouts::kGrid)
          .WithPosition(0, 0)
          .WithSize(8, 13)
          .WithProperties(wpi::StringMap<nt::Value>{
              std::make_pair("Number of columns", nt::Value::MakeDouble(3.0)),
              std::make_pair("Number of rows", nt::Value::MakeDouble(3.0))});

  frc::ShuffleboardLayout &shuffleboardLayoutShoulderMotor =
      shuffleboardShoulderTab.GetLayout("Motor",
                                        frc::BuiltInLayouts::kGrid)
          .WithPosition(8, 0)
          .WithSize(20, 6)
          .WithProperties(wpi::StringMap<nt::Value>{
              std::make_pair("Number of columns", nt::Value::MakeDouble(6.0)),
              std::make_pair("Number of rows", nt::Value::MakeDouble(2.0))});

  frc::ShuffleboardLayout &shuffleboardLayoutElbowMotor =
      shuffleboardElbowTab.GetLayout("Motor",
                                     frc::BuiltInLayouts::kGrid)
          .WithPosition(8, 0)
          .WithSize(20, 6)
          .WithProperties(wpi::StringMap<nt::Value>{
              std::make_pair("Number of columns", nt::Value::MakeDouble(6.0)),
              std::make_pair("Number of rows", nt::Value::MakeDouble(2.0))});

  frc::ShuffleboardLayout &shuffleboardLayoutShoulderPIDSettings =
      shuffleboardShoulderTab.GetLayout("PID Settings",
                                        frc::BuiltInLayouts::kGrid)
          .WithPosition(8, 6)
          .WithSize(19, 7)
          .WithProperties(wpi::StringMap<nt::Value>{
              std::make_pair("Number of columns", nt::Value::MakeDouble(3.0)),
              std::make_pair("Number of rows", nt::Value::MakeDouble(1.0))});

  frc::ShuffleboardLayout &shuffleboardLayoutElbowPIDSettings =
      shuffleboardElbowTab.GetLayout("PID Settings",
                                     frc::BuiltInLayouts::kGrid)
          .WithPosition(8, 6)
          .WithSize(19, 7)
          .WithProperties(wpi::StringMap<nt::Value>{
              std::make_pair("Number of columns", nt::Value::MakeDouble(3.0)),
              std::make_pair("Number of rows", nt::Value::MakeDouble(1.0))});

  shoulderSensor_->ShuffleboardCreate(
      shuffleboardLayoutShoulderSensor,
      [&]() -> std::pair<units::angle::degree_t, units::angle::degree_t>
      { return std::make_pair(commandedShoulderAngle_, shoulderMotor_->GetPosition()); });

  elbowSensor_->ShuffleboardCreate(
      shuffleboardLayoutElbowSensor,
      [&]() -> std::pair<units::angle::degree_t, units::angle::degree_t>
      { return std::make_pair(commandedElbowAngle_, elbowMotor_->GetPosition()); });

  shoulderMotor_->ShuffleboardCreate(
      shuffleboardLayoutShoulderMotor,
      [&](double control) -> void
      { shoulderControlUI_ = control; },
      [&]() -> void
      { shoulderControlUI_ = 0.0; shoulderResetUI_ = true; });

  elbowMotor_->ShuffleboardCreate(
      shuffleboardLayoutElbowMotor,
      [&](double control) -> void
      { elbowControlUI_ = control; },
      [&]() -> void
      { elbowControlUI_ = 0.0; elbowResetUI_ = true; });

  shoulderPIDControllerUI_ = std::make_unique<TuningPID>(pidf::kTurningPositionP,
                                                         pidf::kTurningPositionI,
                                                         pidf::kTurningPositionD,
                                                         pidf::kTurningPositionF);

  elbowPIDControllerUI_ = std::make_unique<TuningPID>(pidf::kTurningPositionP,
                                                      pidf::kTurningPositionI,
                                                      pidf::kTurningPositionD,
                                                      pidf::kTurningPositionF);

  shoulderPIDUI_ = &shuffleboardLayoutShoulderPIDSettings.Add("PID", *shoulderPIDControllerUI_)
                        .WithPosition(0, 0)
                        .WithWidget(frc::BuiltInWidgets::kPIDController);

  elbowPIDUI_ = &shuffleboardLayoutElbowPIDSettings.Add("PID", *elbowPIDControllerUI_)
                     .WithPosition(0, 0)
                     .WithWidget(frc::BuiltInWidgets::kPIDController);
}

void ArmSubsystem::TestExit() noexcept {}

void ArmSubsystem::TestPeriodic() noexcept
{
  Periodic();
}

void ArmSubsystem::DisabledInit() noexcept {}

void ArmSubsystem::DisabledExit() noexcept {}

void ArmSubsystem::SetShoulder(double percent) noexcept { shoulderMotor_->Set(percent); }
void ArmSubsystem::SetElbow(double percent) noexcept { elbowMotor_->Set(percent); }

frc2::CommandPtr ArmSubsystem::ArmMethodExampleCommandFactory() noexcept
{
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return RunOnce([/* this */] { /* one-time action goes here */ });
}
