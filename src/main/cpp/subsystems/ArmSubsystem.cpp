// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ArmSubsystem.h"

#include "Constants.h"

ArmSubsystem::ArmSubsystem() noexcept
{
  shoulderSensor_ = std::make_unique<AngleSensor>(nonDrive::kShoulderEncoderPort, nonDrive::kShoulderAlignmentOffset);
  elbowSensor_ = std::make_unique<AngleSensor>(nonDrive::kElbowEncoderPort, nonDrive::kElbowAlignmentOffset);
  shoulderMotor_ = SparkMaxFactory::CreateSparkMax("Shoulder", nonDrive::kShoulderMotorCanID, nonDrive::kShoulderMotorInverted);
  elbowMotor_ = SparkMaxFactory::CreateSparkMax("Elbow", nonDrive::kElbowMotorCanID, nonDrive::kElbowMotorInverted);
}

void ArmSubsystem::Periodic() noexcept
{
  shoulderSensor_->Periodic();
  elbowSensor_->Periodic();
  shoulderMotor_->Periodic();
  elbowMotor_->Periodic();
}

void ArmSubsystem::TestInit() noexcept {}
void ArmSubsystem::TestExit() noexcept {}
void ArmSubsystem::TestPeriodic() noexcept { Periodic(); }
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
