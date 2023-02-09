// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ArmSubsystem.h"

ArmSubsystem::ArmSubsystem() noexcept
{
  shoulderSensor_ = std::make_unique<AngleSensor>(0, 0);
  elbowSensor_ = std::make_unique<AngleSensor>(1, 0);
  shoulderMotor_ = SparkMaxFactory::CreateSparkMax("Shoulder", 1, false);
  elbowMotor_ = SparkMaxFactory::CreateSparkMax("Elbow", 2, false);
}

frc2::CommandPtr ArmSubsystem::ArmMethodCommand() noexcept
{
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return RunOnce([/* this */] { /* one-time action goes here */ });
}

bool ArmSubsystem::ArmCondition() noexcept
{
  // Query some boolean state, such as a digital sensor.
  return false;
}

void ArmSubsystem::Periodic() noexcept
{
  // Implementation of subsystem periodic method goes here.
}
