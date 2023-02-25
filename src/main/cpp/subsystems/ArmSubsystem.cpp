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
