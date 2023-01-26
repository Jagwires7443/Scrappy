// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include "infrastructure/PWMAngleSensor.h"
#include "infrastructure/SparkMax.h"

#include <memory>

class ArmSubsystem : public frc2::SubsystemBase
{
public:
  ArmSubsystem() noexcept;

  /**
   * Example command factory method.
   */
  frc2::CommandPtr ArmMethodCommand() noexcept;

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  bool ArmCondition() noexcept;

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() noexcept override;

private:
  std::unique_ptr<AngleSensor> shoulderSensor_;
  std::unique_ptr<AngleSensor> elbowSensor_;
  std::unique_ptr<SmartMotorBase> shoulderMotor_;
  std::unique_ptr<SmartMotorBase> elbowMotor_;
};
