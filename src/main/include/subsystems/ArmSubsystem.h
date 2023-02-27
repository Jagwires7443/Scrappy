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

  void Periodic() noexcept override;

  void TestInit() noexcept;
  void TestExit() noexcept;
  void TestPeriodic() noexcept;
  void DisabledInit() noexcept;
  void DisabledExit() noexcept;

  void SetShoulder(double percent) noexcept;
  void SetElbow(double percent) noexcept;

  frc2::CommandPtr ArmMethodExampleCommandFactory() noexcept;

private:
  std::unique_ptr<AngleSensor> shoulderSensor_;
  std::unique_ptr<AngleSensor> elbowSensor_;
  std::unique_ptr<SmartMotorBase> shoulderMotor_;
  std::unique_ptr<SmartMotorBase> elbowMotor_;
};
