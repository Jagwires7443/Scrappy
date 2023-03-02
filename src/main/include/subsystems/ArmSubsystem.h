// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/controller/ProfiledPIDController.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include "infrastructure/PWMAngleSensor.h"
#include "infrastructure/ShuffleboardWidgets.h"
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

  void SetShoulder(double percent) noexcept; // XXX
  void SetElbow(double percent) noexcept;    // XXX

  frc2::CommandPtr ArmMethodExampleCommandFactory() noexcept;

private:
  std::unique_ptr<AngleSensor> shoulderSensor_;
  std::unique_ptr<AngleSensor> elbowSensor_;
  std::unique_ptr<SmartMotorBase> shoulderMotorBase_;
  std::unique_ptr<SmartMotorBase> elbowMotorBase_;
  std::unique_ptr<SmartMotor<units::angle::degrees>> shoulderMotor_;
  std::unique_ptr<SmartMotor<units::angle::degrees>> elbowMotor_;

  std::unique_ptr<frc::ProfiledPIDController<units::angle::degrees>> shoulderPIDController_;
  std::unique_ptr<frc::ProfiledPIDController<units::angle::degrees>> elbowPIDController_;

  std::unique_ptr<TuningPID> shoulderPIDControllerUI_;
  std::unique_ptr<TuningPID> elbowPIDControllerUI_;

  units::angle::degree_t commandedShoulderAngle_{0};
  units::angle::degree_t commandedElbowAngle_{0};

  double shoulderControlUI_{0.0};
  bool shoulderResetUI_{false};
  double elbowControlUI_{0.0};
  bool elbowResetUI_{false};

  frc::ComplexWidget *shoulderPIDUI_{nullptr};
  frc::ComplexWidget *elbowPIDUI_{nullptr};
};
