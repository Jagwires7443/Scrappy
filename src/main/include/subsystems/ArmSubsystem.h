// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/DoubleSolenoid.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include "infrastructure/PWMAngleSensor.h"
#include "infrastructure/ShuffleboardWidgets.h"
#include "infrastructure/SparkMax.h"

#include <units/angle.h>
#include <units/length.h>

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

  units::angle::degree_t GetShoulderAngle() noexcept
  {
    return shoulderAngle_;
  }

  units::angle::degree_t GetElbowAngle() noexcept
  {
    return elbowAngle_;
  }

#ifdef RUNNING_FRC_TESTS
  units::length::meter_t TestGetDottedLength() noexcept
  {
    return dottedLength_;
  }

  units::angle::degree_t TestGetDottedAngle() noexcept
  {
    return dottedAngle_;
  }
#endif

  void SetShoulder(double percent) noexcept
  {
    shoulderControlUI_ = percent;
  } // XXX

  void SetElbow(double percent) noexcept
  {
    elbowControlUI_ = percent;
  } // XXX

  void OpenGrip() noexcept;
  void CloseGrip() noexcept;
  void RelaxGrip() noexcept;

  frc2::CommandPtr ArmMethodExampleCommandFactory() noexcept;

private:
  std::unique_ptr<AngleSensor> shoulderSensor_;
  std::unique_ptr<AngleSensor> elbowSensor_;
  std::unique_ptr<SmartMotorBase> shoulderMotorBase_;
  std::unique_ptr<SmartMotorBase> elbowMotorBase_;
  std::unique_ptr<SmartMotor<units::angle::degrees>> shoulderMotor_;
  std::unique_ptr<SmartMotor<units::angle::degrees>> elbowMotor_;
  std::unique_ptr<frc::DoubleSolenoid> pneuGrip_;
  std::unique_ptr<ctre::phoenix::motorcontrol::can::WPI_TalonSRX> motorGrip_;

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

  units::angle::degree_t shoulderAngle_{0.0_deg};
  units::angle::degree_t elbowAngle_{0.0_deg};

  units::length::meter_t dottedLength_{0.0_m};
  units::angle::degree_t dottedAngle_{0.0_deg};

  uint32_t iteration_{0};
};
