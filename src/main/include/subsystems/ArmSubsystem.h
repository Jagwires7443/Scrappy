// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/DoubleSolenoid.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"
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

  bool Status() noexcept
  {
    return status_;
  }

  bool InPosition() noexcept;

  units::angle::degree_t GetShoulderAngle() noexcept
  {
    return shoulderAngle_;
  }

  units::angle::degree_t GetElbowAngle() noexcept
  {
    return elbowAngle_;
  }

  units::length::meter_t GetElbowX() noexcept
  {
    return elbowX_;
  }

  units::length::meter_t GetElbowY() noexcept
  {
    return elbowY_;
  }

  units::length::meter_t GetGripperX() noexcept
  {
    return gripperX_;
  }

  units::length::meter_t GetGripperY() noexcept
  {
    return gripperY_;
  }

  units::length::meter_t GetDottedLength() noexcept
  {
    return dottedLength_;
  }

  units::angle::degree_t GetDottedAngle() noexcept
  {
    return dottedAngle_;
  }

#ifdef RUNNING_FRC_TESTS
  void TestPrint() noexcept
  {
    print_ = true;
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

  void SetShoulderAngle(units::angle::degree_t angle) noexcept;

  void SetElbowAngle(units::angle::degree_t angle) noexcept;

  void SetAngles(units::angle::degree_t shoulderAngle, units::angle::degree_t elbowAngle) noexcept
  {
    SetShoulderAngle(shoulderAngle);
    SetElbowAngle(elbowAngle);
  }

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

  units::angle::degree_t commandedShoulderAngle_{arm::shoulderPositiveStopLimit};
  units::angle::degree_t commandedElbowAngle_{arm::elbowNegativeStopLimit};

  double shoulderControlUI_{0.0};
  bool shoulderResetUI_{false};
  double elbowControlUI_{0.0};
  bool elbowResetUI_{false};

  frc::ComplexWidget *shoulderPIDUI_{nullptr};
  frc::ComplexWidget *elbowPIDUI_{nullptr};

  units::angle::degree_t shoulderAngle_{0.0_deg};
  units::angle::degree_t elbowAngle_{0.0_deg};

  units::length::meter_t elbowX_{0.0_m};
  units::length::meter_t elbowY_{0.0_m};

  units::length::meter_t gripperX_{0.0_m};
  units::length::meter_t gripperY_{0.0_m};

  units::length::meter_t dottedLength_{0.0_m};
  units::angle::degree_t dottedAngle_{0.0_deg};

  bool status_{false};
  bool print_{false};
  bool test_{false};
};
