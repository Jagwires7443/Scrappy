// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ArmSubsystem.h"

#include "Constants.h"

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardLayout.h>
#include <frc/shuffleboard/ShuffleboardTab.h>

#include <units/angle.h>
#include <units/voltage.h>

#include <cmath>

ArmSubsystem::ArmSubsystem() noexcept
{
    shoulderSensor_ = std::make_unique<AngleSensor>(nonDrive::kShoulderEncoderPort, nonDrive::kShoulderAlignmentOffset);
    elbowSensor_ = std::make_unique<AngleSensor>(nonDrive::kElbowEncoderPort, nonDrive::kElbowAlignmentOffset);
    shoulderMotorBase_ = SparkMaxFactory::CreateSparkMax("Shoulder", nonDrive::kShoulderMotorCanID, nonDrive::kShoulderMotorInverted);
    shoulderMotor_ = std::make_unique<SmartMotor<units::angle::degrees>>(*shoulderMotorBase_);
    elbowMotorBase_ = SparkMaxFactory::CreateSparkMax("Elbow", nonDrive::kElbowMotorCanID, nonDrive::kElbowMotorInverted);
    elbowMotor_ = std::make_unique<SmartMotor<units::angle::degrees>>(*elbowMotorBase_);

    shoulderMotor_->AddConfig(SmartMotorBase::ConfigMap{
        {"kStatus1", uint{250}}, // ms
        {"kStatus2", uint{250}}, // ms
        {"kPositionConversionFactor", double{360.0}},
        {"kVelocityConversionFactor", double{360.0 / 60.0}},
        {"kIdleMode", uint{0}}, // XXX  1 = Brake -- switch after shakedown
    });
    shoulderMotor_->ApplyConfig(false);

    elbowMotor_->AddConfig(SmartMotorBase::ConfigMap{
        {"kStatus1", uint{250}}, // ms
        {"kStatus2", uint{250}}, // ms
        {"kPositionConversionFactor", double{360.0}},
        {"kVelocityConversionFactor", double{360.0 / 60.0}},
        {"kIdleMode", uint{0}},
    });
    elbowMotor_->ApplyConfig(false);

    pneuGrip_ = std::make_unique<frc::DoubleSolenoid>(frc::PneumaticsModuleType::REVPH, nonDrive::kGripPneuOpen, nonDrive::kGripPneuClose);
    motorGrip_ = std::make_unique<ctre::phoenix::motorcontrol::can::WPI_TalonSRX>(nonDrive::kGripMotorCanID);

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

    shoulderAngle_ = shoulderSensor.value();
    elbowAngle_ = elbowSensor.value();

    // Geometry:
    // Upper arm has length, and angle (increasing counter-clockwise, with zero
    // horizontal forward); lower arm has length, and angle (increasing
    // counter-clockwise, with zero when elbow is straight); motors must be set
    // up for same direction for angle increase with "forward" power.

    // Work out the third side of the triangle, formed by the upper and lower
    // arms.  Specifically, find the length of this side as well as the angle
    // between this third side and the upper arm.  This is useful in compensating
    // for gravity, as well as checking for limits.

    // Law of cosines: c = sqrt(a^2 + b^2 - 2*a*b*cos(angle_c))
    dottedLength_ = units::length::meter_t{sqrt(
        pow(units::length::meter_t{arm::upperArmLength}.value(), 2.0) +
        pow(units::length::meter_t{arm::lowerArmLength}.value(), 2.0) -
        2.0 * units::length::meter_t{arm::upperArmLength}.value() *
            units::length::meter_t{arm::lowerArmLength}.value() *
            cos(units::angle::radian_t{elbowAngle_}.value()))};

    // Law of cosines: angle_b = arc_cos((c^2 + a^2 - b^2) / 2*c*a)
    dottedAngle_ = units::angle::radian_t{acos(
        (pow(units::length::meter_t{dottedLength_}.value(), 2.0) +
         pow(units::length::meter_t{arm::upperArmLength}.value(), 2.0) -
         pow(units::length::meter_t{arm::lowerArmLength}.value(), 2.0)) /
        2.0 * units::length::meter_t{dottedLength_}.value() *
        units::length::meter_t{arm::upperArmLength}.value())};

    // Find the moment arm with respect to gravity -- the horizontal projection
    // of the third side of the triangle.
    units::length::meter_t moment = dottedLength_ *
                                    sin(units::angle::radian_t{shoulderAngle_ + dottedAngle_}.value());

    // XXX
    // Safety checks!!!
    // XXX

    // Feedforward calculations
    // PID Tuning / position seeking

    // printf("**** Arm Shoulder: %lf; Elbow: %lf\n", shoulderSensor.value().value(), elbowSensor.value().value());

    double shoulder = shoulderControlUI_;
    double elbow = elbowControlUI_;

    if (shoulder > +0.25)
    {
        shoulder = +0.25;
    }
    if (shoulder < -0.25)
    {
        shoulder = -0.25;
    }

    if (elbow > +0.25)
    {
        elbow = +0.25;
    }
    if (elbow < -0.25)
    {
        elbow = -0.25;
    }

    // Shoulder exclusion zone is centered on 90 degrees.
    if (shoulderAngle_ >= arm::shoulderNegativeStopLimit && shoulderAngle_ < 90.0_deg && shoulder < 0.0)
    {
        shoulder = 0.0;
    }

    if (shoulderAngle_ >= 90.0_deg && shoulderAngle_ < arm::shoulderPositiveStopLimit && shoulder > 0.0)
    {
        shoulder = 0.0;
    }

    if (shoulderAngle_ >= arm::shoulderNegativeParkLimit && shoulderAngle_ < 90.0_deg && shoulder < -arm::shoulderParkPower)
    {
        shoulder = -arm::shoulderParkPower;
    }

    if (shoulderAngle_ >= 90.0_deg && shoulderAngle_ < arm::shoulderPositiveParkLimit && shoulder > arm::shoulderParkPower)
    {
        shoulder = arm::shoulderParkPower;
    }

    if (shoulderAngle_ >= arm::shoulderNegativeSlowLimit && shoulderAngle_ < 90.0_deg && shoulder < -arm::shoulderSlowPower)
    {
        shoulder = -arm::shoulderSlowPower;
    }

    if (shoulderAngle_ >= 90.0_deg && shoulderAngle_ < arm::shoulderPositiveSlowLimit && shoulder > arm::shoulderSlowPower)
    {
        shoulder = arm::shoulderSlowPower;
    }

    // Elbow exclusion zone is centered on 180 degrees (the natural wrapping point).

    shoulderMotor_->Set(shoulder);
    elbowMotor_->Set(elbow);
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

frc2::CommandPtr ArmSubsystem::ArmMethodExampleCommandFactory() noexcept
{
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return RunOnce([/* this */] { /* one-time action goes here */ });
}

void ArmSubsystem::OpenGrip() noexcept
{
    pneuGrip_->Set(frc::DoubleSolenoid::kForward);
    motorGrip_->SetVoltage(+0.25 * 12.0_V);
}

void ArmSubsystem::CloseGrip() noexcept
{
    pneuGrip_->Set(frc::DoubleSolenoid::kReverse);
    motorGrip_->SetVoltage(-0.25 * 12.0_V);
}
