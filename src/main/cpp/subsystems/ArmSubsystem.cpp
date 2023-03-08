// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ArmSubsystem.h"

#include "Constants.h"

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardLayout.h>
#include <frc/shuffleboard/ShuffleboardTab.h>

#include <units/angle.h>
#include <units/torque.h>
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
        {"kStatus1", uint32_t{250}}, // ms
        {"kStatus2", uint32_t{250}}, // ms
        {"kPositionConversionFactor", double{360.0}},
        {"kVelocityConversionFactor", double{360.0 / 60.0}},
        {"kIdleMode", uint32_t{1}},
    });
    shoulderMotor_->ApplyConfig(false);

    elbowMotor_->AddConfig(SmartMotorBase::ConfigMap{
        {"kStatus1", uint32_t{250}}, // ms
        {"kStatus2", uint32_t{250}}, // ms
        {"kPositionConversionFactor", double{360.0}},
        {"kVelocityConversionFactor", double{360.0 / 60.0}},
        {"kIdleMode", uint32_t{1}},
    });
    elbowMotor_->ApplyConfig(false);

    pneuGrip_ = std::make_unique<frc::DoubleSolenoid>(frc::PneumaticsModuleType::REVPH, nonDrive::kGripPneuOpen, nonDrive::kGripPneuClose);
    motorGrip_ = std::make_unique<ctre::phoenix::motorcontrol::can::WPI_TalonSRX>(nonDrive::kGripMotorCanID);

    // XXX Need real constants here!!!
    shoulderPIDController_ = std::make_unique<frc::ProfiledPIDController<units::angle::degrees>>(
        arm::kShoulderPositionP,
        0.0,
        0.0,
        std::move(frc::TrapezoidProfile<units::angle::degrees>::Constraints{
            arm::kShoulderPositionMaxVelocity,
            arm::kShoulderPositionMaxAcceleration}));
    shoulderPIDController_->DisableContinuousInput();

    // XXX Need real constants here!!!
    elbowPIDController_ = std::make_unique<frc::ProfiledPIDController<units::angle::degrees>>(
        arm::kElbowPositionP,
        0.0,
        0.0,
        std::move(frc::TrapezoidProfile<units::angle::degrees>::Constraints{
            arm::kElbowPositionMaxVelocity,
            arm::kElbowPositionMaxAcceleration}));
    elbowPIDController_->DisableContinuousInput();
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

        if (print_)
        {
            printf("**** Arm Sensor Fault!\n");
        }

        return;
    }

    shoulderAngle_ = shoulderSensor.value();
    elbowAngle_ = elbowSensor.value();

    double shoulder = shoulderPIDController_->Calculate(shoulderAngle_);
    double elbow = elbowPIDController_->Calculate(elbowAngle_);

    // Geometry:
    // Upper arm has length, and angle (increasing counter-clockwise, with zero
    // horizontal forward); lower arm has length, and angle (increasing
    // counter-clockwise, with zero when elbow is straight); motors must be set
    // up for same direction for angle increase with "forward" power.

    // Work out the third side of the triangle, formed by the upper and lower
    // arms.  Specifically, find the length of this side as well as the angle
    // between this third side and the upper arm.  This is useful in compensating
    // for gravity, as well as checking limits.

    // For this calculation, the elbow angle needs to be adjusted to follow
    // convention.
    units::angle::degree_t realAngle = elbowAngle_ - 180.0_deg;

    if (realAngle < -180.0_deg)
    {
        realAngle += 360.0_deg;
    }

    // Law of cosines: c = sqrt(a^2 + b^2 - 2*a*b*cos(angle_c))
    dottedLength_ = units::length::meter_t{sqrt(
        pow(units::length::meter_t{arm::upperArmLength}.value(), 2.0) +
        pow(units::length::meter_t{arm::lowerArmLength}.value(), 2.0) -
        2.0 * units::length::meter_t{arm::upperArmLength}.value() *
            units::length::meter_t{arm::lowerArmLength}.value() *
            cos(units::angle::radian_t{realAngle}.value()))};

    // Law of cosines: angle_b = arc_cos((c^2 + a^2 - b^2) / (2*c*a))
    double imprecision =
        (pow(units::length::meter_t{dottedLength_}.value(), 2.0) +
         pow(units::length::meter_t{arm::upperArmLength}.value(), 2.0) -
         pow(units::length::meter_t{arm::lowerArmLength}.value(), 2.0)) /
        (2.0 * units::length::meter_t{dottedLength_}.value() *
         units::length::meter_t{arm::upperArmLength}.value());

    // A bit of defensive logic, in case of slight imprecision in calculations.
    if (imprecision > +1.0)
    {
        imprecision = +1.0;
    }
    if (imprecision < -1.0)
    {
        imprecision = -1.0;
    }

    // Now, adjust this angle to account for it being relative to shoulder angle,
    // so that the final angle is in the robot coordinate system, instead of that
    // of the shoulder.
    if (elbowAngle_ > 0.0_deg)
    {
        dottedAngle_ = shoulderAngle_ + units::angle::radian_t{acos(imprecision)};
    }
    else if (elbowAngle_ < 0.0_deg)
    {
        dottedAngle_ = shoulderAngle_ - units::angle::radian_t{acos(imprecision)};
    }
    else
    {
        dottedAngle_ = shoulderAngle_;
    }

    // Find the moment arm with respect to gravity -- the horizontal projection
    // of the third side of the triangle.  This acts at the shoulder.
    units::length::meter_t shoulderMoment = dottedLength_ *
                                            fabs(cos(units::angle::radian_t{dottedAngle_}.value()));
    units::torque::newton_meter_t shoulderTorqueGravity = shoulderMoment * arm::pointMass * arm::gravity;
    double shoulderPercentTorqueGravity = shoulderTorqueGravity / arm::shoulderMaxTorque;

    // Now do these calculations for the elbow.
    units::length::meter_t elbowMoment = arm::lowerArmLength *
                                         fabs(cos(units::angle::radian_t{180.0_deg - shoulderAngle_ - realAngle}.value()));
    units::torque::newton_meter_t elbowTorqueGravity = elbowMoment * arm::pointMass * arm::gravity;
    double elbowPercentTorqueGravity = elbowTorqueGravity / arm::elbowMaxTorque;

    // XXX
    // Feedforward calculations
    // PID Tuning / position seeking
    // XXX
    shoulder = shoulderControlUI_;
    elbow = elbowControlUI_;

    // Apply shoulder feedforward, compensationg for gravity.
    if (shoulder > 0.0)
    {
        shoulder += arm::shoulderStaticFeedforward + shoulderPercentTorqueGravity;
    }
    else if (shoulder < 0.0)
    {
        shoulder -= arm::shoulderStaticFeedforward + shoulderPercentTorqueGravity;
    }

    // Apply elbow feedforward, compensationg for gravity.
    if (elbow > 0.0)
    {
        elbow += arm::elbowStaticFeedforward + elbowPercentTorqueGravity;
    }
    else if (elbow < 0.0)
    {
        elbow -= arm::elbowStaticFeedforward + elbowPercentTorqueGravity;
    }

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

    std::string comments;

    // Shoulder exclusion zone is centered on 90 degrees.
    if (shoulderAngle_ >= arm::shoulderNegativeStopLimit && shoulderAngle_ < 90.0_deg && shoulder < 0.0)
    {
        comments += " Shoulder -STOP";
        shoulder = 0.0;
    }
    else if (shoulderAngle_ >= arm::shoulderNegativeParkLimit && shoulderAngle_ < 90.0_deg && shoulder < -arm::shoulderParkPower)
    {
        comments += " Shoulder -Park";
        shoulder = -arm::shoulderParkPower;
    }
    else if (shoulderAngle_ >= arm::shoulderNegativeSlowLimit && shoulderAngle_ < 90.0_deg && shoulder < -arm::shoulderSlowPower)
    {
        comments += " Shoulder -Slow";
        shoulder = -arm::shoulderSlowPower;
    }

    if (shoulderAngle_ >= 90.0_deg && shoulderAngle_ < arm::shoulderPositiveStopLimit && shoulder > 0.0)
    {
        comments += " Shoulder +STOP";
        shoulder = 0.0;
    }
    else if (shoulderAngle_ >= 90.0_deg && shoulderAngle_ < arm::shoulderPositiveParkLimit && shoulder > arm::shoulderParkPower)
    {
        comments += " Shoulder +Park";
        shoulder = arm::shoulderParkPower;
    }
    else if (shoulderAngle_ >= 90.0_deg && shoulderAngle_ < arm::shoulderPositiveSlowLimit && shoulder > arm::shoulderSlowPower)
    {
        comments += " Shoulder +Slow";
        shoulder = arm::shoulderSlowPower;
    }

    // Elbow exclusion zone is centered on 180 degrees (the natural wrap point)
    // so no need to guard against wrap.  Because the elbow is on a virtual
    // four-bar through the shoulder, stop the shoulder if things are getting
    // tight.
    if (elbowAngle_ >= arm::elbowNegativeStopLimit || elbowAngle_ < arm::elbowPositiveStopLimit)
    {
        comments += " Shoulder STOP";
        shoulder = 0.0;
    }

    if (elbowAngle_ >= arm::elbowNegativeStopLimit && elbow < 0.0)
    {
        comments += " Elbow -STOP";
        elbow = 0.0;
    }
    else if (elbowAngle_ >= arm::elbowNegativeParkLimit && elbow < -arm::elbowParkPower)
    {
        comments += " Elbow -Park";
        elbow = -arm::elbowParkPower;
    }
    else if (elbowAngle_ >= arm::elbowNegativeSlowLimit && elbow < -arm::elbowSlowPower)
    {
        comments += " Elbow -Slow";
        elbow = -arm::elbowSlowPower;
    }

    if (elbowAngle_ < arm::elbowPositiveStopLimit && elbow > 0.0)
    {
        comments += " Elbow +STOP";
        elbow = 0.0;
    }
    else if (elbowAngle_ < arm::elbowPositiveParkLimit && elbow > arm::elbowParkPower)
    {
        comments += " Elbow +Park";
        elbow = arm::elbowParkPower;
    }
    else if (elbowAngle_ < arm::elbowPositiveSlowLimit && elbow > arm::elbowSlowPower)
    {
        comments += " Elbow +Slow";
        elbow = arm::elbowSlowPower;
    }

    if (print_)
    {
        printf("**** Arm Status: SA=%lf EA=%lf DL=%lf DA=%lf SM=%lf EM=%lf SO=%lf EO=%lf%s\n",
               shoulderAngle_.value(),
               elbowAngle_.value(),
               dottedLength_.value(),
               dottedAngle_.value(),
               shoulderMoment.value(),
               elbowMoment.value(),
               shoulder,
               elbow,
               comments.c_str());
    }

    shoulderMotor_->SetVoltage(shoulder * 12.0_V);
    elbowMotor_->SetVoltage(elbow * 12.0_V);
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

void ArmSubsystem::RelaxGrip() noexcept
{
    pneuGrip_->Set(frc::DoubleSolenoid::kOff);
    motorGrip_->Set(0.0);
}
