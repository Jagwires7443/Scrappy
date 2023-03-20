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
        {"kIdleMode", uint32_t{1}}, // Brake mode
    });
    shoulderMotor_->ApplyConfig(false);

    elbowMotor_->AddConfig(SmartMotorBase::ConfigMap{
        {"kStatus1", uint32_t{250}}, // ms
        {"kStatus2", uint32_t{250}}, // ms
        {"kPositionConversionFactor", double{360.0}},
        {"kVelocityConversionFactor", double{360.0 / 60.0}},
        {"kIdleMode", uint32_t{1}}, // Brake mode
    });
    elbowMotor_->ApplyConfig(false);

    pneuGrip_ = std::make_unique<frc::DoubleSolenoid>(frc::PneumaticsModuleType::REVPH, nonDrive::kGripPneuOpen, nonDrive::kGripPneuClose);
    motorGrip_ = std::make_unique<ctre::phoenix::motorcontrol::can::WPI_TalonSRX>(nonDrive::kGripMotorCanID);

    // Note that these controllers will avoid wrapping at [-180.0_deg, +180.0_deg).
    // For the shoulder, must avoid -90.0_deg; for the elbow, must avoid 0.0_deg.
    // Since gravity is handled via feedforward, it is possible to rotate the no-wrap
    // point -- simply by compensating in both SetGoal() and Calculate().
    // For the shoulder, rotate by -90.0_deg.  For the elbow, rotate by +180.0_deg.

    shoulderPIDController_ = std::make_unique<frc::ProfiledPIDController<units::angle::degrees>>(
        arm::kShoulderPositionP,
        0.0,
        0.0,
        std::move(frc::TrapezoidProfile<units::angle::degrees>::Constraints{
            arm::kShoulderPositionMaxVelocity,
            arm::kShoulderPositionMaxAcceleration}));
    shoulderPIDController_->DisableContinuousInput();
    shoulderPIDController_->SetTolerance(arm::kShoulderTolerance);

    elbowPIDController_ = std::make_unique<frc::ProfiledPIDController<units::angle::degrees>>(
        arm::kElbowPositionP,
        0.0,
        0.0,
        std::move(frc::TrapezoidProfile<units::angle::degrees>::Constraints{
            arm::kElbowPositionMaxVelocity,
            arm::kElbowPositionMaxAcceleration}));
    elbowPIDController_->DisableContinuousInput();
    elbowPIDController_->SetTolerance(arm::kElbowTolerance);

    // Ensure arm initially seeks parked position (otherwise, it would seek
    // zero shoulder, zero elbow).
    SetShoulderAngle(commandedShoulderAngle_);
    SetElbowAngle(commandedElbowAngle_);
}

void ArmSubsystem::SetShoulderAngle(units::angle::degree_t angle) noexcept
{
    units::angle::degree_t rotatedAngle = angle - 90.0_deg;

    while (rotatedAngle >= +180.0_deg)
    {
        rotatedAngle -= 360.0_deg;
    }
    while (rotatedAngle < -180.0_deg)
    {
        rotatedAngle += 360.0_deg;
    }

    commandedShoulderAngle_ = angle;
    shoulderPIDController_->SetGoal(rotatedAngle);
}

void ArmSubsystem::SetElbowAngle(units::angle::degree_t angle) noexcept
{
    units::angle::degree_t rotatedAngle = angle + 180.0_deg;

    while (rotatedAngle >= +180.0_deg)
    {
        rotatedAngle -= 360.0_deg;
    }
    while (rotatedAngle < -180.0_deg)
    {
        rotatedAngle += 360.0_deg;
    }

    commandedElbowAngle_ = angle;
    elbowPIDController_->SetGoal(rotatedAngle);
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

        status_ = false;

        return;
    }

    shoulderAngle_ = shoulderSensor.value();
    elbowAngle_ = elbowSensor.value();

    status_ = true;

    // Geometry:

    // Because of the direction of rotation for the sensors, this is all with the
    // perspective of viewing the robot from the robot's left side, so the front of
    // the robot is at the viewer's left.  The motor inversion settings are for the
    // same direction of rotation as the sensors: counter-clockwise is positive, as
    // in the unit circle of trigonometry.  Following this convention, zero degrees
    // for the shoulder is with the arm horizontal, but to the *rear* of the robot.

    // In the coordinate system and frame of reference of the arm, the origin is at
    // the shoulder joint.  The upper arm has a fixed length, so the elbow folows a
    // circle of this radius.  The range for the shoulder angle is [-180.0_deg,
    // +180.0_deg) -- in other words, this angle is >= -180.0_deg and < +180.0_deg.

    // Because of the structure of the robot, there is an excluded range of angle for
    // the shoulder, centered on -90.0_deg.  This is irrespective of any restrictions
    // on the overall arm.

    // The natural coordinate system for the elbow has it's origin at the elbow joint
    // and moves with the upper arm.  Viewing perspective, direction of rotation, and
    // the range are all the same as for the shoulder.  However, the zero angle
    // reference is fixed to the upper arm.  Specifically, zero angle is when the
    // elbow is folded back onto the upper arm.  Thus, the excluded range for the
    // elbow angle is centered on 0.0_deg.  The lower arm length includes the gripper
    // to facillitate checking for disallowed excursions.

    // The sensor offsets must be adjusted for these coordinate systems, as well as
    // the inversion settings of the motors.  Transforms are performed from these
    // coordinate systems.  Start by getting (X, Y) coordinates of the elbow.

    elbowX_ = arm::upperArmLength * sin(units::angle::radian_t{shoulderAngle_}.value());
    elbowY_ = arm::upperArmLength * cos(units::angle::radian_t{shoulderAngle_}.value());

    // Work out the third side of the triangle formed by the upper and lower arms
    // and transform this into the robot/shoulder coordinate system.  Specifically,
    // find the length of this side, as well as the angle between this third side
    // and the upper arm.  Then, use the angle of the upper arm to transform this
    // angle to the robot/shoulder coordinate system. This is useful in
    // compensating for gravity, as well as checking limits.  Here, the upper arm
    // is "a", the lower arm is "b", and the third side is "c".  The angles are
    // opposite the corresponding side.

    // Law of cosines: c = sqrt(a^2 + b^2 - 2*a*b*cos(angle_c))
    dottedLength_ = units::length::meter_t{sqrt(
        pow(units::length::meter_t{arm::upperArmLength}.value(), 2) +
        pow(units::length::meter_t{arm::lowerArmLength}.value(), 2) -
        2.0 * units::length::meter_t{arm::upperArmLength}.value() *
            units::length::meter_t{arm::lowerArmLength}.value() *
            cos(units::angle::radian_t{elbowAngle_}.value()))};

    // Law of cosines: angle_b = arc_cos((c^2 + a^2 - b^2) / (2*c*a))
    double imprecision =
        (pow(units::length::meter_t{dottedLength_}.value(), 2) +
         pow(units::length::meter_t{arm::upperArmLength}.value(), 2) -
         pow(units::length::meter_t{arm::lowerArmLength}.value(), 2)) /
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
    dottedAngle_ = shoulderAngle_ + units::angle::radian_t{acos(imprecision)};

    gripperX_ = dottedLength_ * sin(units::angle::radian_t{dottedAngle_}.value());
    gripperY_ = dottedLength_ * cos(units::angle::radian_t{dottedAngle_}.value());

    // Obtain base percent power settings, from each ProfiledPIDController.
    units::angle::degree_t rotatedShoulderAngle = shoulderAngle_ - 90.0_deg;
    units::angle::degree_t rotatedElbowAngle = elbowAngle_ + 180.0_deg;

    while (rotatedShoulderAngle >= +180.0_deg)
    {
        rotatedShoulderAngle -= 360.0_deg;
    }
    while (rotatedShoulderAngle < -180.0_deg)
    {
        rotatedShoulderAngle += 360.0_deg;
    }

    while (rotatedElbowAngle >= +180.0_deg)
    {
        rotatedElbowAngle -= 360.0_deg;
    }
    while (rotatedElbowAngle < -180.0_deg)
    {
        rotatedElbowAngle += 360.0_deg;
    }

    double shoulder = shoulderPIDController_->Calculate(rotatedShoulderAngle);
    double elbow = elbowPIDController_->Calculate(rotatedElbowAngle);

    // XXX
    // Find the moment arm with respect to gravity -- the horizontal projection
    // of the third side of the triangle.  This acts at the shoulder.
    units::torque::newton_meter_t shoulderTorqueGravity = gripperX_ * arm::pointMass * arm::gravity;
    double shoulderPercentTorqueGravity = shoulderTorqueGravity / arm::shoulderMaxTorque;

    // Now do these calculations for the elbow.
    units::torque::newton_meter_t elbowTorqueGravity = (gripperX_ - elbowX_) * arm::pointMass * arm::gravity;
    double elbowPercentTorqueGravity = elbowTorqueGravity / arm::elbowMaxTorque;

    // XXX
    // Feedforward calculations
    // PID Tuning / position seeking
    // XXX

    // Apply shoulder feedforward, compensationg for gravity.  Gravity might be
    // adding or subtracting.
#if 0
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
#endif

    // In test mode, override prior calculations and start with UI-supplied values.
    if (test_)
    {
        shoulder = shoulderControlUI_;
        elbow = elbowControlUI_;
    }

    // From here on, apply absolute safety limits to motor power.  This code is
    // intentionally both simplistic and somewhat pedantic.

    // Start by enforcing absolute limits on permitted motor power.
    if (shoulder > +arm::shoulderMaxPower)
    {
        shoulder = +arm::shoulderMaxPower;
    }
    if (shoulder < -arm::shoulderMaxPower)
    {
        shoulder = -arm::shoulderMaxPower;
    }

    if (elbow > +arm::elbowMaxPower)
    {
        elbow = +arm::elbowMaxPower;
    }
    if (elbow < -arm::elbowMaxPower)
    {
        elbow = -arm::elbowMaxPower;
    }

    std::string comments;

    // Excluded range of angle for shoulder, centered on -90.0_deg.  The shoulder
    // is only constrained by these limits; the elbow has further checks.
    if (shoulder < 0.0 && -90.0_deg <= shoulderAngle_ && shoulderAngle_ < arm::shoulderNegativeStopLimit)
    {
        comments += " Shoulder -STOP";
        shoulder = 0.0;
    }
    else if (shoulder < -arm::shoulderParkPower && -90.0_deg <= shoulderAngle_ && shoulderAngle_ < arm::shoulderNegativeParkLimit)
    {
        comments += " Shoulder -Park";
        shoulder = -arm::shoulderParkPower;
    }
    else if (shoulder < -arm::shoulderSlowPower && -90.0_deg <= shoulderAngle_ && shoulderAngle_ < arm::shoulderNegativeSlowLimit)
    {
        comments += " Shoulder -Slow";
        shoulder = -arm::shoulderSlowPower;
    }

    if (shoulder > 0.0 && arm::shoulderPositiveStopLimit <= shoulderAngle_ && shoulderAngle_ < -90.0_deg)
    {
        comments += " Shoulder +STOP";
        shoulder = 0.0;
    }
    else if (shoulder > +arm::shoulderParkPower && arm::shoulderPositiveParkLimit <= shoulderAngle_ && shoulderAngle_ < -90.0_deg)
    {
        comments += " Shoulder +Park";
        shoulder = +arm::shoulderParkPower;
    }
    else if (shoulder > +arm::shoulderSlowPower && arm::shoulderPositiveSlowLimit <= shoulderAngle_ && shoulderAngle_ < -90.0_deg)
    {
        comments += " Shoulder +Slow";
        shoulder = +arm::shoulderSlowPower;
    }

    // Excluded range of angle for elbow, centered on 0.0_deg.  This is the first
    // set of checks for the elbow.  Because the elbow is on a virtual four-bar
    // linkage through the shoulder, the shoulder must also stop or slow if the elbow
    // is in one of these zones and the shoulder is running in the direction that
    // will make things worse.
    if (elbow < 0.0 && 0.0_deg <= elbowAngle_ && elbowAngle_ < arm::elbowNegativeStopLimit)
    {
        comments += " Elbow -STOP";
        elbow = 0.0;

        if (shoulder < 0.0)
        {
            comments += " (Shoulder -STOP)";
            shoulder = 0.0;
        }
    }
    else if (elbow < -arm::elbowParkPower && 0.0_deg <= elbowAngle_ && elbowAngle_ < arm::elbowNegativeParkLimit)
    {
        comments += " Elbow -Park";
        elbow = -arm::elbowParkPower;

        if (shoulder < -arm::shoulderParkPower)
        {
            comments += " (Shoulder -PARK)";
            shoulder = -arm::shoulderParkPower;
        }
    }
    else if (elbow < -arm::elbowSlowPower && 0.0_deg <= elbowAngle_ && elbowAngle_ < arm::elbowNegativeSlowLimit)
    {
        comments += " Elbow -Slow";
        elbow = -arm::elbowSlowPower;

        if (shoulder < -arm::shoulderSlowPower)
        {
            comments += " (Shoulder -SLOW)";
            shoulder = -arm::shoulderSlowPower;
        }
    }

    if (elbow > 0.0 && arm::elbowPositiveStopLimit <= elbowAngle_ && elbowAngle_ < 0.0_deg)
    {
        comments += " Elbow +STOP";
        elbow = 0.0;

        if (shoulder > 0.0)
        {
            comments += " (Shoulder +STOP)";
            shoulder = 0.0;
        }
    }
    else if (elbow > +arm::elbowParkPower && arm::elbowPositiveParkLimit <= elbowAngle_ && elbowAngle_ < 0.0_deg)
    {
        comments += " Elbow +Park";
        elbow = +arm::elbowParkPower;

        if (shoulder > +arm::shoulderParkPower)
        {
            comments += " (Shoulder +PARK)";
            shoulder = +arm::shoulderParkPower;
        }
    }
    else if (elbow > +arm::elbowSlowPower && arm::elbowPositiveSlowLimit <= elbowAngle_ && elbowAngle_ < 0.0_deg)
    {
        comments += " Elbow +Slow";
        elbow = +arm::elbowSlowPower;

        if (shoulder > +arm::shoulderSlowPower)
        {
            comments += " (Shoulder +SLOW)";
            shoulder = +arm::shoulderSlowPower;
        }
    }

    // XXX

    if (print_)
    {
        printf("**** Arm Status: SA=%lf EA=%lf DL=%lf DA=%lf EX=%lf EY=%lf GX=%lf GY=%lf S%%=%lf E%%=%lf SO=%lf EO=%lf%s\n",
               shoulderAngle_.value(),
               elbowAngle_.value(),
               dottedLength_.value(),
               dottedAngle_.value(),
               elbowX_.value(),
               elbowY_.value(),
               gripperX_.value(),
               gripperY_.value(),
               shoulderPercentTorqueGravity,
               elbowPercentTorqueGravity,
               shoulder,
               elbow,
               comments.c_str());
    }

    shoulderMotor_->SetVoltage(shoulder * 12.0_V);
    elbowMotor_->SetVoltage(elbow * 12.0_V);
}

bool ArmSubsystem::InPosition() noexcept
{
    if (!shoulderPIDController_->AtGoal() || !elbowPIDController_->AtGoal())
    {
        return false;
    }

    units::angle::degree_t shoulderError = commandedShoulderAngle_ - shoulderAngle_;
    units::angle::degree_t elbowError = commandedElbowAngle_ - elbowAngle_;

    if (shoulderError < 0.0_deg)
    {
        shoulderError *= -1.0;
    }
    if (elbowError < 0.0_deg)
    {
        elbowError *= -1.0;
    }

    return (shoulderError < arm::kShoulderTolerance) && (elbowError < arm::kElbowTolerance);
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

    test_ = true;

    shoulderMotor_->SetIdleMode(SmartMotorBase::IdleMode::kCoast);
    elbowMotor_->SetIdleMode(SmartMotorBase::IdleMode::kCoast);
}

void ArmSubsystem::TestExit() noexcept
{
    test_ = false;

    shoulderMotor_->SetIdleMode(SmartMotorBase::IdleMode::kBrake);
    elbowMotor_->SetIdleMode(SmartMotorBase::IdleMode::kBrake);
}

void ArmSubsystem::TestPeriodic() noexcept
{
    std::optional<int> shoulderPosition = shoulderSensor_->GetAbsolutePositionWithoutAlignment();
    std::optional<int> elbowPosition = elbowSensor_->GetAbsolutePositionWithoutAlignment();

    // This provides a rough means of zeroing the shoulder position.
    if (shoulderPosition.has_value() && shoulderResetUI_)
    {
        // Work out new alignment so position becomes zero.
        int alignmentOffset = -shoulderPosition.value();
        if (alignmentOffset == +2048)
        {
            alignmentOffset = -2048;
        }

        shoulderSensor_->SetAlignment(alignmentOffset);
    }
    shoulderResetUI_ = false;

    // This provides a rough means of zeroing the elbow position.
    if (elbowPosition.has_value() && elbowResetUI_)
    {
        // Work out new alignment so position becomes zero.
        int alignmentOffset = -elbowPosition.value();
        if (alignmentOffset == +2048)
        {
            alignmentOffset = -2048;
        }

        elbowSensor_->SetAlignment(alignmentOffset);
    }
    shoulderResetUI_ = false;

    Periodic();
}

void ArmSubsystem::DisabledInit() noexcept {}

void ArmSubsystem::DisabledExit() noexcept {}

void ArmSubsystem::BurnConfig() noexcept
{
    shoulderMotor_->ApplyConfig(true);
    elbowMotor_->ApplyConfig(true);
}

void ArmSubsystem::ClearFaults() noexcept
{
    shoulderMotor_->ClearFaults();
    elbowMotor_->ClearFaults();
}

frc2::CommandPtr ArmSubsystem::ArmMethodExampleCommandFactory() noexcept
{
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return RunOnce([/* this */] { /* one-time action goes here */ });
}

void ArmSubsystem::OpenGrip() noexcept
{
    // pneuGrip_->Set(frc::DoubleSolenoid::kForward);
    motorGrip_->SetVoltage(+0.5 * 12.0_V);
}

void ArmSubsystem::CloseGrip() noexcept
{
    // pneuGrip_->Set(frc::DoubleSolenoid::kReverse);
    motorGrip_->SetVoltage(-0.5 * 12.0_V);
}

void ArmSubsystem::RelaxGrip() noexcept
{
    // pneuGrip_->Set(frc::DoubleSolenoid::kOff);
    motorGrip_->Set(0.0);
}
