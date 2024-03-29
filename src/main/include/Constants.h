#pragma once

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/mass.h>
#include <units/torque.h>
#include <units/velocity.h>

namespace physical
{
    // Alignment constants, for each swerve module.  Specified on [-2048, 2048)
    // "count" scale, in (dimensionless) angular units.
    constexpr int kFrontLeftAlignmentOffset = +1427;
    constexpr int kFrontRightAlignmentOffset = -903;
    constexpr int kRearLeftAlignmentOffset = +384;
    constexpr int kRearRightAlignmentOffset = -205;

    // SDS MK4i Middle (L2) Gear Ratio: 6.75:1;
    // Nominal Wheel Diameter (4"): =0.1016m;
    // Nominal Wheel Circumference (pi * Diameter): ~0.3192m;
    // 6.75 / 0.3192 => ~21.15.

    // This should be empirically determined!  This is just an initial guess.
    // This is used for both distance and velocity control.  If this is off, it
    // will throw off kMaxDriveSpeed and kMaxTurnRate, as well as drive values.
    constexpr units::length::meter_t kDriveMetersPerRotation = 1.0_m / 21.15;

    // SDS MK4i Middle (L2) Max Free Speed: 14.5 feet/second;
    // This is an upper bound, for various reasons.  It needs to be empirically
    // measured.  Half of theoretical free speed is a reasonable starting value
    // (since something in the ballpark is needed here in order to to drive).
    constexpr units::velocity::meters_per_second_t kMaxDriveSpeed = 22.5_fps / 1.25;

    // For a square drive base, with +/-11.875" x/y coordinates for each of
    // four swerve modules, the radius of the circle going through the turn
    // point of all modules is:
    // sqrt((11.875")^2 + (11.875")^2) ~= 16.79"; the circumference of such a
    // circle is 2*pi*16.79" ~= 105.518".

    // This is used for rotating the robot in place, about it's center.  This
    // may need to be empirically adjusted, but check kDriveMetersPerRotation
    // before making any adjustment here.
    constexpr units::length::meter_t kDriveMetersPerTurningCircle = 105.518_in;

    // This is the maximum rotational speed -- not of a swerve module, but of
    // the entire robot.  This is a function of the maximum drive speed and the
    // geometry of the robot.  This will occur when the robot spins in place,
    // around the center of a circle which passes through all the drive modules
    // (if there is no single such circle, things are analogous).  If the drive
    // modules are turned to be tangential to this circle and run at maximum,
    // the robot is rotating as fast as possible.  This can be derived from
    // kMaxDriveSpeed and the geometry and does not have to be directly
    // measured.  It is a good idea to check this value empirically though.

    // So the maximum rotational velocity (spinning in place) is kMaxDriveSpeed
    // / kDriveMetersPerTurningCircle * 360 degrees.  This should not need to
    // be empirically adjusted (but check).
    constexpr units::angular_velocity::degrees_per_second_t kMaxTurnRate =
        kMaxDriveSpeed / kDriveMetersPerTurningCircle * 360.0_deg;

    // Drivebase geometry: distance between centers of right and left wheels on
    // robot; distance between centers of front and back wheels on robot.
    constexpr units::length::meter_t kTrackWidth = 23.75_in;
    constexpr units::length::meter_t kWheelBase = 23.75_in;

    // CAN ID and Digital I/O Port assignments.
    constexpr int kFrontLeftDriveMotorCanID = 1;
    constexpr int kFrontLeftTurningMotorCanID = 2;
    constexpr int kFrontRightDriveMotorCanID = 3;
    constexpr int kFrontRightTurningMotorCanID = 4;
    constexpr int kRearLeftDriveMotorCanID = 5;
    constexpr int kRearLeftTurningMotorCanID = 6;
    constexpr int kRearRightDriveMotorCanID = 7;
    constexpr int kRearRightTurningMotorCanID = 8;

    constexpr int kFrontLeftTurningEncoderPort = 0;
    constexpr int kFrontRightTurningEncoderPort = 1;
    constexpr int kRearLeftTurningEncoderPort = 2;
    constexpr int kRearRightTurningEncoderPort = 3;

    // These can flip because of gearing.
    // Note that MK3 SDS turning is not inverted, while MK4i is (the motors are
    // litterally inverted).  The drive motor inversion depends on how things
    // are set up in terms of the zero position.  If the bevel gears are
    // aligned so they face inward on both the left and right side, these are
    // opposite.  If they are all made to face one way or the other when they
    // are aligned, these will be the same -- but might all be inverted or not.
    constexpr bool kRightDriveMotorInverted = true;
    constexpr bool kLeftDriveMotorInverted = false;
    constexpr bool kTurningMotorInverted = true;
}

namespace pidf
{
    // SDS MK4i Steering ratio is 150/7:1; NEO Free Speed is 5676 RPM; 264.88
    // RPM or 1589.28 degrees per second.  For MK3, Steering ratio is 12.8:1.
    constexpr units::angular_velocity::degrees_per_second_t kTurningPositionMaxVelocity = 1250.0_deg_per_s;
    constexpr units::angular_acceleration::degrees_per_second_squared_t kTurningPositionMaxAcceleration = 12500.0_deg_per_s_sq;
    constexpr double kTurningPositionP = 0.005;
    constexpr double kTurningPositionF = 0.003;

    constexpr double kTurningPositionI = 0.0;
    constexpr double kTurningPositionIZ = 0.0;
    constexpr double kTurningPositionIM = 0.0;
    constexpr double kTurningPositionD = 0.0;
    constexpr double kTurningPositionDF = 0.0;

    constexpr double kDrivePositionMaxVelocity = 5700.0;     // Rotations per minute.
    constexpr double kDrivePositionMaxAcceleration = 1000.0; // Rotations per minute per second.
    constexpr double kDrivePositionP = 0.004;
    constexpr double kDrivePositionF = 0.0;
    constexpr double kDrivePositionI = 0.0;
    constexpr double kDrivePositionIZ = 0.0;
    constexpr double kDrivePositionIM = 0.0;
    constexpr double kDrivePositionD = 0.0;
    constexpr double kDrivePositionDF = 0.0;

    constexpr double kDriveVelocityMaxVelocity = 5700.0;
    constexpr double kDriveVelocityMaxAcceleration = 1000.0;
    constexpr double kDriveVelocityMaxJerk = 1.0;
    constexpr double kDriveVelocityP = 0.0;
    constexpr double kDriveVelocityF = 0.0;
    constexpr double kDriveVelocityI = 0.0;
    constexpr double kDriveVelocityIZ = 0.0;
    constexpr double kDriveVelocityIM = 0.0;
    constexpr double kDriveVelocityD = 0.0;
    constexpr double kDriveVelocityDF = 0.0;

    constexpr units::angular_velocity::degrees_per_second_t kDriveThetaMaxVelocity = 45.0_deg_per_s;
    constexpr units::angular_acceleration::degrees_per_second_squared_t kDriveThetaMaxAcceleration = 450.0_deg_per_s_sq;
    constexpr double kDriveThetaP = 0.10;
    constexpr double kDriveThetaF = 0.005;
    constexpr double kDriveThetaI = 0.0;
    constexpr double kDriveThetaD = 0.0;
}

namespace nonDrive
{
    constexpr int kShoulderAlignmentOffset = 1398;
    constexpr int kElbowAlignmentOffset = -254;

    constexpr int kShoulderMotorCanID = 9;
    constexpr int kElbowMotorCanID = 10;

    constexpr int kShoulderEncoderPort = 5;
    constexpr int kElbowEncoderPort = 4;

    // These are set so the motor directions match those of the corresponding sensor.
    constexpr bool kShoulderMotorInverted = false;
    constexpr bool kElbowMotorInverted = true;

    constexpr int kGripPneuOpen = 0;
    constexpr int kGripPneuClose = 1;

    constexpr int kDump1 = 2;
    constexpr int kDump2 = 3;
    constexpr int kDump3 = 4;
    constexpr int kDump4 = 5;

    constexpr int kSuctionMotorsOneCanID = 1;
    constexpr int kSuctionMotorsTwoCanID = 2;
}

namespace arm
{
    // Acceleration of gravity (average @Earth's surface)
    constexpr units::acceleration::meters_per_second_squared_t gravity = 1.0_SG;

    // Constraints to avoid violating a game rule, risking a penalties.
    // And, don't lower gripper below an inch off the ground.
    constexpr units::length::meter_t kMaxHorizontalExtension = +62.5_in;
    constexpr units::length::meter_t kMinHorizontalExtension = -62.5_in;
    constexpr units::length::meter_t kMaxVerticalExtension = +38.5_in;
    constexpr units::length::meter_t kMinVerticalExtension = -38.5_in;

    constexpr units::angular_velocity::degrees_per_second_t kShoulderPositionMaxVelocity = 90.0_deg_per_s;
    constexpr units::angular_acceleration::degrees_per_second_squared_t kShoulderPositionMaxAcceleration = 900.0_deg_per_s_sq;
    constexpr double kShoulderPositionP = 0.012;
    constexpr units::angle::degree_t kShoulderTolerance = 2.5_deg;

    constexpr units::angular_velocity::degrees_per_second_t kElbowPositionMaxVelocity = 90.0_deg_per_s;
    constexpr units::angular_acceleration::degrees_per_second_squared_t kElbowPositionMaxAcceleration = 900.0_deg_per_s_sq;
    constexpr double kElbowPositionP = 0.012;
    constexpr units::angle::degree_t kElbowTolerance = 2.5_deg;

    // Fixed voltage required to overcome static friction to get things moving.
    constexpr double shoulderStaticFeedforward = 0.005;
    constexpr double elbowStaticFeedforward = 0.005;

    // Proportionalities for voltages to counter gravity at different joints.
    // NEO empirical stall torque at 12V is 2.6Nm; gearbox is 256:1.
    constexpr units::torque::newton_meter_t shoulderMaxTorque = 250.0 * 2.6_Nm;
    constexpr units::torque::newton_meter_t elbowMaxTorque = 250.0 * 2.6_Nm;

    constexpr units::length::meter_t upperArmLength = 30.0_in;
    constexpr units::length::meter_t lowerArmLength = 26.5_in;
    constexpr units::mass::kilogram_t pointMass = 2.0_lb;

    constexpr double shoulderParkPower = 0.10;
    constexpr double shoulderSlowPower = 0.25;
    constexpr double shoulderMaxPower = 0.40;

    constexpr double elbowParkPower = 0.10;
    constexpr double elbowSlowPower = 0.25;
    constexpr double elbowMaxPower = 0.40;

    // Excluded range of angle for shoulder, centered on -90.0_deg.
    constexpr units::angle::degree_t shoulderNegativeStopLimit = -58.0_deg;
    constexpr units::angle::degree_t shoulderPositiveStopLimit = -120.0_deg;
    constexpr units::angle::degree_t shoulderNegativeParkLimit = -53.0_deg;
    constexpr units::angle::degree_t shoulderPositiveParkLimit = -125.0_deg;
    constexpr units::angle::degree_t shoulderNegativeSlowLimit = -48.0_deg;
    constexpr units::angle::degree_t shoulderPositiveSlowLimit = -130.0_deg;

    // Excluded range of angle for elbow, centered on 0.0_deg.
    constexpr units::angle::degree_t elbowNegativeStopLimit = +20.0_deg;
    constexpr units::angle::degree_t elbowPositiveStopLimit = -20.0_deg;
    constexpr units::angle::degree_t elbowNegativeParkLimit = +25.0_deg;
    constexpr units::angle::degree_t elbowPositiveParkLimit = -25.0_deg;
    constexpr units::angle::degree_t elbowNegativeSlowLimit = +30.0_deg;
    constexpr units::angle::degree_t elbowPositiveSlowLimit = -30.0_deg;
}

namespace autonomous
{
    constexpr units::angle::degree_t balanced = 15.0_deg;
}
