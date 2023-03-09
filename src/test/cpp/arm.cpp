#include <gtest/gtest.h>

#include <frc/simulation/DutyCycleSim.h>

#include "Constants.h"
#include "subsystems/ArmSubsystem.h"

#include <cmath>

using frc::sim::DutyCycleSim;

class ArmTest : public testing::Test
{
protected:
    // Mock AEAT-8800 PWM Position/Rotation Sensors
    // 122-976Hz Frequency, depending on config; 1-8192us Pulsewidth.
    // In this case, period is set up for 1025us (~976Hz), so the
    // minimum duty cycle is 1/1025 and the max is 1024/1025.
    void SetShoulderAngle(units::angle::degree_t angle) noexcept;
    void SetElbowAngle(units::angle::degree_t angle) noexcept;

    ArmSubsystem arm_;

    DutyCycleSim simShoulderSensor = DutyCycleSim::CreateForChannel(nonDrive::kShoulderEncoderPort);
    DutyCycleSim simElbowSensor = DutyCycleSim::CreateForChannel(nonDrive::kElbowEncoderPort);
};

// Go from [-180.0, +180.0) to [-0.5, +0.5), then to [0.0, 1.0), then
// to [0, 4095], compensating for alignment by subtracting it out and
// handling the possible wrap.  Then, scale to [0, 1023] and finally,
// to [1, 1024].
void ArmTest::SetShoulderAngle(units::angle::degree_t angle) noexcept
{
    double position{units::angle::turn_t{angle}.value()};

    position += 0.5;
    position *= 4096.0;

    position -= nonDrive::kShoulderAlignmentOffset;
    if (position < 0.0)
    {
        position += 4096.0;
    }

    if (position < 0.0 || position > 4095.0)
    {
        position = 0.0;
    }

    position = round(position / 4.0);

    position += 1.0;

    simShoulderSensor.SetFrequency(976);
    simShoulderSensor.SetOutput(position / 1025.0);
}

void ArmTest::SetElbowAngle(units::angle::degree_t angle) noexcept
{
    double position{units::angle::turn_t{angle}.value()};

    position += 0.5;
    position *= 4096.0;

    position -= nonDrive::kElbowAlignmentOffset;
    if (position < 0.0)
    {
        position += 4096.0;
    }

    if (position < 0.0 || position > 4095.0)
    {
        position = 0.0;
    }

    position = round(position / 4.0);

    position += 1.0;

    simElbowSensor.SetFrequency(976);
    simElbowSensor.SetOutput(position / 1025.0);
}

// NOTE: Angles can be off by a significant fraction of a degree, due
// to the granularity of the sensor (and work possibly needed in PWMAngleSensor).

// Ensure the mock sensors are behaving as expected (for both PWM and angle).
TEST_F(ArmTest, SensorsFaked)
{
    SetShoulderAngle(0.0_deg);
    SetElbowAngle(0.0_deg);
    arm_.Periodic();
    EXPECT_GT(0.275, fabs(arm_.GetShoulderAngle().value()));
    EXPECT_GT(0.275, fabs(arm_.GetElbowAngle().value()));

    SetShoulderAngle(-180.0_deg);
    SetElbowAngle(-180.0_deg);
    arm_.Periodic();
    EXPECT_GT(0.275, fabs(arm_.GetShoulderAngle().value() + 180.0));
    EXPECT_GT(0.275, fabs(arm_.GetElbowAngle().value() + 180.0));

    SetShoulderAngle(+179.6484375_deg);
    SetElbowAngle(+179.6484375_deg);
    arm_.Periodic();
    EXPECT_GT(0.275, fabs(arm_.GetShoulderAngle().value() - 180.0));
    EXPECT_GT(0.275, fabs(arm_.GetElbowAngle().value() - 180.0));
}

// Ensure the math is working for the third side of the arm triangle.
TEST_F(ArmTest, DottedCaclulations)
{
    arm_.TestPrint();

    double expectedLength{0.0};
    double expectedAngle{0.0};

    SetShoulderAngle(0.0_deg); // Horizontal forward
    SetElbowAngle(0.0_deg);    // Straight
    arm_.Periodic();
    expectedLength = units::length::meter_t{arm::upperArmLength + arm::lowerArmLength}.value();
    expectedAngle = 0.0;
    EXPECT_GT(0.01, fabs(arm_.TestGetDottedLength().value() - expectedLength));
    EXPECT_GT(0.35, fabs(arm_.TestGetDottedAngle().value() - expectedAngle));

    // Right triangles are a special case, ensure math is correct in these cases.

    // Start off with shoulder at zero angle, so arm and robot coordinate systems
    // are aligned.

    SetShoulderAngle(0.0_deg); // Horizontal forward
    SetElbowAngle(-90.0_deg);  // Up
    arm_.Periodic();
    expectedLength = sqrt(pow(units::length::meter_t{arm::upperArmLength}.value(), 2.0) +
                          pow(units::length::meter_t{arm::lowerArmLength}.value(), 2.0));
    expectedAngle = units::angle::degree_t{
        units::angle::radian_t{
            acos(units::length::meter_t{arm::upperArmLength}.value() / expectedLength)}}
                        .value();
    EXPECT_GT(0.01, fabs(arm_.TestGetDottedLength().value() - expectedLength));
    EXPECT_GT(0.35, fabs(arm_.TestGetDottedAngle().value() + expectedAngle));

    SetShoulderAngle(0.0_deg); // Horizontal forward
    SetElbowAngle(+90.0_deg);  // Down
    arm_.Periodic();
    // expectedLength and expectedAngle are unchanged from prior case, direction is
    // reversed
    EXPECT_GT(0.01, fabs(arm_.TestGetDottedLength().value() - expectedLength));
    EXPECT_GT(0.35, fabs(arm_.TestGetDottedAngle().value() - expectedAngle));

    // Now, rotate shoulder and verify that dotted angle is in the robot coordinate
    // system.

    SetShoulderAngle(-90.0_deg); // Up
    SetElbowAngle(+90.0_deg);    // Horizontal forward
    arm_.Periodic();
    // expectedLength is unchanged from prior case; expectedAngle is rotated 90 deg
    EXPECT_GT(0.01, fabs(arm_.TestGetDottedLength().value() - expectedLength));
    EXPECT_GT(0.35, fabs(arm_.TestGetDottedAngle().value() - expectedAngle + 90.0));

    SetShoulderAngle(-180.0_deg); // Horizontal backward
    SetElbowAngle(+90.0_deg);     // Up
    arm_.Periodic();
    // expectedLength is unchanged from prior case; expectedAngle is rotated 90 deg
    EXPECT_GT(0.01, fabs(arm_.TestGetDottedLength().value() - expectedLength));
    EXPECT_GT(0.35, fabs(arm_.TestGetDottedAngle().value() - expectedAngle + 180.0));
}
