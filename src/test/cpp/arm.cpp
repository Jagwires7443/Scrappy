#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <gtest/gtest-matchers.h>

#include <frc/simulation/DutyCycleSim.h>

#include "Constants.h"
#include "subsystems/ArmSubsystem.h"

#include <cmath>
#include <string_view>

using frc::sim::DutyCycleSim;
using ::testing::EndsWith;

class ArmTest : public testing::Test
{
protected:
    // Mock AEAT-8800 PWM Position/Rotation Sensors
    // 122-976Hz Frequency, depending on config; 1-8192us Pulsewidth.
    // In this case, period is set up for 1025us (~976Hz), so the
    // minimum duty cycle is 1/1025 and the max is 1024/1025.
    void SetShoulderAngle(units::angle::degree_t angle) noexcept;
    void SetElbowAngle(units::angle::degree_t angle) noexcept;

    void SetAngles(units::angle::degree_t shoulderAngle, units::angle::degree_t elbowAngle) noexcept;
    void Park() noexcept { SetAngles(arm::shoulderPositiveStopLimit, arm::elbowNegativeStopLimit); }
    bool CheckData(std::string_view tag) noexcept;

    ArmSubsystem arm_;

    DutyCycleSim simShoulderSensor_ = DutyCycleSim::CreateForChannel(nonDrive::kShoulderEncoderPort);
    DutyCycleSim simElbowSensor_ = DutyCycleSim::CreateForChannel(nonDrive::kElbowEncoderPort);

    units::angle::degree_t shoulderAngle_{0.0_deg};
    units::angle::degree_t elbowAngle_{0.0_deg};
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
    if (position > 4096.0)
    {
        position -= 4096.0;
    }

    if (position < 0.0 || position > 4095.0)
    {
        position = 0.0;
    }

    position = round(position / 4.0);

    position += 1.0;

    simShoulderSensor_.SetFrequency(976);
    simShoulderSensor_.SetOutput(position / 1025.0);
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
    if (position > 4096.0)
    {
        position -= 4096.0;
    }

    if (position < 0.0 || position > 4095.0)
    {
        position = 0.0;
    }

    position = round(position / 4.0);

    position += 1.0;

    simElbowSensor_.SetFrequency(976);
    simElbowSensor_.SetOutput(position / 1025.0);
}

void ArmTest::SetAngles(units::angle::degree_t shoulderAngle, units::angle::degree_t elbowAngle) noexcept
{
    shoulderAngle_ = shoulderAngle;
    elbowAngle_ = elbowAngle;

    SetShoulderAngle(shoulderAngle);
    SetElbowAngle(elbowAngle);
}

bool ArmTest::CheckData(std::string_view tag) noexcept
{
    units::angle::degree_t shoulderAngle = arm_.GetShoulderAngle();
    units::angle::degree_t elbowAngle = arm_.GetElbowAngle();
    units::length::meter_t elbowX = arm_.GetElbowX();
    units::length::meter_t elbowY = arm_.GetElbowY();
    units::length::meter_t gripperX = arm_.GetGripperX();
    units::length::meter_t gripperY = arm_.GetGripperY();
    units::length::meter_t dottedLength = arm_.GetDottedLength();
    units::angle::degree_t dottedAngle = arm_.GetDottedAngle();

    units::angle::degree_t shoulderAngleError = shoulderAngle_ - shoulderAngle;
    units::angle::degree_t elbowAngleError = elbowAngle_ - elbowAngle;

    if (shoulderAngleError > +2.5_deg || shoulderAngleError < -2.5_deg ||
        elbowAngleError > +2.5_deg || elbowAngleError < -2.5_deg)
    {
        return false;
    }

    return true;
}

// NOTE: Angles can be off by a significant fraction of a degree, due
// to the granularity of the sensor (and work possibly needed in PWMAngleSensor).

// Ensure the mock sensors are behaving as expected (for both PWM and angle).
TEST_F(ArmTest, SensorsFaked)
{
    SetAngles(0.0_deg, 0.0_deg);
    arm_.Periodic();
    EXPECT_GT(0.5, fabs(arm_.GetShoulderAngle().value()));
    EXPECT_GT(0.5, fabs(arm_.GetElbowAngle().value()));

    SetAngles(-180.0_deg, -180.0_deg);
    arm_.Periodic();
    EXPECT_GT(0.5, fabs(arm_.GetShoulderAngle().value() + 180.0));
    EXPECT_GT(0.5, fabs(arm_.GetElbowAngle().value() + 180.0));

    SetAngles(+179.6484375_deg, +179.6484375_deg);
    arm_.Periodic();
    EXPECT_GT(0.5, fabs(arm_.GetShoulderAngle().value() + 180.0));
    EXPECT_GT(0.5, fabs(arm_.GetElbowAngle().value() + 180.0));
}

// Ensure the math is working for the third side of the arm triangle.
TEST_F(ArmTest, DottedCaclulations)
{
    arm_.TestPrint(); // Turn on extra arm output

    double expectedLength{0.0};
    double expectedAngle{0.0};

    SetAngles(0.0_deg, -180.0_deg); // Horizontal rearward / Straight
    arm_.Periodic();
    expectedLength = units::length::meter_t{arm::upperArmLength + arm::lowerArmLength}.value();
    expectedAngle = 0.0;
    EXPECT_GT(0.01, fabs(arm_.GetDottedLength().value() - expectedLength));
    EXPECT_GT(0.75, fabs(arm_.GetDottedAngle().value() - expectedAngle));

    // Right triangles are a special case, ensure math is correct in these cases.

    // Start off with shoulder at zero angle, so arm and robot coordinate systems
    // are aligned.

    SetAngles(0.0_deg, -90.0_deg); // Horizontal forward / Up
    arm_.Periodic();
    expectedLength = std::hypot(units::length::meter_t{arm::upperArmLength}.value(), units::length::meter_t{arm::lowerArmLength}.value());
    expectedAngle = units::angle::degree_t{
        units::angle::radian_t{
            acos(units::length::meter_t{arm::upperArmLength}.value() / expectedLength)}}
                        .value();
    EXPECT_GT(0.01, fabs(arm_.GetDottedLength().value() - expectedLength));
    EXPECT_GT(0.75, fabs(arm_.GetDottedAngle().value() - expectedAngle));

    SetAngles(0.0_deg, +90.0_deg); // Horizontal forward / Down
    arm_.Periodic();
    // expectedLength and expectedAngle are unchanged from prior case, direction is
    // reversed
    EXPECT_GT(0.01, fabs(arm_.GetDottedLength().value() - expectedLength));
    EXPECT_GT(0.75, fabs(arm_.GetDottedAngle().value() - expectedAngle));

    // Now, rotate shoulder and verify that dotted angle is in the robot coordinate
    // system.

    SetAngles(-90.0_deg, +90.0_deg); // Up / Horizontal forward
    arm_.Periodic();
    // expectedLength is unchanged from prior case; expectedAngle is rotated 90 deg
    EXPECT_GT(0.01, fabs(arm_.GetDottedLength().value() - expectedLength));
    EXPECT_GT(0.75, fabs(arm_.GetDottedAngle().value() - expectedAngle + 90.0));

    SetAngles(-180.0_deg, +90.0_deg); // Horizontal backward / Up
    arm_.Periodic();
    // expectedLength is unchanged from prior case; expectedAngle is rotated 90 deg
    EXPECT_GT(0.01, fabs(arm_.GetDottedLength().value() - expectedLength));
    EXPECT_GT(0.75, fabs(arm_.GetDottedAngle().value() - expectedAngle + 180.0));
}

TEST_F(ArmTest, DoPresets)
{
    arm_.TestPrint(); // Turn on extra arm output

    SetAngles(-180.0_deg, -180.0_deg); // Up / Horizontal forward
    arm_.Periodic();

    arm_.SetAngles(arm::shoulderPositiveStopLimit, arm::elbowNegativeStopLimit);
    arm_.Periodic();

    SetAngles(arm::shoulderPositiveStopLimit, arm::elbowNegativeStopLimit);
    arm_.SetAngles(+146.3_deg, -153.3_deg);
    arm_.Periodic();

    SetAngles(+146.3_deg, -153.3_deg);
    arm_.SetAngles(+150.4_deg, -142.8_deg);
    arm_.Periodic();

    SetAngles(+150.4_deg, -142.8_deg);
    arm_.SetAngles(-130.0_deg, +20.0_deg);
    arm_.Periodic();
}

TEST_F(ArmTest, SlowParkStop)
{
    arm_.TestPrint(); // Turn on extra arm output

    SetAngles(-118.0_deg, +30.0_deg); // Sensors place shoulder in stop zone; elbow folded
    arm_.Reset();
    arm_.SetAngles(-110.0_deg, +30.0_deg); // Command arm further into stop zone
    arm_.Periodic();
    EXPECT_THAT(arm_.TestNotes(), EndsWith(" Shoulder +STOP"));

    SetAngles(-62.0_deg, -30.0_deg); // Sensors place shoulder in stop zone; elbow folded
    arm_.Reset();
    arm_.SetAngles(-70.0_deg, -30.0_deg); // Command arm further into stop zone
    arm_.Periodic();
    EXPECT_THAT(arm_.TestNotes(), EndsWith(" Shoulder -STOP"));
}

TEST_F(ArmTest, XY)
{
    Park();
    arm_.Reset();
    arm_.TestPrint(); // Turn on extra arm output

    arm_.SetXY(1.0_m, 0.0_m, true);
    arm_.Periodic();

    arm_.IncrementXY(0.1_m, 0.0_m);
    arm_.Periodic();
}
