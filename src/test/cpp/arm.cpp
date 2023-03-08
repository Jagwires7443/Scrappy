#include <gtest/gtest.h>

#include <frc/simulation/PWMSim.h>

#include "Constants.h"
#include "subsystems/ArmSubsystem.h"

class ArmTest : public testing::Test
{
protected:
    ArmSubsystem arm_;
    frc::sim::PWMSim simShoulderSensor{nonDrive::kShoulderEncoderPort};
    frc::sim::PWMSim simElbowSensor{nonDrive::kElbowEncoderPort};
};

// Ensure the mock sensors are behaving as expected (for both PWM and angle).
TEST_F(ArmTest, SensorsFaked)
{
    simShoulderSensor.SetSpeed(0.0);
    simElbowSensor.SetSpeed(0.0);

    arm_.Periodic();

    EXPECT_EQ(0.0_deg, arm_.GetShoulderAngle());
    EXPECT_EQ(0.0_deg, arm_.GetElbowAngle());
}
