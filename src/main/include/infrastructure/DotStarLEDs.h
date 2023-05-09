#pragma once

#include <frc/SPI.h>

#include <units/angle.h>

#include <memory>
#include <vector>

// Handles APA102-style SPI LEDs (e.g. Adafruit DotStar), cascaded to the specified
// length.  Note that "value" is restricted to 5 bits (0-31).  The "index" runs
// from zero to "length" - 1.  Follows OpenCV conventions (BGR, HSV) and uses Okhsv
// for color picking (https://bottosson.github.io/posts/colorpicker/).

class DotStarLEDs
{
public:
    // Native data format for LEDs.
    struct bgrv
    {
        unsigned int blue : 8 = 0;
        unsigned int green : 8 = 0;
        unsigned int red : 8 = 0;
        unsigned int value : 5 = 0;
    };

    // Format designed to allow for picking a color in a user-friendly way.
    struct hsv
    {
        units::angle::degree_t hue = 0.0_deg; // [0.0 - 360.0)
        double saturation = 0.0;              // [0.0 - 1.0]
        double value = 0.0;                   // [0.0 - 1.0]
    };

    // This is computationally expensive, so precompute and cache these values.
    static bgrv HsvToBgrv(const hsv &color, unsigned int ledValue) noexcept;

    DotStarLEDs(uint32_t length) noexcept;

    DotStarLEDs(const DotStarLEDs &) = delete;
    DotStarLEDs &operator=(const DotStarLEDs &) = delete;

    // These update in-memory data only, use Apply() to make visible.
    void SetLED(uint32_t index, const bgrv &color) noexcept;
    void SetAllLEDs(const bgrv &color) noexcept;

    // This updates the actual LEDs and may very briefly block.
    void Apply() noexcept;

private:
    const uint32_t length_{0};

    std::unique_ptr<std::vector<uint8_t>> data_;
    std::unique_ptr<frc::SPI> SPI_;
};
