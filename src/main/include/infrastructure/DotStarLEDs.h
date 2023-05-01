#pragma once

#include <frc/SPI.h>

#include <memory>
#include <vector>

// Handles APA102-style SPI LEDs (e.g. Adafruit DotStar), cascaded to the specified
// length.  Note that "value" is restricted to 5 bits (0-31).  The "index" runs
// from zero to "length" - 1.

class DotStarLEDs
{
public:
    DotStarLEDs(uint32_t length) noexcept;

    DotStarLEDs(const DotStarLEDs &) = delete;
    DotStarLEDs &operator=(const DotStarLEDs &) = delete;

    void SetLED(uint32_t index, uint8_t red, uint8_t green, uint8_t blue, uint8_t value) noexcept;
    void SetAllLEDs(uint8_t red, uint8_t green, uint8_t blue, uint8_t value) noexcept;

    void Apply() noexcept;

private:
    std::unique_ptr<std::vector<uint8_t>> data_;
    std::unique_ptr<frc::SPI> SPI_;

    const uint32_t length_;
};
