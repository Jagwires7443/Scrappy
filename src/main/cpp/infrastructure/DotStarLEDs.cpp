#include <infrastructure/DotStarLEDs.h>

DotStarLEDs::DotStarLEDs(uint32_t length) noexcept : length_{length}
{
    // Create vector to hold data for each LED.
    data_ = std::make_unique<std::vector<uint8_t>>((length_ + 2) * 4);

    // Header and trailer data uses special (and fixed) values.
    (*data_)[0] = 0x00;
    (*data_)[1] = 0x00;
    (*data_)[2] = 0x00;
    (*data_)[3] = 0x00;
    (*data_)[(length_ + 1) * 4 + 0] = 0xff;
    (*data_)[(length_ + 1) * 4 + 1] = 0xff;
    (*data_)[(length_ + 1) * 4 + 2] = 0xff;
    (*data_)[(length_ + 1) * 4 + 3] = 0xff;

    // Set all LEDs to off.
    SetAllLEDs(0, 0, 0, 0);

    // Specify port/pins for SPI output to LED array.
    SPI_ = std::make_unique<frc::SPI>(frc::SPI::Port::kOnboardCS0);

    SPI_->SetMode(frc::SPI::Mode::kMode0);
    SPI_->SetChipSelectActiveLow();
    SPI_->SetClockRate(500000);

    Apply();
}

void DotStarLEDs::SetLED(uint32_t index, uint8_t red, uint8_t green, uint8_t blue, uint8_t value) noexcept
{
    // No modification beyond the specified length, or value of 32 or higher.
    if (index >= length_ || (value & 0xe0) != 0)
    {
        return;
    }

    // Fill in data for a single LED.
    (*data_)[(index + 1) * 4 + 0] = 0xe0 | value;
    (*data_)[(index + 1) * 4 + 1] = blue;
    (*data_)[(index + 1) * 4 + 2] = green;
    (*data_)[(index + 1) * 4 + 3] = red;
}

void DotStarLEDs::SetAllLEDs(uint8_t red, uint8_t green, uint8_t blue, uint8_t value) noexcept
{
    for (uint32_t u = 0; u < length_; u++)
    {
        SetLED(u, red, green, blue, value);
    }
}

void DotStarLEDs::Apply() noexcept
{
    SPI_->Write(data_->data(), data_->size());
}
