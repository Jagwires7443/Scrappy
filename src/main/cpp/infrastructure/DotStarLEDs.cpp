#include <infrastructure/DotStarLEDs.h>

#include <algorithm>
#include <cmath>

DotStarLEDs::DotStarLEDs(uint32_t length) noexcept : length_{length}
{
    // See https://cpldcpu.wordpress.com/2016/12/13/sk9822-a-clone-of-the-apa102/
    // for details; extra clock cycles are needed to get everything to the end of
    // the string, due to the way the clock signal is passed down the line.
    uint32_t extraLength = ((length + 1) / 2 + 31) / 32;

    // Create vector to hold data for each LED, plus SPI data overhead for these.
    data_ = std::make_unique<std::vector<uint8_t>>((length_ + 2 + extraLength) * 4);

    // Header and trailer data uses special (and fixed) values: all zero.
    (*data_)[0] = 0x00;
    (*data_)[1] = 0x00;
    (*data_)[2] = 0x00;
    (*data_)[3] = 0x00;
    (*data_)[(length_ + 1) * 4 + 0] = 0x00;
    (*data_)[(length_ + 1) * 4 + 1] = 0x00;
    (*data_)[(length_ + 1) * 4 + 2] = 0x00;
    (*data_)[(length_ + 1) * 4 + 3] = 0x00;
    for (uint32_t u = (length_ + 2) * 4; u < (length_ + 2 + extraLength) * 4; u++)
    {
        (*data_)[u] = 0x00;
    }

    // Set data for all LEDs for off.
    SetAllLEDs(bgrv{});

    // Specify port/pins for SPI output to LED array.
    SPI_ = std::make_unique<frc::SPI>(frc::SPI::Port::kOnboardCS0);

    // XXX
    // Mode 0 is Clock idle low, data sampled on rising edge   <==
    // Mode 1 is Clock idle low, data sampled on falling edge
    // Mode 2 is Clock idle high, data sampled on falling edge
    // Mode 3 is Clock idle high, data sampled on rising edge  <==
    // XXX

    // SPI_->SetMode(frc::SPI::Mode::kMode0);

    // SPI_->SetChipSelectActiveLow();
    SPI_->SetClockRate(250000);
}

void DotStarLEDs::SetLED(uint32_t index, const bgrv &color) noexcept
{
    // No modification beyond the specified length, or value of 32 or higher.
    if (index >= length_ || (color.value & 0xe0) != 0)
    {
        return;
    }

    // Fill in data for a single LED.
    (*data_)[(index + 1) * 4 + 0] = 0xe0 | color.value;
    (*data_)[(index + 1) * 4 + 1] = color.blue;
    (*data_)[(index + 1) * 4 + 2] = color.green;
    (*data_)[(index + 1) * 4 + 3] = color.red;
}

void DotStarLEDs::SetAllLEDs(const bgrv &color) noexcept
{
    for (uint32_t u = 0; u < length_; u++)
    {
        SetLED(u, color);
    }
}

void DotStarLEDs::Apply() noexcept
{
    SPI_->Write(data_->data(), data_->size());
}

// Forward declaration of OKhsv code (see below).
namespace
{
    DotStarLEDs::bgrv okhsv_to_srgb(double a, double b, double s, double v) noexcept;
}

DotStarLEDs::bgrv DotStarLEDs::HsvToBgrv(const DotStarLEDs::hsv &color, unsigned int ledValue) noexcept
{
    if (color.hue < 0.0_deg || color.hue >= 360.0_deg ||
        color.saturation < 0.0 || color.saturation > 1.0 ||
        color.value < 0.0 || color.value > 1.0 ||
        ledValue > 31)
    {
        return bgrv{};
    }

    // Convert hue to Cartesian coordinates on the unit circle.
    const double x = std::cos(units::angle::radian_t{color.hue}.value());
    const double y = std::sin(units::angle::radian_t{color.hue}.value());

    // HSV to RGB
    bgrv result = okhsv_to_srgb(x, y, color.saturation, color.value);

    result.value = ledValue;

    return result;
}

// ############################################################################

// Code below derived from https://bottosson.github.io/misc/ok_color.h (copyright below):

// Copyright(c) 2021 Björn Ottosson
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of
// this softwareand associated documentation files(the "Software"), to deal in
// the Software without restriction, including without limitation the rights to
// use, copy, modify, merge, publish, distribute, sublicense, and /or sell copies
// of the Software, and to permit persons to whom the Software is furnished to do
// so, subject to the following conditions :
// The above copyright noticeand this permission notice shall be included in all
// copies or substantial portions of the Software.
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

namespace
{
    struct Lab
    {
        double L;
        double a;
        double b;
    };

    struct RGB
    {
        double r;
        double g;
        double b;
    };

    struct HSV
    {
        double h;
        double s;
        double v;
    };

    struct LC
    {
        double L;
        double C;
    };

    // Alternative representation of (L_cusp, C_cusp)
    // Encoded so S = C_cusp/L_cusp and T = C_cusp/(1-L_cusp)
    // The maximum value for C in the triangle is then found as fmin(S*L, T*(1-L)), for a given L
    struct ST
    {
        double S;
        double T;
    };

    uint32_t srgb_transfer_function(double a) noexcept
    {
        const double result = 0.0031308 >= a ? 12.92 * a : 1.055 * std::pow(a, 0.4166666666666667) - 0.055;

        return std::round(255.0 * result);
    }

    RGB oklab_to_linear_srgb(Lab c) noexcept
    {
        double l_ = c.L + 0.3963377774 * c.a + 0.2158037573 * c.b;
        double m_ = c.L - 0.1055613458 * c.a - 0.0638541728 * c.b;
        double s_ = c.L - 0.0894841775 * c.a - 1.2914855480 * c.b;

        double l = std::pow(l_, 3.0);
        double m = std::pow(m_, 3.0);
        double s = std::pow(s_, 3.0);

        return {
            +4.0767416621 * l - 3.3077115913 * m + 0.2309699292 * s,
            -1.2684380046 * l + 2.6097574011 * m - 0.3413193965 * s,
            -0.0041960863 * l - 0.7034186147 * m + 1.7076147010 * s,
        };
    }

    // Finds the maximum saturation possible for a given hue that fits in sRGB
    // Saturation here is defined as S = C/L
    // a and b must be normalized so a^2 + b^2 == 1
    double compute_max_saturation(double a, double b) noexcept
    {
        // Max saturation will be when one of r, g or b goes below zero.

        // Select different coefficients depending on which component goes below zero first
        double k0, k1, k2, k3, k4, wl, wm, ws;

        if (-1.88170328 * a - 0.80936493 * b > 1)
        {
            // Red component
            k0 = +1.19086277;
            k1 = +1.76576728;
            k2 = +0.59662641;
            k3 = +0.75515197;
            k4 = +0.56771245;
            wl = +4.0767416621;
            wm = -3.3077115913;
            ws = +0.2309699292;
        }
        else if (1.81444104 * a - 1.19445276 * b > 1)
        {
            // Green component
            k0 = +0.73956515;
            k1 = -0.45954404;
            k2 = +0.08285427;
            k3 = +0.12541070;
            k4 = +0.14503204;
            wl = -1.2684380046;
            wm = +2.6097574011;
            ws = -0.3413193965;
        }
        else
        {
            // Blue component
            k0 = +1.35733652;
            k1 = -0.00915799;
            k2 = -1.15130210;
            k3 = -0.5055960;
            k4 = +0.00692167;
            wl = -0.0041960863;
            wm = -0.7034186147;
            ws = +1.7076147010;
        }

        // Approximate max saturation using a polynomial:
        double S = k0 + k1 * a + k2 * b + k3 * a * a + k4 * a * b;

        // Do one step Halley's method to get closer
        // this gives an error less than 10e6, except for some blue hues where the dS/dh is close to infinite
        // this should be sufficient for most applications, otherwise do two/three steps

        double k_l = +0.3963377774 * a + 0.2158037573 * b;
        double k_m = -0.1055613458 * a - 0.0638541728 * b;
        double k_s = -0.0894841775 * a - 1.2914855480 * b;

        {
            double l_ = 1.0 + S * k_l;
            double m_ = 1.0 + S * k_m;
            double s_ = 1.0 + S * k_s;

            double l = std::pow(l_, 3.0);
            double m = std::pow(m_, 3.0);
            double s = std::pow(s_, 3.0);

            double l_dS = 3.0 * k_l * std::pow(l_, 2.0);
            double m_dS = 3.0 * k_m * std::pow(m_, 2.0);
            double s_dS = 3.0 * k_s * std::pow(s_, 2.0);

            double l_dS2 = 6.0 * std::pow(k_l, 2.0) * l_;
            double m_dS2 = 6.0 * std::pow(k_m, 2.0) * m_;
            double s_dS2 = 6.0 * std::pow(k_s, 2.0) * s_;

            double f = wl * l + wm * m + ws * s;
            double f1 = wl * l_dS + wm * m_dS + ws * s_dS;
            double f2 = wl * l_dS2 + wm * m_dS2 + ws * s_dS2;

            S = S - f * f1 / (f1 * f1 - 0.5 * f * f2);
        }

        return S;
    }

    // finds L_cusp and C_cusp for a given hue
    // a and b must be normalized so a^2 + b^2 == 1
    LC find_cusp(double a, double b) noexcept
    {
        // First, find the maximum saturation (saturation S = C/L)
        double S_cusp = compute_max_saturation(a, b);

        // Convert to linear sRGB to find the first point where at least one of r,g or b >= 1:
        RGB rgb_at_max = oklab_to_linear_srgb({1, S_cusp * a, S_cusp * b});
        double L_cusp = std::cbrt(1.0 / fmax(fmax(rgb_at_max.r, rgb_at_max.g), rgb_at_max.b));
        double C_cusp = L_cusp * S_cusp;

        return {L_cusp, C_cusp};
    }

    double toe_inv(double x) noexcept
    {
        constexpr double k_1 = 0.206;
        constexpr double k_2 = 0.03;
        constexpr double k_3 = (1.0 + k_1) / (1.0 + k_2);

        return (x * x + k_1 * x) / (k_3 * (x + k_2));
    }

    ST to_ST(LC cusp) noexcept
    {
        double L = cusp.L;
        double C = cusp.C;

        return {C / L, C / (1 - L)};
    }

    DotStarLEDs::bgrv okhsv_to_srgb(double a, double b, double s, double v) noexcept
    {
        LC cusp = find_cusp(a, b);
        ST ST_max = to_ST(cusp);
        double S_max = ST_max.S;
        double T_max = ST_max.T;
        double S_0 = 0.5;
        double k = 1.0 - S_0 / S_max;

        // first we compute L and V as if the gamut is a perfect triangle:

        // L, C when v==1:
        double L_v = 1.0 - s * S_0 / (S_0 + T_max - T_max * k * s);
        double C_v = s * T_max * S_0 / (S_0 + T_max - T_max * k * s);

        double L = v * L_v;
        double C = v * C_v;

        // then we compensate for both toe and the curved top part of the triangle:
        double L_vt = toe_inv(L_v);
        double C_vt = C_v * L_vt / L_v;

        double L_new = toe_inv(L);
        C = C * L_new / L;
        L = L_new;

        RGB rgb_scale = oklab_to_linear_srgb({L_vt, a * C_vt, b * C_vt});
        double scale_L = cbrt(1.0 / fmax(fmax(rgb_scale.r, rgb_scale.g), fmax(rgb_scale.b, 0.0)));

        L = L * scale_L;
        C = C * scale_L;

        RGB rgb = oklab_to_linear_srgb({L, C * a, C * b});

        return DotStarLEDs::bgrv{
            .blue = srgb_transfer_function(rgb.b),
            .green = srgb_transfer_function(rgb.g),
            .red = srgb_transfer_function(rgb.r)};
    }
}

// ############################################################################
