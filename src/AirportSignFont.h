#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace autotaxi {

// Rasterized Open Taxiway Mandatory Sign glyphs.  The font is distributed by
// Open-Runway-Fonts (CC0/OFL) and is used only for airport sign text; the
// surrounding panel continues to use Consolas.
struct AirportSignFont {
    static constexpr int width = 512, height = 192, cellWidth = 32, cellHeight = 32;
    std::array<double, 95> advances{};
    int ascent = 0;
    std::vector<std::uint8_t> rgba;
    bool load(const std::wstring &path);
    bool supports(const std::string &text) const;
    double measure(const std::string &text) const;
};

} // namespace autotaxi
