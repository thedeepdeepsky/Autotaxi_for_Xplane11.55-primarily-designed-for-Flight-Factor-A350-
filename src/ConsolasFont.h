#pragma once
#include <cstdint>
#include <string>
#include <vector>
namespace autotaxi {
struct ConsolasFont {
    static constexpr int width = 256, height = 128, cellWidth = 16, cellHeight = 20;
    int advance = 0, ascent = 0;
    std::vector<std::uint8_t> rgba;
    bool load();
    bool supports(const std::string &text) const;
    double measure(const std::string &text) const;
};
} // namespace autotaxi
