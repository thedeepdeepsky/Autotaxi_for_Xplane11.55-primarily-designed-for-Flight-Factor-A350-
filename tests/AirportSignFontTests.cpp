#include "AirportSignFont.h"
#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>

using namespace autotaxi;

int main(int argc, char **argv) {
    try {
        if (argc < 2)
            throw std::runtime_error("OpenTaxiwayMandatorySign.ttf path is required");
        AirportSignFont font;
        if (!font.load(std::filesystem::u8path(argv[1]).wstring()))
            throw std::runtime_error("Open Taxiway Mandatory Sign font did not load");
        if (!font.supports("RWY 36L|18R<"))
            throw std::runtime_error("Airport sign glyph set is incomplete");
        if (font.measure("|") <= 0 || font.measure("A|B") <= font.measure("AB"))
            throw std::runtime_error("Vertical separator is not included in sign width");
        const int index = '|' - 32;
        bool separatorInk = false;
        for (int y = 0; y < AirportSignFont::cellHeight; ++y)
            for (int x = 0; x < AirportSignFont::cellWidth; ++x)
                separatorInk = separatorInk ||
                               font.rgba[((index / 16 * AirportSignFont::cellHeight + y) * AirportSignFont::width +
                                          (index % 16) * AirportSignFont::cellWidth + x) *
                                             4 +
                                         3] != 0;
        if (!separatorInk)
            throw std::runtime_error("Vertical separator glyph is blank");
        std::cout << "Open Taxiway Mandatory Sign font and vertical separator passed\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
