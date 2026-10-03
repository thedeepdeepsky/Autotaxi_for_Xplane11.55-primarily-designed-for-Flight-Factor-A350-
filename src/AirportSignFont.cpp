#include "AirportSignFont.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace autotaxi {
namespace {
char mappedGlyph(unsigned char code) {
    // OpenTaxiwayMandatorySign assigns its diagonal/cardinal arrows to these
    // lower-case code points to keep the font usable in older software.
    switch (code) {
    case '<': return 'a';
    case '^': return 'w';
    case '>': return 'd';
    case 'v': return 's';
    default: return static_cast<char>(code);
    }
}
}

bool AirportSignFont::load(const std::wstring &path) {
    rgba.clear();
    advances.fill(0);
    ascent = 0;
#if defined(_WIN32)
    const DWORD added = AddFontResourceExW(path.c_str(), FR_PRIVATE, nullptr);
    if (!added)
        return false;
    struct FontGuard {
        std::wstring path;
        ~FontGuard() {
            RemoveFontResourceExW(path.c_str(), FR_PRIVATE, nullptr);
        }
    } guard{path};

    HDC dc = CreateCompatibleDC(nullptr);
    if (!dc)
        return false;
    HFONT font = CreateFontW(-14, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE, ANSI_CHARSET,
                             OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, ANTIALIASED_QUALITY,
                             DEFAULT_PITCH | FF_DONTCARE, L"Open Taxiway Mandatory Sign");
    if (!font) {
        DeleteDC(dc);
        return false;
    }
    HGDIOBJ oldFont = SelectObject(dc, font);
    TEXTMETRICW metrics{};
    if (!GetTextMetricsW(dc, &metrics)) {
        SelectObject(dc, oldFont);
        DeleteObject(font);
        DeleteDC(dc);
        return false;
    }
    BITMAPINFO info{};
    info.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    info.bmiHeader.biWidth = width;
    info.bmiHeader.biHeight = -height;
    info.bmiHeader.biPlanes = 1;
    info.bmiHeader.biBitCount = 32;
    info.bmiHeader.biCompression = BI_RGB;
    void *bits = nullptr;
    HBITMAP bitmap = CreateDIBSection(dc, &info, DIB_RGB_COLORS, &bits, nullptr, 0);
    if (!bitmap || !bits) {
        SelectObject(dc, oldFont);
        DeleteObject(font);
        DeleteDC(dc);
        return false;
    }
    HGDIOBJ oldBitmap = SelectObject(dc, bitmap);
    std::memset(bits, 0, width * height * 4);
    SetBkMode(dc, TRANSPARENT);
    SetTextColor(dc, RGB(255, 255, 255));
    ascent = metrics.tmAscent;

    for (int code = 32; code <= 126; ++code) {
        const int index = code - 32;
        const int x = (index % 16) * cellWidth;
        const int y = (index / 16) * cellHeight;
        if (code == '|') {
            // MH/T 6011-2015 uses a vertical panel divider.  The open font
            // intentionally has no `|` glyph, so draw the standard divider
            // directly instead of substituting the slash glyph.
            const int dividerWidth = 2;
            HPEN pen = CreatePen(PS_SOLID, dividerWidth, RGB(255, 255, 255));
            HGDIOBJ oldPen = SelectObject(dc, pen);
            MoveToEx(dc, x + 4, y + 8, nullptr);
            LineTo(dc, x + 4, y + 24);
            SelectObject(dc, oldPen);
            DeleteObject(pen);
            advances[index] = 8;
            continue;
        }
        const char glyph = mappedGlyph(static_cast<unsigned char>(code));
        const wchar_t wide = static_cast<unsigned char>(glyph);
        ABCFLOAT abc{};
        if (!GetCharABCWidthsFloatW(dc, wide, wide, &abc)) {
            // Keep unsupported control glyphs measurable; they are rendered
            // as an empty cell by the open font.
            advances[index] = 8;
        } else {
            advances[index] = std::max(1.0, static_cast<double>(abc.abcfA + abc.abcfB + abc.abcfC));
        }
        TextOutW(dc, x + 1, y + 1, &wide, 1);
    }
    GdiFlush();
    rgba.resize(width * height * 4);
    auto *source = static_cast<const std::uint8_t *>(bits);
    for (std::size_t i = 0; i < rgba.size(); i += 4) {
        rgba[i] = rgba[i + 1] = rgba[i + 2] = 255;
        rgba[i + 3] = std::max({source[i], source[i + 1], source[i + 2]});
    }
    SelectObject(dc, oldBitmap);
    DeleteObject(bitmap);
    SelectObject(dc, oldFont);
    DeleteObject(font);
    DeleteDC(dc);
    return true;
#else
    (void)path;
    return false;
#endif
}

bool AirportSignFont::supports(const std::string &text) const {
    return !rgba.empty() && std::all_of(text.begin(), text.end(), [](unsigned char code) {
        return code >= 32 && code <= 126;
    });
}

double AirportSignFont::measure(const std::string &text) const {
    double result = 0;
    for (unsigned char code : text)
        result += code >= 32 && code <= 126 ? advances[code - 32] : 8;
    return result;
}

} // namespace autotaxi
