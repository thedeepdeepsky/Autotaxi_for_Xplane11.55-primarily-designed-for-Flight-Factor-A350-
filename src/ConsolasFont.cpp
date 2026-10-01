#include "ConsolasFont.h"
#include <algorithm>
#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <cstring>
#include <cwchar>
#include <windows.h>
#endif
namespace autotaxi {
bool ConsolasFont::load() {
    rgba.clear();
    advance = ascent = 0;
#if defined(_WIN32)
    // GDI rasterizes the installed font once; GL upload happens only in the window draw callback.
    struct Canvas {
        HDC dc = CreateCompatibleDC(nullptr);
        HFONT font = nullptr;
        HBITMAP bitmap = nullptr;
        HGDIOBJ oldFont = nullptr, oldBitmap = nullptr;
        ~Canvas() {
            if (oldFont && oldFont != HGDI_ERROR)
                SelectObject(dc, oldFont);
            if (oldBitmap && oldBitmap != HGDI_ERROR)
                SelectObject(dc, oldBitmap);
            if (font)
                DeleteObject(font);
            if (bitmap)
                DeleteObject(bitmap);
            if (dc)
                DeleteDC(dc);
        }
    } canvas;
    if (!canvas.dc)
        return false;
    canvas.font = CreateFontW(-14, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE, ANSI_CHARSET, OUT_DEFAULT_PRECIS,
                              CLIP_DEFAULT_PRECIS, ANTIALIASED_QUALITY, FIXED_PITCH | FF_MODERN, L"Consolas");
    if (!canvas.font)
        return false;
    canvas.oldFont = SelectObject(canvas.dc, canvas.font);
    wchar_t face[LF_FACESIZE]{};
    TEXTMETRICW metrics{};
    if (!GetTextFaceW(canvas.dc, LF_FACESIZE, face) || std::wcscmp(face, L"Consolas") != 0 ||
        !GetTextMetricsW(canvas.dc, &metrics) || metrics.tmHeight > cellHeight - 2 ||
        metrics.tmAveCharWidth < 1 || metrics.tmAveCharWidth > cellWidth - 2)
        return false;
    BITMAPINFO info{};
    info.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    info.bmiHeader.biWidth = width;
    info.bmiHeader.biHeight = -height;
    info.bmiHeader.biPlanes = 1;
    info.bmiHeader.biBitCount = 32;
    info.bmiHeader.biCompression = BI_RGB;
    void *bits = nullptr;
    canvas.bitmap = CreateDIBSection(canvas.dc, &info, DIB_RGB_COLORS, &bits, nullptr, 0);
    if (!canvas.bitmap || !bits)
        return false;
    canvas.oldBitmap = SelectObject(canvas.dc, canvas.bitmap);
    if (!canvas.oldBitmap || canvas.oldBitmap == HGDI_ERROR)
        return false;
    std::memset(bits, 0, width * height * 4);
    SetBkMode(canvas.dc, TRANSPARENT);
    SetTextColor(canvas.dc, RGB(255, 255, 255));
    for (int code = 32; code <= 126; ++code) {
        int index = code - 32;
        wchar_t character = static_cast<wchar_t>(code);
        if (!TextOutW(canvas.dc, (index % 16) * cellWidth + 1, (index / 16) * cellHeight + 1, &character, 1))
            return false;
    }
    GdiFlush();
    rgba.resize(width * height * 4);
    auto *source = static_cast<const std::uint8_t *>(bits);
    bool visible = false;
    for (std::size_t i = 0; i < rgba.size(); i += 4) {
        rgba[i] = rgba[i + 1] = rgba[i + 2] = 255;
        rgba[i + 3] = std::max({source[i], source[i + 1], source[i + 2]});
        visible = visible || rgba[i + 3] != 0;
    }
    if (!visible) {
        rgba.clear();
        return false;
    }
    advance = metrics.tmAveCharWidth;
    ascent = metrics.tmAscent;
    return true;
#else
    return false;
#endif
}
bool ConsolasFont::supports(const std::string &text) const {
    return !rgba.empty() && std::all_of(text.begin(), text.end(),
                                        [](unsigned char code) { return code >= 32 && code <= 126; });
}
double ConsolasFont::measure(const std::string &text) const {
    return static_cast<double>(text.size()) * advance;
}
} // namespace autotaxi
