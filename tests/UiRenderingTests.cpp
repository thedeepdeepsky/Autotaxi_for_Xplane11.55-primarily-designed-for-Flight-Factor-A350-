#include "TaxiUI.h"
#include "XPLMGraphics.h"
#include <GL/gl.h>
#include <algorithm>
#include <chrono>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <windows.h>
#ifndef GL_CLAMP_TO_EDGE
#define GL_CLAMP_TO_EDGE 0x812F
#endif
using namespace autotaxi;
namespace {
constexpr int screenWidth = 1200, screenHeight = 850;
XPLMCreateWindow_t callbacks{};
int left = 30, top = 820, panelWidth = 1080, panelHeight = 720;
int stateCalls = 0, binds = 0, nativeCalls = 0;
void check(bool value, const char *message) {
    if (!value)
        throw std::runtime_error(message);
}
struct Context {
    HWND window = nullptr;
    HDC device = nullptr;
    HGLRC context = nullptr;
    Context() {
        WNDCLASSA wc{};
        wc.lpfnWndProc = DefWindowProcA;
        wc.hInstance = GetModuleHandle(nullptr);
        wc.lpszClassName = "AutoTaxiUiTest";
        RegisterClassA(&wc);
        window = CreateWindowA(wc.lpszClassName, "", WS_POPUP, 0, 0, screenWidth, screenHeight, nullptr,
                               nullptr, wc.hInstance, nullptr);
        check(window != nullptr, "Cannot create rendering window");
        device = GetDC(window);
        PIXELFORMATDESCRIPTOR format{};
        format.nSize = sizeof(format);
        format.nVersion = 1;
        format.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
        format.iPixelType = PFD_TYPE_RGBA;
        format.cColorBits = 32;
        const int selected = ChoosePixelFormat(device, &format);
        check(selected && SetPixelFormat(device, selected, &format), "Cannot set pixel format");
        context = wglCreateContext(device);
        check(context && wglMakeCurrent(device, context), "Cannot create test context");
        glViewport(0, 0, screenWidth, screenHeight);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(0, screenWidth, 0, screenHeight, -1, 1);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glDrawBuffer(GL_BACK);
        glReadBuffer(GL_BACK);
    }
    ~Context() {
        wglMakeCurrent(nullptr, nullptr);
        if (context)
            wglDeleteContext(context);
        if (device)
            ReleaseDC(window, device);
        if (window)
            DestroyWindow(window);
    }
};
std::vector<unsigned char> pixels() {
    std::vector<unsigned char> result(screenWidth * screenHeight * 3);
    glReadPixels(0, 0, screenWidth, screenHeight, GL_RGB, GL_UNSIGNED_BYTE, result.data());
    return result;
}
void point(double x, double y) {
    glVertex2d(left + x, top - y);
}
struct ReferenceRenderer {
    ConsolasFont font;
    int texture = 0;
    MapRenderer map;
    ReferenceRenderer() {
        check(font.load(), "Cannot load Consolas");
        XPLMGenerateTextureNumbers(&texture, 1);
        XPLMBindTexture2d(texture, 0);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, ConsolasFont::width, ConsolasFont::height, 0, GL_RGBA,
                     GL_UNSIGNED_BYTE, font.rgba.data());
    }
    ~ReferenceRenderer() {
        const auto id = static_cast<GLuint>(texture);
        glDeleteTextures(1, &id);
    }
    void text(const DrawCommand &d) {
        double x = std::round(left + d.box.x), baseline = top - d.box.y - 12;
        if (!font.supports(d.text)) {
            float color[] = {d.color.r, d.color.g, d.color.b};
            auto label = d.text;
            XPLMDrawString(color, static_cast<int>(left + d.box.x), static_cast<int>(baseline), label.data(),
                           nullptr, xplmFont_Proportional);
            XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);
            return;
        }
        XPLMSetGraphicsState(0, 1, 0, 0, 1, 0, 0);
        XPLMBindTexture2d(texture, 0);
        GLint environment;
        glGetTexEnviv(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, &environment);
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
        glColor3f(d.color.r, d.color.g, d.color.b);
        const double y = std::round(baseline) + font.ascent + 1;
        glBegin(GL_QUADS);
        for (unsigned char c : d.text) {
            const int index = c - 32;
            const double u = static_cast<double>(index % 16 * ConsolasFont::cellWidth) / ConsolasFont::width;
            const double v =
                static_cast<double>(index / 16 * ConsolasFont::cellHeight) / ConsolasFont::height;
            const double u2 = u + static_cast<double>(ConsolasFont::cellWidth) / ConsolasFont::width;
            const double v2 = v + static_cast<double>(ConsolasFont::cellHeight) / ConsolasFont::height;
            glTexCoord2d(u, v);
            glVertex2d(x - 1, y);
            glTexCoord2d(u2, v);
            glVertex2d(x - 1 + ConsolasFont::cellWidth, y);
            glTexCoord2d(u2, v2);
            glVertex2d(x - 1 + ConsolasFont::cellWidth, y - ConsolasFont::cellHeight);
            glTexCoord2d(u, v2);
            glVertex2d(x - 1, y - ConsolasFont::cellHeight);
            x += font.advance;
        }
        glEnd();
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, environment);
        XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);
    }
    void draw(const PanelFrame &frame, std::size_t revision) {
        XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);
        bool staticMapDrawn = false;
        for (std::size_t i = 0; i < frame.commands.size();) {
            if (!staticMapDrawn && i == frame.staticMapBegin && frame.staticMapEnd > i)
                staticMapDrawn = map.draw(frame, revision, left, top);
            if (staticMapDrawn && i >= frame.staticMapBegin && i < frame.staticMapEnd &&
                frame.commands[i].kind == DrawKind::Line) {
                ++i;
                continue;
            }
            const auto &d = frame.commands[i];
            glColor3f(d.color.r, d.color.g, d.color.b);
            if (d.kind == DrawKind::Line) {
                auto end = lineBatchEnd(frame.commands, i);
                if (i < frame.staticMapBegin)
                    end = std::min(end, frame.staticMapBegin);
                glLineWidth(static_cast<float>(d.width));
                glBegin(GL_LINES);
                for (; i < end; ++i) {
                    const auto &segment = frame.commands[i];
                    glColor3f(segment.color.r, segment.color.g, segment.color.b);
                    point(segment.box.x, segment.box.y);
                    point(segment.x2, segment.y2);
                }
                glEnd();
                glLineWidth(1);
                continue;
            }
            if (d.kind == DrawKind::Rectangle) {
                glBegin(GL_QUADS);
                point(d.box.x, d.box.y);
                point(d.box.x + d.box.w, d.box.y);
                point(d.box.x + d.box.w, d.box.y + d.box.h);
                point(d.box.x, d.box.y + d.box.h);
                glEnd();
            } else if (d.kind == DrawKind::Triangle) {
                glBegin(GL_TRIANGLES);
                point(d.box.x, d.box.y);
                point(d.x2, d.y2);
                point(d.x3, d.y3);
                glEnd();
            } else
                text(d);
            ++i;
        }
    }
};
template <class F> double timing(F draw) {
    glFinish();
    const auto begin = std::chrono::steady_clock::now();
    for (int i = 0; i < 120; ++i) {
        glClear(GL_COLOR_BUFFER_BIT);
        draw();
    }
    glFinish();
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
}
} // namespace
extern "C" {
void XPLMDebugString(const char *) {}
XPLMWindowID XPLMCreateWindowEx(XPLMCreateWindow_t *params) {
    callbacks = *params;
    return &callbacks;
}
void XPLMDestroyWindow(XPLMWindowID) {}
void XPLMSetWindowTitle(XPLMWindowID, const char *) {}
void XPLMSetWindowResizingLimits(XPLMWindowID, int, int, int, int) {}
void XPLMGetScreenBoundsGlobal(int *l, int *t, int *r, int *b) {
    *l = 0;
    *t = screenHeight;
    *r = screenWidth;
    *b = 0;
}
void XPLMGetWindowGeometry(XPLMWindowID, int *l, int *t, int *r, int *b) {
    *l = left;
    *t = top;
    *r = left + panelWidth;
    *b = top - panelHeight;
}
void XPLMGetMouseLocationGlobal(int *x, int *y) {
    *x = -1;
    *y = -1;
}
void XPLMSetWindowIsVisible(XPLMWindowID, int) {}
int XPLMGetWindowIsVisible(XPLMWindowID) {
    return 1;
}
void XPLMBringWindowToFront(XPLMWindowID) {}
void XPLMTakeKeyboardFocus(XPLMWindowID) {}
int XPLMHasKeyboardFocus(XPLMWindowID) {
    return 0;
}
float XPLMMeasureString(XPLMFontID, const char *, int count) {
    return count * 8.0f;
}
void XPLMGenerateTextureNumbers(int *textures, int count) {
    glGenTextures(count, reinterpret_cast<GLuint *>(textures));
}
void XPLMBindTexture2d(int texture, int) {
    ++binds;
    glBindTexture(GL_TEXTURE_2D, texture);
}
void XPLMSetGraphicsState(int, int textures, int, int, int, int, int) {
    ++stateCalls;
    if (textures)
        glEnable(GL_TEXTURE_2D);
    else
        glDisable(GL_TEXTURE_2D);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}
void XPLMDrawString(float *color, int x, int y, char *, int *, XPLMFontID) {
    ++nativeCalls;
    glDisable(GL_TEXTURE_2D);
    glColor3fv(color);
    glBegin(GL_QUADS);
    glVertex2i(x, y);
    glVertex2i(x + 8, y);
    glVertex2i(x + 8, y + 12);
    glVertex2i(x, y + 12);
    glEnd();
    // The SDK permits native string drawing to change graphics state.
    glEnable(GL_TEXTURE_2D);
}
}
int main() {
    try {
        Context context;
        TaxiUI ui({});
        ReferenceRenderer reference;
        PanelState state;
        state.aircraft = {{31, 121}, 27, 2, true};
        state.airport.id = "TEST";
        state.airport.name = "Synthetic rendering";
        state.airport.groundLines = {
            {1, "Paint", {state.aircraft.position, unproject(state.aircraft.position, {100, 100})}}};
        state.airport.ramps = {{state.aircraft.position, 0, 'E', "Test stand", {}}};
        state.choices = destinations(state.airport);
        fitAirport(state);
        ui.setTelemetry(state.aircraft, state.output, 0, false);
        ui.setAirport(state.airport);
        state.availability["Test stand"] = {true, false, 100, {}};
        ui.setDestinationAvailability("Test stand", state.availability.at("Test stand"));
        const auto draw = [&] { callbacks.drawWindowFunc(&callbacks, callbacks.refcon); };
        std::size_t revision = 0;
        for (bool fallback : {false, true}) {
            state.status = fallback ? std::string("Airport ") + char(0x80) : "Synthetic status";
            ui.setStatus(state.status);
            for (bool compact : {false, true}) {
                panelWidth = compact ? 900 : 1080;
                panelHeight = compact ? 600 : 720;
                left = compact ? 75 : 30;
                top = compact ? 770 : 820;
                const auto frame = buildPanel(state, panelWidth, panelHeight, [&](const auto &text) {
                    return reference.font.supports(text) ? reference.font.measure(text) : text.size() * 8.0;
                });
                glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
                glClear(GL_COLOR_BUFFER_BIT);
                stateCalls = binds = nativeCalls = 0;
                reference.draw(frame, ++revision);
                const auto expected = pixels();
                const int originalStateCalls = stateCalls, originalBinds = binds;
                glClear(GL_COLOR_BUFFER_BIT);
                stateCalls = binds = nativeCalls = 0;
                draw();
                check(pixels() == expected, "Batched UI pixels changed text/color/geometry/order");
                check(stateCalls < originalStateCalls && binds < originalBinds,
                      "Text batching did not reduce graphics state and texture calls");
                check(nativeCalls == (fallback ? 1 : 0), "Native font fallback was lost");
                GLint environment;
                glGetTexEnviv(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, &environment);
                check(environment == GL_REPLACE && !glIsEnabled(GL_TEXTURE_2D),
                      "UI did not restore text environment/state");
                check(glGetError() == GL_NO_ERROR, "OpenGL error in panel drawing");
                if (!compact && !fallback) {
                    std::cout << "Graphics state calls/frame: " << originalStateCalls << " -> " << stateCalls
                              << "; texture binds: " << originalBinds << " -> " << binds << '\n';
                    const auto before = timing([&] { reference.draw(frame, revision); });
                    const auto after = timing(draw);
                    std::cout << "120 synthetic panel draws (GPU complete): unbatched " << before
                              << " ms; batched " << after << " ms\n";
                }
            }
        }
        std::cout << "UI pixels, text batching, moving/resizing and native fallback passed\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
