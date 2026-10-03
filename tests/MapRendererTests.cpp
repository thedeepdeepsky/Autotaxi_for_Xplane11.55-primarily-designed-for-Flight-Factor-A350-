#include "MapRenderer.h"
#include <GL/gl.h>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <windows.h>
using namespace autotaxi;
namespace {
constexpr int width = 320, height = 240;
void check(bool condition, const char *message) {
    if (!condition)
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
        wc.lpszClassName = "AutoTaxiMapTest";
        RegisterClassA(&wc);
        window = CreateWindowA(wc.lpszClassName, "", WS_POPUP, 0, 0, 1080, 720, nullptr, nullptr,
                               wc.hInstance, nullptr);
        check(window != nullptr, "Cannot create hidden rendering window");
        device = GetDC(window);
        PIXELFORMATDESCRIPTOR format{};
        format.nSize = sizeof(format);
        format.nVersion = 1;
        format.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
        format.iPixelType = PFD_TYPE_RGBA;
        format.cColorBits = 32;
        const int selected = ChoosePixelFormat(device, &format);
        check(selected && SetPixelFormat(device, selected, &format), "Cannot set OpenGL pixel format");
        context = wglCreateContext(device);
        check(context && wglMakeCurrent(device, context), "Cannot create OpenGL test context");
        glViewport(0, 0, width, height);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(0, width, 0, height, -1, 1);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glTranslated(2, 3, 0);
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
void immediate(const PanelFrame &frame, double left, double top) {
    glPushAttrib(GL_CURRENT_BIT | GL_LINE_BIT);
    for (std::size_t i = frame.staticMapBegin; i < frame.staticMapEnd;) {
        const auto end = std::min(lineBatchEnd(frame.commands, i), frame.staticMapEnd);
        glLineWidth(static_cast<float>(frame.commands[i].width));
        glBegin(GL_LINES);
        for (; i < end; ++i) {
            const auto &line = frame.commands[i];
            glColor3f(line.color.r, line.color.g, line.color.b);
            glVertex2d(left + line.box.x, top - line.box.y);
            glVertex2d(left + line.x2, top - line.y2);
        }
        glEnd();
    }
    glPopAttrib();
}
std::vector<unsigned char> pixels() {
    std::vector<unsigned char> result(width * height * 3);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, result.data());
    return result;
}
void overlay() {
    glPushAttrib(GL_CURRENT_BIT | GL_LINE_BIT);
    glColor3f(0, 1, 0);
    glLineWidth(3);
    glBegin(GL_LINES);
    glVertex2d(70, 160);
    glVertex2d(190, 30);
    glEnd();
    glPopAttrib();
}
void compare(MapRenderer &renderer, PanelFrame &frame, std::size_t revision, double left, double top) {
    glClear(GL_COLOR_BUFFER_BIT);
    immediate(frame, left, top);
    overlay();
    const auto expected = pixels();
    glClear(GL_COLOR_BUFFER_BIT);
    glColor4f(.25f, .5f, .75f, .5f);
    glLineWidth(2);
    glMatrixMode(GL_PROJECTION);
    GLdouble before[16], after[16];
    glGetDoublev(GL_MODELVIEW_MATRIX, before);
    check(renderer.draw(frame, revision, left, top), "Cached map allocation failed");
    GLint mode = 0;
    GLfloat color[4], lineWidth = 0;
    glGetIntegerv(GL_MATRIX_MODE, &mode);
    glGetDoublev(GL_MODELVIEW_MATRIX, after);
    glGetFloatv(GL_CURRENT_COLOR, color);
    glGetFloatv(GL_LINE_WIDTH, &lineWidth);
    check(mode == GL_PROJECTION && std::equal(before, before + 16, after),
          "Cached drawing changed simulator matrices");
    check(color[0] == .25f && color[1] == .5f && color[2] == .75f && color[3] == .5f && lineWidth == 2,
          "Cached drawing changed simulator color or line state");
    overlay();
    check(pixels() == expected, "Cached map pixels differ from immediate drawing");
    check(glGetError() == GL_NO_ERROR, "OpenGL error during cached drawing");
}
template <class F> double timing(F draw) {
    glFinish();
    const auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < 120; ++i) {
        glClear(GL_COLOR_BUFFER_BIT);
        draw();
    }
    glFinish();
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}
void benchmark(const PanelFrame &frame, double top = height) {
    MapRenderer renderer;
    renderer.draw(frame, 1, 0, top);
    const auto baseline = timing([&] { immediate(frame, 0, top); });
    const auto cached = timing([&] { renderer.draw(frame, 1, 0, top); });
    std::cout << std::fixed << std::setprecision(2) << "120 map draws, "
              << frame.staticMapEnd - frame.staticMapBegin << " static segments: immediate " << baseline
              << " ms; cached " << cached << " ms; compiles=" << renderer.compileCount() << '\n';
}
} // namespace
int main(int argc, char **argv) {
    try {
        Context context;
        std::cout << "OpenGL: " << glGetString(GL_RENDERER) << '\n';
        PanelFrame frame;
        DrawCommand outside{};
        outside.kind = DrawKind::Rectangle;
        frame.commands.push_back(outside);
        frame.staticMapBegin = frame.commands.size();
        for (int i = 0; i < 150; ++i) {
            DrawCommand line{};
            line.kind = DrawKind::Line;
            line.box = {10.0 + i, 20, 0, 0};
            line.x2 = 250 - i;
            line.y2 = 210;
            line.color = {i % 2 ? .8f : .3f, .2f, i % 2 ? .7f : .9f};
            line.width = i < 50 ? 1 : (i < 100 ? 3 : 2);
            frame.commands.push_back(line);
        }
        frame.staticMapEnd = frame.commands.size();
        outside.kind = DrawKind::Line;
        outside.box = {0, 0, 0, 0};
        outside.x2 = 300;
        outside.y2 = 220;
        outside.color = {1, 0, 0};
        frame.commands.push_back(outside);
        MapRenderer renderer;
        for (int i = 0; i < 60; ++i)
            compare(renderer, frame, 1, 5, 225);
        check(renderer.compileCount() == 1, "Unchanged map recompiled every frame");
        compare(renderer, frame, 1, 20, 210);
        check(renderer.compileCount() == 1, "Moving the window recompiled airport geometry");
        frame.commands[frame.staticMapBegin].color = {1, 1, 0};
        compare(renderer, frame, 2, 20, 210);
        check(renderer.compileCount() == 2, "Changed geometry did not replace the map cache");
        PanelFrame empty;
        compare(renderer, empty, 3, 20, 210);
        check(renderer.compileCount() == 2, "Empty maps must not compile stale geometry");
        compare(renderer, frame, 4, 20, 210);
        check(renderer.compileCount() == 3, "Restoring airport geometry must update the cache");
        const auto rendered = pixels();
        check(std::any_of(rendered.begin(), rendered.end(), [](auto channel) { return channel > 0; }),
              "Map rendering was blank");
        // Synthetic pixels only; commercial scenery is never exported by this test.
        BITMAPFILEHEADER file{};
        BITMAPINFOHEADER info{};
        file.bfType = 0x4d42;
        file.bfOffBits = sizeof(file) + sizeof(info);
        file.bfSize = file.bfOffBits + rendered.size();
        info.biSize = sizeof(info);
        info.biWidth = width;
        info.biHeight = height;
        info.biPlanes = 1;
        info.biBitCount = 24;
        auto bgr = rendered;
        for (std::size_t i = 0; i < bgr.size(); i += 3)
            std::swap(bgr[i], bgr[i + 2]);
        std::ofstream output("map-render-test.bmp", std::ios::binary);
        output.write(reinterpret_cast<const char *>(&file), sizeof(file));
        output.write(reinterpret_cast<const char *>(&info), sizeof(info));
        output.write(reinterpret_cast<const char *>(bgr.data()), bgr.size());
        if (argc == 2) {
            std::ifstream input(std::filesystem::u8path(argv[1]));
            check(static_cast<bool>(input), "Cannot open local apt.dat");
            PanelState state;
            state.airport = parseAirport(input);
            fitAirport(state);
            glViewport(0, 0, 1080, 720);
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            glOrtho(0, 1080, 0, 720, -1, 1);
            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();
            benchmark(buildPanel(state, 1080, 720, [](const auto &s) { return s.size() * 8.0; }), 720);
        } else
            benchmark(frame);
        std::cout << "Map pixels, colors, widths, transforms and cache lifecycle passed\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "FAILED: " << e.what() << '\n';
        return 1;
    }
}
