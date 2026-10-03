#include "TaxiUI.h"
#include "XPLMGraphics.h"
#include "XPLMUtilities.h"
#if IBM
#include <windows.h>
#endif
#if APL
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#include <GL/glext.h>
#endif
#include <algorithm>
#include <cmath>
#include <filesystem>
namespace autotaxi {
double TaxiUI::measure(const std::string &text) const {
    if (font_.supports(text))
        return font_.measure(text);
    return XPLMMeasureString(xplmFont_Proportional, text.data(), static_cast<int>(text.size()));
}
double TaxiUI::measureSign(const std::string &text) const {
    if (signFont_.supports(text))
        return signFont_.measure(text);
    return measure(text);
}
TaxiUI::TaxiUI(UIActions actions) : actions_(std::move(actions)) {
    if (font_.load())
        XPLMDebugString("[A350AutoTaxi] UI font: Consolas 14 px\n");
    else
        XPLMDebugString("[A350AutoTaxi] Consolas unavailable; using X-Plane UI font\n");
#if IBM
    HMODULE module = nullptr;
    const auto address = reinterpret_cast<LPCSTR>(reinterpret_cast<const void *>(&TaxiUI::draw));
    if (GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
                               GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                           address, &module)) {
        char modulePath[MAX_PATH]{};
        if (GetModuleFileNameA(module, modulePath, MAX_PATH)) {
            const auto path = std::filesystem::path(modulePath).parent_path() / "OpenTaxiwayMandatorySign.ttf";
            std::wstring widePath = path.wstring();
            if (signFont_.load(widePath))
                XPLMDebugString("[A350AutoTaxi] airport signs: Open Taxiway Mandatory Sign\n");
            else
                XPLMDebugString("[A350AutoTaxi] airport sign font missing; using panel font\n");
        }
    }
#endif
    int left, top, right, bottom;
    XPLMGetScreenBoundsGlobal(&left, &top, &right, &bottom);
    int width = std::min(1080, right - left - 60), height = std::min(720, top - bottom - 80);
    width = std::max(900, width);
    height = std::max(600, height);
    XPLMCreateWindow_t params{};
    params.structSize = sizeof(params);
    params.left = left + (right - left - width) / 2;
    params.top = top - 40;
    params.right = params.left + width;
    params.bottom = params.top - height;
    params.drawWindowFunc = draw;
    params.handleMouseClickFunc = mouse;
    params.handleKeyFunc = key;
    params.handleCursorFunc = cursor;
    params.handleMouseWheelFunc = wheel;
    params.refcon = this;
    params.decorateAsFloatingWindow = xplm_WindowDecorationRoundRectangle;
    params.layer = xplm_WindowLayerFloatingWindows;
    window_ = XPLMCreateWindowEx(&params);
    XPLMSetWindowTitle(window_, "FF A350 AutoTaxi");
    XPLMSetWindowResizingLimits(window_, 900, 600, 1600, 1100);
}
TaxiUI::~TaxiUI() {
    if (window_)
        XPLMDestroyWindow(window_);
#if IBM
    if (fontTexture_ && wglGetCurrentContext()) {
        GLuint texture = static_cast<GLuint>(fontTexture_);
        glDeleteTextures(1, &texture);
    }
    if (signFontTexture_ && wglGetCurrentContext()) {
        GLuint texture = static_cast<GLuint>(signFontTexture_);
        glDeleteTextures(1, &texture);
    }
#endif
}
void TaxiUI::show() {
    XPLMSetWindowIsVisible(window_, 1);
    XPLMBringWindowToFront(window_);
}
bool TaxiUI::visible() const {
    return window_ && XPLMGetWindowIsVisible(window_);
}
void TaxiUI::setAirport(const Airport &a) {
    state_.airport = a;
    state_.choices = destinations(a);
    state_.availability.clear();
    state_.selected = -1;
    state_.scroll = 0;
    state_.route.reset();
    resetPanelTiming(state_);
    state_.clearance = false;
    state_.mapView = MapView::Airport;
    state_.preferredMapView.reset();
    fitAirport(state_);
    frameCache_.invalidate(true);
}
void TaxiUI::setStatus(const std::string &text) {
    state_.status = text;
    frameCache_.invalidate();
}
void TaxiUI::setBusy(bool busy, bool loading) {
    state_.busy = busy;
    state_.loading = loading;
    frameCache_.invalidate();
    if (busy || loading) {
        state_.optionsOpen = false;
        state_.searchFocus = false;
        state_.viaFocus = state_.pickVia = false;
        if (XPLMHasKeyboardFocus(window_))
            XPLMTakeKeyboardFocus(nullptr);
    }
}
void TaxiUI::setRoute(const Route &route) {
    const auto center = state_.mapCenter;
    const auto scale = state_.mapScale;
    const auto extent = state_.mapFitExtent;
    setPanelRoute(state_, route);
    if (state_.pickVia) {
        state_.mapCenter = center;
        state_.mapScale = scale;
        state_.mapFitExtent = extent;
    }
    frameCache_.invalidate();
}
void TaxiUI::clearRoute() {
    state_.route.reset();
    resetPanelTiming(state_);
    frameCache_.invalidate();
}
void TaxiUI::setTiming(RouteTiming timing, double elapsed) {
    setPanelTiming(state_, std::move(timing), elapsed);
    frameCache_.invalidate();
}
void TaxiUI::setAvailability(const std::unordered_map<std::string, Availability> &info) {
    state_.availability = info;
    frameCache_.invalidate();
}
void TaxiUI::setDestinationAvailability(const std::string &label, const Availability &availability) {
    state_.availability[label] = availability;
    frameCache_.invalidate();
}
void TaxiUI::setTelemetry(const AircraftState &s, const ControlOutput &out, double actual, bool waiting) {
    state_.aircraft = s;
    state_.output = out;
    state_.actualSteer = actual;
    state_.waitingPushback = waiting;
    state_.emergencyHeld = out.phase == TaxiPhase::Hold && state_.busy;
    if (waiting || state_.emergencyHeld)
        resetPanelTiming(state_);
    frameCache_.invalidate();
}
void TaxiUI::setSpeed(double knots) {
    state_.speedKnots = std::clamp(knots, 1.0, state_.controllerConfig.maxTaxiSpeed / .514444);
    frameCache_.invalidate();
}
void TaxiUI::setRouteOptions(const RouteOptions &options) {
    state_.routeOptions = options;
    state_.viaText.clear();
    for (const auto &token : options.via) {
        if (!state_.viaText.empty())
            state_.viaText += " ";
        state_.viaText += token;
    }
    frameCache_.invalidate();
}
void TaxiUI::applyRouteSettings() {
    try {
        state_.routeOptions.via = parseRouteVia(state_.viaText);
        state_.availability.clear();
        state_.route.reset();
        resetPanelTiming(state_);
        if (actions_.routeSettings)
            actions_.routeSettings(state_.routeOptions);
        preview();
    } catch (const std::exception &e) {
        setStatus(e.what());
    }
}
void TaxiUI::setControllerConfig(const ControllerConfig &config) {
    state_.controllerConfig = config;
    frameCache_.invalidate();
}
const PanelFrame &TaxiUI::frame() const {
    int l, t, r, b;
    XPLMGetWindowGeometry(window_, &l, &t, &r, &b);
    return frameCache_.get(state_, r - l, t - b, [this](const std::string &text) { return measure(text); },
                           [this](const std::string &text) { return measureSign(text); });
}
void TaxiUI::drawText(double x, double baseline, const std::string &text, Color color, TextStyle style) {
    if (style == TextStyle::AirportSign && signFont_.supports(text)) {
        const auto previousEnvironment = beginSignText();
        drawSignGlyphs(x, baseline, text, color);
        endText(previousEnvironment);
        return;
    }
    if (!font_.supports(text)) {
        float rgb[] = {color.r, color.g, color.b};
        auto label = text;
        XPLMDrawString(rgb, static_cast<int>(x), static_cast<int>(baseline), label.data(), nullptr,
                       xplmFont_Proportional);
        XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);
        return;
    }
    const auto previousEnvironment = beginText();
    drawGlyphs(x, baseline, text, color);
    endText(previousEnvironment);
}
int TaxiUI::beginSignText() {
    XPLMSetGraphicsState(0, 1, 0, 0, 1, 0, 0);
    if (!signFontTexture_) {
        XPLMGenerateTextureNumbers(&signFontTexture_, 1);
        XPLMBindTexture2d(signFontTexture_, 0);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, AirportSignFont::width, AirportSignFont::height, 0, GL_RGBA,
                     GL_UNSIGNED_BYTE, signFont_.rgba.data());
    } else
        XPLMBindTexture2d(signFontTexture_, 0);
    GLint previousEnvironment = GL_MODULATE;
    glGetTexEnviv(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, &previousEnvironment);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    glBegin(GL_QUADS);
    return previousEnvironment;
}
void TaxiUI::drawSignGlyphs(double x, double baseline, const std::string &text, Color color) {
    glColor3f(color.r, color.g, color.b);
    x = std::round(x);
    const double top = std::round(baseline) + signFont_.ascent + 1;
    for (unsigned char character : text) {
        const int index = character - 32;
        const double u = static_cast<double>((index % 16) * AirportSignFont::cellWidth) /
                         AirportSignFont::width;
        const double v = static_cast<double>((index / 16) * AirportSignFont::cellHeight) /
                         AirportSignFont::height;
        const double u2 = u + static_cast<double>(AirportSignFont::cellWidth) / AirportSignFont::width;
        const double v2 = v + static_cast<double>(AirportSignFont::cellHeight) / AirportSignFont::height;
        glTexCoord2d(u, v);
        glVertex2d(x - 1, top);
        glTexCoord2d(u2, v);
        glVertex2d(x - 1 + AirportSignFont::cellWidth, top);
        glTexCoord2d(u2, v2);
        glVertex2d(x - 1 + AirportSignFont::cellWidth, top - AirportSignFont::cellHeight);
        glTexCoord2d(u, v2);
        glVertex2d(x - 1, top - AirportSignFont::cellHeight);
        x += signFont_.advances[index];
    }
}
int TaxiUI::beginText() {
    XPLMSetGraphicsState(0, 1, 0, 0, 1, 0, 0);
    if (!fontTexture_) {
        XPLMGenerateTextureNumbers(&fontTexture_, 1);
        XPLMBindTexture2d(fontTexture_, 0);
        // The atlas has fixed 16x20 glyph cells. Linear filtering samples the
        // neighbouring glyphs and makes small Consolas text visibly blurry.
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, ConsolasFont::width, ConsolasFont::height, 0, GL_RGBA,
                     GL_UNSIGNED_BYTE, font_.rgba.data());
    } else
        XPLMBindTexture2d(fontTexture_, 0);
    GLint previousEnvironment = GL_MODULATE;
    glGetTexEnviv(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, &previousEnvironment);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    glBegin(GL_QUADS);
    return previousEnvironment;
}
void TaxiUI::drawGlyphs(double x, double baseline, const std::string &text, Color color) {
    glColor3f(color.r, color.g, color.b);
    x = std::round(x);
    double top = std::round(baseline) + font_.ascent + 1;
    for (unsigned char character : text) {
        int index = character - 32;
        double u = static_cast<double>((index % 16) * ConsolasFont::cellWidth) / ConsolasFont::width;
        double v = static_cast<double>((index / 16) * ConsolasFont::cellHeight) / ConsolasFont::height;
        double u2 = u + static_cast<double>(ConsolasFont::cellWidth) / ConsolasFont::width;
        double v2 = v + static_cast<double>(ConsolasFont::cellHeight) / ConsolasFont::height;
        glTexCoord2d(u, v);
        glVertex2d(x - 1, top);
        glTexCoord2d(u2, v);
        glVertex2d(x - 1 + ConsolasFont::cellWidth, top);
        glTexCoord2d(u2, v2);
        glVertex2d(x - 1 + ConsolasFont::cellWidth, top - ConsolasFont::cellHeight);
        glTexCoord2d(u, v2);
        glVertex2d(x - 1, top - ConsolasFont::cellHeight);
        x += font_.advance;
    }
}
void TaxiUI::endText(int previousEnvironment) {
    glEnd();
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, previousEnvironment);
    XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);
}
void TaxiUI::preview() {
    if (state_.selected >= 0 && !state_.busy && !state_.loading)
        actions_.planOrStart(state_.choices.at(state_.selected), state_.clearance, state_.speedKnots, false);
}
void TaxiUI::act(Hit hit) {
    if (!hit.enabled)
        return;
    frameCache_.invalidate();
    switch (hit.action) {
    case Action::Runways:
    case Action::Ramps:
    case Action::Nodes:
        state_.category = static_cast<int>(hit.action) - static_cast<int>(Action::Runways);
        state_.scroll = 0;
        break;
    case Action::Search:
        state_.searchFocus = true;
        XPLMTakeKeyboardFocus(window_);
        return;
    case Action::Destination:
        state_.selected = hit.index;
        state_.route.reset();
        preview();
        break;
    case Action::SpeedDown:
        state_.speedKnots = std::max(1.0, state_.speedKnots - 1);
        state_.controllerConfig.taxiSpeed = state_.speedKnots * .514444;
        if (actions_.speedChanged)
            actions_.speedChanged(state_.speedKnots);
        break;
    case Action::SpeedUp:
        state_.speedKnots = std::min(state_.controllerConfig.maxTaxiSpeed / .514444, state_.speedKnots + 1);
        state_.controllerConfig.taxiSpeed = state_.speedKnots * .514444;
        if (actions_.speedChanged)
            actions_.speedChanged(state_.speedKnots);
        break;
    case Action::RouteOptions:
        state_.optionsOpen = !state_.optionsOpen;
        state_.pickVia = false;
        break;
    case Action::AllowOversteer:
        state_.routeOptions.allowOversteer = !state_.routeOptions.allowOversteer;
        applyRouteSettings();
        break;
    case Action::RelaxPavement:
        state_.routeOptions.ignorePavementLimits = !state_.routeOptions.ignorePavementLimits;
        applyRouteSettings();
        break;
    case Action::RelaxStand:
        state_.routeOptions.ignoreStandSize = !state_.routeOptions.ignoreStandSize;
        applyRouteSettings();
        break;
    case Action::ToggleStandLabels:
        state_.showStandLabels = !state_.showStandLabels;
        frameCache_.invalidate(true);
        break;
    case Action::ToggleAirportSigns:
        state_.showAirportSigns = !state_.showAirportSigns;
        frameCache_.invalidate(true);
        break;
    case Action::GoNow:
        if (actions_.goNow)
            actions_.goNow();
        break;
    case Action::ViaInput:
        state_.searchFocus = false;
        state_.viaFocus = true;
        XPLMTakeKeyboardFocus(window_);
        return;
    case Action::ApplyVia:
        applyRouteSettings();
        state_.optionsOpen = false;
        break;
    case Action::ClearVia:
        state_.viaText.clear();
        applyRouteSettings();
        break;
    case Action::UndoVia: {
        auto last = state_.viaText.find_last_not_of(" \t,;>");
        if (last == std::string::npos)
            state_.viaText.clear();
        else {
            auto separator = state_.viaText.find_last_of(" \t,;>", last);
            state_.viaText = separator == std::string::npos ? "" : state_.viaText.substr(0, separator);
        }
        applyRouteSettings();
        break;
    }
    case Action::PickVia:
        state_.pickVia = !state_.pickVia;
        state_.optionsOpen = !state_.pickVia;
        break;
    case Action::EmergencyBrake:
        if (actions_.emergencyBrake)
            actions_.emergencyBrake();
        state_.emergencyHeld = !state_.emergencyHeld;
        break;
    case Action::Clearance:
        state_.clearance = !state_.clearance;
        preview();
        break;
    case Action::Preview:
        preview();
        break;
    case Action::Start:
        if (state_.selected >= 0)
            actions_.planOrStart(state_.choices.at(state_.selected), state_.clearance, state_.speedKnots,
                                 true);
        break;
    case Action::Stop:
        actions_.stop();
        break;
    case Action::Manual:
        actions_.disconnect();
        break;
    case Action::Refresh:
        actions_.scan();
        break;
    case Action::Reload:
        actions_.reload();
        break;
    case Action::Fit:
        fitMap(state_);
        break;
    case Action::LocalMap:
        selectMapView(state_, MapView::Local);
        break;
    case Action::AirportMap:
        selectMapView(state_, MapView::Airport);
        break;
    case Action::ZoomIn:
        state_.mapScale = std::min(4.0, frame().mapScale * 1.3);
        break;
    case Action::ZoomOut:
        state_.mapScale = std::max(0.008, frame().mapScale / 1.3);
        break;
    default:
        break;
    }
    state_.searchFocus = false;
    state_.viaFocus = false;
    if (XPLMHasKeyboardFocus(window_))
        XPLMTakeKeyboardFocus(nullptr);
}
void TaxiUI::draw(XPLMWindowID window, void *ref) {
    auto *self = static_cast<TaxiUI *>(ref);
    const auto &frame = self->frame();
    int l, t, r, b;
    XPLMGetWindowGeometry(window, &l, &t, &r, &b);
    XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);
    auto point = [&](double x, double y) { glVertex2d(l + x, t - y); };
    bool staticMapDrawn = false;
    for (std::size_t i = 0; i < frame.commands.size();) {
        if (!staticMapDrawn && i == frame.staticMapBegin && frame.staticMapEnd > i)
            staticMapDrawn = self->mapRenderer_.draw(frame, self->frameCache_.mapRebuildCount(), l, t);
        // MapRenderer has already drawn cached line commands.  Keep walking
        // the range so text, triangles, and marker rectangles still go
        // through the normal UI renderer.
        if (staticMapDrawn && i >= frame.staticMapBegin && i < frame.staticMapEnd &&
            frame.commands[i].kind == DrawKind::Line) {
            ++i;
            continue;
        }
        const auto &d = frame.commands[i];
        const bool customText = d.kind == DrawKind::Text && d.textStyle == TextStyle::AirportSign &&
                                self->signFont_.supports(d.text);
        const bool uiText = d.kind == DrawKind::Text && d.textStyle == TextStyle::Ui &&
                            self->font_.supports(d.text);
        if (customText || uiText) {
            const auto previousEnvironment = customText ? self->beginSignText() : self->beginText();
            do {
                const auto &label = frame.commands[i++];
                if (customText)
                    self->drawSignGlyphs(l + label.box.x, t - label.box.y - 12, label.text, label.color);
                else
                    self->drawGlyphs(l + label.box.x, t - label.box.y - 12, label.text, label.color);
            } while (i < frame.commands.size() && frame.commands[i].kind == DrawKind::Text &&
                     frame.commands[i].textStyle == d.textStyle &&
                     (customText ? self->signFont_.supports(frame.commands[i].text)
                                 : self->font_.supports(frame.commands[i].text)));
            self->endText(previousEnvironment);
            continue;
        }
        glColor3f(d.color.r, d.color.g, d.color.b);
        if (d.kind == DrawKind::Line) {
            // Preserve painter order and per-segment colors while sharing GL setup.
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
        } else {
            self->drawText(l + d.box.x, t - d.box.y - 12, d.text, d.color, d.textStyle);
        }
        ++i;
    }
    int mx, my;
    XPLMGetMouseLocationGlobal(&mx, &my);
    double x = mx - l, y = t - my;
    for (const auto &hit : frame.hits)
        if (!hit.tooltip.empty() && hit.box.contains(x, y)) {
            double width = self->measure(hit.tooltip) + 18, tx = std::clamp(x + 10, 0.0, r - l - width),
                   ty = std::min(y + 25.0, t - b - 25.0);
            glColor3f(.06f, .06f, .06f);
            glBegin(GL_QUADS);
            point(tx, ty);
            point(tx + width, ty);
            point(tx + width, ty + 23);
            point(tx, ty + 23);
            glEnd();
            self->drawText(l + tx + 9, t - ty - 16, hit.tooltip, {.92f, .93f, .94f});
            break;
        }
}
int TaxiUI::mouse(XPLMWindowID window, int x, int y, XPLMMouseStatus status, void *ref) {
    auto *self = static_cast<TaxiUI *>(ref);
    int l, t, r, b;
    XPLMGetWindowGeometry(window, &l, &t, &r, &b);
    Vec2 p{static_cast<double>(x - l), static_cast<double>(t - y)};
    if (status == xplm_MouseUp) {
        self->dragging_ = false;
        return 1;
    }
    if (status == xplm_MouseDrag && self->dragging_) {
        const auto &frame = self->frame();
        Vec2 delta = p - self->dragStart_;
        self->state_.mapScale = frame.mapScale;
        self->state_.mapCenter = self->dragCenter_ + Vec2{-delta.x, delta.y} * (1 / frame.mapScale);
        self->frameCache_.invalidate();
        return 1;
    }
    if (status == xplm_MouseDown) {
        const auto &frame = self->frame();
        self->frameCache_.invalidate();
        for (auto it = frame.hits.rbegin(); it != frame.hits.rend(); ++it) {
            const auto &hit = *it;
            if (hit.action == Action::Map)
                continue;
            if (hit.box.contains(p.x, p.y)) {
                self->act(hit);
                return 1;
            }
        }
        if (frame.map.contains(p.x, p.y)) {
            if (self->state_.pickVia && !self->state_.busy && frame.mapContent.contains(p.x, p.y)) {
                Vec2 world =
                    self->state_.mapCenter + Vec2{p.x - frame.mapContent.x - frame.mapContent.w / 2,
                                                  -(p.y - frame.mapContent.y - frame.mapContent.h / 2)} *
                                                 (1 / frame.mapScale);
                double gap = 0;
                int node = nearestNode(self->state_.airport, unproject(self->state_.mapOrigin, world), &gap);
                if (node >= 0 && gap * frame.mapScale < 24) {
                    if (!self->state_.viaText.empty())
                        self->state_.viaText += " ";
                    self->state_.viaText += "#" + std::to_string(node);
                    self->applyRouteSettings();
                }
                return 1;
            }
            self->state_.searchFocus = self->state_.viaFocus = false;
            if (XPLMHasKeyboardFocus(window))
                XPLMTakeKeyboardFocus(nullptr);
            self->dragging_ = true;
            self->dragStart_ = p;
            self->dragCenter_ = self->state_.mapCenter;
            return 1;
        }
        self->state_.searchFocus = false;
        if (XPLMHasKeyboardFocus(window))
            XPLMTakeKeyboardFocus(nullptr);
    }
    return 1;
}
int TaxiUI::wheel(XPLMWindowID window, int x, int y, int, int clicks, void *ref) {
    auto *self = static_cast<TaxiUI *>(ref);
    int l, t, r, b;
    XPLMGetWindowGeometry(window, &l, &t, &r, &b);
    const auto &frame = self->frame();
    self->frameCache_.invalidate();
    if (frame.list.contains(x - l, t - y)) {
        auto choices = filteredChoices(self->state_);
        self->state_.scroll = std::clamp(self->state_.scroll - clicks * 2, 0,
                                         std::max(0, static_cast<int>(choices.size()) - frame.visibleRows));
    } else if (frame.map.contains(x - l, t - y))
        self->state_.mapScale = std::clamp(frame.mapScale * std::pow(1.15, clicks), 0.008, 4.0);
    return 1;
}
void TaxiUI::key(XPLMWindowID, char key, XPLMKeyFlags flags, char, void *ref, int losing) {
    auto *self = static_cast<TaxiUI *>(ref);
    self->frameCache_.invalidate();
    if (losing) {
        self->state_.searchFocus = false;
        self->state_.viaFocus = false;
        return;
    }
    if (!(flags & xplm_DownFlag) || (!self->state_.searchFocus && !self->state_.viaFocus) ||
        self->state_.busy || self->state_.loading)
        return;
    std::string &input = self->state_.viaFocus ? self->state_.viaText : self->state_.query;
    if (key == 8 && !input.empty())
        input.pop_back();
    else if (key == 27 || key == 13) {
        if (key == 13 && self->state_.viaFocus)
            self->applyRouteSettings();
        self->state_.searchFocus = false;
        self->state_.viaFocus = false;
        XPLMTakeKeyboardFocus(nullptr);
    } else if (static_cast<unsigned char>(key) >= 32 && static_cast<unsigned char>(key) < 127 &&
               input.size() < (self->state_.viaFocus ? 240 : 60))
        input += key;
    self->state_.scroll = 0;
}
XPLMCursorStatus TaxiUI::cursor(XPLMWindowID, int, int, void *) {
    return xplm_CursorDefault;
}
} // namespace autotaxi
