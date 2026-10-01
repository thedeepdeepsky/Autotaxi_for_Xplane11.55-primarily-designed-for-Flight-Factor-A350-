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
namespace autotaxi {
double TaxiUI::measure(const std::string &text) const {
    if (font_.supports(text))
        return font_.measure(text);
    return XPLMMeasureString(xplmFont_Proportional, text.data(), static_cast<int>(text.size()));
}
TaxiUI::TaxiUI(UIActions actions) : actions_(std::move(actions)) {
    if (font_.load())
        XPLMDebugString("[A350AutoTaxi] UI font: Consolas 14 px\n");
    else
        XPLMDebugString("[A350AutoTaxi] Consolas unavailable; using X-Plane UI font\n");
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
#endif
}
void TaxiUI::show() {
    XPLMSetWindowIsVisible(window_, 1);
    XPLMBringWindowToFront(window_);
}
void TaxiUI::setAirport(const Airport &a) {
    state_.airport = a;
    state_.choices = destinations(a);
    state_.availability.clear();
    state_.selected = -1;
    state_.scroll = 0;
    state_.route.reset();
    state_.clearance = false;
    state_.mapView = MapView::Airport;
    state_.preferredMapView.reset();
    fitAirport(state_);
}
void TaxiUI::setStatus(const std::string &text) {
    state_.status = text;
}
void TaxiUI::setBusy(bool busy, bool loading) {
    state_.busy = busy;
    state_.loading = loading;
    if (busy || loading) {
        state_.searchFocus = false;
        if (XPLMHasKeyboardFocus(window_))
            XPLMTakeKeyboardFocus(nullptr);
    }
}
void TaxiUI::setRoute(const Route &route) {
    setPanelRoute(state_, route);
}
void TaxiUI::clearRoute() {
    state_.route.reset();
}
void TaxiUI::setAvailability(const std::unordered_map<std::string, Availability> &info) {
    state_.availability = info;
}
void TaxiUI::setTelemetry(const AircraftState &s, const ControlOutput &out, double actual, bool waiting) {
    state_.aircraft = s;
    state_.output = out;
    state_.actualSteer = actual;
    state_.waitingPushback = waiting;
}
void TaxiUI::setSpeed(double knots) {
    state_.speedKnots = std::clamp(knots, 2.0, 20.0);
}
PanelFrame TaxiUI::frame() const {
    int l, t, r, b;
    XPLMGetWindowGeometry(window_, &l, &t, &r, &b);
    return buildPanel(state_, r - l, t - b, [this](const std::string &text) { return measure(text); });
}
void TaxiUI::drawText(double x, double baseline, const std::string &text, Color color) {
    if (!font_.supports(text)) {
        float rgb[] = {color.r, color.g, color.b};
        auto label = text;
        XPLMDrawString(rgb, static_cast<int>(x), static_cast<int>(baseline), label.data(), nullptr,
                       xplmFont_Proportional);
        return;
    }
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
    glColor3f(color.r, color.g, color.b);
    x = std::round(x);
    double top = std::round(baseline) + font_.ascent + 1;
    glBegin(GL_QUADS);
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
    glEnd();
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, previousEnvironment);
    XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);
}
void TaxiUI::preview() {
    if (state_.selected >= 0 && !state_.busy && !state_.loading)
        actions_.planOrStart(state_.choices.at(state_.selected), state_.clearance, state_.speedKnots, false);
}
void TaxiUI::act(const Hit &hit) {
    if (!hit.enabled)
        return;
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
        state_.speedKnots = std::max(2.0, state_.speedKnots - 1);
        break;
    case Action::SpeedUp:
        state_.speedKnots = std::min(20.0, state_.speedKnots + 1);
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
    if (XPLMHasKeyboardFocus(window_))
        XPLMTakeKeyboardFocus(nullptr);
}
void TaxiUI::draw(XPLMWindowID window, void *ref) {
    auto *self = static_cast<TaxiUI *>(ref);
    auto frame = self->frame();
    int l, t, r, b;
    XPLMGetWindowGeometry(window, &l, &t, &r, &b);
    XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);
    auto point = [&](double x, double y) { glVertex2d(l + x, t - y); };
    for (const auto &d : frame.commands) {
        glColor3f(d.color.r, d.color.g, d.color.b);
        if (d.kind == DrawKind::Rectangle) {
            glBegin(GL_QUADS);
            point(d.box.x, d.box.y);
            point(d.box.x + d.box.w, d.box.y);
            point(d.box.x + d.box.w, d.box.y + d.box.h);
            point(d.box.x, d.box.y + d.box.h);
            glEnd();
        } else if (d.kind == DrawKind::Line) {
            glLineWidth(static_cast<float>(d.width));
            glBegin(GL_LINES);
            point(d.box.x, d.box.y);
            point(d.x2, d.y2);
            glEnd();
            glLineWidth(1);
        } else if (d.kind == DrawKind::Triangle) {
            glBegin(GL_TRIANGLES);
            point(d.box.x, d.box.y);
            point(d.x2, d.y2);
            point(d.x3, d.y3);
            glEnd();
        } else {
            self->drawText(l + d.box.x, t - d.box.y - 12, d.text, d.color);
        }
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
        auto frame = self->frame();
        Vec2 delta = p - self->dragStart_;
        self->state_.mapScale = frame.mapScale;
        self->state_.mapCenter = self->dragCenter_ + Vec2{-delta.x, delta.y} * (1 / frame.mapScale);
        return 1;
    }
    if (status == xplm_MouseDown) {
        auto frame = self->frame();
        for (const auto &hit : frame.hits)
            if (hit.box.contains(p.x, p.y)) {
                if (hit.action == Action::Map) {
                    self->state_.searchFocus = false;
                    if (XPLMHasKeyboardFocus(window))
                        XPLMTakeKeyboardFocus(nullptr);
                    self->dragging_ = true;
                    self->dragStart_ = p;
                    self->dragCenter_ = self->state_.mapCenter;
                } else
                    self->act(hit);
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
    auto frame = self->frame();
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
    if (losing) {
        self->state_.searchFocus = false;
        return;
    }
    if (!(flags & xplm_DownFlag) || !self->state_.searchFocus || self->state_.busy || self->state_.loading)
        return;
    if (key == 8 && !self->state_.query.empty())
        self->state_.query.pop_back();
    else if (key == 27 || key == 13) {
        self->state_.searchFocus = false;
        XPLMTakeKeyboardFocus(nullptr);
    } else if (static_cast<unsigned char>(key) >= 32 && static_cast<unsigned char>(key) < 127 &&
               self->state_.query.size() < 60)
        self->state_.query += key;
    self->state_.scroll = 0;
}
XPLMCursorStatus TaxiUI::cursor(XPLMWindowID, int, int, void *) {
    return xplm_CursorDefault;
}
} // namespace autotaxi
