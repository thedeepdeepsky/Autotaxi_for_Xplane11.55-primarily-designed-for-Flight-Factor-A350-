#include "Panel.h"
#include "RouteTiming.h"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>
namespace autotaxi {
const PanelFrame &PanelFrameCache::get(const PanelState &state, double width, double height,
                                       const MeasureText &measure, MeasureText signMeasure) {
    if (dirty_ || width != width_ || height != height_) {
        frame_ = buildPanel(state, width, height, measure, &mapCache_, std::move(signMeasure));
        width_ = width;
        height_ = height;
        dirty_ = false;
        ++rebuildCount_;
    }
    return frame_;
}
std::size_t lineBatchEnd(const std::vector<DrawCommand> &commands, std::size_t begin) {
    std::size_t end = begin + 1;
    while (end < commands.size() && commands[end].kind == DrawKind::Line &&
           commands[end].width == commands[begin].width)
        ++end;
    return end;
}
namespace {
constexpr Color background{0.085f, 0.089f, 0.095f}, surface{0.12f, 0.125f, 0.135f},
    lineColor{0.24f, 0.25f, 0.27f};
constexpr Color textColor{0.92f, 0.93f, 0.94f}, muted{0.58f, 0.62f, 0.65f}, accent{0.27f, 0.85f, 0.71f},
    amber{1.f, 0.72f, 0.30f}, purple{0.74f, 0.53f, 1.f}, red{0.96f, 0.35f, 0.34f};
std::string upper(std::string s) {
    for (char &c : s)
        c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return s;
}
std::string displaySignText(const std::string &raw) {
    std::string out;
    for (std::size_t i = 0; i < raw.size();) {
        if (raw[i] == '{') {
            const auto end = raw.find('}', i + 1);
            if (end != std::string::npos) {
                const std::string code = upper(raw.substr(i + 1, end - (i + 1)));
                if (code.find('L') != std::string::npos)
                    out += '<';
                else if (code.find('R') != std::string::npos)
                    out += '>';
                else if (code.find('U') != std::string::npos)
                    out += '^';
                else if (code.find('D') != std::string::npos)
                    out += 'v';
                i = end + 1;
                continue;
            }
        }
        const unsigned char c = static_cast<unsigned char>(raw[i++]);
        if (c >= 32 && c <= 126) {
            // apt.dat uses `|` for a panel separator.  A few commercial
            // scenery exporters write `/` instead; render the same ICAO
            // separator in both cases.  This is deliberately local to
            // airport-sign text so UI labels such as "CMD / ACT" keep their
            // normal slash punctuation.
            const char normalized = c == '/' ? '|' : static_cast<char>(c);
            // Airport marking boards use capitals; keeping this normalization
            // here also guarantees the dedicated sign font has a glyph for
            // every letter present on the board.
            out += static_cast<char>(std::toupper(static_cast<unsigned char>(normalized)));
        }
    }
    return out;
}
std::string decimal(double n, int digits = 1) {
    std::ostringstream s;
    s << std::fixed << std::setprecision(digits) << n;
    return s.str();
}
std::string eta(double seconds) {
    if (!std::isfinite(seconds) || seconds < 0)
        return "--:--";
    int total = std::clamp(static_cast<int>(std::lround(seconds)), 0, 99 * 60 + 59);
    std::ostringstream out;
    out << std::setfill('0') << std::setw(2) << total / 60 << ':' << std::setw(2) << total % 60;
    return out.str();
}
double plannedCrossTrack(const std::vector<RoutePathPoint> &path, Vec2 point, double progress) {
    if (path.size() < 2)
        return -1;
    auto start = std::lower_bound(path.begin(), path.end(), progress - 40,
                                  [](const RoutePathPoint &p, double at) { return p.distance < at; });
    if (start != path.begin())
        --start;
    double best = std::numeric_limits<double>::infinity();
    for (auto next = start + 1; next != path.end(); ++start, ++next) {
        best = std::min(best, onSegment(point, start->position, next->position).distance);
        if (next->distance >= progress + 40)
            break;
    }
    return std::isfinite(best) ? best : -1;
}
struct NavigationInfo {
    double progress = 0, nextTurnDistance = -1, nextTurnAngle = 0;
    std::string node = "--", taxiway = "--";
    double nodeEta = -1, taxiwayEta = -1, totalEta = -1, taxiwayRemaining = -1;
};
NavigationInfo navigation(const PanelState &s) {
    NavigationInfo info;
    if (!s.route || s.route->points.size() < 2)
        return info;
    const auto &route = *s.route;
    info.progress = s.busy && !s.waitingPushback ? s.output.progress : 0;
    if (!s.busy && !route.requiresPushback && !s.waitingPushback) {
        Vec2 aircraft = trackingPosition(route, s.controllerConfig, s.aircraft);
        double cumulative = 0, best = std::numeric_limits<double>::infinity();
        for (std::size_t i = 1; i < route.points.size(); ++i) {
            auto snap = onSegment(aircraft, route.points[i - 1], route.points[i]);
            if (snap.distance < best) {
                best = snap.distance;
                info.progress = cumulative + length(route.points[i] - route.points[i - 1]) * snap.t;
            }
            cumulative += length(route.points[i] - route.points[i - 1]);
        }
    }
    const auto *timing =
        s.timing && !route.requiresPushback && !s.waitingPushback && s.output.phase != TaxiPhase::Hold
            ? &*s.timing
            : nullptr;
    const double elapsed = timing ? timing->secondsTo(info.progress) : -1;
    auto secondsTo = [&](double at) {
        return timing && elapsed >= 0 ? std::max(0., timing->secondsTo(at) - elapsed) : -1.;
    };
    double distanceAt = 0;
    for (std::size_t i = 1; i + 1 < route.points.size(); ++i) {
        double before = length(route.points[i] - route.points[i - 1]);
        distanceAt += before;
        double after = length(route.points[i + 1] - route.points[i]);
        if (distanceAt <= info.progress + 5 || before < 1 || after < 1)
            continue;
        double angle = std::abs(wrap180(heading(route.points[i + 1] - route.points[i]) -
                                        heading(route.points[i] - route.points[i - 1])));
        if (angle >= 12) {
            info.nextTurnDistance = distanceAt - info.progress;
            info.nextTurnAngle = angle;
            break;
        }
    }
    for (const auto &marker : route.nodesAlongRoute)
        if (marker.distance > info.progress + 3) {
            info.node = marker.label;
            info.nodeEta = secondsTo(marker.distance);
            break;
        }
    for (std::size_t i = 0; i < route.taxiwaysAlongRoute.size(); ++i) {
        const auto &marker = route.taxiwaysAlongRoute[i];
        const double nextStart = i + 1 < route.taxiwaysAlongRoute.size()
                                     ? route.taxiwaysAlongRoute[i + 1].distance
                                     : route.length;
        if (marker.distance <= info.progress + 5 && info.progress < nextStart + 5) {
            info.taxiway = marker.label;
            info.taxiwayRemaining = std::max(0., nextStart - info.progress);
            if (i + 1 < route.taxiwaysAlongRoute.size())
                info.taxiwayEta = secondsTo(nextStart);
            break;
        }
        if (marker.distance > info.progress + 5) {
            info.taxiway = marker.label;
            info.taxiwayRemaining = std::max(0., nextStart - marker.distance);
            info.taxiwayEta = secondsTo(marker.distance);
            break;
        }
    }
    info.totalEta = s.output.phase == TaxiPhase::Complete && info.progress >= route.length - 4 ? 0
                    : timing && timing->totalSeconds >= 0
                        ? std::max(0., timing->totalSeconds - std::max(0., elapsed))
                        : !route.requiresPushback && route.length > info.progress && s.controllerConfig.taxiSpeed > .1
                            ? (route.length - info.progress) / s.controllerConfig.taxiSpeed
                            : -1;
    return info;
}
struct Painter {
    PanelFrame frame;
    const MeasureText &measure;
    const MeasureText &signMeasure;
    void rect(Box b, Color c) {
        frame.commands.push_back({DrawKind::Rectangle, b, c, {}});
    }
    void line(double x, double y, double x2, double y2, Color c, double width = 1) {
        DrawCommand d{DrawKind::Line, {x, y, 0, 0}, c, {}};
        d.x2 = x2;
        d.y2 = y2;
        d.width = width;
        frame.commands.push_back(d);
    }
    void triangle(Vec2 a, Vec2 b, Vec2 c, Color color) {
        DrawCommand d{DrawKind::Triangle, {a.x, a.y, 0, 0}, color, {}};
        d.x2 = b.x;
        d.y2 = b.y;
        d.x3 = c.x;
        d.y3 = c.y;
        frame.commands.push_back(d);
    }
    void text(double x, double y, std::string label, Color c, double maxWidth = 10000,
              TextStyle style = TextStyle::Ui) {
        const auto &textMeasure = style == TextStyle::AirportSign ? signMeasure : measure;
        while (!label.empty() && textMeasure(label) > maxWidth) {
            label.pop_back();
            if (label.size() > 3)
                label.replace(label.size() - 3, 3, "...");
        }
        DrawCommand d{DrawKind::Text, {x, y, maxWidth, 16}, c, label};
        d.textStyle = style;
        frame.commands.push_back(std::move(d));
    }
    void button(Box b, std::string label, Action action, bool enabled = true, bool primary = false,
                bool danger = false, int index = -1) {
        Color color = primary ? accent : danger ? red : surface;
        rect(b, enabled ? color : surface);
        Color labelColor = enabled && (primary || danger) ? background : enabled ? textColor : muted;
        text(b.x + std::max(8.0, (b.w - measure(label)) / 2), b.y + (b.h - 14) / 2, label, labelColor,
             b.w - 16);
        frame.hits.push_back({action, b, index, enabled, {}});
    }
    void icon(Box b, Action action, const std::string &tooltip, bool enabled = true) {
        rect(b, surface);
        Color color = enabled ? textColor : muted;
        double cx = b.x + b.w / 2, cy = b.y + b.h / 2;
        if (action == Action::ZoomIn || action == Action::ZoomOut) {
            line(cx - 5, cy, cx + 5, cy, color, 1.5);
            if (action == Action::ZoomIn)
                line(cx, cy - 5, cx, cy + 5, color, 1.5);
        } else if (action == Action::Fit) {
            for (double sx : {-1., 1.})
                for (double sy : {-1., 1.}) {
                    line(cx + sx * 6, cy + sy * 6, cx + sx * 2, cy + sy * 6, color);
                    line(cx + sx * 6, cy + sy * 6, cx + sx * 6, cy + sy * 2, color);
                }
        } else if (action == Action::Reload) {
            for (int i = 0; i < 3; ++i) {
                double y = cy - 6 + i * 6;
                double knob = cx + (i == 1 ? 3 : -3);
                line(cx - 7, y, cx + 7, y, color);
                rect({knob - 1, y - 2, 2, 4}, color);
            }
        } else {
            line(cx - 5, cy - 4, cx + 4, cy - 4, color);
            line(cx + 4, cy - 4, cx + 4, cy + 5, color);
            line(cx + 4, cy + 5, cx - 5, cy + 5, color);
            line(cx - 5, cy + 5, cx - 5, cy + 1, color);
            triangle({cx - 8, cy + 2}, {cx - 2, cy + 2}, {cx - 5, cy - 2}, color);
        }
        frame.hits.push_back({action, b, -1, enabled, tooltip});
    }
};
} // namespace
std::vector<int> filteredChoices(const PanelState &s) {
    std::vector<int> out;
    auto query = upper(s.query);
    for (std::size_t i = 0; i < s.choices.size(); ++i) {
        const bool runwayTab = s.category == static_cast<int>(DestinationKind::Runway) &&
                               (s.choices[i].kind == DestinationKind::Runway ||
                                s.choices[i].kind == DestinationKind::HoldShort);
        if ((runwayTab || static_cast<int>(s.choices[i].kind) == s.category) &&
            upper(s.choices[i].label).find(query) != std::string::npos)
            out.push_back(static_cast<int>(i));
    }
    return out;
}
void fitAirport(PanelState &s) {
    s.mapScale = 0;
    s.mapOrigin = !s.airport.runways.empty() ? s.airport.runways[0].ends[0].position : s.aircraft.position;
    Vec2 lo{0, 0}, hi{0, 0};
    auto include = [&](GeoPoint p) {
        Vec2 q = project(s.mapOrigin, p);
        if (length(q) > 30000)
            return;
        lo.x = std::min(lo.x, q.x);
        lo.y = std::min(lo.y, q.y);
        hi.x = std::max(hi.x, q.x);
        hi.y = std::max(hi.y, q.y);
    };
    for (const auto &r : s.airport.runways)
        for (const auto &end : r.ends)
            include(end.position);
    for (const auto &n : s.airport.nodes)
        include(n.second.position);
    // Airport maps must be fitted to the rendered layers as well as the
    // routing graph.  Commercial apt.dat files often place stand lead-ins,
    // signs and apron polygons outside the taxi node envelope; omitting them
    // makes those lines clip across the viewport and hides their stand labels.
    for (const auto &r : s.airport.ramps)
        include(r.position);
    for (const auto &line : s.airport.groundLines)
        for (const auto &point : line.points)
            include(point);
    for (const auto &sign : s.airport.signs)
        include(sign.position);
    for (const auto &hold : s.airport.holdShortPoints)
        include(hold.position);
    for (const auto &surface : s.airport.pavements)
        for (const auto &ring : surface.rings)
            for (const auto &point : ring)
                include(point);
    for (const auto &surface : s.airport.sceneryContours)
        for (const auto &ring : surface.rings)
            for (const auto &point : ring)
                include(point);
    include(s.aircraft.position);
    s.mapCenter = (lo + hi) * 0.5;
    s.mapFitExtent = {std::max(500.0, (hi.x - lo.x) * .5), std::max(500.0, (hi.y - lo.y) * .5)};
}
void fitMap(PanelState &s) {
    if (s.mapView == MapView::Airport) {
        fitAirport(s);
        return;
    }
    s.mapScale = 0;
    s.mapOrigin = s.route && s.route->requiresPushback ? s.route->origin : s.aircraft.position;
    Vec2 lo = project(s.mapOrigin, s.aircraft.position), hi = lo;
    auto include = [&](Vec2 point) {
        lo.x = std::min(lo.x, point.x);
        lo.y = std::min(lo.y, point.y);
        hi.x = std::max(hi.x, point.x);
        hi.y = std::max(hi.y, point.y);
    };
    if (s.route && s.route->requiresPushback) {
        std::size_t count = s.route->pushbackPointCount ? s.route->pushbackPointCount : 2;
        count = std::min(count, s.route->points.size());
        include({});
        for (std::size_t i = 0; i < count; ++i)
            include(s.route->points[i]);
    }
    s.mapCenter = (lo + hi) * .5;
    s.mapFitExtent = {std::max(50.0, (hi.x - lo.x) * .5 + 20), std::max(50.0, (hi.y - lo.y) * .5 + 20)};
}
void selectMapView(PanelState &s, MapView view) {
    s.preferredMapView = view;
    s.mapView = view;
    fitMap(s);
}
void setPanelRoute(PanelState &s, const Route &route) {
    resetPanelTiming(s);
    bool changed =
        !s.route || s.route->label != route.label || s.route->requiresPushback != route.requiresPushback ||
        s.route->pushbackPointCount != route.pushbackPointCount ||
        s.route->points.size() != route.points.size() || distance(s.route->origin, route.origin) > 1;
    if (!changed)
        for (std::size_t i = 0; i < route.points.size(); ++i)
            if (length(s.route->points[i] - route.points[i]) > 1) {
                changed = true;
                break;
            }
    s.route = route;
    MapView view = s.preferredMapView.value_or(route.requiresPushback ? MapView::Local : MapView::Airport);
    if (changed || view != s.mapView) {
        s.mapView = view;
        fitMap(s);
    }
}
void resetPanelTiming(PanelState &s) {
    s.timing.reset();
    s.predictedCockpitPath.clear();
    s.predictionInvalidSeconds = 0;
}
void setPanelTiming(PanelState &s, RouteTiming timing, double elapsed) {
    const bool forward = s.route && !s.route->requiresPushback && !s.waitingPushback && !s.emergencyHeld &&
                         s.output.phase != TaxiPhase::Hold;
    std::vector<RoutePathPoint> predicted;
    if (forward && timing.totalSeconds >= 0)
        predicted = smoothCockpitPrediction(s.predictedCockpitPath, timing.cockpitPath, elapsed,
                                            s.output.brake > .05 || s.output.phase == TaxiPhase::Braking);
    if (!predicted.empty()) {
        s.predictedCockpitPath = std::move(predicted);
        s.predictionInvalidSeconds = 0;
    } else {
        s.predictionInvalidSeconds += std::isfinite(elapsed) ? std::max(0., elapsed) : 5;
        const bool temporaryFailure =
            forward && s.busy &&
            (s.output.phase == TaxiPhase::Taxi || s.output.phase == TaxiPhase::Align ||
             s.output.phase == TaxiPhase::Braking);
        if (!temporaryFailure)
            s.predictedCockpitPath.clear();
        else if (s.predictionInvalidSeconds >= 5 && !s.predictedCockpitPath.empty()) {
            std::vector<RoutePathPoint> fallback{
                {cockpitPosition(s.route->origin, s.controllerConfig, s.aircraft), s.output.progress, false}};
            for (const auto &point : s.route->cockpitPath)
                if (point.distance > s.output.progress)
                    fallback.push_back(point);
            s.predictedCockpitPath =
                smoothCockpitPrediction(s.predictedCockpitPath, std::move(fallback), elapsed, true);
        }
    }
    if (s.timing && s.busy && !s.waitingPushback)
        timing = stabilizeTiming(*s.timing, std::move(timing), elapsed);
    s.timing = std::move(timing);
}
PanelFrame buildPanel(const PanelState &s, double w, double h, const MeasureText &measure,
                      MapDrawCache *mapCache, MeasureText signMeasure) {
    const auto &effectiveSignMeasure = signMeasure ? signMeasure : measure;
    Painter p{{}, measure, effectiveSignMeasure};
    if (mapCache && mapCache->valid) {
        const auto routeCommands = s.route ? s.route->points.size() : 0;
        const auto nodeCommands = s.pickVia ? s.airport.nodes.size() : 0;
        p.frame.commands.reserve(mapCache->commands.size() + routeCommands + nodeCommands + 512);
    }
    p.rect({0, 0, w, h}, background);
    p.text(22, 20, "A350 AUTOTAXI", textColor);
    p.text(220, 20, s.airport.id.empty() ? "Airport detection" : s.airport.id + "   " + s.airport.name, muted,
           w - 345);
    p.icon({w - 96, 12, 30, 30}, Action::Refresh, "Detect airport", !s.busy && !s.loading);
    p.icon({w - 56, 12, 30, 30}, Action::Reload, "Reload settings", !s.busy && !s.loading);
    p.line(0, 52, w, 52, lineColor);
    auto nav = navigation(s);
    double cockpitCte = -1, axleCte = -1;
    const bool forwardRoute = s.route && !s.route->requiresPushback && !s.waitingPushback;
    if (forwardRoute && valid(s.aircraft.position)) {
        cockpitCte =
            plannedCrossTrack(s.route->cockpitPath,
                              cockpitPosition(s.route->origin, s.controllerConfig, s.aircraft), nav.progress);
        axleCte = plannedCrossTrack(s.route->mainAxlePath,
                                    mainAxlePosition(s.route->origin, s.controllerConfig, s.aircraft),
                                    nav.progress);
        if (s.busy)
            (s.route->cockpitGuidance ? cockpitCte : axleCte) = s.output.crossTrack;
    }
    const std::string labels[] = {"GROUND SPEED", "NOSEWHEEL CMD / ACT", "REMAINING", "COCKPIT CTE",
                                  "MAIN AXLE CTE"};
    const std::string values[] = {decimal(s.aircraft.speed / 0.514444) + " kt",
                                  decimal(s.output.steerDegrees) + " / " + decimal(s.actualSteer) + " deg",
                                  s.busy && !s.waitingPushback ? decimal(s.output.remaining, 0) + " m"
                                  : s.route                    ? decimal(s.route->length, 0) + " m"
                                                               : "--",
                                  cockpitCte >= 0 ? decimal(cockpitCte) + " m" : "--",
                                  axleCte >= 0 ? decimal(axleCte) + " m" : "--"};
    const double weights[] = {.16, .24, .16, .22, .22};
    double x = 22;
    for (int i = 0; i < 5; ++i) {
        const double column = (w - 44) * weights[i];
        const bool controlling = forwardRoute && i == (s.route->cockpitGuidance ? 3 : 4);
        p.text(x, 65, labels[i], controlling ? accent : muted, column - 12);
        p.text(x, 86, values[i], i == 0 || controlling ? accent : textColor, column - 12);
        x += column;
    }
    p.line(0, 115, w, 115, lineColor);
    double leftWidth = 280, footer = h - 145;
    p.text(22, 128, "FROM", muted);
    double nodeDistance = 0;
    int nodeId = nearestNode(s.airport, s.aircraft.position, &nodeDistance);
    std::string nodeLabel =
        nodeId >= 0 ? "Node " + std::to_string(nodeId) + " / " + decimal(nodeDistance, 0) + " m" : "";
    p.text(78, 128, departureLabel(s.airport, s.aircraft.position), textColor, leftWidth - 90);
    if (nodeId >= 0)
        p.text(22, 146, nodeLabel, muted, leftWidth - 35);
    if (s.route && !s.waitingPushback) {
        std::string turn = nav.nextTurnDistance >= 0 ? "NEXT TURN " + decimal(nav.nextTurnDistance, 0) +
                                                           " m / " + decimal(nav.nextTurnAngle, 0) + " deg"
                                                     : "NEXT TURN --";
        std::string node = "NEXT " + nav.node + " / " + eta(nav.nodeEta);
        std::string taxiway = "TAXIWAY " + nav.taxiway + " / " +
                              (nav.taxiwayRemaining >= 0 ? decimal(nav.taxiwayRemaining, 0) + " m" : "--") +
                              " / " + eta(nav.taxiwayEta);
        p.text(leftWidth + 30, 123, turn + "   " + node, accent, w - leftWidth - 52);
        p.text(leftWidth + 30, 141, taxiway + "   TOTAL " + eta(nav.totalEta), muted, w - leftWidth - 52);
    }
    p.line(leftWidth + 12, 158, leftWidth + 12, footer, lineColor);
    const char *tabs[] = {"Runways", "Stands", "Nodes"};
    for (int i = 0; i < 3; ++i) {
        Box b{18. + i * 88, 161, 84, 30};
        p.button(b, tabs[i], static_cast<Action>(static_cast<int>(Action::Runways) + i),
                 !s.busy && !s.loading);
        if (s.category == i)
            p.rect({b.x, b.y + b.h - 2, b.w, 2}, accent);
    }
    Box search{18, 203, 260, 34};
    p.rect(search, surface);
    p.text(29, 213,
           s.query.empty() && !s.searchFocus ? "Search destination" : s.query + (s.searchFocus ? "|" : ""),
           s.query.empty() ? muted : textColor, 238);
    p.frame.hits.push_back({Action::Search, search, -1, !s.busy && !s.loading, {}});
    Box list{18, 249, 260, std::max(40.0, footer - 257)};
    p.frame.list = list;
    auto filtered = filteredChoices(s);
    int rows = std::max(1, static_cast<int>(list.h / 47));
    p.frame.visibleRows = rows;
    int scroll = std::clamp(s.scroll, 0, std::max(0, static_cast<int>(filtered.size()) - rows));
    for (int row = 0; row < rows && row + scroll < static_cast<int>(filtered.size()); ++row) {
        int id = filtered[row + scroll];
        const auto &d = s.choices[id];
        char standWidth = d.kind == DestinationKind::Ramp ? s.airport.ramps.at(d.index).width : 0;
        bool fits =
            s.routeOptions.ignoreStandSize || !standWidth || standWidth >= s.routeOptions.minimumWidth;
        Box b{list.x, list.y + row * 47, list.w, 43};
        bool selected = id == s.selected;
        if (selected) {
            p.rect(b, surface);
            p.rect({b.x, b.y, 3, b.h}, accent);
        }
        p.text(b.x + 12, b.y + 5, d.label, selected ? textColor : muted, 235);
        auto entry = s.availability.find(d.label);
        std::string detail = d.kind == DestinationKind::HoldShort
                                 ? "Select hold-short point"
                                 : d.kind == DestinationKind::Runway ? "Route pending"
                                 : "";
        Color color = muted;
        if (!fits) {
            detail = "Class " + std::string(1, standWidth) + " / requires " +
                     std::string(1, s.routeOptions.minimumWidth);
            color = red;
        } else if (entry != s.availability.end()) {
            const auto &info = entry->second;
            detail = info.reachable ? (info.pushback ? "Pushback  /  " : "Available  /  ") +
                                          decimal(info.length / 1000, 2) + " km"
                     : info.reason.find("entry") != std::string::npos      ? "No runway entry"
                     : info.reason.find("runway-axis connection") != std::string::npos ? "No runway connection"
                     : info.reason.find("connection") != std::string::npos ? "No apron connection"
                                                                           : "No compatible route";
            color = info.reachable ? (info.pushback ? purple : accent) : red;
        }
        p.text(b.x + 12, b.y + 24, detail, color, 235);
        p.frame.hits.push_back({Action::Destination, b, id, !s.busy && !s.loading && fits,
                                fits ? "" : "Stand below configured aircraft width class"});
    }
    if (filtered.empty())
        p.text(30, list.y + 18, "No matching destinations", muted, 235);
    if (static_cast<int>(filtered.size()) > rows) {
        double thumb = std::max(20.0, list.h * rows / filtered.size());
        double y = list.y + (list.h - thumb) * scroll / std::max(1, static_cast<int>(filtered.size()) - rows);
        p.rect({280, y, 3, thumb}, muted);
    }
    Box map{leftWidth + 30, 160, w - leftWidth - 48, footer - 171};
    p.frame.map = map;
    p.rect(map, surface);
    Box content{map.x + 60, map.y + 44, map.w - 80, map.h - 84};
    p.frame.mapContent = content;
    double scale =
        s.mapScale > 0
            ? s.mapScale
            : std::clamp(std::min(content.w / (2 * s.mapFitExtent.x), content.h / (2 * s.mapFitExtent.y)),
                         .008, 4.0);
    p.frame.mapScale = scale;
    auto xy = [&](Vec2 point) {
        Vec2 q = (point - s.mapCenter) * scale;
        return Vec2{content.x + content.w / 2 + q.x, content.y + content.h / 2 - q.y};
    };
    auto geo = [&](GeoPoint point) {
        // apt.dat/DSF records occasionally contain a default 0,0 point or a
        // contour belonging to another tile.  Projecting it creates a very
        // long segment that clips through the map from its upper-left corner.
        // All airport geometry is comfortably inside this guard distance.
        if (!valid(point) || distance(s.mapOrigin, point) > 100000.0)
            return Vec2{std::numeric_limits<double>::quiet_NaN(),
                        std::numeric_limits<double>::quiet_NaN()};
        return xy(project(s.mapOrigin, point));
    };
        auto mapLine = [&](Vec2 a, Vec2 b, Color color, double width = 1) {
        // A malformed scenery point must never reach the renderer.  OpenGL
        // treats NaN/Inf line coordinates inconsistently; on some drivers the
        // segment is effectively anchored at the viewport origin, which
        // appears as a white thread from the panel's upper-left corner.
        if (!std::isfinite(a.x) || !std::isfinite(a.y) || !std::isfinite(b.x) || !std::isfinite(b.y) ||
            !std::isfinite(width) || width <= 0 || std::abs(a.x) > 1e7 || std::abs(a.y) > 1e7 ||
            std::abs(b.x) > 1e7 || std::abs(b.y) > 1e7)
            return;
        // Clip line segments to the map in layout coordinates, without changing global GL scissor state.
        double lo = 0, hi = 1;
        Vec2 delta = b - a;
        auto clip = [&](double u, double v) {
            if (std::abs(u) < 1e-9)
                return v >= 0;
            double t = v / u;
            if (u < 0)
                lo = std::max(lo, t);
            else
                hi = std::min(hi, t);
            return lo <= hi;
        };
        if (clip(-delta.x, a.x - content.x) && clip(delta.x, content.x + content.w - a.x) &&
            clip(-delta.y, a.y - content.y) && clip(delta.y, content.y + content.h - a.y)) {
            Vec2 start = a + delta * lo, end = a + delta * hi;
                p.line(start.x, start.y, end.x, end.y, color, width);
        }
    };
    auto leadInMarker = [&](Vec2 point, Vec2 tangent, double halfLength, double width, Color color) {
        if (length(tangent) < .01)
            return;
        tangent = tangent * (1 / length(tangent));
        Vec2 normal{-tangent.y, tangent.x};
        mapLine(point - normal * halfLength, point + normal * halfLength, color, width);
    };
    struct LeadInVisual {
        bool valid = false;
        std::size_t stopIndex = 0;
        std::size_t straightStart = 0;
        int step = -1;
    };
    auto leadInVisual = [&](const GroundLine &line) {
        LeadInVisual visual;
        if (!line.standLeadIn || !line.paintedCenterline() || line.points.size() < 2)
            return visual;
        visual.valid = true;
        if (line.standLeadInRamp >= 0 && line.standLeadInRamp < static_cast<int>(s.airport.ramps.size())) {
            const auto &ramp = s.airport.ramps[line.standLeadInRamp];
            const bool frontAtStop = distance(ramp.position, line.points.front()) <=
                                     distance(ramp.position, line.points.back());
            visual.stopIndex = frontAtStop ? 0 : line.points.size() - 1;
            visual.step = frontAtStop ? 1 : -1;
        }
        const std::size_t count = line.points.size();
        visual.straightStart = visual.stopIndex;
        const auto local = [&](std::size_t index) { return project(s.mapOrigin, line.points[index]); };
        std::size_t current = visual.stopIndex;
        std::size_t next = static_cast<std::size_t>(static_cast<int>(current) + visual.step);
        Vec2 terminal = local(next) - local(current);
        double straightLength = 0;
        while (next < count) {
            Vec2 segment = local(next) - local(current);
            if (length(segment) < .01)
                break;
            double turn = std::abs(wrap180(heading(segment) - heading(terminal)));
            turn = std::min(turn, 180.0 - turn);
            if (turn > 12.0 && straightLength >= 20.0)
                break;
            straightLength += length(segment);
            visual.straightStart = next;
            current = next;
            const int candidate = static_cast<int>(current) + visual.step;
            if (candidate < 0 || candidate >= static_cast<int>(count))
                break;
            next = static_cast<std::size_t>(candidate);
        }
        return visual;
    };
    bool reuseMap = mapCache && mapCache->valid && mapCache->airport == &s.airport &&
                    mapCache->origin.lat == s.mapOrigin.lat && mapCache->origin.lon == s.mapOrigin.lon &&
                    mapCache->center.x == s.mapCenter.x && mapCache->center.y == s.mapCenter.y &&
                    mapCache->view == s.mapView && mapCache->scale == scale &&
                    mapCache->content.x == content.x && mapCache->content.y == content.y &&
                    mapCache->content.w == content.w && mapCache->content.h == content.h;
    p.frame.staticMapBegin = p.frame.commands.size();
    if (reuseMap) {
        p.frame.commands.insert(p.frame.commands.end(), mapCache->commands.begin(), mapCache->commands.end());
    } else {
        std::size_t mapBegin = p.frame.commands.size();
        if (s.mapView == MapView::Local)
            for (const auto &pavement : s.airport.pavements)
                for (const auto &ring : pavement.rings)
                    for (std::size_t i = 0; i < ring.size(); ++i)
                        mapLine(geo(ring[i]), geo(ring[(i + 1) % ring.size()]), {0.21f, 0.23f, 0.25f});
        if (s.mapView == MapView::Local)
            for (const auto &outline : s.airport.sceneryContours)
                for (const auto &ring : outline.rings)
                    for (std::size_t i = 0; i < ring.size(); ++i)
                        mapLine(geo(ring[i]), geo(ring[(i + 1) % ring.size()]), {.22f, .25f, .27f});
        // Runway pavement edges are derived from row 100's surveyed axis and
        // width.  This remains available even when the scenery package omits
        // explicit edge polygons, and is deliberately heavier than taxiway
        // network lines so the runway limits are unambiguous.
        for (const auto &runway : s.airport.runways) {
            Vec2 a = project(s.mapOrigin, runway.ends[0].position);
            Vec2 b = project(s.mapOrigin, runway.ends[1].position);
            Vec2 axis = b - a;
            const double span = length(axis);
            if (span < 1)
                continue;
            axis = axis * (1 / span);
            Vec2 normal{-axis.y, axis.x};
            const double half = runway.width * .5;
            mapLine(geo(unproject(s.mapOrigin, a + normal * half)),
                    geo(unproject(s.mapOrigin, b + normal * half)), {.86f, .87f, .88f}, 3.5);
            mapLine(geo(unproject(s.mapOrigin, a - normal * half)),
                    geo(unproject(s.mapOrigin, b - normal * half)), {.86f, .87f, .88f}, 3.5);
        }
        for (const auto &line : s.airport.groundLines) {
            const LeadInVisual visual = leadInVisual(line);
            for (std::size_t i = 1; i < line.points.size(); ++i) {
                bool highlighted = false;
                if (visual.valid) {
                    if (visual.step > 0)
                        highlighted = i > visual.stopIndex && i <= visual.straightStart;
                    else
                        highlighted = i >= visual.straightStart && i < visual.stopIndex;
                }
                mapLine(geo(line.points[i - 1]), geo(line.points[i]),
                        line.paintedCenterline() ? Color{.95f, .68f, .16f}
                                                 : line.standLeadIn ? Color{.92f, .93f, .95f}
                                                                    : line.whiteMarking() ? Color{.74f, .77f, .80f}
                                                                                          : Color{.34f, .36f, .39f},
                        highlighted ? 1.9
                                    : line.standLeadIn ? (line.paintedCenterline() ? 1.35 : 2.0)
                                         : line.paintedCenterline() ? 1.35
                                                                     : line.whiteMarking() ? 1.2 : 1.15);
            }
        }
        for (const auto &line : s.airport.groundLines) {
            if (!line.standLeadIn || line.points.size() < 2)
                continue;
            Vec2 stopLocal = project(s.mapOrigin, line.points.back());
            Vec2 stopTangent = stopLocal - project(s.mapOrigin, line.points[line.points.size() - 2]);
            Vec2 first = geo(line.points.front()), firstNext = geo(line.points[1]);
            const LeadInVisual visual = leadInVisual(line);
            if (line.standLeadInRamp >= 0 && line.standLeadInRamp < static_cast<int>(s.airport.ramps.size())) {
                const Vec2 ramp = project(s.mapOrigin, s.airport.ramps[line.standLeadInRamp].position);
                const bool frontAtStop = distance(s.airport.ramps[line.standLeadInRamp].position,
                                                  line.points.front()) <=
                                         distance(s.airport.ramps[line.standLeadInRamp].position,
                                                  line.points.back());
                const std::size_t startIndex = visual.valid ? visual.straightStart
                                                             : (frontAtStop ? line.points.size() - 1 : 0);
                const int next = static_cast<int>(startIndex) + (visual.valid ? visual.step : (frontAtStop ? -1 : 1));
                const std::size_t nextIndex = static_cast<std::size_t>(std::clamp(next, 0,
                                                                                  static_cast<int>(line.points.size() - 1)));
                first = geo(line.points[startIndex]);
                firstNext = geo(line.points[nextIndex]);
                double best = 1e30;
                for (std::size_t i = 1; i < line.points.size(); ++i) {
                    Vec2 a = project(s.mapOrigin, line.points[i - 1]);
                    Vec2 b = project(s.mapOrigin, line.points[i]);
                    auto snap = onSegment(ramp, a, b);
                    if (snap.distance < best) {
                        best = snap.distance;
                        stopLocal = snap.point;
                        stopTangent = b - a;
                    }
                }
            }
            leadInMarker(first, firstNext - first, 5.0, 3.0, {.98f, .73f, .08f});
            bool existingStop = false;
            for (const auto &mark : s.airport.groundLines) {
                if (mark.style != 4 && mark.style != 5 && mark.style != 6 && mark.style != 103 && mark.style != 104)
                    continue;
                for (std::size_t i = 1; i < mark.points.size(); ++i) {
                    Vec2 a = project(s.mapOrigin, mark.points[i - 1]);
                    Vec2 b = project(s.mapOrigin, mark.points[i]);
                    auto snap = onSegment(stopLocal, a, b);
                    if (snap.distance <= 4 && length(b - a) > .01 &&
                        std::abs(dot((b - a) * (1 / length(b - a)),
                                     stopTangent * (1 / std::max(.01, length(stopTangent))))) < .35) {
                        existingStop = true;
                        break;
                    }
                }
                if (existingStop)
                    break;
            }
            if (!existingStop)
                leadInMarker(xy(stopLocal), stopTangent, 8.0, 3.8, {.99f, .82f, .10f});
        }
        for (const auto &edge : s.airport.edges) {
            if (edge.painted)
                continue;
            Vec2 a = geo(s.airport.nodes.at(edge.from).position),
                 b = geo(s.airport.nodes.at(edge.to).position);
            if (!edge.runway && !s.airport.groundLines.empty()) {
                Vec2 delta = b - a;
                double extent = length(delta);
                if (extent > 0)
                    for (double along = 0; along < extent; along += 10)
                        mapLine(a + delta * (along / extent),
                                a + delta * (std::min(along + 4, extent) / extent), {.25f, .28f, .30f});
                continue;
            }
            mapLine(a, b, edge.runway ? Color{0.48f, 0.49f, 0.50f} : Color{0.29f, 0.32f, 0.34f},
                    edge.runway ? 3 : 1);
            if (edge.runway && edge.oneWay) {
                Vec2 delta = b - a;
                double span = length(delta);
                if (span > 1) {
                    Vec2 unit = delta * (1 / span), normal{-unit.y, unit.x};
                    // Reverse travel is prohibited at the terminal end.
                    mapLine(b - normal * 12, b + normal * 12, {.95f, .12f, .12f}, 4);
                    Vec2 tip = a + delta * .58;
                    Vec2 base = tip - unit * 8;
                    p.triangle(tip, base + normal * 4, base - normal * 4, {.95f, .30f, .25f});
                }
            }
        }
        // Taxiway/runway signs are display-only scenery annotations.  Their
        // text is intentionally gated by zoom so an airport overview remains
        // legible while a local view can show the complete arrow/sign label.
        for (const auto &sign : s.airport.signs) {
            if (!s.showAirportSigns)
                break;
            Vec2 point = geo(sign.position);
            if (!content.contains(point.x - 4, point.y - 4) || !content.contains(point.x + 4, point.y + 4))
                continue;
            Vec2 dir = direction(sign.heading);
            mapLine(point - Vec2{dir.y, -dir.x} * 4, point + Vec2{dir.y, -dir.x} * 4,
                    sign.noEntry ? Color{.96f, .12f, .12f}
                                 : sign.runway ? Color{.95f, .25f, .22f} : Color{.88f, .90f, .92f},
                    scale >= .25 ? 2.4 : 1.5);
            const std::string label = displaySignText(sign.text);
            if (scale >= .25 && (label.find('<') != std::string::npos || label.find('>') != std::string::npos ||
                                 label.find('^') != std::string::npos || label.find('v') != std::string::npos)) {
                Vec2 tip = point + dir * 7, base = point + dir * 1;
                Vec2 normal{-dir.y, dir.x};
                p.triangle(tip, base + normal * 3.5, base - normal * 3.5,
                           sign.noEntry ? Color{.98f, .20f, .20f} : Color{.88f, .90f, .92f});
            }
            if (scale >= .25 && !label.empty()) {
                const double width = effectiveSignMeasure(label);
                const double x = std::clamp(point.x + 7, content.x + 2, content.x + content.w - width - 2);
                const double y = std::clamp(point.y - 17, content.y + 2, content.y + content.h - 17);
                const bool yellowText = sign.text.find("{@Y") != std::string::npos ||
                                        sign.text.find("{@Y,") != std::string::npos;
                const Color board = sign.noEntry       ? Color{.34f, .06f, .07f}
                                    : sign.runway       ? Color{.46f, .07f, .08f}
                                    : Color{.035f, .055f, .065f};
                const Color foreground = sign.noEntry || sign.runway
                                             ? Color{.98f, .94f, .86f}
                                             : yellowText ? Color{1.f, .78f, .18f} : Color{.92f, .94f, .96f};
                const Box boardBox{x - 5, y - 3, width + 10, 21};
                p.rect(boardBox, board);
                p.line(boardBox.x, boardBox.y, boardBox.x + boardBox.w, boardBox.y, foreground, 1);
                p.line(boardBox.x, boardBox.y + boardBox.h, boardBox.x + boardBox.w,
                       boardBox.y + boardBox.h, foreground, 1);
                p.text(x, y, label, foreground, width + 1, TextStyle::AirportSign);
            }
        }
        for (const auto &hold : s.airport.holdShortPoints) {
            Vec2 point = geo(hold.position);
            if (!content.contains(point.x - 5, point.y - 5) || !content.contains(point.x + 5, point.y + 5))
                continue;
            Vec2 dir = direction(hold.heading);
            Vec2 normal{-dir.y, dir.x};
            mapLine(point - normal * 7, point + normal * 7, {.98f, .74f, .16f}, 3.2);
            if (scale >= .25 && !hold.label.empty()) {
                const double width = measure(hold.label);
                const double x = std::clamp(point.x + 8, content.x + 2, content.x + content.w - width - 2);
                const double y = std::clamp(point.y + 7, content.y + 2, content.y + content.h - 17);
                p.rect({x - 2, y - 1, width + 4, 17}, background);
                p.text(x, y, hold.label, {.98f, .78f, .28f}, width + 1);
            }
        }
        // Keep every stand visible on the map even when it is not currently
        // selected.  Markers stay in the cached airport layer, while names
        // appear only after zooming in far enough to remain readable.
        std::vector<Box> standLabels;
        int selectedRamp = -1;
        if (s.selected >= 0 && s.selected < static_cast<int>(s.choices.size()) &&
            s.choices[s.selected].kind == DestinationKind::Ramp)
            selectedRamp = s.choices[s.selected].index;
        const bool showStandNames = s.showStandLabels && scale >= .12;
        const bool showAllStandNames = scale >= .50;
        for (std::size_t rampIndex = 0; rampIndex < s.airport.ramps.size(); ++rampIndex) {
            const auto &ramp = s.airport.ramps[rampIndex];
            const Vec2 point = geo(ramp.position);
            if (!content.contains(point.x - 5, point.y - 5) || !content.contains(point.x + 5, point.y + 5))
                continue;
            const bool selected = static_cast<int>(rampIndex) == selectedRamp;
            const Color marker = selected ? amber : Color{.62f, .68f, .71f};
            const double half = selected ? 4.0 : 2.5;
            p.rect({point.x - half, point.y - half, half * 2, half * 2}, background);
            p.rect({point.x - (selected ? 2.0 : 1.25), point.y - (selected ? 2.0 : 1.25),
                    selected ? 4.0 : 2.5, selected ? 4.0 : 2.5}, marker);
            if (!showStandNames || ramp.name.empty())
                continue;
            const double labelWidth = measure(ramp.name);
            Box label{std::clamp(point.x + 7, content.x + 3, content.x + content.w - labelWidth - 3),
                      std::clamp(point.y - 17, content.y + 3, content.y + content.h - 17), labelWidth, 16};
            bool collision = false;
            for (const auto &other : standLabels)
                if (label.x < other.x + other.w + 2 && label.x + label.w + 2 > other.x &&
                    label.y < other.y + other.h + 2 && label.y + label.h + 2 > other.y) {
                    collision = true;
                    break;
                }
            if (collision && !selected && !showAllStandNames)
                continue;
            standLabels.push_back(label);
            p.rect({label.x - 2, label.y - 1, label.w + 4, label.h + 2}, background);
            p.text(label.x, label.y, ramp.name, selected ? amber : textColor, label.w + 1);
        }
        if (mapCache) {
            mapCache->commands.assign(p.frame.commands.begin() + mapBegin, p.frame.commands.end());
            mapCache->airport = &s.airport;
            mapCache->origin = s.mapOrigin;
            mapCache->center = s.mapCenter;
            mapCache->content = content;
            mapCache->scale = scale;
            mapCache->view = s.mapView;
            mapCache->valid = true;
            ++mapCache->rebuildCount;
        }
    }
    p.frame.staticMapEnd = p.frame.commands.size();
    // Stand markers are interactive targets as well as visual labels.  Keep
    // these hit regions outside the cached drawing layer so selection remains
    // responsive while the expensive airport geometry stays cached.
    for (std::size_t rampIndex = 0; rampIndex < s.airport.ramps.size(); ++rampIndex) {
        const auto &ramp = s.airport.ramps[rampIndex];
        const Vec2 point = geo(ramp.position);
        if (!content.contains(point.x - 12, point.y - 12) || !content.contains(point.x + 12, point.y + 12))
            continue;
        auto choice = std::find_if(s.choices.begin(), s.choices.end(), [&](const Destination &d) {
            return d.kind == DestinationKind::Ramp && d.index == static_cast<int>(rampIndex);
        });
        if (choice == s.choices.end())
            continue;
        const bool fits = s.routeOptions.ignoreStandSize || !ramp.width || ramp.width >= s.routeOptions.minimumWidth;
        p.frame.hits.push_back({Action::Destination, {point.x - 12, point.y - 12, 24, 24},
                                static_cast<int>(choice - s.choices.begin()),
                                !s.busy && !s.loading && fits,
                                fits ? "Select " + rampLabel(ramp) : "Stand below configured aircraft width class"});
    }
    std::vector<Box> runwayLabels;
    for (const auto &runway : s.airport.runways) {
        Vec2 a = geo(runway.ends[0].position), b = geo(runway.ends[1].position);
        mapLine(a, b, {0.68f, 0.69f, 0.70f}, 4);
        for (int i = 0; i < 2; ++i) {
            Vec2 point = i ? b : a;
            if (content.contains(point.x, point.y)) {
                double labelWidth = measure(runway.ends[i].name);
                Box label{std::clamp(point.x + 7, content.x + 3, content.x + content.w - labelWidth - 3),
                          point.y - 6, labelWidth, 14};
                for (int attempt = 0; attempt < 10; ++attempt) {
                    bool collision = false;
                    for (const auto &other : runwayLabels)
                        if (label.x < other.x + other.w + 4 && label.x + label.w + 4 > other.x &&
                            label.y < other.y + other.h + 3 && label.y + label.h + 3 > other.y) {
                            collision = true;
                            break;
                        }
                    if (!collision)
                        break;
                    label.y += point.y < map.y + map.h / 2 ? -18 : 18;
                }
                if (content.contains(label.x, label.y) &&
                    content.contains(label.x + label.w, label.y + label.h)) {
                    runwayLabels.push_back(label);
                    p.text(label.x, label.y, runway.ends[i].name, textColor, labelWidth + 1);
                }
            }
        }
    }
    if (s.pickVia) {
        for (const auto &node : s.airport.nodes) {
            auto point = geo(node.second.position);
            if (content.contains(point.x - 2, point.y - 2) && content.contains(point.x + 2, point.y + 2))
                p.rect({point.x - 2, point.y - 2, 4, 4}, muted);
        }
    }
    for (std::size_t i = 0; i < s.routeOptions.via.size(); ++i) {
        const auto &token = s.routeOptions.via[i];
        if (token.empty() || token[0] != '#')
            continue;
        auto found = std::find_if(s.airport.nodes.begin(), s.airport.nodes.end(), [&](const auto &node) {
            return token == "#" + std::to_string(node.first);
        });
        if (found == s.airport.nodes.end())
            continue;
        auto point = geo(found->second.position);
        if (content.contains(point.x - 4, point.y - 4) && content.contains(point.x + 4, point.y + 4)) {
            p.rect({point.x - 4, point.y - 4, 8, 8}, purple);
            std::string label = std::to_string(i + 1) + ": " + token;
            const double labelWidth = measure(label);
            const double x = std::clamp(point.x + 9, content.x + 3, content.x + content.w - labelWidth - 3);
            const double y = std::clamp(point.y + 8, content.y + 3, content.y + content.h - 18);
            p.rect({x - 2, y - 1, labelWidth + 4, 17}, background);
            p.text(x, y, label, purple, labelWidth + 1);
        }
    }
    if (s.route) {
        const bool predicted = !s.predictedCockpitPath.empty() && !s.route->requiresPushback &&
                               !s.waitingPushback && !s.emergencyHeld;
        const auto &cockpitPath = predicted ? s.predictedCockpitPath : s.route->cockpitPath;
        for (std::size_t i = 1; i < s.route->points.size(); ++i) {
            bool tow = s.route->pushbackPointCount ? i < s.route->pushbackPointCount
                                                   : s.route->requiresPushback && i == 1;
            if (!tow && !cockpitPath.empty())
                continue;
            Vec2 a = geo(unproject(s.route->origin, s.route->points[i - 1])),
                 b = geo(unproject(s.route->origin, s.route->points[i]));
            const bool risk = i < s.route->pavementRisk.size() && s.route->pavementRisk[i];
            if (tow || cockpitPath.empty())
                mapLine(a, b, tow ? purple : risk ? amber : accent, tow ? 4 : 3);
        }
        std::size_t first = 1;
        if (predicted && s.busy) {
            while (first < cockpitPath.size() && cockpitPath[first].distance <= nav.progress)
                ++first;
            if (first < cockpitPath.size())
                mapLine(geo(unproject(s.route->origin,
                                      cockpitPosition(s.route->origin, s.controllerConfig, s.aircraft))),
                        geo(unproject(s.route->origin, cockpitPath[first].position)),
                        cockpitPath[first].pavementRisk ? amber : accent, 3);
            ++first;
        }
        for (std::size_t i = first; i < cockpitPath.size(); ++i)
            mapLine(geo(unproject(s.route->origin, cockpitPath[i - 1].position)),
                    geo(unproject(s.route->origin, cockpitPath[i].position)),
                    cockpitPath[i].pavementRisk ? amber : accent, 3);
        std::size_t towCount = std::min(s.route->pushbackPointCount, s.route->points.size());
        if (towCount >= 2) {
            auto target = geo(unproject(s.route->origin, s.route->points[towCount - 1]));
            if (content.contains(target.x - 5, target.y - 5) &&
                content.contains(target.x + 5, target.y + 5)) {
                p.rect({target.x - 5, target.y - 5, 10, 10}, purple);
                std::string label = s.route->taxiStartNode >= 0
                                        ? "Node " + std::to_string(s.route->taxiStartNode)
                                        : "Taxi start";
                double labelWidth = measure(label);
                double x = std::clamp(target.x + 12, content.x + 3, content.x + content.w - labelWidth - 3);
                double y = std::clamp(target.y - 22, content.y + 3, content.y + content.h - 17);
                p.rect({x - 3, y - 2, labelWidth + 6, 18}, surface);
                p.text(x, y, label, purple, labelWidth + 1);
            }
        }
        auto stop =
            geo(unproject(s.route->origin, s.route->ramp ? s.route->cockpitStop : s.route->points.back()));
        if (content.contains(stop.x - 4, stop.y - 4) && content.contains(stop.x + 4, stop.y + 4))
            p.rect({stop.x - 4, stop.y - 4, 8, 8}, s.route->destinationRisk ? amber : accent);
    }
    if (valid(s.aircraft.position)) {
        const auto cockpit = xy(cockpitPosition(s.mapOrigin, s.controllerConfig, s.aircraft));
        const auto axlePosition = mainAxlePosition(s.mapOrigin, s.controllerConfig, s.aircraft);
        const auto axle = xy(axlePosition);
        const auto forward = direction(s.aircraft.trueHeading);
        const Vec2 side{forward.y, -forward.x};
        const auto leftGear = xy(axlePosition - side * s.controllerConfig.mainGearHalfTrack);
        const auto rightGear = xy(axlePosition + side * s.controllerConfig.mainGearHalfTrack);
        mapLine(leftGear, rightGear, textColor, 1.5);
        mapLine(leftGear, cockpit, muted, 1);
        mapLine(rightGear, cockpit, muted, 1);
        mapLine(axle, cockpit, textColor, 1.5);
        auto ring = [&](Vec2 point, Color color, double radius) {
            if (!content.contains(point.x - radius, point.y - radius) ||
                !content.contains(point.x + radius, point.y + radius))
                return;
            for (int i = 0; i < 12; ++i) {
                Vec2 a = point + direction(i * 30.) * radius;
                Vec2 b = point + direction((i + 1) * 30.) * radius;
                p.line(a.x, a.y, b.x, b.y, color, 1.5);
            }
        };
        ring(leftGear, amber, 3);
        ring(rightGear, amber, 3);
        std::vector<Box> markerLabels;
        auto marker = [&](Vec2 point, const std::string &label, Color color, double size) {
            if (!content.contains(point.x - size, point.y - size) ||
                !content.contains(point.x + size, point.y + size))
                return;
            ring(point, color, size);
            double width = measure(label);
            Box box{std::clamp(point.x + 9, content.x + 3, content.x + content.w - width - 3),
                    std::clamp(point.y - 17, content.y + 3, content.y + content.h - 17), width, 14};
            for (const auto &other : markerLabels)
                if (box.x < other.x + other.w + 3 && box.x + box.w + 3 > other.x &&
                    box.y < other.y + other.h + 3 && box.y + box.h + 3 > other.y)
                    box.y = other.y + 18 <= content.y + content.h - 17 ? other.y + 18 : other.y - 18;
            if (content.contains(box.x, box.y) && content.contains(box.x + box.w, box.y + box.h)) {
                markerLabels.push_back(box);
                p.text(box.x, box.y, label, color, box.w + 1);
            }
        };
        marker(cockpit, "COCKPIT", textColor, 3);
        marker(axle, "AXLE", amber, 3);
    }
    p.text(map.x + 14, map.y + 12,
           s.route && s.route->taxiStartNode >= 0
               ? "PUSHBACK -> NODE " + std::to_string(s.route->taxiStartNode)
           : s.mapView == MapView::Local ? "LOCAL"
                                         : "AIRPORT",
           s.route && s.route->taxiStartNode >= 0 ? purple : muted, map.w - 260);
    for (int i = 0; i < 2; ++i) {
        bool selected = s.mapView == (i == 0 ? MapView::Local : MapView::Airport);
        Box mode{map.x + map.w - 232 + i * 88, map.y + 7, 84, 27};
        p.rect(mode, selected ? lineColor : background);
        p.text(mode.x + 10, mode.y + 6, i == 0 ? "Local" : "Airport", selected ? textColor : muted,
               mode.w - 20);
        if (selected)
            p.rect({mode.x, mode.y + mode.h - 2, mode.w, 2}, s.mapView == MapView::Local ? purple : accent);
        p.frame.hits.push_back({i == 0 ? Action::LocalMap : Action::AirportMap, mode, -1, true,
                                i == 0 ? "Local map" : "Airport overview"});
    }
    p.text(map.x + map.w - 28, map.y + 12, "N", textColor, 20);
    p.line(map.x + map.w - 23, map.y + 30, map.x + map.w - 23, map.y + 45, muted);
    p.icon({map.x + 14, map.y + 39, 30, 30}, Action::Fit,
           s.mapView == MapView::Local ? "Fit local route" : "Fit airport");
    p.icon({map.x + 14, map.y + 77, 30, 30}, Action::ZoomIn, "Zoom in");
    p.icon({map.x + 14, map.y + 115, 30, 30}, Action::ZoomOut, "Zoom out");
    p.frame.hits.push_back({Action::Map, map, -1, true, {}});
    std::string mapCaption;
    Color captionColor = accent;
    if (s.loading)
        mapCaption = "Loading airport...";
    else if (s.route) {
        if (s.mapView == MapView::Local && s.route->requiresPushback) {
            std::size_t count = std::min(s.route->pushbackPointCount ? s.route->pushbackPointCount : 2,
                                         s.route->points.size());
            double towLength = 0;
            for (std::size_t i = 1; i < count; ++i)
                towLength += length(s.route->points[i] - s.route->points[i - 1]);
            mapCaption = "Pushback  /  " + decimal(towLength, 0) + " m";
            if (s.route->taxiStartNode >= 0)
                mapCaption += "  /  Node " + std::to_string(s.route->taxiStartNode);
            captionColor = purple;
        } else
            mapCaption = s.route->label + "  /  " + decimal(s.route->length / 1000, 2) + " km";
    }
    p.text(map.x + 14, map.y + map.h - 24, mapCaption, captionColor, map.w - 28);
    p.line(0, footer, w, footer, lineColor);
    p.text(22, footer + 16, "TAXI SPEED", muted);
    p.button({126, footer + 8, 32, 30}, "-", Action::SpeedDown, !s.loading && !s.waitingPushback);
    p.text(171, footer + 16, decimal(s.speedKnots, 0) + " kt", textColor, 57);
    p.button({238, footer + 8, 32, 30}, "+", Action::SpeedUp, !s.loading && !s.waitingPushback);
    Box clearance{312, footer + 9, 235, 28};
    p.rect({clearance.x, clearance.y + 4, 18, 18}, surface);
    if (s.clearance) {
        p.line(clearance.x + 4, clearance.y + 13, clearance.x + 8, clearance.y + 17, accent, 2);
        p.line(clearance.x + 8, clearance.y + 17, clearance.x + 15, clearance.y + 8, accent, 2);
    }
    p.text(clearance.x + 27, clearance.y + 6, "Runway clearance", s.clearance ? accent : muted, 200);
    p.frame.hits.push_back({Action::Clearance, clearance, -1, !s.busy && !s.loading, {}});
    p.text(w - 235, footer + 16,
           s.waitingPushback
               ? (s.route && s.route->pushbackPointCount ? "PUSHBACK DEPARTURE" : "WAITING FOR PUSHBACK")
           : s.busy ? "AUTOTAXI ACTIVE"
                    : "READY",
           s.waitingPushback ? purple
           : s.busy          ? amber
                             : muted,
           210);
    bool edit = !s.busy && !s.loading, selected = s.selected >= 0;
    bool permitted = s.route && (!(s.route->runway || !s.route->crossedRunways.empty()) || s.clearance);
    p.button({22, footer + 48, 90, 36}, "Preview", Action::Preview, edit && selected);
    p.button({122, footer + 48, 130, 36},
             s.route && s.route->requiresPushback ? "Begin departure" : "Start taxi", Action::Start,
             edit && selected && permitted, true);
    p.button({262, footer + 48, 108, 36}, "Route", Action::RouteOptions, edit);
    if (s.waitingPushback)
        p.button({378, footer + 48, 90, 36}, "GO NOW", Action::GoNow, !s.loading, true);
    else
        p.button({w - 520, footer + 48, 180, 36}, s.emergencyHeld ? "Resume taxi" : "Emergency brake",
                 Action::EmergencyBrake, s.busy, false, !s.emergencyHeld);
    p.button({w - 330, footer + 48, 160, 36}, "Manual control", Action::Manual, s.busy);
    p.button({w - 160, footer + 48, 138, 36}, "Stop taxi", Action::Stop, s.busy, false, true);
    std::string status = s.status;
    if (s.output.pavementChecked && s.output.outsideKnownPavement)
        status += " | Wheels outside known pavement (coverage incomplete)";
    if (s.route && std::any_of(s.route->pavementRisk.begin(), s.route->pavementRisk.end(),
                               [](bool risk) { return risk; }))
        status += " | Pavement clearance risk (orange)";
    if (s.route && s.route->destinationRisk)
        status += " | Undersized stand (orange)";
    if (s.route && !s.route->crossedRunways.empty()) {
        status += " | Crossings:";
        for (const auto &runway : s.route->crossedRunways)
            status += " " + runway;
    }
    if (s.route && s.route->runway)
        status += " | Runway remaining " + decimal(s.route->runwayRemaining, 0) + " m";
    if (s.route && (s.route->cockpitGuidance || s.route->mainAxleOversteer) && !s.route->requiresPushback) {
        if (s.route->mainAxleOversteer)
            status += " | Main-axle oversteer";
        if (s.busy && s.output.oversteerOffset > .25)
            status += " | Oversteer " + decimal(s.output.oversteerOffset, 1) + " m";
        else if (s.route->maximumOversteer > .25)
            status += " | Planned oversteer " + decimal(s.route->maximumOversteer, 1) + " m";
        if (!s.route->turnClearanceKnown)
            status += " | Turn pavement clearance unverified";
    }
    std::istringstream words(status);
    std::string row, word;
    double y = footer + 98;
    while (words >> word) {
        std::string next = row.empty() ? word : row + " " + word;
        if (measure(next) > w - 44 && !row.empty()) {
            p.text(22, y, row, textColor, w - 44);
            y += 18;
            row = word;
            if (y > h - 17)
                break;
        } else
            row = next;
    }
    if (y <= h - 17)
        p.text(22, y, row, textColor, w - 44);
    if (s.optionsOpen) {
        const double width = std::min(560.0, w - 44), x = w - width - 22, top = 162;
        p.rect({x, top, width, 350}, background);
        p.frame.hits.push_back({Action::None, {x, top, width, 350}, -1, true, {}});
        p.text(x + 18, top + 14, "ROUTE OPTIONS", textColor, width - 90);
        p.button({x + width - 46, top + 8, 28, 28}, "X", Action::RouteOptions, true);
        auto toggle = [&](double offset, const char *label, Action action, bool checked) {
            Box box{x + 18, top + offset, width - 36, 26};
            p.rect({box.x, box.y + 3, 18, 18}, surface);
            if (checked) {
                p.line(box.x + 3, box.y + 12, box.x + 7, box.y + 16, accent, 2);
                p.line(box.x + 7, box.y + 16, box.x + 15, box.y + 6, accent, 2);
            }
            p.text(box.x + 28, box.y + 5, label, checked ? textColor : muted, width - 70);
            p.frame.hits.push_back({action, box, -1, edit, {}});
        };
        toggle(48, "Allow oversteer", Action::AllowOversteer, s.routeOptions.allowOversteer);
        toggle(80, "Relax pavement clearance", Action::RelaxPavement, s.routeOptions.ignorePavementLimits);
        toggle(112, "Allow undersized stands", Action::RelaxStand, s.routeOptions.ignoreStandSize);
        toggle(144, "Show stand labels", Action::ToggleStandLabels, s.showStandLabels);
        toggle(176, "Show airport signs", Action::ToggleAirportSigns, s.showAirportSigns);
        p.text(x + 18, top + 213, "ORDERED TAXIWAYS / NODES", muted, width - 36);
        Box input{x + 18, top + 236, width - 36, 32};
        p.rect(input, surface);
        std::string text = s.viaText + (s.viaFocus ? "|" : "");
        while (!text.empty() && measure(text) > input.w - 20)
            text.erase(text.begin());
        p.text(input.x + 10, input.y + 9, text, textColor, input.w - 20);
        p.frame.hits.push_back({Action::ViaInput, input, -1, edit, "Ordered taxiway names or #node IDs"});
        p.button({x + 18, top + 286, 110, 32}, s.pickVia ? "Done picking" : "Pick nodes", Action::PickVia,
                 edit);
        p.button({x + 138, top + 286, 64, 32}, "Undo", Action::UndoVia, edit);
        p.button({x + 212, top + 286, 74, 32}, "Clear", Action::ClearVia, edit);
        p.button({x + width - 128, top + 286, 110, 32}, "Apply", Action::ApplyVia, edit, true);
    }
    return std::move(p.frame);
}
} // namespace autotaxi
