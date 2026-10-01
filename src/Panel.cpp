#include "Panel.h"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
namespace autotaxi {
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
struct NavigationInfo {
    double progress = 0, nextTurnDistance = -1, nextTurnAngle = 0;
    std::string node = "--", taxiway = "--";
    double nodeEta = -1, taxiwayEta = -1, totalEta = -1;
};
NavigationInfo navigation(const PanelState &s) {
    NavigationInfo info;
    if (!s.route || s.route->points.size() < 2)
        return info;
    const auto &route = *s.route;
    info.progress = s.busy && !s.waitingPushback ? s.output.progress : 0;
    if (info.progress <= 0.01 && !s.waitingPushback) {
        Vec2 aircraft = project(route.origin, s.aircraft.position);
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
    auto speed = std::max(1.0, s.speedKnots * 0.514444);
    auto effectiveSpeed = std::max(1.0, std::min(speed, 20.0 * 0.514444));
    auto seconds = [&](double distance) { return distance / effectiveSpeed; };
    for (std::size_t i = 1; i + 1 < route.points.size(); ++i) {
        double before = length(route.points[i] - route.points[i - 1]);
        double distanceAt = 0;
        for (std::size_t k = 1; k <= i; ++k)
            distanceAt += length(route.points[k] - route.points[k - 1]);
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
            info.nodeEta = seconds(marker.distance - info.progress);
            break;
        }
    for (const auto &marker : route.taxiwaysAlongRoute)
        if (marker.distance > info.progress + 5) {
            info.taxiway = marker.label;
            info.taxiwayEta = seconds(marker.distance - info.progress);
            break;
        }
    info.totalEta = seconds(std::max(0.0, route.length - info.progress));
    return info;
}
struct Painter {
    PanelFrame frame;
    const MeasureText &measure;
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
    void text(double x, double y, std::string label, Color c, double maxWidth = 10000) {
        while (!label.empty() && measure(label) > maxWidth) {
            label.pop_back();
            if (label.size() > 3)
                label.replace(label.size() - 3, 3, "...");
        }
        frame.commands.push_back({DrawKind::Text, {x, y, maxWidth, 16}, c, label});
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
    for (std::size_t i = 0; i < s.choices.size(); ++i)
        if (static_cast<int>(s.choices[i].kind) == s.category &&
            upper(s.choices[i].label).find(query) != std::string::npos)
            out.push_back(static_cast<int>(i));
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
PanelFrame buildPanel(const PanelState &s, double w, double h, const MeasureText &measure) {
    Painter p{{}, measure};
    p.rect({0, 0, w, h}, background);
    p.text(22, 20, "A350 AUTOTAXI", textColor);
    p.text(220, 20, s.airport.id.empty() ? "Airport detection" : s.airport.id + "   " + s.airport.name, muted,
           w - 345);
    p.icon({w - 96, 12, 30, 30}, Action::Refresh, "Detect airport", !s.busy && !s.loading);
    p.icon({w - 56, 12, 30, 30}, Action::Reload, "Reload settings", !s.busy && !s.loading);
    p.line(0, 52, w, 52, lineColor);
    const double column = w / 4;
    const std::string labels[] = {"GROUND SPEED", "NOSEWHEEL CMD / ACT", "REMAINING", "TRACK ERROR"};
    const std::string values[] = {decimal(s.aircraft.speed / 0.514444) + " kt",
                                  decimal(s.output.steerDegrees) + " / " + decimal(s.actualSteer) + " deg",
                                  s.busy && !s.waitingPushback ? decimal(s.output.remaining, 0) + " m"
                                  : s.route                    ? decimal(s.route->length, 0) + " m"
                                                               : "--",
                                  s.busy && !s.waitingPushback ? decimal(s.output.crossTrack) + " m" : "--"};
    for (int i = 0; i < 4; ++i) {
        p.text(22 + column * i, 65, labels[i], muted, column - 35);
        p.text(22 + column * i, 86, values[i], i == 0 ? accent : textColor, column - 35);
    }
    p.line(0, 115, w, 115, lineColor);
    double leftWidth = 280, footer = h - 145;
    p.text(22, 128, "FROM", muted);
    double nodeDistance = 0;
    int nodeId = nearestNode(s.airport, s.aircraft.position, &nodeDistance);
    std::string nodeLabel =
        nodeId >= 0 ? "Node " + std::to_string(nodeId) + " / " + decimal(nodeDistance, 0) + " m" : "";
    double nodeWidth = measure(nodeLabel);
    p.text(78, 128, departureLabel(s.airport, s.aircraft.position), textColor, w - 130 - nodeWidth);
    if (nodeId >= 0)
        p.text(w - nodeWidth - 22, 128, nodeLabel, muted, nodeWidth + 1);
    auto nav = navigation(s);
    if (s.route && !s.waitingPushback) {
        std::string turn = nav.nextTurnDistance >= 0 ? "NEXT TURN " + decimal(nav.nextTurnDistance, 0) +
                                                           " m / " + decimal(nav.nextTurnAngle, 0) + " deg"
                                                     : "NEXT TURN --";
        std::string node = "NEXT " + nav.node + " / " + eta(nav.nodeEta);
        std::string taxiway = "TAXIWAY " + nav.taxiway + " / " + eta(nav.taxiwayEta);
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
        Box b{list.x, list.y + row * 47, list.w, 43};
        bool selected = id == s.selected;
        if (selected) {
            p.rect(b, surface);
            p.rect({b.x, b.y, 3, b.h}, accent);
        }
        p.text(b.x + 12, b.y + 5, d.label, selected ? textColor : muted, 235);
        auto entry = s.availability.find(d.label);
        std::string detail = d.kind == DestinationKind::Runway ? "Route pending" : "";
        Color color = muted;
        if (entry != s.availability.end()) {
            const auto &info = entry->second;
            detail = info.reachable ? (info.pushback ? "Pushback  /  " : "Available  /  ") +
                                          decimal(info.length / 1000, 2) + " km"
                     : info.reason.find("entry") != std::string::npos      ? "No runway entry"
                     : info.reason.find("connection") != std::string::npos ? "No apron connection"
                                                                           : "No compatible route";
            color = info.reachable ? (info.pushback ? purple : accent) : red;
        }
        p.text(b.x + 12, b.y + 24, detail, color, 235);
        p.frame.hits.push_back({Action::Destination, b, id, !s.busy && !s.loading, {}});
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
    auto geo = [&](GeoPoint point) { return xy(project(s.mapOrigin, point)); };
    auto mapLine = [&](Vec2 a, Vec2 b, Color color, double width = 1) {
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
    if (s.mapView == MapView::Local)
        for (const auto &pavement : s.airport.pavements)
            for (const auto &ring : pavement.rings)
                for (std::size_t i = 0; i < ring.size(); ++i)
                    mapLine(geo(ring[i]), geo(ring[(i + 1) % ring.size()]), {0.21f, 0.23f, 0.25f});
    for (const auto &line : s.airport.groundLines)
        for (std::size_t i = 1; i < line.points.size(); ++i)
            mapLine(geo(line.points[i - 1]), geo(line.points[i]),
                    line.centerline() ? Color{.72f, .61f, .28f} : Color{.34f, .33f, .29f},
                    line.centerline() ? 1.3 : 1);
    for (const auto &edge : s.airport.edges) {
        if (edge.painted)
            continue;
        Vec2 a = geo(s.airport.nodes.at(edge.from).position), b = geo(s.airport.nodes.at(edge.to).position);
        if (!edge.runway && !s.airport.groundLines.empty()) {
            Vec2 delta = b - a;
            double extent = length(delta);
            if (extent > 0)
                for (double along = 0; along < extent; along += 10)
                    mapLine(a + delta * (along / extent), a + delta * (std::min(along + 4, extent) / extent),
                            {.25f, .28f, .30f});
            continue;
        }
        mapLine(a, b, edge.runway ? Color{0.48f, 0.49f, 0.50f} : Color{0.29f, 0.32f, 0.34f},
                edge.runway ? 3 : 1);
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
    if (s.route) {
        for (std::size_t i = 1; i < s.route->points.size(); ++i) {
            Vec2 a = geo(unproject(s.route->origin, s.route->points[i - 1])),
                 b = geo(unproject(s.route->origin, s.route->points[i]));
            bool tow = s.route->pushbackPointCount ? i < s.route->pushbackPointCount
                                                   : s.route->requiresPushback && i == 1;
            mapLine(a, b, tow ? purple : accent, tow ? 4 : 3);
        }
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
        auto stop = geo(unproject(s.route->origin, s.route->points.back()));
        if (content.contains(stop.x - 4, stop.y - 4) && content.contains(stop.x + 4, stop.y + 4))
            p.rect({stop.x - 4, stop.y - 4, 8, 8}, accent);
    }
    auto aircraft = geo(s.aircraft.position);
    if (content.contains(aircraft.x - 12, aircraft.y - 12) &&
        content.contains(aircraft.x + 12, aircraft.y + 12)) {
        Vec2 f = direction(s.aircraft.trueHeading);
        f.y = -f.y;
        Vec2 side{f.y, -f.x};
        p.triangle(aircraft + f * 13, aircraft - f * 8 + side * 7, aircraft - f * 8 - side * 7, accent);
        p.rect({aircraft.x - 2, aircraft.y - 2, 4, 4}, textColor);
        p.text(std::clamp(aircraft.x + 14, content.x + 3, content.x + content.w - 42),
               std::clamp(aircraft.y - 19, content.y + 3, content.y + content.h - 17), "A350", accent, 40);
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
    p.button({126, footer + 8, 32, 30}, "-", Action::SpeedDown, !s.busy && !s.loading);
    p.text(171, footer + 16, decimal(s.speedKnots, 0) + " kt", textColor, 57);
    p.button({238, footer + 8, 32, 30}, "+", Action::SpeedUp, !s.busy && !s.loading);
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
    p.button({22, footer + 48, 126, 36}, "Preview", Action::Preview, edit && selected);
    p.button({158, footer + 48, 160, 36},
             s.route && s.route->requiresPushback ? "Begin departure" : "Start taxi", Action::Start,
             edit && selected && permitted, true);
    p.button({w - 360, footer + 48, 162, 36}, "Manual control", Action::Manual, s.busy);
    p.button({w - 184, footer + 48, 162, 36}, "Stop", Action::Stop, s.busy, false, true);
    std::string status = s.status;
    if (s.route && !s.route->crossedRunways.empty()) {
        status += " | Crossings:";
        for (const auto &runway : s.route->crossedRunways)
            status += " " + runway;
    }
    if (s.route && s.route->runway)
        status += " | Runway remaining " + decimal(s.route->runwayRemaining, 0) + " m";
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
    return std::move(p.frame);
}
} // namespace autotaxi
