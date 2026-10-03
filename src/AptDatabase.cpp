#include "AptDatabase.h"
#include "CenterlineNetwork.h"
#include <algorithm>
#include <fstream>
#include <limits>
#include <locale>
#include <set>
#include <sstream>
#include <stdexcept>
#include <unordered_set>
namespace autotaxi {
namespace {
std::string trim(std::string s) {
    auto a = s.find_first_not_of(" \t\r\n"), b = s.find_last_not_of(" \t\r\n");
    return a == std::string::npos ? std::string{} : s.substr(a, b - a + 1);
}
std::vector<std::string> tokens(const std::string &line) {
    std::istringstream s(line);
    s.imbue(std::locale::classic());
    std::vector<std::string> t;
    for (std::string v; s >> v;)
        t.push_back(v);
    return t;
}
double number(const std::string &s) {
    std::istringstream in(s);
    in.imbue(std::locale::classic());
    double n;
    if (!(in >> n) || !std::isfinite(n))
        throw std::runtime_error("Invalid apt.dat number");
    return n;
}
std::string tail(const std::vector<std::string> &t, std::size_t from) {
    std::string out;
    for (std::size_t i = from; i < t.size(); ++i) {
        if (!out.empty())
            out += ' ';
        out += t[i];
    }
    return out;
}
void addPoint(AirportIndex &a, GeoPoint p) {
    if (!valid(p))
        return;
    if (!a.hasPosition) {
        a.anchor = p;
        a.hasPosition = true;
    }
    a.radius = std::max(a.radius, distance(a.anchor, p));
}
struct CurveNode {
    GeoPoint point, control;
    bool curved = false;
    int style = 0;
};
std::vector<GeoPoint> curve(const CurveNode &from, const CurveNode &to) {
    if ((!from.curved && !to.curved) ||
        (distance(from.point, to.point) < .001 && !(from.curved && to.curved)))
        return {from.point, to.point};
    Vec2 end = project(from.point, to.point);
    Vec2 c1 = from.curved ? project(from.point, from.control) : Vec2{};
    Vec2 c2 = to.curved ? end * 2 - project(from.point, to.control) : end;
    double extent = length(c1) + length(c2 - c1) + length(end - c2);
    int samples = std::clamp(static_cast<int>(std::ceil(extent / 3)), 1, 10000);
    std::vector<GeoPoint> result{from.point};
    for (int i = 1; i <= samples; ++i) {
        double t = static_cast<double>(i) / samples, u = 1 - t;
        result.push_back(
            unproject(from.point, c1 * (3 * u * u * t) + c2 * (3 * u * t * t) + end * (t * t * t)));
    }
    result.back() = to.point;
    return result;
}
} // namespace
std::vector<std::filesystem::path> discoverAptFiles(const std::filesystem::path &root) {
    namespace fs = std::filesystem;
    std::vector<fs::path> files;
    std::set<fs::path> seen;
    auto add = [&](const fs::path &p) {
        std::error_code ec;
        auto n = fs::absolute(p, ec).lexically_normal();
        if (!ec && fs::is_regular_file(n, ec) && seen.insert(n).second)
            files.push_back(n);
    };
    std::ifstream ini(root / "Custom Scenery" / "scenery_packs.ini");
    bool hasSceneryOrder = ini.is_open();
    for (std::string line; std::getline(ini, line);) {
        if (line.compare(0, 3, "\xEF\xBB\xBF") == 0)
            line.erase(0, 3);
        line = trim(line);
        const std::string prefix = "SCENERY_PACK ";
        if (line.compare(0, prefix.size(), prefix) != 0)
            continue;
        std::string pack = trim(line.substr(prefix.size()));
        if (pack == "*GLOBAL_AIRPORTS*") {
            add(root / "Global Scenery" / "Global Airports" / "Earth nav data" / "apt.dat");
        } else {
            fs::path p = fs::u8path(pack);
            add((p.is_absolute() ? p : root / p) / "Earth nav data" / "apt.dat");
        }
    }
    // The scenery order is authoritative; disabled and unlisted packs stay excluded.
    if (!hasSceneryOrder) {
        add(root / "Custom Scenery" / "Global Airports" / "Earth nav data" / "apt.dat");
        add(root / "Global Scenery" / "Global Airports" / "Earth nav data" / "apt.dat");
    }
    add(root / "Resources" / "default scenery" / "default apt dat" / "Earth nav data" / "apt.dat");
    if (files.empty())
        throw std::runtime_error("No active apt.dat found under X-Plane root");
    return files;
}
Airport parseAirport(std::istream &input, const std::string &source, bool buildNetwork) {
    Airport a;
    a.source = source;
    bool started = false;
    int pavement = -1;
    std::vector<CurveNode> ring;
    bool linear = false;
    std::string lineName;
    auto finishRing = [&](bool closed) {
        if (ring.size() < 2) {
            ring.clear();
            return;
        }
        std::vector<GeoPoint> outline;
        std::size_t count = closed ? ring.size() : ring.size() - 1;
        for (std::size_t i = 0; i < count; ++i) {
            auto points = curve(ring[i], ring[(i + 1) % ring.size()]);
            if (outline.empty())
                outline.push_back(points.front());
            outline.insert(outline.end(), points.begin() + 1, points.end());
            // Node attributes in a 110 pavement ring are pavement edge or
            // texture metadata, not painted linear features. Only row 120
            // linear features may populate the ground-marking layer.
            if (linear && ring[i].style > 0) {
                if (!a.groundLines.empty() && a.groundLines.back().style == ring[i].style &&
                    a.groundLines.back().name == lineName &&
                    distance(a.groundLines.back().points.back(), points.front()) < .01)
                    a.groundLines.back().points.insert(a.groundLines.back().points.end(), points.begin() + 1,
                                                       points.end());
                else
                    a.groundLines.push_back({ring[i].style, lineName, std::move(points)});
            }
        }
        if (closed && pavement >= 0 && outline.size() >= 4) {
            outline.pop_back();
            a.pavements[pavement].rings.push_back(std::move(outline));
        }
        ring.clear();
    };
    for (std::string line; std::getline(input, line);) {
        auto t = tokens(line);
        if (t.empty())
            continue;
        try {
            int code = std::stoi(t[0]);
            if (code == 1 || code == 16 || code == 17) {
                if (started)
                    break;
                if (t.size() < 6)
                    continue;
                a.id = t[4];
                a.name = tail(t, 5);
                started = true;
                continue;
            }
            if (!started)
                continue;
            if (code == 99)
                break;
            if (code == 110) {
                pavement = -1;
                linear = false;
                lineName = tail(t, 4);
                ring.clear();
                if (t.size() >= 2) {
                    int surface = std::stoi(t[1]);
                    if (surface == 1 || surface == 2 || surface >= 15) {
                        a.pavements.push_back({});
                        pavement = static_cast<int>(a.pavements.size() - 1);
                    }
                }
            } else if (code == 120) {
                pavement = -1;
                linear = true;
                lineName = tail(t, 1);
                ring.clear();
            } else if (code >= 111 && code <= 116 && (pavement >= 0 || linear) && t.size() >= 3) {
                CurveNode n;
                n.point = {number(t[1]), number(t[2])};
                n.curved = code == 112 || code == 114 || code == 116;
                std::size_t attributes = n.curved ? 5 : 3;
                if (n.curved) {
                    if (t.size() < 5) {
                        ring.clear();
                        continue;
                    }
                    n.control = {number(t[3]), number(t[4])};
                    if (!valid(n.control)) {
                        ring.clear();
                        continue;
                    }
                }
                if (code != 115 && code != 116 && attributes < t.size()) {
                    // The first optional value is the line type (0..108).
                    // A second value is the lighting code; do not let it
                    // overwrite the painted line type.  In particular, a
                    // centerline may be encoded as `1 101`.
                    int style = std::stoi(t[attributes]);
                    if (style > 0 && style <= 108)
                        n.style = style;
                }
                if (valid(n.point))
                    ring.push_back(n);
                if (code >= 113)
                    finishRing(code == 113 || code == 114);
            } else {
                pavement = -1;
                linear = false;
                ring.clear();
            }
            if (code == 100 && t.size() >= 26) {
                Runway r;
                r.width = number(t[1]);
                r.ends[0] = {t[8], {number(t[9]), number(t[10])}, number(t[11])};
                r.ends[1] = {t[17], {number(t[18]), number(t[19])}, number(t[20])};
                if (valid(r.ends[0].position) && valid(r.ends[1].position) && r.width > 0)
                    a.runways.push_back(r);
            } else if (code == 1201 && t.size() >= 5) {
                TaxiNode n{std::stoi(t[4]), {number(t[1]), number(t[2])}, tail(t, 5)};
                if (valid(n.position))
                    a.nodes[n.id] = n;
            } else if (code == 1202 && t.size() >= 5) {
                TaxiEdge e;
                e.from = std::stoi(t[1]);
                e.to = std::stoi(t[2]);
                if (t[3] != "oneway" && t[3] != "twoway")
                    continue;
                e.oneWay = t[3] == "oneway";
                e.runway = t[4] == "runway";
                e.name = tail(t, 5);
                if (!e.runway && t[4].rfind("taxiway", 0) != 0)
                    continue;
                if (t[4].size() == 9 && t[4][7] == '_' && t[4][8] >= 'A' && t[4][8] <= 'F')
                    e.width = t[4][8];
                a.edges.push_back(e);
            } else if (code == 1204 && t.size() >= 3 && !a.edges.empty()) {
                for (std::size_t i = 2; i < t.size(); ++i) {
                    std::istringstream names(t[i]);
                    for (std::string n; std::getline(names, n, ',');)
                        if (!n.empty())
                            a.edges.back().activeRunways.push_back(n);
                }
            } else if (code == 15 && t.size() >= 5) {
                Ramp r{{number(t[1]), number(t[2])}, number(t[3]), 0, tail(t, 4), {}};
                if (valid(r.position) && std::isfinite(r.heading))
                    a.ramps.push_back(r);
            } else if (code == 1300 && t.size() >= 7) {
                Ramp r{{number(t[1]), number(t[2])}, number(t[3]), 0, tail(t, 6), {}};
                if (valid(r.position))
                    a.ramps.push_back(r);
            } else if (code == 1301 && t.size() >= 2 && !a.ramps.empty())
                a.ramps.back().width = t[1][0];
            else if (code == 20 && t.size() >= 7) {
                AirportSign sign;
                sign.position = {number(t[1]), number(t[2])};
                sign.heading = number(t[3]);
                sign.text = tail(t, 6);
                sign.runway = sign.text.find("RWY") != std::string::npos ||
                              sign.text.find("10-") != std::string::npos ||
                              sign.text.find("18-") != std::string::npos;
                sign.noEntry = sign.text.find("NO ENTRY") != std::string::npos ||
                               sign.text.find("NO_ENTRY") != std::string::npos;
                if (valid(sign.position) && !sign.text.empty())
                    a.signs.push_back(std::move(sign));
            }
        } catch (const std::exception &) { /* Ignore malformed records, never synthesize connections. */
            ring.clear();
            pavement = -1;
            linear = false;
        }
    }
    for (const auto &line : a.groundLines) {
        if (line.points.size() < 2 ||
            !(line.style == 4 || line.style == 5 || line.style == 6 || line.style == 103 ||
              line.style == 104))
            continue;
        Vec2 local = project(line.points.front(), line.points.back());
        GeoPoint midpoint = unproject(line.points.front(), local * .5);
        int runway = -1;
        double best = 1e30;
        const double holdHeading = heading(local) + 90;
        for (std::size_t i = 0; i < a.runways.size(); ++i) {
            Vec2 start = project(midpoint, a.runways[i].ends[0].position);
            Vec2 end = project(midpoint, a.runways[i].ends[1].position);
            auto snap = onSegment({}, start, end);
            if (snap.distance < best && snap.distance <= a.runways[i].width * .5 + 55) {
                best = snap.distance;
                runway = static_cast<int>(i);
            }
        }
        if (runway < 0)
            continue;
        int end = distance(midpoint, a.runways[runway].ends[1].position) <
                          distance(midpoint, a.runways[runway].ends[0].position)
                      ? 1
                      : 0;
        if (std::none_of(a.holdShortPoints.begin(), a.holdShortPoints.end(), [&](const HoldShortPoint &p) {
                return p.runway == runway && distance(p.position, midpoint) < 8;
            }))
            a.holdShortPoints.push_back({midpoint, runway, end, holdHeading,
                                         "RWY " + a.runways[runway].ends[end].name + " HOLD " +
                                             std::to_string(a.holdShortPoints.size() + 1)});
    }
    a.edges.erase(std::remove_if(a.edges.begin(), a.edges.end(),
                                 [&](const TaxiEdge &e) {
                                     return e.from == e.to || !a.nodes.count(e.from) || !a.nodes.count(e.to);
                                 }),
                  a.edges.end());
    if (buildNetwork)
        buildCenterlineNetwork(a);
    return a;
}
void AptDatabase::scan(const std::filesystem::path &root) {
    index_.clear();
    alternatives_.clear();
    std::unordered_set<std::string> ids;
    for (const auto &file : discoverAptFiles(root)) {
        std::ifstream in(file, std::ios::binary);
        if (!in)
            continue;
        AirportIndex current;
        bool recording = false;
        auto flush = [&]() {
            if (recording && current.hasPosition)
                index_.push_back(current);
        };
        while (in) {
            auto offset = in.tellg();
            std::string line;
            if (!std::getline(in, line))
                break;
            // Most apt.dat rows are polygons and lights; skip these without tokenizing.
            if (line.rfind("1 ", 0) != 0 && line.rfind("16 ", 0) != 0 && line.rfind("17 ", 0) != 0 &&
                line.rfind("100 ", 0) != 0 && line.rfind("1201 ", 0) != 0 && line.rfind("1300 ", 0) != 0 &&
                line.rfind("15 ", 0) != 0)
                continue;
            auto t = tokens(line);
            if (t.empty())
                continue;
            try {
                int code = std::stoi(t[0]);
                if (code == 1 || code == 16 || code == 17) {
                    flush();
                    current = {};
                    recording = false;
                    if (t.size() >= 6 && ids.insert(t[4]).second) {
                        current.id = t[4];
                        current.name = tail(t, 5);
                        current.file = file;
                        current.offset = offset;
                        recording = true;
                    } else if (t.size() >= 6) {
                        AirportIndex alternate;
                        alternate.id = t[4];
                        alternate.file = file;
                        alternate.offset = offset;
                        alternatives_.push_back(alternate);
                    }
                } else if (recording && code == 100 && t.size() >= 26) {
                    addPoint(current, {number(t[9]), number(t[10])});
                    addPoint(current, {number(t[18]), number(t[19])});
                } else if (recording && (code == 1201 || code == 1300 || code == 15) && t.size() >= 3)
                    addPoint(current, {number(t[1]), number(t[2])});
            } catch (const std::exception &) {
            }
        }
        flush();
    }
    if (index_.empty())
        throw std::runtime_error("No airports with coordinates in active apt.dat files");
}
Airport AptDatabase::nearest(GeoPoint p, double maxDistance) const {
    if (!valid(p))
        throw std::runtime_error("Invalid aircraft position");
    std::vector<std::pair<double, const AirportIndex *>> candidates;
    for (const auto &a : index_) {
        double lower = std::max(0.0, distance(p, a.anchor) - a.radius);
        if (lower <= maxDistance)
            candidates.push_back({lower, &a});
    }
    std::sort(candidates.begin(), candidates.end(),
              [](const auto &a, const auto &b) { return a.first < b.first; });
    Airport best;
    double bestDistance = maxDistance;
    for (const auto &candidate : candidates) {
        if (candidate.first > bestDistance)
            break;
        std::ifstream in(candidate.second->file, std::ios::binary);
        in.seekg(candidate.second->offset);
        auto a = parseAirport(in, candidate.second->file.u8string());
        double score = std::numeric_limits<double>::infinity();
        for (const auto &n : a.nodes)
            score = std::min(score, distance(p, n.second.position));
        for (const auto &r : a.runways)
            score = std::min(
                score,
                onSegment({}, project(p, r.ends[0].position), project(p, r.ends[1].position)).distance);
        for (const auto &r : a.ramps)
            score = std::min(score, distance(p, r.position));
        if (score < bestDistance) {
            best = std::move(a);
            bestDistance = score;
        }
    }
    if (best.id.empty())
        throw std::runtime_error("No airport within 5 km of current aircraft position");
    for (const auto &alternate : alternatives_)
        if (alternate.id == best.id) {
            std::ifstream in(alternate.file, std::ios::binary);
            in.seekg(alternate.offset);
            auto metadata = parseAirport(in);
            for (const auto &other : metadata.ramps) {
                Ramp *match = nullptr;
                double gap = 25;
                for (auto &ramp : best.ramps) {
                    double d = distance(ramp.position, other.position);
                    if (d < gap) {
                        gap = d;
                        match = &ramp;
                    }
                }
                if (match && other.name != match->name &&
                    std::find(match->aliases.begin(), match->aliases.end(), other.name) ==
                        match->aliases.end())
                    match->aliases.push_back(other.name);
            }
        }
    return best;
}
int nearestNode(const Airport &a, GeoPoint p, double *meters) {
    int id = -1;
    double d = std::numeric_limits<double>::infinity();
    for (const auto &n : a.nodes) {
        double x = distance(p, n.second.position);
        if (x < d) {
            d = x;
            id = n.first;
        }
    }
    if (meters)
        *meters = d;
    return id;
}
} // namespace autotaxi
