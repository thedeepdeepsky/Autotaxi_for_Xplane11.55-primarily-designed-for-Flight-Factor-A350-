#include "CenterlineNetwork.h"
#include "DsfLoader.h"
#include "Panel.h"
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
using namespace autotaxi;
namespace {
double measure(const std::string &text) {
    return text.size() * 8.0;
}
void benchmark(PanelState state, const char *scenario) {
    constexpr int draws = 120, updateEvery = 15;
    auto begin = std::chrono::steady_clock::now();
    std::size_t commands = 0;
    for (int i = 0; i < draws; ++i) {
        auto frame = buildPanel(state, 1080, 720, measure);
        commands += frame.commands.size();
    }
    double uncached =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
    PanelFrameCache cache;
    begin = std::chrono::steady_clock::now();
    std::size_t cachedCommands = 0;
    for (int i = 0; i < draws; ++i) {
        if (i % updateEvery == 0)
            cache.invalidate();
        cachedCommands += cache.get(state, 1080, 720, measure).commands.size();
    }
    double cached =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
    if (commands != cachedCommands || cache.rebuildCount() != 8 || cache.mapRebuildCount() != 1)
        throw std::runtime_error("Cached frames changed command count or rebuilt more than eight times");
    const auto &frame = cache.get(state, 1080, 720, measure);
    std::size_t lines = 0, batches = 0;
    for (std::size_t i = 0; i < frame.commands.size();) {
        if (frame.commands[i].kind == DrawKind::Line) {
            auto end = lineBatchEnd(frame.commands, i);
            lines += end - i;
            ++batches;
            i = end;
        } else
            ++i;
    }
    std::cout << scenario << ": " << draws << " draws, commands/frame=" << commands / draws
              << ", uncached CPU=" << uncached << " ms, cached CPU=" << cached << " ms"
              << ", rebuilds=" << cache.rebuildCount() << ", map rebuilds=" << cache.mapRebuildCount()
              << ", line segments=" << lines << ", GL line batches=" << batches << '\n';
}
} // namespace
int main(int argc, char **argv) {
    try {
        PanelState state;
        if (argc == 2 || argc == 4) {
            std::ifstream input(std::filesystem::u8path(argv[1]));
            if (!input)
                throw std::runtime_error("Cannot open local apt.dat");
            state.airport = parseAirport(input, argv[1], argc != 4);
            if (argc == 4) {
                const auto dsf = loadDsfPaintedLines(state.airport, argv[2], argv[3]);
                buildCenterlineNetwork(state.airport);
                std::cout << "Local DSF: " << dsf.lines << " painted chains, " << dsf.contours
                          << " contours\n";
            }
        } else if (argc == 1) {
            state.airport.id = "SYNTHETIC";
            GeoPoint origin{31, 121};
            state.aircraft.position = origin;
            for (int chain = 0; chain < 3000; ++chain) {
                GroundLine line{1, "Test centerline", {}};
                for (int point = 0; point < 20; ++point)
                    line.points.push_back(unproject(
                        origin, {static_cast<double>(point * 20), static_cast<double>(chain % 400)}));
                state.airport.groundLines.push_back(std::move(line));
            }
        } else
            throw std::runtime_error("Usage: panel_benchmark [local-apt.dat [xplane-root DSFTool.exe]]");
        if (!state.airport.ramps.empty())
            state.aircraft.position = state.airport.ramps.front().position;
        else if (!state.airport.runways.empty())
            state.aircraft.position = state.airport.runways.front().ends[0].position;
        state.choices = destinations(state.airport);
        fitAirport(state);
        std::cout << std::fixed << std::setprecision(2) << "Airport " << state.airport.id
                  << " (CPU layout benchmark, not X-Plane FPS)\n";
        benchmark(state, "Airport overview");
        Route route;
        route.origin = state.aircraft.position;
        route.label = "Synthetic ETA benchmark";
        for (int i = 0; i <= 2500; ++i)
            route.points.push_back({0, i * 2.0});
        route.length = 5000;
        state.route = std::move(route);
        state.timing = estimateRouteTiming(*state.route, 0, 0, state.controllerConfig);
        state.busy = true;
        benchmark(state, "Overview with 5000 m ETA");
        selectMapView(state, MapView::Local);
        state.mapScale = 1;
        benchmark(state, "Local map with 5000 m ETA");
        return 0;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
