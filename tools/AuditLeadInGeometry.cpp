#include "AptDatabase.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
using namespace autotaxi;
int main(int argc, char **argv) {
    std::ifstream input(argv[1]);
    for (std::string row; input;) {
        const auto offset = input.tellg();
        if (!std::getline(input, row)) break;
        std::istringstream h(row); int code, a, b, c; std::string id;
        if (h >> code >> a >> b >> c >> id && code == 1 && id == "ZSSS") { input.clear(); input.seekg(offset); break; }
    }
    auto airport = parseAirport(input, argv[1]);
    std::cout << std::fixed << std::setprecision(1);
    for (const auto &line : airport.groundLines) if (line.standLeadIn) {
        double len = 0; for (size_t i=1;i<line.points.size();++i) len += distance(line.points[i-1],line.points[i]);
        const auto &r = airport.ramps[line.standLeadInRamp];
        double best=1e9, end0=distance(r.position,line.points.front()), end1=distance(r.position,line.points.back());
        for (size_t i=1;i<line.points.size();++i) best=std::min(best,onSegment({},project(r.position,line.points[i-1]),project(r.position,line.points[i])).distance);
        std::cout << "style=" << line.style << " points=" << line.points.size() << " length=" << len
                  << " gap=" << best << " ends=" << end0 << ',' << end1 << " ramp=" << r.name << '\n';
    }
}
