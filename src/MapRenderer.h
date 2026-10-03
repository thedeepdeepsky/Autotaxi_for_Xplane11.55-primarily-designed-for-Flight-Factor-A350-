#pragma once
#include "Panel.h"
namespace autotaxi {
class MapRenderer {
  public:
    MapRenderer() = default;
    ~MapRenderer();
    MapRenderer(const MapRenderer &) = delete;
    MapRenderer &operator=(const MapRenderer &) = delete;
    bool draw(const PanelFrame &frame, std::size_t revision, double left, double top);
    std::size_t compileCount() const {
        return compileCount_;
    }

  private:
    unsigned int list_ = 0;
    std::size_t revision_ = 0, compileCount_ = 0;
};
} // namespace autotaxi
