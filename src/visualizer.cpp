#include "world.cpp"

#include <fstream>
#include <sstream>

std::string world_to_json(const World& world) {
  std::ostringstream json;
  json << "{\"particles\":[";

  const auto& particles = world.get_particles();
  std::size_t nanCount = 0;
  std::size_t firstNanIdx = std::numeric_limits<std::size_t>::max();
  for (std::size_t i = 0; i < particles.size(); ++i) {
    const Vec2& pos = particles[i].get_pos();
    if (std::isnan(pos.get_x()) || std::isnan(pos.get_y())) {
      nanCount++;
      if (firstNanIdx == std::numeric_limits<std::size_t>::max()) firstNanIdx = i;
    }
    json << "{\"x\":" << pos.get_x() << ",\"y\":" << pos.get_y() << "}";
    if (i < particles.size() - 1) json << ",";
  }

  json << "],\"rects\":[";
  const auto& rects = world.get_rects();
  for (std::size_t i = 0; i < rects.size(); ++i) {
    json << "{\"nx\":" << rects[i].get_nx()
         << ",\"ny\":" << rects[i].get_ny()
         << ",\"start_idx\":" << rects[i].get_start_idx() << "}";
    if (i < rects.size() - 1) json << ",";
  }
  json << "]}";
  
  return json.str();
}