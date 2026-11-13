#include "world.cpp"

#include <fstream>
#include <sstream>
#include <string>

std::string world_to_json(const World& world) {
  std::ostringstream json;
  json << "{\"particles\":[";

  const auto& particles = world.get_particles();
  std::size_t nanCount = 0;
  std::size_t firstNanIdx = std::numeric_limits<std::size_t>::max();
  bool firstAdded = false;
  for (std::size_t i = 0; i < particles.size(); ++i) {
    const Vec2& pos = particles[i].get_pos();
    double x = pos.get_x(), y = pos.get_y();
    if (!std::isfinite(x) || !std::isfinite(y)) {
      nanCount++;
      if (firstNanIdx == std::numeric_limits<std::size_t>::max()) firstNanIdx = i;
    }
    std::string sx = (std::isfinite(x) ? std::to_string(x) : "null");
    std::string sy = (std::isfinite(y) ? std::to_string(y) : "null");
    if (firstAdded) json << ",";
    json << "{\"x\":" << sx << ",\"y\":" << sy << "}";
    firstAdded = true;
  }
  json << "],";

  json << "\"bodies\":[";
  const auto& rects = world.get_rects();
  for (std::size_t i = 0; i < rects.size(); ++i) {
    if (i) json << ",";
    json << "{\"springs\":[";
    const auto& springs = rects[i].get_springs();
    for (std::size_t j = 0; j < springs.size(); ++j) {
      if (j) json << ",";
      json << "[" << springs[j].get_i() << "," << springs[j].get_j() << "]";
    }
    json << "],\"boundary\":[";
    const auto& boundary = rects[i].get_boundary();
    for (std::size_t j = 0; j < boundary.size(); ++j) {
      if (j) json << ",";
      json << boundary[j];
    }
    json << "]}";
  }
  json << "]}";
  
  return json.str();
}