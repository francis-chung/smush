#include "world.cpp"

#include <fstream>
#include <sstream>

std::string world_to_json(const World& world) {
  std::ostringstream json;
  json << "{\"particles\":[";

  const auto& particles = world.get_particles();
  for (size_t i = 0; i < particles.size(); ++i) {
    const Vec2& pos = particles[i].get_pos();
    json << "{\"x\":" << pos.get_x() << ",\"y\":" << pos.get_y() << "}";
    if (i < particles.size() - 1) json << ",";
  }

  json << "],\"rects\":[";
  const auto& rects = world.get_rects();
  for (size_t i = 0; i < rects.size(); ++i) {
    json << "{\"nx\":" << rects[i].get_nx()
         << ",\"ny\":" << rects[i].get_ny()
         << ",\"start_idx\":" << rects[i].get_start_idx() << "}";
    if (i < rects.size() - 1) json << ",";
  }
  json << "]}";
  
  return json.str();
}