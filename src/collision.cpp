#pragma once

#include "particle.cpp"
#include "softrect.cpp"

#include <vector>

class CollisionManager {
private: 
  std::vector<Particle>& particles;
  std::vector<SoftRect>& rects;

public: 
  CollisionManager(std::vector<Particle>& parts, std::vector<SoftRect>& recs) : particles(parts), rects(recs) {}

  void update() {
    size_t n = rects.size();
    std::vector<AABB> aabbs(n);
    for (size_t i = 0; i < n; ++i) {
      aabbs[i] = rects[i].get_aabb(particles);
    }
    for (size_t i = 0; i < n; ++i) {
      for (size_t j = i+1; j < n; ++j) {
        if (!rects[i].colliding_aabb(aabbs[i], aabbs[j])) continue;
        std::vector<int> iPointsCandidates = rects[i].point_candidates_in_polygon(particles, aabbs[j]);
        std::vector<int> jPointsCandidates = rects[j].point_candidates_in_polygon(particles, aabbs[i]);
        for (size_t k = 0; k < iPointsCandidates.size(); ++k) {
          if (rects[j].point_in_polygon(iPointsCandidates[k], particles)) {
            // handle collision: PIP for j particle inside i
          }
        }
        for (size_t k = 0; k < jPointsCandidates.size(); ++k) {
          if (rects[i].point_in_polygon(jPointsCandidates[k], particles)) {
            // handle collision: PIP for i particle inside j
          }
        }
      }
    }
  }
};