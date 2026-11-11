#pragma once

#include "particle.cpp"
#include "softrect.cpp"

#include <vector>
#include <algorithm>

class CollisionManager {
private: 
  std::vector<Particle>& particles;
  std::vector<SoftRect>& rects;

public: 
  CollisionManager(std::vector<Particle>& parts, std::vector<SoftRect>& recs) : particles(parts), rects(recs) {}

  // moves the particle parallel to the normal of the nearest SoftRect edge
  void collision_displace(Particle& p, SoftRect& rect) {
    const std::vector<int>& boundary = rect.get_boundary();
    std::size_t sz = boundary.size();
    Vec2 displacement{1e9, 1e9};
    for (size_t i = 0, j = sz-1; i < sz; j = i++) {
      const Particle& tmp1 = particles[boundary[i]], tmp2 = particles[boundary[j]];
      const Vec2& pos = p.get_pos();
      const Vec2& a = tmp1.get_pos(), b = tmp2.get_pos();

      // calculates position along edge from a to b where closest point would be
      Vec2 edge = b - a;
      double t = dot(pos - a, edge) / edge.normsq();
      t = std::clamp(t, 0.0, 1.0);
      Vec2 closest = a + t * edge;
      displacement = std::min({displacement, closest - pos}, [](const Vec2& a, const Vec2& b) { return a.norm() < b.norm(); });
    }
    p.set_pos(p.get_pos() + displacement);
  }
  
  void update() {
    std::size_t n = rects.size();
    std::vector<AABB> aabbs(n);
    for (std::size_t i = 0; i < n; ++i) {
      aabbs[i] = rects[i].get_aabb(particles);
    }
    for (std::size_t i = 0; i < n; ++i) {
      for (std::size_t j = i+1; j < n; ++j) {
        if (!rects[i].colliding_aabb(aabbs[i], aabbs[j])) continue;
        std::vector<int> iPointsCandidates = rects[i].point_candidates_in_polygon(particles, aabbs[j]);
        std::vector<int> jPointsCandidates = rects[j].point_candidates_in_polygon(particles, aabbs[i]);
        for (std::size_t k = 0; k < iPointsCandidates.size(); ++k) {
          int candidate = iPointsCandidates[k];
          if (rects[j].point_in_polygon(candidate, particles)) {
            // handle collision: PIP for j particle inside i
            collision_displace(particles[candidate], rects[j]);
          }
        }
        for (std::size_t k = 0; k < jPointsCandidates.size(); ++k) {
          int candidate = jPointsCandidates[k];
          if (rects[i].point_in_polygon(candidate, particles)) {
            // handle collision: PIP for i particle inside j
            collision_displace(particles[candidate], rects[i]);
          }
        }
      }
    }
  }
};