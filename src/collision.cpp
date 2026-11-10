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

  // moves the particle parallel to the normal of the nearest SoftRect edge
  void collision_displace(Particle& p, SoftRect& rect) {
    const std::vector<int>& boundary = rect.get_boundary();
    std::size_t sz = boundary.size();
    double minDist = __DBL_MAX__;
    const Particle *p1 = nullptr, *p2 = nullptr;
    Vec2 *minNormal = nullptr;
    for (size_t i = 0, j = sz-1; i < sz; j = i++) {
      const Particle& tmp1 = particles[boundary[i]], tmp2 = particles[boundary[j]];
      const Vec2& pos1 = tmp1.get_pos();
      const Vec2& pos2 = tmp2.get_pos();
      Vec2 normal = (pos1.get_y() - pos2.get_y(), pos2.get_x() - pos1.get_x());
      Vec2 pointProj = proj(normal, pos1 - p.get_pos());
      double dist = pointProj.norm();
      if (dist < minDist) {
        minDist = dist;
        p1 = &tmp1, p2 = &tmp2;
        minNormal = &normal;
      }
    }
    Vec2 displacement = minDist * *minNormal;
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
          if (rects[j].point_in_polygon(iPointsCandidates[k], particles)) {
            // handle collision: PIP for j particle inside i
          }
        }
        for (std::size_t k = 0; k < jPointsCandidates.size(); ++k) {
          if (rects[i].point_in_polygon(jPointsCandidates[k], particles)) {
            // handle collision: PIP for i particle inside j
          }
        }
      }
    }
  }
};