#pragma once

#include "particle.cpp"
#include "softrect.cpp"

#include <vector>
#include <algorithm>
#include <memory>

class CollisionManager {
private: 
  std::vector<Particle>& particles;
  std::vector<std::unique_ptr<SoftRect>>& rects;

public: 
  CollisionManager(std::vector<Particle>& parts, std::vector<std::unique_ptr<SoftRect>>& recs) : particles(parts), rects(recs) {}

  void collision_resolution(Particle& p, SoftRect& rect, double restitution) {
    // find the nearest edge of the polygon to the point and prep displacement
    const std::vector<std::size_t>& boundary = rect.get_boundary();
    std::size_t sz = boundary.size();
    Vec2 displacement{1e9, 1e9};
    Particle *a = nullptr, *b = nullptr;
    double tReq = 0.0;
    for (size_t i = 0, j = sz-1; i < sz; j = i++) {
      Particle& tmp1 = particles[boundary[i]];
      Particle& tmp2 = particles[boundary[j]];
      const Vec2& pos = p.get_pos();
      const Vec2& pa = tmp1.get_pos(), pb = tmp2.get_pos();

      // calculates position along edge from a to b where closest point would be
      Vec2 edge = pb - pa;
      if (edge.normsq() < 1e-9) continue;
      double t = dot(pos - pa, edge) / edge.normsq();
      t = std::clamp(t, 0.0, 1.0);
      Vec2 closest = pa + t * edge;
      if ((closest - pos).norm() < displacement.norm()) {
        displacement = closest - pos;
        a = &tmp1, b = &tmp2;
        tReq = t;
      }
    }
    if (a != nullptr) {
      // apply displacement to displace the point outside the polygon
      if (p.get_inv_mass()) p.set_pos(p.get_pos() + displacement);

      // calculate impulses after collision for point and vertices of polygon edge
      double depth = displacement.norm();
      if (depth < 1e-9) return;
      // unit normal from point to polygon edge
      Vec2 n = displacement / depth;

      double weightA = 1.0 - tReq;
      double weightB = tReq;
      Vec2 edgeVel = weightA * a->get_vel() + weightB * b->get_vel();
      double vn = dot(p.get_vel() - edgeVel, n);
      if (vn >= 0.0) return;  // already separating or not approaching

      double denom = p.get_inv_mass() + (weightA * weightA * a->get_inv_mass()) + (weightB * weightB * b->get_inv_mass());
      if (denom <= 0.0) return;

      // magnitude of impulse on either side of the collision
      double j = -(1.0 + restitution) * vn / denom;
      Vec2 J = j * n;
      if (p.get_inv_mass()) p.set_vel(p.get_vel() + p.get_inv_mass() * J);
      if (a->get_inv_mass()) a->set_vel(a->get_vel() - weightA * a->get_inv_mass() * J);
      if (b->get_inv_mass()) b->set_vel(b->get_vel() - weightB * b->get_inv_mass() * J);
    }
  }
  
  void update() {
    std::size_t n = rects.size();
    
    double restitution = 0.7;
    for (std::size_t i = 0; i < n; ++i) {
      for (std::size_t j = i+1; j < n; ++j) {
        // bounding boxes can change after particle collisions are applied
        AABB iBB = rects[i]->get_aabb(particles);
        AABB jBB = rects[j]->get_aabb(particles);
        if (!iBB.colliding_aabb(jBB)) continue;

        std::vector<std::size_t> iPointsCandidates = rects[i]->point_candidates_in_polygon(particles, jBB);
        std::vector<std::size_t> jPointsCandidates = rects[j]->point_candidates_in_polygon(particles, iBB);
        for (std::size_t k = 0; k < iPointsCandidates.size(); ++k) {
          std::size_t candidate = iPointsCandidates[k];
          if (rects[j]->point_in_polygon(candidate, particles)) {
            collision_resolution(particles[candidate], *rects[j], restitution);
            // regenerate candidates after collision resolution for one pair
            iPointsCandidates = rects[i]->point_candidates_in_polygon(particles, jBB);
            jPointsCandidates = rects[j]->point_candidates_in_polygon(particles, iBB);
          }
        }

        for (std::size_t k = 0; k < jPointsCandidates.size(); ++k) {
          std::size_t candidate = jPointsCandidates[k];
          if (rects[i]->point_in_polygon(candidate, particles)) {
            collision_resolution(particles[candidate], *rects[i], restitution);
            iPointsCandidates = rects[i]->point_candidates_in_polygon(particles, jBB);
            jPointsCandidates = rects[j]->point_candidates_in_polygon(particles, iBB);
          }
        }
      }
    }
  }
};