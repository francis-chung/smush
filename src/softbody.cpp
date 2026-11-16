#pragma once

#include "mathlib.cpp"
#include "spring.cpp"
#include "aabb.cpp"

#include <cfloat>

class SoftBody {
protected: 
  double mass, restitution;
  std::vector<std::size_t> particleIndices;
  std::vector<std::size_t> boundary;
  std::vector<Spring> springs;
  Vec2 externalForce;

  void add_spring(std::vector<Particle>& particles, std::size_t i, std::size_t j, double stiff, double damp) {
    Vec2 d = particles[j].get_pos() - particles[i].get_pos();
    double len = d.norm();
    springs.push_back(Spring(i, j, len, stiff, damp));
  }

  void pin_index(std::vector<Particle>& particles, std::size_t idx) { particles[idx].pin(); }

public: 
  virtual ~SoftBody() = default;

  const std::vector<std::size_t>& get_boundary() const { return boundary; }
  const std::vector<Spring>& get_springs() const { return springs; }
  void set_external_force(const Vec2& f) { externalForce = f; }
  
  void update_springs(std::vector<Particle>& particles) {
    for (Spring& s : springs) { s.apply_spring_force(particles); }
  }
  
  void apply_external_force(std::vector<Particle>& particles) {
    const Vec2 particleForce = externalForce / particleIndices.size();
    for (std::size_t idx : particleIndices) { particles[idx].add_force(particleForce); }
  }
  
  void apply_external_impulse(std::vector<Particle>& particles, const Vec2& impulse) {
    const Vec2 particleImpulse = impulse / particleIndices.size();
    for (std::size_t idx : particleIndices) {
      Particle& p = particles[idx];
      const Vec2& v = p.get_vel();
      const double invM = p.get_inv_mass();
      particles[idx].set_vel(v + particleImpulse * invM);
    }
  }

  bool point_in_polygon(std::size_t pIndex, const std::vector<Particle>& particles) {
    double px = particles[pIndex].get_pos().get_x(), py = particles[pIndex].get_pos().get_y();
    bool inside = false;
    std::size_t sz = boundary.size();
    for (std::size_t i = 0, j = sz-1; i < sz; j = i++) {
      double v1x = particles[boundary[i]].get_pos().get_x(), v1y = particles[boundary[i]].get_pos().get_y();
      double v2x = particles[boundary[j]].get_pos().get_x(), v2y = particles[boundary[j]].get_pos().get_y();
      if (v2y != v1y && ((v1y > py) != (v2y > py)) && (py-v1y)*(v2x-v1x)/(v2y-v1y) + v1x >= px) {
        inside = !inside;
      }
    }
    return inside;
  }

  AABB get_aabb(const std::vector<Particle>& particles) {
    double minX = DBL_MIN, maxX = DBL_MAX;
    double minY = DBL_MIN, maxY = DBL_MAX;
    for (std::size_t idx : particleIndices) {
      double curX = particles[idx].get_pos().get_x();
      double curY = particles[idx].get_pos().get_y();
      minX = std::min(minX, curX); maxX = std::max(maxX, curX);
      minY = std::min(minY, curY); maxY = std::max(maxY, curY);
    }
    Vec2 minPos(minX, minY);
    Vec2 maxPos(maxX, maxY);
    return AABB(minPos, maxPos);
  }

  std::vector<std::size_t> point_candidates_in_body(const std::vector<Particle>& particles, const AABB& otherBox) {
    std::vector<std::size_t> points;
    double minX = otherBox.get_min().get_x(), minY = otherBox.get_min().get_y();
    double maxX = otherBox.get_max().get_x(), maxY = otherBox.get_max().get_y();
    for (std::size_t idx : particleIndices) {
      double x = particles[idx].get_pos().get_x();
      double y = particles[idx].get_pos().get_y();
      if (x > minX && x < maxX && y > minY && y < maxY) points.push_back(idx);
    }
    return points;
  }
};