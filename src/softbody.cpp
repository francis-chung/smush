#pragma once

#include "mathlib.cpp"
#include "spring.cpp"
#include "aabb.cpp"

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

  virtual void apply_external_force(std::vector<Particle>&) = 0;
  virtual void apply_external_impulse(std::vector<Particle>&, const Vec2&) = 0;
  virtual bool point_in_polygon(std::size_t, const std::vector<Particle>&) = 0;
  virtual AABB get_aabb(const std::vector<Particle>&) = 0;
  virtual std::vector<std::size_t> point_candidates_in_body(const std::vector<Particle>&, const AABB&) = 0;
};