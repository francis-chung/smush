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
  Vec2 externalForce{0.0, 0.0};

public: 
  virtual ~SoftBody() = default;

  const std::vector<std::size_t>& get_boundary() const { return boundary; }
  const std::vector<Spring>& get_springs() const { return springs; }  

  void set_external_force(const Vec2& f) { externalForce = f; }
  void update_springs(std::vector<Particle>&);
  void apply_external_force(std::vector<Particle>&);
  bool point_in_polygon(std::size_t, const std::vector<Particle>&);
  AABB get_aabb(const std::vector<Particle>&);
};