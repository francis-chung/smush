#pragma once

#include "mathlib.cpp"
#include "particle.cpp"

#include <vector>

class Spring {
private: 
  std::size_t i, j;    // World's particle vector indices
  double restLength, stiffness, dampCoeff;

public: 
  Spring(std::size_t i, std::size_t j, double len, double st, double dc) : 
    i(i), j(j), restLength(len), stiffness(st), dampCoeff(dc) {}

  void apply_spring_force(std::vector<Particle>& particles) {
    Particle& p1 = particles[i];
    Particle& p2 = particles[j];

    Vec2 dp = p1.get_pos() - p2.get_pos();
    double dist = dp.norm();
    if (dist < 1e-9) return;    // guard against dividing by zero
    Vec2 dir = dp / dist;

    Vec2 springForce = -stiffness * (dist - restLength) * dir;
    p1.add_force(springForce); 
    p2.add_force(-springForce); 

    Vec2 dv = p1.get_vel() - p2.get_vel();
    Vec2 dampingForce = -dampCoeff * dot(dv, dir) * dir;
    p1.add_force(dampingForce);
    p2.add_force(-dampingForce);
  }
};