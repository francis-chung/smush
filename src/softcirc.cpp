#pragma once

#include "softbody.cpp"
#include "particle.cpp"

constexpr double PI = 3.14159265358979323846;

class SoftCirc : public SoftBody {
private: 
  double radius;
  std::size_t n;     // number of particles on circumference

public: 
  SoftCirc(std::vector<Particle>& particles, Vec2 c, double rad, std::size_t num, double ms, double rest) {
    assert(rad > 0);
    assert(num > 0);
    assert(ms > 0);
    assert(rest >= 0 && rest <= 1);

    radius = rad, n = num, mass = ms, restitution = rest;

    double pm = mass / (num + 1);
    Vec2 vel{0.0, 0.0};
    
    Particle center = Particle(c, vel, pm, rest);
    particleIndices.push_back(particles.size());
    particles.push_back(center);
    for (std::size_t i = 0; i < n; ++i) {
      double angle = 2.0 * PI / n;
      double localX = radius * std::cos(angle);
      double localY = radius * std::sin(angle);
      Vec2 pos = c + Vec2(localX, localY);
      Particle p = Particle(pos, vel, mass, restitution);
      particleIndices.push_back(particles.size());
      particles.push_back(p);
    }
  }
};