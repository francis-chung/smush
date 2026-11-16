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
      boundary.push_back(particles.size());
      particles.push_back(p);
    }

    double rimStiff = 25.0, rimDamp = 0.5;
    double skpStiff = 20.0, skpDamp = 0.5;
    double ctrStiff = 45.0, ctrDamp = 0.6;
    for (std::size_t i = 0; i < n; ++i) {
      std::size_t idx = boundary[i];
      add_spring(particles, idx, boundary[(i+1)%n], rimStiff, rimDamp);
      add_spring(particles, idx, boundary[(i+2)%n], skpStiff, skpDamp);
      add_spring(particles, idx, particleIndices.front(), ctrStiff, ctrDamp);
    }
  }
};