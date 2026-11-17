#pragma once

#include "softbody.cpp"
#include "particle.cpp"

class SoftCirc : public SoftBody {
private: 
  double radius;
  std::size_t n;     // number of particles on circumference

public: 
  SoftCirc(std::vector<Particle>& particles, Vec2 c, double rad, std::size_t num, double ms, double rest, double angleRad = 0.0) {
    assert(rad > 0);
    assert(num > 0);
    assert(ms > 0);
    assert(rest >= 0 && rest <= 1);

    radius = rad, n = num, mass = ms, restitution = rest;

    double pm = mass / (num + 1);
    Vec2 vel{0.0, 0.0};
    const double pi = std::acos(-1.0);
    
    Particle center = Particle(c, vel, pm, restitution);
    particleIndices.push_back(particles.size());
    particles.push_back(center);
    for (std::size_t i = 0; i < n; ++i) {
      double angle = (2.0 * pi * i / n) + angleRad;
      double localX = radius * std::cos(angle);
      double localY = radius * std::sin(angle);
      Vec2 pos = c + Vec2(localX, localY);
      Particle p = Particle(pos, vel, pm, restitution);
      particleIndices.push_back(particles.size());
      boundary.push_back(particles.size());
      particles.push_back(p);
    }

    double rimStiff = 30.0, rimDamp = 0.5;
    double skpStiff = 25.0, skpDamp = 0.5;
    double ctrStiff = 50.0, ctrDamp = 0.6;
    for (std::size_t i = 0; i < n; ++i) {
      std::size_t idx = boundary[i];
      add_spring(particles, idx, boundary[(i+1)%n], rimStiff, rimDamp);
      add_spring(particles, idx, boundary[(i+2)%n], skpStiff, skpDamp);
      add_spring(particles, idx, particleIndices.front(), ctrStiff, ctrDamp);
    }
  }

  std::size_t get_particle(bool center, std::size_t pos = -1) {
    assert(center || pos != -1);
    if (center) return particleIndices.front();
    return particleIndices[pos + 1];
  }
  
  void pin(std::vector<Particle>& particles, bool center, std::size_t pos = -1) {
    particles[get_particle(center, pos)].pin();
  }
};