#pragma once

#include "mathlib.cpp"
#include "particle.cpp"
#include "softrect.cpp"

class World {
private:
  std::vector<Particle> particles;
  std::vector<SoftRect> rects;
  double time;
  double linearDamping;
  inline static const Vec2 g = Vec2(0, -9.81);

public:
  World(double ld) : particles(std::vector<Particle>()), rects(std::vector<SoftRect>()), time(0), linearDamping(ld) {}

  void add_rect(Vec2 origin, double w, double h, double mass, int subdiv) {
    rects.push_back(SoftRect(particles, origin, w, h, mass, subdiv));
  }

  void update(double t) {
    double dt = t - time;
    for (Particle& p : particles) {
      p.set_force_acc(Vec2(0, 0));
      p.add_force(g * p.get_mass());
    }
    for (SoftRect& r : rects) {
      r.update_springs(particles);
    }
    for (Particle& p : particles) {
      p.integrate(dt, linearDamping);
    }
    time = t;
  }
};