#pragma once

#include "mathlib.cpp"
#include "particle.cpp"
#include "softrect.cpp"

class World {
private:
  std::vector<Particle> particles;
  std::vector<SoftRect> rects;
  double linearDamping;
  Vec2 lower_bounds, upper_bounds;
  inline static const Vec2 g = Vec2(0, -9.81);

public:
  World(double ld, Vec2 lower_bounds, Vec2 upper_bounds) : 
    particles(std::vector<Particle>()), rects(std::vector<SoftRect>()), 
    linearDamping(ld), lower_bounds(lower_bounds), upper_bounds(upper_bounds) {}

  void add_rect(Vec2 origin, double w, double h, double mass, int subdiv) {
    rects.push_back(SoftRect(particles, origin, w, h, mass, subdiv));
  }
  
  void pin_particle(SoftRect& rect, int i, int j) { 
    if (i < 0 || i >= rect.get_nx() || j < 0 || j >= rect.get_ny()) { return; }
    particles[rect.get_particle(i, j)].pin(); 
  }

  void update(double dt) {
    for (Particle& p : particles) {
      p.set_force_acc(Vec2(0, 0));
      p.add_force(g * p.get_mass());
    }
    for (SoftRect& r : rects) {
      r.update_springs(particles);
    }
    for (Particle& p : particles) {
      p.integrate(dt, linearDamping);
      p.boundary_collision(lower_bounds, upper_bounds);
    }
  }
};