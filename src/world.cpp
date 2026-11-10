#pragma once

#include "mathlib.cpp"
#include "particle.cpp"
#include "softrect.cpp"
#include "collision.cpp"

#include <iostream>

class World {
private:
  std::vector<Particle> particles;
  std::vector<SoftRect> rects;
  CollisionManager colls;
  double linearDamping;
  Vec2 lower_bounds, upper_bounds;
  inline static const Vec2 g = Vec2(0, -9.81);

public:
  World(double ld, Vec2 lower_bounds = Vec2(-1e9, -1e9), Vec2 upper_bounds = (1e9, 1e9)) : 
    particles(), rects(), colls(particles, rects), linearDamping(ld), lower_bounds(lower_bounds), upper_bounds(upper_bounds) {}

  const std::vector<Particle>& get_particles() const { return particles; }
  const std::vector<SoftRect>& get_rects() const { return rects; }

  void add_rect(Vec2 origin, double w, double h, double mass, double restitution, int subdiv) {
    rects.push_back(SoftRect(particles, origin, w, h, mass, restitution, subdiv));
  }
  void add_rect(SoftRect r) {
    rects.push_back(r);
  }
  
  void pin_particle(SoftRect& rect, int i, int j) { 
    if (i < 0 || i >= rect.get_nx() || j < 0 || j >= rect.get_ny()) { return; }
    particles[rect.get_particle(i, j)].pin(); 
  }

  void update(double dt) {
    for (Particle& p : particles) {
      p.set_force_acc((0, 0));
      p.add_force(g * p.get_mass());
    }
    for (SoftRect& r : rects) {
      r.update_springs(particles);
    }
    for (Particle& p : particles) {
      p.integrate(dt, linearDamping);
      p.boundary_collision(lower_bounds, upper_bounds);
    }
    colls.update();
  }

  void debug_particles() {
    for (int i = 0; i < (int)particles.size(); ++i) {
      Vec2 pos = particles[i].get_pos();
      if (std::isnan(pos.get_x()) || std::isnan(pos.get_y())) {
        std::cerr << "Particle " << i << " has NaN position\n" << std::flush;
      }
    }
  }
};