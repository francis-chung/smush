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
  World(double ld, Vec2 lower_bounds = Vec2(-1e9, -1e9), Vec2 upper_bounds = Vec2(1e9, 1e9)) : 
    particles(), rects(), colls(particles, rects), linearDamping(ld), lower_bounds(lower_bounds), upper_bounds(upper_bounds) {}

  const std::vector<Particle>& get_particles() const { return particles; }
  const std::vector<SoftRect>& get_rects() const { return rects; }

  void add_rect(Vec2 origin, double w, double h, double mass, double restitution, int subdiv, double angleRad = 0.0) {
    rects.push_back(SoftRect(particles, origin, w, h, mass, restitution, subdiv, angleRad));
  }
  void add_rect(SoftRect& r) {
    rects.push_back(r);
  }
  
  void pin_particle(SoftRect& rect, std::size_t i, std::size_t j) { 
    if (i >= rect.get_nx() || j >= rect.get_ny()) { return; }
    particles[rect.get_particle(i, j)].pin(); 
  }
  void set_rect_force(std::size_t rectIndex, const Vec2& force) {
    rects.at(rectIndex).set_external_force(force);
  }
  void apply_external_impulse(std::size_t rectIndex, const Vec2& impulse) {
    const SoftRect& r = rects.at(rectIndex);
    std::size_t nx = r.get_nx(), ny = r.get_ny();
    const Vec2 particleImpulse = impulse / (nx * ny);
    for (std::size_t i = 0; i < nx; ++i) {
      for (std::size_t j = 0; j < ny; ++j) {
        Particle& p = particles[r.get_particle(i, j)];
        const Vec2& v = p.get_vel();
        const double invM = p.get_inv_mass();
        particles[r.get_particle(i, j)].set_vel(v + particleImpulse * invM);
      }
    }
  }

  void update(double dt) {
    for (Particle& p : particles) {
      p.set_force_acc(Vec2(0, 0));
      p.add_force(g * p.get_mass());
    }
    for (SoftRect& r : rects) {
      r.apply_external_force(particles);
      r.update_springs(particles);
    }
    for (Particle& p : particles) {
      p.integrate(dt, linearDamping);
      p.boundary_collision(lower_bounds, upper_bounds);
    }
    colls.update();
  }

  void debug_particles() {
    for (std::size_t i = 0; i < particles.size(); ++i) {
      Vec2 pos = particles[i].get_pos();
      if (std::isnan(pos.get_x()) || std::isnan(pos.get_y())) {
        std::cerr << "Particle " << i << " has NaN position\n" << std::flush;
      }
    }
  }
};