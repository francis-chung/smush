#pragma once

#include "mathlib.cpp"
#include "particle.cpp"
#include "softrect.cpp"
#include "softcirc.cpp"
#include "collision.cpp"

#include <iostream>
#include <memory>

class World {
private:
  std::vector<Particle> particles;
  std::vector<std::unique_ptr<SoftBody>> bodies;
  CollisionManager colls;
  double linearDamping;
  Vec2 lower_bounds, upper_bounds;
  inline static const Vec2 g = Vec2(0, -9.81);

public:
  World(double ld, Vec2 lower_bounds = Vec2(-1e9, -1e9), Vec2 upper_bounds = Vec2(1e9, 1e9)) : 
    particles(), bodies(), colls(particles, bodies), linearDamping(ld), lower_bounds(lower_bounds), upper_bounds(upper_bounds) {}
  World(const World&) = delete;
  World& operator=(const World&) = delete;
  World(World&&) = delete;
  World& operator=(World&&) = delete;

  const std::vector<Particle>& get_particles() const { return particles; }
  const std::vector<std::unique_ptr<SoftBody>>& get_bodies() const { return bodies; }

  void add_rect(Vec2 origin, double w, double h, double mass, double restitution, std::size_t subdiv, double angleRad = 0.0) {
    bodies.push_back(std::make_unique<SoftRect>(particles, origin, w, h, mass, restitution, subdiv, angleRad));
  }

  void add_circ(Vec2 center, double r, double mass, double restitution, std::size_t subdiv) {
    bodies.push_back(std::make_unique<SoftCirc>(particles, center, r, subdiv, mass, restitution));
  }
  
  void set_body_force(std::size_t bodyIndex, const Vec2& force) {
    bodies.at(bodyIndex)->set_external_force(force);
  }

  void apply_impulse(std::size_t bodyIndex, const Vec2& impulse) {
    bodies.at(bodyIndex)->apply_external_impulse(particles, impulse);
  }

  void update(double dt) {
    for (Particle& p : particles) {
      p.set_force_acc(Vec2(0, 0));
      p.add_force(g * p.get_mass());
    }
    for (auto& b : bodies) {
      b->apply_external_force(particles);
      b->update_springs(particles);
    }
    for (Particle& p : particles) {
      p.integrate(dt, linearDamping);
    }
    colls.update();
    for (Particle& p : particles) {
      p.boundary_collision(lower_bounds, upper_bounds);
    }
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