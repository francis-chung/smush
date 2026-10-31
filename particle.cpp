#pragma once

#include "mathlib.cpp"

#include <cassert>

class Particle {
private:
  Vec2 pos, vel;
  Vec2 forceAcc;
  double mass, invMass;

public:
  Particle(Vec2 p, Vec2 v, double mass) : pos(p), vel(v), forceAcc(0.0), mass(mass) {
    assert(mass > 0);
    invMass = 1.0 / mass;
  }

  const Vec2& get_pos() const { return pos; }
  const Vec2& get_vel() const { return vel; }
  double get_mass() const { return mass; }
  double get_inv_mass() const { return invMass; }

  void set_force_acc(const Vec2& force) { forceAcc = force; }
  void add_force(const Vec2& force) { forceAcc += force; }
  void set_vel(const Vec2& v) { vel = v; }

  void pin() { invMass = 0; vel = (0.0, 0.0); }
  void unpin() { invMass = 1.0 / mass; }

  void integrate(double dt, double ld) {
    Vec2 acc = forceAcc * invMass;
    vel += acc*dt;
    vel *= pow(ld, dt);
    pos += vel*dt;
  }

  void boundary_collision(const Vec2& lower_bounds, const Vec2& upper_bounds) {
    double x = pos.get_x(), y = pos.get_y();
    if (x < lower_bounds.get_x() || x > upper_bounds.get_x()) set_vel({ vel.get_x(), -vel.get_y() });
    if (y < lower_bounds.get_y() || y > upper_bounds.get_y()) set_vel({ -vel.get_x(), vel.get_y() }); 
  }
};