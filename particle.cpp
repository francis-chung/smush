#pragma once

#include "mathlib.cpp"

#include <cassert>

class Particle {
private:
  Vec2 pos, vel;
  Vec2 forceAcc;
  double mass, invMass;
  bool isPinned = false;

public:
  Particle(Vec2 p, Vec2 v, double mass) : pos(p), vel(v), forceAcc(0.0), mass(mass) {
    assert(mass > 0);
    invMass = 1.0 / mass;
  }

  const Vec2& get_pos() const { return pos; }
  const Vec2& get_vel() const { return vel; }
  double get_mass() const { return mass; }
  double get_inv_mass() const { return invMass; }
  bool get_pinned() const { return isPinned; }

  void set_force_acc(const Vec2& force) { forceAcc = force; }
  void add_force(const Vec2& force) { forceAcc += force; }

  void pin() { isPinned = true; }
  void unpin() { isPinned = false; }

  void integrate(double dt, double ld) {
    Vec2 acc = forceAcc * invMass;
    pos += vel*dt + 0.5*acc*dt*dt;
    vel += acc*dt;
    vel *= pow(ld, dt);
  }
};