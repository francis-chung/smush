#pragma once

#include "mathlib.cpp"

#include <cassert>

class Particle {
private:
  Vec2 pos, vel;
  Vec2 forceAcc;
  double mass, invMass;
  double restitution;

public:
  Particle(Vec2 p, Vec2 v, double mass, double rest) : pos(p), vel(v), forceAcc(), mass(mass), restitution(rest) {
    assert(mass > 0);
    invMass = 1.0 / mass;
  }

  const Vec2& get_pos() const { return pos; }
  const Vec2& get_vel() const { return vel; }
  double get_mass() const { return mass; }
  double get_inv_mass() const { return invMass; }

  void set_force_acc(const Vec2& force) { forceAcc = force; }
  void add_force(const Vec2& force) { forceAcc += force; }
  void set_pos(const Vec2& p) { pos = p; }
  void set_vel(const Vec2& v) { vel = v; }

  void pin() { invMass = 0; vel = Vec2(0.0, 0.0); }
  void unpin() { invMass = 1.0 / mass; }

  void integrate(double dt, double ld) {
    Vec2 acc = forceAcc * invMass;
    vel += acc*dt;
    vel *= exp(-ld * dt);
    pos += vel*dt;
  }

  void boundary_collision(const Vec2& lower_bounds, const Vec2& upper_bounds) {
    double x = pos.get_x(), y = pos.get_y();
    double vx = vel.get_x(), vy = vel.get_y();
    double lowerX = lower_bounds.get_x(), lowerY = lower_bounds.get_y();
    double upperX = upper_bounds.get_x(), upperY = upper_bounds.get_y();
    if (x < lowerX) {
      set_pos({ lowerX, y });
      set_vel({ restitution * -vx, restitution * vy });
    } else if (x > upperX) {
      set_pos({ upperX, y });
      set_vel({ restitution * -vx, restitution * vy });
    }
    if (y < lowerY) {
      set_pos({ x, lowerY });
      set_vel({ restitution * vx, restitution * -vy });
    } else if (y > upperY) {
      set_pos({ x, upperY });
      set_vel({ restitution * vx, restitution * -vy });
    }
  }
};