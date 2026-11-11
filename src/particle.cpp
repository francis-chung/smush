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

  // pinned particles should still be moved back in bounds
  void boundary_collision(const Vec2& lower_bounds, const Vec2& upper_bounds) {
    double x = pos.get_x(), y = pos.get_y();
    double vx = vel.get_x(), vy = vel.get_y();
    double lowerX = lower_bounds.get_x(), lowerY = lower_bounds.get_y();
    double upperX = upper_bounds.get_x(), upperY = upper_bounds.get_y();
    
    double newX = x, newY = y;
    double newVx = vx, newVy = vy;
    if (x < lowerX) {
      newX = lowerX;
      if (vx < 0) newVx = -restitution * vx;
    } else if (x > upperX) {
      newX = upperX; 
      if (vx > 0) newVx = -restitution * vx;
    }
    if (y < lowerY) {
      newY = lowerY;
      if (vy < 0) newVy = -restitution * vy;
    } else if (y > upperY) {
      newY = upperY;
      if (vy > 0) newVy = -restitution * vy;
    }
    set_pos({ newX, newY });
    set_vel({ newVx, newVy });
  }
};