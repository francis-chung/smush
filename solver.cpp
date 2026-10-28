#include <vector>

using namespace std;

class Vec2 {
private:
  double x, y;

public:
  Vec2(double x, double y): x(x), y(y) {}

  void set(Vec2 v) {
    x = v.x, y = v.y;
  }
  
  friend Vec2 operator+(const Vec2& v1, const Vec2& v2) {
    return Vec2(v1.x, v1.y);
  }

  friend Vec2 operator*(double c, const Vec2& v) {
    return Vec2(c*v.x, c*v.y);
  }

  friend Vec2 operator*(const Vec2& v, double c) {
    return c * v;
  }
};

class Particle {
private:
  Vec2 pos, vel, acc;

public:
  Particle(Vec2 p, Vec2 v, Vec2 a): pos(p), vel(v), acc(a) {}

  const Vec2& get_pos() {
    return pos;
  }

  const Vec2& get_vel() {
    return vel;
  }

  const Vec2& get_acc() {
    return acc;
  }

  void set_pos(Vec2 v) {
    pos.set(v);
  }

  void set_vel(Vec2 v) {
    vel.set(v);
  }
  
  void set_acc(Vec2 v) {
    acc.set(v);
  }
};

class World {
private:
  vector<Particle> particles;
  double time;

public:
  World(vector<Particle> p): particles(p), time(0) {}

  void update(double t) {
    for (Particle& p : particles) {
      double dt = t - time;
      p.set_pos(p.get_pos() + p.get_vel() + 0.5*p.get_acc()*dt);
      p.set_vel(p.get_vel() + p.get_acc()*dt);
    }
  }
};