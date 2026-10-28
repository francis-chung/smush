#include <vector>
#include <cmath>

using namespace std;

class Vec2 {
private:
  double x, y;

public:
  Vec2(double x = 0.0, double y = 0.0): x(x), y(y) {}

  void set(Vec2 v) { x = v.x, y = v.y; }
  
  friend Vec2 operator+(const Vec2& v1, const Vec2& v2) { return Vec2(v1.x+v2.x, v1.y+v2.y); }
  friend Vec2 operator-(const Vec2& v1, const Vec2& v2) { return Vec2(v1.x-v2.x, v1.y-v2.y); }
  friend Vec2 operator*(double c, const Vec2& v) { return Vec2(c*v.x, c*v.y); }
  friend Vec2 operator*(const Vec2& v, double c) { return c * v; }
  Vec2& operator+=(const Vec2& v) { x += v.x; y += v.y; return *this; }
  Vec2& operator-=(const Vec2& v) { x -= v.x; y -= v.y; return *this; }
  Vec2& operator*=(const Vec2& v) { x *= v.x; y *= v.y; return *this; }
};

class Particle {
private:
  Vec2 pos, vel, acc;

public:
  Particle(Vec2 p, Vec2 v, Vec2 a): pos(p), vel(v), acc(a) {}

  const Vec2& get_pos() const { return pos; }
  const Vec2& get_vel() const { return vel; }
  const Vec2& get_acc() const { return acc; }

  void integrate(double dt, double ld) {
    pos += vel*dt + 0.5*acc*dt*dt;
    vel += acc*dt;
    vel *= pow(ld, dt);
  }
};

class World {
private:
  vector<Particle> particles;
  double time;
  double linear_damping;

public:
  World(vector<Particle> p, double ld): particles(p), time(0), linear_damping(ld) {}

  void update(double t) {
    double dt = t - time;
    for (Particle& p : particles) {
      p.integrate(dt, linear_damping);
    }
    time = t;
  }
};