#include <vector>
#include <cmath>

using namespace std;

class Vec2 {
private:
  double x, y;

public:
  Vec2(double x = 0.0, double y = 0.0): x(x), y(y) {}

  void set(Vec2 v) { x = v.x, y = v.y; }
  
  Vec2 operator-() const { return Vec2(-x, -y); }
  friend Vec2 operator+(const Vec2& v1, const Vec2& v2) { return Vec2(v1.x+v2.x, v1.y+v2.y); }
  friend Vec2 operator-(const Vec2& v1, const Vec2& v2) { return Vec2(v1.x-v2.x, v1.y-v2.y); }
  friend Vec2 operator*(double c, const Vec2& v) { return Vec2(c*v.x, c*v.y); }
  friend Vec2 operator*(const Vec2& v, double c) { return c * v; }
  friend Vec2 operator/(const Vec2& v, double c) { return v * (1.0/c); }
  Vec2& operator+=(const Vec2& v) { x += v.x; y += v.y; return *this; }
  Vec2& operator-=(const Vec2& v) { x -= v.x; y -= v.y; return *this; }
  Vec2& operator*=(const Vec2& v) { x *= v.x; y *= v.y; return *this; }
  Vec2& operator*=(double c) { x *= c; y *= c; return *this; }

  double norm() { return x*x + y*y; }
  double length() { return sqrt(norm()); }
  friend double dot(const Vec2& v1, const Vec2& v2) { return v1.x*v2.x + v1.y*v2.y; }
};

class Particle {
private:
  Vec2 pos, vel;
  Vec2 forceAcc;
  double mass, invMass;

public:
  Particle(Vec2 p, Vec2 v, double mass): pos(p), vel(v), mass(mass), invMass(1.0/mass), forceAcc(0.0) {}

  const Vec2& get_pos() const { return pos; }
  const Vec2& get_vel() const { return vel; }
  const double get_mass() const { return mass; }
  const double get_inv_mass() const { return invMass; }

  void set_force_acc(const Vec2& force) { forceAcc = force; }
  void add_force(const Vec2& force) { forceAcc += force; }

  void integrate(double dt, double ld) {
    Vec2 acc = forceAcc * invMass;
    pos += vel*dt + 0.5*acc*dt*dt;
    vel += acc*dt;
    vel *= pow(ld, dt);
  }
};

class Spring {
private: 
  Particle p1, p2;
  double restLength, stiffness, dampCoeff;

public: 
  Spring(Particle p1, Particle p2, double len, double st, double dc): p1(p1), p2(p2), restLength(len), stiffness(st), dampCoeff(dc) {}

  void apply_spring_force() {
    Vec2 dp = p1.get_pos() - p2.get_pos();
    double dist = dp.length();
    Vec2 dir = dp / dist;

    Vec2 spring_force = -stiffness * (dist - restLength) * dir;
    p1.add_force(spring_force);
    p2.add_force(-spring_force);

    Vec2 dv = p1.get_vel() - p2.get_vel();
    Vec2 damping_force = -dampCoeff * dot(dv, dir) * dir;
    p1.add_force(damping_force);
    p2.add_force(-damping_force);
  }
};

class World {
private:
  vector<Particle> particles;
  double time;
  double linearDamping;
  inline static const Vec2 g = Vec2(0, -9.81);

public:
  World(vector<Particle> p, double ld): particles(p), time(0), linearDamping(ld) {}

  void update(double t) {
    double dt = t - time;
    for (Particle& p : particles) {
      p.set_force_acc(0.0);
      p.add_force(g * p.get_mass());
    }
    for (Particle& p : particles) {
      p.integrate(dt, linearDamping);
    }
    time = t;
  }
};