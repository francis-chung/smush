#include <vector>
#include <cmath>

using namespace std;

class Vec2 {
private:
  double x, y;

public:
  Vec2(double x = 0.0, double y = 0.0): x(x), y(y) {}

  double get_x() const { return x; }
  double get_y() const { return y; }

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

  double norm() { return sqrt(x*x + y*y); }
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
    double dist = dp.norm();
    Vec2 dir = dp / dist;

    Vec2 springForce = -stiffness * (dist - restLength) * dir;
    p1.add_force(springForce);
    p2.add_force(-springForce);

    Vec2 dv = p1.get_vel() - p2.get_vel();
    Vec2 dampingForce = -dampCoeff * dot(dv, dir) * dir;
    p1.add_force(dampingForce);
    p2.add_force(-dampingForce);
  }
};

class SoftRect {
private: 
  double w, h;
  double mass;
  double density; // number of particles per edge length
  vector<vector<Particle>> particles;
  vector<Spring> springs;

  SoftRect(Vec2 p, double wd, double hi, double ms, double dens) {
    w = wd, h = hi;
    mass = ms;
    density = dens;
    particles = vector<vector<Particle>>();
    
    double pm = mass / (density * density);
    double px = p.get_x(), py = p.get_y();
    for (double x = px, i = 0; x <= px + wd; x += wd / density, ++i) {
      particles.push_back(vector<Particle>());
      for (double y = py; y <= py + hi; y += hi / density) {
        Vec2 pos = Vec2(x, y);
        Vec2 vel = Vec2(0, 0);
        Particle p = Particle(pos, vel, pm);
        particles[i].push_back(p);
      }
    }
    
    double strStiff = 1.0, strDamp = 0.9;
    double sheStiff = 0.3, sheDamp = 0.9;
    double benStiff = 0.3, benDamp = 0.9;
    int n = particles.size();
    int m = particles[0].size();
    for (int i = 0; i < n; ++i) {
      for (int j = 0; j < m; ++j) {
        const Particle& p1 = particles[i][j];
        if (j+1 < m) { _add_spring(p1, particles[i][j+1], strStiff, strDamp); }
        if (i+1 < n) { _add_spring(p1, particles[i+1][j], strStiff, strDamp); }
        if (i+1 < n && j+1 < m) { _add_spring(p1, particles[i+1][j+1], sheStiff, sheDamp); }
        if (i+1 < n && j > 0) { _add_spring(p1, particles[i+1][j-1], sheStiff, sheDamp); }
        if (j+2 < m) { _add_spring(p1, particles[i][j+2], benStiff, benDamp); }
        if (i+2 < n) { _add_spring(p1, particles[i+2][j], benStiff, benDamp); }
      }
    }
  }

  void _add_spring(const Particle& p1, const Particle& p2, double stiff, double damp) {
    Vec2 d = p2.get_pos() - p1.get_pos();
    double len = d.norm();
    springs.push_back(Spring(p1, p2, len, stiff, damp));
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