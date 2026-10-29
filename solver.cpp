#include <vector>
#include <cmath>

using namespace std;

class Vec2 {
private:
  double x, y;

public:
  Vec2(double x = 0.0, double y = 0.0) : x(x), y(y) {}

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

  double normsq() { return x*x + y*y; }
  double norm() { return sqrt(norm()); }
  friend double dot(const Vec2& v1, const Vec2& v2) { return v1.x*v2.x + v1.y*v2.y; }
};

class Particle {
private:
  Vec2 pos, vel;
  Vec2 forceAcc;
  double mass, invMass;

public:
  Particle(Vec2 p, Vec2 v, double mass) : pos(p), vel(v), forceAcc(0.0), mass(mass), invMass(1.0/mass) {}

  const Vec2& get_pos() const { return pos; }
  const Vec2& get_vel() const { return vel; }
  double get_mass() const { return mass; }
  double get_inv_mass() const { return invMass; }

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
  int i, j;    // World's particle vector indices
  double restLength, stiffness, dampCoeff;

public: 
  Spring(int i, int j, double len, double st, double dc) : i(i), j(j), restLength(len), stiffness(st), dampCoeff(dc) {}

  void apply_spring_force(vector<Particle>& particles) {
    Particle& p1 = particles[i];
    Particle& p2 = particles[j];

    Vec2 dp = p1.get_pos() - p2.get_pos();
    double dist = dp.norm();
    if (dist < 1e-9) return;    // guard against dividing by zero
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
  vector<vector<int>> grid;
  vector<Spring> springs;

  void add_spring(vector<Particle>& particles, int i, int j, double stiff, double damp) {
    Vec2 d = particles[j].get_pos() - particles[i].get_pos();
    double len = d.norm();
    springs.push_back(Spring(i, j, len, stiff, damp));
  }

public:
  SoftRect(vector<Particle>& particles, Vec2 p, double wd, double hi, double ms, int subd) :
    w(wd), h(hi), mass(ms)
  {
    int nx = subd + 1, ny = subd + 1;
    double pm = mass / (nx * ny);
    double px = p.get_x(), py = p.get_y();

    for (int i = 0; i < nx; ++i) {
      grid.push_back(vector<int>());
      for (int j = 0; j < ny; ++j) {
        double x = px + (wd / subd) * i;
        double y = py + (hi / subd) * j;
        Vec2 pos = Vec2(x, y);
        Vec2 vel = Vec2(0, 0);
        Particle p = Particle(pos, vel, pm);
        particles.push_back(p);
        grid[(int)grid.size()-1].push_back((int)particles.size()-1);
      }
    }
    
    double strStiff = 1.0, strDamp = 0.9;
    double sheStiff = 0.3, sheDamp = 0.9;
    double benStiff = 0.3, benDamp = 0.9;
    for (int i = 0; i < nx; ++i) {
      for (int j = 0; j < ny; ++j) {
        int p1 = grid[i][j];
        if (j+1 < ny)              add_spring(particles, p1, grid[i][j+1], strStiff, strDamp); 
        if (i+1 < nx)              add_spring(particles, p1, grid[i+1][j], strStiff, strDamp); 
        if (i+1 < nx && j+1 < ny)  add_spring(particles, p1, grid[i+1][j+1], sheStiff, sheDamp); 
        if (i+1 < nx && j > 0)     add_spring(particles, p1, grid[i+1][j-1], sheStiff, sheDamp); 
        if (j+2 < ny)              add_spring(particles, p1, grid[i][j+2], benStiff, benDamp); 
        if (i+2 < nx)              add_spring(particles, p1, grid[i+2][j], benStiff, benDamp); 
      }
    }
  }

  const vector<Spring>& get_springs() const { return springs; }

  void update_springs(vector<Particle>& particles) {
    for (Spring& s : springs) { s.apply_spring_force(particles); }
  }
};

class World {
private:
  vector<Particle> particles;
  vector<SoftRect> rects;
  double time;
  double linearDamping;
  inline static const Vec2 g = Vec2(0, -9.81);

public:
  World(double ld) : particles(vector<Particle>()), rects(vector<SoftRect>()), time(0), linearDamping(ld) {}

  void add_rect(Vec2 origin, double w, double h, double mass, int subdiv) {
    rects.push_back(SoftRect(particles, origin, w, h, mass, subdiv));
  }

  void update(double t) {
    double dt = t - time;
    for (Particle& p : particles) {
      p.set_force_acc(Vec2(0, 0));
      p.add_force(g * p.get_mass());
    }
    for (SoftRect& r : rects) {
      r.update_springs(particles);
    }
    for (Particle& p : particles) {
      p.integrate(dt, linearDamping);
    }
    time = t;
  }
};