#pragma once

#include "mathlib.cpp"
#include "particle.cpp"
#include "spring.cpp"

class SoftRect {
private: 
  double w, h;
  double mass;
  std::vector<std::vector<int>> grid;
  std::vector<Spring> springs;

  void add_spring(std::vector<Particle>& particles, int i, int j, double stiff, double damp) {
    Vec2 d = particles[j].get_pos() - particles[i].get_pos();
    double len = d.norm();
    springs.push_back(Spring(i, j, len, stiff, damp));
  }

public:
  SoftRect(std::vector<Particle>& particles, Vec2 p, double wd, double hi, double ms, int subd) :
    w(wd), h(hi), mass(ms)
  {
    int nx = subd + 1, ny = subd + 1;
    double pm = mass / (nx * ny);
    double px = p.get_x(), py = p.get_y();

    for (int i = 0; i < nx; ++i) {
      grid.push_back(std::vector<int>());
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

  const std::vector<Spring>& get_springs() const { return springs; }

  void update_springs(std::vector<Particle>& particles) {
    for (Spring& s : springs) { s.apply_spring_force(particles); }
  }
};
