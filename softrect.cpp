#pragma once

#include "mathlib.cpp"
#include "particle.cpp"
#include "spring.cpp"

class AABB {
private: 
  Vec2 min, max;

public: 
  AABB(Vec2 min, Vec2 max) : min(min), max(max) {}

  const Vec2& get_min() const { return min; }
  const Vec2& get_max() const { return max; }
};

class SoftRect {
private: 
  double w, h;
  double mass, restitution;
  int nx, ny;   // number of particles aligned to axes
  std::vector<std::vector<int>> grid;
  std::vector<int> boundary;
  std::vector<Spring> springs;

  void add_spring(std::vector<Particle>& particles, int i, int j, double stiff, double damp) {
    Vec2 d = particles[j].get_pos() - particles[i].get_pos();
    double len = d.norm();
    springs.push_back(Spring(i, j, len, stiff, damp));
  }

public:
  SoftRect(std::vector<Particle>& particles, Vec2 p, double wd, double hi, double ms, double rest, int subd) :
    w(wd), h(hi), mass(ms), restitution(rest), nx(subd + 1), ny(subd + 1)
  {
    double pm = mass / (nx * ny);
    double px = p.get_x(), py = p.get_y();

    for (int i = 0; i < nx; ++i) {
      grid.push_back(std::vector<int>());
      for (int j = 0; j < ny; ++j) {
        double x = px + (wd / subd) * i;
        double y = py + (hi / subd) * j;
        Vec2 pos = Vec2(x, y);
        Vec2 vel = Vec2(0, 0);
        Particle p = Particle(pos, vel, pm, restitution);
        particles.push_back(p);
        grid[(int)grid.size()-1].push_back((int)particles.size()-1);
      }
    }
    for (int i = 1; i < ny; ++i)   boundary.push_back(grid[nx-1][i]);
    for (int i = 0; i < nx; ++i)   boundary.push_back(grid[i][0]);
    for (int i = 1; i < nx; ++i)   boundary.push_back(grid[nx-1-i][ny-1]);
    for (int i = 1; i < ny-1; ++i) boundary.push_back(grid[0][ny-1-i]);
    
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

  int get_particle(int i, int j) const { return grid[i][j]; }
  int get_nx() const { return nx; }
  int get_ny() const { return ny; }

  void update_springs(std::vector<Particle>& particles) {
    for (Spring& s : springs) { s.apply_spring_force(particles); }
  }

  bool point_in_polygon(int pIndex, const std::vector<Particle>& particles) {
    double px = particles[pIndex].get_pos().get_x(), py = particles[pIndex].get_pos().get_y();
    bool inside = false;
    int sz = (int)boundary.size();
    for (int i = 0, j = sz-1; i < sz; j = i++) {
      double v1x = particles[i].get_pos().get_x(), v1y = particles[i].get_pos().get_y();
      double v2x = particles[j].get_pos().get_x(), v2y = particles[j].get_pos().get_y();
      if (v2y != v1y && (py-v1y)*(v2x-v1x)/(v2y-v1y) + v1x >= px) {
        inside = !inside;
      }
    }
    return inside;
  }

  AABB get_aabb(const std::vector<Particle>& particles) {
    double minX = particles[grid[0][0]].get_pos().get_x(), maxX = particles[grid[0][0]].get_pos().get_x();
    double minY = particles[grid[0][0]].get_pos().get_y(), maxY = particles[grid[0][0]].get_pos().get_y();
    for (int i = 0; i < nx; ++i) {
      for (int j = 0; j < ny; ++j) {
        double cx = particles[grid[nx][ny]].get_pos().get_x();
        double cy = particles[grid[nx][ny]].get_pos().get_y();
        minX = std::min(minX, cx); maxX = std::max(maxX, cx);
        minY = std::min(minY, cy); maxY = std::max(maxY, cy);
      }
    }
    Vec2 min_pos = Vec2(minX, minY);
    Vec2 max_pos = Vec2(maxX, maxY);
    AABB aabb = AABB(min_pos, max_pos);
    return aabb;
  }

  std::vector<int> point_candidates_in_polygon(const std::vector<Particle>& particles, const AABB& otherBox) {
    std::vector<int> points;
    double minX = otherBox.get_min().get_x(), minY = otherBox.get_min().get_y();
    double maxX = otherBox.get_max().get_x(), maxY = otherBox.get_max().get_y();
    for (int i = 0; i < nx; ++i) {
      for (int j = 0; j < ny; ++j) {
        int p = grid[nx][ny];
        int x = particles[p].get_pos().get_x();
        int y = particles[p].get_pos().get_y();
        if (x > minX && x < maxX && y > minY && y < maxY) points.push_back(p);
      }
    }
    return points;
  }
};
