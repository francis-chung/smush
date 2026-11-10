#pragma once

#include "mathlib.cpp"
#include "particle.cpp"
#include "spring.cpp"

class AABB {
private: 
  Vec2 min, max;

public: 
  AABB() : min(), max() {}
  AABB(Vec2 min, Vec2 max) : min(min), max(max) {}

  const Vec2& get_min() const { return min; }
  const Vec2& get_max() const { return max; }
};

class SoftRect {
private: 
  double w, h;
  double mass, restitution;
  std::size_t nx, ny;                             // number of particles aligned to axes
  std::size_t partStartIdx;                       // first index of particles part of SoftRect 
  std::vector<std::vector<int>> grid;
  std::vector<int> boundary;
  std::vector<Spring> springs;
  Vec2 externalForce{0.0, 0.0};

  void add_spring(std::vector<Particle>& particles, int i, int j, double stiff, double damp) {
    Vec2 d = particles[j].get_pos() - particles[i].get_pos();
    double len = d.norm();
    springs.push_back(Spring(i, j, len, stiff, damp));
  }

public:
  SoftRect(std::vector<Particle>& particles, Vec2 p, double wd, double hi, double ms, double rest, int subd, double angleRad = 0.0) :
    w(wd), h(hi), mass(ms), restitution(rest), nx(subd + 1), ny(subd + 1), partStartIdx(particles.size())
  {
    double pm = mass / (nx * ny);

    const Vec2 center = p + Vec2(wd * 0.5, hi * 0.5);
    const double c = std::cos(angleRad);
    const double s = std::sin(angleRad);

    for (std::size_t i = 0; i < nx; ++i) {
      grid.push_back(std::vector<int>());
      for (std::size_t j = 0; j < ny; ++j) {
        // get coordinates relative to center first
        const double localX = (wd / subd) * i - wd * 0.5;
        const double localY = (hi / subd) * j - hi * 0.5;
        const Vec2 rotatedFromCenter(
          c * localX - s * localY, 
          s * localX + c * localY
        );

        Vec2 pos = center + rotatedFromCenter;
        Vec2 vel(0.0, 0.0);
        Particle p = Particle(pos, vel, pm, restitution);
        particles.push_back(p);
        grid[i].push_back((int)particles.size()-1);
      }
    }
    for (std::size_t i = 1; i < ny; ++i)   boundary.push_back(grid[nx-1][i]);
    for (std::size_t i = 0; i < nx; ++i)   boundary.push_back(grid[i][0]);
    for (std::size_t i = 1; i < nx; ++i)   boundary.push_back(grid[nx-1-i][ny-1]);
    for (std::size_t i = 1; i < ny-1; ++i) boundary.push_back(grid[0][ny-1-i]);
    
    double strStiff = 30.0, strDamp = 0.5;
    double sheStiff = 20.0, sheDamp = 0.5;
    double benStiff = 20.0, benDamp = 0.6;
    for (std::size_t i = 0; i < nx; ++i) {
      for (std::size_t j = 0; j < ny; ++j) {
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
  int get_start_idx() const { return partStartIdx; }

  void set_external_force(const Vec2& f) { externalForce = f; }
  void apply_external_force(std::vector<Particle>& particles) {
    const Vec2 particleForce = externalForce / (nx * ny);
    for (std::size_t i = 0; i < nx; ++i) {
      for (std::size_t j = 0; j < ny; ++j) {
        particles[grid[i][j]].add_force(particleForce);
      }
    }
  }

  void update_springs(std::vector<Particle>& particles) {
    for (Spring& s : springs) { s.apply_spring_force(particles); }
  }

  bool point_in_polygon(int pIndex, const std::vector<Particle>& particles) {
    double px = particles[pIndex].get_pos().get_x(), py = particles[pIndex].get_pos().get_y();
    bool inside = false;
    std::size_t sz = boundary.size();
    for (std::size_t i = 0, j = sz-1; i < sz; j = i++) {
      double v1x = particles[boundary[i]].get_pos().get_x(), v1y = particles[boundary[i]].get_pos().get_y();
      double v2x = particles[boundary[j]].get_pos().get_x(), v2y = particles[boundary[j]].get_pos().get_y();
      if (v2y != v1y && ((v1y > py) != (v2y > py)) && (py-v1y)*(v2x-v1x)/(v2y-v1y) + v1x >= px) {
        inside = !inside;
      }
    }
    return inside;
  }

  AABB get_aabb(const std::vector<Particle>& particles) {
    double minX = particles[grid[0][0]].get_pos().get_x(), maxX = particles[grid[0][0]].get_pos().get_x();
    double minY = particles[grid[0][0]].get_pos().get_y(), maxY = particles[grid[0][0]].get_pos().get_y();
    for (std::size_t i = 0; i < nx; ++i) {
      for (std::size_t j = 0; j < ny; ++j) {
        double cx = particles[grid[i][j]].get_pos().get_x();
        double cy = particles[grid[i][j]].get_pos().get_y();
        minX = std::min(minX, cx); maxX = std::max(maxX, cx);
        minY = std::min(minY, cy); maxY = std::max(maxY, cy);
      }
    }
    Vec2 min_pos(minX, minY);
    Vec2 max_pos(maxX, maxY);
    AABB aabb(min_pos, max_pos);
    return aabb;
  }

  bool colliding_aabb(const AABB& currentBox, const AABB& otherBox) {
    Vec2 cMin = currentBox.get_min(), cMax = currentBox.get_max();
    Vec2 oMin = otherBox.get_min(), oMax = otherBox.get_max();
    bool intersectX = (std::min(cMax.get_x(), oMax.get_x()) > std::max(cMin.get_x(), oMin.get_x()));
    bool intersectY = (std::min(cMax.get_y(), oMax.get_y()) > std::max(cMin.get_y(), oMin.get_y()));
    return intersectX && intersectY;
  }

  std::vector<int> point_candidates_in_polygon(const std::vector<Particle>& particles, const AABB& otherBox) {
    std::vector<int> points;
    double minX = otherBox.get_min().get_x(), minY = otherBox.get_min().get_y();
    double maxX = otherBox.get_max().get_x(), maxY = otherBox.get_max().get_y();
    for (std::size_t i = 0; i < nx; ++i) {
      for (std::size_t j = 0; j < ny; ++j) {
        int p = grid[i][j];
        int x = particles[p].get_pos().get_x();
        int y = particles[p].get_pos().get_y();
        if (x > minX && x < maxX && y > minY && y < maxY) points.push_back(p);
      }
    }
    return points;
  }
};
