#pragma once

#include "softbody.cpp"
#include "particle.cpp"

class SoftRect : public SoftBody {
private: 
  double w, h;
  std::size_t nx, ny;                             // number of particles aligned to axes

  void add_spring(std::vector<Particle>& particles, std::size_t i, std::size_t j, double stiff, double damp) {
    Vec2 d = particles[j].get_pos() - particles[i].get_pos();
    double len = d.norm();
    springs.push_back(Spring(i, j, len, stiff, damp));
  }

public:
  SoftRect(std::vector<Particle>& particles, Vec2 p, double wd, double hi, double ms, double rest, std::size_t subd, double angleRad = 0.0) {
    assert(wd > 0);
    assert(hi > 0);
    assert(ms > 0);
    assert(rest >= 0 && rest <= 1);
    assert(subd > 0);
    
    w = wd, h = hi, mass = ms, restitution = rest, nx = subd + 1, ny = subd + 1;
    start_idx_ = particles.size();

    double pm = mass / (nx * ny);
    const Vec2 center = p + Vec2(wd * 0.5, hi * 0.5);
    const double c = std::cos(angleRad);
    const double s = std::sin(angleRad);
    std::vector<std::vector<std::size_t>> grid;

    for (std::size_t i = 0; i < nx; ++i) {
      grid.push_back(std::vector<std::size_t>());
      for (std::size_t j = 0; j < ny; ++j) {
        // get coordinates relative to center first
        const double localX = (wd / subd) * i - wd * 0.5;
        const double localY = (hi / subd) * j - hi * 0.5;
        const Vec2 rotatedFromCenter(
          c * localX - s * localY, 
          s * localX + c * localY
        );

        Vec2 pos = center + rotatedFromCenter;
        Vec2 vel{0.0, 0.0};
        Particle p = Particle(pos, vel, pm, restitution);
        particles.push_back(p);
        grid[i].push_back(particles.size()-1);
      }
    }
    for (std::size_t i = 0; i < nx; ++i)   boundary.push_back(grid[i][0]);
    for (std::size_t i = 1; i < ny; ++i)   boundary.push_back(grid[nx-1][i]);
    for (std::size_t i = 1; i < nx; ++i)   boundary.push_back(grid[nx-1-i][ny-1]);
    for (std::size_t i = 1; i < ny-1; ++i) boundary.push_back(grid[0][ny-1-i]);
    
    double strStiff = 30.0, strDamp = 0.5;
    double sheStiff = 20.0, sheDamp = 0.5;
    double benStiff = 20.0, benDamp = 0.6;
    for (std::size_t i = 0; i < nx; ++i) {
      for (std::size_t j = 0; j < ny; ++j) {
        std::size_t p1 = grid[i][j];
        if (j+1 < ny)              add_spring(particles, p1, grid[i][j+1], strStiff, strDamp); 
        if (i+1 < nx)              add_spring(particles, p1, grid[i+1][j], strStiff, strDamp); 
        if (i+1 < nx && j+1 < ny)  add_spring(particles, p1, grid[i+1][j+1], sheStiff, sheDamp); 
        if (i+1 < nx && j > 0)     add_spring(particles, p1, grid[i+1][j-1], sheStiff, sheDamp); 
        if (j+2 < ny)              add_spring(particles, p1, grid[i][j+2], benStiff, benDamp); 
        if (i+2 < nx)              add_spring(particles, p1, grid[i+2][j], benStiff, benDamp); 
      }
    }
  }

  std::size_t get_particle(std::size_t i, std::size_t j) const { return start_idx_ + i*ny+j; }
  std::size_t get_nx() const { return nx; }
  std::size_t get_ny() const { return ny; }

  void pin(std::vector<Particle>& particles, std::size_t i, std::size_t j) {
    pin_index(particles, start_idx_ + i*ny+j);
  }

  void update_springs(std::vector<Particle>& particles) override {
    for (Spring& s : springs) { s.apply_spring_force(particles); }
  }

  void apply_external_force(std::vector<Particle>& particles) override {
    const Vec2 particleForce = externalForce / (nx * ny);
    for (std::size_t i = 0; i < nx; ++i) {
      for (std::size_t j = 0; j < ny; ++j) {
        particles[get_particle(i, j)].add_force(particleForce);
      }
    }
  }

  void apply_external_impulse(std::vector<Particle>& particles, const Vec2& impulse) override {
    const Vec2 particleImpulse = impulse / (nx * ny);
    for (std::size_t i = 0; i < nx; ++i) {
      for (std::size_t j = 0; j < ny; ++j) {
        std::size_t idx = get_particle(i, j);
        Particle& p = particles[idx];
        const Vec2& v = p.get_vel();
        const double invM = p.get_inv_mass();
        particles[idx].set_vel(v + particleImpulse * invM);
      }
    }
  }

  bool point_in_polygon(std::size_t pIndex, const std::vector<Particle>& particles) override {
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

  AABB get_aabb(const std::vector<Particle>& particles) override {
    double minX = particles[start_idx_].get_pos().get_x(), maxX = particles[start_idx_].get_pos().get_x();
    double minY = particles[start_idx_].get_pos().get_y(), maxY = particles[start_idx_].get_pos().get_y();
    for (std::size_t i = 0; i < nx; ++i) {
      for (std::size_t j = 0; j < ny; ++j) {
        double cx = particles[get_particle(i, j)].get_pos().get_x();
        double cy = particles[get_particle(i, j)].get_pos().get_y();
        minX = std::min(minX, cx); maxX = std::max(maxX, cx);
        minY = std::min(minY, cy); maxY = std::max(maxY, cy);
      }
    }
    Vec2 min_pos(minX, minY);
    Vec2 max_pos(maxX, maxY);
    AABB aabb(min_pos, max_pos);
    return aabb;
  }

  std::vector<std::size_t> point_candidates_in_body(const std::vector<Particle>& particles, const AABB& otherBox) override {
    std::vector<std::size_t> points;
    double minX = otherBox.get_min().get_x(), minY = otherBox.get_min().get_y();
    double maxX = otherBox.get_max().get_x(), maxY = otherBox.get_max().get_y();
    for (std::size_t i = 0; i < nx; ++i) {
      for (std::size_t j = 0; j < ny; ++j) {
        int p = get_particle(i, j);
        double x = particles[p].get_pos().get_x();
        double y = particles[p].get_pos().get_y();
        if (x > minX && x < maxX && y > minY && y < maxY) points.push_back(p);
      }
    }
    return points;
  }
};
