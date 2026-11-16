#pragma once

#include "softbody.cpp"
#include "particle.cpp"

class SoftRect : public SoftBody {
private: 
  double w, h;
  std::size_t nx, ny;                             // number of particles aligned to axes

public:
  SoftRect(std::vector<Particle>& particles, const Vec2 bl, double wd, double hi, double ms, double rest, std::size_t subd, double angleRad = 0.0) {
    assert(wd > 0);
    assert(hi > 0);
    assert(ms > 0);
    assert(rest >= 0 && rest <= 1);
    assert(subd > 0);
    
    w = wd, h = hi, mass = ms, restitution = rest, nx = subd + 1, ny = subd + 1;

    double pm = mass / (nx * ny);
    Vec2 vel{0.0, 0.0};
    const Vec2 center = bl + Vec2(w * 0.5, h * 0.5);
    const double c = std::cos(angleRad);
    const double s = std::sin(angleRad);
    // only used to facilitate initializing the springs
    std::vector<std::vector<std::size_t>> grid;

    for (std::size_t i = 0; i < nx; ++i) {
      grid.push_back(std::vector<std::size_t>());
      for (std::size_t j = 0; j < ny; ++j) {
        // get coordinates relative to center first
        const double localX = (w / subd) * i - w * 0.5;
        const double localY = (h / subd) * j - h * 0.5;
        const Vec2 rotatedFromCenter(
          c * localX - s * localY, 
          s * localX + c * localY
        );

        Vec2 pos = center + rotatedFromCenter;
        Particle p = Particle(pos, vel, pm, restitution);
        particleIndices.push_back(particles.size());
        grid[i].push_back(particles.size());
        particles.push_back(p);
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

  std::size_t get_particle(std::size_t i, std::size_t j) const { return particleIndices.front() + i*ny+j; }
  std::size_t get_nx() const { return nx; }
  std::size_t get_ny() const { return ny; }

  void pin(std::vector<Particle>& particles, std::size_t i, std::size_t j) {
    pin_index(particles, get_particle(i, j));
  }
};
