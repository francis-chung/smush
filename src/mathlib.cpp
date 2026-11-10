#pragma once

#include <cmath>

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
  const double normsq() const { return normsq(); }
  double norm() { return sqrt(normsq()); }
  const double norm() const { return norm(); }
  friend double dot(const Vec2& v1, const Vec2& v2) { return v1.x*v2.x + v1.y*v2.y; }
  friend Vec2 proj(const Vec2& u, const Vec2& v) {
    return dot(u, v) / u.normsq() * u;
  }
  friend Vec2 perp(const Vec2& u, const Vec2& v) {
    return v - proj(u, v);
  }
};
