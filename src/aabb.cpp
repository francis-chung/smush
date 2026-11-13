#pragma once

#include "mathlib.cpp"

class AABB {
private: 
  Vec2 min, max;

public: 
  AABB() : min(), max() {}
  AABB(Vec2 min, Vec2 max) : min(min), max(max) {}

  const Vec2& get_min() const { return min; }
  const Vec2& get_max() const { return max; }

  bool colliding_aabb(const AABB& otherBox, double tolerance = 1e-5) {
    Vec2 cMin = min, cMax = max;
    Vec2 oMin = otherBox.min, oMax = otherBox.max;
    bool intersectX = (std::min(cMax.get_x(), oMax.get_x()) > std::max(cMin.get_x(), oMin.get_x()) - tolerance);
    bool intersectY = (std::min(cMax.get_y(), oMax.get_y()) > std::max(cMin.get_y(), oMin.get_y()) - tolerance);
    return intersectX && intersectY;
  }
};