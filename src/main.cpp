#include "softrect.cpp"
#include "solver.cpp"

int main() {
  double linearDamp = 0.05;
  Vec2 lower_bounds{-50.0, -50.0};
  Vec2 upper_bounds{50.0, 50.0};

  Solver solver = Solver(linearDamp, lower_bounds, upper_bounds);

  const double pi = std::acos(-1.0);
  const double angle1 = 25.0 * pi / 180.0;
  const double angle2 = 55.0 * pi / 180.0;
  solver.world.add_rect(Vec2(-20, 20), 10, 10, 1.0, 0.8, 5, angle1);
  solver.world.add_rect(Vec2(5, 30), 8, 8, 1.0, 0.8, 4, angle2);
  solver.world.add_circ(Vec2(5, -15), 6, 1.0, 0.8, 16);
  solver.world.apply_impulse(0, Vec2(20.0, 10.0));

  solver.run();
}
