#include "softrect.cpp"
#include "solver.cpp"

int main() {
  double linearDamp = 0.05;
  Vec2 lower_bounds(-50.0, -50.0);
  Vec2 upper_bounds(50.0, 50.0);

  Solver solver = Solver(linearDamp, lower_bounds, upper_bounds);
  solver.world.add_rect(Vec2(-10, 20), 10, 10, 1.0, 0.8, 5);
  solver.world.add_rect(Vec2(5, 30), 8, 8, 1.0, 0.8, 4);

  solver.run();
}
