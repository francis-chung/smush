#include "world.cpp"

#include <vector>
#include <cmath>
#include <chrono> 
#include <thread>

using Clock = std::chrono::high_resolution_clock;
using TimePoint = std::chrono::time_point<Clock>;
using Duration = std::chrono::duration<double>;


int main() {
  double linearDamp = 0.05;
  Vec2 lower_bounds = Vec2(-50.0, -50.0);
  Vec2 upper_bounds = Vec2(50.0, 50.0);
  World world = World(linearDamp, lower_bounds, upper_bounds);
  
  const double physicsHz = 120.0;
  const double fixedDeltaTime = 1.0 / physicsHz;
  TimePoint currentTime = Clock::now();
  double accum = 0.0;

  bool running = true;
  while (running) {
    TimePoint newTime = Clock::now();
    Duration frameTime = newTime - currentTime;
    currentTime = newTime;

    // caps max frame time to prevent spiral of death
    double dt = frameTime.count();
    if (dt > 0.25) dt = 0.25;

    accum += dt;
    while (accum >= fixedDeltaTime) {
      world.update(fixedDeltaTime);
      accum -= fixedDeltaTime;
    }

    // sleep briefly to prevent 100% CPU pinning
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}
