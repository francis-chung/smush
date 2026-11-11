#include "softrect.cpp"
#include "world.cpp"
#include "visualizer.cpp"

#include <vector>
#include <chrono> 
#include <thread>
#include <fstream>

using Clock = std::chrono::high_resolution_clock;
using TimePoint = std::chrono::time_point<Clock>;
using Duration = std::chrono::duration<double>;

class Solver {
public: 
  World world;

  Solver(double linearDamp, Vec2 lower_bounds, Vec2 upper_bounds, std::vector<SoftRect> rects = {}) : 
    world(linearDamp, lower_bounds, upper_bounds) {
    for (SoftRect& r : rects) {
      world.add_rect(r);
    }
  }

  void run() {  
    const double physicsHz = 120.0;
    const double fixedDeltaTime = 1.0 / physicsHz;
    TimePoint currentTime = Clock::now();
    double accum = 0.0;

    bool running = true;
    static std::size_t frameCount = 0;
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
        
        // write state to file every 2 frames
        if (frameCount++ % 2 == 0) {
          std::ofstream out("visualization/physics_state.json");
          if (out.is_open()) {
            out << world_to_json(world);
            out.close();
          } else {
            std::cerr << "failed to open file" << std::endl;
          }
        }
      }

      // sleep briefly to prevent 100% CPU pinning
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
};
