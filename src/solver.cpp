#include "softrect.cpp"
#include "world.cpp"
#include "visualizer.cpp"
#include "spawn.cpp"

#include <vector>
#include <chrono> 
#include <thread>
#include <fstream>
#include <mutex>

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

using Clock = std::chrono::high_resolution_clock;
using TimePoint = std::chrono::time_point<Clock>;
using Duration = std::chrono::duration<double>;

class Solver {
public: 
  World world;

  Solver(double linearDamp, Vec2 lower_bounds, Vec2 upper_bounds) : 
    world(linearDamp, lower_bounds, upper_bounds) {}

  void run() {  
    const double physicsHz = 120.0;
    const double fixedDeltaTime = 1.0 / physicsHz;
    TimePoint currentTime = Clock::now();
    double accum = 0.0;

    std::thread serverThread(start_server);
    serverThread.detach();    // runs independently rather than joining
    
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
        // drain spawns
        std::vector<SpawnCmd> toSpawn;
        {
          std::lock_guard<std::mutex> lock(spawnMx);
          std::swap(toSpawn, pendingSpawns);
        }
        for (const SpawnCmd& c : toSpawn) {
          if (c.type == "rect") {
            world.add_rect(Vec2(c.x, c.y), c.w, c.h, c.mass, c.rest, c.subdiv, c.angleDeg * pi / 180.0);
          } else {
            world.add_circ(Vec2(c.x, c.y), c.radius, c.mass, c.rest, c.num);
          }
        }

        world.update(fixedDeltaTime);
        accum -= fixedDeltaTime;
        
        // write state to file every 2 frames
        if (frameCount++ % 2 == 0) {
          const char *temp = "visualization/physics_state_tmp.json";
          std::ofstream out(temp);
          out << world_to_json(world);
          out.close(); 

          if (!out) {
            std::cerr << "failed to write snapshot\n";
          } else if (!MoveFileExA(temp, "visualization/physics_state.json", MOVEFILE_REPLACE_EXISTING)) {
            const auto error = GetLastError();
            std::cerr << "failed to publish snapshot: " << error << '\n';
          }
        }
      }

      // sleep briefly to prevent 100% CPU pinning
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
};
