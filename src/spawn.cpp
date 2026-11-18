#include "httplib.h"

#include <string>
#include <mutex>

std::mutex spawnMx;
std::vector<SpawnCmd> pendingSpawns;

struct SpawnCmd {
  std::string type;    // "rect" | "circ"
  double x, y, mass, rest;
  double w, h, radius, angleDeg;
  std::size_t subdiv, num;
};

void start_server() {  
  httplib::Server svr;
  svr.set_mount_point("/", "visualization");   // serves visualizer.html

  svr.Post("/spawn", [&](const httplib::Request& req, httplib::Response& res) {
    SpawnCmd cmd = parse(req.body); // TODO: JSON parsing
    {
      std::lock_guard<std::mutex> lock(spawnMx);
      pendingSpawns.push_back(cmd);
    }
    res.set_content("{\"ok\":true}", "application/json");
  });
}
