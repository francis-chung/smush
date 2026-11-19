#pragma once

#include "httplib.h"
#include "json.hpp"
#include "softbody.cpp"
#include "softrect.cpp"
#include "softcirc.cpp"
#include "world.cpp"

#include <string>
#include <mutex>

using nlohmann::json;

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

SpawnCmd parse(std::string str) {
  json j = json::parse(str);
  SpawnCmd cmd{};
  cmd.type = j.at("type").get<std::string>();
  cmd.x    = j.at("x").get<double>();
  cmd.y    = j.at("y").get<double>();
  cmd.mass = j.at("mass").get<double>();
  cmd.rest = j.at("restitution").get<double>();

  if (cmd.type == "rect") {
    cmd.w        = j.at("w").get<double>();
    cmd.h        = j.at("h").get<double>();
    cmd.subdiv   = j.at("subdiv").get<std::size_t>();
    cmd.angleDeg = j.at("angleDeg").get<double>();
  } else { // "circ"
    cmd.radius = j.at("radius").get<double>();
    cmd.num    = j.at("num").get<std::size_t>();
  }
  return cmd;
}