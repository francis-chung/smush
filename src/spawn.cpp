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

struct SpawnCmd {
  std::string type;    // "rect" | "circ"
  double x, y, mass, rest;
  double w, h, radius, angleDeg;
  std::size_t subdiv, num;
};

std::mutex spawnMx;
std::vector<SpawnCmd> pendingSpawns;

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

void start_server() {  
  httplib::Server svr;
  svr.set_mount_point("/", "visualization");   // serves visualizer.html

  svr.Options("/spawn", [](const httplib::Request&, httplib::Response& res) {
    res.set_header("Access-Control-Allow-Methods", "POST, OPTIONS");
    res.set_header("Access-Control-Allow-Headers", "Content-Type");
    res.status = 204;   // no content
  });

  svr.Post("/spawn", [](const httplib::Request& req, httplib::Response& res) {
    res.set_header("Access-Control-Allow-Origin", "*");
    try {
      SpawnCmd cmd = parse(req.body); // TODO: JSON parsing
      {
        std::lock_guard<std::mutex> lock(spawnMx);
        pendingSpawns.push_back(cmd);
      }
      res.set_content("{\"ok\":true}", "application/json");
    } catch (const std::exception& e) {
      res.status = 400;
      res.set_content(std::string("{\"error\":\"") + e.what() + "\"}", "application/json");
    }
  });
  svr.listen("127.0.0.1", 8000);   // this blocks, so it runs on its own thread
}
