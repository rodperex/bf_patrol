#ifndef BF_PATROL__UTILS_HPP_
#define BF_PATROL__UTILS_HPP_

#include <string>
#include <vector>
#include <sstream>
#include <iostream>

struct Waypoint
{
  std::string id;
  double x;
  double y;
  bool visited;
  bool in_process;
};

std::vector<Waypoint> deserialize_wps(std::string s_wps);
std::string serialize_wps(std::vector<Waypoint> wps);
void print_wps(std::vector<Waypoint> wps);

#endif
