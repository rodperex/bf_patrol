#ifndef BF_PATROL__DATATYPES_HPP_
#define BF_PATROL__DATATYPES_HPP_

#include <string>

struct Waypoint {
    std::string id;
    double x;
    double y;
    bool visited;
  };

#endif 