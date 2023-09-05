# include "bf_patrol/utils.hpp"

std::vector<Waypoint> deserialize_wps(std::string s_wps)
{
  std::vector<Waypoint> wps;
  std::vector<std::string> wp_tokens;

  std::stringstream ss_wp(s_wps);
  std::string wp_token;

  while (std::getline(ss_wp, wp_token, ';')) {
    std::stringstream ss_field(wp_token);
    std::string field_token;

    Waypoint wp;
    
    std::getline(ss_field, field_token, ',');
    wp.id = field_token;
    
    std::getline(ss_field, field_token, ',');
    wp.visited = (bool)std::stoi(field_token);

    std::getline(ss_field, field_token, ',');
    wp.in_process = (bool)std::stoi(field_token);
    
    std::getline(ss_field, field_token, ',');
    wp.x = std::stof(field_token);
    
    std::getline(ss_field, field_token, ',');
    wp.y = std::stof(field_token);
    
    wps.push_back(wp);
  }

  return wps;
}

std::string serialize_wps(std::vector<Waypoint> wps)
{
  std::string s_wps = "";

  for (auto wp : wps) {
    s_wps += wp.id
    + "," + std::to_string(wp.visited)
    + "," + std::to_string(wp.in_process) // newline
    + "," + std::to_string(wp.x)
    + "," + std::to_string(wp.y)
    + ";";
  }

  return s_wps;
}
