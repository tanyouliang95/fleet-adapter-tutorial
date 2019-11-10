#ifndef RMF_MAP_TO_GRAPH__LOAD_GRAPH_HPP
#define RMF_MAP_TO_GRAPH__LOAD_GRAPH_HPP

#include <yaml-cpp/yaml.h>
#include <rmf_traffic/agv/Graph.hpp>

namespace rmf_map_to_graph {

rmf_traffic::agv::Graph load(std::string test_map_name, std::string path);

} // namespace rmf_map_to_graph

#endif // RMF_MAP_TO_GRAPH__LOAD_GRAPH_HPP
