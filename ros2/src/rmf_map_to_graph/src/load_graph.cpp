#include <load_graph.hpp>
#include <iostream>

rmf_traffic::agv::Graph load(std::string test_map_name, std::string path)
{
    rmf_traffic::agv::Graph graph;
    YAML::Node y;

    try
    {
        y = YAML::LoadFile(path);
    }
    catch (const std::exception &e)
    {
        std::cerr << "couldn't parse " << path.c_str() << "Error: " << e.what() << std::endl;
        return graph;
    }

    std::cout << "parsed " << path.c_str() << " successfully!" << std::endl;

    const YAML::Node y_vtx = y["levels"]["L1"]["vertices"];
    std::map<std::string, int> vertex_name_id_map;
    std::map<int, std::string> vertex_id_name_map;
    int vertex_id = 0;

    for (YAML::const_iterator it = y_vtx.begin(); it != y_vtx.end(); ++it)
    {
        std::string vertex_name = (*it)[3].as<std::string>();
        std::cout << "Vertex " << vertex_name
                  << " is at position "
                  << "(" << (*it)[0].as<int>()
                  << "," << (*it)[1].as<int>() << "," << (*it)[2].as<int>()
                  << ")\n";
                  
        // It seems graph indexes nodes by order of addition into the graph.
        // We should probably give a mapping of "vertex name -> graph index".
        vertex_name_id_map[vertex_name] = vertex_id; // consider returning this
        vertex_id_name_map[vertex_id] = vertex_name;
        vertex_id++;
    }

    const YAML::Node y_lanes = y["levels"]["L1"]["lanes"];

    for (YAML::const_iterator it = y_lanes.begin(); it != y_lanes.end(); ++it)
    {
        std::string origin_vertex_name = (*vertex_id_name_map.find((*it)[0].as<int>())).second;
        std::string tail_vertex_name = (*vertex_id_name_map.find((*it)[1].as<int>())).second;
        YAML::const_iterator bidirectional_info = (*it)[2]["bidirectional"].begin();
        bidirectional_info++;
        bool is_bidirectional = (*bidirectional_info).as<bool>();
        if (is_bidirectional)
        {
            std::cout << "Edge from " << origin_vertex_name << " to " << tail_vertex_name
                      << " is bidirectional." << std::endl;
        } else {
            std::cout << "Edge from " << origin_vertex_name << " to " << tail_vertex_name
                  << " not bidirectional." << std::endl;
        }
    }

    return graph;
}

int main()
{
    rmf_traffic::agv::Graph graph = load("test_map", "/home/bhan/dev/fleet-adapter-tutorial/maps/maze-complete/maze.yaml");
    return 0;
}