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
        int x_pos = (*it)[0].as<int>();
        int y_pos = (*it)[1].as<int>();
        int z_pos = (*it)[2].as<int>();
        std::cout << "Vertex " << vertex_name
                  << " is at position "
                  << "(" << x_pos
                  << "," << y_pos 
                  << "," << z_pos
                  << ")\n";
        
        graph.add_waypoint(test_map_name, {x_pos, y_pos});

        // It seems graph indexes nodes by order of addition into the graph.
        // We should probably give a mapping of "vertex name -> graph index".
        vertex_name_id_map[vertex_name] = vertex_id; // consider returning this
        vertex_id_name_map[vertex_id] = vertex_name;
        vertex_id++;
    }

    const YAML::Node y_lanes = y["levels"]["L1"]["lanes"];

    for (YAML::const_iterator it = y_lanes.begin(); it != y_lanes.end(); ++it)
    {
        int origin_vertex_id = (*it)[0].as<int>();
        int tail_vertex_id = (*it)[1].as<int>();
        std::string origin_vertex_name = (*vertex_id_name_map.find(origin_vertex_id)).second;
        std::string tail_vertex_name = (*vertex_id_name_map.find(tail_vertex_id)).second;
        YAML::const_iterator bidirectional_info = (*it)[2]["bidirectional"].begin();
        bidirectional_info++;
        bool is_bidirectional = (*bidirectional_info).as<bool>();
        graph.add_lane(origin_vertex_id, tail_vertex_id);
        
        if (is_bidirectional)
        {
            std::cout << "Edge from " << origin_vertex_name << " to " << tail_vertex_name
                      << " is bidirectional." << std::endl;
            graph.add_lane(tail_vertex_id, origin_vertex_id);
            
        } else {
            std::cout << "Edge from " << origin_vertex_name << " to " << tail_vertex_name
                  << " not bidirectional." << std::endl;
        }
    }

    // TODO: Add door functionality. Need to check how this is represented in traffic-editor

    return graph;
}

int main()
{
    rmf_traffic::agv::Graph graph = load("test_map", "/home/bhan/dev/fleet-adapter-tutorial/maps/maze-complete/maze.yaml");
    return 0;
}