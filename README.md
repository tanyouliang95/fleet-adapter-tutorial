# Parse YAML to Graph
This section will focus on parsing the `maze.yaml` file, which is the output of `rmf-editor`, into the `Graph` data structure which is found in RMF. Libraries have already been written to do this task. This section is meant to give understanding of how this process works.

## Mechanics
All data is stored in the `maze.yaml` file. In particular, the information under `vertices` and `lanes`. In `rmf_map_to_graph`, we iterate through each of the entries. For each entry, the `load_graph` function will add the corresponding `vertex` or `lane` into a Graph. This graph representation is then used by the RMF Planner to decide the best path from one vertex to another.

Note that this is a high level path planning. For actual robot movement between two nodes in terms motor controls such as `cmd_vel`, we should expect that the robots own navstack will handle it.