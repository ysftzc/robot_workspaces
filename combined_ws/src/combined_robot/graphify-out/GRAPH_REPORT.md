# Graph Report - combined_robot  (2026-05-04)

## Corpus Check
- 30 files · ~8,351,504 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 102 nodes · 143 edges · 6 communities detected
- Extraction: 94% EXTRACTED · 6% INFERRED · 0% AMBIGUOUS · INFERRED: 9 edges (avg confidence: 0.8)
- Token cost: 0 input · 0 output

## Community Hubs (Navigation)
- [[_COMMUNITY_Community 0|Community 0]]
- [[_COMMUNITY_Community 1|Community 1]]
- [[_COMMUNITY_Community 2|Community 2]]
- [[_COMMUNITY_Community 3|Community 3]]
- [[_COMMUNITY_Community 4|Community 4]]
- [[_COMMUNITY_Community 5|Community 5]]

## God Nodes (most connected - your core abstractions)
1. `MissionManager` - 19 edges
2. `handle_dsv_types_except_source()` - 7 edges
3. `handle_dsv_types_except_source()` - 7 edges
4. `main()` - 6 edges
5. `_include_comments()` - 6 edges
6. `main()` - 6 edges
7. `_include_comments()` - 6 edges
8. `get_packages()` - 4 edges
9. `order_packages()` - 4 edges
10. `process_dsv_file()` - 4 edges

## Surprising Connections (you probably didn't know these)
- `generate_launch_description()` --calls--> `Node`  [INFERRED]
  launch/combined_gazebo.launch (Copy).py →   _Bridges community 4 → community 0_
- `generate_launch_description()` --calls--> `Node`  [INFERRED]
  launch/combined_gazebo.launch.py →   _Bridges community 5 → community 0_
- `MissionManager` --inherits--> `Node`  [EXTRACTED]
  combined_robot/mission_manager.py →   _Bridges community 3 → community 0_

## Communities (14 total, 1 thin omitted)

### Community 0 - "Community 0"
Cohesion: 0.1
Nodes (12): generate_launch_description(), combined_gazebo.launch.py Launches the Panther + Franka FR3 combined robot in Ga, # NOTE: imu_broadcaster is intentionally NOT spawned here., generate_launch_description(), combined.launch.py Minimal launch: starts robot_state_publisher only (no Gazebo,, generate_launch_description(), generate_launch_description(), generate_launch_description() (+4 more)

### Community 1 - "Community 1"
Cohesion: 0.2
Nodes (18): add_package_runtime_dependencies(), _append_unique_value(), get_commands(), get_packages(), handle_dsv_types_except_source(), _include_comments(), main(), order_packages() (+10 more)

### Community 2 - "Community 2"
Cohesion: 0.2
Nodes (18): add_package_runtime_dependencies(), _append_unique_value(), get_commands(), get_packages(), handle_dsv_types_except_source(), _include_comments(), main(), order_packages() (+10 more)

### Community 4 - "Community 4"
Cohesion: 0.33
Nodes (5): generate_launch_description(), combined_gazebo.launch.py Launches the Panther + Franka FR3 combined robot in Ga, # IMPORTANT: This is delayed (see below) so that drive_controller's, # NOTE: odometry/wheels remap REMOVED — drive_controller already, # NOTE: imu_broadcaster is intentionally NOT spawned here.

### Community 5 - "Community 5"
Cohesion: 0.5
Nodes (3): generate_launch_description(), combined_gazebo.launch.py Launches the Panther + Franka FR3 combined robot in Ga, # NOTE: imu_broadcaster is intentionally NOT spawned here.

## Knowledge Gaps
- **18 isolated node(s):** `combined.launch.py Minimal launch: starts robot_state_publisher only (no Gazebo,`, `combined_gazebo.launch.py Launches the Panther + Franka FR3 combined robot in Ga`, `# IMPORTANT: This is delayed (see below) so that drive_controller's`, `# NOTE: odometry/wheels remap REMOVED — drive_controller already`, `# NOTE: imu_broadcaster is intentionally NOT spawned here.` (+13 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **1 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `MissionManager` connect `Community 3` to `Community 0`?**
  _High betweenness centrality (0.130) - this node is a cross-community bridge._
- **Why does `generate_launch_description()` connect `Community 4` to `Community 0`?**
  _High betweenness centrality (0.043) - this node is a cross-community bridge._
- **Are the 9 inferred relationships involving `Node` (e.g. with `generate_launch_description()` and `generate_launch_description()`) actually correct?**
  _`Node` has 9 INFERRED edges - model-reasoned connections that need verification._
- **What connects `combined.launch.py Minimal launch: starts robot_state_publisher only (no Gazebo,`, `combined_gazebo.launch.py Launches the Panther + Franka FR3 combined robot in Ga`, `# IMPORTANT: This is delayed (see below) so that drive_controller's` to the rest of the system?**
  _18 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `Community 0` be split into smaller, more focused modules?**
  _Cohesion score 0.1 - nodes in this community are weakly interconnected._