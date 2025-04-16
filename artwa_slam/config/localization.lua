include "slam.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}
POSE_GRAPH.optimize_every_n_nodes = 20
global_constraint_search_after_n_seconds = 1.0

return options