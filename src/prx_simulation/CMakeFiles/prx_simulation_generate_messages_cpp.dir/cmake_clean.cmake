FILE(REMOVE_RECURSE
  "CMakeFiles/prx_simulation_generate_messages_cpp"
  "devel/include/prx_simulation/plant_locations_msg.h"
  "devel/include/prx_simulation/query_msg.h"
  "devel/include/prx_simulation/world_config_msg.h"
  "devel/include/prx_simulation/graph_msg.h"
  "devel/include/prx_simulation/lqr_msg.h"
  "devel/include/prx_simulation/manipulation_acknowledgement.h"
  "devel/include/prx_simulation/plan_msg.h"
  "devel/include/prx_simulation/interval_msg.h"
  "devel/include/prx_simulation/bomberman_info_msg.h"
  "devel/include/prx_simulation/control_msg.h"
  "devel/include/prx_simulation/state_msg.h"
  "devel/include/prx_simulation/request_ground_truth_srv.h"
  "devel/include/prx_simulation/send_plans_srv.h"
  "devel/include/prx_simulation/send_lqr_srv.h"
  "devel/include/prx_simulation/set_sim_state_srv.h"
  "devel/include/prx_simulation/request_space_information_srv.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/prx_simulation_generate_messages_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
