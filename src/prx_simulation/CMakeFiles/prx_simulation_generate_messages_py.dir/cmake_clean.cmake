FILE(REMOVE_RECURSE
  "CMakeFiles/prx_simulation_generate_messages_py"
  "devel/lib/python2.7/dist-packages/prx_simulation/msg/_plant_locations_msg.py"
  "devel/lib/python2.7/dist-packages/prx_simulation/msg/_query_msg.py"
  "devel/lib/python2.7/dist-packages/prx_simulation/msg/_world_config_msg.py"
  "devel/lib/python2.7/dist-packages/prx_simulation/msg/_graph_msg.py"
  "devel/lib/python2.7/dist-packages/prx_simulation/msg/_lqr_msg.py"
  "devel/lib/python2.7/dist-packages/prx_simulation/msg/_manipulation_acknowledgement.py"
  "devel/lib/python2.7/dist-packages/prx_simulation/msg/_plan_msg.py"
  "devel/lib/python2.7/dist-packages/prx_simulation/msg/_interval_msg.py"
  "devel/lib/python2.7/dist-packages/prx_simulation/msg/_bomberman_info_msg.py"
  "devel/lib/python2.7/dist-packages/prx_simulation/msg/_control_msg.py"
  "devel/lib/python2.7/dist-packages/prx_simulation/msg/_state_msg.py"
  "devel/lib/python2.7/dist-packages/prx_simulation/srv/_request_ground_truth_srv.py"
  "devel/lib/python2.7/dist-packages/prx_simulation/srv/_send_plans_srv.py"
  "devel/lib/python2.7/dist-packages/prx_simulation/srv/_send_lqr_srv.py"
  "devel/lib/python2.7/dist-packages/prx_simulation/srv/_set_sim_state_srv.py"
  "devel/lib/python2.7/dist-packages/prx_simulation/srv/_request_space_information_srv.py"
  "devel/lib/python2.7/dist-packages/prx_simulation/msg/__init__.py"
  "devel/lib/python2.7/dist-packages/prx_simulation/srv/__init__.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/prx_simulation_generate_messages_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
