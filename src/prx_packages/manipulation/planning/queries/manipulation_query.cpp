/**
 * @file manipulation_query.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "planning/queries/manipulation_query.hpp"
#include "simulation/systems/plants/movable_body_plant.hpp"

#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/goals/goal.hpp"


#include <pluginlib/class_list_macros.h> 
#include <ros/ros.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::manipulation_query_t, prx::plan::query_t)

namespace prx
{
    using namespace util;
    using namespace sim;

    namespace packages
    {
        namespace manipulation
        {

            manipulation_query_t::manipulation_query_t()
            {
            	ik_steer_paths = false;
                manipulation_context_name = "";
                mode = PRX_MOVE;                
                manipulator_initial_state = NULL;
                manipulator_target_state = NULL;
                object_initial_state = NULL;
                object_target_state = NULL;
                object = NULL;
                grasping_index = -1;
                default_open_mode = -1;
                found_path = false;
                manipulator_final_state = NULL;
            }

            manipulation_query_t::manipulation_query_t(std::string manipulation_context_name, path_mode_t mode, movable_body_plant_t* object, int grasping_index, int open_end_effector_mode, util::config_t& retract_config, state_t* manipulator_initial_state, state_t* manipulator_target_state, sim::state_t* manip_final_state, state_t* object_initial_state, state_t* object_target_state)
            {
            	setup(manipulation_context_name,mode,object,grasping_index,open_end_effector_mode,retract_config,manipulator_initial_state,manipulator_target_state,manip_final_state,object_initial_state,object_target_state);
            }

            manipulation_query_t::~manipulation_query_t()
            {
                clear();
            }

            void manipulation_query_t::setup(std::string manipulation_context_name, path_mode_t mode, movable_body_plant_t* object, int grasping_index, int open_end_effector_mode, util::config_t& retract_config, state_t* manipulator_initial_state, state_t* manipulator_target_state, sim::state_t* manip_final_state, state_t* object_initial_state, state_t* object_target_state)
            {
            	this->manipulation_context_name = manipulation_context_name;
            	this->mode = mode;
            	this->object = object;
            	this->grasping_index = grasping_index;
                default_open_mode = open_end_effector_mode;
            	this->manipulator_initial_state = manipulator_initial_state;
            	this->manipulator_target_state = manipulator_target_state;
                this->manipulator_final_state = manip_final_state;
            	this->object_initial_state = object_initial_state;
            	this->object_target_state = object_target_state;
                found_path = false;
                ik_steer_paths = false;
                retraction_config = retract_config;
            }

            void manipulation_query_t::setup_pick(std::string manipulation_context_name, movable_body_plant_t* object, int grasping_index, int open_end_effector_mode, util::config_t& retract_config, state_t* manipulator_initial_state, sim::state_t* manip_final_state, state_t* object_initial_state)
            {
            	setup(manipulation_context_name,PRX_PICK,object,grasping_index,open_end_effector_mode,retract_config,manipulator_initial_state,NULL,manip_final_state,object_initial_state,NULL);
            }

            void manipulation_query_t::setup_place(std::string manipulation_context_name, movable_body_plant_t* object, int grasping_index, int open_end_effector_mode, util::config_t& retract_config, state_t* manipulator_initial_state, state_t* object_initial_state, sim::state_t* manip_final_state, state_t* object_target_state)
            {
            	setup(manipulation_context_name,PRX_PLACE,object,grasping_index,open_end_effector_mode,retract_config,manipulator_initial_state,NULL,manip_final_state, object_initial_state,object_target_state);
            }
            void manipulation_query_t::setup_transfer(std::string manipulation_context_name, movable_body_plant_t* object, state_t* manipulator_initial_state, state_t* object_target_state)
            {
                config_t dummy_config;
                setup(manipulation_context_name,PRX_TRANSFER,object,-1,-1,dummy_config,manipulator_initial_state,NULL,NULL, NULL,object_target_state);
            }

            void manipulation_query_t::setup_move(std::string manipulation_context_name, state_t* manipulator_initial_state, state_t* manipulator_target_state)
            {
                config_t conf;
            	setup(manipulation_context_name,PRX_MOVE,NULL,-1,-1,conf,manipulator_initial_state,manipulator_target_state,NULL,NULL,NULL);
            }

            void manipulation_query_t::clear()
            {            	
                manipulation_context_name = "";
                manipulator_initial_state = NULL;
                manipulator_target_state = NULL;
                object_initial_state = NULL;
                object_target_state = NULL;
                manipulator_final_state = NULL;
                object = NULL;
                grasping_index = -1;
                default_open_mode = -1;
                found_path = false;
                ik_steer_paths = false;
                retraction_config.zero();
                motion_planning_query_t::clear();
            }
        }
    }
}
