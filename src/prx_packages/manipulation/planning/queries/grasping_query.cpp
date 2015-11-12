/**
 * @file grasping_query.cpp
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

#include "planning/queries/grasping_query.hpp"
#include "simulation/systems/plants/movable_body_plant.hpp"

#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/goals/goal.hpp"


#include <pluginlib/class_list_macros.h> 
#include <ros/ros.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::grasping_query_t, prx::plan::query_t)

namespace prx
{
    using namespace util;
    using namespace sim;

    namespace packages
    {
        namespace manipulation
        {

            grasping_query_t::grasping_query_t()
            {
                state_space = NULL;
                control_space = NULL;
                grasping_mode = -1;
                release_mode = 1;
            	clear();
            }

            grasping_query_t::grasping_query_t(const util::space_t* state_space, const util::space_t* control_space, movable_body_plant_t* in_object, int grasping_index, sim::state_t* object_initial_state,util::config_t& retract_config)
            {
            	setup(state_space, control_space, in_object, grasping_index, object_initial_state,retract_config);
            }

            grasping_query_t::~grasping_query_t()
            {
                clear();
            }

            void grasping_query_t::setup(const util::space_t* in_state_space, const util::space_t* in_control_space, movable_body_plant_t* in_object, int grasping_index, sim::state_t* object_initial_state,util::config_t& retract_config)
            {
                if(state_space != in_state_space)
                {
                    if(state_space != NULL)
                    {
                        state_space->free_point(releasing_state);
                        state_space->free_point(grasping_state);
                        state_space->free_point(retracted_open_state);
                        state_space->free_point(retracted_close_state);
                        state_space->free_point(tmp_state);
                    }
                    state_space = in_state_space;

                    releasing_state = state_space->alloc_point();
                    grasping_state = state_space->alloc_point();
                    retracted_open_state = state_space->alloc_point();
                    retracted_close_state = state_space->alloc_point();
                    tmp_state = state_space->alloc_point();                    
                    tmp_path.link_space(state_space);
                }

                if(control_space != in_control_space)
                {
                    reaching_plan.link_control_space(in_control_space);
                    retracting_plan.link_control_space(in_control_space);
                    control_space = in_control_space;
                }

                setup(in_object,grasping_index,object_initial_state,retract_config);
            }

            void grasping_query_t::setup(movable_body_plant_t* in_object, int grasping_index, sim::state_t* object_initial_state,util::config_t& retract_config)
            {
            	this->object = in_object;
            	this->grasping_index = grasping_index;
            	this->object_initial_state = object_initial_state;                
                tmp_path.clear();
                reaching_plan.clear();
                retracting_plan.clear();
                grasping_mode = -1;
                release_mode = 1;
                found_grasp = false;
                reason_for_failure = "";
                retraction_config = retract_config;
            }

            void grasping_query_t::clear()
            {
                if(state_space != NULL)
                {
                    state_space->free_point(releasing_state);
                    state_space->free_point(grasping_state);
                    state_space->free_point(retracted_open_state);
                    state_space->free_point(retracted_close_state);
                    state_space->free_point(tmp_state);
                }
                state_space = NULL;
                control_space = NULL;
                object = NULL;
                object_initial_state = NULL;
                grasping_index = -1;
                grasping_mode = -1;
                release_mode = 1;
                found_grasp = false;
                reason_for_failure = "";
                retraction_config.zero();
                //We don't have to delete the points because both plans and trajectories have the state space 
                //and the control space stored internally. When you link a new state or control space they both
                //going to delete the points and initialize new ones.                
                tmp_path.clear();
                reaching_plan.clear();
                retracting_plan.clear();
            }

            void grasping_query_t::get_object_configuration(util::config_t& config)
            {
                config.set_position(object_initial_state->memory[0], object_initial_state->memory[1], object_initial_state->memory[2]);
                config.set_orientation(object_initial_state->memory[3], object_initial_state->memory[4], object_initial_state->memory[5], object_initial_state->memory[6]);
            }

            std::string grasping_query_t::print(unsigned prec)
            {
                std::stringstream out(std::stringstream::out);
                out << "Releasing state       : " << state_space->print_point(releasing_state, prec) << std::endl;
                out << "Grasping state        : " << state_space->print_point(grasping_state, prec) << std::endl;
                out << "Retracted open state  : " << state_space->print_point(retracted_open_state, prec) << std::endl;
                out << "Retracted close state : " << state_space->print_point(retracted_close_state, prec) << std::endl;
                out << "Reaching plan: " << std::endl;
                out << reaching_plan.print(prec);
                out << "Retracting plan: " << std::endl;
                out << retracting_plan.print(prec);
                
                return out.str();
            }
        }
    }
}
