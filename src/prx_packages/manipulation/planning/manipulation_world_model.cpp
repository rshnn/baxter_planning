/**
 * @file manipulation_world_model.cpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2015, Rutgers the State University of New Jersey, New Brunswick
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Rahul Shome, Zakary Littlefield, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#include "planning/manipulation_world_model.hpp"
#include "simulation/simulators/manipulation_simulator.hpp"
#include "simulation/systems/plants/manipulator.hpp"
#include "simulation/systems/plants/movable_body_plant.hpp"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::manipulation_world_model_t, prx::plan::world_model_t)

namespace prx
{
    namespace packages
    {
        using namespace util;
        using namespace sim;
        using namespace plan;    

        namespace manipulation
        {
            void manipulation_world_model_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                IK_steer_paths = false;
                world_model_t::init(reader,template_reader);
                manip_simulator = dynamic_cast<manipulation_simulator_t*>(simulator);
                if(manip_simulator == NULL)
                    PRX_FATAL_S("Manipulation_world_model works only with manipulation_simulator_t!");

                active_manipulation_info = NULL;
                std::vector<manipulator_t*> manipulators;
                manip_simulator->get_manipulators(manipulators);
                ik_seeds = parameters::get_attribute_as<int>("num_ik_seeds",reader,template_reader,100);

                if(reader->has_attribute("IK_databases"))
                {
                    parameter_reader_t::reader_map_t ik_map = parameters::get_map("IK_databases",reader,NULL);

                    foreach(const parameter_reader_t::reader_map_t::value_type key_value, ik_map)
                    {
                        IK_seed_database_t* database = new IK_seed_database_t();
                        database->init(key_value.second);
                        std::string manipulator_name = key_value.second->get_attribute("manipulator");
                        manipulator_t* manip;
                        foreach(manipulator_t* manipulator,manipulators)
                        {
                            if(reverse_split_path(manipulator->get_pathname()).second==manipulator_name)
                            {
                                manip = manipulator;
                            }
                        }

                        if(key_value.second->has_attribute("deserialize_file"))
                        {
                            char* w = std::getenv("PRACSYS_PATH");
                            std::string dir(w);
                            dir += ("/prx_roadmaps/");
                            std::string file = dir + key_value.second->get_attribute("deserialize_file");
                            std::ifstream fin;
                            fin.open(file.c_str());
                            database->deserialize(fin,manip->get_state_space());
                            fin.close();
                        }
                        else
                        {
                            state_t* full_wm_state = get_full_state_space()->alloc_point();
                            int iterations = key_value.second->get_attribute_as<int>("samples");
                            std::vector<std::string> names = manip->get_end_effector_names();
                            const space_t* manip_space = manip->get_state_space();
                            state_t* manip_state = manip_space->alloc_point();
                            config_t left_config;
                            config_t right_config;
                            std::string left_name = reverse_split_path(names[0]).second;
                            std::string right_name = reverse_split_path(names[1]).second;
                            for (int i = 0; i < iterations; ++i)
                            {
                                PRX_STATUS_S("Building IK Database "<<i+1<<"/"<<iterations);
                                manip_space->uniform_sample(manip_state);
                                manip_space->copy_from_point(manip_state);
                                manip->FK_solver(left_config,left_name);
                                // double x,y,z;
                                // left_config.get_position(x,y,z);
                                // if(.7<x && x <  .9 && -.1 < y && y < .1 && .75 < z && z < .95)
                                // {
                                    manip->FK_solver(right_config,right_name);
                                    database->add_pair(manip_space, left_config, right_config, manip_state );
                                // }
                                // else
                                //     i--;
                            }
                            if(key_value.second->has_attribute("serialize_file"))
                            {
                                char* w = std::getenv("PRACSYS_PATH");
                                std::string dir(w);
                                dir += ("/prx_roadmaps/");
                                std::string file = dir + key_value.second->get_attribute("serialize_file");
                                std::ofstream fout;
                                fout.open(file.c_str());
                                database->serialize(fout, manip->get_state_space());
                                fout.close();

                            }
                            get_full_state_space()->copy_from_point(full_wm_state);
                            get_full_state_space()->free_point(full_wm_state);
                        }
                        ik_databases.push_back(std::make_pair(key_value.first,database));
                    }
                }

                if( parameters::has_attribute("planning_contexts",reader,template_reader))
                {
                    parameter_reader_t::reader_map_t contexts_map = parameters::get_map("planning_contexts",reader,template_reader);
                    foreach(const parameter_reader_t::reader_map_t::value_type key_value, contexts_map)
                    {                    
                        if(key_value.second->has_attribute("manipulation_context_info"))
                        {
                             world_model_t::use_context(key_value.first);
                             manipulator_t* manipulator = find_active_manipulator(manipulators);
                             if(manipulator == NULL)
                                PRX_FATAL_S("The manipulation context info " << key_value.first << " has been initialized without active manipulator!");

                            std::string full_arm_context_name = "";
                            std::string arm_context_name = "";
                            std::string end_effector_context_name = "";
                            std::string start_link = "";
                            std::string end_link = "";
                            std::string ik_database = "";
                            bool left_arm_ik = true;

                            if(parameters::has_attribute("manipulation_context_info/full_arm_context_name", key_value.second, NULL))
                                full_arm_context_name = parameters::get_attribute("manipulation_context_info/full_arm_context_name",key_value.second,NULL);
                            else
                                PRX_FATAL_S("The manipulation context info " << key_value.first << " has been initialized without full_arm_context_name");

                            if(parameters::has_attribute("manipulation_context_info/arm_context_name", key_value.second, NULL))
                                arm_context_name = parameters::get_attribute("manipulation_context_info/arm_context_name",key_value.second,NULL);
                            else
                                PRX_FATAL_S("The manipulation context info " << key_value.first << " has been initialized without arm_context_name");
                            
                            if(parameters::has_attribute("manipulation_context_info/end_effector_context_name", key_value.second, NULL))
                                end_effector_context_name = parameters::get_attribute("manipulation_context_info/end_effector_context_name",key_value.second,NULL);
                            else
                                PRX_FATAL_S("The manipulation context info " << key_value.first << " has been initialized without end_effector_context_name");
                            
                            if(parameters::has_attribute("manipulation_context_info/start_link", key_value.second, NULL))
                                start_link = parameters::get_attribute("manipulation_context_info/start_link",key_value.second,NULL);
                            else
                                PRX_FATAL_S("The manipulation context info " << key_value.first << " has been initialized without start_link");
                            
                            if(parameters::has_attribute("manipulation_context_info/end_link", key_value.second, NULL))
                                end_link = parameters::get_attribute("manipulation_context_info/end_link",key_value.second,NULL);
                            else
                                PRX_FATAL_S("The manipulation context info " << key_value.first << " has been initialized without end_link");
                             
                            contexts[key_value.first].info = new manipulation_context_info_t(this,manipulator,full_arm_context_name,arm_context_name,end_effector_context_name,start_link,end_link);

                            if(parameters::has_attribute("manipulation_context_info/ik_database", key_value.second, NULL))
                            {
                                manipulation_context_info_t* active_info = (manipulation_context_info_t*)contexts[key_value.first].info;
                                ik_database = parameters::get_attribute("manipulation_context_info/ik_database",key_value.second,NULL);
                                left_arm_ik = parameters::get_attribute_as<bool>("manipulation_context_info/left_arm_ik",key_value.second,NULL,true);
                                active_info->ik_database = NULL;
                                for(int i=0;i<ik_databases.size();i++)
                                {
                                    if(ik_databases[i].first==ik_database)
                                    {
                                        active_info->ik_database = ik_databases[i].second;
                                    }
                                }
                                active_info->ik_left_arm = left_arm_ik;
                            }
                            else
                            {
                                manipulation_context_info_t* active_info = (manipulation_context_info_t*)contexts[key_value.first].info;
                                active_info->ik_database = NULL;
                            }

                        }
                    }
                }
            }

            void manipulation_world_model_t::push_state(const sim::state_t * const source)
            {
                world_model_t::push_state(source);
            }

            void manipulation_world_model_t::use_context(std::string name)
            {
                if( context_name != name )
                {
                    // PRX_DEBUG_COLOR("Going to change the active_manipulation_info " << name << "    info: " <<  contexts[name].info ,PRX_TEXT_CYAN);
                    active_manipulation_info = dynamic_cast<manipulation_context_info_t*>(contexts[name].info);                    
                }
                world_model_t::use_context(name);
            }

            void manipulation_world_model_t::get_objects(std::vector<movable_body_plant_t* >& objects)
            {
                manip_simulator->get_movable_objects(objects);
            }

            void manipulation_world_model_t::set_static_relative_config( bool flag)
            {
                //If the end effector is grasping an object and we want to maintain the relative configuration static
                if(flag && manip_simulator->is_end_effector_grasping(active_manipulation_info->manipulator, active_manipulation_info->chain.second))
                {
                    manip_simulator->set_static_relative_config(active_manipulation_info->end_effector_pathname, flag);
                }
                //If we want to stop using the relative configuration.
                else if (!flag)
                {
                    manip_simulator->set_static_relative_config(active_manipulation_info->end_effector_pathname, flag);
                }
                // else
                //     PRX_WARN_S("You tried to set static relative configuration when the end effector " << active_manipulation_info->chain.second << " does not grasp anything.");
            }
            
            void manipulation_world_model_t::engage_grasp( plan_t& plan, int grasping_mode, bool static_relative_flag)
            {
                PRX_ASSERT(selected_control_space == active_manipulation_info->full_arm_control_space);
                //PRX_ASSERT(!manip_simulator->is_end_effector_grasping(active_manipulation_info->manipulator, active_manipulation_info->chain.second));
                active_manipulation_info->end_effector_control->memory[0] = grasping_mode;
                active_manipulation_info->end_effector_control_space->copy_from_point(active_manipulation_info->end_effector_control);
                active_manipulation_info->arm_control_space->zero(active_manipulation_info->arm_control);
                active_manipulation_info->arm_control_space->copy_from_point(active_manipulation_info->arm_control);
                active_manipulation_info->full_arm_control_space->copy_to_point(active_manipulation_info->full_arm_control);
                active_manipulation_info->full_arm_state_space->copy_to_point(active_manipulation_info->full_arm_state);                
                propagate_once(active_manipulation_info->full_arm_state, active_manipulation_info->full_arm_control, simulation::simulation_step, active_manipulation_info->full_arm_state);
                plan.copy_onto_back(active_manipulation_info->full_arm_control, simulation::simulation_step);
                set_static_relative_config(static_relative_flag);
            }

            int manipulation_world_model_t::get_current_grasping_mode()
            {
                active_manipulation_info->end_effector_state_space->copy_to_point(active_manipulation_info->end_effector_state);
                return active_manipulation_info->end_effector_state->at(0);
            }

            manipulation_context_info_t* manipulation_world_model_t::get_current_manipulation_info()
            {
                return active_manipulation_info;
            }

            void manipulation_world_model_t::steering_function(plan_t& result_plan, const state_t* start, const state_t* goal)
            {
                selected_state_space->copy_from_point(goal);
                active_manipulation_info->full_manipulator_state_space->copy_to_point(active_manipulation_info->full_target_manipulator_state);

                selected_state_space->copy_from_point(start);
                active_manipulation_info->full_manipulator_state_space->copy_to_point(active_manipulation_info->full_manipulator_state);

                active_manipulation_info->manipulator_plan.clear();
                active_manipulation_info->manipulator->steering_function(active_manipulation_info->full_manipulator_state, active_manipulation_info->full_target_manipulator_state, active_manipulation_info->manipulator_plan);
                convert_plan(result_plan, selected_control_space, active_manipulation_info->manipulator_plan, active_manipulation_info->full_manipulator_control_space);                
            }

            void manipulation_world_model_t::steering_function(const sim::state_t* start, const sim::state_t* goal, sim::plan_t& result)
            {
                if(!IK_steer_paths)
                {
                    world_model_t::steering_function(start,goal,result);
                }
                else
                {
                    std::string old_context = get_current_context();
                    space_t* s_space = selected_state_space;
                    space_t* c_space = selected_control_space;
                    use_context(IK_steer_context);
                    convert_spaces(active_manipulation_info->full_manipulator_state_space, active_manipulation_info->full_target_manipulator_state, s_space, goal);
                    convert_spaces(active_manipulation_info->full_manipulator_state_space, active_manipulation_info->full_manipulator_state, s_space, start);
                    active_manipulation_info->manipulator_plan.clear();
                    config_t goal_config;
                    active_manipulation_info->full_manipulator_state_space->copy_from_point(active_manipulation_info->full_manipulator_state);
                    active_manipulation_info->manipulator->FK_solver(goal_config, active_manipulation_info->chain.second);

                    if(active_manipulation_info->manipulator->IK_steering(active_manipulation_info->manipulator_plan, active_manipulation_info->full_manipulator_state, active_manipulation_info->full_manipulator_state, goal_config, active_manipulation_info->chain.first, active_manipulation_info->chain.second))
                    {
                        convert_plan(result, c_space, active_manipulation_info->manipulator_plan, active_manipulation_info->full_manipulator_control_space);
                    }
                    else
                    {
                        world_model_t::steering_function(start,goal,result);
                    }

                    use_context(old_context);
                }
            }


            bool manipulation_world_model_t::IK( sim::state_t* result_state, const util::space_point_t* start_state, const util::config_t& goal_config,bool validate)
            {
                selected_state_space->copy_from_point(start_state);
                active_manipulation_info->full_manipulator_state_space->copy_to_point(active_manipulation_info->full_manipulator_state);
                if(active_manipulation_info->ik_database!=NULL)
                {
                    state_t* temp_full_point = active_manipulation_info->full_manipulator_state_space->alloc_point();
                    active_manipulation_info->end_effector_state_space->copy_to_point(active_manipulation_info->end_effector_state);
                    std::vector< state_t* > states;
                    active_manipulation_info->ik_database->get_near_neighbors(  states, goal_config, ik_seeds, active_manipulation_info->ik_left_arm );
                    for (int i = 0; i < states.size(); ++i)
                    {
                        active_manipulation_info->full_manipulator_state_space->copy_from_point(states[i]);
                        active_manipulation_info->end_effector_state_space->copy_from_point(active_manipulation_info->end_effector_state);
                        active_manipulation_info->full_manipulator_state_space->copy_to_point(states[i]);
                        if(active_manipulation_info->manipulator->IK_solver(temp_full_point, states[i], goal_config, active_manipulation_info->chain.first, active_manipulation_info->chain.second))
                        {
                            active_manipulation_info->full_manipulator_state_space->copy_from_point(temp_full_point);
                            selected_state_space->copy_to_point(result_state);
                            active_manipulation_info->full_manipulator_state_space->copy_from_point(active_manipulation_info->full_manipulator_state);
                            selected_state_space->copy_from_point(result_state);
                            if(!validate || !simulator->in_collision())
                            {
                                active_manipulation_info->full_manipulator_state_space->free_point(temp_full_point);
                                return true;
                            }
                        }
                    }
                    active_manipulation_info->full_manipulator_state_space->free_point(temp_full_point);
                }
                else
                {
                    state_t* temp_full_point = active_manipulation_info->full_manipulator_state_space->alloc_point();
                    active_manipulation_info->end_effector_state_space->copy_to_point(active_manipulation_info->end_effector_state);
                    int seeds = ik_seeds;
                    if(ik_seeds==0)
                    {
                        seeds = 1;
                    }
                    for (int i = 0; i < seeds; ++i)
                    {
                        active_manipulation_info->full_manipulator_state_space->uniform_sample(temp_full_point);
                        active_manipulation_info->full_manipulator_state_space->copy_from_point(temp_full_point);
                        active_manipulation_info->end_effector_state_space->copy_from_point(active_manipulation_info->end_effector_state);
                        active_manipulation_info->full_manipulator_state_space->copy_to_point(temp_full_point);
                        if(active_manipulation_info->manipulator->IK_solver(temp_full_point, temp_full_point, goal_config, active_manipulation_info->chain.first, active_manipulation_info->chain.second))
                        {
                            active_manipulation_info->full_manipulator_state_space->copy_from_point(temp_full_point);
                            selected_state_space->copy_to_point(result_state);
                            active_manipulation_info->full_manipulator_state_space->copy_from_point(active_manipulation_info->full_manipulator_state);
                            selected_state_space->copy_from_point(result_state);
                            if(!validate || !simulator->in_collision())
                            {
                                selected_state_space->copy_to_point(result_state);
                                active_manipulation_info->full_manipulator_state_space->free_point(temp_full_point);
                                return true;
                            }
                        }
                    }
                    active_manipulation_info->full_manipulator_state_space->free_point(temp_full_point);
                }

                return false;
            }

            bool manipulation_world_model_t::IK_steering( plan_t& result_plan, sim::state_t* result_state, const util::space_point_t* start_state, const util::config_t& goal_config)
            {
                selected_state_space->copy_from_point(start_state);
                active_manipulation_info->full_manipulator_state_space->copy_to_point(active_manipulation_info->full_manipulator_state);
                active_manipulation_info->manipulator_plan.clear();
                if(active_manipulation_info->manipulator->IK_steering(active_manipulation_info->manipulator_plan, active_manipulation_info->full_manipulator_state, active_manipulation_info->full_manipulator_state, goal_config, active_manipulation_info->chain.first, active_manipulation_info->chain.second))
                {
                    convert_plan(result_plan, selected_control_space, active_manipulation_info->manipulator_plan, active_manipulation_info->full_manipulator_control_space);
                    active_manipulation_info->full_manipulator_state_space->copy_from_point(active_manipulation_info->full_manipulator_state);
                    selected_state_space->copy_to_point(result_state);
                    return true;
                }
                return false;
            }

            bool manipulation_world_model_t::IK_steering( plan_t& result_plan, sim::state_t* result_state, const util::space_point_t* start_state, const util::space_point_t* final_state, const util::config_t& goal_config)
            {
                if(IK_steering(result_plan,result_state,start_state,goal_config))
                {
                    steering_function(result_plan,result_state,final_state);
                    selected_state_space->copy_point(result_state, final_state);
                    return true;
                }
                // selected_state_space->copy_from_point(start_state);
                // active_manipulation_info->state_space->copy_to_point(active_manipulation_info->full_manipulator_state);
                // active_manipulation_info->manipulator_plan.clear();
                // if(active_manipulation_info->manipulator->IK_steering(active_manipulation_info->manipulator_plan, active_manipulation_info->full_manipulator_state, active_manipulation_info->full_manipulator_state, goal_config, active_manipulation_info->chain.first, active_manipulation_info->chain.second))
                // {
                //     steering_function(result_state, final_state, result_plan);
                //     convert_plan(result_plan, selected_control_space, active_manipulation_info->manipulator_plan, active_manipulation_info->control_space);
                //     active_manipulation_info->state_space->copy_from_point(active_manipulation_info->full_manipulator_state);
                //     selected_state_space->copy_to_point(result_state);
                //     return true;
                // }
                return false;
            }
            
            void manipulation_world_model_t::FK(util::config_t& link)
            {
                active_manipulation_info->manipulator->FK_solver(link, active_manipulation_info->chain.second);
            }
            
            std::pair<std::string, std::string> manipulation_world_model_t::get_chain()
            {
                return active_manipulation_info->chain;
            }
            
            std::string manipulation_world_model_t::get_end_effector_context()
            {
                return active_manipulation_info->end_effector_context_name;
            }

            void manipulation_world_model_t::get_manipulators(std::vector<manipulator_t* >& manipulators)
            {
                manip_simulator->get_manipulators(manipulators);
            }

            manipulator_t* manipulation_world_model_t::find_active_manipulator(const std::vector<manipulator_t*>& manipulators)
            {
                foreach(manipulator_t* manip, manipulators)
                {
                    if(manip->is_active())
                        return manip;
                }
                return NULL;
            }

            void manipulation_world_model_t::get_end_effector_local_config(util::config_t& config)
            {
                manip_simulator->get_end_effector_configuration(config, active_manipulation_info->manipulator, active_manipulation_info->chain.second);                
            }

            void manipulation_world_model_t::convert_plan(sim::plan_t& output_plan, const util::space_t* output_space, const sim::plan_t& input_plan, const util::space_t* input_space)
            {
                control_t* ctrl = output_space->alloc_point();
                output_space->zero(ctrl);
                output_space->copy_from_point(ctrl);
                foreach(plan_step_t step, input_plan)
                {
                    input_space->copy_from_point(step.control);
                    output_space->copy_to_point(ctrl);
                    output_plan.copy_onto_back(ctrl,step.duration);
                }
                output_space->free_point(ctrl);
            }
        }
    }
}