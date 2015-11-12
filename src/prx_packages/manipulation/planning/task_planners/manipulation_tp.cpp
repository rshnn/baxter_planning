/**
 * @file manipulation_tp.cpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#include "planning/task_planners/manipulation_tp.hpp"

#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/definitions/random.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/statistics/statistics.hpp"
#include "prx/planning/communication/visualization_comm.hpp"

#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"

#include <boost/range/adaptor/map.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::manipulation_tp_t, prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        namespace manipulation
        {

            manipulation_tp_t::manipulation_tp_t()
            {

            }

            manipulation_tp_t::~manipulation_tp_t()
            {

            }

            void manipulation_tp_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                PRX_INFO_S("Initializing Manipulation task planner ...");

                std::string template_name;
                planner_t::init(reader, template_reader);
                
                //initialize the motion planners
                if( reader->has_attribute("planners") )
                {
                    parameter_reader_t::reader_map_t planner_map = reader->get_map("planners");

                    foreach(const parameter_reader_t::reader_map_t::value_type key_value, planner_map)
                    {
                        PRX_INFO_S("Creating a planner with name: " << key_value.first);
                        const parameter_reader_t* child_template_reader = NULL;

                        if( key_value.second->has_attribute("template") )
                        {
                            template_name = key_value.second->get_attribute("template");
                            child_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + template_name);
                        }
                        planner_t* new_planner = parameters::create_from_loader<planner_t > ("prx_planning", key_value.second, "", child_template_reader, "");
                        std::string planner_name = key_value.first;
                        new_planner->set_pathname(path + "/" + planner_name);
                        parameters::initialize(new_planner,key_value.second,"",child_template_reader,"");
                        //perform an operation to get the mappings from the world model and pass them to the motion planners
                        planner_names.push_back(planner_name);
                        planners[planner_name] = new_planner;
                        planners[planner_name]->set_pathname(path + "/" + planner_name);

                        space_names[planner_name] = parameters::get_attribute_as<std::string > ("space_name", key_value.second, child_template_reader);

                        //Initialize the specification for this motion planner.
                        if(!parameters::has_attribute("specification", key_value.second,child_template_reader))
                            PRX_FATAL_S("The motion planner inside the manipulation task planner has to setup a specification!");

                        const parameter_reader_t* specification_template_reader = NULL;
                        if(parameters::has_attribute("specification/template", key_value.second, child_template_reader))
                        {
                            template_name = parameters::get_attribute("specification/template",key_value.second,child_template_reader);
                            specification_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + template_name);
                        }

                        specification_t* new_spec = parameters::initialize_from_loader<specification_t > ("prx_planning", key_value.second, "specification", specification_template_reader, "");
                        
                        //Initialize the query for this motion planner.
                        if(!parameters::has_attribute("query", key_value.second,child_template_reader))
                            PRX_FATAL_S("The motion planner inside the manipulation task planner has to setup a query!");

                        const parameter_reader_t* query_template_reader = NULL;
                        if(parameters::has_attribute("query/template", key_value.second, child_template_reader))
                        {
                            template_name = parameters::get_attribute("query/template",key_value.second,child_template_reader);
                            query_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + template_name);
                        }

                        query_t* new_query = parameters::initialize_from_loader<query_t > ("prx_planning", key_value.second, "query", query_template_reader, "");

                        std::string planning_context_name = parameters::get_attribute_as<std::string > ("planning_context_name", key_value.second, child_template_reader);
                        planner_info_map[planning_context_name] = new planner_info_t(space_names[planner_name], planning_context_name, new_planner, new_spec, new_query);

                        if( child_template_reader != NULL )
                        {
                            delete child_template_reader;
                            child_template_reader = NULL;
                        }

                        if( query_template_reader != NULL )
                        {
                            delete query_template_reader;
                            query_template_reader = NULL;
                        }

                        if(specification_template_reader != NULL)
                        {
                            delete specification_template_reader;
                            specification_template_reader = NULL;
                        }
                    }
                } 

                //Initializing the grasping planner.
                if(!parameters::has_attribute("grasping_planner",reader,template_reader))
                    PRX_FATAL_S("Manipulation task planner needs a grasping planner!");

                const parameter_reader_t* grasping_planner_template_reader = NULL;

                if( parameters::has_attribute("grasping_planner/template",reader,template_reader) )
                {
                    std::string template_name = parameters::get_attribute("grasping_planner/template",reader,template_reader);
                    grasping_planner_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + template_name);
                }

                planner_t* tmp_grasping_planner = parameters::create_from_loader<planner_t > ("prx_planning", reader, "grasping_planner", grasping_planner_template_reader, "");
                grasping_planner = dynamic_cast<grasping_planner_t *>(tmp_grasping_planner);
                grasping_planner->set_pathname(path + "/grasping_planner");
                parameters::initialize(grasping_planner, reader, "grasping_planner", grasping_planner_template_reader, "");

                if( grasping_planner_template_reader != NULL )
                {
                    delete grasping_planner_template_reader;
                    grasping_planner_template_reader = NULL;
                }

                
                IK_steer_movements = parameters::get_attribute_as<bool>("IK_steer_grasping", reader, template_reader, false);

                serialize_flag = parameters::get_attribute_as<bool>("serialize_flag", reader, template_reader, false);
            }

            void manipulation_tp_t::setup()
            {
                PRX_INFO_S("Setup manipulation tp ...");
                std::string old_context = manipulation_model->get_current_context();
                foreach(planner_info_t* planner_info, planner_info_map | boost::adaptors::map_values)
                {
                    manipulation_model->use_context(planner_info->construction_context_name);
                    planner_info->setup(manipulation_model);
                    planner_info->planner->setup();
                    manipulation_model->use_context(planner_info->planning_context_name);
                }
                grasping_planner->setup();

                manipulation_model->use_context(old_context);
                grasping_query = new grasping_query_t();
            }

            void manipulation_tp_t::reset()
            {
                
            }

            void manipulation_tp_t::link_world_model(world_model_t * const model)
            {
                task_planner_t::link_world_model(model);
                manipulation_model = dynamic_cast<manipulation_world_model_t*>(model);
                if(manipulation_model == NULL)
                    PRX_FATAL_S("The manipulation task planner can work only with manipulation world model!");
                grasping_planner->link_world_model(model);
            }

            const statistics_t* manipulation_tp_t::get_statistics()
            {
                PRX_WARN_S("Get statistics for manipulation task planner is not implemented!");
                return new statistics_t();
            }

            void manipulation_tp_t::link_specification(specification_t* new_spec)
            {
                task_planner_t::link_specification(new_spec);
            }

            void manipulation_tp_t::link_query(query_t* new_query)
            {
                task_planner_t::link_query(new_query);
                manipulation_query = dynamic_cast<manipulation_query_t*>(new_query);
                if(manipulation_query == NULL) 
                    PRX_FATAL_S("The manipulation task planner operate only over a manipulation query as input!");

                PRX_DEBUG_COLOR("link query manipulation_tp context name: " << manipulation_query->manipulation_context_name, PRX_TEXT_BROWN);
                manipulation_model->use_context(manipulation_query->manipulation_context_name);
                current_manipulation_context_info = manipulation_model->get_current_manipulation_info();
                active_planner = planner_info_map[current_manipulation_context_info->arm_context_name];
                // PRX_DEBUG_COLOR("control space: " << current_manipulation_context_info->full_arm_control_space->get_space_name(), PRX_TEXT_BROWN);
                manipulation_query->link_spaces(current_manipulation_context_info->full_arm_state_space,current_manipulation_context_info->full_arm_control_space);
                // PRX_DEBUG_COLOR("manipulation_query control space: " << manipulation_query->control_space->get_space_name(), PRX_TEXT_BLUE);                
            }        

            bool manipulation_tp_t::serialize()
            {
                std::string old_context = manipulation_model->get_current_context();
                foreach(planner_info_t* planner_info, planner_info_map | boost::adaptors::map_values)
                {
                    manipulation_model->use_context(planner_info->planning_context_name);
                    if(!planner_info->planner->serialize())
                        PRX_FATAL_S("Planner " << planner_info->planner_name << " failed to serialize!");
                }
                manipulation_model->use_context(old_context);
                return true;
            }

            bool manipulation_tp_t::deserialize()
            {
                std::string old_context = manipulation_model->get_current_context();
                foreach(planner_info_t* planner_info, planner_info_map | boost::adaptors::map_values)
                {
                    manipulation_model->use_context(planner_info->planning_context_name);
                    if(!planner_info->planner->deserialize())
                        PRX_FATAL_S("Planner " << planner_info->planner_name << " failed to deserialize!");
                }
                manipulation_model->use_context(old_context);                
                return true;
            }

            bool manipulation_tp_t::succeeded() const
            {
                return true;
                //return false;
            }

            bool manipulation_tp_t::execute()
            {
                
                std::string old_context = manipulation_model->get_current_context();
                foreach(planner_info_t* planner_info, planner_info_map | boost::adaptors::map_values)
                {
                    manipulation_model->use_context(planner_info->construction_context_name);
                    try
                    {
                        if(!planner_info->planner->execute())
                            PRX_FATAL_S("Planner " << planner_info->planner_name << " failed to execute!");
                    }
                    catch( stopping_criteria_t::stopping_criteria_satisfied e )                
                    {

                    }
                }
                manipulation_model->use_context(old_context);

                if( serialize_flag )
                    serialize();
                update_vis_info();
                ((plan::comm::visualization_comm_t*)plan::comm::vis_comm)->send_geometries();

                return true;
            }

            void manipulation_tp_t::resolve_query()
            {
                PRX_DEBUG_COLOR("Manip TP :: Resolve Query",PRX_TEXT_MAGENTA)
                if(manipulation_query->mode == manipulation_query_t::PRX_PICK)
                    manipulation_query->found_path = pick(manipulation_query->plan, manipulation_query->manipulator_final_state, manipulation_query->manipulator_initial_state, manipulation_query->grasping_index);
                else if(manipulation_query->mode == manipulation_query_t::PRX_PLACE)
                    manipulation_query->found_path = place(manipulation_query->plan, manipulation_query->manipulator_final_state, manipulation_query->manipulator_initial_state, manipulation_query->grasping_index);
                else if(manipulation_query->mode == manipulation_query_t::PRX_TRANSFER)
                    manipulation_query->found_path = transfer(manipulation_query->plan, manipulation_query->manipulator_initial_state, manipulation_query->manipulator_final_state);
                else if(manipulation_query->mode == manipulation_query_t::PRX_MOVE)
                    manipulation_query->found_path = move(manipulation_query->plan, manipulation_query->manipulator_initial_state, manipulation_query->manipulator_target_state);
                else if(manipulation_query->mode == manipulation_query_t::PRX_PICK_AND_PLACE)
                    manipulation_query->found_path = pick_and_place(manipulation_query->plan, manipulation_query->manipulator_initial_state, manipulation_query->manipulator_target_state, manipulation_query->grasping_index);
                PRX_DEBUG_COLOR("Manip TP :: Resolve Query End",PRX_TEXT_MAGENTA)
            }

            bool manipulation_tp_t::resolve_grasp_query(state_t* object_state, int grasping_index, int default_open_mode)
            {
                plan_t plan;
                int current_end_effector_mode = -1;
                //We store the current state of the manipulator to restore the manipulator after the grasping planner is finished.
                state_t* our_full_manipulator = current_manipulation_context_info->full_manipulator_state_space->alloc_point();

                if(current_manipulation_context_info->is_end_effector_closed())
                {
                    current_end_effector_mode = manipulation_model->get_current_grasping_mode();
                    plan.link_control_space(current_manipulation_context_info->full_arm_control_space);
                    //We are going to open the hand and use the default_open_mode for that. 
                    manipulation_model->engage_grasp(plan, default_open_mode, false);
                    current_manipulation_context_info->full_manipulator_state_space->copy_to_point(our_full_manipulator);
                }

                grasping_query->setup(current_manipulation_context_info->full_arm_state_space, current_manipulation_context_info->full_arm_control_space, manipulation_query->object, grasping_index, object_state, manipulation_query->retraction_config);
                grasping_planner->link_query(grasping_query);
                grasping_planner->resolve_query();

                current_manipulation_context_info->full_manipulator_state_space->copy_from_point(our_full_manipulator);
                if(current_end_effector_mode != -1)
                {
                    PRX_DEBUG_COLOR("Engaging a grasp since we were grasping before: "<<current_end_effector_mode,PRX_TEXT_MAGENTA);
                    // PRX_DEBUG_COLOR("-- "<<current_manipulation_context_info->full_manipulator_state_space->print_point(our_full_manipulator,3),PRX_TEXT_RED);
                    PRX_DEBUG_COLOR(manipulation_model->get_full_state_space()->print_memory(3),PRX_TEXT_RED);
                    manipulation_model->engage_grasp(plan,current_end_effector_mode,true);
                    PRX_DEBUG_COLOR(manipulation_model->get_full_state_space()->print_memory(3),PRX_TEXT_RED);
                }                
                return grasping_query->found_grasp;
            }

            bool manipulation_tp_t::pick(plan_t& plan, state_t* manipulator_final_state, state_t* manipulator_initial_state, int grasping_index)
            {
                manipulation_model->enable_IK_steering(IK_steer_movements,manipulation_model->get_current_context());
                manipulation_model->get_state_space()->copy_from_point(manipulator_initial_state);
                PRX_DEBUG_COLOR("--Manip TP :: Pick "<<manipulation_model->get_full_state_space()->print_memory(1),PRX_TEXT_MAGENTA)
                if(resolve_grasp_query(manipulation_query->object_initial_state, grasping_index, manipulation_query->default_open_mode))
                {
                    PRX_DEBUG_COLOR("Grasp found",PRX_TEXT_BROWN);

                    state_t* actual_start_state = manipulation_model->get_state_space()->clone_point(manipulator_initial_state);
                    manipulation_model->get_state_space()->copy_from_point(actual_start_state);

                    if(manipulation_model->get_current_grasping_mode()!=grasping_query->release_mode)
                    {
                        PRX_DEBUG_COLOR("Different open configuration: "<<grasping_query->release_mode,PRX_TEXT_LIGHTGRAY);
                        manipulation_model->engage_grasp(plan, grasping_query->release_mode, false);
                        manipulation_model->get_state_space()->copy_to_point(actual_start_state);
                    }
                    bool success=false;
                    //compute a path from the motion planner in the manipulation task planner. The compute solution will change the plan to be the correct space.
                    if(active_planner->compute_solution(plan, manipulation_model, actual_start_state, grasping_query->retracted_open_state,manipulation_query->ik_steer_paths))
                    {
                        PRX_DEBUG_COLOR("Path found",PRX_TEXT_BROWN);
                        plan += grasping_query->reaching_plan;
                        manipulation_model->push_state(grasping_query->releasing_state);
                        manipulation_model->engage_grasp(plan, grasping_query->grasping_mode, true);
                        if(manipulator_final_state!=NULL)
                            current_manipulation_context_info->full_arm_state_space->copy_point(manipulator_final_state,grasping_query->grasping_state);
                        PRX_DEBUG_COLOR("--Manip TP :: Pick End True "<<manipulation_model->get_full_state_space()->print_memory(1),PRX_TEXT_MAGENTA)
                        manipulation_query->grasping_index = grasping_index;
                        success = true;
                    }
                    manipulation_model->get_state_space()->free_point(actual_start_state);
                    if(success)
                    {
                        return true;
                    }

                }
                manipulation_query->grasping_index = -1;
                PRX_DEBUG_COLOR("--Manip TP :: Pick End  False "<<manipulation_model->get_full_state_space()->print_memory(1),PRX_TEXT_MAGENTA)
                return false;
            }

            bool manipulation_tp_t::place(plan_t& plan, state_t* manipulator_final_state, state_t* manipulator_initial_state, int grasping_index)
            {
                manipulation_model->enable_IK_steering(IK_steer_movements,manipulation_model->get_current_context());
                manipulation_model->get_state_space()->copy_from_point(manipulator_initial_state);
                PRX_ASSERT(grasping_index!=-1);
                PRX_DEBUG_COLOR("--Manip TP :: Place "<<manipulation_model->get_full_state_space()->print_memory(1),PRX_TEXT_MAGENTA)   
                if(resolve_grasp_query(manipulation_query->object_target_state, grasping_index, manipulation_query->default_open_mode))
                {
                    PRX_DEBUG_COLOR("Grasp found",PRX_TEXT_BROWN);
                    //compute a path from the motion planner in the manipulation task planner. The compute solution will change the plan to be the correct space.
                    if(active_planner->compute_solution(plan, manipulation_model, manipulator_initial_state, grasping_query->grasping_state,manipulation_query->ik_steer_paths))
                    {
                        PRX_DEBUG_COLOR("Path found",PRX_TEXT_BROWN);
                        //Here we have to release the object.
                        manipulation_model->push_state(grasping_query->grasping_state);

                        if(manipulation_query->default_open_mode!=grasping_query->release_mode)
                        {
                            manipulation_model->engage_grasp(plan, grasping_query->release_mode, false);
                        }
                        else
                        {
                            manipulation_model->engage_grasp(plan, manipulation_query->default_open_mode, false);
                        }
                        plan += grasping_query->retracting_plan;

                        manipulation_model->push_state(grasping_query->retracted_open_state);

                        if(manipulation_query->default_open_mode!=grasping_query->release_mode)
                        {
                            manipulation_model->engage_grasp(plan, manipulation_query->default_open_mode, false);
                        }

                        if(manipulator_final_state!=NULL)
                            current_manipulation_context_info->full_arm_state_space->copy_point(manipulator_final_state,grasping_query->retracted_open_state);                        
                        PRX_DEBUG_COLOR("--Manip TP :: Place End True "<<manipulation_model->get_full_state_space()->print_memory(1),PRX_TEXT_MAGENTA)
                        return true;
                    }

                }
                PRX_DEBUG_COLOR("--Manip TP :: Place End False "<<manipulation_model->get_full_state_space()->print_memory(1),PRX_TEXT_MAGENTA)
                return false;
            }

            bool manipulation_tp_t::move(plan_t& plan, state_t* manipulator_initial_state, state_t* manipulator_target_state)
            {
                manipulation_model->enable_IK_steering(false,"");
                manipulation_model->get_state_space()->copy_from_point(manipulator_initial_state);
                PRX_DEBUG_COLOR("--Manip TP :: Move "<<manipulation_model->get_full_state_space()->print_memory(1),PRX_TEXT_MAGENTA)
                bool success = active_planner->compute_solution(plan, manipulation_model, manipulator_initial_state, manipulator_target_state,manipulation_query->ik_steer_paths);
                if(success)
                    manipulation_model->push_state(manipulator_target_state);
                PRX_DEBUG_COLOR("--Manip TP :: Move End "<<success,PRX_TEXT_MAGENTA)
                return success;
            }

            bool manipulation_tp_t::transfer(plan_t& plan, state_t* manipulator_initial_state, state_t* manipulator_target_state)
            {
                manipulation_model->enable_IK_steering(false,"");
                PRX_DEBUG_COLOR("--Manip TP :: Transfer "<<manipulation_model->get_full_state_space()->print_memory(1),PRX_TEXT_MAGENTA)

                //get the config from the object to the end effector
                config_t ee_config;
                manipulation_model->FK(ee_config);
                config_t object_config;
                manipulation_query->object->get_configuration(object_config);
                ee_config.global_to_relative(object_config);

                //compute new ee placement
                state_t* init_object = manipulation_query->object->get_state_space()->alloc_point();
                manipulation_query->object->get_state_space()->copy_from_point(manipulation_query->object_target_state);
                manipulation_query->object->get_configuration(object_config);
                manipulation_query->object->get_state_space()->copy_from_point(init_object);
                ee_config.relative_to_global(object_config);

                state_t* final_state = NULL;
                if(manipulator_target_state!=NULL)
                    final_state = manipulator_target_state;
                else
                    final_state = current_manipulation_context_info->full_arm_state_space->alloc_point();
                bool success = manipulation_model->IK(final_state, manipulator_initial_state, ee_config);
                

                PRX_DEBUG_COLOR("--Manip TP :: Transfer IK: "<<success,PRX_TEXT_MAGENTA);

                if(!success)
                {
                    if(manipulator_target_state==NULL)
                        current_manipulation_context_info->full_arm_state_space->free_point(final_state);
                    return success;
                }

                //move
                success = active_planner->compute_solution(plan, manipulation_model, manipulator_initial_state, final_state,manipulation_query->ik_steer_paths);

                manipulation_model->push_state(final_state);

                if(manipulator_target_state==NULL)
                    current_manipulation_context_info->full_arm_state_space->free_point(final_state);
                PRX_DEBUG_COLOR("--Manip TP :: Transfer End "<<success,PRX_TEXT_MAGENTA);
                manipulation_query->object->get_state_space()->free_point(init_object);
                return success;
            }

            bool manipulation_tp_t::pick_and_place(plan_t& plan, state_t* manipulator_initial_state, state_t* manipulator_target_state, int grasping_index)
            {
                state_t* tmp_state = current_manipulation_context_info->full_arm_state_space->alloc_point();
                //If the grasping id was -1 we have to be sure that the same grasping id will be used for the action place, 
                //thats why we are using the grasping id from the grasping query. 
                bool success = false;

                int previous_grasping_mode = manipulation_model->get_current_grasping_mode();

                if(grasping_index == -1)
                {
                    int sz = grasping_planner->nr_grasps(current_manipulation_context_info->full_arm_context_name, manipulation_query->object);
                    for(int i = 0; i<sz && !success; ++i)
                    {
                        manipulation_query->object->get_state_space()->copy_from_point(manipulation_query->object_initial_state);
                        plan.clear();
                        if(pick(plan, tmp_state, manipulator_initial_state, i))
                        {
                            if(place(plan, tmp_state, tmp_state, grasping_query->grasping_index))
                            {
                                manipulation_query->object->get_state_space()->copy_from_point(manipulation_query->object_target_state);
                                if(move(plan, tmp_state, manipulator_target_state))
                                {
                                    PRX_INFO_S("Successful Grasp Index: " << grasping_query->grasping_index);
                                    success = true;
                                }
                            }
                            else
                            {
                                manipulation_model->engage_grasp(plan, previous_grasping_mode, false); 
                                manipulation_query->grasping_index = -1;
                            }
                        }
                    }
                }
                else
                {
                    manipulation_query->object->get_state_space()->copy_from_point(manipulation_query->object_initial_state);
                    plan.clear();
                    if(pick(plan, tmp_state, manipulator_initial_state, grasping_index))
                    {
                        if(place(plan, tmp_state, tmp_state, grasping_query->grasping_index))
                        {
                            manipulation_query->object->get_state_space()->copy_from_point(manipulation_query->object_target_state);
                            if(move(plan, tmp_state, manipulator_target_state))
                                success = true;
                        }
                        else
                        {
                            manipulation_model->engage_grasp(plan, previous_grasping_mode, false); 
                            manipulation_query->grasping_index = -1;
                        }
                    }
                }
                manipulation_query->object->get_state_space()->copy_from_point(manipulation_query->object_initial_state);
                current_manipulation_context_info->full_arm_state_space->free_point(tmp_state);
                return success;
            }

            void manipulation_tp_t::update_vis_info() const
            {
                foreach(planner_info_t* planner_info, planner_info_map | boost::adaptors::map_values)
                {
                    manipulation_model->use_context(planner_info->construction_context_name);
                    planner_info->planner->update_visualization();
                }
            }
        }
    }
}
