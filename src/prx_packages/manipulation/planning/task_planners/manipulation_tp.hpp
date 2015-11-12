/**
 * @file manipulation_tp_t.hpp
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

#pragma once

#ifndef PRX_MANIPULATION_TP_HPP
#define	PRX_MANIPULATION_TP_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/spaces/space.hpp"

#include "prx/simulation/plan.hpp"
#include "prx/simulation/trajectory.hpp"

#include "prx/planning/task_planners/task_planner.hpp"
#include "prx/planning/motion_planners/motion_planner.hpp"
#include "prx/planning/queries/query.hpp"
#include "prx/planning/problem_specifications/specification.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"

#include "planning/manipulation_world_model.hpp"
#include "planning/queries/manipulation_query.hpp"
#include "planning/queries/grasping_query.hpp"
#include "planning/task_planners/grasping_planner.hpp"

namespace prx
{
    namespace packages
    {
        namespace manipulation
        {
            class planner_info_t
            {
              public:
                std::string construction_context_name;
                std::string planning_context_name;
                std::string planner_name;
                plan::motion_planner_t* planner;
                plan::motion_planning_query_t* query;
                plan::specification_t* specification;
                //The spaces for the motion planner. 
                const util::space_t* state_space;
                const util::space_t* control_space;
                sim::state_t* start_point;
                sim::state_t* end_point;

                planner_info_t(std::string construction_context_name, std::string planning_context_name, plan::planner_t* planner, plan::specification_t* specification, plan::query_t* query)
                {
                    this->construction_context_name = construction_context_name;
                    this->planning_context_name = planning_context_name;
                    this->planner = dynamic_cast<plan::motion_planner_t*>(planner);
                    planner_name = planner->get_name();
                    if(this->planner == NULL)
                        PRX_FATAL_S("Manipulation task planner can only work with motion planners!");
                    this->query = dynamic_cast<plan::motion_planning_query_t*>(query);
                    if(this->query == NULL)
                        PRX_FATAL_S("Manipulation task planner van have only motion planning queries as output queries.");
                    this->specification = specification;
                }

                virtual ~planner_info_t()
                {
                    state_space->free_point(start_point);
                    state_space->free_point(end_point);
                }

                void setup(manipulation_world_model_t* model)
                {                    
                    state_space = model->get_state_space();
                    control_space = model->get_control_space();
                    start_point = state_space->alloc_point();
                    end_point = state_space->alloc_point();
                    specification->link_spaces(state_space, control_space);
                    specification->setup(model);
                    query->link_spaces(state_space, control_space);   
                    planner->link_specification(specification);
                }

                bool compute_solution(sim::plan_t& plan, manipulation_world_model_t* model, sim::state_t* start_state, sim::state_t* goal_state, bool ik_steer)
                {
                    if(ik_steer)
                    {
                        PRX_PRINT("Trying IK steering first...",PRX_TEXT_BLUE);
                        util::config_t goal_config;
                        model->get_state_space()->copy_from_point(goal_state);
                        model->FK(goal_config);
                        sim::state_t* result_state = model->get_state_space()->alloc_point();
                        sim::plan_t new_plan(model->get_control_space());
                        bool success = model->IK_steering( new_plan, result_state, start_state, goal_state, goal_config);
                        model->get_state_space()->free_point(result_state);
                        sim::trajectory_t traj(model->get_state_space());
                        if(success)
                        {
                            PRX_PRINT("IK steering success...",PRX_TEXT_BLUE);
                            specification->local_planner->propagate(start_state,new_plan,traj);
                            if(specification->validity_checker->is_valid(traj))
                            {
                                PRX_PRINT("Valid IK steering trajectory...",PRX_TEXT_BLUE);
                                // PRX_INFO_S("\n"<<traj.print(3));
                                plan+=new_plan;
                                return true;
                            }
                        }

                        PRX_PRINT("Failed IK steering. Continuing to query roadmap...",PRX_TEXT_BLUE);
                    }


                    model->get_state_space()->copy_from_point(goal_state);
                    state_space->copy_to_point(end_point);

                    model->get_state_space()->copy_from_point(start_state);
                    state_space->copy_to_point(start_point);

                    // PRX_DEBUG_COLOR("Start point: " << state_space->print_point(start_point), PRX_TEXT_LIGHTGRAY);
                    // PRX_DEBUG_COLOR("END point  : " << state_space->print_point(end_point), PRX_TEXT_MAGENTA);

                    std::string old_context = model->get_current_context();
                    model->use_context(planning_context_name);
                    query->setup(start_point, end_point);
                    planner->link_query(query);
                    planner->resolve_query();

                    // {
                    //     sim::trajectory_t new_traj(model->get_state_space());
                    //     specification->local_planner->propagate(start_point,query->plan,new_traj);

                    //     foreach(sim::state_t* state, new_traj)
                    //     {
                    //         if(specification->validity_checker->is_valid(state))
                    //         {
                    //             PRX_PRINT("Valid Roadmap State...\n"<<model->get_full_state_space()->print_memory(4),PRX_TEXT_BLUE);
                    //         }
                    //         else
                    //         {
                    //             PRX_PRINT("Invalid Roadmap trajectory...",PRX_TEXT_RED);
                    //         }
                    //     }
                    // }

                    model->use_context(old_context);
                    model->convert_plan(plan, model->get_control_space(), query->plan, control_space);
                    // if(found)
                    // {
                    //     PRX_DEBUG_COLOR("Found solution : Qp:" << query->plan.size() << "     plan: " << plan.size(),  PRX_TEXT_GREEN);
                    // }
                    // else
                    // {
                    //     PRX_DEBUG_COLOR("NO SOLUTION : Qp:" << query->plan.size() << "     plan: " << plan.size(), PRX_TEXT_RED);
                    // }
                    return query->found_solution;
                }
            };

            /**
             * Manipulation task planner. Computes the path for moving an object from an
             * initial to a target position.
             *
             * @authors Athanasios Krontiris
             */
            class manipulation_tp_t : public plan::task_planner_t
            {

              public:

                manipulation_tp_t();
                virtual ~manipulation_tp_t();

                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

                /**
                 * @copydoc motion_planner_t::reset()
                 */
                virtual void reset();

                /**
                 * @copydoc motion_planner_t::link_world_model()
                 */
                virtual void link_world_model(plan::world_model_t * const model);

                /**
                 * @copydoc motion_planner_t::get_statistics()
                 */
                virtual const util::statistics_t* get_statistics();

                /**
                 * @copydoc planner_t::link_specification(specification_t*)
                 */
                virtual void link_specification(plan::specification_t* new_spec);

                /**
                 * @copydoc motion_planner_t::link_query()
                 */
                virtual void link_query(plan::query_t* new_query);

                /**
                 * @copydoc motion_planner_t::setup()
                 *
                 * Will occupy memory for the random_open_point and the new_control, after
                 * planning_query has been linked.
                 */
                virtual void setup();

                /**
                 * @copydoc motion_planner_t::execute()
                 */
                virtual bool execute();


                /**
                 * @copydoc motion_planner_t::succeeded() const
                 */
                virtual bool succeeded() const;

                /**
                 * @copydoc motion_planner_t::resolve_query()
                 *
                 * At the end of the resolve_query the algorithm will remove the vertices
                 * for the start and the goal for this specific query from the graph.
                 */
                virtual void resolve_query();

                bool serialize();
                bool deserialize();

                int get_nr_grasps(std::string manipulation_context_name, movable_body_plant_t* requested_object)
                {
                    return grasping_planner->nr_grasps(manipulation_context_name, requested_object);
                }

              protected:

                bool IK_steer_movements;

                manipulation_world_model_t* manipulation_model;
                manipulation_context_info_t* current_manipulation_context_info;

                /** @brief A map from planning context to manipulation contexts (planner, query and spaces)*/
                util::hash_t<std::string, planner_info_t*> planner_info_map;
                /** @brief The current active context (planner, query and spaces)*/
                planner_info_t* active_planner;

                /** @brief The input manipulation query */
                manipulation_query_t* manipulation_query;

                /** @brief The grasping planner.*/
                grasping_planner_t* grasping_planner;
                /** @brief The grasping query that the manipulation task planner will use to communicate with the grasping planner.*/
                grasping_query_t* grasping_query;

                /**
                 * @brief If we want to serialize all the roadmaps after we finish building them. 
                 * If want to serialize the roadmaps the motion planners have to read the output file
                 * from their input. Otherwise the serialization will failed.
                 */
                bool serialize_flag;

                /**
                 * @brief It computes all the information we need for the object at the object_intial_state.
                 * @details [long description]
                 * 
                 * @param object_initial_state [description]
                 * @param grasping_index [description]
                 * @param default_open_mode [description]
                 * @return [description]
                 */
                virtual bool resolve_grasp_query(sim::state_t* object_state, int grasping_index, int default_open_mode);

                virtual bool pick(sim::plan_t& plan, sim::state_t* manipulator_final_state, sim::state_t* manipulator_initial_state, int grasping_id);
                virtual bool place(sim::plan_t& plan, sim::state_t* manipulator_final_state, sim::state_t* manipulator_initial_state, int grasping_id);
                virtual bool move(sim::plan_t& plan, sim::state_t* manipulator_initial_state, sim::state_t* manipulator_target_state);
                virtual bool pick_and_place(sim::plan_t& plan, sim::state_t* manipulator_initial_state, sim::state_t* manipulator_target_state, int grasping_id);
                virtual bool transfer(sim::plan_t& plan, sim::state_t* manipulator_initial_state, sim::state_t* manipulator_target_state);


                /**
                 * @copydoc planner_t::update_vis_info() const
                 */
                virtual void update_vis_info() const;
            };
        }
    }
}


#endif
