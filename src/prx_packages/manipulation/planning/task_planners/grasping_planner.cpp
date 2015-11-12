/**
 * @file grasping_planner.cpp
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

#include "planning/task_planners/grasping_planner.hpp"

#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/definitions/random.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/statistics/statistics.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"

#include "prx/planning/motion_planners/motion_planner.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"

#include "planning/manipulation_world_model.hpp"
#include "planning/queries/grasping_query.hpp"

#include <yaml-cpp/yaml.h>
#include <boost/range/adaptor/map.hpp>
#include <boost/filesystem.hpp>
#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/path.hpp"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::grasping_planner_t, prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        namespace manipulation
        {

            void operator >> (const YAML::Node& node, std::vector<double>& config) 
            {
                config.resize(7);
                config[0]=node[0].as<double>();
                config[1]=node[1].as<double>();
                config[2]=node[2].as<double>();
                config[3]=node[3].as<double>();
                config[4]=node[4].as<double>();
                config[5]=node[5].as<double>();
                config[6]=node[6].as<double>();
               
            }
            void operator >> (const YAML::Node& node, config_t& config) 
            {
                std::vector<double> vec;
                node["relative_config"] >> vec;
                config.set_position(vec[0],vec[1],vec[2]);
                config.set_xyzw_orientation(vec[3],vec[4],vec[5],vec[6]);
            }

            void operator >> (const YAML::Node& node, int& mode) 
            {
                mode = node["mode"].as<int>();
            }


            grasping_planner_t::grasping_planner_t()
            {
                char* w = std::getenv("PRACSYS_PATH");
                std::string filename(w);
                pracsys_path = filename;

                original_object_state = NULL;
            }

            grasping_planner_t::~grasping_planner_t()
            {

            }

            void grasping_planner_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                planner_t::init(reader, template_reader);
                PRX_INFO_S("Initializing Grasping task planner ...");

                if( parameters::has_attribute("validity_checker", reader, template_reader) )
                {
                    validity_checker = parameters::initialize_from_loader<validity_checker_t > ("prx_planning", reader, "validity_checker", template_reader, "validity_checker");
                }
                else
                {
                    PRX_WARN_S("Missing validity_checker attribute in planning specification!");
                }

                if( parameters::has_attribute("sampler", reader, template_reader) )
                {
                    sampler = parameters::initialize_from_loader<sampler_t > ("prx_planning", reader, "sampler", template_reader, "sampler");
                }
                else
                {
                    PRX_WARN_S("Missing sampler attribute in planning specification!");
                }

                if(parameters::has_attribute("data_folders", reader, NULL))
                {
                    parameter_reader_t::reader_map_t planner_map = reader->get_map("data_folders");

                    foreach(const parameter_reader_t::reader_map_t::value_type key_value, planner_map)
                    {
                        data_folders.push_back(std::make_pair(key_value.first,pracsys_path +"/"+ parameters::get_attribute("data_folders/"+key_value.first,reader,NULL)));
                        PRX_WARN_S(data_folders.back().first<<" "<<data_folders.back().second );
                    }
                    PRX_WARN_S(data_folders.size());
                }
                else
                {
                    PRX_FATAL_S("There is no data base specified for the grasping planner");
                }

                // if(parameters::has_attribute("retraction_config", reader, template_reader))
                // {
                //     parameters::initialize<config_t > (&retraction_config,reader,"retraction_config",template_reader,"retraction_config");
                // }
                // else
                // {
                //     retraction_config.set_position(0,0,0);
                //     retraction_config.set_orientation(0,0,0,1);
                // }
                
            }

            void grasping_planner_t::reset()
            {
                
            }

            void grasping_planner_t::link_world_model(world_model_t * const model)
            {
                manipulation_model = dynamic_cast<manipulation_world_model_t*>(model);
                if( validity_checker != NULL )
                {
                    validity_checker->link_model(model);
                }
            }

            const statistics_t* grasping_planner_t::get_statistics()
            {
                PRX_WARN_S("Get statistics for grasping planner is not implemented!");
                return new statistics_t();
            }

            void grasping_planner_t::link_specification(specification_t* new_spec)
            {
                input_specification = new_spec;
            }

            void grasping_planner_t::link_query(query_t* new_query)
            {
                grasping_query = dynamic_cast<grasping_query_t*>(new_query);
                if(grasping_query == NULL)
                    PRX_FATAL_S("The grasping planner operate only over a grasping query as input!");
            }

            void grasping_planner_t::setup()
            {
                PRX_INFO_S("Setup grasping planner ...");
                std::vector<movable_body_plant_t* > objects;
                manipulation_model->get_objects(objects);

                foreach(movable_body_plant_t* object, objects)
                {
                    std::string object_type =  object->get_object_type();
                    for (std::vector<std::pair<std::string,std::string> >::iterator iter = data_folders.begin(); iter != data_folders.end(); ++iter)
                    {
                        std::string data_folder = iter->second;
                        std::string hash_string = object_type+iter->first;
                        std::string dir = data_folder + object_type + ".yaml"; //the yaml file with the list of configurations
                        if(grasps[hash_string].size() == 0 && boost::filesystem::exists( dir ))
                        {
                            YAML::Node doc = YAML::LoadFile(dir);
                            for(unsigned i=0;i<doc.size();i++)
                            {
                                config_t config;
                                std::vector<double> vec;
                                doc[i]["relative_config"] >> vec;
                                config.set_position(vec[0],vec[1],vec[2]);
                                config.set_xyzw_orientation(vec[3],vec[4],vec[5],vec[6]);
                                int release_mode = 1;
                                if(doc[i]["release_mode"])
                                    release_mode = doc[i]["release_mode"].as<int>();
                                int grasp_mode = doc[i]["grasp_mode"].as<int>();
                                grasps[ hash_string ].push_back(grasp_entry_t(config,release_mode,grasp_mode));
                            }
                            PRX_DEBUG_COLOR("Number of Grasps for "<<hash_string<<": "<<grasps[ hash_string ].size(), PRX_TEXT_BLUE)
                        }
                    }
                }
            }

            bool grasping_planner_t::succeeded() const
            {
                return true;
            }

            bool grasping_planner_t::execute()
            {
                
                //Do we need execute??? 
                return true;
            }            

            void grasping_planner_t::resolve_query()
            {
                std::vector<grasp_entry_t>* type_grasps = &grasps[grasping_query->object->get_object_type()+manipulation_model->get_current_manipulation_info()->full_arm_context_name];
                evaluate_the_query(type_grasps);
            }

            void grasping_planner_t::evaluate_the_query(std::vector<grasp_entry_t>* type_grasps)
            {
                manipulation_context_info_t* manipulator_info = manipulation_model->get_current_manipulation_info();

                if(original_object_state == NULL)
                    original_object_state = grasping_query->object->get_state_space()->alloc_point();
                else
                    grasping_query->object->get_state_space()->copy_to_point(original_object_state);

                config_t object_pose;
                grasping_query->get_object_configuration(object_pose);

                config_t ee_local_config;
                manipulation_model->get_end_effector_local_config(ee_local_config);

                config_t grasping_config;
                config_t retracted_config;
                bool done_computing = false;
                grasping_query->found_grasp = false;
                plan_t tmp_plan;
                tmp_plan.link_control_space(manipulator_info->full_arm_control_space);
                grasping_query->object->get_state_space()->copy_from_point(grasping_query->object_initial_state);
                for(unsigned i=0; i<type_grasps->size() && !done_computing; ++i)
                {
                    
                    grasping_config = ee_local_config;
                    if(grasping_query->grasping_index != -1)
                    {
                        PRX_DEBUG_COLOR("Grasping TP :: Set Index "<<grasping_query->grasping_index,PRX_TEXT_MAGENTA)
                        grasping_config.relative_to_global(type_grasps->at(grasping_query->grasping_index).relative_config);
                        grasping_query->grasping_mode = type_grasps->at(grasping_query->grasping_index).grasp_mode;
                        grasping_query->release_mode = type_grasps->at(grasping_query->grasping_index).release_mode;
                        done_computing = true;
                    }
                    else 
                    {
                        PRX_DEBUG_COLOR("Grasping TP :: Index "<<i,PRX_TEXT_MAGENTA)
                        grasping_config.relative_to_global(type_grasps->at(i).relative_config);
                        grasping_query->grasping_mode = type_grasps->at(i).grasp_mode;
                        grasping_query->release_mode = type_grasps->at(i).release_mode;
                        grasping_query->grasping_index = i;
                    }                    
                    grasping_config.relative_to_global( object_pose );

                    int first_mode = manipulation_model->get_current_grasping_mode();

                    if(first_mode!=grasping_query->release_mode)
                    {
                        manipulation_model->engage_grasp(tmp_plan, grasping_query->release_mode, false);
                    }



                    //using the releasing state as tmp variable in order to get the initial state of the manipulator.
                    manipulator_info->full_arm_state_space->copy_to_point(grasping_query->releasing_state);
                    PRX_DEBUG_COLOR("Grasping TP :: Seed state: " << manipulator_info->full_arm_state_space->print_point(grasping_query->releasing_state,5), PRX_TEXT_CYAN);

                    bool successIK = false;
                    if(manipulation_model->IK(grasping_query->releasing_state, grasping_query->releasing_state, grasping_config, true))  
                    {                   
                        if(validity_checker->is_valid(grasping_query->releasing_state))
                        {
                            successIK = true;
                        }
                        else
                        {
                            grasping_query->reason_for_failure="Initial IK invalid";
                            // PRX_INFO_S("Ik success, but invalid");
                        }
                    }
                    else
                    {
                        // PRX_INFO_S("IK failed");
                        grasping_query->reason_for_failure="Initial IK failed";
                    }
                    

                    PRX_DEBUG_COLOR("Grasping TP :: Releasing state (" << successIK << ") " << manipulator_info->full_arm_state_space->print_point(grasping_query->releasing_state,5), PRX_TEXT_CYAN);
                    //Checks if the releasing state is collision free
                    if(successIK)
                    {
                        PRX_DEBUG_COLOR("-Grasping TP :: Releasing state is valid " << manipulator_info->full_arm_state_space->print_point(grasping_query->releasing_state, 5),PRX_TEXT_MAGENTA);
                        PRX_DEBUG_COLOR("-Object: " << manipulation_model->get_active_space()->print_memory(5),PRX_TEXT_BLUE);
                        retracted_config = grasping_query->retraction_config;
                        // retracted_config = retraction_config;
                        retracted_config.relative_to_global( grasping_config );

                        //Compute the IK steering for the retraction plan
                        grasping_query->retracting_plan.clear();
                        if(manipulation_model->IK_steering(grasping_query->retracting_plan, grasping_query->retracted_open_state, grasping_query->releasing_state, retracted_config))
                        {
                            PRX_DEBUG_COLOR("--Grasping TP :: IK steering for retraction, retracted_open_state:" << manipulator_info->full_arm_state_space->print_point(grasping_query->retracted_open_state, 5),PRX_TEXT_MAGENTA)
                            PRX_DEBUG_COLOR("--Object: " << manipulation_model->get_active_space()->print_memory(5),PRX_TEXT_BLUE);
                            //Checks if the retracted state is collision free before we check for the entire path.
                            if(validity_checker->is_valid(grasping_query->retracted_open_state))
                            {
                                PRX_DEBUG_COLOR("---Grasping TP :: Retracted state is valid",PRX_TEXT_MAGENTA)
                                //Propagates the plan from the IK steering and checks if the whole path is collision free.
                                grasping_query->tmp_path.clear();
                                manipulation_model->propagate_plan(grasping_query->releasing_state, grasping_query->retracting_plan, grasping_query->tmp_path);
                                if(validity_checker->is_valid(grasping_query->tmp_path))
                                {
                                    PRX_DEBUG_COLOR("----Grasping TP :: Retraction trajectory is valid , end of path: " << manipulator_info->full_arm_state_space->print_point(grasping_query->tmp_path.back(),5) ,PRX_TEXT_MAGENTA)
                                    PRX_DEBUG_COLOR("----Object: " << manipulation_model->get_active_space()->print_memory(5),PRX_TEXT_BLUE);
                                    //The manipulator should be at the retracted_open_state right now. 
                                    grasping_query->reaching_plan.clear();
                                    if(manipulation_model->IK_steering(grasping_query->reaching_plan, grasping_query->tmp_state, grasping_query->retracted_open_state, grasping_query->releasing_state, grasping_config ))
                                    {
                                        PRX_DEBUG_COLOR("-----Grasping TP :: IK steer toward object worked (" << grasping_query->reaching_plan.size() << "), the new realizing state  : " <<  manipulator_info->full_arm_state_space->print_point(grasping_query->tmp_state,5),PRX_TEXT_MAGENTA)
                                        PRX_DEBUG_COLOR("-----Object: " << manipulation_model->get_active_space()->print_memory(5),PRX_TEXT_BLUE);                                        
                                        //Propagates the plan from the IK steering and checks if the whole path is collision free.
                                        grasping_query->tmp_path.clear();
                                        manipulation_model->propagate_plan(grasping_query->retracted_open_state, grasping_query->reaching_plan, grasping_query->tmp_path);
                                        if(validity_checker->is_valid(grasping_query->tmp_path))
                                        {
                                            PRX_DEBUG_COLOR("------Grasping TP :: Reaching plan is collision free , end of reaching path: " << manipulator_info->full_arm_state_space->print_point(grasping_query->tmp_path.back(),5),PRX_TEXT_MAGENTA)
                                            PRX_DEBUG_COLOR("------Object: " << manipulation_model->get_active_space()->print_memory(5),PRX_TEXT_BLUE);
                                            //Everything from the open mode are valid. Now we have to check and the grasping states. 
                                            //Setting up the manipulator and the object that the pose. 
                                            manipulator_info->full_arm_state_space->copy_from_point(grasping_query->releasing_state);
                                            tmp_plan.clear();
                                            int last_mode = manipulation_model->get_current_grasping_mode();
                                            PRX_DEBUG_COLOR("------Current mode: " << last_mode,PRX_TEXT_BLUE);
                                            manipulation_model->engage_grasp(tmp_plan, grasping_query->grasping_mode, true);
                                            manipulator_info->full_arm_state_space->copy_to_point(grasping_query->grasping_state);
                                            PRX_DEBUG_COLOR("-------Grasping TP :: Got the grasped state: " << manipulator_info->full_arm_state_space->print_point(grasping_query->grasping_state,5),PRX_TEXT_MAGENTA)
                                            grasping_query->tmp_path.clear();
                                            manipulation_model->propagate_plan(grasping_query->grasping_state, grasping_query->retracting_plan, grasping_query->tmp_path);
                                            if(validity_checker->is_valid(grasping_query->tmp_path))
                                            {
                                                PRX_DEBUG_COLOR("-------Grasping TP :: retract with object valid and back of the path is: " << manipulator_info->full_arm_state_space->print_point(grasping_query->tmp_path.back(),5),PRX_TEXT_MAGENTA)
                                                PRX_DEBUG_COLOR("-------Object: " << manipulation_model->get_active_space()->print_memory(5),PRX_TEXT_BLUE);
                                                manipulator_info->full_arm_state_space->copy_point(grasping_query->retracted_close_state, grasping_query->tmp_path.back());
                                                PRX_DEBUG_COLOR("-------Grasping TP :: the retracted closed state is : " << manipulator_info->full_arm_state_space->print_point(grasping_query->retracted_close_state,5),PRX_TEXT_MAGENTA)                                                
                                                grasping_query->tmp_path.clear();
                                                manipulation_model->propagate_plan(grasping_query->retracted_close_state, grasping_query->reaching_plan, grasping_query->tmp_path);
                                                if(validity_checker->is_valid(grasping_query->tmp_path))
                                                {
                                                    PRX_DEBUG_COLOR("--------Grasping TP :: reach with object valid  path_end: " << manipulator_info->full_arm_state_space->print_point(grasping_query->tmp_path.back(),5),PRX_TEXT_MAGENTA)
                                                    PRX_DEBUG_COLOR("--------Object: " << manipulation_model->get_active_space()->print_memory(5),PRX_TEXT_BLUE);
                                                    
                                                    grasping_query->found_grasp = true;
                                                    done_computing = true;
                                                }
                                                else
                                                {
                                                    grasping_query->reason_for_failure="Reach with object invalid";
                                                }
                                            }
                                            else
                                            {
                                                grasping_query->reason_for_failure="Retract with object invalid";                        
                                            }                                            
                                            manipulation_model->engage_grasp(tmp_plan, last_mode, false); 
                                            manipulator_info->full_arm_state_space->copy_from_point(grasping_query->releasing_state);
                                            PRX_DEBUG_COLOR("Previous grasping mode: "<<last_mode,PRX_TEXT_BLUE);                                           
                                        }
                                        else
                                        {
                                            grasping_query->reason_for_failure="Retract with object invalid";
                                        }
                                    }
                                    else
                                    {
                                        grasping_query->reason_for_failure="Reaching IK steering failed";
                                    }
                                }
                                else
                                {
                                    grasping_query->reason_for_failure="Retracting path invalid";
                                }
                            }
                            else
                            {
                                grasping_query->reason_for_failure="Retracted state invalid";
                            }
                        }
                        else
                        {
                            grasping_query->reason_for_failure="Retracting IK steering failed";
                        }
                    }

                    if(!grasping_query->found_grasp)
                    {
                        grasping_query->grasping_index = -1;
                    }
                    if(first_mode!=grasping_query->release_mode)
                    {
                        manipulation_model->engage_grasp(tmp_plan, first_mode, false);
                    }
                }

                if(!grasping_query->found_grasp)
                {
                    grasping_query->grasping_index = -1;
                    PRX_DEBUG_COLOR("----Grasping TP :: Grasping Failed : "<<grasping_query->reason_for_failure,PRX_TEXT_RED)
                }
                else
                    PRX_DEBUG_COLOR("Got solution: index:" << grasping_query->grasping_index << "   mode:" << grasping_query->grasping_mode, PRX_TEXT_GREEN);

                grasping_query->object->get_state_space()->copy_from_point(original_object_state);
                // PRX_PRINT(grasping_query->object->get_state_space()->print_memory(3),PRX_TEXT_MAGENTA);
                // PRX_PRINT(manipulation_model->get_full_state_space()->print_memory(3),PRX_TEXT_MAGENTA);
            }

            int grasping_planner_t::nr_grasps( std::string context_name,movable_body_plant_t* object)
            {
                return grasps[object->get_object_type()+context_name].size();
            }

            void grasping_planner_t::set_param(const std::string& path, const std::string& parameter_name, const boost::any& value)
            {
                std::string name;
                std::string subpath;
                boost::tie(name, subpath) = split_path(path);

                if( !name.empty() )
                    PRX_FATAL_S("Error! You are trying to set a parameter to a planner under grasping planner, where it is empty.");
                    
                set_param(parameter_name, value);
                
            }

            void grasping_planner_t::set_param(const std::string& parameter_name, const boost::any& value)
            {
                planner_t::set_param(parameter_name, value);
            }
        }
    }
}
