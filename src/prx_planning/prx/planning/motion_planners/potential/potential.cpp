/**
 * @file prm.cpp
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

#include "prx/planning/motion_planners/potential/potential.hpp"

#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/goals/goal.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"
#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"
#include "prx/planning/problem_specifications/motion_planning_specification.hpp"
#include "prx/planning/communication/visualization_comm.hpp"
#include "prx/simulation/collision_checking/collision_list.hpp"
#include "prx/utilities/statistics/statistics.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"

#include <pluginlib/class_list_macros.h>
#include <boost/assign/list_of.hpp>
#include <boost/graph/subgraph.hpp>
#include <boost/range/adaptor/map.hpp>
#include <boost/graph/connected_components.hpp>


PLUGINLIB_EXPORT_CLASS(prx::plan::potential_t, prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;

    namespace plan
    {

        potential_t::potential_t()
        {
        }

        potential_t::~potential_t()
        {
        }

        void potential_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {
            motion_planner_t::init(reader, template_reader);
            statistics = new statistics_t();
            time_step = simulation::simulation_step;

        }

        void potential_t::reset()
        {
        }

        void potential_t::link_specification(specification_t* new_spec)
        {
            motion_planner_t::link_specification(new_spec);

            //collision_checker gives access to closest point queries
            if (dynamic_cast<pqp_collision_checker_t*>(validity_checker->get_collision_checker()) != NULL)
                collision_checker = dynamic_cast<pqp_collision_checker_t*>(validity_checker->get_collision_checker()) ;
            else
                PRX_FATAL_S("Needs PQP Collision Checker");


            collision_list_t* collision_list = validity_checker->get_collision_checker()->get_collision_list();

            //These pairs are the bodies that are considered for collision checking
            //These names have to be used to ask the collision checker for additional information
            foreach(collision_pair_t pair, collision_list->get_body_pairs())
            {
                PRX_DEBUG_COLOR("Collision pairs:: ( "<< pair.first <<", "<< pair.second<<" )", PRX_TEXT_BROWN);
            }

            //The manipulation world model: provides access to FK interfaces
            world_model = dynamic_cast< prx::packages::manipulation::manipulation_world_model_t* >(validity_checker->get_world_model());

            //The path to the baxter will depend upon the system tree. This might be different in another setup. Either the path has to be taken as input or this should be set to where the manipulator can be found.
            if(world_model !=NULL)
                PRX_INFO_S("World model Loaded");
            else
                PRX_FATAL_S("World model not loaded!");


        }

        void potential_t::setup()
        {
            temp_plan.link_control_space(control_space);
            temp_plan.link_state_space(state_space);
            output_plan.link_control_space(control_space);
            output_plan.link_state_space(state_space);
            temp_trajectory.link_space(state_space);

            start_state = state_space->alloc_point();
            current_state = state_space->alloc_point();
        }

        bool potential_t::execute()
        {
            throw stopping_criteria_t::stopping_criteria_satisfied(" Stopping criteria is satisfied ");
        }

        void potential_t::step()
        {
            double  etta[7]     = {1, 1, 1, 1, 1, 1, 1};
            double  zeta[7]     = {1, 1, 1, 1, 1, 1, 1};
            double  roh_0       = 1.0;
            double  trans_d     = 1.0;
            char *links[7] = {"left_arm_mount","left_upper_shoulder", "left_lower_shoulder","left_upper_elbow","left_lower_elbow", "left_upper_forearm", "left_lower_forearm"}

            //>>>>>>>>>>>>>>>>>>CLOSEST POINTS BETWEEN TWO BODIES
            //Call the collision checker with the two points and the valid names of the two bodies
            //Dist stores the distance between the points
            //The two points are updated with the coordinates in the GLOBAL coordinate frame, that correspond to the closest points on the two bodies
            //dist stores the distance between the two volumes
            //Call the function with a link from the robot and the obstacle to get the distance of the link from the obstacle. This gives the magnitude of the vector
            std::vector<double> pt1, pt2; //The two points which would be populated by the closest points on the two bodies
            double dist = collision_checker->closest_points(pt1, pt2, "simulator/baxter/left_lower_elbow", "simulator/obstacles/ball/ball"); //The only obstacle in the given environment that has to be considered is the ball
            PRX_DEBUG_COLOR("\n-----------------------", PRX_TEXT_GREEN);
            PRX_DEBUG_COLOR("PT 1 : "<<pt1[0]<<", "<<pt1[1]<<", "<<pt1[2], PRX_TEXT_BLUE);
            PRX_DEBUG_COLOR("PT 2 : "<<pt2[0]<<", "<<pt2[1]<<", "<<pt2[2], PRX_TEXT_BLUE);
            PRX_DEBUG_COLOR("Distance : "<<dist, PRX_TEXT_RED);
            PRX_DEBUG_COLOR("-----------------------\n", PRX_TEXT_GREEN);

            //>>>>>>>>>>>>>>>>>>FK OF LINKS
            //The world model provides the FK interface for the current active manipulator. In this case there is only one manipulator, the Baxter.
            //The FK function expects the link name(not the full path) and returns the SE3 configuration of the link. 
            util::config_t curr_config;
            world_model->FK(curr_config, "left_lower_elbow");
            PRX_WARN_S(curr_config);


            //>>>>>>>>>>>>>>>>>>DISTANCE CALCULATIONS
            double distance1 = metric->distance_function(current_state, goal_state);
            //This function returns the distance between two points in the state_space of the motion planner where the metric is defined.
            


            // std::cout << state_space->print_point(current_state, 3) << std::endl;
            // std::cout << state_space->print_point(goal_state, 3) << std::endl;
            // std::cout << *state_space << std::endl;


            //>>>>>>>>>>>>>>>>>>FORCE CALCULATIONS

            /* ATTRACTIVE FORCE */

            //Finding configuration of goal_state 
            util::config_t goal_config;
            state_space->copy_from_point(goal_state);
            world_model->FK(goal_config, "left_lower_elbow");
            PRX_WARN_S(goal_config);
            //Change it back cause...idunno
            state_space->copy_from_point(current_state);

            double goal_x;
            double goal_y;
            double goal_z;
            double curr_x;
            double curr_y;
            double curr_z;
            
            goal_config.get_position(goal_x, goal_y, goal_z);
            curr_config.get_position(curr_x, curr_y, curr_z);
                    std::cout << "Curr origin: " << curr_x << curr_y << curr_z << std::endl;
                    std::cout << "Goal origin: " << goal_x << goal_y << goal_z << std::endl;

            //Vectors that represent the point of the current origin and goal origin
            vector_t curr_point = vector_t(curr_x, curr_y, curr_z);
            vector_t goal_point = vector_t(goal_x, goal_y, goal_z);
            
            //Vector representing the distance between curr and goal
            vector_t goal_distance = curr_point - goal_point;
                    std::cout << "Goal vector: " << goal_distance << std::endl;
                    std::cout << "Goal norm:   " << goal_distance.norm() << std::endl;


            if(goal_distance.norm() <= trans_d){
                vector_t Force_attr = (goal_distance) * (-1)*(zeta[0]);
            }else{
                vector_t Force_attr = (goal_distance / goal_distance.norm() ) * trans_d * zeta[0] * (-1);
            }


    
            /* REPULSIVE FORCE */
            vector_t point1 = vector_t(pt1[0], pt1[1], pt1[2]);
            vector_t point2 = vector_t(pt2[0], pt2[1], pt2[2]);
            double obst_distance = point1.distance(point2);
            vector_t obst_vector = point1-point2;
                    // std::cout << obst_distance << std::endl;
                    // std::cout << obst_vector.norm() <<std::endl;
            if(obst_vector.norm() < roh_0){
                double prefix = etta[0] * ((1/obst_vector.norm()) - (1/roh_0)) * (1 / (obst_vector.norm())*obst_vector.norm());
                vector_t Force_rep = (obst_vector/obst_vector.norm()) * prefix;
            }else{
                vector_t Force_rep = vector_t(0, 0, 0);
            }

            //>>>>>>>>>>>>>>>>>>TORQUE CALCULATIONS

                /* ATTRACTIVE FORCE JACOBIAN */

                /* REPULSIVE FORCE JACOBIAN */


            //>>>>>>>>>>>>>>>>>>CREATING PLANS FROM INDIVIDUAL CONTROLS
            //The controls are angular velocities on the joints.
            //Once you have the control c, for time_step(=simulation::simulation_step) 
            //Create a plan with only that control
            // temp_plan.clear();
            // temp_plan.copy_onto_back(c,time_step);

            //>>>>>>>>>>>>>>>>>>PROPAGATING PLANS TO FIND THE REACHED STATE
            //If you have a plan and want to apply the control sequence to a state, the propagate returns the trajectory this would generate
            // local_planner->propagate(current_state, temp_plan, temp_trajectory);


            //>>>>>>>>>>>>>>>>>>CHECKING IF THE PLAN AND PATH ARE VALID
            //The generated plan and path can be checked if valid
            // if( temp_plan.size() != 0 && validity_checker->is_valid(temp_trajectory) )
            // {   
                //Plan is valid and trajectory is collision free
                //The last state in the trajectory gives the state the robot will reach if acted on by the controls in the temp_plan.
                //Update the variable current_state with this.
                //Append temp_plan to output_plan
            // }


            //General approach
            /*

                >>Given the latest state of the motion planner, calculate the forces applied to the robot at this state. This includes computing both 
                the point on the robot where these forces are applied, their magnitude and direction.    
                >>Compute the control that arise due to these forces. This involves the computation of a Jacobian matrix that 
                depends on the point that these forces are applied.
                >>Propagate the state of the motion planner given the computed control. This involves calling functionality from the local_planner.
                >>Append the control to the output plan and the state to the output path.              
               
            */

            //NOTE: At the end of the step() the variable current_state should be updated with the next state that the motion planner shall continue from.




        }

        bool potential_t::succeeded() const
        {
            PRX_INFO_S("potential_t");
            if( input_specification->get_stopping_criterion()->satisfied() )
                return true;
            return false;
        }

        void potential_t::resolve_query()
        {
            PRX_ASSERT(input_specification != NULL);
            unsigned goals_size;
            sys_clock_t clck;
            std::vector<space_point_t*> goals = input_query->get_goal()->get_goal_points(goals_size);
            if(goals.size()>0)
            {
                goal_state = goals[0];
                PRX_INFO_S("Goals have been linked. Taking the first one : "<<state_space->print_point(goal_state, 4));

            }
            else
            {
                PRX_WARN_S("No Goals linked.");
            }



            try
            {
                do
                {
                    // This enforces that the current state is valid 
                    // This updates the internal configurations of the collision checker to agree with the current_state 
                    // This updates the internal state of the world_model simulator (does an internal copy_from_point)
                    // The variable current_state should be correctly updated in the step() function
                    if(!validity_checker->is_valid(current_state))
                    {
                        PRX_FATAL_S("Stepping from an invalid state in the motion planner.");
                    }
                    step();
                }
                while( !input_specification->get_stopping_criterion()->satisfied() );
                //You need to decide when it actually stops planning. Check the different stopping criteria to stop this once the goal is reached.
            }
            catch( stopping_criteria_t::stopping_criteria_satisfied e )
            {
                double timeout = clck.measure();
                PRX_INFO_S(e.what()<<" at time: "<<timeout);
                input_query->plan = output_plan;
            }
            

        }

        void potential_t::update_vis_info() const
        {

        }

        

        bool potential_t::is_valid_trajectory(const sim::trajectory_t& path)
        {
            return validity_checker->is_valid(path);
        }

        const statistics_t* potential_t::get_statistics()
        {
            statistics->time = 0;
            statistics->steps = 0;
            return statistics;
        }


    }
}
