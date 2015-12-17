/**
 * @file baxter_sim_application.cpp
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


#include "simulation/applications/baxter_sim_application.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/definitions/random.hpp"

#include "prx_simulation/plan_msg.h"
#include "prx_simulation/control_msg.h"
#include <pluginlib/class_list_macros.h>

#include "prx/simulation/communication/sim_base_communication.hpp"
#include "prx/simulation/communication/planning_comm.hpp"
#include "prx/simulation/communication/simulation_comm.hpp"
#include "prx/simulation/communication/visualization_comm.hpp"
#include "simulation/simulators/manipulation_simulator.hpp"
#include "simulation/systems/plants/movable_body_plant.hpp"

#include "image_listener/Num.h"

#include "prx/utilities/math/configurations/config.hpp"


#define MAX_MANIPULATIONS 10

PLUGINLIB_EXPORT_CLASS( prx::packages::manipulation::baxter_sim_application_t, prx::sim::application_t)

namespace prx
{
    using namespace util;
    using namespace sim::comm;
    namespace packages
    {        
        namespace manipulation
        {

            baxter_sim_application_t::baxter_sim_application_t() { }

            baxter_sim_application_t::~baxter_sim_application_t() { }

            void baxter_sim_application_t::init(const parameter_reader_t * const reader)
            {
                prx::sim::empty_application_t::init(reader);
                PRX_DEBUG_COLOR("Initialized", PRX_TEXT_RED);
                received_plan_sub = node.subscribe("/ready_to_plan", 1, &baxter_sim_application_t::planning_ready_callback, this);
                planning_ready_sub = node.subscribe("/planning/plans", 1, &baxter_sim_application_t::received_plan_callback, this);
               
                detected_objects_sub = node.subscribe("/detected_objects", 1, &baxter_sim_application_t::detected_objects_callback, this);

                manipulation_request_pub = node.advertise<std_msgs::String> ("/manipulation_request", 1);
                simulator_state = simulator->get_state_space()->alloc_point();
                PRX_ERROR_S("\n\n\n SIMULATOR AT START::: "<<simulator->get_state_space()->print_point(simulator_state,4)<<"\n\n\n");
                simulator_running = true;
                counter = 0;

            }


            void baxter_sim_application_t::detected_objects_callback(const image_listener::Num& msg)
            {
                PRX_ERROR_S("Placing detected objects in simulation.");

                std::string blue = ("blue");
                std::string yellow = ("yellow");
                std::string green = ("green");
                std::string red = ("red");


                std::vector<movable_body_plant_t* > objects;
                manipulation_simulator_t* manip_sim = dynamic_cast<manipulation_simulator_t* >(simulator);
                
                manip_sim->get_movable_objects(objects);

               int OBJECT_INDEX;

                /* yellow */
                OBJECT_INDEX = 0;
                const space_t* object_space = objects[OBJECT_INDEX]->get_state_space();
                util::space_point_t * object = object_space->alloc_point();
                object->at(0) = msg.object1[0];
                object->at(1) = msg.object1[1];
                object->at(2) = msg.object1[2];
                object->at(3) = msg.object1[3];
                object->at(4) = msg.object1[4];
                object->at(5) = msg.object1[5];
                object->at(6) = msg.object1[6];
                object_space->copy_from_point(object);

                /* red */
                OBJECT_INDEX = 1;
                object_space = objects[OBJECT_INDEX]->get_state_space();
                object = object_space->alloc_point();
                object->at(0) = msg.object2[0];
                object->at(1) = msg.object2[1];
                object->at(2) = msg.object2[2];
                object->at(3) = msg.object2[3];
                object->at(4) = msg.object2[4];
                object->at(5) = msg.object2[5];
                object->at(6) = msg.object2[6];
                object_space->copy_from_point(object);
            

                /* red */
                OBJECT_INDEX = 2;
                object_space = objects[OBJECT_INDEX]->get_state_space();
                object = object_space->alloc_point();
                object->at(0) = msg.object3[0];
                object->at(1) = msg.object3[1];
                object->at(2) = msg.object3[2];
                object->at(3) = msg.object3[3];
                object->at(4) = msg.object3[4];
                object->at(5) = msg.object3[5];
                object->at(6) = msg.object3[6];                
                object_space->copy_from_point(object);

                /* green */
                OBJECT_INDEX = 3;
                object_space = objects[OBJECT_INDEX]->get_state_space();
                object = object_space->alloc_point();
                object->at(0) = msg.object4[0];
                object->at(1) = msg.object4[1];
                object->at(2) = msg.object4[2];
                object->at(3) = msg.object4[3];
                object->at(4) = msg.object4[4];
                object->at(5) = msg.object4[5];
                object->at(6) = msg.object4[6];
                object_space->copy_from_point(object);


                /* yellow */
                OBJECT_INDEX  = 4;
                object_space = objects[OBJECT_INDEX]->get_state_space();
                object = object_space->alloc_point();
                object->at(0) = msg.object5[0];
                object->at(1) = msg.object5[1];
                object->at(2) = msg.object5[2];
                object->at(3) = msg.object5[3];
                object->at(4) = msg.object5[4];
                object->at(5) = msg.object5[5];
                object->at(6) = msg.object5[6];
                object_space->copy_from_point(object);
                

                /* green */
                OBJECT_INDEX  = 5;
                object_space = objects[OBJECT_INDEX]->get_state_space();
                object = object_space->alloc_point();
                object->at(0) = msg.object6[0];
                object->at(1) = msg.object6[1];
                object->at(2) = msg.object6[2];
                object->at(3) = msg.object6[3];
                object->at(4) = msg.object6[4];
                object->at(5) = msg.object6[5];
                object->at(6) = msg.object6[6];
                object_space->copy_from_point(object);
                

                /* blue */
                OBJECT_INDEX  = 6;
                object_space = objects[OBJECT_INDEX]->get_state_space();
                object = object_space->alloc_point();
                object->at(0) = msg.object7[0];
                object->at(1) = msg.object7[1];
                object->at(2) = msg.object7[2];
                object->at(3) = msg.object7[3];
                object->at(4) = msg.object7[4];
                object->at(5) = msg.object7[5];
                object->at(6) = msg.object7[6];
                object_space->copy_from_point(object);
                

                /* blue */
                OBJECT_INDEX  = 7;
                object_space = objects[OBJECT_INDEX]->get_state_space();
                object = object_space->alloc_point();
                object->at(0) = msg.object8[0];
                object->at(1) = msg.object8[1];
                object->at(2) = msg.object8[2];
                object->at(3) = msg.object8[3];
                object->at(4) = msg.object8[4];
                object->at(5) = msg.object8[5];
                object->at(6) = msg.object8[6];
                object_space->copy_from_point(object);
            

                simulator_state = manip_sim->get_state_space()->alloc_point();
                simulator->push_state(simulator_state);//By default the objects return to their original position
                //PRX_ERROR_S("SIMULATOR STATE SPACE CHANGED TO::: "<<simulator->get_state_space()->print_memory(4));



            };



            void baxter_sim_application_t::planning_ready_callback(const std_msgs::String& msg)
            {
                //randomize_positions();
                std_msgs::String send_msg;
                std::stringstream ss;
                ss<<"start_manipulate";
                send_msg.data = ss.str();
                sleep(2);
                manipulation_request_pub.publish(send_msg);
            }

            void baxter_sim_application_t::received_plan_callback(const prx_simulation::plan_msg& msg)
            {
                double duration = 0;
                foreach(const prx_simulation::control_msg& control_msg, msg.plan)
                {
                    duration += control_msg.duration;
                }
                PRX_ERROR_S("\n\n\nSleeping for : \n\n\n"<<duration*1.5<<" seconds");
                sleep(duration*1.5);
                if(counter++>MAX_MANIPULATIONS)
                {
                    PRX_ERROR_S("End of experiment!");
                    return;
                }
                //randomize_positions();
                std_msgs::String send_msg;
                std::stringstream ss;
                ss<<"manipulate";
                send_msg.data = ss.str();
                manipulation_request_pub.publish(send_msg);
            }

            void baxter_sim_application_t::randomize_positions()
            {
                PRX_ERROR_S("\n\n\n SIMULATOR STATE SPACE ::: "<<simulator->get_state_space()->print_memory(4));
                PRX_ERROR_S(" SIMULATOR STATE SPACE SET TO::: "<<simulator->get_state_space()->print_point(simulator_state,4));
                std::vector<movable_body_plant_t* > objects;
                manipulation_simulator_t* manip_sim = dynamic_cast<manipulation_simulator_t* >(simulator);
                
                // AI_PRX_TODO : Randomize the position of the objects.
                // First obtain the list of movable objects from the manipulator simulator.
                // Then get the state space of each object
                // Then update the state space of each with a point/vector that you change here randomly within the following range
                // RANGE OF POSITIONS:
                // X belongs to [0.75, 0.95]
                // Y belongs to [0.40, 0.60]
                // Z = 0.87
                // Q = <0 0 0 1>
                // After you update every object, 
                // Get the full state space and copy to simulator_state

                /* Obtain the list of moveable objects from the manipulator simulator. */
                manip_sim->get_movable_objects(objects);

                /* Get the state space of (each) object. */ 
                int OBJECT_INDEX;

                for(OBJECT_INDEX = 0; OBJECT_INDEX < objects.size(); OBJECT_INDEX++){

                    const space_t* object_space = objects[OBJECT_INDEX]->get_state_space();
                    util::space_point_t * object = object_space->alloc_point();

                    /* Update the state space of each with a point/vector that you change randomly within range */
                    double x = uniform_random(0.75, 0.95);
                    double y = uniform_random(0.40, 0.60);
                    double z = 0.87;

                    object->at(0) = x;
                    object->at(1) = y;
                    object->at(2) = z;
                    object_space->copy_from_point(object);
                }

                /* Get full state space and copy to simulator_state */
                simulator_state = manip_sim->get_state_space()->alloc_point();

                /*
                    When you are dealing with the manipulation simulator, there are additional internal state variables the simulator uses for grasping properly. 
                    These might not be set correctly if you only do copy_from_point, since that can update only part of the state space while the rest is unchanged. 
                    push_state is done by the application to both copy the state values and also update the manipulation simulator information. 
                    You do not need to do this unless you are at the application level. 
                */
                simulator->push_state(simulator_state);//By default the objects return to their original position
                PRX_ERROR_S("SIMULATOR STATE SPACE CHANGED TO::: "<<simulator->get_state_space()->print_memory(4));
            }

            void baxter_sim_application_t::frame(const ros::TimerEvent& event)
            {
                handle_key();

                if( simulator_mode == 1 )
                {
                    if( simulator_counter > 0 )
                    {
                        simulator_running = true;
                        simulator_counter--;
                        loop_timer.reset();
                    }
                    else
                        simulator_running = false;
                }
                if( loop_timer.measure() > 1.0 )
                    loop_timer.reset();
                if( simulator_running )
                {

                    if( replays_states )
                    {
                        if( loop_counter > (int)replay_simulation_states.size() )
                            loop_counter = 0;
                        simulation_state_space->copy_from_point(replay_simulation_states[loop_counter]);
                        simulation_state_space->copy_to_point(simulation_state);
                    }
                    else
                    {
                        simulator->push_control(simulation_control);
                        simulator->propagate_and_respond();
                    }
                    if( stores_states )
                        store_simulation_states.copy_onto_back(simulation_state);

                    if( screenshot_every_simstep )
                    {
                        ((visualization_comm_t*)vis_comm)->take_screenshot(0, 1);
                    }
                    loop_total += loop_timer.measure_reset();
                    loop_counter++;
                    loop_avg = loop_total / loop_counter;

                }

                if(visualization_counter++%visualization_multiple == 0)
                {
                    tf_broadcasting();
                }



            }
        }


    }
}
