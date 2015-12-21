/**
 * @file baxter.cpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Rahul Shome, Zakary Littlefield, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */
#include "simulation/systems/plants/baxter/baxter.hpp"

#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/definitions/random.hpp"
#include "prx/simulation/simulators/simulator.hpp"
#include "prx/simulation/collision_checking/collision_checker.hpp"
#include "simulation/systems/plants/manipulator.hpp"

#include <boost/tuple/tuple.hpp> // boost::tie
#include <boost/assign/list_of.hpp>
#include <boost/range/adaptor/map.hpp> //adaptors
#include <pluginlib/class_list_macros.h>
#include <urdf/model.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <sys/param.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <baxter_core_msgs/JointCommand.h>
#include <baxter_core_msgs/EndEffectorCommand.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::baxter_t, prx::sim::system_t)

namespace prx
{
    using namespace sim;
    using namespace util;
    namespace packages
    {
        namespace manipulation
        {
            baxter_t::baxter_t()
            {
                input_path = pracsys_path + "/prx_packages/manipulation/input/baxter/";
                execute_on_robot = false;
                prev = 1;
            }

            void baxter_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                manipulator_t::init(reader, template_reader);
                if( !parameters::has_attribute( "execute_on_robot", reader, template_reader ) )
                {
                    PRX_WARN_S("EXECUTE ON ROBOT SET TO false");
                }
                else
                {
                    execute_on_robot = parameters::get_attribute_as< bool >( "execute_on_robot", reader, template_reader, false);
                    PRX_WARN_S("EXECUTE ON ROBOT SET TO "<<execute_on_robot);
                }
                //The publisher can publish a joint command to the topic being listened to by the Baxter. 
                //If this code is run on the same ROS Master as the Baxter, the Baxter will respond to commands being published on this topic.
                pub_left = nh.advertise< baxter_core_msgs::JointCommand >("/robot/limb/left/joint_command",1);
                pub_ee = nh.advertise< baxter_core_msgs::JointCommand> ("/robot/end_effector/left_gripper/command",1);
            }

            void baxter_t::create_spaces()
            {
                state_space = new space_t("Baxter", state_memory);
                input_control_space = new space_t("Baxter", control_memory);
            }

            void baxter_t::propagate(const double simulation_step)
            {
                manipulator_t::propagate(simulation_step);
                if(execute_on_robot)//The execute_on_robot flag is set only in simulation and not in planning. During planning, the planner's internal simulator also moves the robot. However, we only want the real robot to execute the motions being carried out by the robot on the simulation side.
                {
                    baxter_core_msgs::JointCommand com_msg;
                    com_msg.mode = baxter_core_msgs::JointCommand::POSITION_MODE;

                   baxter_core_msgs::EndEffectorCommand ee_msg;


                    // Names of joints: 
                    // left_s0
                    // left_s1
                    // left_e0
                    // left_e1
                    // left_w0
                    // left_w1
                    // left_w2
                    // for all joints
                    // {
                    //     com_msg.names.push_back( name of joint );
                    //     com_msg.command.push_back( current angle in state space );
                    // }

                    state_t* joints = state_space->alloc_point();

                    if(joints->at(14) != prev){

                        if(joints->at(14) == 2){
                            ee_msg.id = 65664;
                            ee_msg.command = "CMD_GRIP";
                        }
                        else{
                            ee_msg.id = 65664;
                            ee_msg.command = "CMD_RELEASE";
                        }

                        pub_ee.publish(ee_msg);

                        prev = joints->at(14);
                    }

                    com_msg.names.push_back("left_s0");
                    com_msg.command.push_back(joints->at(0));

                    com_msg.names.push_back("left_s1");
                    com_msg.command.push_back(joints->at(1));
                    
                    com_msg.names.push_back("left_e0");
                    com_msg.command.push_back(joints->at(2));
                    
                    com_msg.names.push_back("left_e1");
                    com_msg.command.push_back(joints->at(3));
            
                    com_msg.names.push_back("left_w0");
                    com_msg.command.push_back(joints->at(4));
                    
                    com_msg.names.push_back("left_w1");
                    com_msg.command.push_back(joints->at(5));
                    
                    com_msg.names.push_back("left_w2");
                    com_msg.command.push_back(joints->at(6));

                    com_msg.names.push_back("left_gripper");
                    com_msg.command.push_back(joints->at(14));


                    pub_left.publish(com_msg);
                    // Can you also operate the grippers? Do you need a separate message? 
                    // The state of the robot wrt grippers is: 1 - open, 2 - closed
                    PRX_INFO_S(state_space->print_memory(3));
                }

            }
        }
    }
}





