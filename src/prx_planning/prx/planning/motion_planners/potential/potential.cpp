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
            collision_checker = validity_checker->get_collision_checker();

        }

        void potential_t::reset()
        {
        }

        void potential_t::link_specification(specification_t* new_spec)
        {
            motion_planner_t::link_specification(new_spec);
        }

        void potential_t::setup()
        {

        }

        bool potential_t::execute()
        {
            PRX_ASSERT(input_specification != NULL);

            do
            {
                // PRX_INFO_S("EXECUTE FROM MOTION PLANNER");
                step();
            }
            while( !input_specification->get_stopping_criterion()->satisfied() );
            
            return succeeded();
        }

        void potential_t::step()
        {
            //TODO
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
            //TODO
        }

        void potential_t::update_vis_info() const
        {

        }

        

        bool potential_t::is_valid_trajectory(const sim::trajectory_t& path)
        {
            return validity_checker->is_valid(path);
        }


    }
}
