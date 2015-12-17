
#pragma once

#ifndef PRX_BAXTER_HPP
#define	PRX_BAXTER_HPP

#include "prx/utilities/definitions/defs.hpp"

#include "simulation/systems/plants/manipulator.hpp"

namespace prx
{
    namespace packages
    {
        namespace manipulation
        {

            /**
             * 
             */
            class baxter_t : public manipulator_t
            {

              public:
                baxter_t();

                virtual ~baxter_t(){ }

                /** @copydoc plant_t::init(const util::parameter_reader_t *, const util::parameter_reader_t*) */
                virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

                virtual void propagate(const double simulation_step);

              protected:

                virtual void create_spaces();
                //bool isOpen = false;
                int prev; 
                bool execute_on_robot;
                ros::Publisher pub_left ;
                ros::NodeHandle nh;
                ros::Publisher pub_ee; 
            };
        }

    }
}

#endif