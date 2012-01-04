/*
    Copyright 2012 Jay Graham

    Distributed under the MIT License (see accompanying file LICENSE_1_0_0.txt
    or http://www.opensource.org/licenses/mit-license.php)
*/

/**************************************************************************************************/

#ifndef CMDG_DE_SIM_STEP_H
#define CMDG_DE_SIM_STEP_H

/**************************************************************************************************/

#include <boost/mpl/inherit.hpp>
#include <boost/mpl/placeholders.hpp>
#include <boost/mpl/empty.hpp>
#include <boost/mpl/empty_base.hpp>
#include <boost/mpl/for_each.hpp>
#include <boost/mpl/reverse_fold.hpp>

#include <cmdg/de/clock.h>
#include <cmdg/de/clock_traits.h>

/**************************************************************************************************/

namespace cmdg {
namespace de {

/**************************************************************************************************/

struct null_step_sim_impl
{
  template< typename T >
  static bool step(T &sim) {return true;}

  template< typename T >
  static bool init_objs(T &sim) { return true;}

  template< typename T >
  static typename clock_traits< typename T::clock_tag >::time_type get_dt(T &sim)
  {
    return clock_traits<typename T::clock_tag >::zero(); 
  }
};

template< typename HEAD, typename TAIL >
struct step_sim_impl
{
  template< typename T >
  static typename clock_traits<typename T::clock_tag>::time_type get_dt(T &sim)
  {
    return sim.template stage_<HEAD>().dts;
  }

  template< typename T >
  static bool init_objs(T &sim)
  {
    sim.template stage_<HEAD>().init_objs();
    return false;
  }

  template< typename T >
  static bool step( T &sim )
  {
    if (!sim.template stage_<HEAD>().completed)
    {
      bool end_of_sim = false;
      bool end_of_step = false;
    
      while (!end_of_step)
      {
        sim.clock.event(sim.tmax);
        
        sim.template stage_<HEAD>().update(sim, sim.clock);
      
        // First tick of this stage?
        if (sim.clock.tick_first)
        {
          sim.template stage_<HEAD>().report(sim.clock);
          sim.clock.tick_first = false;
        }
        
        sim.template stage_<HEAD>().propagate<T::algorithm>(sim.clock);
    
        sim.update_clock();
        
        // End of step?
        if (sim.clock.ready)
        {
          end_of_step = true;
      
          sim.template stage_<HEAD>().report(sim.clock);
       
          // Advance to next stage?
          if (sim.next_sim_stage)
          {
            sim.next_sim_stage = false;
            sim.template stage_<HEAD>().completed = true;

            sim.clock.reset(TAIL::get_dt(sim));
            sim.clock.tick_first = true;

            end_of_sim = TAIL::init_objs(sim);
          }
          
          // End of simulation?
          if ((sim.clock.t + clock_traits<T::clock_tag>::epsilon()) >= sim.tmax)
          {
            end_of_sim = true;
          }
       
          // Simulation stopped? 
          if (sim.halt_sim)
          {
            end_of_sim = true;
          }
        }
      }
    
      if (end_of_sim)
      {
        sim.clock.tick_last = true;
        sim.template stage_<HEAD>().report(sim.clock);
      }
    
      return end_of_sim;
    }
    
    return TAIL::step( sim );
  }
};

template< typename T >
struct step_sim
{
  typedef typename mpl::reverse_fold<
    typename T::stage_vector,
    null_step_sim_impl,
    step_sim_impl< mpl::placeholders::_2, mpl::placeholders::_1 >
  >::type type;
};

/**************************************************************************************************/

template< typename T >
inline bool step( T &sim )
{
  return step_sim<T>::type::step( sim );
}

/**************************************************************************************************/

}}

/**************************************************************************************************/

#endif

/**************************************************************************************************/
