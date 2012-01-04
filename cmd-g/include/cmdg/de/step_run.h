/*
    Copyright 2012 Jay Graham

    Distributed under the MIT License (see accompanying file LICENSE_1_0_0.txt
    or http://www.opensource.org/licenses/mit-license.php)
*/

/**************************************************************************************************/

#ifndef CMDG_DE_STEP_RUN_H
#define CMDG_DE_STEP_RUN_H

/**************************************************************************************************/

#include <cmdg/de/step.h>

/**************************************************************************************************/

namespace cmdg {
namespace de {

/**************************************************************************************************/

template< typename T >
inline void step_run(T &sim)
{
  sim.init();

  bool sim_is_terminated = false;
  
  while( !sim_is_terminated )
  {
    sim_is_terminated = step(sim);
  }
}

/**************************************************************************************************/

}}

/**************************************************************************************************/

#endif

/**************************************************************************************************/
