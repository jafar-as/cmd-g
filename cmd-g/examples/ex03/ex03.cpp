/*
    Copyright 2012 Jay Graham

    Distributed under the MIT License (see accompanying file LICENSE_1_0_0.txt
    or http://www.opensource.org/licenses/mit-license.php)
*/

/**************************************************************************************************/

#include <boost/mpl/vector.hpp>

#include <cmdg/de/sim.h>
#include <cmdg/de/stage.h>
#include <cmdg/algorithm/euler.h>
#include <cmdg/de/run.h>
#include <cmdg/de/double_clock.h>

#include "./model.h"

/**************************************************************************************************/

using namespace boost;
using namespace cmdg;

struct ex03_dts
{
  typedef double time_type;
  static time_type value() { return 0.01; }
};

struct stage_1 : de::stage< ex03_dts, mpl::vector< Model > > { };

/**************************************************************************************************/

int main(int argc, char* argv[])
{
  const double tmax = 2.00;
  

  typedef de::sim< de::double_clock_tag, algorithm::euler, mpl::vector< stage_1 > > sim_type;
  
  sim_type sim(tmax);
  
  Model model(0.0);

  sim.stage_<stage_1>().obj_<Model>() = &model;
  
  de::run(sim);
  
  return 0;
}

/**************************************************************************************************/

  
