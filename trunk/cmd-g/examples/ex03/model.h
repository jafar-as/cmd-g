/*
    Copyright 2012 Jay Graham

    Distributed under the MIT License (see accompanying file LICENSE_1_0_0.txt
    or http://www.opensource.org/licenses/mit-license.php)
*/

/**************************************************************************************************/

#ifndef MODEL_H
#define MODEL_H

/**************************************************************************************************/

#include <cstdio>
#include <iostream>

#include <boost/mpl/vector.hpp>

#include <cmdg/de/state.h>
#include <cmdg/de/obj.h>

/**************************************************************************************************/

struct gamma : cmdg::de::state< double, double > {};

/**************************************************************************************************/
 
struct Model : cmdg::de::obj< mpl::vector< gamma > >
{
  Model(double gamma_)
  {
    gamma0 = gamma_;
    add_integrator<gamma>(gammad);
  }
 
  Model()
  {
    gamma0 = 0.0;
    add_integrator<gamma>(gammad);
  }
  
  void init()
  {
    std::cout << "starting Model...\n";
    
    if( init_count == 0)
    {
      s_<gamma>().value = gamma0;
      k = 1000.0;
    }
  }
  
  template< typename S, typename C >     
  void update(S &, C & )
  {
    gamma_cmd = 1.0;
    v = 1000.0;
    a_cmd = k * ( gamma_cmd - s_<gamma>().value);
    gammad = a_cmd / v;
  }
    
  template< typename C >
  void report(C & clock)
  {
    if( clock.sample(0.1))
    {
      printf( "%8.3f %8.6f\n", clock.now(), s_<gamma>().value); 
    }
  }
  
  double gamma0, gammad;
  double gamma_cmd;
  double k;
  double v;
  double a_cmd;
};

/**************************************************************************************************/

#endif

/**************************************************************************************************/
