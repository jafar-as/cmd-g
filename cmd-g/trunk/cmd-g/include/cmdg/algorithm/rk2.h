/*
    Copyright 2012 Jay Graham

    Distributed under the MIT License (see accompanying file LICENSE_1_0_0.txt
    or http://www.opensource.org/licenses/mit-license.php)
*/

/**************************************************************************************************/

#ifndef CMDG_ALGORITHM_RK2_H
#define CMDG_ALGORITHM_RK2_H

/**************************************************************************************************/

#include <cmath>

#include <cmdg/de/clock.h>
#include <cmdg/de/clock_traits.h>

/**************************************************************************************************/

namespace cmdg {
namespace algorithm {

/**************************************************************************************************/
struct rk2
{
  template< typename T, typename C >
  struct propagate_state
  {
    T &obj;
    C &c;
  
    propagate_state( T &obj, C &c ) : obj(obj), c(c) {}
  
    template< typename S >
    inline void operator()(S &)
    {
      switch(c.k_pass)
      {
        case 0:
          obj.template s_<S>().x0 = obj.template s_<S>().value;
          obj.template s_<S>().xd0 = *obj.template s_<S>().xd;
          obj.template s_<S>().value = obj.template s_<S>().x0 + c.dt/2.0 * obj.template s_<S>().xd0;
          break;
        case 1:
          obj.template s_<S>().xd1 = *obj.template s_<S>().xd;
          obj.template s_<S>().value = obj.template s_<S>().x0 + c.dt * obj.template s_<S>().xd1;
          break;
      }
    }
  };
  
  template< typename T, typename C >  
  static void propagate( T &obj, C &c )
  {
    mpl::for_each< typename T::state_vector >(propagate_state<T, C>(obj, c));
  }

  template< typename C >
  static void update( C &c )
  {
      if (c.k_pass == 0)
      {
        c.t = c.t + c.dt/2.0;
      }
    
      if (c.k_pass == 1)
      {
        c.t = c.t1;
      }
   
      ++(c.k_pass);
      c.k_pass = c.k_pass % 2;
    
      if( c.k_pass == 0)
      {
        c.ready = true;
        // Add clock::EPSILON to current time c.t just in case the current time
        // is within "zero" time units of the next time step c.::dtp
        // - or -
        // if (abs(floor(t/dtp)*dtp - t) < EPSILON)
        //    c.::t is within "zero" units of the next time step.
       
        c.t1 = floor((c.now() + cmdg::de::clock_traits<typename C::clock_tag>::epsilon())/c.dtp + 1) * c.dtp;
      }
      else
      {
        c.ready = false;
      }
  }
};

/**************************************************************************************************/

}}

/**************************************************************************************************/

#endif

/**************************************************************************************************/
