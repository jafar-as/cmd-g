/*
    Copyright 2012 Jay Graham

    Distributed under the MIT License (see accompanying file LICENSE_1_0_0.txt
    or http://www.opensource.org/licenses/mit-license.php)
*/

/**************************************************************************************************/

#ifndef CMDG_ALGORITHM_EULER_H
#define CMDG_ALGORITHM_EULER_H

/**************************************************************************************************/

#include <cmath>

#include <boost/mpl/for_each.hpp>

#include <cmdg/de/clock.h>
#include <cmdg/de/clock_traits.h>

/**************************************************************************************************/

namespace cmdg {
namespace algorithm {

/**************************************************************************************************/

struct euler
{
  template< typename T, typename C >
  struct propagate_state
  {
    T &obj;
    C &clock;
  
    propagate_state( T &obj, C &c ) : obj(obj), clock(c) {}
  
    template< typename S >
    inline void operator()(S &)
    {
      obj.template s_<S>().x0 = obj.template s_<S>().value;
      obj.template s_<S>().value = obj.template s_<S>().x0 + (clock.dt * (*obj.template s_<S>().xd));
    }
  };
  
  template< typename T, typename C >  
  static void propagate(T &obj, C &clock)
  {
    mpl::for_each< typename T::state_vector >(propagate_state<T, C>(obj, clock));
  }

  template< typename C >
  static void update( C &clock )
  {
    clock.t = clock.t1;
    clock.t1 = floor((clock.t + cmdg::de::clock_traits<typename C::clock_tag>::epsilon())/clock.dtp + 1) * clock.dtp;
    clock.ready = true;
  }
};

/**************************************************************************************************/

}}

/**************************************************************************************************/

#endif

/**************************************************************************************************/
