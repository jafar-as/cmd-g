/*
    Copyright 2012 Jay Graham

    Distributed under the MIT License (see accompanying file LICENSE_1_0_0.txt
    or http://www.opensource.org/licenses/mit-license.php)
*/

/**************************************************************************************************/

#ifndef CMDG_ALGORITHM_RK4_H
#define CMDG_ALGORITHM_RK4_H

/**************************************************************************************************/
#include <cmath>

#include <boost/mpl/for_each.hpp>
#include <boost/mpl/inherit.hpp>
#include <boost/mpl/placeholders.hpp>
#include <boost/mpl/empty.hpp>
#include <boost/mpl/empty_base.hpp>
#include <boost/mpl/for_each.hpp>
#include <boost/mpl/reverse_fold.hpp>

#include <cmdg/de/state.h>
#include <cmdg/de/clock.h>
#include <cmdg/de/clock_traits.h>

/**************************************************************************************************/

namespace cmdg {
namespace algorithm {

/**************************************************************************************************/

 struct null_prop_state_impl
  {
    template< typename T, typename C >
    inline static void prop_state(T &, C &) {}
  };
  
  template< typename HEAD, typename TAIL >
  struct prop_state_impl
  {
    template< typename T, typename C >
    inline static void prop_state(T &obj, C &c)
    {
      obj.template s_<HEAD>().prop_state(c);
      TAIL::prop_state(obj, c);
    }
  };
  
  template< typename T >
  struct prop_state_struct
  {
    typedef typename mpl::reverse_fold<
      typename T::state_vector,
      null_prop_state_impl,
      prop_state_impl< mpl::placeholders::_2, mpl::placeholders::_1 >
    >::type type;
  };

struct rk4
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
      cmdg::de::state_base< S > &s = obj.template s_<S>();
     
      switch(c.k_pass)
      {
      case 0:
        s.x0 = s.value;
        s.xd0 = *s.xd;
        s.value = s.x0 + (c.dt/2.0) * s.xd0;
        break;
      case 1:
        s.xd1 = *s.xd;
        s.value = s.x0 + (c.dt/2.0) * s.xd1;
        break;
      case 2:
        s.xd2 = *s.xd;
        s.value = s.x0 + c.dt * s.xd2;
        break;
      case 3:
        s.xd3 = *s.xd;
        s.value = s.x0 + (c.dt/6.0) * ( s.xd0 + (2.0 * s.xd1) + (2.0 * s.xd2) + s.xd3);
        break;
      }
      
     #if 0 
      switch(c.k_pass)
      {
      case 0:
        obj.template s_<S>().x0 = obj.template s_<S>().value;
        obj.template s_<S>().xd0 = *obj.template s_<S>().xd;
        obj.template s_<S>().value = obj.template s_<S>().x0 + (c.dt/2.0) * obj.template s_<S>().xd0;
        break;
      case 1:
        obj.template s_<S>().xd1 = *obj.template s_<S>().xd;
        obj.template s_<S>().value = obj.template s_<S>().x0 + (c.dt/2.0) * obj.template s_<S>().xd1;
        break;
      case 2:
        obj.template s_<S>().xd2 = *obj.template s_<S>().xd;
        obj.template s_<S>().value = obj.template s_<S>().x0 + c.dt * obj.template s_<S>().xd2;
        break;
      case 3:
        obj.template s_<S>().xd3 = *obj.template s_<S>().xd;
        obj.template s_<S>().value = obj.template s_<S>().x0 + (c.dt/6.0) * ( obj.template s_<S>().xd0 + (2.0 * obj.template s_<S>().xd1) + (2.0 * obj.template s_<S>().xd2) + obj.template s_<S>().xd3);
        break;
      }
     #endif
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
    
    if (c.k_pass == 2)
    {
      c.t = c.t1;
    }
    
    ++(c.k_pass);
    c.k_pass = c.k_pass % 4;
    
    if (c.k_pass == 0)
    {
      c.ready = true;
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
