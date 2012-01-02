/*
    Copyright 2012 Jay Graham

    Distributed under the MIT License (see accompanying file LICENSE_1_0_0.txt
    or http://www.opensource.org/licenses/mit-license.php)
*/

/**************************************************************************************************/

#ifndef CMDG_DE_STAGE_H
#define CMDG_DE_STAGE_H

/**************************************************************************************************/

#include <boost/mpl/inherit.hpp>
#include <boost/mpl/placeholders.hpp>
#include <boost/mpl/empty.hpp>
#include <boost/mpl/empty_base.hpp>
#include <boost/mpl/for_each.hpp>

#include <boost/static_assert.hpp>

#include <boost/units/quantity.hpp>
#include <boost/units/systems/si.hpp>

#include <cmdg/de/propagate.h>

/**************************************************************************************************/

using namespace boost;
using namespace boost::units;
using namespace boost::units::si;

/**************************************************************************************************/

namespace cmdg {
namespace de {

/**************************************************************************************************/

template< typename T >
struct sim_obj_base
{
  typedef T obj_type;
  T *obj;
};

/**************************************************************************************************/

template< typename T >
struct init_obj_init_count
{
  T &stage_obj;
  
  init_obj_init_count( T &s ) : stage_obj(s) {}
  
  template< typename S >
  void operator()(S &)
  {
    stage_obj.template obj_<S>()->init_count = 0;
  }
};

/**************************************************************************************************/

#if 0
template< typename T >
struct inc_obj_init_count
{
  T &stage_obj;
  
  inc_obj_init_count( T &s ) : stage_obj(s) {}
  
  template< typename S >
  void operator()(S &)
  {
    ++(stage_obj.template obj_<S>()->init_count);
  }
};
#endif

/**************************************************************************************************/

template< typename T >
struct init_obj
{
  T &stage_obj;
  
  init_obj( T &s ) : stage_obj(s) {}
  
  template< typename S >
  void operator()(S &)
  {
    stage_obj.template obj_<S>()->init();
    ++(stage_obj.template obj_<S>()->init_count);
  }
};

/**************************************************************************************************/

template< typename T, typename C >
struct report_obj
{
  T &stage_obj;
  C &c;
  
  report_obj( T &s, C &c ) : stage_obj(s), c(c) {}
  
  template< typename S >
  inline void operator()(S &)
  {
    stage_obj.template obj_<S>()->report(c);
  }
};

template< typename T, typename S, typename C >
struct update_obj
{
  T &stage_obj;
  S &sim;
  C &c;
  
  update_obj( T &s, S &sim, C &c ) : stage_obj(s), sim(sim), c(c) {}
  
  template< typename S >
  inline void operator()(S &)
  {
    stage_obj.template obj_<S>()->update(sim, c);
  }
};

template< typename A, typename T, typename C >
struct propagate_obj
{
  T &stage_obj;
  C &c;
  
  propagate_obj( T &s, C &c ) : stage_obj(s), c(c) {}
  
  template< typename S >
  inline void operator()(S &)
  {
    propagate<A>( *(stage_obj.template obj_<S>()), c );
  }
};

/**************************************************************************************************/

template< typename DTS, typename OBJ_VECTOR >
struct stage : public mpl::reverse_fold<
  OBJ_VECTOR,
  mpl::empty_base,
  mpl::inherit< sim_obj_base< mpl::placeholders::_2 >, mpl::placeholders::_1 >
>::type
{
  BOOST_STATIC_ASSERT( (boost::mpl::empty< OBJ_VECTOR >::value != true) );
  
  typedef OBJ_VECTOR obj_vector;
  
  stage() : dts(DTS::value()) {}
  
  void init()
  {
    completed = false;
    mpl::for_each< OBJ_VECTOR >(init_obj_init_count< stage >(*this));
  }
  
  void init_objs()
  {
    mpl::for_each< OBJ_VECTOR >(init_obj< stage >(*this));
    //mpl::for_each< OBJ_VECTOR >(inc_obj_init_count< stage >(*this));
  }

  template< typename C >
  void report(C &c)
  {
    mpl::for_each< OBJ_VECTOR >(report_obj< stage, C >(*this, c));
  }

  template< typename S, typename C >
  void update(S &sim, C &c)
  {
    mpl::for_each< OBJ_VECTOR >(update_obj< stage, S, C >(*this, sim, c));
  }

  template< typename A, typename C >
  void propagate(C &c)
  {
    mpl::for_each< OBJ_VECTOR >(propagate_obj< A, stage, C >(*this, c));
  }
  
  template< typename T >
  inline T const *obj_() const
  {
    return( static_cast< const sim_obj_base< T > & >(*this).obj );
  }
  
  template< typename T >
  inline T * &obj_() 
  {
    return( static_cast< sim_obj_base< T > & >(*this).obj );
  }
 
  typename DTS::time_type dts;
  bool completed;
};

/**************************************************************************************************/

}}

/**************************************************************************************************/

#endif

/**************************************************************************************************/
