/*
    Copyright 2012 Jay Graham

    Distributed under the MIT License (see accompanying file LICENSE_1_0_0.txt
    or http://www.opensource.org/licenses/mit-license.php)
*/

/**************************************************************************************************/

#ifndef CMDG_DE_SIM_H
#define CMDG_DE_SIM_H

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

template< typename T >
struct init_stage
{
  T &sim_obj;
  
  init_stage( T &s ) : sim_obj(s) {}
  
  template< typename S >
  void operator()(S &)
  {
    sim_obj.template stage_<S>().init();
  }
};

/**************************************************************************************************/

template< typename T >
struct stage_base
{
  typedef T stage_type;
  T stage;
};

/**************************************************************************************************/

template< typename CLOCK_TAG, typename ALGORITHM, typename STAGE_VECTOR >
struct sim : public
  mpl::reverse_fold<
    STAGE_VECTOR,
    mpl::empty_base,
    mpl::inherit< stage_base< mpl::placeholders::_2 >, mpl::placeholders::_1 >
  >::type
{
  BOOST_STATIC_ASSERT( (boost::mpl::empty< STAGE_VECTOR >::value != true) );
  
  typedef CLOCK_TAG clock_tag;
  
  typedef STAGE_VECTOR stage_vector;
  typedef ALGORITHM algorithm;
  
  sim(typename clock_traits<CLOCK_TAG>::time_type t) : tmax(t) {}
  
  inline void init()
  {
    clock.init();
    clock.reset(stage_<mpl::front<STAGE_VECTOR>::type>().dts);
    
    next_sim_stage = false;
    halt_sim = false;
 
    mpl::for_each< STAGE_VECTOR >(init_stage< sim >(*this));
    
    stage_<mpl::front<STAGE_VECTOR>::type>().init_objs();
  }
  
  template< typename T >
  inline T const &stage_() const
  {
    return( static_cast< const stage_base< T > & >(*this).stage );
  }
  
  template< typename T >
  inline T &stage_() 
  {
    return( static_cast< stage_base< T > & >(*this).stage );
  }
  
  inline void next_stage() {next_sim_stage = true;}

  inline void halt() {halt_sim = true;}

  inline void update_clock()
  {
    ALGORITHM::update(clock);
  }
  
  cmdg::de::clock<CLOCK_TAG> clock;
  
  const typename clock_traits<CLOCK_TAG>::time_type tmax;
 
  bool next_sim_stage; 
  bool halt_sim; 
};

/**************************************************************************************************/

}}

/**************************************************************************************************/

#endif

/**************************************************************************************************/
