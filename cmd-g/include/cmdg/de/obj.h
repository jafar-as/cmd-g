/*
    Copyright 2012 Jay Graham

    Distributed under the MIT License (see accompanying file LICENSE_1_0_0.txt
    or http://www.opensource.org/licenses/mit-license.php)
*/

/**************************************************************************************************/

#ifndef CMDG_DE_OBJ_H
#define CMDG_DE_OBJ_H

/**************************************************************************************************/

#include <boost/mpl/inherit.hpp>
#include <boost/mpl/placeholders.hpp>
#include <boost/mpl/empty.hpp>
#include <boost/mpl/empty_base.hpp>
#include <boost/mpl/for_each.hpp>
#include <boost/mpl/reverse_fold.hpp>

#include <boost/static_assert.hpp>

#include <cmdg/de/state.h>

/**************************************************************************************************/

namespace cmdg {
namespace de {

/**************************************************************************************************/

using namespace boost;

/**************************************************************************************************/

template< typename STATE_VECTOR = mpl::vector<> >
struct obj : public mpl::reverse_fold<
  STATE_VECTOR,
  mpl::empty_base,
  mpl::inherit< state_base< mpl::placeholders::_2 >, mpl::placeholders::_1 >
>::type
{
  //BOOST_STATIC_ASSERT( (boost::mpl::empty< STATE_VECTOR >::value != true) );
  
  typedef STATE_VECTOR state_vector;
  
  template< typename T >
  inline state_base< T > const &s_() const
  {
    return( static_cast< const state_base< T > & >(*this) );
  }
  
  template< typename T >
  inline state_base< T > &s_()
  {
    return( static_cast< state_base< T > & >(*this) );
  }
  
  template< typename T1, typename T2 >
  void add_integrator()
  {
    s_<T1>().xd = &s_<T2>().value;
  }
  
  template< typename T1, typename T2 >
  void add_integrator(T2 &v2)
  {
    s_<T1>().xd = &v2;
  }
  
  unsigned init_count;
};

/**************************************************************************************************/

}}

/**************************************************************************************************/

#endif

/**************************************************************************************************/
