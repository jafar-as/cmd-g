/*
    Copyright 2012 Jay Graham

    Distributed under the MIT License (see accompanying file LICENSE_1_0_0.txt
    or http://www.opensource.org/licenses/mit-license.php)
*/

/**************************************************************************************************/

#ifndef CMDG_DE_STATE_H
#define CMDG_DE_STATE_H

/**************************************************************************************************/

namespace cmdg {
namespace de {

/**************************************************************************************************/

template< typename T >
struct state_base
{
  typename T::value_type value;
  typename T::derivative_type *xd;
  
  typename T::value_type x0;
  typename T::derivative_type xd0, xd1, xd2, xd3;
  
  state_base() : xd(0) {}
};

/**************************************************************************************************/

template< typename TX, typename TXD >
struct state
{
  typedef TX value_type;
  typedef TXD derivative_type;
};

/**************************************************************************************************/

}}

/**************************************************************************************************/

#endif

/**************************************************************************************************/
