/*
    Copyright 2012 Jay Graham

    Distributed under the MIT License (see accompanying file LICENSE_1_0_0.txt
    or http://www.opensource.org/licenses/mit-license.php)
*/

/**************************************************************************************************/

#ifndef CMDG_DE_DOUBLE_CLOCK_H
#define CMDG_DE_DOUBLE_CLOCK_H

/**************************************************************************************************/

#include <cmath>

#include <cmdg/de/clock_traits.h>
#include <cmdg/de/clock_abs_policy.h>

/**************************************************************************************************/

namespace cmdg {
namespace de {

/**************************************************************************************************/

struct double_clock_tag;

/**************************************************************************************************/

template<>
struct clock_traits<double_clock_tag>
{
  typedef double time_type;
  inline static time_type epsilon() { return 1.0e-8; }
  inline static time_type zero() { return 0.0; }
};

template<>
struct clock_abs_policy<double_clock_tag>
{
  typedef clock_traits<double_clock_tag>::time_type time_type;
  
  inline static time_type abs(time_type v)
  {
    return fabs(v);
  };
};

}}

/**************************************************************************************************/

#endif

/**************************************************************************************************/
