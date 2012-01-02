/*
    Copyright 2012 Jay Graham

    Distributed under the MIT License (see accompanying file LICENSE_1_0_0.txt
    or http://www.opensource.org/licenses/mit-license.php)
*/

/**************************************************************************************************/

#ifndef CMDG_DE_SI_CLOCK_H
#define CMDG_DE_SI_CLOCK_H

/**************************************************************************************************/

#include <boost/units/cmath.hpp>
#include <boost/units/quantity.hpp>
#include <boost/units/systems/si.hpp>

#include <cmdg/de/clock_traits.h>
#include <cmdg/de/clock_abs_policy.h>

/**************************************************************************************************/

using namespace boost::units;

/**************************************************************************************************/

namespace cmdg {
namespace de {

/**************************************************************************************************/

struct si_clock_tag;

/**************************************************************************************************/

template<>
struct clock_traits<si_clock_tag>
{
  typedef quantity<si::time> time_type;
  inline static time_type epsilon() { return 1e-8*si::seconds; }
  inline static time_type zero() { return 0.0*si::seconds; }
};

template<>
struct clock_abs_policy<si_clock_tag>
{
  typedef clock_traits<si_clock_tag>::time_type time_type;
  
  inline static time_type abs(time_type v)
  {
    return boost::units::fabs(v);
  };
};

}}

/**************************************************************************************************/

#endif

/**************************************************************************************************/
