/*
    Copyright 2012 Jay Graham

    Distributed under the MIT License (see accompanying file LICENSE_1_0_0.txt
    or http://www.opensource.org/licenses/mit-license.php)
*/

/**************************************************************************************************/

#ifndef CMDG_DE_CLOCK_H
#define CMDG_DE_CLOCK_H

/**************************************************************************************************/

#include <cmath>

#include <boost/units/cmath.hpp>
#include <boost/units/quantity.hpp>
#include <boost/units/systems/si.hpp>

/**************************************************************************************************/

using namespace boost;
using namespace boost::units;

/**************************************************************************************************/

#include <cmdg/de/clock_traits.h>
#include <cmdg/de/clock_abs_policy.h>

/**************************************************************************************************/

namespace cmdg {
namespace de {

/**************************************************************************************************/

template< typename TAG >
class clock
{
public:
  typedef TAG clock_tag;
  
  clock() {}
  ~clock() {}
  
  inline bool sample() const { return clock::ready; }

  inline bool sample(typename clock_traits<TAG>::time_type sdt)
  {
    if (!ready)
    {
      return false;
    }
  
    // calculate end of next sample time
    const typename clock_traits<TAG>::time_type ts = floor((t + clock_traits<TAG>::epsilon())/sdt + 1) * sdt;
 
    // Is the next sample time less than the current earliest event? 
    if (ts < (t1 - clock_traits<TAG>::epsilon()))
    {
      t1 = ts; // new earliest event
    }
  
    dt = t1 - t; // new time step
  
    // check to see if it's time to sample
    //
    // It's time to sample when the difference between the end of the next
    // sample time (ts) and the current time (t) is within EPSILON of the
    // sample rate (sdt).
    //
    // Note that (ts - t) is always positive and less than or equal to sdt,
    //
    //   (ts -t) - sdt <= 0
    //
    // Therefore, it is time to sample when (ts - t) - sdt is within EPSILON 
    // of zero, i.e., when
    //
    //  (ts - t) - sdt > -EPSILON
    //
    //        or
    // 
    //  t - ts + sdt < EPSILON
    //
    if ((t - ts + sdt) < clock_traits<TAG>::epsilon())
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  
  inline bool event(typename clock_traits<TAG>::time_type t_event)
  {
    if (!ready)
    {
      return false;
    }
  
    // Is this event earlier then the current earliest event and greater than the current time?
    if ((t_event < (t1 - clock_traits<TAG>::epsilon())) && (t_event >= (t + clock_traits<TAG>::epsilon())))
    {
      t1 = t_event; // new earliest event
    }
    
    dt = t1 - t; // new time step
    
    // Is this event occuring now i.e., is it within EPSILON of the current time?
    if (clock_abs_policy<TAG>::abs(t_event - t) < clock_traits<TAG>::epsilon())
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  
  inline typename clock_traits<TAG>::time_type now() const { return t; }

  inline void reset(typename clock_traits<TAG>::time_type new_dtp)
  {
    dt = dtp = new_dtp;
    t1 = t + dtp;
  }
  
  inline void init()
  {
    k_pass = 0;
    ready = true;
    t = clock_traits<TAG>::zero(); 
    tick_first = true;
    tick_last = false;
  }

  typename clock_traits<TAG>::time_type t;   // Current time
  typename clock_traits<TAG>::time_type dt;  // Size of the next time step. Not always equal to the
                                             // integration time step size.
  typename clock_traits<TAG>::time_type dtp; // Integration time step size.
  typename clock_traits<TAG>::time_type t1;  // Time of next event/sample/integration step,
                                             // whichever is smallest.
                          
  int k_pass;
  
  bool tick_first; // True on the very first tick of the clock.
  bool tick_last;  // True on the very last tick of the clock.
  bool ready;      // True when not in the middle of performing an integration step
                   // i.e., current time falls on an integration time step.
  
private:
  // off
  clock(const clock &);
  clock &operator=(const clock &);
};

/**************************************************************************************************/

}}

/**************************************************************************************************/

#endif

/**************************************************************************************************/
