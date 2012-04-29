/*
    Copyright 2012 Jay Graham

    Distributed under the MIT License (see accompanying file LICENSE_1_0_0.txt
    or http://www.opensource.org/licenses/mit-license.php)
*/

/**************************************************************************************************/

#include <iostream>
#include <cmath>

#include <boost/units/quantity.hpp>
#include <boost/units/systems/si.hpp>
#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/io.hpp>

#include <cmdg/de/sim.h>
#include <cmdg/algorithm/euler.h>
#include <cmdg/algorithm/rk4.h>
#include <cmdg/algorithm/rk2.h>
#include <cmdg/de/stage.h>
#include <cmdg/de/state.h>
#include <cmdg/de/obj.h>
#include <cmdg/de/run.h>

#include <cmdg/de/si_clock.h>

/**************************************************************************************************/

using namespace std;

using namespace boost;
using namespace boost::units;
using namespace boost::units::si;

using namespace cmdg;
using namespace cmdg::de;

  static const double PI = 4.0 * atan(1.0);
  static const double R = 180.0 / PI; // rad to deg conversion factor
  
  struct x_pos : state< quantity<si::length>, quantity<si::velocity> > {};
  struct y_pos : state< quantity<si::length>, quantity<si::velocity> > {};
  
  struct x_vel : state< quantity<si::velocity>, quantity<si::acceleration> > {};
  struct y_vel : state< quantity<si::velocity>, quantity<si::acceleration> > {};
  
  struct Target : obj< mpl::vector< x_pos, y_pos > >
  {
    quantity<si::velocity> vx; 
    quantity<si::velocity> vy; 
    
    Target( quantity<length> x, quantity<length> y, quantity<velocity> vx, quantity<velocity> vy)
    {
      s_<x_pos>().value = x;
      s_<y_pos>().value = y;
      
      this->vx = vx;
      this->vy = vy;
      
      add_integrator<x_pos>(this->vx);
      add_integrator<y_pos>(this->vy);
    }
  
    Target()
    {
      add_integrator<x_pos>(this->vx);
      add_integrator<y_pos>(this->vy);
    }
    
    void init() {}

    template< typename S, typename C >     
    void update(S &, C & c)
    {
      //std::cout << "update obj_a\n";
    }
    
    template< typename C >
    void report(C & c)
    {
      if( c.sample(1.0*si::second) || c.tick_first || c.tick_last)
      {
        //printf( "%12s %8.3f %8.3f %8.3f\n", "Target", c.t, x_pos_value, y_pos_value);
    	std::cout << "target " << c.t
    			  << " " << s_<x_pos>().value
    			  << " " << s_<y_pos>().value
    			  << std::endl;
      }
    }
  };
  
  struct Missile : obj< mpl::vector< x_pos, y_pos > >
  {
    Missile()
    {
      add_integrator<x_pos>(vx);
      add_integrator<y_pos>(vy);
    }
    
    Missile( Target *target, quantity<length> x, quantity<length> y, quantity<velocity> vel)
    {
      s_<x_pos>().value = x;
      s_<y_pos>().value = y;
      
      this->vel = vel;
      this->target = target;
      
      add_integrator<x_pos>(vx);
      add_integrator<y_pos>(vy);
    }
    
    void init() {}
   
   template< typename S, typename C > 
   void update(S &s, C & c)
   {
      quantity<length> dx = target->s_<x_pos>().value - s_<x_pos>().value; 
      quantity<length> dy = target->s_<y_pos>().value - s_<y_pos>().value; 
      
      d =  boost::units::sqrt( dx * dx + dy * dy );
      vx = vel * ( target->s_<x_pos>().value - s_<x_pos>().value) / d;
      vy = vel * ( target->s_<y_pos>().value - s_<y_pos>().value) / d;
      
      if( d <= 0.1*meters)
      {
        s.halt();
      }
    }
   
   template< typename C > 
   void report(C & c)
   {
      if( c.sample(1.0*si::seconds) || c.tick_first || c.tick_last)
      {
        //printf( "%12s %8.3f %8.3f %8.3f %8.3f\n", "Missile", c.now(), x_pos_value, y_pos_value, d);
    	std::cout << "Missile " << c.now()
    			  << " " << s_<x_pos>().value
    			  << " " << s_<y_pos>().value
    			  << std::endl;
      }
    }
   
    quantity<length> d;
    quantity<si::velocity> vx; 
    quantity<si::velocity> vy; 
    quantity<velocity> vel;
    Target *target;
  };
 
  struct x1 : state< quantity<si::plane_angle>, quantity<si::angular_velocity> > {};
  struct x2 : state< quantity<si::angular_velocity>, quantity<si::angular_acceleration> > {};
  
  struct Radar : obj< mpl::vector< x1, x2 > >
  {
    Radar()
    {
      target_engaged = false;
      s_<x2>().value = 0.0*si::radian/si::seconds;
      
      add_integrator<x1>(x1d);
      add_integrator<x2>(x2d);
    }
    
    Radar( Target *target, quantity<si::plane_angle> theta, quantity<frequency> wn, double zeta)
    {
      target_engaged = false;
      s_<x1>().value = theta;
      s_<x2>().value = 0.0*radian/seconds;
      
      this->wn = wn;
      this->zeta = zeta;
      this->target = target;
      
      add_integrator<x1>(x1d);
      add_integrator<x2>(x2d);
    }
    
    void init() {}

   template< typename S, typename C >    
   void update(S &s, C & c)
   {
      quantity<si::plane_angle> theta_target
        = atan((target->s_<y_pos>().value / target->s_<x_pos>().value).value())*si::radian;
      
      theta_err = (theta_target - s_<x1>().value); 
      if( (fabs( (theta_err * R).value() ) < 1.0) && (!target_engaged))
      {
        target_engaged = true;
        s.next_stage();
      }
      x1d = s_<x2>().value;
      x2d = (theta_err * wn * wn) - ((2.0 * zeta) * wn * s_<x2>().value);
    }
   
   template< typename C > 
   void report(C & c)
   {
      if( c.sample(1.0*si::seconds) || c.tick_first || c.tick_last)
      {
        //printf( "%12s %8.3f %8.3f\n", "Radar", c.now(), theta_r);
        //printf( "\n");

    	std::cout << "Radar " << c.now()
    			  << " " << (theta_err * R)
    			  << std::endl;
      }
    }
    
    quantity<si::plane_angle> theta_err;
    quantity<angular_velocity> x1d;
    quantity<angular_acceleration> x2d;
    quantity<frequency> wn;
    quantity<dimensionless> zeta;
    
    Target *target;
    bool target_engaged;
  };
  
  struct dts
  {
    typedef quantity<si::time> time_type;
    static time_type value() { return 0.0001*si::seconds; }
  };
  
  struct stage_1 : stage< dts, mpl::vector< Target, Radar > > { };
  struct stage_2 : stage< dts, mpl::vector< Target, Missile, Radar > > { };
  
  typedef sim< si_clock_tag, cmdg::algorithm::rk4, mpl::vector< stage_1, stage_2 > > my_sim;

int main(int argc, char* argv[])
{

  Target target( 20.0*meters, 5.0*meters, -1.0*meters/second, 0.0*meters/second);
  Missile missile( &target, 0.0*meters, 0.0*meters, 2.0*meters/second);
  Radar radar( &target, (60.0/R)*radians, 2.643*hertz, 0.7);
  
  my_sim s(10.0*si::seconds);
  
  s.stage_<stage_1>().obj_<Target>() = &target;
  s.stage_<stage_1>().obj_<Radar>() = &radar;
  
  s.stage_<stage_2>().obj_<Target>() = &target;
  s.stage_<stage_2>().obj_<Missile>() = &missile;
  s.stage_<stage_2>().obj_<Radar>() = &radar;
  
  cmdg::de::run(s);
  
  return 0;
}
