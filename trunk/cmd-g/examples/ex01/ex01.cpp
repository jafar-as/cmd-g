/*
    Copyright 2012 Jay Graham

    Distributed under the MIT License (see accompanying file LICENSE_1_0_0.txt
    or http://www.opensource.org/licenses/mit-license.php)
*/

/**************************************************************************************************/
#include <stdio.h>

#include <cmdg/de/sim.h>
#include <cmdg/algorithm/euler.h>
#include <cmdg/algorithm/rk4.h>
#include <cmdg/algorithm/rk2.h>
#include <cmdg/de/stage.h>
#include <cmdg/de/state.h>
#include <cmdg/de/obj.h>
#include <cmdg/de/run.h>

#include <cmdg/de/double_clock.h>

/**************************************************************************************************/

using namespace std;

using namespace cmdg;
using namespace cmdg::de;

static const double PI = 4.0 * atan(1.0);
static const double R = 180.0 / PI; // rad to deg conversion factor
  
struct x_pos : state< double, double > {};
struct y_pos : state< double, double > {};
  
struct Target : obj< mpl::vector< x_pos, y_pos > >
{
  Target( double x, double y, double vx, double vy)
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
    void report(C & clock)
    {
      if( clock.sample(1.0) || clock.tick_first || clock.tick_last)
      {
        printf( "%12s %8.3f %8.3f %8.3f\n", "Target", clock.now(), s_<x_pos>().value, s_<y_pos>().value);
      }
    }
    
  double vx;
  double vy;
};
  
struct Missile : obj< mpl::vector< x_pos, y_pos > >
{
    Missile()
    {
      add_integrator<x_pos>(vx);
      add_integrator<y_pos>(vy);
    }
    
    Missile( Target *target, double x, double y, double vel)
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
   void update(S &sim, C &)
   {
      double dx = target->s_<x_pos>().value - s_<x_pos>().value; 
      double dy = target->s_<y_pos>().value - s_<y_pos>().value; 
      
      d =  sqrt( dx * dx + dy * dy );
      vx = vel * ( target->s_<x_pos>().value - s_<x_pos>().value) / d;
      vy = vel * ( target->s_<y_pos>().value - s_<y_pos>().value) / d;
      
      if( d <= 0.1)
      {
        sim.halt();
      }
    }
   
   template< typename C > 
   void report(C & clock)
   {
      if( clock.sample(1.0) || clock.tick_first || clock.tick_last)
      {
        printf( "%12s %8.3f %8.3f %8.3f %8.3f\n", "Missile", clock.now(), s_<x_pos>().value, s_<y_pos>().value, d);
      }
    }
   
    double d;
    double vx; 
    double vy; 
    double vel;
    Target *target;
  };
 
  struct x1 : state< double, double > {};
  struct x2 : state< double, double > {};
  
  struct Radar : obj< mpl::vector< x1, x2 > >
  {
    Radar()
    {
      target_engaged = false;
      s_<x2>().value = 0.0;
      
      add_integrator<x1>(x1d);
      add_integrator<x2>(x2d);
    }
    
    Radar( Target *target, double theta, double wn, double zeta)
    {
      target_engaged = false;
      s_<x1>().value = theta;
      s_<x2>().value = 0.0;
      
      this->wn = wn;
      this->zeta = zeta;
      this->target = target;
      
      add_integrator<x1>(x1d);
      add_integrator<x2>(x2d);
    }

    void init() {}
    
   template< typename S, typename C >    
   void update(S &sim, C &)
   {
      double theta_target = atan(target->s_<y_pos>().value / target->s_<x_pos>().value);
      
      theta_err = (theta_target - s_<x1>().value); 
      if( (fabs( theta_err * R ) < 1.0) && (!target_engaged))
      {
        target_engaged = true;
        sim.next_stage();
      }
      x1d = s_<x2>().value;
      x2d = (theta_err * wn * wn) - ((2.0 * zeta) * wn * s_<x2>().value);
    }
   
   template< typename C > 
   void report(C & clock)
   {
      if( clock.sample(1.0) || clock.tick_first || clock.tick_last)
      {
        printf( "%12s %8.3f %8.3f\n", "Radar", clock.now(), theta_err * R);
        printf( "\n");
      }
    }
    
    double theta_err;
    double x1d;
    double x2d;
    double wn;
    double zeta;
    
    Target *target;
    bool target_engaged;
  };
  
  struct dts
  {
    typedef double time_type;
    static time_type value() { return 0.0001; }
  };
  
  struct stage_1 : stage< dts, mpl::vector< Target, Radar > > { };
  struct stage_2 : stage< dts, mpl::vector< Target, Missile, Radar > > { };
  
  typedef cmdg::de::sim< double_clock_tag, cmdg::algorithm::rk4, mpl::vector< stage_1, stage_2 > > sim_type;

int main(int argc, char* argv[])
{
  Target target( 20.0, 5.0, -1.0, 0.0);
  Missile missile( &target, 0.0, 0.0, 2.0);
  Radar radar( &target, (60.0/R), 2.643, 0.7);
  
  sim_type sim(10.0);
  
  sim.stage_<stage_1>().obj_<Target>() = &target;
  sim.stage_<stage_1>().obj_<Radar>() = &radar;
  
  sim.stage_<stage_2>().obj_<Target>() = &target;
  sim.stage_<stage_2>().obj_<Missile>() = &missile;
  sim.stage_<stage_2>().obj_<Radar>() = &radar;
  
  cmdg::de::run(sim);
  
  return 0;
}
