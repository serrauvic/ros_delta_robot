#include "delta_robot_kinematics/delta_kinematics.h"


  /*
  Inverse kinematics
  càlcul de l'angle a partir de la posició del endeffector
  posicio x i y de l'end effector seràn variables, z cte
  retorna un array que conté els angles theta1, theta2, theta3 respectivament.
  */
  void delta_kinematics::inversekinematics (double x, double y, double z, double thetas[])
  {
    	/*float64 <-> double c++*/
    	//double thetas [3] = {0, 0, 0};

    	double e1 = 2*lsup*(y+a);
    	double f1 = 2*z*lsup;
    	double g1 = (x*x) + (y*y)+ (z*z) + (lsup*lsup) + (a*a) + 2*y*a - (linf*linf);

      double e2 = -lsup*(sqrt(3)*(x+b)+y+c);
      double f2 = 2*z*lsup;
      double g2 = (x*x) + (y*y) + (z*z) + (b*b) + (c*c) + (lsup*lsup) +2*(x*b+y*c)-(linf*linf);

      double e3 = lsup*(sqrt(3)*(x-b)-y-c);
      double f3 = 2*z*lsup;
      double g3 = (x*x) + (y*y) + (z*z) + (b*b) + (c*c) + (lsup*lsup) +2*(-x*b+y*c)-(linf*linf);

      double sol1plus = 2 * atan((-f1+sqrt((e1*e1) + (f1*f1)-(g1*g1)))/(g1-e1));
      double sol1min  = 2 * atan((-f1-sqrt((e1*e1) + (f1*f1)-(g1*g1)))/(g1-e1));
      double sol2plus = 2 * atan((-f2+sqrt((e2*e2) + (f2*f2)-(g2*g2)))/(g2-e2));
      double sol2min  = 2 * atan((-f2-sqrt((e2*e2) + (f2*f2)-(g2*g2)))/(g2-e2));
      double sol3plus = 2 * atan((-f3+sqrt((e3*e3) + (f3*f3)-(g3*g3)))/(g3-e3));
      double sol3min  = 2 * atan((-f3-sqrt((e3*e3) + (f3*f3)-(g3*g3)))/(g3-e3));

    	//isreal?
    	// comprobació de si els valors són reals
    	if ((sol1min < pi/2) && (sol1min > -pi/2))
    	{
    		thetas[0] = sol1min + offset;
    	}
    	else if ((sol1plus < pi/2) && (sol1plus > -pi/2))
    	{
    		thetas[0] = sol1plus + offset;
    	}
    	else
    	{
    		thetas[0] = 2*pi; //plus offset 360º error sing
    	}
    	if ((sol2min < pi/2) && (sol2min > -pi/2))
    	{
    		thetas[1] = sol2min + offset;
    	}
    	else if ((sol2plus < pi/2) && (sol2plus > -pi/2))
    	{
    		thetas[1] = sol2plus + offset;
    	}
    	else
    	{
    		thetas[1] = 2*pi; //plus offset 360º error sing
    	}
    	if ((sol3min < pi/2) && (sol3min > -pi/2))
    	{
    		thetas[2] = sol3min + offset;
    	}
    	else if ((sol3plus < pi/2) && (sol3plus > -pi/2))
    	{
    		thetas[2] = sol3plus + offset;
    	}
    	else
    	{
    		thetas[2] = 2*pi; //plus offset 360º error sing
    	}

  	//return thetas;
  }
