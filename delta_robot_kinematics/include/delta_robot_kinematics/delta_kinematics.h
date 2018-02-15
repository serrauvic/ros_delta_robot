#ifndef DELTA_KINEMATICS_H
#define DELTA_KINEMATICS_H

#include "ros/ros.h"

/********************Contants***********************************************/
const double pi = 3.14159265358979323846;
const double sb = 0.2;
const double sp = 0.08;
const double lsup = 0.095;
const double linf = 0.315;//0.46;
const double conversio = 180/pi;

//densitats
const double densitylsup = 0.2;// Kg/m
const double densitylinf = 0.15*2;//Kg/m
const double weighteff = 0.1;//Kg

// distancies al punt de càlcul
const double g = 9.81;
const double wb = sb*sqrt(3)/6;
const double ub = sb*sqrt(3)/3;
const double wp = sp*sqrt(3)/6;
const double up = sp*sqrt(3)/3;

const double a = wb-up;
const double b = (sp/2)-(wb*sqrt(3)/2);
const double c = wp-(0.5*wb);
// pes dels components
const double peslsup = lsup*densitylsup*g;
const double peslinf = linf*densitylinf*g;
const double peseff = weighteff*g;
const double offset = pi/2;

const double angleTrim = 0.8;

namespace delta_kinematics
{
	/*
  Inverse kinematics
  càlcul de l'angle a partir de la posició del endeffector
  posicio x i y de l'end effector seràn variables, z cte
  retorna un array que conté els angles theta1, theta2, theta3 respectivament.
  */
	void inversekinematics (double x, double y, double z, double thetas[]);

} // delta_kinematics

#endif
