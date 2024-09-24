#include "trajectory_generation.h"

Polynomial::Polynomial(){};

Polynomial::Polynomial(const double &piIn, const double &pfIn, const double & DtIn){
  //TODO initialize the object polynomial coefficients
  double pi = piIn; //Position initialle
  double pf = pfIn; //Position final
  double vi = 0;    //Vitesse de début = 0
  double vf = 0;    //Vitesse final = 0
  double ai = 0;    //L'accélération de début = 0
  double af = 0;    //L'accélération finale = 0
  double Dt = DtIn;  //La durée de movement

  a[0] = pi;
  a[1] = 0;
  a[2] = 0,
  a[3] = 10 * (pf - pi) / (Dt* Dt * Dt);
  a[4] = -15 * (pf - pi) / (Dt * Dt * Dt * Dt);
  a[5] = 6 * (pf - pi) / (Dt * Dt * Dt * Dt * Dt);

};

void Polynomial::update(const double &piIn, const double &pfIn, const double & DtIn){
  //TODO update polynomial coefficients
  double pi = piIn; //Position initialle
  double pf = pfIn; //Position final
  double vi = 0;    //Vitesse de début = 0
  double vf = 0;    //Vitesse final = 0
  double ai = 0;    //L'accélération de début = 0
  double af = 0;    //L'accélération finale = 0
  double Dt = DtIn;  //La durée de movement

  a[0] = pi;
  a[1] = 0;
  a[2] = 0,
  a[3] = 10 * (pf - pi) / (Dt* Dt * Dt);
  a[4] = -15 * (pf - pi) / (Dt * Dt * Dt * Dt);
  a[5] = 6 * (pf - pi) / (Dt * Dt * Dt * Dt * Dt);
};

const double  Polynomial::p     (const double &t){
  //TODO compute position
  return a[0] + a[1]*t + a[2]*t*t + a[3]*t*t*t + a[4]*t*t*t*t + a[5]*t*t*t*t*t;;
};

const double  Polynomial::dp    (const double &t){
  //TODO compute velocity
  return a[1] + 2*a[2]*t + 3*a[3]*t*t + 4*a[4]*t*t*t + 5*a[5]*t*t*t*t;;
};

Point2Point::Point2Point(const Eigen::Vector2d & xi, const Eigen::Vector2d & xf, const double & DtIn){
  //TODO initialize object and polynomials
  polx.update(xi(0),xf(0),DtIn);
  poly.update(xi(1),xf(1),DtIn);
}

Eigen::Vector2d Point2Point::X(const double & time){
  //TODO compute cartesian position
  double x = polx.p(time);
  double y = poly.p(time);
  return Eigen::Vector2d (x,y);
}

Eigen::Vector2d Point2Point::dX(const double & time){
  //TODO compute cartesian velocity
  double dx = polx.dp(time);
  double dy = poly.dp(time);
  return Eigen::Vector2d(dx,dy);
}
