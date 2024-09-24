#pragma once
#include "eigen3/Eigen/Dense"

//! \brief Fifth degree polynomial with zero velocity and acceleration at initial and final time
//!
class Polynomial{
  public:
    //! \brief Default constructor
    Polynomial      ();

    //! \brief Polynomial for given initial and final position + time interval
    //!
    //! \param pi initial position
    //! \param pf final position
    //! \param Dt time interval
    Polynomial      (const double &piIn, const double &pfIn, const double & DtIn);

    //! \brief Update polynomial parameters
    //!
    //! \param pi initial position
    //! \param pf final position
    //! \param Dt time interval
    void      update(const double &piIn, const double &pfIn, const double & Dt);  

    //! \brief Compute position at given time
    //!
    //! \param t time
    const double p     (const double &t);

    //! \brief Compute velocity at given time
    //!
    //! \param t time
    const double dp    (const double &t);
  private: 
    std::array<double,6>  a     {   0   };    // Polynomial coefficients
    double                Dt    {   0   };    // Time interval
    double                pi    {   0   };    // Initial position
    double                pf    {   0   };    // Final position
};

//! \brief Point to point trajectory generator
class Point2Point{
  public:
    //! \brief Point to point trajectory generator with given initial and final positions + time interval
    //!
    //! \param X_i initial position (x,y)
    //! \param X_f final   position (x,y)
    //! \param Dt  time interval
    Point2Point(const Eigen::Vector2d & X_i, const Eigen::Vector2d & X_f, const double & DtIn);

    //! \brief Compute position at given time
    //!
    //! \param time 
    Eigen::Vector2d X (const double & time) ;

    //! \brief Compute velocity at given time
    //!
    //! \param time
    Eigen::Vector2d dX(const double & time) ;
  private:
    Polynomial      polx                    ;   // Polynomial to x coordinates
    Polynomial      poly                    ;   // Polynomial to y coordinates    
    double          Dt          {   0   }   ;   // time interval
};