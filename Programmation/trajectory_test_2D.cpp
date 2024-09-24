#include "trajectory_generation.h"
#include <iostream>
int main(){

  // Compute the trajectory for given initial and final positions. 
  // Display some of the computed points
  // Check numerically the velocities given by the library 
  // Check initial and final velocities
  Eigen::Vector2d xi (0.0,0.0);
  Eigen::Vector2d xf (10.0,10.0);
  double Dt = 10;


   // Calculer la position et la vitesse de la trajectoire à différents moments.
  Point2Point trajectory (xi,xf,Dt);

  double timestep = 0.5;
  for (double t = 0.0; t <= Dt ; t+= timestep)
    {
      Eigen::Vector2d position = trajectory.X(t);
      Eigen::Vector2d vitesse = trajectory.dX(t);

      std::cout << "Time: " << t << "s\n";
      std::cout << "Position: (" << position(0) << "," << position(1) << ")\n";
      std::cout << "Vitesse: (" << vitesse(0) << "," << vitesse(1) << ")\n";
    }

   // Vérification de la vitesse aux moments initial et final

  Eigen::Vector2d v0 = trajectory.dX(0.0);
  Eigen::Vector2d vFinal = trajectory.dX(Dt);
  std::cout << "\nInitial vitesse: (" << v0(0) << ", " << v0(1) << ")\n";
  std::cout << "Final vitesse: (" << vFinal(0) << ", " << vFinal(1) << ")\n";

  return 0;

} 
