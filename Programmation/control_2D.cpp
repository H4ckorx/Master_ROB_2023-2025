  #include "control.h"
  #include "trajectory_generation.h"
  #include <iostream>

  int main(){
    // Show using all three libraries, simulate the motion of a planar robot
    // For a initial position 
    //      q   = M_PI_2, M_PI_4
    // and  
    //      l1  = 0.4, l2 = 0.5
    // simulate a motion achieving 
    //      Xf  = 0.0, 0.6
    

      RobotModel robot(0.4, 0.5);
      Controller controller(robot);
      Eigen::Vector2d q(M_PI_2, M_PI_4);

      Eigen::Vector2d Xf(0.0, 0.6);  
      Eigen::Vector2d Dxd_ff(0.0, 0.0);  
      Eigen::Vector2d xOut;
      Eigen::Matrix2d JOut;

      double timeStep = 0.01;  
      double totalTime = 5.0;  
      int numSteps = totalTime / timeStep;

      for (int i = 0; i < numSteps; ++i) {
          // Calculer la vitesse actuelle de l'articulation Dqd
          Eigen::Vector2d Dqd = controller.Dqd(q, Xf, Dxd_ff);

          q = q + Dqd * timeStep;


          robot.FwdKin(xOut, JOut, q);

          // Sortie de l'angle actuel de l'articulation et de la position finale
          std::cout << "Time: " << i * timeStep << " s\n";
          std::cout << "Joint angles (q1, q2): (" << q(0) << ", " << q(1) << ")\n";
          std::cout << "End-effector position (x, y): (" << xOut(0) << ", " << xOut(1) << ")\n\n";
      }

      return 0;


        
  }
