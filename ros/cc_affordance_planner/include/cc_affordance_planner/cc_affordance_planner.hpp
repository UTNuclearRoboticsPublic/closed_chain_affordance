/*
   Author: Crasun Jans
*/
#ifndef CC_AFFORDANCE_PLANNER
#define CC_AFFORDANCE_PLANNER

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <affordance_util/affordance_util.hpp>
#include <vector>

class CcAffordancePlanner {
public:
  // Variables
  // Algorithm control parameters
  double affStep;
  double accuracy; // accuracy for error threshold
  double taskErrThreshold;
  int maxItr;        // for IK solver
  int stepperMaxItr; // for total steps
  double dt;         // time step to compute joint velocities
  int taskOffset;
  Eigen::VectorXd qp_guess;
  Eigen::VectorXd qsb_guess;
  CcAffordancePlanner(Eigen::MatrixXd screwAxes, Eigen::Matrix4d mErr,
                      Eigen::VectorXd thetalist0)
      : screwAxes_(screwAxes), mErr_(mErr), thetalist0_(thetalist0) {
    affStep = 0.1;
    accuracy = 1.0 * (1.0 / 100.0);
    taskErrThreshold = accuracy * affStep;
    maxItr = 50;
    stepperMaxItr = 75;
    dt = 1e-2;
    taskOffset = 1;

    // Guesses
    qsb_guess = thetalist0.tail(taskOffset);
    qp_guess = thetalist0.head(thetalist0.size() - taskOffset);
  };

  Eigen::VectorXd affordance_stepper() {

    int stepperItr = 0;
    while (stepperItr < stepperMaxItr) {
      // Define Network Matrices as relevant Jacobian columns

      // Set desired secondary task (affordance and maybe gripper orientation)
      // as just a few radians away from the current position

      // Set starting guess for the primary joint angles

      // Set closure error to zero and pass it along with joint angles to the
      // ik_solver
    }
  }

  Eigen::VectorXd cc_ik_solver() {

    int ikIter = 0;

    // Check Newton-Raphson error

    while (err && ikIter < maxIter) {

      // Update Jacobians

      // Compute primary joint angles

      // Optimize closure error right here

      // Check Newton-Raphson error
      ikIter = ikIter + 1;
    }
  }

private:
  const Eigen::MatrixXd screwAxes_;
  const Eigen::Matrix4d mErr_;
  const Eigen::VectorXd thetalist0_;
  const Eigen::VectorXd thetalist_;
};
#endif // CC_AFFORDANCE_PLANNER
