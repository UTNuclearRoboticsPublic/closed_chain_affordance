/*
   Author: Crasun Jans
*/
#ifndef CC_AFFORDANCE_PLANNER
#define CC_AFFORDANCE_PLANNER

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <affordance_util/affordance_util.hpp>
#include <vector>

class CcAffordancePlanner : public AffordanceUtil {
public:
  // Variables
  // Algorithm control parameters
  const double affStep = 0.1;
  const double accuracy = 1.0 * (1.0 / 100.0); // accuracy for error threshold
  double taskErrThreshold;
  const int maxItr = 50;        // for IK solver
  const int stepperMaxItr = 75; // for total steps
  const double dt = 1e-2;       // time step to compute joint velocities
  const int taskOffset = 1;
  const Eigen::MatrixXd slist; // Matrix containing list of screw axes as
                               // columns

  CcAffordancePlanner(const Eigen::MatrixXd &slist, const Eigen::Matrix4d &mErr,
                      const Eigen::VectorXd &thetalist0);

  std::vector<Eigen::VectorXd> affordance_stepper();

  bool cc_ik_solver();

private:
  const int twistLength_ = 6;
  const Eigen::MatrixXd screwAxes_;
  const Eigen::Matrix4d mErr_;
  const Eigen::VectorXd thetalist0_;
  const Eigen::VectorXd thetalist_;
  Eigen::VectorXd qp_guess_;
  Eigen::VectorXd qsb_guess_;
  Eigen::VectorXd qp_;
  Eigen::VectorXd oldqp_;
  Eigen::VectorXd qsb_;
  Eigen::VectorXd qsd_;
  Eigen::VectorXd errTwist_;
};
#endif // CC_AFFORDANCE_PLANNER
